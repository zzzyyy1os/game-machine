/**
 * @file lcd.c
 * @brief ST7789/ILI9341 SPI LCD 驱动 - ESP32-S3 移植版
 *
 * 移植自 retro-go_chaeng:
 *   drivers/display/ili9341.h  — SPI 异步 DMA 队列 (事务池/缓冲池/spi_task 回收线程)
 *   targets/t320-s3/config.h   — ST7789 初始化序列 (INVON, MADCTL 横屏, gamma)
 *
 * 与 retro-go 的差异 (可移植化):
 *   - rg_alloc(MEM_DMA)  → heap_caps_malloc(MALLOC_CAP_DMA)
 *   - rg_task_create     → xTaskCreate
 *   - RG_PANIC/RG_LOGx   → ESP_LOGx / 不复位
 *   - 颜色统一为 RGB565 主机字节序, 发送前自动字节交换 (同 rg_display_clear_rect)
 *   - 绘图 API 增加互斥锁, 支持多任务调用 (retro-go 仅单任务绘图)
 */
#include "lcd.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include <string.h>

static const char *TAG = "lcd";

/* ===== SPI 异步 DMA 队列 (同 retro-go ili9341.h) ===== */
#define SPI_TRANSACTION_COUNT   10
#define SPI_BUFFER_COUNT        5
#define LCD_LINE_PIXELS         (LCD_WIDTH * 4)          /* 每次 DMA 缓冲像素数, 同 retro-go */
#define SPI_BUFFER_BYTES        (LCD_LINE_PIXELS * 2)

static spi_device_handle_t s_spi_dev;
static QueueHandle_t s_spi_transactions;   /* 空闲 spi_transaction_t* 池 */
static QueueHandle_t s_spi_buffers;        /* 空闲 DMA 缓冲池 (uint16_t*) */
static SemaphoreHandle_t s_draw_lock;      /* 绘图互斥: set_window+数据不可被拆散 */
static SemaphoreHandle_t s_done_sem;       /* 事务完成唤醒 (LCD_Sync 用) */
static portMUX_TYPE s_inflight_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile int s_inflight;            /* 在途 SPI 事务数 */
static int s_width = LCD_WIDTH;
static int s_height = LCD_HEIGHT;
static int s_rotation = LCD_ROTATION;

/* 屏幕指令: cmd + 可变长度数据 (数据 <5B 走 tx_data, >=5B 拷入 DMA 缓冲) */
#define LCD_CMD(cmd, ...)  do {                                          \
        const uint8_t c_ = (uint8_t)(cmd);                               \
        const uint8_t d_[] = {0, ##__VA_ARGS__};                         \
        lcd_queue_transaction(&c_, 1, 0);                                \
        if (sizeof(d_) > 1) lcd_queue_transaction(d_ + 1, sizeof(d_) - 1, 1); \
    } while (0)

static void lcd_queue_transaction(const void *data, size_t length, uint32_t type);

static inline uint16_t *spi_take_buffer(void)
{
    uint16_t *buffer;
    if (xQueueReceive(s_spi_buffers, &buffer, pdMS_TO_TICKS(2500)) != pdTRUE) {
        ESP_LOGE(TAG, "spi buffer timeout");
        return NULL;
    }
    return buffer;
}

static inline void spi_give_buffer(uint16_t *buffer)
{
    xQueueSend(s_spi_buffers, &buffer, portMAX_DELAY);
}

/* type: bit0 = D/C 线 (1=数据), bit1 = tx_buffer 是池缓冲, 发完由 spi_task 归还 */
static void lcd_queue_transaction(const void *data, size_t length, uint32_t type)
{
    if (!data || !length) {
        return;
    }

    spi_transaction_t *t;
    if (xQueueReceive(s_spi_transactions, &t, pdMS_TO_TICKS(2500)) != pdTRUE) {
        ESP_LOGE(TAG, "spi transaction timeout");
        return;
    }
    if (!t) {
        ESP_LOGE(TAG, "spi transaction pool empty");
        return;
    }

    *t = (spi_transaction_t){
        .tx_buffer = NULL,
        .length = length * 8,   /* 位数 */
        .user = (void *)type,
        .flags = 0,
    };

    if (type & 2) {
        /* 像素数据: data 本身就是池缓冲, 直接零拷贝发出 */
        t->tx_buffer = data;
    } else if (length < 5) {
        memcpy(t->tx_data, data, length);
        t->flags = SPI_TRANS_USE_TXDATA;
    } else {
        uint16_t *buf = spi_take_buffer();
        if (!buf) {
            xQueueSend(s_spi_transactions, &t, portMAX_DELAY);
            return;
        }
        t->tx_buffer = memcpy(buf, data, length);
        t->user = (void *)(type | 2);
    }

    portENTER_CRITICAL(&s_inflight_mux);
    s_inflight++;
    portEXIT_CRITICAL(&s_inflight_mux);

    if (spi_device_queue_trans(s_spi_dev, t, pdMS_TO_TICKS(2500)) != ESP_OK) {
        ESP_LOGE(TAG, "spi_device_queue_trans failed");
        portENTER_CRITICAL(&s_inflight_mux);
        s_inflight--;
        portEXIT_CRITICAL(&s_inflight_mux);
        if ((int)t->user & 2) {
            spi_give_buffer((uint16_t *)t->tx_buffer);
        }
        xQueueSend(s_spi_transactions, &t, portMAX_DELAY);
        return;
    }
}

IRAM_ATTR static void spi_pre_transfer_cb(spi_transaction_t *t)
{
    /* 事务发出前根据 type bit0 设置 D/C 线 (SPI ISR 上下文, 同 retro-go) */
    gpio_set_level(LCD_GPIO_DC, (int)t->user & 1);
}

/* spi_task: 事务回收线程 (同 retro-go rg_spi 线程), 归还 DMA 缓冲与事务结构 */
static void spi_task(void *arg)
{
    spi_transaction_t *t;

    while (spi_device_get_trans_result(s_spi_dev, &t, portMAX_DELAY) == ESP_OK) {
        if ((int)t->user & 2) {
            spi_give_buffer((uint16_t *)t->tx_buffer);
        }
        xQueueSend(s_spi_transactions, &t, portMAX_DELAY);
        portENTER_CRITICAL(&s_inflight_mux);
        s_inflight--;
        portEXIT_CRITICAL(&s_inflight_mux);
        xSemaphoreGive(s_done_sem);
    }
    vTaskDelete(NULL);
}

static void spi_init(void)
{
    s_spi_transactions = xQueueCreate(SPI_TRANSACTION_COUNT, sizeof(spi_transaction_t *));
    s_spi_buffers = xQueueCreate(SPI_BUFFER_COUNT, sizeof(uint16_t *));

    while (uxQueueSpacesAvailable(s_spi_transactions)) {
        void *trans = malloc(sizeof(spi_transaction_t));
        if (!trans) {
            ESP_LOGE(TAG, "transaction pool alloc failed");
            break;
        }
        xQueueSend(s_spi_transactions, &trans, portMAX_DELAY);
    }

    while (uxQueueSpacesAvailable(s_spi_buffers)) {
        void *buffer = heap_caps_malloc(SPI_BUFFER_BYTES, MALLOC_CAP_DMA);
        if (!buffer) {
            ESP_LOGE(TAG, "dma buffer alloc failed");
            break;
        }
        xQueueSend(s_spi_buffers, &buffer, portMAX_DELAY);
    }

    const spi_bus_config_t buscfg = {
        .miso_io_num = LCD_GPIO_MISO,
        .mosi_io_num = LCD_GPIO_MOSI,
        .sclk_io_num = LCD_GPIO_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = SPI_BUFFER_BYTES,
    };

    const spi_device_interface_config_t devcfg = {
        .clock_speed_hz = LCD_SPI_FREQ_HZ,
        .mode = 0,
        .spics_io_num = LCD_GPIO_CS,
        .queue_size = SPI_TRANSACTION_COUNT,
        .pre_cb = &spi_pre_transfer_cb,
        .flags = SPI_DEVICE_NO_DUMMY,
    };

    esp_err_t ret = spi_bus_initialize(LCD_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = spi_bus_add_device(LCD_SPI_HOST, &devcfg, &s_spi_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(ret));
        return;
    }

    xTaskCreate(spi_task, "lcd_spi", 2048, NULL, 6, NULL);
    ESP_LOGI(TAG, "spi ready: MOSI=%d CLK=%d CS=%d DC=%d, %dMHz",
             LCD_GPIO_MOSI, LCD_GPIO_CLK, LCD_GPIO_CS, LCD_GPIO_DC,
             LCD_SPI_FREQ_HZ / 1000000);
}

static void spi_deinit(void)
{
    /* 在途事务未完成时 spi_bus_remove_device 会失败, 仅记录不复位 */
    if (spi_bus_remove_device(s_spi_dev) == ESP_OK) {
        spi_bus_free(LCD_SPI_HOST);
    } else {
        ESP_LOGE(TAG, "failed to properly terminate SPI driver");
    }
}

/* ===== 背光 (LEDC 渐变, 同 retro-go; 极性由 LCD_BCKL_INVERT 决定) ===== */
#if LCD_GPIO_BCKL >= 0 && LCD_BCKL_MODE
static int s_ledc_fade_installed;

static void lcd_ledc_setup(void)
{
    ledc_timer_config(&(ledc_timer_config_t){
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_13_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
    });
    ledc_channel_config(&(ledc_channel_config_t){
        .gpio_num = LCD_GPIO_BCKL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
    });
    if (!s_ledc_fade_installed) {
        ledc_fade_func_install(0);
        s_ledc_fade_installed = true;
    }
}
#endif

static void lcd_set_backlight_raw(float level)
{
#if LCD_GPIO_BCKL >= 0
    if (LCD_BCKL_INVERT) {
        level = 1.0f - level;
    }
#if LCD_BCKL_MODE
    int err = ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                           (int)(0x1FFF * level), 50, 0);
    if (err) {
        ESP_LOGE(TAG, "set backlight %d%% failed: %d", (int)(100 * level), err);
    }
#else
    gpio_set_level(LCD_GPIO_BCKL, level > 0.5f ? 1 : 0);
#endif
#endif
}

void LCD_SetBacklight(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    lcd_set_backlight_raw(percent / 100.f);
    ESP_LOGI(TAG, "backlight set to %d%%", percent);
}

/* 背光诊断 (Layer 1): 不依赖 SPI/初始化, 在 LCD_Init 之前调用。
   5 档各 2s, 串口打点; 肉眼观察哪一档亮, 据此填 lcd.h 的 LCD_BCKL_INVERT/MODE。
   全不亮 → 硬件问题 (GND 共地/BLK 线/供电/FPC), 停止折腾初始化序列。 */
void LCD_BacklightDiag(void)
{
#if LCD_GPIO_BCKL >= 0
    /* 先用纯 GPIO 输出, 不占 LEDC */
    gpio_config(&(gpio_config_t){
        .pin_bit_mask = 1ULL << LCD_GPIO_BCKL,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    });

    ESP_LOGI(TAG, "bckl diag 1/5: GPIO HIGH 2s (BLK 高有效应亮)");
    gpio_set_level(LCD_GPIO_BCKL, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "bckl diag 2/5: GPIO LOW 2s (低有效应亮)");
    gpio_set_level(LCD_GPIO_BCKL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

#if LCD_BCKL_MODE
    /* PWM 档: 复用与 LCD_Init 相同的 LEDC 配置 (诊断直接写 duty, 不过 INVERT) */
    lcd_ledc_setup();

    ESP_LOGI(TAG, "bckl diag 3/5: PWM duty 80%% 2s (引脚多为高)");
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int)(0x1FFF * 0.8f), 50, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));

    ESP_LOGI(TAG, "bckl diag 4/5: PWM duty 20%% 2s (引脚多为低)");
    ledc_set_fade_time_and_start(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (int)(0x1FFF * 0.2f), 50, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
#else
    ESP_LOGI(TAG, "bckl diag 3/5: GPIO HIGH 2s (MODE=0 复测)");
    gpio_set_level(LCD_GPIO_BCKL, 1);
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "bckl diag 4/5: GPIO LOW 2s (MODE=0 复测)");
    gpio_set_level(LCD_GPIO_BCKL, 0);
    vTaskDelay(pdMS_TO_TICKS(2000));
#endif

    ESP_LOGI(TAG, "bckl diag 5/5: 交替慢闪 1Hz 2s (肉眼与日志对时)");
    for (int i = 0; i < 2; i++) {
#if LCD_BCKL_MODE
        lcd_set_backlight_raw(1.0f);   /* LEDC 已接管引脚, gpio_set_level 无效 */
#else
        gpio_set_level(LCD_GPIO_BCKL, 1);
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
#if LCD_BCKL_MODE
        lcd_set_backlight_raw(0.0f);
#else
        gpio_set_level(LCD_GPIO_BCKL, 0);
#endif
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "bckl diag done — 记录哪一档亮, 回填 LCD_BCKL_INVERT/LCD_BCKL_MODE");
#else
    ESP_LOGW(TAG, "bckl diag skipped (LCD_GPIO_BCKL < 0)");
#endif
}

/* ===== 窗口与像素发送 (调用前需持有 s_draw_lock) ===== */
static void lcd_set_window(int left, int top, int width, int height)
{
    int right = left + width - 1;
    int bottom = top + height - 1;

    if (left < 0 || top < 0 || right >= s_width || bottom >= s_height) {
        ESP_LOGW(TAG, "bad window (x0=%d, y0=%d, x1=%d, y1=%d)", left, top, right, bottom);
    }

    LCD_CMD(0x2A, left >> 8, left & 0xff, right >> 8, right & 0xff);   /* 列地址 */
    LCD_CMD(0x2B, top >> 8, top & 0xff, bottom >> 8, bottom & 0xff);   /* 行地址 */
    LCD_CMD(0x2C);                                                     /* 写显存 */
}

/* 取 DMA 缓冲并送出 length 像素 (源像素为 RGB565 主机字节序, 发送时交换) */
static void lcd_write_pixels_locked(int x, int y, int width, int height, const uint16_t *pixels)
{
    if (width <= 0 || height <= 0 || x < 0 || y < 0 || x + width > s_width || y + height > s_height) {
        return;
    }

    lcd_set_window(x, y, width, height);

    int remaining = width * height;
    while (remaining > 0) {
        uint16_t *buffer = spi_take_buffer();
        if (!buffer) {
            return;
        }
        int n = remaining > LCD_LINE_PIXELS ? LCD_LINE_PIXELS : remaining;
        for (int i = 0; i < n; i++) {
            buffer[i] = (pixels[i] << 8) | (pixels[i] >> 8);   /* 大端发往屏幕 */
        }
        lcd_queue_transaction(buffer, n * 2, 3);   /* type 3 = 数据 + 池缓冲所有权转移 */
        pixels += n;
        remaining -= n;
    }
}

static void lcd_fill_locked(int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0) {
        return;
    }
    /* 裁剪到屏幕范围 */
    if (x < 0) { width += x; x = 0; }
    if (y < 0) { height += y; y = 0; }
    if (x + width > s_width)  width  = s_width  - x;
    if (y + height > s_height) height = s_height - y;
    if (width <= 0 || height <= 0) {
        return;
    }

    lcd_set_window(x, y, width, height);

    const uint16_t color_be = (uint16_t)((color << 8) | (color >> 8));
    int remaining = width * height;
    while (remaining > 0) {
        uint16_t *buffer = spi_take_buffer();
        if (!buffer) {
            return;
        }
        int n = remaining > LCD_LINE_PIXELS ? LCD_LINE_PIXELS : remaining;
        for (int i = 0; i < n; i++) {
            buffer[i] = color_be;
        }
        lcd_queue_transaction(buffer, n * 2, 3);
        remaining -= n;
    }
}

/* ===== 屏幕初始化序列 =====
   LCD_CMD_T: 逐条日志 + LCD_Sync (日志=真实发完, 由 LCD_INIT_TRACE 控制);
   高频绘图路径仍用 LCD_CMD 异步裸发。 */
#define LCD_CMD_T(cmd, ...) do {                                          \
        if (LCD_INIT_TRACE) {                                             \
            ESP_LOGI(TAG, "init cmd 0x%02X", (uint8_t)(cmd));             \
            LCD_CMD(cmd, ##__VA_ARGS__);                                  \
            LCD_Sync();                                                   \
        } else {                                                          \
            LCD_CMD(cmd, ##__VA_ARGS__);                                  \
        }                                                                 \
    } while (0)

/* 0=retro-go chaeng (t320-s3/config.h RG_SCREEN_INIT), INVON 由 LCD_INVON 控制 */
static void lcd_panel_init_retrogo(void)
{
    LCD_CMD_T(0x01);          /* Software Reset */
    vTaskDelay(pdMS_TO_TICKS(120));   /* ST7789 手册: SWRESET 后 ≥120ms */
    LCD_CMD_T(0x3A, 0x55);    /* COLMOD: RGB565 65k */
    LCD_CMD_T(0x36, (uint8_t)((LCD_RGB_BGR ? 0x08 : 0x00) | (LCD_ROTATION << 5))); /* MADCTL */

    LCD_CMD_T(0xCF, 0x00, 0xc3, 0x30);
    LCD_CMD_T(0xED, 0x64, 0x03, 0x12, 0x81);
    LCD_CMD_T(0xE8, 0x85, 0x00, 0x78);
    LCD_CMD_T(0xCB, 0x39, 0x2c, 0x00, 0x34, 0x02);
    LCD_CMD_T(0xF7, 0x20);
    LCD_CMD_T(0xEA, 0x00, 0x00);
    LCD_CMD_T(0xC0, 0x1B);                /* Power control */
    LCD_CMD_T(0xC1, 0x12);                /* Power control */
    LCD_CMD_T(0xC5, 0x32, 0x3C);          /* VCOM */
    LCD_CMD_T(0xC7, 0x91);                /* VCOM offset */
    LCD_CMD_T(0xB1, 0x00, 0x10);          /* Frame rate */
    LCD_CMD_T(0xB6, 0x0A, 0xA2);          /* Display function control */
    LCD_CMD_T(0xF6, 0x01, 0x30);
    LCD_CMD_T(0xF2, 0x00);
    LCD_CMD_T(0x26, 0x01);                /* Gamma */
    LCD_CMD_T(0xE0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00); /* Gamma+ */
    LCD_CMD_T(0xE1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F); /* Gamma- */

    if (LCD_INVON) {
        LCD_CMD_T(0x21);   /* INVON: IPS 常黑屏反显, 颜色全反时关掉 */
    } else {
        LCD_CMD_T(0x13);   /* INVOFF */
    }

    LCD_CMD_T(0x11);      /* Exit Sleep */
    vTaskDelay(pdMS_TO_TICKS(120));   /* 手册要求 ≥120ms (原移植 10ms 不足) */
    LCD_CMD_T(0x29);      /* Display ON */
}

/* 1=BOE3.2 金逸晨 GM demo (TFT_Init) 照搬 — 本项目 BOE3.2 裸屏的默认序列 */
static void lcd_panel_init_boe32(void)
{
    /* GM 顺序: 0x11 → 0x21 → 电源/porch/gamma → 0x29 → 0x2C → 0x3A → 0x36 */
    LCD_CMD_T(0x11);      /* SLPOUT */
    vTaskDelay(pdMS_TO_TICKS(120));   /* GM: 120ms */

    if (LCD_INVON) {
        LCD_CMD_T(0x21);  /* INVON (GM 硬编码 0x21) */
    } else {
        LCD_CMD_T(0x13);  /* INVOFF */
    }

    LCD_CMD_T(0xB2, 0x05, 0x05, 0x00, 0x33, 0x33);  /* porch */
    LCD_CMD_T(0xB7, 0x23);                          /* gate */
    LCD_CMD_T(0xBB, 0x22);                          /* VCOM */
    LCD_CMD_T(0xC0, 0x2C);                          /* LCM */
    LCD_CMD_T(0xC2, 0x01);
    LCD_CMD_T(0xC3, 0x13);                          /* VRH */
    LCD_CMD_T(0xC4, 0x20);                          /* VDV */
    LCD_CMD_T(0xC6, 0x0F);                          /* frame rate */
    LCD_CMD_T(0xD0, 0xA4, 0xA1);                    /* power */
    LCD_CMD_T(0xD6, 0xA1);
    LCD_CMD_T(0xE0, 0x70, 0x06, 0x0C, 0x08, 0x09, 0x27, 0x2E, 0x34, 0x46, 0x37, 0x13, 0x13, 0x25, 0x2A); /* Gamma+ */
    LCD_CMD_T(0xE1, 0x70, 0x04, 0x08, 0x09, 0x07, 0x03, 0x2C, 0x42, 0x42, 0x38, 0x14, 0x14, 0x27, 0x2C); /* Gamma- */
    LCD_CMD_T(0x29);        /* Display ON */
    LCD_CMD_T(0x2C);        /* GM 紧跟写 GRAM (无窗口, 无害, 照搬消变量) */
    LCD_CMD_T(0x3A, 0x55);  /* COLMOD: RGB565 (GM 为 0x66 RGB666; 565 走本驱动 2B/px 路径) */
    LCD_CMD_T(0x36, (uint8_t)((LCD_RGB_BGR ? 0x08 : 0x00) | (LCD_ROTATION << 5))); /* MADCTL */
}

static void lcd_panel_init(void)
{
    ESP_LOGI(TAG, "init[%s] start", LCD_PANEL_SEQ ? "BOE32" : "retrogo");
    if (LCD_PANEL_SEQ) {
        lcd_panel_init_boe32();
    } else {
        lcd_panel_init_retrogo();
    }
}

void LCD_Init(void)
{
    s_draw_lock = xSemaphoreCreateMutex();
    s_done_sem = xSemaphoreCreateBinary();
    s_width = LCD_WIDTH;
    s_height = LCD_HEIGHT;
    s_rotation = LCD_ROTATION;

    /* 背光先保持 0%, 避免初始化花屏 */
#if LCD_GPIO_BCKL >= 0 && LCD_BCKL_MODE
    lcd_ledc_setup();
#elif LCD_GPIO_BCKL >= 0
    gpio_set_direction(LCD_GPIO_BCKL, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_GPIO_BCKL, 0);
#endif

    spi_init();

    gpio_set_direction(LCD_GPIO_DC, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_GPIO_DC, 1);

#if LCD_GPIO_RST >= 0
    gpio_set_direction(LCD_GPIO_RST, GPIO_MODE_OUTPUT);
    gpio_set_level(LCD_GPIO_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(LCD_GPIO_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(100));   /* GM: RST 释放后 100ms */
#endif

    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    lcd_panel_init();
    xSemaphoreGive(s_draw_lock);

    ESP_LOGI(TAG, "ST7789 initialized: %dx%d rotation=%d", s_width, s_height, s_rotation);
}

void LCD_Deinit(void)
{
    LCD_SetBacklight(0);
    spi_deinit();
}

void LCD_Clear(uint16_t color)
{
    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    lcd_fill_locked(0, 0, s_width, s_height, color);
    xSemaphoreGive(s_draw_lock);
}

void LCD_FillRect(int x, int y, int width, int height, uint16_t color)
{
    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    lcd_fill_locked(x, y, width, height, color);
    xSemaphoreGive(s_draw_lock);
}

void LCD_DrawPixel(int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= s_width || y >= s_height) {
        return;
    }
    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    lcd_set_window(x, y, 1, 1);
    uint16_t be = (uint16_t)((color << 8) | (color >> 8));
    lcd_queue_transaction(&be, 2, 1);
    xSemaphoreGive(s_draw_lock);
}

void LCD_WritePixels(int x, int y, int width, int height, const uint16_t *pixels)
{
    if (!pixels) {
        return;
    }
    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    lcd_write_pixels_locked(x, y, width, height, pixels);
    xSemaphoreGive(s_draw_lock);
}

void LCD_SetRotation(int rotation)
{
    rotation &= 7;
    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    uint8_t madctl = (uint8_t)((LCD_RGB_BGR ? 0x08 : 0x00) | (rotation << 5));
    LCD_CMD(0x36, madctl);
    if ((rotation ^ s_rotation) & 0x20) {   /* MV 位翻转 → 宽高互换 */
        int t = s_width;
        s_width = s_height;
        s_height = t;
    }
    s_rotation = rotation;
    xSemaphoreGive(s_draw_lock);
    ESP_LOGI(TAG, "rotation=%d size=%dx%d", rotation, s_width, s_height);
}

void LCD_Sync(void)
{
    while (1) {
        portENTER_CRITICAL(&s_inflight_mux);
        int n = s_inflight;
        portEXIT_CRITICAL(&s_inflight_mux);
        if (n <= 0) {
            break;
        }
        xSemaphoreTake(s_done_sem, pdMS_TO_TICKS(100));
    }
}

/* ===== 内置 6x8 ASCII 字体 (同 oled.c) ===== */
static const uint8_t lcd_font_6x8[][6] = {
    {0x00,0x00,0x00,0x00,0x00,0x00}, /* sp */
    {0x00,0x00,0x5F,0x00,0x00,0x00}, /* ! */
    {0x00,0x07,0x00,0x07,0x00,0x00}, /* " */
    {0x14,0x7F,0x14,0x7F,0x14,0x00}, /* # */
    {0x24,0x2A,0x7F,0x2A,0x12,0x00}, /* $ */
    {0x23,0x13,0x08,0x64,0x62,0x00}, /* % */
    {0x36,0x49,0x55,0x22,0x50,0x00}, /* & */
    {0x00,0x05,0x03,0x00,0x00,0x00}, /* ' */
    {0x00,0x1C,0x22,0x41,0x00,0x00}, /* ( */
    {0x00,0x41,0x22,0x1C,0x00,0x00}, /* ) */
    {0x14,0x08,0x3E,0x08,0x14,0x00}, /* * */
    {0x08,0x08,0x3E,0x08,0x08,0x00}, /* + */
    {0x00,0x50,0x30,0x00,0x00,0x00}, /* , */
    {0x08,0x08,0x08,0x08,0x08,0x00}, /* - */
    {0x00,0x60,0x60,0x00,0x00,0x00}, /* . */
    {0x20,0x10,0x08,0x04,0x02,0x00}, /* / */
    {0x3E,0x51,0x49,0x45,0x3E,0x00}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46,0x00}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31,0x00}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10,0x00}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39,0x00}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30,0x00}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03,0x00}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36,0x00}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E,0x00}, /* 9 */
    {0x00,0x36,0x36,0x00,0x00,0x00}, /* : */
    {0x00,0x56,0x36,0x00,0x00,0x00}, /* ; */
    {0x08,0x14,0x22,0x41,0x00,0x00}, /* < */
    {0x14,0x14,0x14,0x14,0x14,0x00}, /* = */
    {0x00,0x41,0x22,0x14,0x08,0x00}, /* > */
    {0x02,0x01,0x51,0x09,0x06,0x00}, /* ? */
    {0x32,0x49,0x79,0x41,0x3E,0x00}, /* @ */
    {0x7E,0x11,0x11,0x11,0x7E,0x00}, /* A */
    {0x7F,0x49,0x49,0x49,0x36,0x00}, /* B */
    {0x3E,0x41,0x41,0x41,0x22,0x00}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C,0x00}, /* D */
    {0x7F,0x49,0x49,0x49,0x41,0x00}, /* E */
    {0x7F,0x09,0x09,0x09,0x01,0x00}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A,0x00}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F,0x00}, /* H */
    {0x00,0x41,0x7F,0x41,0x00,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01,0x00}, /* J */
    {0x7F,0x08,0x14,0x22,0x41,0x00}, /* K */
    {0x7F,0x40,0x40,0x40,0x40,0x00}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F,0x00}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F,0x00}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E,0x00}, /* O */
    {0x7F,0x09,0x09,0x09,0x06,0x00}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E,0x00}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46,0x00}, /* R */
    {0x46,0x49,0x49,0x49,0x31,0x00}, /* S */
    {0x01,0x01,0x7F,0x01,0x01,0x00}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F,0x00}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F,0x00}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F,0x00}, /* W */
    {0x63,0x14,0x08,0x14,0x63,0x00}, /* X */
    {0x07,0x08,0x70,0x08,0x07,0x00}, /* Y */
    {0x61,0x51,0x49,0x45,0x43,0x00}, /* Z */
    {0x00,0x7F,0x41,0x41,0x00,0x00}, /* [ */
    {0x02,0x04,0x08,0x10,0x20,0x00}, /* \ */
    {0x00,0x41,0x41,0x7F,0x00,0x00}, /* ] */
    {0x04,0x02,0x01,0x02,0x04,0x00}, /* ^ */
    {0x40,0x40,0x40,0x40,0x40,0x00}, /* _ */
    {0x00,0x01,0x02,0x04,0x00,0x00}, /* ` */
    {0x20,0x54,0x54,0x54,0x78,0x00}, /* a */
    {0x7F,0x48,0x44,0x44,0x38,0x00}, /* b */
    {0x38,0x44,0x44,0x44,0x20,0x00}, /* c */
    {0x38,0x44,0x44,0x48,0x7F,0x00}, /* d */
    {0x38,0x54,0x54,0x54,0x18,0x00}, /* e */
    {0x08,0x7E,0x09,0x01,0x02,0x00}, /* f */
    {0x0C,0x52,0x52,0x52,0x3E,0x00}, /* g */
    {0x7F,0x08,0x04,0x04,0x78,0x00}, /* h */
    {0x00,0x44,0x7D,0x40,0x00,0x00}, /* i */
    {0x20,0x40,0x44,0x3D,0x00,0x00}, /* j */
    {0x7F,0x10,0x28,0x44,0x00,0x00}, /* k */
    {0x00,0x41,0x7F,0x40,0x00,0x00}, /* l */
    {0x7C,0x04,0x18,0x04,0x78,0x00}, /* m */
    {0x7C,0x08,0x04,0x04,0x78,0x00}, /* n */
    {0x38,0x44,0x44,0x44,0x38,0x00}, /* o */
    {0x7C,0x14,0x14,0x14,0x08,0x00}, /* p */
    {0x08,0x14,0x14,0x18,0x7C,0x00}, /* q */
    {0x7C,0x08,0x04,0x04,0x08,0x00}, /* r */
    {0x48,0x54,0x54,0x54,0x20,0x00}, /* s */
    {0x04,0x3F,0x44,0x40,0x20,0x00}, /* t */
    {0x3C,0x40,0x40,0x20,0x7C,0x00}, /* u */
    {0x1C,0x20,0x40,0x20,0x1C,0x00}, /* v */
    {0x3C,0x40,0x30,0x40,0x3C,0x00}, /* w */
    {0x44,0x28,0x10,0x28,0x44,0x00}, /* x */
    {0x0C,0x50,0x50,0x50,0x3C,0x00}, /* y */
    {0x44,0x64,0x54,0x4C,0x44,0x00}, /* z */
    {0x00,0x08,0x36,0x41,0x00,0x00}, /* { */
    {0x00,0x00,0x7F,0x00,0x00,0x00}, /* | */
    {0x00,0x41,0x36,0x08,0x00,0x00}, /* } */
    {0x10,0x08,0x08,0x10,0x08,0x00}, /* ~ */
};

void LCD_ShowChar(int x, int y, char ch, uint16_t fg, uint16_t bg, int scale)
{
    if (ch < 32 || ch > 126) {
        ch = ' ';
    }
    if (scale < 1) scale = 1;
    if (scale > 3) scale = 3;

    /* 整格渲染到缓冲再一次性发出 (减少 SPI 事务数); cell 为共享缓冲, 全程持锁 */
    static uint16_t cell[LCD_FONT_WIDTH * 3 * LCD_FONT_HEIGHT * 3];
    const uint8_t *glyph = lcd_font_6x8[ch - 32];
    int cw = LCD_FONT_WIDTH * scale;
    int chh = LCD_FONT_HEIGHT * scale;

    xSemaphoreTake(s_draw_lock, portMAX_DELAY);
    for (int col = 0; col < LCD_FONT_WIDTH; col++) {
        uint8_t bits = glyph[col];
        for (int row = 0; row < LCD_FONT_HEIGHT; row++) {
            uint16_t color = (bits & (1 << row)) ? fg : bg;
            for (int sy = 0; sy < scale; sy++) {
                uint16_t *line = &cell[(row * scale + sy) * cw + col * scale];
                for (int sx = 0; sx < scale; sx++) {
                    line[sx] = color;
                }
            }
        }
    }
    lcd_write_pixels_locked(x, y, cw, chh, cell);
    xSemaphoreGive(s_draw_lock);
}

void LCD_ShowString(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale)
{
    if (!str) {
        return;
    }
    if (scale < 1) scale = 1;
    if (scale > 3) scale = 3;

    int start_x = x;
    while (*str) {
        if (x + LCD_FONT_WIDTH * scale > s_width) {
            x = start_x;
            y += LCD_FONT_HEIGHT * scale;
        }
        if (y + LCD_FONT_HEIGHT * scale > s_height) {
            break;
        }
        LCD_ShowChar(x, y, *str, fg, bg, scale);
        x += LCD_FONT_WIDTH * scale;
        str++;
    }
}
