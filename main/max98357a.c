/**
 * @file max98357a.c
 * @brief MAX98357A I2S 数字功放驱动 (可移植模块, 功放专用线程)
 * @note  GPIO 引脚配置见 max98357a.h
 *
 * 移植自 retro-go drivers/audio/i2s.c (Ext DAC 分支), 改用新版 i2s_std API:
 * 16bit 有符号立体声 / Philips 标准 I2S / 主机发送 / DMA 4x180 帧 / 软件音量缩放
 *
 * 线程模型 (无轮询):
 *   MAX98357A_Write() 只把 PCM 拷入缓冲池并入队 (队满阻塞背压);
 *   功放线程阻塞等队列, 出队后做音量缩放并写 I2S, 写完回收缓冲。
 *   采样率热切换以消息形式走同一队列, 与 PCM 严格保序。
 */
#include "max98357a.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/i2s_std.h"
#include "driver/i2s_common.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "max98357a";

#define AMP_TASK_STACK     3072
#define AMP_TASK_PRIO      5

/* 队列消息: PCM 块 / 采样率切换 */
typedef enum {
    AMP_MSG_PCM = 0,
    AMP_MSG_RATE,
} amp_msg_type_t;

typedef struct {
    amp_msg_type_t type;
    int      buf_idx;       /* AMP_MSG_PCM: 缓冲池下标 */
    size_t   frames;        /* AMP_MSG_PCM: 帧数 */
    int      rate;          /* AMP_MSG_RATE: 目标采样率 */
} amp_msg_t;

static i2s_chan_handle_t s_tx_chan = NULL;
static int s_volume = MAX98357A_DEFAULT_VOLUME;
static bool s_muted = false;
static volatile int s_sample_rate = 0;

/* PCM 缓冲池 + 双队列 (自由缓冲 / 待播消息) */
static int16_t   *s_pcm_bufs[MAX98357A_PCM_BUF_NUM];
static QueueHandle_t s_free_q = NULL;    /* 空闲缓冲下标 */
static QueueHandle_t s_work_q = NULL;    /* 待播消息 (PCM/RATE 保序) */
static SemaphoreHandle_t s_rate_sem = NULL;   /* SetSampleRate 同步等待 */
static volatile int s_rate_result = 0;        /* 功放线程回填切换结果 (信号量保序) */
static volatile int s_pending = 0;       /* 在途消息数 (队列中+正在写) */
static TaskHandle_t s_amp_task = NULL;
static volatile bool s_amp_run = false;

/* 可选功放使能脚 (SD_MODE) 控制 */
#if MAX98357A_AMP_ENABLE_IO >= 0
static inline void amp_enable_set(bool on)
{
#if MAX98357A_AMP_ENABLE_INVERT
    gpio_set_level(MAX98357A_AMP_ENABLE_IO, on ? 0 : 1);
#else
    gpio_set_level(MAX98357A_AMP_ENABLE_IO, on ? 1 : 0);
#endif
}
#endif

/* ===== I2S 底层写 (仅功放线程调用) ===== */
static size_t i2s_write_frames(const int16_t *samples, size_t frames)
{
    const int vol = s_muted ? 0 : s_volume;
    int16_t chunk[MAX98357A_DMA_FRAME_NUM * 2];     /* L/R 交错栈缓冲 */
    size_t done = 0;

    while (done < frames) {
        size_t n = frames - done;
        if (n > MAX98357A_DMA_FRAME_NUM) {
            n = MAX98357A_DMA_FRAME_NUM;
        }

        if (vol == 100) {
            memcpy(chunk, samples + done * 2, n * 2 * sizeof(int16_t));
        } else {
            const int16_t *src = samples + done * 2;
            for (size_t i = 0; i < n * 2; ++i) {
                chunk[i] = (int16_t)(((int32_t)src[i] * vol) / 100);
            }
        }

        size_t written_bytes = 0;
        esp_err_t err = i2s_channel_write(s_tx_chan, chunk,
                                          n * 2 * sizeof(int16_t),
                                          &written_bytes,
                                          MAX98357A_WRITE_TIMEOUT_MS);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "write err=%s, %u/%u bytes", esp_err_to_name(err),
                     (unsigned)written_bytes, (unsigned)(n * 2 * sizeof(int16_t)));
            break;
        }
        done += n;
    }
    return done;
}

/* 采样率切换 (仅功放线程调用, 此时队列中已无旧数据在写) */
static int do_set_sample_rate(int sample_rate)
{
    if (!s_tx_chan || sample_rate <= 0) {
        return 0;
    }
    if (sample_rate == s_sample_rate) {
        return s_sample_rate;
    }

    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
    ESP_ERROR_CHECK(i2s_channel_disable(s_tx_chan));
    esp_err_t err = i2s_channel_reconfig_std_clock(s_tx_chan, &clk_cfg);
    if (err == ESP_OK) {
        err = i2s_channel_enable(s_tx_chan);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set sample rate %d failed: %s", sample_rate, esp_err_to_name(err));
        i2s_channel_enable(s_tx_chan);              /* 恢复运行 */
        return 0;
    }
    s_sample_rate = sample_rate;
    ESP_LOGI(TAG, "sample rate -> %d Hz", sample_rate);
    return s_sample_rate;
}

/* ===== 功放线程: 阻塞等消息队列, 不轮询 ===== */
static void amp_task(void *arg)
{
    amp_msg_t msg;

    while (s_amp_run) {
        if (xQueueReceive(s_work_q, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;                               /* 超时仅用于响应退出标志 */
        }

        if (msg.type == AMP_MSG_PCM) {
            i2s_write_frames(s_pcm_bufs[msg.buf_idx], msg.frames);
            int idx = msg.buf_idx;
            xQueueSend(s_free_q, &idx, portMAX_DELAY);
        } else {                                    /* AMP_MSG_RATE */
            s_rate_result = do_set_sample_rate(msg.rate);
            xSemaphoreGive(s_rate_sem);
        }
        s_pending--;
    }

    s_amp_task = NULL;
    vTaskDelete(NULL);
}

void MAX98357A_Init(int sample_rate)
{
    if (s_tx_chan) {
        return;                                     /* 已初始化 */
    }
    if (sample_rate <= 0) {
        sample_rate = MAX98357A_DEFAULT_SAMPLE_RATE;
    }
    s_volume = MAX98357A_DEFAULT_VOLUME;
    s_muted = false;

    /* 1) 分配 TX 通道 (主机, 仅发送) */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = MAX98357A_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = MAX98357A_DMA_FRAME_NUM;
    chan_cfg.auto_clear = true;                     /* DMA 欠载自动补零, 避免残留杂音 (略增中断开销) */
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_tx_chan, NULL));

    /* 2) 标准模式 (Philips / 16bit / 立体声) */
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MAX98357A_GPIO_BCLK,
            .ws = MAX98357A_GPIO_WS,
            .dout = MAX98357A_GPIO_DOUT,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_tx_chan, &std_cfg));

    /* 3) 启动通道 */
    ESP_ERROR_CHECK(i2s_channel_enable(s_tx_chan));

#if MAX98357A_AMP_ENABLE_IO >= 0
    gpio_reset_pin(MAX98357A_AMP_ENABLE_IO);
    gpio_set_direction(MAX98357A_AMP_ENABLE_IO, GPIO_MODE_OUTPUT);
    amp_enable_set(true);
#endif

    s_sample_rate = sample_rate;

    /* 4) PCM 缓冲池 + 双队列 + 功放线程 */
    s_free_q = xQueueCreate(MAX98357A_PCM_BUF_NUM, sizeof(int));
    s_work_q = xQueueCreate(MAX98357A_PCM_BUF_NUM, sizeof(amp_msg_t));
    s_rate_sem = xSemaphoreCreateBinary();
    for (int i = 0; i < MAX98357A_PCM_BUF_NUM; i++) {
        s_pcm_bufs[i] = malloc((size_t)MAX98357A_PCM_BUF_FRAMES * 2 * sizeof(int16_t));
        if (s_pcm_bufs[i] == NULL) {
            ESP_LOGE(TAG, "pcm buf %d alloc failed", i);
            abort();
        }
        xQueueSend(s_free_q, &i, 0);
    }
    s_pending = 0;
    s_amp_run = true;
    xTaskCreate(amp_task, "amp_task", AMP_TASK_STACK, NULL, AMP_TASK_PRIO, &s_amp_task);

    ESP_LOGI(TAG, "MAX98357A initialized: %d Hz, BCLK=%d WS=%d DOUT=%d, vol=%d, %d bufs x %d frames",
             sample_rate, MAX98357A_GPIO_BCLK, MAX98357A_GPIO_WS,
             MAX98357A_GPIO_DOUT, s_volume,
             MAX98357A_PCM_BUF_NUM, MAX98357A_PCM_BUF_FRAMES);
}

void MAX98357A_Deinit(void)
{
    if (!s_tx_chan) {
        return;
    }
    s_amp_run = false;
    while (s_amp_task) {
        vTaskDelay(pdMS_TO_TICKS(10));              /* 等功放线程自删 */
    }
    i2s_channel_disable(s_tx_chan);
    i2s_del_channel(s_tx_chan);
    s_tx_chan = NULL;
    s_sample_rate = 0;
    for (int i = 0; i < MAX98357A_PCM_BUF_NUM; i++) {
        free(s_pcm_bufs[i]);
        s_pcm_bufs[i] = NULL;
    }
    vQueueDelete(s_free_q);
    vQueueDelete(s_work_q);
    vSemaphoreDelete(s_rate_sem);
    s_free_q = s_work_q = NULL;
    s_rate_sem = NULL;
    gpio_reset_pin(MAX98357A_GPIO_BCLK);
    gpio_reset_pin(MAX98357A_GPIO_WS);
    gpio_reset_pin(MAX98357A_GPIO_DOUT);
#if MAX98357A_AMP_ENABLE_IO >= 0
    gpio_reset_pin(MAX98357A_AMP_ENABLE_IO);
#endif
    ESP_LOGI(TAG, "deinitialized");
}

size_t MAX98357A_Write(const int16_t *samples, size_t frames)
{
    if (!s_tx_chan || !samples || frames == 0) {
        return 0;
    }

    size_t done = 0;
    while (done < frames) {
        size_t n = frames - done;
        if (n > MAX98357A_PCM_BUF_FRAMES) {
            n = MAX98357A_PCM_BUF_FRAMES;
        }

        /* 取空闲缓冲 (队满即背压阻塞) */
        int idx;
        if (xQueueReceive(s_free_q, &idx, pdMS_TO_TICKS(MAX98357A_WRITE_TIMEOUT_MS)) != pdTRUE) {
            ESP_LOGW(TAG, "pcm pool exhausted, short write %u/%u", (unsigned)done, (unsigned)frames);
            break;
        }
        memcpy(s_pcm_bufs[idx], samples + done * 2, n * 2 * sizeof(int16_t));

        amp_msg_t msg = {
            .type = AMP_MSG_PCM,
            .buf_idx = idx,
            .frames = n,
        };
        s_pending++;
        if (xQueueSend(s_work_q, &msg, pdMS_TO_TICKS(MAX98357A_WRITE_TIMEOUT_MS)) != pdTRUE) {
            s_pending--;
            xQueueSend(s_free_q, &idx, 0);
            ESP_LOGW(TAG, "work queue full, short write %u/%u", (unsigned)done, (unsigned)frames);
            break;
        }
        done += n;
    }
    return done;
}

void MAX98357A_WaitDrained(uint32_t timeout_ms)
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);
    while (s_pending > 0) {
        if ((int32_t)(xTaskGetTickCount() - deadline) >= 0) {
            ESP_LOGW(TAG, "WaitDrained timeout, pending=%d", s_pending);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

void MAX98357A_SetVolume(int percent)
{
    if (percent < 0) {
        percent = 0;
    }
    if (percent > 100) {
        percent = 100;
    }
    s_volume = percent;
}

void MAX98357A_SetMute(bool mute)
{
    s_muted = mute;
    /* 冲刷 DMA: 静音时输出 0, 避免残留旧样本 */
    if (s_tx_chan) {
        int16_t silence[MAX98357A_DMA_FRAME_NUM * 2] = {0};
        MAX98357A_Write(silence, MAX98357A_DMA_FRAME_NUM);
    }
#if MAX98357A_AMP_ENABLE_IO >= 0
    amp_enable_set(!mute);
#endif
    ESP_LOGI(TAG, "mute=%d", (int)mute);
}

int MAX98357A_SetSampleRate(int sample_rate)
{
    if (!s_tx_chan || sample_rate <= 0) {
        return 0;
    }
    if (sample_rate == s_sample_rate) {
        return s_sample_rate;
    }

    /* 经功放线程执行, 与在途 PCM 保序 */
    amp_msg_t msg = {
        .type = AMP_MSG_RATE,
        .rate = sample_rate,
    };
    s_rate_result = 0;
    s_pending++;
    if (xQueueSend(s_work_q, &msg, pdMS_TO_TICKS(MAX98357A_WRITE_TIMEOUT_MS)) != pdTRUE) {
        s_pending--;
        ESP_LOGE(TAG, "rate msg enqueue failed");
        return 0;
    }
    if (xSemaphoreTake(s_rate_sem, pdMS_TO_TICKS(MAX98357A_WRITE_TIMEOUT_MS)) != pdTRUE) {
        ESP_LOGE(TAG, "rate switch wait timeout");
        return 0;
    }
    return s_rate_result;
}
