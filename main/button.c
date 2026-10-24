/**
 * @file button.c
 * @brief 按键检测 - 消抖 + 短按/长按/双击 (可移植模块, 事件驱动)
 *        按键接法：接 GND, 低电平有效 (KEY_ACTIVE_LOW=1, 上拉输入)
 * @note  GPIO 引脚/极性配置见 button.h
 *
 * 线程模型 (无轮询):
 *   GPIO 双边沿中断 → 任务通知唤醒按键线程; 线程按状态机下一到期时刻
 *   计算阻塞超时 (消抖 20ms / 长按 1s / 双击窗 300ms), 到期才醒来推进。
 *   无事件时线程完全挂起。
 */
#include "button.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "button";

#define KEY_TASK_STACK      3072
#define KEY_TASK_PRIO       5

/* 状态机状态 */
typedef enum {
    ST_IDLE = 0,           /* 空闲，等待按下 */
    ST_DEBOUNCE_PRESS,     /* 按下消抖 */
    ST_PRESSED,            /* 已确认按下，等待松开或长按超时 */
    ST_DEBOUNCE_RELEASE,   /* 松开消抖 */
    ST_WAIT_2ND_PRESS,     /* 等待第二次按下 (双击窗口) */
    ST_DEBOUNCE_2ND,       /* 第二次按下消抖 */
    ST_WAIT_2ND_RELEASE,   /* 等待第二次松开 */
} KeyState_t;

/* 按键实例 */
typedef struct {
    gpio_num_t    gpio;
    KeyState_t    state;
    uint32_t      tick;           /* 状态计时 */
} Key_t;

static Key_t keys[KEY_NUM] = {
    { .gpio = KEY_GPIO_1,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_2,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_3,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_4,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_5,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_6,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_7,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_8,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_9,  .state = ST_IDLE },
    { .gpio = KEY_GPIO_10, .state = ST_IDLE },
};

static TaskHandle_t    s_key_task = NULL;
static KeyCallback_t   s_cb = NULL;

/* 读取引脚: 1=按下, 0=释放 (按 KEY_ACTIVE_LOW 区分极性) */
static inline uint8_t read_pin(Key_t *k)
{
    int lv = gpio_get_level(k->gpio);
#if KEY_ACTIVE_LOW
    return (lv == 0) ? 1 : 0;
#else
    return (lv != 0) ? 1 : 0;
#endif
}

/* GPIO 中断: 只唤醒按键线程 (IRAM 内不做其他事) */
static void IRAM_ATTR key_isr(void *arg)
{
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(s_key_task, &hpw);
    if (hpw) {
        portYIELD_FROM_ISR(hpw);
    }
}

/* 状态机下一到期时刻的阻塞 tick 数 (无定时状态时永久阻塞) */
static TickType_t key_wait_ticks(TickType_t now)
{
    TickType_t wait = portMAX_DELAY;

    for (int i = 0; i < KEY_NUM; i++) {
        Key_t *k = &keys[i];
        TickType_t deadline;

        switch (k->state) {
        case ST_DEBOUNCE_PRESS:
        case ST_DEBOUNCE_RELEASE:
        case ST_DEBOUNCE_2ND:
            deadline = k->tick + pdMS_TO_TICKS(KEY_DEBOUNCE_MS);
            break;
        case ST_PRESSED:
            deadline = k->tick + pdMS_TO_TICKS(KEY_LONG_PRESS_MS);
            break;
        case ST_WAIT_2ND_PRESS:
            deadline = k->tick + pdMS_TO_TICKS(KEY_DOUBLE_CLICK_MS);
            break;
        default:
            continue;
        }

        TickType_t remain = deadline - now;     /* tick 回绕安全 */
        if (remain == 0) {
            return 0;
        }
        if (remain < wait) {
            wait = remain;
        }
    }
    return wait;
}

/* 推进一轮状态机 (边沿唤醒或定时到期时调用), 事件经回调送出 */
static void key_step(void)
{
    uint32_t now = xTaskGetTickCount();

    for (int i = 0; i < KEY_NUM; i++) {
        Key_t *k = &keys[i];
        uint8_t pressed = read_pin(k);
        KeyEvent_t ev = KEY_EVENT_NONE;

        switch (k->state) {

        case ST_IDLE:
            if (pressed) {
                k->state = ST_DEBOUNCE_PRESS;
                k->tick = now;
            }
            break;

        case ST_DEBOUNCE_PRESS:
            if (!pressed) {
                k->state = ST_IDLE;
            } else if ((now - k->tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)) {
                k->state = ST_PRESSED;
            }
            break;

        case ST_PRESSED:
            if (!pressed) {
                k->state = ST_DEBOUNCE_RELEASE;
                k->tick = now;
            } else if ((now - k->tick) >= pdMS_TO_TICKS(KEY_LONG_PRESS_MS)) {
                ev = KEY_EVENT_LONG_PRESS;
                k->state = ST_IDLE;
            }
            break;

        case ST_DEBOUNCE_RELEASE:
            if (pressed) {
                k->state = ST_PRESSED;
            } else if ((now - k->tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)) {
                k->state = ST_WAIT_2ND_PRESS;
                k->tick = now;
            }
            break;

        case ST_WAIT_2ND_PRESS:
            if (pressed) {
                k->state = ST_DEBOUNCE_2ND;
                k->tick = now;
            } else if ((now - k->tick) >= pdMS_TO_TICKS(KEY_DOUBLE_CLICK_MS)) {
                ev = KEY_EVENT_SHORT_PRESS;
                k->state = ST_IDLE;
            }
            break;

        case ST_DEBOUNCE_2ND:
            if (!pressed) {
                k->state = ST_WAIT_2ND_PRESS;
            } else if ((now - k->tick) >= pdMS_TO_TICKS(KEY_DEBOUNCE_MS)) {
                k->state = ST_WAIT_2ND_RELEASE;
            }
            break;

        case ST_WAIT_2ND_RELEASE:
            if (!pressed) {
                ev = KEY_EVENT_DOUBLE_CLICK;
                k->state = ST_IDLE;
            }
            break;
        }

        if (ev != KEY_EVENT_NONE && s_cb) {
            s_cb((KeyId_t)i, ev);
        }
    }
}

/* 按键线程: 边沿事件 / 状态机到期 才唤醒, 其余时间挂起 */
static void key_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, key_wait_ticks(xTaskGetTickCount()));
        key_step();
    }
}

void Key_Init(KeyCallback_t cb)
{
    s_cb = cb;

    /* 引脚掩码由 keys[] 表生成, 与 KEY_GPIO_x 自动保持一致 */
    uint64_t pin_mask = 0;
    for (int i = 0; i < KEY_NUM; i++) {
        pin_mask |= (1ULL << keys[i].gpio);
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
#if KEY_ACTIVE_LOW
        .pull_up_en = GPIO_PULLUP_ENABLE,       /* 接 GND: 上拉, 低有效 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
#else
        .pull_up_en = GPIO_PULLUP_DISABLE,      /* 接 3.3V: 下拉, 高有效 */
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
#endif
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&io_conf);

    for (int i = 0; i < KEY_NUM; i++) {
        keys[i].state = ST_IDLE;
        keys[i].tick = 0;
    }

    xTaskCreate(key_task, "key_task", KEY_TASK_STACK, NULL, KEY_TASK_PRIO, &s_key_task);

    /* 安装 GPIO 中断服务 (可能已被其他模块装过) */
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "isr service install failed: %s", esp_err_to_name(err));
    }
    for (int i = 0; i < KEY_NUM; i++) {
        gpio_isr_handler_add(keys[i].gpio, key_isr, (void *)(intptr_t)i);
    }

#if KEY_ACTIVE_LOW
    ESP_LOGI(TAG, "Keys initialized (%d keys, IRQ-driven, active low, pull-up)", KEY_NUM);
#else
    ESP_LOGI(TAG, "Keys initialized (%d keys, IRQ-driven, active high, pull-down)", KEY_NUM);
#endif
}

/* 获取按键当前状态 (0=释放, 1=按下) */
uint8_t Key_IsPressed(KeyId_t id)
{
    if (id >= KEY_NUM) return 0;
    return read_pin(&keys[id]);
}
