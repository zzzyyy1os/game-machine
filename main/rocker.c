/**
 * @file rocker.c
 * @brief 4通道ADC摇杆驱动 - 可移植模块 (定时器事件驱动, 无轮询)
 * @note  ADC通道映射、中心值等配置见 rocker.h
 *
 * 线程模型: esp_timer 每 ROCKER_PERIOD_MS 发任务通知唤醒采样线程,
 *   线程采样+滤波+映射后更新缓存并触发回调, 然后挂起等待下次通知。
 */
#include "rocker.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "rocker";

#define ROCKER_TASK_STACK   3072
#define ROCKER_TASK_PRIO    4

/* ADC 常量 */
#define ADC_CHANNELS       4
#define ADC_ATTEN          ADC_ATTEN_DB_12
#define ADC_BITWIDTH       ADC_BITWIDTH_12
#define ADC_MAX_VALUE      4095

/* 各通道配置 (引用 rocker.h 宏) */
static const adc_channel_t adc_channels[ADC_CHANNELS] = {
    ROCKER_CH_LX,
    ROCKER_CH_LY,
    ROCKER_CH_RY,
    ROCKER_CH_RX,
};

static const int adc_center[ADC_CHANNELS] = {
    ROCKER_CENTER_LX,
    ROCKER_CENTER_LY,
    ROCKER_CENTER_RY,
    ROCKER_CENTER_RX,
};

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc_cali_handles[ADC_CHANNELS] = {NULL};

/* 最近一次采样值缓存 */
static rocker_value_t s_value = {0, 0, 0, 0};

static TaskHandle_t      s_rocker_task = NULL;
static RockerCallback_t  s_cb = NULL;
static esp_timer_handle_t s_timer = NULL;

/* 定时器到点 (esp_timer 线程上下文): 只唤醒采样线程 */
static void rocker_timer_cb(void *arg)
{
    xTaskNotifyGive(s_rocker_task);
}

static void rocker_task(void *arg);   /* 定义见下 */

/* ADC calibration init */
static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel,
                                  adc_atten_t atten, adc_cali_handle_t *out_handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH,
    };
    return (adc_cali_create_scheme_curve_fitting(&cali_config, out_handle) == ESP_OK);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH,
    };
    return (adc_cali_create_scheme_line_fitting(&cali_config, out_handle) == ESP_OK);
#else
    return false;
#endif
}

void rocker_init(void)
{
    /* ADC Init */
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ROCKER_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    /* Configure each channel */
    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };

    for (int i = 0; i < ADC_CHANNELS; i++) {
        ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc_channels[i], &chan_config));
        if (!adc_calibration_init(ROCKER_ADC_UNIT, adc_channels[i], ADC_ATTEN, &adc_cali_handles[i])) {
            ESP_LOGW(TAG, "ADC calibration failed for channel %d", i);
        }
    }

    ESP_LOGI(TAG, "ADC initialized");

    /* 采样线程 + 周期定时器 (事件驱动唤醒, 不轮询) */
    xTaskCreate(rocker_task, "rocker_task", ROCKER_TASK_STACK, NULL, ROCKER_TASK_PRIO, &s_rocker_task);

    const esp_timer_create_args_t targs = {
        .callback = rocker_timer_cb,
        .name = "rocker_tm",
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &s_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(s_timer, (uint64_t)ROCKER_PERIOD_MS * 1000));
    ESP_LOGI(TAG, "rocker task + %dms timer started", ROCKER_PERIOD_MS);
}

/* Map ADC raw value to -OUTPUT_MAX ~ +OUTPUT_MAX */
static int map_adc_to_range(int raw, int ch)
{
    int center = adc_center[ch];
    int diff = raw - center;

    /* 死区处理：中心附近视为0 */
    if (diff > -ROCKER_DEADZONE && diff < ROCKER_DEADZONE) {
        return 0;
    }

    /* 正方向映射 */
    if (diff > 0) {
        int mapped = (diff - ROCKER_DEADZONE) * ROCKER_OUTPUT_MAX / (ADC_MAX_VALUE - center - ROCKER_DEADZONE);
        return (mapped > ROCKER_OUTPUT_MAX) ? ROCKER_OUTPUT_MAX : mapped;
    }

    /* 负方向映射 */
    int mapped = (-diff - ROCKER_DEADZONE) * ROCKER_OUTPUT_MAX / (center - ROCKER_DEADZONE);
    return (mapped > ROCKER_OUTPUT_MAX) ? -ROCKER_OUTPUT_MAX : -mapped;
}

void rocker_read(rocker_value_t *val)
{
    /* 每个通道独立连续采样，采样完再切下一个通道 */
    int sum[ADC_CHANNELS] = {0};
    for (int i = 0; i < ADC_CHANNELS; i++) {
        esp_rom_delay_us(500);  /* 切换通道前等待500us */
        for (int s = 0; s < ROCKER_SAMPLES; s++) {
            int v;
            ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc_channels[i], &v));
            sum[i] += v;
        }
    }

    /* 映射到 -OUTPUT_MAX ~ +OUTPUT_MAX */
    s_value.lx = map_adc_to_range(sum[0] / ROCKER_SAMPLES, 0);
    s_value.ly = map_adc_to_range(sum[1] / ROCKER_SAMPLES, 1);
#if ROCKER_INVERT_RY
    s_value.ry = -map_adc_to_range(sum[2] / ROCKER_SAMPLES, 2);
#else
    s_value.ry = map_adc_to_range(sum[2] / ROCKER_SAMPLES, 2);
#endif
#if ROCKER_INVERT_RX
    s_value.rx = -map_adc_to_range(sum[3] / ROCKER_SAMPLES, 3);
#else
    s_value.rx = map_adc_to_range(sum[3] / ROCKER_SAMPLES, 3);
#endif

#if ROCKER_DEBUG_LOG
    /* 列顺序统一为 LX LY RX RY (通道表里 RX/RY 是反的, 这里按 val 列对齐) */
    ESP_LOGI(TAG, "raw: %4d %4d %4d %4d  val: %4d %4d %4d %4d",
             sum[0] / ROCKER_SAMPLES, sum[1] / ROCKER_SAMPLES,
             sum[3] / ROCKER_SAMPLES, sum[2] / ROCKER_SAMPLES,
             s_value.lx, s_value.ly, s_value.rx, s_value.ry);
#endif

    if (val) {
        *val = s_value;
    }
    if (s_cb) {
        s_cb(&s_value);
    }
}

/* 采样线程: 定时器通知唤醒才采样, 其余时间挂起 */
static void rocker_task(void *arg)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        rocker_read(NULL);
    }
}

void rocker_set_callback(RockerCallback_t cb)
{
    s_cb = cb;
}

void rocker_get(rocker_value_t *val)
{
    if (val) {
        *val = s_value;
    }
}
