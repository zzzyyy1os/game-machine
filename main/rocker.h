/**
 * @file rocker.h
 * @brief 4通道ADC摇杆驱动 (可移植模块)
 *
 * 线程模型: rocker_init() 内部创建摇杆采样线程 + esp_timer 周期定时器。
 *   定时器到点发任务通知唤醒线程采样, 平时线程挂起 —— 不轮询。
 *   可选回调 rocker_set_callback() 在每次采样后收到结果 (线程上下文)。
 *
 * 使用方法：
 *   1. 修改下方"硬件配置"区的宏定义
 *   2. rocker_init();                  // ADC + 采样线程
 *   3. rocker_set_callback(cb);        // 可选: 每周期收到采样值
 *   4. rocker_get(&val)                // 随时读最近一次缓存值
 *
 * 需要 ESP-IDF 组件: esp_adc
 */
#ifndef __ROCKER_H__
#define __ROCKER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_adc/adc_oneshot.h"

/* ===== 硬件配置（移植时修改此处） ===== */

/* ADC 通道映射 (ESP32-S3 ADC1) — 接线 GPIO5/6/1/2, 顺序 LX/LY/RY/RX */
#define ROCKER_ADC_UNIT       ADC_UNIT_1
#define ROCKER_CH_LX          ADC_CHANNEL_4   /* GPIO5  */
#define ROCKER_CH_LY          ADC_CHANNEL_5   /* GPIO6  — 必须用 IO6 (原理图 L_ADC2); 同线在 IO4 验证过是好的, 若浮空贴 4095 查 IO6 焊点 */
#define ROCKER_CH_RY          ADC_CHANNEL_0   /* GPIO1  */
#define ROCKER_CH_RX          ADC_CHANNEL_1   /* GPIO2  */

/* 各通道静止时的中心值 (根据实际测量校准, 2026-10-25 串口实测) */
#define ROCKER_CENTER_LX      1691
#define ROCKER_CENTER_LY      1938   /* 实测静止值 (wiper 本身完好, 与接哪个脚无关) */
#define ROCKER_CENTER_RY      1721
#define ROCKER_CENTER_RX      1660

/* 参数配置 */
#define ROCKER_DEADZONE       120   /* 死区范围，中心附近视为0 */
#define ROCKER_SAMPLES        32    /* 每通道采样次数 */
#define ROCKER_OUTPUT_MAX     100   /* 输出最大值 (-100 ~ +100) */
#define ROCKER_INVERT_RY      1     /* 右摇杆Y轴反转 */
#define ROCKER_INVERT_RX      1     /* 右摇杆X轴反转 */
#define ROCKER_DEBUG_LOG      1     /* 调试打印开关 (1=开, 0=关) */
#define ROCKER_PERIOD_MS      50    /* 采样周期 (定时器唤醒间隔) */

/* ===== API ===== */

/* 摇杆数值结构 (-100 ~ +100) */
typedef struct {
    int lx;
    int ly;
    int rx;
    int ry;
} rocker_value_t;

/* 采样回调: 每周期采样完成后调用 (摇杆线程上下文), 勿做阻塞操作 */
typedef void (*RockerCallback_t)(const rocker_value_t *val);

void rocker_init(void);
void rocker_set_callback(RockerCallback_t cb);  /* 可选, 任意时机注册 */
void rocker_read(rocker_value_t *val);   /* 采样+滤波+映射，并更新缓存 */
void rocker_get(rocker_value_t *val);    /* 仅读缓存 */

#ifdef __cplusplus
}
#endif

#endif /* __ROCKER_H__ */
