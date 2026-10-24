/**
 * @file button.h
 * @brief 按键检测模块 - 消抖 + 短按/长按/双击 (可移植模块, 事件驱动)
 *
 * 线程模型: Key_Init() 内部创建按键线程 + GPIO 双边沿中断。
 *   中断只做唤醒 (任务通知), 线程在"边沿事件 / 状态机到期"时才运行,
 *   空闲时阻塞挂起 —— 不轮询。事件通过回调送达 (运行在按键线程上下文)。
 *
 * 使用方法：
 *   1. 修改下方"硬件配置"区的 GPIO 宏定义
 *   2. Key_Init(on_event);          // 传回调, 收 短按/长按/双击 事件
 *   3. Key_IsPressed(id)            // 随时读当前电平
 *
 * 硬件接法: ESP掌机 Board1 原理图 (SCH_Schematic1_2026-10-21) 所有按键接 GND,
 *   低电平有效 (KEY_ACTIVE_LOW=1 → 上拉输入)。若硬件改接 3.3V, 置 0 (下拉输入)。
 *
 * 需要 ESP-IDF 组件: driver (GPIO)
 */
#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"
#include <stdint.h>

/* ===== 硬件配置（移植时修改此处） ===== */
/* ESP掌机 Board1 原理图: 8 个 K2-1817UQ 侧键 + 确认/返回 2 键 (TS4538B3J), 共 10 键 */
#define KEY_NUM               10

/* 8 个侧键 (原理图左列自上而下, 依次) */
#define KEY_GPIO_1            GPIO_NUM_17
#define KEY_GPIO_2            GPIO_NUM_18
#define KEY_GPIO_3            GPIO_NUM_19
#define KEY_GPIO_4            GPIO_NUM_20
#define KEY_GPIO_5            GPIO_NUM_7
#define KEY_GPIO_6            GPIO_NUM_8
#define KEY_GPIO_7            GPIO_NUM_21
#define KEY_GPIO_8            GPIO_NUM_16
/* 确认/返回 (原理图"确认与返回") */
#define KEY_GPIO_9            GPIO_NUM_15    /* 确认 */
#define KEY_GPIO_10           GPIO_NUM_38    /* 返回 */

/* 按键极性: 原理图按键一端接 IO 一端接 GND → 低电平有效, 上拉输入 */
#define KEY_ACTIVE_LOW        1   /* 1=低电平有效(接GND, 上拉, 原理图接法)
                                   * 0=高电平有效(接3.3V, 下拉) */

/* 时序参数 */
#define KEY_DEBOUNCE_MS       20     /* 消抖时间 */
#define KEY_LONG_PRESS_MS     1000   /* 长按阈值 */
#define KEY_DOUBLE_CLICK_MS   300    /* 双击间隔窗口 */

/* ===== 类型定义 ===== */

typedef enum {
    KEY_1 = 0,
    KEY_2 = 1,
    KEY_3 = 2,
    KEY_4 = 3,
    KEY_5 = 4,
    KEY_6 = 5,
    KEY_7 = 6,
    KEY_8 = 7,
    KEY_9 = 8,          /* 确认 (IO15) */
    KEY_10 = 9,         /* 返回 (IO38) */
    KEY_CONFIRM = 8,    /* = KEY_9  确认 */
    KEY_BACK = 9,       /* = KEY_10 返回 */
} KeyId_t;

typedef enum {
    KEY_EVENT_NONE = 0,
    KEY_EVENT_SHORT_PRESS,   /* 短按 (松开后触发) */
    KEY_EVENT_LONG_PRESS,    /* 长按 (达到阈值触发一次) */
    KEY_EVENT_DOUBLE_CLICK,  /* 双击 */
} KeyEvent_t;

/* 事件回调: 在按键线程上下文调用, 勿做阻塞操作 */
typedef void (*KeyCallback_t)(KeyId_t id, KeyEvent_t event);

/* ===== API ===== */

void    Key_Init(KeyCallback_t cb);   /* GPIO+中断+按键线程; cb 可为 NULL */
uint8_t Key_IsPressed(KeyId_t id);    /* 获取按键当前状态 (0=释放, 1=按下) */

#ifdef __cplusplus
}
#endif

#endif /* __BUTTON_H__ */
