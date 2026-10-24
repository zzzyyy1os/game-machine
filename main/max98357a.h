/**
 * @file max98357a.h
 * @brief MAX98357A I2S 数字功放驱动 (可移植模块, 功放专用线程)
 *
 * 线程模型: MAX98357A_Init() 内部创建功放线程。
 *   生产者 (如 MP3 解码任务) 调 MAX98357A_Write() 把 PCM 入队即返回继续解码,
 *   功放线程出队并写 I2S —— 解码与放音并行, 无轮询。
 *   采样率热切换 (SetSampleRate) 也经队列下达, 与 PCM 保序。
 *
 * 使用方法：
 *   1. 修改下方"硬件配置"区的引脚宏 (BCLK/WS/DOUT)
 *   2. MAX98357A_Init(32000);              // I2S + 功放线程; 采样率按应用传入
 *   3. MAX98357A_SetVolume(50);            // 0-100
 *   4. MAX98357A_Write(samples, frames);   // 16bit 有符号立体声交错, 入队 (满则阻塞)
 *   5. MAX98357A_WaitDrained();            // 可选: 等队列全部播完 (换曲/停播前)
 *
 * 需要 ESP-IDF 组件: esp_driver_i2s (新版 driver/i2s_std.h), driver (GPIO)
 */
#ifndef __MAX98357A_H__
#define __MAX98357A_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ===== 硬件配置（移植时修改此处） ===== */
#define MAX98357A_GPIO_BCLK          41
#define MAX98357A_GPIO_WS            42      /* LRCLK */
#define MAX98357A_GPIO_DOUT          40

#define MAX98357A_AMP_ENABLE_IO      (-1)    /* SD_MODE/功放使能脚, -1 = 不使用(硬件拉高) */
#define MAX98357A_AMP_ENABLE_INVERT  0       /* 1 = 低电平使能功放 */

#define MAX98357A_DEFAULT_SAMPLE_RATE 32000
#define MAX98357A_DMA_DESC_NUM        4       /* DMA 描述符个数 */
#define MAX98357A_DMA_FRAME_NUM       180     /* 每个 DMA 缓冲帧数 */
#define MAX98357A_DEFAULT_VOLUME      50      /* 0-100 */
#define MAX98357A_WRITE_TIMEOUT_MS    1000    /* 入队/出队写超时 */

/* PCM 队列深度 (缓冲块数) 与 每块最大帧数 (1 个 MP3 帧 = 1152 采样) */
#define MAX98357A_PCM_BUF_NUM         4
#define MAX98357A_PCM_BUF_FRAMES      1152

/* ===== 类型定义 ===== */
/* 样本格式: 16bit 有符号, 立体声交错 [L0 R0 L1 R1 ...], 1帧 = 2 个 int16 */

/* ===== API ===== */
void   MAX98357A_Init(int sample_rate);                  /* <=0 时用 MAX98357A_DEFAULT_SAMPLE_RATE; 启动功放线程 */
void   MAX98357A_Deinit(void);
size_t MAX98357A_Write(const int16_t *samples, size_t frames); /* 入队给功放线程, 返回已接收帧数 (队满超时则短写) */
void   MAX98357A_WaitDrained(uint32_t timeout_ms);       /* 等 PCM 队列播完; 超时返回 */
void   MAX98357A_SetVolume(int percent);                 /* 0-100, 超出自动钳位 */
void   MAX98357A_SetMute(bool mute);
int    MAX98357A_SetSampleRate(int sample_rate);         /* 热切换(经队列保序), 返回生效采样率; 失败返回 0 */

#ifdef __cplusplus
}
#endif

#endif /* __MAX98357A_H__ */
