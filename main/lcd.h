/**
 * @file lcd.h
 * @brief ST7789/ILI9341 SPI LCD 驱动 (可移植模块)
 *
 * 移植自 retro-go_chaeng:
 *   components/retro-go/drivers/display/ili9341.h   (SPI 异步 DMA 队列 + 回收线程)
 *   components/retro-go/targets/t320-s3/config.h    (屏幕初始化序列/引脚)
 *
 * 使用方法：
 *   1. 修改下方"硬件配置"区的引脚/旋转宏
 *   2. LCD_Init();                  // 背光保持 0%, 防止初始化花屏
 *   3. LCD_Clear(LCD_BLACK); LCD_Sync(); vTaskDelay(80ms);
 *   4. LCD_SetBacklight(80);        // 清屏完成后再开背光
 *   5. LCD_ShowString(0, 0, "Hello", LCD_WHITE, LCD_BLACK, 2);
 *
 * 线程说明：
 *   LCD_Init() 内部创建高优先级 spi_task 线程, 负责回收 DMA 缓冲/事务
 *   (同 retro-go rg_spi 线程)。绘图 API 可多任务调用 (内部互斥锁保护),
 *   异步发送; 需要等待刷屏完成时调用 LCD_Sync()。
 *
 * 需要 ESP-IDF 组件: driver (SPI master + LEDC)
 */
#ifndef __LCD_H__
#define __LCD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "driver/spi_master.h"

/* ===== 硬件配置（移植时修改此处） ===== */
/* 引脚为 chaeng/t320-s3 板默认; 注意 LCD_RST=GPIO3 与 OLED SDA=GPIO3 冲突, 两者只能启用一个 */
#define LCD_SPI_HOST        SPI2_HOST
#define LCD_GPIO_MOSI       12
#define LCD_GPIO_MISO       (-1)                    /* 无 MISO */
#define LCD_GPIO_CLK        48
#define LCD_GPIO_CS         14
#define LCD_GPIO_DC         47
#define LCD_GPIO_RST        3                       /* 可设 -1 表示无复位脚 */
#define LCD_GPIO_BCKL       39                      /* 可设 -1 表示无背光脚 */
#define LCD_SPI_FREQ_HZ     (20 * 1000 * 1000)      /* 视频需要带宽: 320x240x2B @20M ≈ 16fps 上限; 屏花/不稳时降 10/5/2M, 稳定可试 SPI_MASTER_FREQ_40M */

/* 背光: 转接板 BLK 丝印"高电平点亮" */
#define LCD_BCKL_MODE       1   /* 0=GPIO 纯电平 1=LEDC PWM 渐变 (默认) */
#define LCD_BCKL_INVERT     0   /* 0=高电平亮 (BLK 高有效, 默认) 1=低电平亮; 背光诊断哪档亮填哪档 */

/* 面板初始化序列 */
#define LCD_PANEL_SEQ       1   /* 0=retro-go chaeng (t320-s3 屏调校)
                                   1=BOE3.2 金逸晨 GM 照搬 (默认: 本项目就是 BOE3.2 裸屏) */

/* init 逐条日志 (每条 cmd 打印并 LCD_Sync, 日志=真实发完; 定稿后置 0) */
#define LCD_INIT_TRACE      1

/* 屏幕尺寸/方向 — ST7789 3.2" IPS 240×320, 横屏 320×240 */
#define LCD_WIDTH           320
#define LCD_HEIGHT          240
/* MADCTL = (LCD_ROTATION << 5) | (LCD_RGB_BGR ? 0x08 : 0)
 * GM(金逸晨) USE_HORIZONTAL → madctl → 等价 LCD_ROTATION:
 *   0 → 0x40 竖屏        → 2
 *   1 → 0x80 竖屏(默认)  → 4
 *   2 → 0x20 横屏        → 1
 *   3 → 0xE0 横屏翻转    → 7
 * 本默认 3 → 0x60 (MX|MV) 横屏 (原 5→0xA0 底朝上, 已转 180° 修正) */
#define LCD_ROTATION        3
#define LCD_RGB_BGR         0   /* 0=RGB 1=BGR; 红蓝反色时改此值 */
#define LCD_INVON           1   /* 1=开启反显 (IPS 常黑屏需要, 颜色全反时改此值) */

/* 参数配置 */
#define LCD_BRIGHTNESS      80  /* 默认背光亮度 0-100 */
#define LCD_FONT_WIDTH      6   /* 内置 ASCII 字体 6x8 */
#define LCD_FONT_HEIGHT     8

/* ===== RGB565 颜色 (主机字节序, 驱动内部自动换为大端发往屏幕) ===== */
#define LCD_RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define LCD_BLACK           0x0000
#define LCD_WHITE           0xFFFF
#define LCD_RED             0xF800
#define LCD_GREEN           0x07E0
#define LCD_BLUE            0x001F
#define LCD_CYAN            0x07FF
#define LCD_MAGENTA         0xF81F
#define LCD_YELLOW          0xFFE0
#define LCD_ORANGE          0xFD20
#define LCD_GRAY            0x8410
#define LCD_DARKGRAY        0x4208

/* ===== API ===== */
void LCD_Init(void);             /* SPI 总线 + 屏幕初始化 + 启动 spi_task 线程 */
void LCD_Deinit(void);           /* 关背光并释放 SPI */
void LCD_Clear(uint16_t color);  /* 全屏填充 */
void LCD_FillRect(int x, int y, int width, int height, uint16_t color);
void LCD_DrawPixel(int x, int y, uint16_t color);
void LCD_WritePixels(int x, int y, int width, int height, const uint16_t *pixels); /* RGB565 源像素流 */
void LCD_ShowChar(int x, int y, char ch, uint16_t fg, uint16_t bg, int scale);
void LCD_ShowString(int x, int y, const char *str, uint16_t fg, uint16_t bg, int scale);
void LCD_SetBacklight(int percent);   /* 0-100 */
void LCD_SetRotation(int rotation);   /* MADCTL 0-7, 自动更新宽高 */
void LCD_Sync(void);                  /* 等待已排队的刷屏数据全部发完 */
void LCD_BacklightDiag(void);         /* 背光诊断: 高/低电平+PWM 各档, 串口打点; 在 LCD_Init 之前调用 */

#ifdef __cplusplus
}
#endif

#endif /* __LCD_H__ */
