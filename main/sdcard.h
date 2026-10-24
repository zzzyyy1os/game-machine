/**
 * @file sdcard.h
 * @brief SD 卡驱动 (SPI + FATFS) (可移植模块)
 *
 * 使用方法：
 *   1. 修改下方"硬件配置"区的 SPI 引脚宏
 *   2. SD_Init();                 // 初始化 SPI 总线并挂载 FAT 文件系统
 *   3. 之后直接用标准 C 库访问: fopen("/sd/xxx.txt", "r") / opendir("/sd") ...
 *   4. SD_Deinit();               // 卸载并释放 SPI 总线
 *
 * 需要 ESP-IDF 组件: fatfs (esp_vfs_fat), sdmmc (sdspi), driver (SPI master)
 */
#ifndef __SDCARD_H__
#define __SDCARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"

/* ===== 硬件配置（移植时修改此处） ===== */
/* 引脚为 chaeng/t320-s3 板默认; 原 KEY_4 (GPIO9) 已删除, GPIO9 现专用于 SD MISO */
#define SD_SPI_HOST           SPI3_HOST
#define SD_GPIO_MOSI          11
#define SD_GPIO_MISO          9
#define SD_GPIO_CLK           13
#define SD_GPIO_CS            10

#define SD_MOUNT_POINT        "/sd"
#define SD_MAX_FREQ_KHZ       20000       /* 20MHz (SDMMC_FREQ_DEFAULT), 失败自动降 400kHz 重试 */
#define SD_MAX_FILES          4

/* ===== 类型定义 ===== */
/* (无, 挂载后通过标准 C 库 stdio/dirent 访问文件) */

/* ===== API ===== */
void SD_Init(void);             /* SPI 总线 + 挂载; 失败仅记录日志不复位 (卡可不在位) */
bool SD_IsReady(void);          /* 是否已挂载 */
void SD_Deinit(void);           /* 卸载文件系统并释放 SPI 总线 */

#ifdef __cplusplus
}
#endif

#endif /* __SDCARD_H__ */
