/**
 * @file sdcard.c
 * @brief SD 卡驱动 (SPI + FATFS) (可移植模块)
 * @note  SPI 引脚配置见 sdcard.h
 *
 * 移植自 retro-go rg_storage.c (SDSPI 分支): spi_bus_initialize + esp_vfs_fat_sdspi_mount,
 * 挂载失败(TIMEOUT/INVALID_RESPONSE/INVALID_CRC)自动降 400kHz 重试一次;
 * 挂载点 /sd, 之后文件访问用标准 C 库 (fopen/fread/opendir ...).
 */
#include "sdcard.h"

#include <stdio.h>
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "sdcard";

static sdmmc_card_t *s_card = NULL;
static bool s_mounted = false;

void SD_Init(void)
{
    if (s_mounted) {
        return;
    }

    /* CS 先拉高, 避免挂载前 SD 卡占用总线 (同 retro-go platform_init) */
    gpio_reset_pin(SD_GPIO_CS);
    gpio_set_direction(SD_GPIO_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(SD_GPIO_CS, 1);

    /* 1) 初始化 SPI 总线 */
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_GPIO_MOSI,
        .miso_io_num = SD_GPIO_MISO,
        .sclk_io_num = SD_GPIO_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    esp_err_t err = spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return;
    }

    /* 2) 挂载 FAT 文件系统 */
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SD_SPI_HOST;
    host.max_freq_khz = SD_MAX_FREQ_KHZ;

    sdspi_device_config_t slot = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot.host_id = SD_SPI_HOST;
    slot.gpio_cs = SD_GPIO_CS;

    esp_vfs_fat_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = 0,
    };

    err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
    if (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_RESPONSE || err == ESP_ERR_INVALID_CRC) {
        ESP_LOGW(TAG, "mount @%d kHz failed (%s), retry @%d kHz",
                 SD_MAX_FREQ_KHZ, esp_err_to_name(err), SDMMC_FREQ_PROBING);
        host.max_freq_khz = SDMMC_FREQ_PROBING;
        err = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mount failed: %s (check wiring/card/format)", esp_err_to_name(err));
        spi_bus_free(SD_SPI_HOST);
        return;
    }

    s_mounted = true;
    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "SD card mounted at %s (MOSI=%d MISO=%d CLK=%d CS=%d)",
             SD_MOUNT_POINT, SD_GPIO_MOSI, SD_GPIO_MISO, SD_GPIO_CLK, SD_GPIO_CS);
}

bool SD_IsReady(void)
{
    return s_mounted;
}

void SD_Deinit(void)
{
    if (!s_mounted) {
        return;
    }
    esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, s_card);
    s_card = NULL;
    s_mounted = false;
    spi_bus_free(SD_SPI_HOST);
    ESP_LOGI(TAG, "SD card unmounted");
}
