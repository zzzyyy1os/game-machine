/**
 * @file mp3play.c
 * @brief MP3 文件播放 (解码 → I2S 喇叭) (可移植模块)
 * @note  喇叭引脚见 max98357a.h
 *
 * 流式解码: 4KB 输入缓冲读文件 → esp_mp3_dec 解出 16bit PCM → MAX98357A_Write。
 * 自动跳过 ID3v2 标签; 解码失步时逐字节前移重新找同步; 单声道原地扩立体声 (L=R)。
 */
#include "mp3play.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_mp3_dec.h"
#include "esp_audio_dec.h"
#include "max98357a.h"

static const char *TAG = "mp3play";

#define IN_BUF_SIZE    4096     /* 文件读取缓冲 */
#define OUT_BUF_SIZE   8192     /* PCM 输出缓冲 (> 1152样本x2声道x2字节 最大帧) */

static volatile bool s_stop = false;

/* 跳过 ID3v2 标签 (含封面图可达几百 KB, 不跳过解码器会逐字节扫描) */
static void skip_id3v2(FILE *f)
{
    uint8_t h[10];
    if (fread(h, 1, 10, f) != 10) {
        fseek(f, 0, SEEK_SET);
        return;
    }
    if (memcmp(h, "ID3", 3) != 0) {
        fseek(f, 0, SEEK_SET);
        return;
    }
    /* 标签长度 = synchsafe 28bit (不含 10 字节头); bit4=1 时还有 10 字节 footer */
    uint32_t sz = ((uint32_t)(h[6] & 0x7f) << 21) | ((uint32_t)(h[7] & 0x7f) << 14) |
                  ((uint32_t)(h[8] & 0x7f) << 7)  |  (uint32_t)(h[9] & 0x7f);
    if (h[5] & 0x10) {
        sz += 10;
    }
    fseek(f, sz, SEEK_CUR);
    ESP_LOGI(TAG, "skip ID3v2 tag (%lu bytes)", (unsigned long)sz);
}

bool MP3Play_PlayFile(const char *path)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        ESP_LOGE(TAG, "open %s failed", path);
        return false;
    }
    skip_id3v2(f);

    void *dec = NULL;
    esp_audio_err_t ret = esp_mp3_dec_open(NULL, 0, &dec);
    if (ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGE(TAG, "mp3 dec open failed: %d", (int)ret);
        fclose(f);
        return false;
    }

    uint8_t *in_buf  = malloc(IN_BUF_SIZE);
    uint8_t *pcm_buf = malloc(OUT_BUF_SIZE);
    if (!in_buf || !pcm_buf) {
        ESP_LOGE(TAG, "malloc failed");
        free(in_buf);
        free(pcm_buf);
        esp_mp3_dec_close(dec);
        fclose(f);
        return false;
    }

    size_t in_len = 0;          /* in_buf 内有效数据 */
    bool eof = false;
    bool ok = true;
    int channels = 2;
    int cur_rate = 0;
    s_stop = false;

    ESP_LOGI(TAG, "playing %s", path);

    while (!s_stop) {
        /* 1) 补输入 (保留未消费数据) */
        if (!eof && in_len < IN_BUF_SIZE) {
            size_t n = fread(in_buf + in_len, 1, IN_BUF_SIZE - in_len, f);
            if (n == 0) {
                eof = true;
            }
            in_len += n;
        }
        if (in_len == 0) {
            break;              /* 文件结束且缓冲已空 */
        }

        /* 2) 解一帧 */
        esp_audio_dec_in_raw_t  raw = { .buffer = in_buf, .len = in_len };
        esp_audio_dec_out_frame_t out = { .buffer = pcm_buf, .len = OUT_BUF_SIZE };
        esp_audio_dec_info_t info = {0};
        ret = esp_mp3_dec_decode(dec, &raw, &out, &info);

        if (ret == ESP_AUDIO_ERR_DATA_LACK) {
            /* 输入不够一帧: 继续读文件; 尾部残帧在 EOF 后自然退出 */
            if (eof) {
                break;
            }
            if (in_len >= IN_BUF_SIZE) {
                ESP_LOGW(TAG, "buff full but frame incomplete, resync skip 1");
                memmove(in_buf, in_buf + 1, --in_len);   /* 理论不会: 单帧 < 4KB */
            }
            continue;
        }

        if (ret == ESP_AUDIO_ERR_OK && out.decoded_size > 0) {
            /* 采样率变化 (含首帧) 时热切换 I2S 时钟 */
            if (info.sample_rate && (int)info.sample_rate != cur_rate) {
                if (MAX98357A_SetSampleRate((int)info.sample_rate) == 0) {
                    ESP_LOGE(TAG, "set sample rate %lu failed", (unsigned long)info.sample_rate);
                    ok = false;
                    break;
                }
                cur_rate = (int)info.sample_rate;
                channels = info.channel ? (int)info.channel : 2;
                ESP_LOGI(TAG, "stream: %d Hz, %d ch", cur_rate, channels);
            }

            /* 3) PCM 送喇叭 */
            int16_t *s = (int16_t *)pcm_buf;
            size_t samples = out.decoded_size / 2;      /* 全部通道样本数 */
            size_t frames;
            if (channels == 1) {
                /* 单声道扩立体声 (原地从后往前, dst>src 不会踩) */
                for (size_t i = samples; i > 0; i--) {
                    s[2 * i - 2] = s[i - 1];
                    s[2 * i - 1] = s[i - 1];
                }
                frames = samples;
            } else {
                frames = samples / 2;
            }
            /* 送功放线程 (入队); 短写只在超时, 重试剩余部分 */
            size_t wrote = 0;
            while (wrote < frames) {
                size_t n = MAX98357A_Write(s + wrote * 2, frames - wrote);
                if (n == 0) {
                    ESP_LOGE(TAG, "write short: %u/%u", (unsigned)wrote, (unsigned)frames);
                    ok = false;
                    break;
                }
                wrote += n;
            }
            if (!ok) {
                break;
            }
        } else if (ret != ESP_AUDIO_ERR_OK && ret != ESP_AUDIO_ERR_CONTINUE) {
            /* 坏帧/失步: 落到下面逐字节前移重新找同步 */
            ESP_LOGD(TAG, "decode ret=%d, resync", (int)ret);
        }

        /* 4) 消费输入; consumed=0 时至少前进 1 字节防死循环 */
        uint32_t consumed = raw.consumed ? raw.consumed : 1;
        if (consumed > in_len) {
            consumed = in_len;
        }
        memmove(in_buf, in_buf + consumed, in_len - consumed);
        in_len -= consumed;
    }

    esp_mp3_dec_close(dec);
    free(in_buf);
    free(pcm_buf);
    fclose(f);
    ESP_LOGI(TAG, "%s: %s", path, s_stop ? "stopped" : (ok ? "done" : "FAILED"));
    return ok;
}

void MP3Play_Stop(void)
{
    s_stop = true;
}
