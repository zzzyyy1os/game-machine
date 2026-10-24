/**
 * @file mp4play.c
 * @brief SD 卡 MP4 视频 → LCD 播放 (精简解码管线, 只要画面)
 *
 * 移植自 esp-vedio (ESP-GMF) packages/esp_video_render/examples/video_player:
 *   去掉 GMF 管线 / esp_video_render / A-V 同步, 双线程直连:
 *     fopen → esp_extractor (MP4 解封装, 只取视频轨)
 *           → esp_video_dec  (SW H.264 / MJPEG 解码)          [解码线程]
 *           → [YUV420P→RGB565 软件转换, 解码器直出 RGB565 时跳过]
 *           → 帧队列 → LCD 渲染线程 (pts 节流 + LCD_WritePixels)
 *
 * 线程模型 (无轮询):
 *   解码线程 (调用方, app_main 创建) 解完一帧入队 (缓冲池满则阻塞背压);
 *   LCD 渲染线程 lcd_task 阻塞等帧队列, 按 pts 定点推屏后归还缓冲。
 *   LCD 内部另有 spi_task 回收 DMA (见 lcd.h), 三层流水。
 *
 * 节流: 按帧 pts (毫秒) 用 esp_timer 定点显示; pts 异常时退回固定帧率。
 * 循环: 播到 EOS 关闭重开, 永不返回; 文件打不开时 LCD 报错并 3s 重试。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_extractor.h"
#include "esp_extractor_defaults.h"
#include "esp_video_codec_types.h"
#include "esp_video_dec.h"
#include "esp_video_dec_default.h"
#include "esp_video_codec_utils.h"
#include "lcd.h"
#include "mp4play.h"

static const char *TAG = "mp4play";

#define IO_CACHE_SIZE       (32 * 1024)             /* stdio 读缓存 (加速 SD 随机读) */
#define EXTRACT_POOL_SIZE   (256 * 1024)            /* 抽取器输出帧池 (同官方例程) */
#define DEFAULT_FPS         15
#define MAX_CONSEC_ERR      50                      /* 连续读帧错误超过此次数放弃本轮 */
#define MAX_PROBE_FAIL      30                      /* 探测分辨率连续失败次数上限 */
#define RETRY_DELAY_MS      3000

#define FRAME_POOL_NUM      3                       /* RGB565 帧缓冲池 (解码/显示流水) */
#define LCD_TASK_STACK      (3 * 1024)
#define LCD_TASK_PRIO       5

/* ===== 文件 IO 回调 (喂给 esp_extractor) ===== */
typedef struct {
    FILE    *fp;
    uint8_t *io_cache;
} file_src_t;

static int file_read_cb(void *buffer, uint32_t size, void *ctx)
{
    return (int)fread(buffer, 1, size, ((file_src_t *)ctx)->fp);
}

static int file_seek_cb(uint32_t position, void *ctx)
{
    return fseek(((file_src_t *)ctx)->fp, position, SEEK_SET);
}

static uint32_t file_size_cb(void *ctx)
{
    FILE *fp = ((file_src_t *)ctx)->fp;
    long old = ftell(fp);
    fseek(fp, 0, SEEK_END);
    long end = ftell(fp);
    fseek(fp, old, SEEK_SET);
    return end > 0 ? (uint32_t)end : 0;
}

/* ===== 帧队列: 解码线程 → LCD 渲染线程 ===== */
typedef struct {
    uint16_t   *buf;            /* RGB565 帧缓冲 (池中取出) */
    int         w, h;
    uint32_t    pts;            /* 帧 pts (毫秒) */
    uint32_t    interval_ms;    /* pts 异常时兜底帧间隔 */
} frame_msg_t;

static QueueHandle_t s_pool_free = NULL;    /* 空闲帧缓冲指针 */
static QueueHandle_t s_frame_q   = NULL;    /* 已解码待显示帧 */
static int s_pool_n = 0;                    /* 池中已分配缓冲数 */

/* pts 节流状态 (LCD 渲染线程持有; open 时复位) */
static struct {
    uint32_t last_pts;
    int64_t  t0_us;
    uint32_t pts0;
    bool     paced;
    int      shown;
    int64_t  fps_t0;
} s_pace;

/* ===== 播放上下文 ===== */
typedef struct {
    FILE                       *fp;
    uint8_t                    *io_cache;
    file_src_t                 *src;          /* 抽取器 in_ctx, close 时统一释放 */
    esp_extractor_handle_t      extractor;
    esp_video_dec_handle_t      dec;
    esp_video_codec_pixel_fmt_t out_fmt;
    bool                        need_convert;   /* 解码出 YUV420P 需软件转 RGB565 */
    uint8_t                     out_align;      /* 解码输出缓冲对齐 */
    int                         probe_fail;     /* 分辨率探测失败计数 */
    uint8_t                    *out_buf;        /* 解码输出 (YUV420P 或 RGB565_LE) */
    uint32_t                    out_buf_size;
    int                         vid_w, vid_h;
    uint32_t                    interval_ms;    /* 兜底帧间隔 */
} play_ctx_t;

/* ===== YUV420P → RGB565 (BT.601 有限范围) ===== */
static void yuv420p_to_rgb565(const uint8_t *yuv, int w, int h, uint16_t *rgb)
{
    const uint8_t *yp = yuv;
    const uint8_t *up = yuv + w * h;
    const uint8_t *vp = up + (w / 2) * (h / 2);
    for (int y = 0; y < h; y++) {
        const uint8_t *yrow = yp + y * w;
        const uint8_t *urow = up + (y / 2) * (w / 2);
        const uint8_t *vrow = vp + (y / 2) * (w / 2);
        uint16_t *out = rgb + y * w;
        for (int x = 0; x < w; x++) {
            int c = yrow[x] - 16;
            int d = urow[x >> 1] - 128;
            int e = vrow[x >> 1] - 128;
            int r = (298 * c + 409 * e + 128) >> 8;
            int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
            int b = (298 * c + 516 * d + 128) >> 8;
            if (r < 0) r = 0; else if (r > 255) r = 255;
            if (g < 0) g = 0; else if (g > 255) g = 255;
            if (b < 0) b = 0; else if (b > 255) b = 255;
            out[x] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        }
    }
}

/* ===== 推屏: 整帧居中, 超屏部分裁剪 ===== */
static void present_rgb(const uint16_t *rgb, int w, int h)
{
    int dw = w < LCD_WIDTH ? w : LCD_WIDTH;
    int dh = h < LCD_HEIGHT ? h : LCD_HEIGHT;
    int x_off = (LCD_WIDTH - dw) / 2;
    int y_off = (LCD_HEIGHT - dh) / 2;
    int x_src = (w - dw) / 2;
    int y_src = (h - dh) / 2;

    if (w == dw) {
        /* 无水平裁剪: 整块连续, 一次推完 */
        LCD_WritePixels(x_off, y_off, dw, dh, rgb + y_src * w);
    } else {
        /* 水平裁剪: 逐行推 */
        for (int y = 0; y < dh; y++) {
            LCD_WritePixels(x_off, y_off + y, dw, 1, rgb + (y_src + y) * w + x_src);
        }
    }
}

/* ===== LCD 渲染线程: 阻塞等帧队列 → pts 节流 → 推屏 → 归还缓冲 ===== */
static void pace_wait(uint32_t pts, uint32_t interval_ms);   /* 定义见下 */

static void lcd_task(void *arg)
{
    frame_msg_t msg;

    while (1) {
        if (xQueueReceive(s_frame_q, &msg, portMAX_DELAY) != pdTRUE) {
            continue;
        }
        pace_wait(msg.pts, msg.interval_ms);
        present_rgb(msg.buf, msg.w, msg.h);
        xQueueSend(s_pool_free, &msg.buf, portMAX_DELAY);

        /* 每 5s 报一次帧率 */
        s_pace.shown++;
        int64_t now = esp_timer_get_time();
        if (now - s_pace.fps_t0 >= 5000000) {
            ESP_LOGI(TAG, "fps=%.1f shown=%d heap=%u",
                     s_pace.shown * 1000000.0 / (now - s_pace.fps_t0), s_pace.shown,
                     (unsigned)esp_get_free_heap_size());
            s_pace.shown = 0;
            s_pace.fps_t0 = now;
        }
    }
}

/* ===== pts 定点 (毫秒); pts 不前进时退回固定帧间隔 (LCD 渲染线程调用) ===== */
static void pace_wait(uint32_t pts, uint32_t interval_ms)
{
    if (pts <= s_pace.last_pts) {
        pts = s_pace.last_pts + interval_ms;   /* pts 全 0 / 不连续时合成递增 */
    }
    s_pace.last_pts = pts;

    if (!s_pace.paced) {
        s_pace.t0_us = esp_timer_get_time();
        s_pace.pts0 = pts;
        s_pace.paced = true;
        return;
    }
    int64_t target_us = s_pace.t0_us + (int64_t)(pts - s_pace.pts0) * 1000;
    int64_t now_us = esp_timer_get_time();
    if (target_us > now_us + 1000) {
        vTaskDelay(pdMS_TO_TICKS((target_us - now_us) / 1000));
    }
}

/* ===== 按真实分辨率分配输出缓冲 + RGB565 帧缓冲池 ===== */
static bool alloc_out_bufs(play_ctx_t *c)
{
    /* 按 16 像素对齐留余量, 防解码器行距填充 */
    uint32_t aw = ((uint32_t)c->vid_w + 15) & ~15u;
    uint32_t ah = ((uint32_t)c->vid_h + 15) & ~15u;
    uint32_t img_size = esp_video_codec_get_image_size(c->out_fmt,
                        &(esp_video_codec_resolution_t){ .width = aw, .height = ah });
    if (img_size == 0) {
        img_size = aw * ah * 2;   /* 兜底按 RGB565 算 */
    }
    c->out_buf = esp_video_codec_align_alloc(c->out_align, img_size, &c->out_buf_size);
    if (c->out_buf == NULL) {
        ESP_LOGE(TAG, "out buf alloc %u failed", (unsigned)img_size);
        return false;
    }

    /* RGB565 帧缓冲池: 解码/转换写入 → 队列 → LCD 线程消费后归还 */
    size_t frame_size = (size_t)c->vid_w * c->vid_h * 2;
    for (int i = 0; i < FRAME_POOL_NUM; i++) {
        uint16_t *buf = heap_caps_malloc(frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (buf == NULL) {
            buf = malloc(frame_size);
        }
        if (buf == NULL) {
            ESP_LOGE(TAG, "frame buf %d alloc failed", i);
            return false;
        }
        xQueueSend(s_pool_free, &buf, 0);
        s_pool_n++;
    }
    return true;
}

/* 释放帧缓冲池: 阻塞等 LCD 线程把在途帧全部归还后再释放 */
static void free_frame_pool(void)
{
    uint16_t *buf;
    while (s_pool_n > 0) {
        if (xQueueReceive(s_pool_free, &buf, pdMS_TO_TICKS(2000)) != pdTRUE) {
            ESP_LOGW(TAG, "frame pool drain timeout, %d bufs lost", s_pool_n);
            return;   /* 防死等: 泄漏剩余缓冲, 不释放仍被引用的内存 */
        }
        free(buf);
        s_pool_n--;
    }
}

/* ===== 解码一帧并入帧队列; false=致命错误放弃本轮 ===== */
static bool decode_one(play_ctx_t *c, const esp_extractor_frame_info_t *frame)
{
    /* 分辨率探测: 首帧用 32B 小缓冲触发解码器解析头, 拿到真实分辨率后再分配大缓冲。
       缓冲不够时解码器已把帧解好并缓存, 同输入再 process 直接拷出 (不重复解码) */
    if (c->out_buf == NULL) {
        uint32_t tiny_size = 32;
        uint8_t *tiny = esp_video_codec_align_alloc(64, tiny_size, &tiny_size);
        if (tiny == NULL) {
            ESP_LOGE(TAG, "probe buf alloc failed");
            return false;
        }
        esp_video_dec_in_frame_t pin = {
            .pts  = frame->pts,
            .data = frame->frame_buffer,
            .size = frame->frame_size,
        };
        esp_video_dec_out_frame_t pout = { .data = tiny, .size = tiny_size };
        (void)esp_video_dec_process(c->dec, &pin, &pout);   /* BUF_NOT_ENOUGH 是预期 */
        esp_video_codec_free(tiny);

        esp_video_codec_frame_info_t fi = {0};
        if (esp_video_dec_get_frame_info(c->dec, &fi) != ESP_VC_ERR_OK ||
            fi.res.width == 0 || fi.res.height == 0) {
            /* 还没解析出分辨率 (如纯 SPS/PPS 帧), 等下一帧再试 */
            if (++c->probe_fail > MAX_PROBE_FAIL) {
                ESP_LOGE(TAG, "no resolution after %d frames", c->probe_fail);
                return false;
            }
            return true;
        }
        c->vid_w = fi.res.width;
        c->vid_h = fi.res.height;
        if (fi.fps > 0) {
            c->interval_ms = 1000 / fi.fps;
        }
        if (!alloc_out_bufs(c)) {
            return false;
        }
        ESP_LOGI(TAG, "decode %dx%d fps=%u fmt=%c%c%c%c convert=%d",
                 c->vid_w, c->vid_h, (unsigned)fi.fps,
                 (char)(c->out_fmt), (char)(c->out_fmt >> 8),
                 (char)(c->out_fmt >> 16), (char)(c->out_fmt >> 24),
                 c->need_convert);
        /* 继续往下: 同一输入再 process, H264 从内部缓存直接拷出 */
    }

    esp_video_dec_in_frame_t in = {
        .pts  = frame->pts,
        .data = frame->frame_buffer,
        .size = frame->frame_size,
    };
    esp_video_dec_out_frame_t out = {
        .data = c->out_buf,
        .size = c->out_buf_size,
    };
    esp_vc_err_t ret = esp_video_dec_process(c->dec, &in, &out);
    if (ret == ESP_VC_ERR_BUF_NOT_ENOUGH) {
        /* 输出缓冲不够: 扩容重试 (解码器缓存了已解帧, 重试即拷出) */
        uint32_t new_size = c->out_buf_size + c->out_buf_size / 2;
        uint8_t *p = esp_video_codec_align_alloc(c->out_align, new_size, &new_size);
        if (p == NULL) {
            ESP_LOGE(TAG, "realloc out buf %u failed", (unsigned)new_size);
            return true;   /* 丢帧继续 */
        }
        esp_video_codec_free(c->out_buf);
        c->out_buf = p;
        c->out_buf_size = new_size;
        out.data = c->out_buf;
        out.size = c->out_buf_size;
        ret = esp_video_dec_process(c->dec, &in, &out);
    }
    if (ret != ESP_VC_ERR_OK || out.decoded_size == 0) {
        return true;   /* 非图像 NAL (SPS/PPS 等) 或坏帧, 丢帧继续 */
    }

    /* 取空闲帧缓冲 (池满即背压: LCD 线程消费后归还) */
    uint16_t *fbuf;
    if (xQueueReceive(s_pool_free, &fbuf, pdMS_TO_TICKS(1000)) != pdTRUE) {
        return true;   /* LCD 长时间未消费, 丢帧继续 */
    }

    if (c->need_convert) {
        yuv420p_to_rgb565(c->out_buf, c->vid_w, c->vid_h, fbuf);
    } else {
        memcpy(fbuf, c->out_buf, (size_t)c->vid_w * c->vid_h * 2);
    }

    frame_msg_t msg = {
        .buf = fbuf,
        .w = c->vid_w,
        .h = c->vid_h,
        .pts = frame->pts,
        .interval_ms = c->interval_ms,
    };
    if (xQueueSend(s_frame_q, &msg, pdMS_TO_TICKS(1000)) != pdTRUE) {
        xQueueSend(s_pool_free, &fbuf, 0);   /* 归还, 丢帧继续 */
        return true;
    }
    return true;
}

/* ===== 播一整遍直到 EOS; true=正常播完 false=出错放弃 ===== */
static bool play_pass(play_ctx_t *c)
{
    int consec_err = 0;
    while (1) {
        esp_extractor_frame_info_t frame = {0};
        esp_extractor_err_t ret = esp_extractor_read_frame(c->extractor, &frame);
        if (ret == ESP_EXTRACTOR_ERR_EOS) {
            return true;
        }
        if (ret == ESP_EXTRACTOR_ERR_WAITING_OUTPUT || ret == ESP_EXTRACTOR_ERR_SKIPPED) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (ret != ESP_EXTRACTOR_ERR_OK) {
            if (++consec_err > MAX_CONSEC_ERR) {
                ESP_LOGE(TAG, "read_frame error %d (x%d), give up", ret, consec_err);
                return false;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        consec_err = 0;

        bool eos = EXTRACTOR_IS_EOS(frame.frame_flag);
        if (frame.frame_buffer && frame.frame_size &&
            frame.stream_type == ESP_EXTRACTOR_STREAM_TYPE_VIDEO) {
            if (!decode_one(c, &frame)) {
                (void)esp_extractor_release_frame(c->extractor, &frame);
                return false;
            }
        }
        (void)esp_extractor_release_frame(c->extractor, &frame);
        if (eos) {
            return true;   /* 最后一帧带 EOS 旗 */
        }
    }
}

/* ===== 打开文件 + 抽取器 + 解码器; 成功返回 true, 失败时 *err 给出具体原因 ===== */
static bool open_media(play_ctx_t *c, const char *path, const char **err)
{
    static bool registered;
    *err = "unknown";
    if (!registered) {
        if (esp_extractor_register_default() != ESP_EXTRACTOR_ERR_OK ||
            esp_video_dec_register_default() != ESP_VC_ERR_OK) {
            ESP_LOGE(TAG, "register extractor/decoder failed");
            *err = "codec register fail";
            return false;
        }
        registered = true;
    }

    c->fp = fopen(path, "rb");
    if (c->fp == NULL) {
        ESP_LOGE(TAG, "open %s failed (file missing? name must be test.mp4 in root)", path);
        *err = "file not found";
        return false;
    }
    c->io_cache = heap_caps_malloc(IO_CACHE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (c->io_cache == NULL) {
        c->io_cache = heap_caps_malloc(IO_CACHE_SIZE, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
    }
    if (c->io_cache) {
        setvbuf(c->fp, (char *)c->io_cache, _IOFBF, IO_CACHE_SIZE);
    }

    c->src = calloc(1, sizeof(*c->src));
    if (c->src == NULL) {
        *err = "no memory";
        goto fail;
    }
    c->src->fp = c->fp;
    c->src->io_cache = c->io_cache;

    esp_extractor_config_t ecfg = {
        .type          = ESP_EXTRACTOR_TYPE_MP4,
        .extract_mask  = ESP_EXTRACT_MASK_VIDEO,   /* 只要视频轨 */
        .in_read_cb    = file_read_cb,
        .in_seek_cb    = file_seek_cb,
        .in_size_cb    = file_size_cb,
        .in_ctx        = c->src,
        .out_pool_size = EXTRACT_POOL_SIZE,
        .out_align     = 64,                       /* 满足解码器输入对齐 */
    };
    if (esp_extractor_open(&ecfg, &c->extractor) != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "extractor open failed");
        *err = "extractor open fail";
        goto fail;
    }
    if (esp_extractor_parse_stream(c->extractor) != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "parse stream failed (not MP4/H264?)");
        *err = "not valid MP4";
        goto fail;
    }

    uint16_t vnum = 0;
    if (esp_extractor_get_stream_num(c->extractor, ESP_EXTRACTOR_STREAM_TYPE_VIDEO, &vnum) != ESP_EXTRACTOR_ERR_OK ||
        vnum == 0) {
        ESP_LOGE(TAG, "no video track");
        *err = "no video track";
        goto fail;
    }
    esp_extractor_stream_info_t sinfo = {0};
    if (esp_extractor_get_stream_info(c->extractor, ESP_EXTRACTOR_STREAM_TYPE_VIDEO, 0, &sinfo) != ESP_EXTRACTOR_ERR_OK) {
        ESP_LOGE(TAG, "get stream info failed");
        *err = "stream info fail";
        goto fail;
    }
    c->vid_w = sinfo.video_info.width;
    c->vid_h = sinfo.video_info.height;
    c->interval_ms = sinfo.video_info.fps ? (1000 / sinfo.video_info.fps) : (1000 / DEFAULT_FPS);
    ESP_LOGI(TAG, "stream %dx%d fps=%d dur=%ums",
             c->vid_w, c->vid_h, sinfo.video_info.fps, (unsigned)sinfo.duration);
    if (c->vid_w <= 0 || c->vid_h <= 0) {
        /* 容器没报宽高没关系: 首帧探测解码取真实分辨率 */
        ESP_LOGW(TAG, "container res unknown, will probe from first frame");
    }

    /* 选择解码输出格式: 能直出 RGB565_LE 最好, 否则 YUV420P 软件转 */
    esp_video_codec_type_t ctype;
    if (sinfo.video_info.format == ESP_EXTRACTOR_VIDEO_FORMAT_H264) {
        ctype = ESP_VIDEO_CODEC_TYPE_H264;
    } else if (sinfo.video_info.format == ESP_EXTRACTOR_VIDEO_FORMAT_MJPEG) {
        ctype = ESP_VIDEO_CODEC_TYPE_MJPEG;
    } else {
        static char fourcc_msg[24];
        uint32_t fcc = (uint32_t)sinfo.video_info.format;
        /* FOURCC 低字节在前, 转可打印标签 (如 hvc1/av01/mp4v) */
        char p[4];
        for (int i = 0; i < 4; i++) {
            char ch = (char)(fcc >> (8 * i));
            p[i] = (ch >= 0x20 && ch < 0x7f) ? ch : '?';
        }
        ESP_LOGE(TAG, "unsupported codec fourcc=0x%08x ('%c%c%c%c')",
                 (unsigned)fcc, p[0], p[1], p[2], p[3]);
        snprintf(fourcc_msg, sizeof(fourcc_msg), "%c%c%c%c not H264/MJPG", p[0], p[1], p[2], p[3]);
        *err = fourcc_msg;
        goto fail;
    }
    c->out_fmt = ESP_VIDEO_CODEC_PIXEL_FMT_YUV420P;
    c->need_convert = true;
    esp_video_codec_query_t query = { .codec_type = ctype, .codec_cc = 0 };
    esp_video_dec_caps_t caps;
    if (esp_video_dec_query_caps(&query, &caps) == ESP_VC_ERR_OK) {
        for (int i = 0; i < caps.out_fmt_num; i++) {
            if (caps.out_fmts[i] == ESP_VIDEO_CODEC_PIXEL_FMT_RGB565_LE) {
                c->out_fmt = ESP_VIDEO_CODEC_PIXEL_FMT_RGB565_LE;
                c->need_convert = false;
                break;
            }
        }
    }

    esp_video_dec_cfg_t dcfg = {
        .codec_type = ctype,
        .codec_cc   = 0,
        .out_fmt    = c->out_fmt,
    };
    if (esp_video_dec_open(&dcfg, &c->dec) != ESP_VC_ERR_OK) {
        ESP_LOGE(TAG, "decoder open failed");
        *err = "decoder open fail";
        goto fail;
    }

    uint8_t in_align = 64, out_align = 64;
    (void)esp_video_dec_get_frame_align(c->dec, &in_align, &out_align);
    c->out_align = out_align ? out_align : 64;

    /* 复位节流/探测状态 (输出缓冲等首帧探测出分辨率后再分配) */
    c->out_buf = NULL;
    c->out_buf_size = 0;
    c->probe_fail = 0;
    s_pace.last_pts = 0;
    s_pace.paced = false;
    s_pace.shown = 0;
    s_pace.fps_t0 = esp_timer_get_time();
    return true;

fail:
    /* 半开的资源由调用方 close_media() 统一收 */
    return false;
}

/* 释放本轮资源 (可重复调用) */
static void close_media(play_ctx_t *c)
{
    if (c->dec) {
        esp_video_dec_close(c->dec);
        c->dec = NULL;
    }
    if (c->extractor) {
        esp_extractor_close(c->extractor);
        c->extractor = NULL;
    }
    if (c->out_buf) {
        esp_video_codec_free(c->out_buf);
        c->out_buf = NULL;
    }
    free_frame_pool();          /* 等 LCD 线程归还在途帧后统一释放 */
    if (c->fp) {
        fclose(c->fp);
        c->fp = NULL;
    }
    if (c->io_cache) {
        heap_caps_free(c->io_cache);
        c->io_cache = NULL;
    }
    if (c->src) {
        free(c->src);
        c->src = NULL;
    }
}

void MP4Play_LoopForever(const char *path)
{
    play_ctx_t *c = calloc(1, sizeof(*c));
    if (c == NULL) {
        ESP_LOGE(TAG, "ctx alloc failed");
        vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        return;
    }

    /* 帧队列 + LCD 渲染线程 (只创建一次, 跨轮复用) */
    if (s_frame_q == NULL) {
        s_pool_free = xQueueCreate(FRAME_POOL_NUM, sizeof(uint16_t *));
        s_frame_q   = xQueueCreate(FRAME_POOL_NUM, sizeof(frame_msg_t));
        xTaskCreatePinnedToCore(lcd_task, "lcd_render", LCD_TASK_STACK, NULL, LCD_TASK_PRIO, NULL, 0);
        ESP_LOGI(TAG, "frame queue + lcd_render task up (pool=%d)", FRAME_POOL_NUM);
    }

    ESP_LOGI(TAG, "player start: %s", path);
    LCD_FillRect(0, 0, LCD_WIDTH, 24, LCD_BLACK);
    LCD_ShowString(4, 4, "MP4 player starting...", LCD_CYAN, LCD_BLACK, 2);

    while (1) {
        const char *err = "";
        if (open_media(c, path, &err)) {
            LCD_FillRect(0, 0, LCD_WIDTH, 24, LCD_BLACK);
            if (!play_pass(c)) {
                LCD_FillRect(0, 108, LCD_WIDTH, 40, LCD_BLACK);
                LCD_ShowString(4, 110, "decode fail:", LCD_RED, LCD_BLACK, 2);
                LCD_ShowString(4, 132, "retrying...", LCD_YELLOW, LCD_BLACK, 2);
            }
            close_media(c);
            ESP_LOGI(TAG, "loop: replay %s", path);
        } else {
            close_media(c);   /* 清掉半开的资源 */
            LCD_FillRect(0, 108, LCD_WIDTH, 40, LCD_BLACK);
            LCD_ShowString(4, 110, "MP4 open fail:", LCD_RED, LCD_BLACK, 2);
            LCD_ShowString(4, 132, err, LCD_YELLOW, LCD_BLACK, 2);
            ESP_LOGW(TAG, "open failed (%s), retry in %dms", err, RETRY_DELAY_MS);
            vTaskDelay(pdMS_TO_TICKS(RETRY_DELAY_MS));
        }
    }
}
