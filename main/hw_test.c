/**
 * @file hw_test.c
 * @brief 硬件模块逐项测试 main (ESP掌机 Board1)
 *
 * 一个模块一个模块测, 当前阶段: SD 卡视频 → LCD 显示。
 *   [已完成] LCD 测试画面确认正常 (白屏/彩条/灰阶/棋盘/文字)
 *   [当前]   SD 视频上屏: SD 挂载后找根目录第一个 *.mp4 循环播放
 *   [待测]   按键 / 摇杆 / 喇叭 / MP3 (代码在下方, 调用处已注释, 逐个打开)
 *
 * 片源要求 (PC 端 ffmpeg 预转换, H.264 baseline 320x240):
 *   ffmpeg -i 输入.mp4 -vf scale=320:240 -c:v libx264 -profile:v baseline \
 *          -level 3.0 -pix_fmt yuv420p -r 15 -g 30 -b:v 800k -an \
 *          -movflags +faststart /sd卡/xxx.mp4
 *
 * 引脚依据: E:\esp\MP3日志\SCH_Schematic1_2026-10-21.pdf (ESP掌机 Board1)
 * 用法:  build.bat build flash monitor        (串口 115200, tag: hw_test)
 */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <math.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "lcd.h"
#include "sdcard.h"
#include "button.h"
#include "rocker.h"
#include "max98357a.h"
#include "mp3play.h"
#include "mp4play.h"

/* 喇叭音量 (0-100), 修改此处调音量 */
#define SPK_TEST_VOLUME   5       /* 蜂鸣诊断音量 (方波很响, 别开大) */
#define MP3_PLAY_VOLUME   50      /* MP3 播放音量 (0-100) */

/* 交互测试窗口 */
#define BUTTON_WINDOW_MS  30000   /* 按键测试窗口 30s */
#define ROCKER_WINDOW_MS  15000   /* 摇杆测试窗口 15s */
#define ROCKER_SPAN_PASS  40      /* 每轴行程判定阈值 (满量程 ±100) */
#define MP3_SAMPLE_MS     8000    /* MP3 试听时长 */

static const char *TAG = "hw_test";

/* ===== 状态画面 (字库仅 ASCII) ===== */
static void lcd_status(const char *line1, const char *line2, const char *line3)
{
    LCD_Clear(LCD_BLACK);
    LCD_ShowString(8, 8,  line1, LCD_CYAN,  LCD_BLACK, 2);
    if (line2) {
        LCD_ShowString(8, 40, line2, LCD_WHITE, LCD_BLACK, 1);
    }
    if (line3) {
        LCD_ShowString(8, 56, line3, LCD_GRAY,  LCD_BLACK, 1);
    }
    LCD_Sync();
}

/* ===== SD 自检: 列目录 + 写/读回校验 (当前未调用, 想验 SD 写读时打开) ===== */
static bool __attribute__((unused)) sd_selftest(void)
{
    static const char *STAG = "sd_test";
    char path[64];
    bool ok = true;

    if (!SD_IsReady()) {
        SD_Init();
    }
    if (!SD_IsReady()) {
        ESP_LOGE(STAG, "mount failed (card? wiring?)");
        return false;
    }

    /* 1) 列出根目录 */
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (dir) {
        struct dirent *e;
        int n = 0;
        while ((e = readdir(dir)) != NULL && n < 20) {
            ESP_LOGI(STAG, "  %s %s", (e->d_type == DT_DIR) ? "[DIR ]" : "[FILE]", e->d_name);
            n++;
        }
        closedir(dir);
        ESP_LOGI(STAG, "root listing done (%d entries)", n);
    } else {
        ESP_LOGE(STAG, "opendir %s failed", SD_MOUNT_POINT);
        ok = false;
    }

    /* 2) 写测试文件 */
    snprintf(path, sizeof(path), "%s/test.txt", SD_MOUNT_POINT);
    FILE *f = fopen(path, "w");
    if (f) {
        fprintf(f, "hello game-machine sd test\n");
        fclose(f);
        ESP_LOGI(STAG, "wrote %s", path);
    } else {
        ESP_LOGE(STAG, "open %s for write failed", path);
        ok = false;
    }

    /* 3) 读回验证 */
    f = fopen(path, "r");
    if (f) {
        char line[64] = {0};
        if (fgets(line, sizeof(line), f)) {
            line[strcspn(line, "\r\n")] = 0;
            ESP_LOGI(STAG, "read back: \"%s\"", line);
            if (strcmp(line, "hello game-machine sd test") != 0) {
                ESP_LOGE(STAG, "readback mismatch!");
                ok = false;
            }
        } else {
            ESP_LOGE(STAG, "read %s failed", path);
            ok = false;
        }
        fclose(f);
    } else {
        ESP_LOGE(STAG, "open %s for read failed", path);
        ok = false;
    }
    return ok;
}

/* ===== 按键测试: 30s 依次按 10 键, 覆盖率判定 (当前未调用) ===== */
static const gpio_num_t s_key_gpio[KEY_NUM] = {
    KEY_GPIO_1,  KEY_GPIO_2,  KEY_GPIO_3,  KEY_GPIO_4,  KEY_GPIO_5,
    KEY_GPIO_6,  KEY_GPIO_7,  KEY_GPIO_8,  KEY_GPIO_9,  KEY_GPIO_10,
};

static volatile uint32_t s_key_seen = 0;          /* bit i = KEY_i+1 触发过 */
static volatile uint32_t s_key_ev_cnt[4] = {0};   /* none/short/long/double */

static void on_key_event(KeyId_t id, KeyEvent_t ev)
{
    static const char *evt_name[] = { "none", "short press", "long press", "double click" };
    if ((int)id >= KEY_NUM || (int)ev >= 4) {
        return;
    }
    s_key_seen |= 1u << id;
    s_key_ev_cnt[ev]++;
    ESP_LOGI("button", "KEY_%d (IO%d): %s", (int)id + 1, s_key_gpio[id], evt_name[ev]);
}

static void key_missing_str(char *out, size_t sz)
{
    size_t pos = 0;
    out[0] = 0;
    for (int i = 0; i < KEY_NUM; i++) {
        if (!(s_key_seen & (1u << i))) {
            int w = snprintf(out + pos, sz - pos, "%sK%d", pos ? "," : "", i + 1);
            if (w < 0 || (size_t)w >= sz - pos) {
                break;
            }
            pos += (size_t)w;
        }
    }
    if (!out[0]) {
        snprintf(out, sz, "-");
    }
}

static void __attribute__((unused)) test_button(void)
{
    char miss[64];
    const uint32_t all_mask = (1u << KEY_NUM) - 1;
    int elapsed = 0;

    Key_Init(on_key_event);
    s_key_seen = 0;
    memset((void *)s_key_ev_cnt, 0, sizeof(s_key_ev_cnt));
    ESP_LOGI(TAG, "请在 %ds 内依次按下 10 个按键 (短按即可, 全按完提前结束)", BUTTON_WINDOW_MS / 1000);
    lcd_status("TEST: BUTTON", "press all 10 keys", "see serial for progress");

    while (elapsed < BUTTON_WINDOW_MS) {
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;

        /* 电平巡检补漏: 没等到松开事件也算见过 */
        for (int i = 0; i < KEY_NUM; i++) {
            if (Key_IsPressed((KeyId_t)i)) {
                s_key_seen |= 1u << i;
            }
        }
        if ((s_key_seen & all_mask) == all_mask) {
            break;
        }
        if (elapsed % 5000 == 0) {
            key_missing_str(miss, sizeof(miss));
            ESP_LOGI(TAG, "progress: %u/10 keys, missing: %s (%ds/%ds)",
                     (unsigned)__builtin_popcount(s_key_seen), miss,
                     elapsed / 1000, BUTTON_WINDOW_MS / 1000);
        }
    }

    /* 常通键检查: 全部松开应为 0 */
    vTaskDelay(pdMS_TO_TICKS(300));
    int stuck = 0;
    for (int i = 0; i < KEY_NUM; i++) {
        if (Key_IsPressed((KeyId_t)i)) {
            stuck++;
        }
    }

    int seen_cnt = __builtin_popcount(s_key_seen);
    key_missing_str(miss, sizeof(miss));
    ESP_LOGI(TAG, "events: short=%u long=%u double=%u",
             (unsigned)s_key_ev_cnt[KEY_EVENT_SHORT_PRESS],
             (unsigned)s_key_ev_cnt[KEY_EVENT_LONG_PRESS],
             (unsigned)s_key_ev_cnt[KEY_EVENT_DOUBLE_CLICK]);
    if (seen_cnt == KEY_NUM) {
        ESP_LOGI(TAG, "BUTTON: %s (10/10 keys%s)", stuck ? "FAIL" : "PASS",
                 stuck ? ", BUT stuck key(s)!" : "");
    } else {
        ESP_LOGI(TAG, "BUTTON: FAIL (%d/10 keys, missing %s)", seen_cnt, miss);
    }
}

/* ===== 摇杆测试: 15s 两杆打满 4 轴, 行程判定 (当前未调用) ===== */
static void __attribute__((unused)) test_rocker(void)
{
    int rmin[4] = { 999, 999, 999, 999};   /* lx ly rx ry */
    int rmax[4] = {-999, -999, -999, -999};
    static const char *axis[4] = { "LX", "LY", "RX", "RY" };
    int elapsed = 0;

    rocker_init();
    ESP_LOGI(TAG, "请在 %ds 内把两个摇杆各方向打满 (4 轴行程>%d 均过提前结束)",
             ROCKER_WINDOW_MS / 1000, ROCKER_SPAN_PASS);
    lcd_status("TEST: ROCKER", "move both sticks fully", "all 4 axes");

    while (elapsed < ROCKER_WINDOW_MS) {
        rocker_value_t v;
        rocker_read(&v);
        int a[4] = { v.lx, v.ly, v.rx, v.ry };
        for (int i = 0; i < 4; i++) {
            if (a[i] < rmin[i]) rmin[i] = a[i];
            if (a[i] > rmax[i]) rmax[i] = a[i];
        }
        int ok_cnt = 0;
        for (int i = 0; i < 4; i++) {
            if (rmax[i] - rmin[i] >= ROCKER_SPAN_PASS) {
                ok_cnt++;
            }
        }
        if (ok_cnt == 4) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        elapsed += 100;
        if (elapsed % 5000 == 0) {
            ESP_LOGI(TAG, "progress: %d/4 axes ok  span LX=%d LY=%d RX=%d RY=%d",
                     ok_cnt, rmax[0] - rmin[0], rmax[1] - rmin[1],
                     rmax[2] - rmin[2], rmax[3] - rmin[3]);
        }
    }

    int pass = 0;
    for (int i = 0; i < 4; i++) {
        int span = rmax[i] - rmin[i];
        if (span >= ROCKER_SPAN_PASS) {
            pass++;
        }
        ESP_LOGI(TAG, "%s: min=%4d max=%4d span=%3d  %s", axis[i], rmin[i], rmax[i], span,
                 (span >= ROCKER_SPAN_PASS) ? "ok" : "TOO SMALL");
    }
    ESP_LOGI(TAG, "ROCKER: %s (%d/4 axes, span LX=%d LY=%d RX=%d RY=%d)",
             pass == 4 ? "PASS" : "FAIL", pass,
             rmax[0] - rmin[0], rmax[1] - rmin[1], rmax[2] - rmin[2], rmax[3] - rmin[3]);
}

/* ===== 喇叭诊断波形: 50Hz 方波嗡嗡 0.4s + 440Hz 正弦音调 0.5s (当前未调用) ===== */
#define BEEP_CHUNK_FRAMES 1024

static void spk_buzz(int sample_rate)
{
    static const char *STAG = "spk_test";
    static int16_t buf[BEEP_CHUNK_FRAMES * 2];
    int total = sample_rate * 400 / 1000;
    int sent = 0;

    while (sent < total) {
        int n = total - sent;
        if (n > BEEP_CHUNK_FRAMES) {
            n = BEEP_CHUNK_FRAMES;
        }
        for (int i = 0; i < n; i++) {
            int idx = sent + i;
            int16_t v = ((idx / (sample_rate / 100)) & 1) ? 30000 : -30000;  /* 每半周期翻转 */
            buf[i * 2]     = v;
            buf[i * 2 + 1] = v;
        }
        size_t wrote = MAX98357A_Write(buf, n);
        if (wrote < (size_t)n) {
            ESP_LOGE(STAG, "buzz write short: %u/%d", (unsigned)wrote, n);
            return;
        }
        sent += n;
    }
}

static void spk_tone(int sample_rate)
{
    static const char *STAG = "spk_test";
    static int16_t buf[BEEP_CHUNK_FRAMES * 2];
    const int fade_frames = sample_rate * 5 / 1000;
    const float dphi = 2.0f * (float)M_PI * 440.0f / (float)sample_rate;
    int total = sample_rate / 2;
    int sent = 0;
    float phase = 0.0f;

    while (sent < total) {
        int n = total - sent;
        if (n > BEEP_CHUNK_FRAMES) {
            n = BEEP_CHUNK_FRAMES;
        }
        for (int i = 0; i < n; i++) {
            float env = 1.0f;
            int idx = sent + i;
            if (idx < fade_frames) {
                env = (float)idx / (float)(fade_frames - 1);
            } else if (idx >= total - fade_frames) {
                env = (float)(total - 1 - idx) / (float)(fade_frames - 1);
            }
            int16_t v = (int16_t)(sinf(phase) * 12000.0f * env);
            buf[i * 2]     = v;
            buf[i * 2 + 1] = v;
            phase += dphi;
            if (phase >= 2.0f * (float)M_PI) {
                phase -= 2.0f * (float)M_PI;
            }
        }
        size_t wrote = MAX98357A_Write(buf, n);
        if (wrote < (size_t)n) {
            ESP_LOGE(STAG, "tone write short: %u/%d", (unsigned)wrote, n);
            return;
        }
        sent += n;
    }
}

static void __attribute__((unused)) test_spk(void)
{
    const int sample_rate = 44100;

    MAX98357A_Init(sample_rate);
    MAX98357A_SetVolume(SPK_TEST_VOLUME);
    ESP_LOGI(TAG, "diag volume=%d (方波很响, 别开大)", SPK_TEST_VOLUME);
    lcd_status("TEST: SPEAKER", "buzz + 440Hz tone x2", "listen!");

    for (int cycle = 1; cycle <= 2; cycle++) {
        spk_buzz(sample_rate);
        vTaskDelay(pdMS_TO_TICKS(200));
        spk_tone(sample_rate);
        ESP_LOGI(TAG, "diag cycle %d done (buzz+tone)", cycle);
        if (cycle < 2) {
            vTaskDelay(pdMS_TO_TICKS(1500));
        }
    }
    MAX98357A_WaitDrained(2000);
    ESP_LOGI(TAG, "SPEAKER: waves written ok, confirm by ear");
}

/* ===== MP3 试听: SD 根目录第一个 .mp3 播 8s 自动停 (当前未调用) ===== */
static void mp3_stop_after(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(MP3_SAMPLE_MS));
    ESP_LOGI(TAG, "mp3 sample time reached (%dms), stop", MP3_SAMPLE_MS);
    MP3Play_Stop();                      /* 跨任务停止请求 */
    vTaskDelete(NULL);
}

static void __attribute__((unused)) test_mp3(void)
{
    static const char *STAG = "mp3_test";
    char path[288];                      /* "/sd/" + FAT LFN 255 + NUL */
    char name[64] = {0};

    if (!SD_IsReady()) {
        ESP_LOGE(STAG, "SD not ready");
        return;
    }

    /* 找第一个 .mp3 (不区分大小写) */
    DIR *dir = opendir(SD_MOUNT_POINT);
    if (!dir) {
        ESP_LOGE(STAG, "opendir %s failed", SD_MOUNT_POINT);
        return;
    }
    struct dirent *e;
    while ((e = readdir(dir)) != NULL) {
        const char *n = e->d_name;
        size_t len = strlen(n);
        if (len < 4 || n[len - 4] != '.' ||
            (n[len - 3] != 'm' && n[len - 3] != 'M') ||
            (n[len - 2] != 'p' && n[len - 2] != 'P') || n[len - 1] != '3') {
            continue;
        }
        snprintf(name, sizeof(name), "%s", n);
        break;
    }
    closedir(dir);

    if (!name[0]) {
        ESP_LOGW(STAG, "no .mp3 in %s", SD_MOUNT_POINT);
        return;
    }

    snprintf(path, sizeof(path), "%s/%s", SD_MOUNT_POINT, name);
    ESP_LOGI(STAG, "play %s (max %ds, auto-stop)", path, MP3_SAMPLE_MS / 1000);
    lcd_status("TEST: MP3", name, "listen!");
    MAX98357A_Init(44100);
    MAX98357A_SetVolume(MP3_PLAY_VOLUME);
    xTaskCreate(mp3_stop_after, "mp3_stop", 2048, NULL, 4, NULL);
    bool ok = MP3Play_PlayFile(path);    /* 阻塞: 解码入队, 功放线程并行放音 */
    MAX98357A_WaitDrained(2000);
    ESP_LOGI(TAG, "MP3: %s (%s)", ok ? "played ok, confirm by ear" : "decode/write FAILED", name);
}

/* ===== 当前测试: SD 视频 → LCD ===== */
/* 判断文件名是否 .mp4 (不区分大小写) */
static bool is_mp4_name(const char *n)
{
    size_t len = strlen(n);
    return len >= 5 && n[len - 4] == '.' &&
           (n[len - 3] == 'm' || n[len - 3] == 'M') &&
           (n[len - 2] == 'p' || n[len - 2] == 'P') &&
           n[len - 1] == '4';
}

/* 在 dirpath 下找 .mp4; depth>0 时再进子目录找 (视频放子文件夹也能找到) */
static bool scan_mp4(const char *dirpath, char *path, size_t path_sz, char *name, size_t name_sz, int depth)
{
    DIR *dir = opendir(dirpath);
    if (!dir) {
        return false;
    }
    bool found = false;
    struct dirent *e;
    while (!found && (e = readdir(dir)) != NULL) {
        if (is_mp4_name(e->d_name)) {
            snprintf(path, path_sz, "%s/%s", dirpath, e->d_name);
            snprintf(name, name_sz, "%s", e->d_name);
            found = true;
        } else if (depth > 0 && e->d_type == DT_DIR) {
            char sub[520];
            snprintf(sub, sizeof(sub), "%s/%s", dirpath, e->d_name);
            found = scan_mp4(sub, path, path_sz, name, name_sz, depth - 1);
        }
    }
    closedir(dir);
    return found;
}

/* 打印目录树 (串口排障用: 看固件到底看到了什么文件) */
static void list_tree(const char *dirpath, int depth)
{
    DIR *dir = opendir(dirpath);
    if (!dir) {
        ESP_LOGW(TAG, "  (cannot open %s)", dirpath);
        return;
    }
    struct dirent *e;
    while ((e = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "  %*s%s %s", depth * 2, "", (e->d_type == DT_DIR) ? "[DIR ]" : "[FILE]", e->d_name);
        if (e->d_type == DT_DIR && depth < 2) {
            char sub[520];
            snprintf(sub, sizeof(sub), "%s/%s", dirpath, e->d_name);
            list_tree(sub, depth + 1);
        }
    }
    closedir(dir);
}

/* 视频任务: 挂 SD → 找视频 → 循环播 (永不返回); 三种状态各有明确提示 */
static void video_task(void *arg)
{
    (void)arg;
    char path[520];
    char name[64];

    while (1) {
        /* 状态 1: SD 挂载失败 (卡没插好 / 接线 / 不是 FAT32) */
        if (!SD_IsReady()) {
            SD_Init();
        }
        if (!SD_IsReady()) {
            ESP_LOGW(TAG, "SD mount FAILED (card? wiring? format must be FAT32), retry in 3s");
            lcd_status("SD CARD NOT READY", "insert card / format FAT32", "exFAT+NTFS not supported");
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        /* 状态 2: 挂载成功, 找到 .mp4 (根目录 + 两层子目录) */
        if (scan_mp4(SD_MOUNT_POINT, path, sizeof(path), name, sizeof(name), 2)) {
            ESP_LOGI(TAG, "play %s -> LCD (loop)", path);
            lcd_status("VIDEO PLAYING", name, "H.264 320x240 loop");
            MP4Play_LoopForever(path);   /* 阻塞, 永不返回 (文件打不开会 3s 自动重试) */
        }

        /* 状态 3: 挂载成功但没找到 .mp4 —— 打印目录树帮排障 */
        ESP_LOGW(TAG, "SD mounted but no .mp4 under %s, listing:", SD_MOUNT_POINT);
        list_tree(SD_MOUNT_POINT, 0);
        lcd_status("NO *.MP4 ON SD CARD", "copy video (xxx.mp4) to card", "see serial for file list");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "==== ESP掌机 HW TEST (module-by-module) ====");
    ESP_LOGI(TAG, "free heap = %u", (unsigned)esp_get_free_heap_size());
    ESP_LOGI(TAG, "current: SD video -> LCD  (LCD done, others commented)");
    ESP_LOGI(TAG, "pins: LCD MOSI=%d CLK=%d CS=%d DC=%d RST=%d BCKL=%d",
             LCD_GPIO_MOSI, LCD_GPIO_CLK, LCD_GPIO_CS, LCD_GPIO_DC, LCD_GPIO_RST, LCD_GPIO_BCKL);
    ESP_LOGI(TAG, "pins: SD  MOSI=%d MISO=%d CLK=%d CS=%d",
             SD_GPIO_MOSI, SD_GPIO_MISO, SD_GPIO_CLK, SD_GPIO_CS);

    /* LCD 只做初始化 (测试画面已确认正常, 不再跑) */
    LCD_Init();
    LCD_Clear(LCD_BLACK);
    LCD_Sync();
    vTaskDelay(pdMS_TO_TICKS(80));     /* 清屏发完再开背光, 防开机花屏 */
    LCD_SetBacklight(LCD_BRIGHTNESS);

    /* SD 挂载 (视频来源) */
    SD_Init();

    /* --- 逐项测试: 一个模块一个模块来, 之后的先注释, 测哪个打开哪个 --- */
    /* (void)sd_selftest();  */     /* [待测] SD 列目录/写读自检 */
    /* test_button();        */     /* [待测] 10 键 30s 覆盖率 */
    /* test_rocker();        */     /* [待测] 双摇杆 4 轴行程 */
    /* test_spk();           */     /* [待测] 喇叭蜂鸣/音调 */
    /* test_mp3();           */     /* [待测] MP3 试听 8s */

    /* [当前] SD 视频 → LCD: 找根目录第一个 *.mp4 循环播 */
    xTaskCreatePinnedToCore(video_task, "video", 16 * 1024, NULL, 5, NULL, 1);
}
