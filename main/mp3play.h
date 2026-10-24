/**
 * @file mp3play.h
 * @brief MP3 文件播放 (解码 → I2S 喇叭) (可移植模块)
 *
 * 使用方法：
 *   1. 先初始化喇叭: MAX98357A_Init(44100); MAX98357A_SetVolume(50);
 *   2. MP3Play_PlayFile("/sd/xxx.mp3");   // 阻塞播放整个文件, 结束返回
 *   3. 其他任务可随时 MP3Play_Stop();     // 请求停止当前播放
 *
 * 内部用 esp_audio_codec (乐鑫官方组件 espressif/esp_audio_codec) 流式解码,
 * 自动跳过 ID3v2 标签, 单声道自动扩立体声, 采样率按码流热切换。
 * 需要: max98357a 模块, CMakeLists PRIV_REQUIRES 加 espressif__esp_audio_codec
 */
#ifndef __MP3PLAY_H__
#define __MP3PLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

/* ===== 硬件配置（移植时修改此处） ===== */
/* (无引脚; 喇叭引脚在 max98357a.h, 文件系统挂载点在 sdcard.h) */

/* ===== API ===== */
bool MP3Play_PlayFile(const char *path);  /* 阻塞播放整个 MP3 文件; 成功播完返回 true */
void MP3Play_Stop(void);                  /* 请求停止当前播放 (可在其他任务调用) */

#ifdef __cplusplus
}
#endif

#endif /* __MP3PLAY_H__ */
