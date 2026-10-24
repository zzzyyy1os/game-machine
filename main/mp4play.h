/**
 * @file mp4play.h
 * @brief SD 卡 MP4 视频播放 (esp_extractor + esp_video_codec 精简解码管线)
 *
 * 只要画面不要声音 (音频轨由 extract_mask 过滤, 不解码)。
 * 移植自 esp-vedio (ESP-GMF) packages/esp_video_render/examples/video_player,
 * 去掉 GMF 管线/渲染/同步, 双线程流水: 解封装→解码→帧队列→LCD 推屏。
 *
 * 线程: 调用方线程做解码生产; MP4Play_LoopForever 内部起 lcd_render
 *       消费线程 (pts 节流 + 推屏)。无轮询, 两端都阻塞在队列上。
 *
 * 使用方法:
 *   1. SD 已挂载 (SD_Init), LCD 已初始化 (LCD_Init)
 *   2. xTaskCreatePinnedToCore(task, "mp4play", 12*1024, NULL, 5, NULL, 1);
 *      task 内调用 MP4Play_LoopForever("/sd/test.mp4");   // 阻塞, 永不返回
 *
 * 片源要求 (PC 端 ffmpeg 预转换, H.264 baseline 320x240):
 *   ffmpeg -i 输入.mp4 -vf scale=320:240 -c:v libx264 -profile:v baseline \
 *          -level 3.0 -pix_fmt yuv420p -r 15 -g 30 -b:v 800k -an \
 *          -movflags +faststart /sd卡/test.mp4
 *
 * 需要 ESP-IDF 组件: esp_extractor, esp_video_codec (idf_component.yml 自动下载)
 */
#ifndef __MP4PLAY_H__
#define __MP4PLAY_H__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 循环播放 MP4 文件 (视频轨), 播到结尾自动从头再来
 *
 * 阻塞函数, 永不返回。文件打不开时在 LCD 显示错误并每 3s 重试
 * (方便热插 SD 卡后自动恢复)。
 *
 * @param path  文件路径, 如 "/sd/test.mp4"
 */
void MP4Play_LoopForever(const char *path);

#ifdef __cplusplus
}
#endif

#endif /* __MP4PLAY_H__ */
