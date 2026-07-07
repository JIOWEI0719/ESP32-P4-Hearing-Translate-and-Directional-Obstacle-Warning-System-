#pragma once
#include "esp_err.h"
#include "esp_codec_dev.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the audio player task.
 *
 * Must be called after bsp_audio_codec_speaker_init() and audio_sr_init().
 *
 * @param play_handle  OUT-only codec handle from bsp_audio_codec_speaker_init().
 * @return ESP_OK on success.
 */
esp_err_t music_player_audio_init(esp_codec_dev_handle_t play_handle);

/**
 * @brief 播放单个 MP3 文件（完整路径）。
 *
 * 内部调用 bsp_extra_player_play_file()，由已有的 audio_player 实例处理。
 *
 * @param file_path  完整 SPIFFS 路径，如 "/spiffs/help.mp3"
 * @return ESP_OK 成功
 */
esp_err_t music_player_play_file(const char *file_path);

/**
 * @brief 按文件名播放单个 MP3（自动拼接 SPIFFS 挂载路径）。
 *
 * 等价于 music_player_play_file("/spiffs/" filename)。
 *
 * @param filename  文件名，如 "help.mp3"
 * @return ESP_OK 成功
 */
esp_err_t music_player_play_by_name(const char *filename);

/**
 * @brief 停止当前播放。
 */
void music_player_stop(void);

/**
 * @brief 查询播放器当前是否正在播放音频。
 *
 * @return true 正在播放
 */
bool music_player_is_playing(void);

#ifdef __cplusplus
}
#endif
