#include "music_player.h"
#include "audio_player.h"
#include "bsp_board_extra.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "music_player";

/* BSP OUT-only codec handle.  Independent of audio_sr — ESP-SR cannot touch this. */
static esp_codec_dev_handle_t s_play_handle = NULL;
static int s_volume = 100;

/* ====== Audio player hardware callbacks ====== */

static esp_err_t mp_mute_fn(AUDIO_PLAYER_MUTE_SETTING setting)
{
    if (!s_play_handle) return ESP_FAIL;
    bool mute = (setting == AUDIO_PLAYER_MUTE);
    int64_t t0 = esp_timer_get_time();
    esp_err_t ret = esp_codec_dev_set_out_mute(s_play_handle, mute);
    int64_t t1 = esp_timer_get_time();
    ESP_LOGI(TAG, "[MUTE] %s → %s (%lld us)",
             mute ? "MUTE" : "UNMUTE", ret == ESP_OK ? "OK" : esp_err_to_name(ret), t1 - t0);
    if (!mute) {
        ret = esp_codec_dev_set_out_vol(s_play_handle, s_volume);
        ESP_LOGI(TAG, "[MUTE] set_vol(%d) → %s", s_volume, ret == ESP_OK ? "OK" : esp_err_to_name(ret));
    } else {
        vTaskDelay(pdMS_TO_TICKS(50));  /* I2C settling after MUTE */
    }
    return ret;
}

static esp_err_t mp_write_fn(void *audio_buffer, size_t len, size_t *bytes_written, uint32_t timeout_ms)
{
    if (!s_play_handle) return ESP_FAIL;
    esp_err_t ret = esp_codec_dev_write(s_play_handle, audio_buffer, len);
    *bytes_written = len;
    return ret;
}

static esp_err_t mp_clk_set_fn(uint32_t rate, uint32_t bits_cfg, i2s_slot_mode_t ch)
{
    ESP_LOGI(TAG, "[CLK] rate=%lu bits=%lu ch=%d (no-op)", rate, bits_cfg, ch);
    return ESP_OK;
}

/* ====== Init ====== */

esp_err_t music_player_audio_init(esp_codec_dev_handle_t play_handle)
{
    if (!play_handle) return ESP_ERR_INVALID_ARG;
    s_play_handle = play_handle;

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel         = 2,
        .sample_rate     = 16000,
    };
    esp_err_t ret = esp_codec_dev_open(s_play_handle, &fs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_codec_dev_open: %s (0x%x)", esp_err_to_name(ret), ret);
        return ret;
    }
    ESP_LOGI(TAG, "Codec opened: %luHz/%lubit/%luch",
             fs.sample_rate, fs.bits_per_sample, fs.channel);

    audio_player_config_t config = {
        .mute_fn     = mp_mute_fn,
        .write_fn    = mp_write_fn,
        .clk_set_fn  = mp_clk_set_fn,
        .priority    = 5,
    };
    ret = audio_player_new(config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "audio_player_new: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Initialized (prio=%d)", config.priority);
    return ESP_OK;
}

/* ====== Single-file playback ====== */

esp_err_t music_player_play_file(const char *file_path)
{
    if (!file_path) return ESP_ERR_INVALID_ARG;

    esp_err_t ret = bsp_extra_player_play_file(file_path);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "play_file '%s': %s", file_path, esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Playing: %s", file_path);
    return ESP_OK;
}

esp_err_t music_player_play_by_name(const char *filename)
{
    if (!filename) return ESP_ERR_INVALID_ARG;

    char full_path[128];
    snprintf(full_path, sizeof(full_path), "%s/%s",
             CONFIG_BSP_SPIFFS_MOUNT_POINT, filename);

    return music_player_play_file(full_path);
}

void music_player_stop(void)
{
    audio_player_stop();
    ESP_LOGI(TAG, "Stopped");
}

bool music_player_is_playing(void)
{
    return (audio_player_get_state() == AUDIO_PLAYER_STATE_PLAYING);
}
