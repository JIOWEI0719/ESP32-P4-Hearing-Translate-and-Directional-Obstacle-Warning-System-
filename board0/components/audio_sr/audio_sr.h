#pragma once
#include "esp_err.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ====== Queue message types (shared between audio_sr and ui_speech) ====== */

typedef struct {
    int cmd_id;        /* Speech command ID (1-5), or -1 for wake word only */
    int doa_angle;     /* DOA angle 0-180, or -1 if unavailable */
} sr_event_t;

typedef struct {
    char label[32];    /* "door_bell" or "honk" */
    float probability; /* Detection probability */
    int doa_angle;     /* DOA angle 0-180, or -1 if unavailable */
} sr_ei_event_t;

/* Queue handles — created by audio_sr_init(), read by ui_speech */
extern QueueHandle_t g_sr_queue;
extern QueueHandle_t g_ei_queue;

/**
 * @brief Initialize audio speech recognition (AFE + MultiNet + DOA + EI).
 *
 * Sets up I2S, I2C, ES8311 codec, loads WakeNet/MultiNet models,
 * starts feed+detect task and environment sound classification task,
 * and creates FreeRTOS queues for UI event delivery.
 *
 * @param codec_already_init  Set to true if bsp_extra_codec_init() has
 *                            already been called (I2S_NUM_0 + ES8311 + I2C
 *                            already initialized by BSP). When true, the
 *                            I2S_NUM_0 bus init and codec init are skipped.
 *                            I2S_NUM_1 (dual INMP441 mic array) is always
 *                            initialized regardless of this parameter.
 *
 * @return ESP_OK on success.
 */
esp_err_t audio_sr_init(bool codec_already_init);

/**
 * @brief Get the ES8311 codec device handle for external audio output.
 *
 * @return esp_codec_dev_handle_t (may be NULL if not initialized).
 */
esp_codec_dev_handle_t audio_sr_get_codec_dev(void);

/**
 * @brief Get the I2S_NUM_0 TX channel handle for direct I2S output testing.
 *
 * @return i2s_chan_handle_t (may be NULL if not initialized).
 */
i2s_chan_handle_t audio_sr_get_tx_handle(void);

/**
 * @brief Get the I2C bus handle used for ES8311 control.
 */
i2c_master_bus_handle_t audio_sr_get_i2c_bus(void);

/**
 * @brief Suspend feed_task and detect_task (for sign language mode).
 *
 * Stops CPU-intensive AFE/ WakeNet / MultiNet processing to prevent
 * watchdog starvation. I2S peripherals keep running.
 */
void audio_sr_suspend(void);

/**
 * @brief Resume feed_task and detect_task (for voice recognition mode).
 */
void audio_sr_resume(void);

#ifdef __cplusplus
}
#endif
