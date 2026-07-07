#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "audio_sr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

/* ESP-SR headers */
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_afe_config.h"
#include "model_path.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"

/* Edge Impulse environment sound classification */
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"

static const char *TAG = "audio_sr";

/* ====== GPIO macros (direct definition, no BSP dependency) ====== */
/* Fix 1: explicit (gpio_num_t) casts for C++ */
#define I2C_SDA_IO       ((gpio_num_t)7)
#define I2C_SCL_IO       ((gpio_num_t)8)
#define I2S_MCLK_IO      ((gpio_num_t)13)
#define I2S_BCLK_IO      ((gpio_num_t)12)
#define I2S_WS_IO        ((gpio_num_t)10)
#define I2S_DOUT_IO      ((gpio_num_t)9)
#define I2S_DIN_IO       ((gpio_num_t)11)
#define PA_CTRL_IO       ((gpio_num_t)53)

/* External dual INMP441 mic array pins */
#define I2S_MIC_SCK_IO   ((gpio_num_t)2)
#define I2S_MIC_WS_IO    ((gpio_num_t)3)
#define I2S_MIC_SD_IO    ((gpio_num_t)4)

#define SAMPLE_RATE      16000

/* ====== DOA parameters ====== */
#define MIC_DISTANCE_M       0.035f
#define SOUND_SPEED_M_S      343.0f
#define DOA_PI               3.14159265358979f
#define DOA_FRAME_SAMPLES    1024
#define DOA_BUF_SIZE         (DOA_FRAME_SAMPLES * 2)
#define MAX_TDOA_SAMPLES     ((int)((MIC_DISTANCE_M / SOUND_SPEED_M_S) * SAMPLE_RATE) + 2)

/* ====== Global handles ====== */
static i2s_chan_handle_t       tx_handle      = NULL;
static i2s_chan_handle_t       rx_handle      = NULL;
static i2s_chan_handle_t       rx_mic_handle  = NULL;
static esp_codec_dev_handle_t  codec_dev      = NULL;
static i2c_master_bus_handle_t s_i2c_bus      = NULL;

/* AFE / MultiNet */
static const esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t  *afe_data   = NULL;
static const esp_mn_iface_t *multinet = NULL;
static model_iface_data_t   *mn_data  = NULL;

/* State */
static volatile bool is_woken_up = false;

/* Edge Impulse environment sound classification buffers */
static float ei_audio_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static float ei_classify_snapshot[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];
static int ei_buffer_idx = 0;
static SemaphoreHandle_t ei_infer_sem = NULL;

/* FreeRTOS queues for UI events (extern "C" to match header linkage) */
extern "C" {
    QueueHandle_t g_sr_queue = NULL;
    QueueHandle_t g_ei_queue = NULL;
}

/* DOA circular buffers */
static int16_t *doa_left_buf  = NULL;
static int16_t *doa_right_buf = NULL;
static volatile int doa_write_idx = 0;
static SemaphoreHandle_t doa_mutex = NULL;

/* Task handles — for external suspend/resume (mode switching) */
static TaskHandle_t s_feed_task_handle   = NULL;
static TaskHandle_t s_detect_task_handle = NULL;

/* ====== 1. I2S bus init (I2S_NUM_0 for ES8311 codec) ====== */
static void i2s_bus_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_MCLK_IO, .bclk = I2S_BCLK_IO, .ws = I2S_WS_IO,
            .dout = I2S_DOUT_IO, .din  = I2S_DIN_IO, .invert_flags = {0},
        },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
}

/* ====== 2. Dual mic I2S init (I2S_NUM_1 for external INMP441 array) ====== */
static void dual_mic_i2s_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &rx_mic_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_MIC_SCK_IO,
            .ws   = I2S_MIC_WS_IO,
            .dout = I2S_GPIO_UNUSED,
            .din  = I2S_MIC_SD_IO,
            .invert_flags = {0},
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_mic_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_mic_handle));
    ESP_LOGI(TAG, "Dual mic I2S (I2S_NUM_1) initialized");
}

/* ====== 3. I2C bus init ====== */
static i2c_master_bus_handle_t i2c_bus_init(void)
{
    /* Fix 2: zero-init then field-by-field to avoid C++ designated-initializer issues */
    i2c_master_bus_config_t cfg = {};
    cfg.clk_source                   = I2C_CLK_SRC_DEFAULT;
    cfg.i2c_port                     = I2C_NUM_0;
    cfg.scl_io_num                   = I2C_SCL_IO;
    cfg.sda_io_num                   = I2C_SDA_IO;
    cfg.glitch_ignore_cnt            = 7;
    cfg.flags.enable_internal_pullup = true;

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    return bus;
}

/* ====== 4. ES8311 codec init ====== */
static void codec_init(i2c_master_bus_handle_t i2c_bus)
{
    audio_codec_i2s_cfg_t i2s_cfg = {
        .port = I2S_NUM_0, .rx_handle = rx_handle, .tx_handle = tx_handle
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .addr = ES8311_CODEC_DEFAULT_ADDR, .bus_handle = i2c_bus
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    /* Fix 3: zero-init then field-by-field assignment */
    es8311_codec_cfg_t es_cfg = {};
    es_cfg.codec_mode                = ESP_CODEC_DEV_WORK_MODE_BOTH;
    es_cfg.ctrl_if                   = ctrl_if;
    es_cfg.gpio_if                   = gpio_if;
    es_cfg.pa_pin                    = PA_CTRL_IO;
    es_cfg.use_mclk                  = true;
    es_cfg.hw_gain.pa_voltage        = 5.0f;
    es_cfg.hw_gain.codec_dac_voltage = 3.3f;
    const audio_codec_if_t *codec_if = es8311_codec_new(&es_cfg);

    esp_codec_dev_cfg_t dev_cfg = {};
    dev_cfg.dev_type  = ESP_CODEC_DEV_TYPE_IN_OUT;
    dev_cfg.codec_if  = codec_if;
    dev_cfg.data_if   = data_if;
    codec_dev = esp_codec_dev_new(&dev_cfg);

    esp_codec_dev_sample_info_t fs = {};
    fs.bits_per_sample = 16;
    fs.channel         = 2;
    fs.sample_rate     = SAMPLE_RATE;
    esp_codec_dev_open(codec_dev, &fs);
    esp_codec_dev_set_in_gain(codec_dev, 24.0f);
}

/* ====== 5. Custom command injection ====== */
static void inject_custom_commands(void)
{
    ESP_LOGI(TAG, "Injecting custom speech commands");
    esp_mn_commands_clear();
    esp_mn_commands_add(1, "ni hao");
    esp_mn_commands_add(2, "qing rang yi xia");
    esp_mn_commands_add(3, "zhu yi an quan");
    esp_mn_commands_add(3, "xiao xin yi dian");
    esp_mn_commands_add(4, "wo lai bang ni");
    esp_mn_commands_add(5, "xie xie");
    esp_mn_commands_update();
}

/* ====== 6. DOA cross-correlation TDOA ====== */
static int compute_doa_angle(void)
{
    if (doa_left_buf == NULL || doa_right_buf == NULL || doa_mutex == NULL) {
        return -1;
    }

    static int16_t snap_left[DOA_FRAME_SAMPLES];
    static int16_t snap_right[DOA_FRAME_SAMPLES];

    if (xSemaphoreTake(doa_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
        return -1;
    }
    int snap_pos = doa_write_idx;
    for (int i = 0; i < DOA_FRAME_SAMPLES; i++) {
        int idx = (snap_pos - DOA_FRAME_SAMPLES + i + DOA_BUF_SIZE) % DOA_BUF_SIZE;
        snap_left[i]  = doa_left_buf[idx];
        snap_right[i] = doa_right_buf[idx];
    }
    xSemaphoreGive(doa_mutex);

    /* Energy check */
    int64_t energy = 0;
    for (int i = 0; i < DOA_FRAME_SAMPLES; i++) {
        energy += (int64_t)snap_left[i]  * snap_left[i];
        energy += (int64_t)snap_right[i] * snap_right[i];
    }
    if (energy < 1000000LL) return -1;

    /* Cross-correlation peak search */
    int best_lag = 0;
    int64_t max_corr = INT64_MIN;
    for (int lag = -MAX_TDOA_SAMPLES; lag <= MAX_TDOA_SAMPLES; lag++) {
        int64_t corr = 0;
        int start = (lag > 0) ? lag : 0;
        int end   = (lag < 0) ? DOA_FRAME_SAMPLES + lag : DOA_FRAME_SAMPLES;
        for (int i = start; i < end; i++) {
            corr += (int64_t)snap_left[i] * (int64_t)snap_right[i - lag];
        }
        if (corr > max_corr) {
            max_corr = corr;
            best_lag = lag;
        }
    }

    /* Convert lag to angle */
    float ratio = (float)best_lag * SOUND_SPEED_M_S / (MIC_DISTANCE_M * (float)SAMPLE_RATE);
    if (ratio >  1.0f) ratio =  1.0f;
    if (ratio < -1.0f) ratio = -1.0f;
    float theta_rad = asinf(ratio);
    int angle_deg = 90 - (int)(theta_rad * 180.0f / DOA_PI);

    ESP_LOGI(TAG, "DOA: lag=%d samples, angle=%d deg", best_lag, angle_deg);
    return angle_deg;
}

/* ====== 7. Edge Impulse callback + environment sound AI task ====== */

static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    memcpy(out_ptr, ei_classify_snapshot + offset, length * sizeof(float));
    return 0;
}

static void env_sound_AI_Task(void *arg)
{
    ESP_LOGI(TAG, "Environment sound monitor started (door_bell, honk)");

    signal_t features;
    features.total_length = EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE;
    features.get_data = &microphone_audio_signal_get_data;

    while (1) {
        xSemaphoreTake(ei_infer_sem, portMAX_DELAY);

        /* Checkpoint 1: if woken up during wait, discard */
        if (is_woken_up) continue;

        ei_impulse_result_t result = { 0 };
        EI_IMPULSE_ERROR res = run_classifier(&features, &result, false);

        if (res != EI_IMPULSE_OK) continue;

        /* Checkpoint 2: if woken up during inference, discard */
        if (is_woken_up) continue;

        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
            if ((strcmp(result.classification[ix].label, "door_bell") == 0 ||
                 strcmp(result.classification[ix].label, "honk") == 0) &&
                result.classification[ix].value > 0.95) {

                int doa_angle = compute_doa_angle();

                ESP_LOGE(TAG, "========================================");
                ESP_LOGE(TAG, "  ALERT: [%s] detected! Prob: %.2f",
                         result.classification[ix].label, result.classification[ix].value);
                if (doa_angle >= 0) {
                    ESP_LOGE(TAG, "  DOA: sound from %d deg", doa_angle);
                }
                ESP_LOGE(TAG, "========================================");

                /* Send EI event to UI queue */
                sr_ei_event_t ei_evt;
                strncpy(ei_evt.label, result.classification[ix].label, sizeof(ei_evt.label) - 1);
                ei_evt.label[sizeof(ei_evt.label) - 1] = '\0';
                ei_evt.probability = result.classification[ix].value;
                ei_evt.doa_angle = doa_angle;
                xQueueSend(g_ei_queue, &ei_evt, 0);

                vTaskDelay(pdMS_TO_TICKS(2000));
            }
        }
    }
}

/* ====== 8a. Feed task: I2S read → AFE feed (runs on one core, never blocks on fetch) ====== */
static void feed_task(void *arg)
{
    int audio_chunksize = afe_handle->get_feed_chunksize(afe_data);
    int feed_channel    = afe_handle->get_feed_channel_num(afe_data);
    int chunk_bytes     = audio_chunksize * feed_channel * (int)sizeof(int16_t);
    size_t read_size    = (size_t)audio_chunksize * feed_channel * sizeof(int32_t);

    int32_t *i2s_read_buff = (int32_t *)heap_caps_aligned_alloc(
        64, audio_chunksize * feed_channel * sizeof(int32_t), MALLOC_CAP_SPIRAM);
    int16_t *audio_buf     = (int16_t *)heap_caps_aligned_alloc(
        64, chunk_bytes, MALLOC_CAP_SPIRAM);

    if (!i2s_read_buff || !audio_buf) {
        ESP_LOGE(TAG, "feed_task: failed to allocate audio buffers");
        if (i2s_read_buff) free(i2s_read_buff);
        if (audio_buf) free(audio_buf);
        vTaskDelete(NULL);
        return;
    }

    size_t bytes_read;
    bool last_woken_state = false;
    unsigned loop_count = 0;

    ESP_LOGI(TAG, "Feed task started. Waiting for wake word 'Hi Lexin'...");

    while (1) {

        loop_count++;
        if (loop_count % 100 == 0) {
            ESP_LOGI(TAG, "[FEED] loop=%u alive", loop_count);
        }

        /* 1. Read 32-bit stereo raw data from dual mic I2S
         *    (portMAX_DELAY blocks ~32ms/帧 → IDLE 自然获 CPU, 无需额外 yield) */
        i2s_channel_read(rx_mic_handle, i2s_read_buff,
                         read_size, &bytes_read, portMAX_DELAY);

        /* 2. 32→16 bit conversion */
        for (int i = 0; i < audio_chunksize * feed_channel; i++) {
            audio_buf[i] = (int16_t)(i2s_read_buff[i] >> 14);
        }

        /* 3. Feed left channel to Edge Impulse buffer for environment classification */
        if (is_woken_up) {
            ei_buffer_idx = 0;
        } else {
            for (int i = 0; i < audio_chunksize; i++) {
                ei_audio_buffer[ei_buffer_idx] = (float)audio_buf[i * feed_channel];
                ei_buffer_idx++;
                if (ei_buffer_idx >= EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) {
                    memcpy(ei_classify_snapshot, ei_audio_buffer, sizeof(ei_audio_buffer));
                    ei_buffer_idx = 0;
                    if (ei_infer_sem != NULL) {
                        xSemaphoreGive(ei_infer_sem);
                    }
                }
            }
        }

        /* 4. Store both channels in DOA circular buffer */
        if (feed_channel >= 2 && doa_left_buf && doa_right_buf) {
            xSemaphoreTake(doa_mutex, portMAX_DELAY);
            int w = doa_write_idx;
            for (int i = 0; i < audio_chunksize; i++) {
                doa_left_buf[w]  = audio_buf[i * feed_channel];
                doa_right_buf[w] = audio_buf[i * feed_channel + 1];
                w = (w + 1) % DOA_BUF_SIZE;
            }
            doa_write_idx = w;
            xSemaphoreGive(doa_mutex);
        }

        /* 5. Log state transitions */
        if (is_woken_up != last_woken_state) {
            if (is_woken_up) {
                ESP_LOGI(TAG, "Entering command listening mode");
            } else {
                ESP_LOGI(TAG, "Returning to wake-word detection mode");
            }
            last_woken_state = is_woken_up;
        }

        /* 6. Feed AFE (non-blocking, never waits on fetch) */
        afe_handle->feed(afe_data, audio_buf);
    }
}

/* ====== 8b. Detect task: continuous fetch → wake word / command (runs on other core) ====== */
static void detect_task(void *arg)
{
    ESP_LOGI(TAG, "Detect task started. Listening for wake word...");
    unsigned loop_count = 0;
    unsigned fail_count = 0;

    while (1) {
        loop_count++;

        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            /* 空转时每 ~50 次休眠 1 tick 让 IDLE 喂狗，其余用 taskYIELD 保持响应 */
            fail_count++;
            if (fail_count % 50 == 0) {
                vTaskDelay(1);
            } else {
                taskYIELD();
            }
            continue;
        }
        fail_count = 0;  /* 有结果时重置计数 */

        if (!is_woken_up) {
            /* WakeNet: detect wake word */
            if (res->wakeup_state == WAKENET_DETECTED) {

                ESP_LOGW(TAG, "Wake word detected! Listening for command (6s timeout)...");

                sr_event_t sr_evt;
                sr_evt.cmd_id = -1;
                sr_evt.doa_angle = -1;
                xQueueSend(g_sr_queue, &sr_evt, 0);

                is_woken_up = true;
            }
        } else {
            /* MultiNet: detect command */
            esp_mn_state_t mn_state = multinet->detect(mn_data, res->data);
            if (mn_state == ESP_MN_STATE_DETECTED) {
                esp_mn_results_t *mn_result = multinet->get_results(mn_data);
                int cmd_id = mn_result->command_id[0];
                ESP_LOGE(TAG, "Command detected! ID: %d", cmd_id);
                ESP_LOGW(TAG, "========================================");

                sr_event_t sr_evt;
                sr_evt.cmd_id = cmd_id;
                sr_evt.doa_angle = -1;
                xQueueSend(g_sr_queue, &sr_evt, 0);

                is_woken_up = false;
            } else if (mn_state == ESP_MN_STATE_TIMEOUT) {
                ESP_LOGW(TAG, "Command listen timeout");

                /* Notify UI to restore idle state */
                sr_event_t sr_evt;
                sr_evt.cmd_id = -2;  /* -2 = timeout, return to idle */
                sr_evt.doa_angle = -1;
                xQueueSend(g_sr_queue, &sr_evt, 0);

                is_woken_up = false;
            }
        }
    }
}

/* ====== 9. Public init function ====== */
esp_err_t audio_sr_init(bool codec_already_init)
{
    ESP_LOGI(TAG, "Initializing audio speech recognition...");
    ESP_LOGI(TAG, "  [DIAG] free internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Hardware init */
    if (!codec_already_init) {
        i2s_bus_init();
        s_i2c_bus = i2c_bus_init();
        codec_init(s_i2c_bus);
    } else {
        ESP_LOGI(TAG, "Codec already initialized by bsp_extra, skipping I2S_NUM_0 + I2C + ES8311 init");
    }
    dual_mic_i2s_init();
    ESP_LOGI(TAG, "  [DIAG] after HW init: internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* DOA buffers + mutex */
    doa_mutex     = xSemaphoreCreateMutex();
    doa_left_buf  = (int16_t *)heap_caps_aligned_alloc(64, DOA_BUF_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    doa_right_buf = (int16_t *)heap_caps_aligned_alloc(64, DOA_BUF_SIZE * sizeof(int16_t), MALLOC_CAP_SPIRAM);
    if (doa_left_buf)  memset(doa_left_buf,  0, DOA_BUF_SIZE * sizeof(int16_t));
    if (doa_right_buf) memset(doa_right_buf, 0, DOA_BUF_SIZE * sizeof(int16_t));
    if (!doa_mutex || !doa_left_buf || !doa_right_buf) {
        ESP_LOGE(TAG, "DOA buffer allocation failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "DOA ready: mic_dist=%.3fm, max_tdoa=%d samples", MIC_DISTANCE_M, MAX_TDOA_SAMPLES);
    ESP_LOGI(TAG, "  [DIAG] after DOA+queue: internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Create UI event queues */
    g_sr_queue = xQueueCreate(5, sizeof(sr_event_t));
    g_ei_queue = xQueueCreate(5, sizeof(sr_ei_event_t));
    if (!g_sr_queue || !g_ei_queue) {
        ESP_LOGW(TAG, "Failed to create UI queues (continuing without UI)");
    }

    /* Load models from SPIFFS "model" partition */
    srmodel_list_t *models = esp_srmodel_init("model");
    if (models == NULL) {
        ESP_LOGE(TAG, "Failed to load SR models from 'model' partition");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "  [DIAG] after model load: internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Configure AFE: dual mic + speech enhancement + high perf */
    afe_config_t *afe_config = afe_config_init("MM", models, AFE_TYPE_SR, AFE_MODE_HIGH_PERF);
    afe_config->aec_init     = false;
    afe_config->se_init      = true;
    afe_config->vad_init     = true;
    afe_config->wakenet_init = true;
    afe_config->vad_mode     = VAD_MODE_3;
    afe_config->pcm_config.mic_num = 2;
    afe_config->pcm_config.ref_num = 0;

    afe_handle = esp_afe_handle_from_config(afe_config);
    afe_data   = afe_handle->create_from_config(afe_config);
    afe_config_free(afe_config);
    ESP_LOGI(TAG, "  [DIAG] after AFE create: internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Load MultiNet */
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, ESP_MN_CHINESE);
    multinet = esp_mn_handle_from_name(mn_name);
    mn_data  = multinet->create(mn_name, 6000);
    inject_custom_commands();
    ESP_LOGI(TAG, "  [DIAG] after MultiNet: internal=%u  PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* ====== Create feed_task + detect_task (split architecture, matching P4) ======
     * feed_task: I2S read → AFE feed (never blocks on fetch)
     * detect_task: continuous AFE fetch → wake word → MultiNet (never blocks I2S)
     * Split prevents the deadlock where fetch() waits for feed() data that can't
     * arrive because the single-task is stuck inside fetch().
     */
    #define FEED_STACK_BYTES   (8 * 1024)
    #define DETECT_STACK_BYTES (8 * 1024)

    static StaticTask_t s_feed_tcb;
    StackType_t *feed_stack = (StackType_t *)heap_caps_malloc(FEED_STACK_BYTES, MALLOC_CAP_SPIRAM);
    if (!feed_stack) {
        ESP_LOGE(TAG, "Failed to allocate feed_task stack");
        return ESP_ERR_NO_MEM;
    }
    s_feed_task_handle = xTaskCreateStatic(
        feed_task, "feed", FEED_STACK_BYTES / sizeof(StackType_t),
        NULL, 15, feed_stack, &s_feed_tcb);
    if (s_feed_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create feed_task");
        free(feed_stack);
        return ESP_FAIL;
    }

    static StaticTask_t s_detect_tcb;
    StackType_t *detect_stack = (StackType_t *)heap_caps_malloc(DETECT_STACK_BYTES, MALLOC_CAP_SPIRAM);
    if (!detect_stack) {
        ESP_LOGE(TAG, "Failed to allocate detect_task stack");
        return ESP_ERR_NO_MEM;
    }
    s_detect_task_handle = xTaskCreateStatic(
        detect_task, "detect", DETECT_STACK_BYTES / sizeof(StackType_t),
        NULL, 14, detect_stack, &s_detect_tcb);
    if (s_detect_task_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create detect_task");
        free(detect_stack);
        return ESP_FAIL;
    }

    /* Create EI semaphore and environment sound classification task */
    ei_infer_sem = xSemaphoreCreateBinary();
    #define EI_TASK_STACK_BYTES (8 * 1024)
    static StaticTask_t s_ei_tcb;
    StackType_t *ei_stack = (StackType_t *)heap_caps_malloc(EI_TASK_STACK_BYTES, MALLOC_CAP_SPIRAM);
    if (ei_stack && ei_infer_sem) {
        xTaskCreateStatic(env_sound_AI_Task, "env_ai",
            EI_TASK_STACK_BYTES / sizeof(StackType_t),
            NULL, 4, ei_stack, &s_ei_tcb);
        ESP_LOGI(TAG, "Environment sound AI task created");
    } else {
        ESP_LOGW(TAG, "Failed to create EI task (continuing without env sound)");
    }

    ESP_LOGI(TAG, "Audio SR initialized successfully");
    return ESP_OK;
}

esp_codec_dev_handle_t audio_sr_get_codec_dev(void)
{
    return codec_dev;
}

i2s_chan_handle_t audio_sr_get_tx_handle(void)
{
    return tx_handle;
}

i2c_master_bus_handle_t audio_sr_get_i2c_bus(void)
{
    return s_i2c_bus;
}

/* ====== Mode switching: suspend / resume SR tasks ====== */

void audio_sr_suspend(void)
{
    if (s_feed_task_handle) {
        vTaskSuspend(s_feed_task_handle);
        ESP_LOGI(TAG, "feed_task suspended");
    }
    if (s_detect_task_handle) {
        vTaskSuspend(s_detect_task_handle);
        ESP_LOGI(TAG, "detect_task suspended");
    }
}

void audio_sr_resume(void)
{
    if (s_feed_task_handle) {
        vTaskResume(s_feed_task_handle);
        ESP_LOGI(TAG, "feed_task resumed");
    }
    if (s_detect_task_handle) {
        vTaskResume(s_detect_task_handle);
        ESP_LOGI(TAG, "detect_task resumed");
    }
}
