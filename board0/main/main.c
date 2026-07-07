/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_memory_utils.h"
#include "esp_heap_caps.h"
#include "esp_spiffs.h"
#include "esp_lv_adapter.h"
#include "lvgl.h"
#include "bsp/esp-bsp.h"
#include "bsp/display.h"
#include "lv_demos.h"
#include "lvgl_adapter_init.h"
#include <assert.h>

/* WiFi includes */
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

/* Email UI */
#include "email_ui.h"

/* Audio Speech Recognition */
#include "audio_sr.h"
#include "ui_speech.h"

/* Music Player */
#include "music_player.h"

/* UART Gesture Receiver */
#include "uart_gesture.h"

static const char *TAG = "main";

/* ========== WiFi State ========== */
EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define EXAMPLE_ESP_WIFI_SSID      "YourWiFiSSID"
#define EXAMPLE_ESP_WIFI_PASS      "YourWiFiPassword"
#define EXAMPLE_ESP_MAXIMUM_RETRY  5

/* ========== Mode Switch ========== */

typedef enum {
    MODE_VOICE_RECOGNITION,   /* 语音识别: audio_sr running, UART gestures ignored */
    MODE_SIGN_LANGUAGE,       /* 手语识别: audio_sr suspended, UART gestures → MP3 */
} app_mode_t;

static app_mode_t   s_app_mode = MODE_VOICE_RECOGNITION;
static lv_obj_t    *s_mode_btn = NULL;
static lv_obj_t    *s_mode_btn_label = NULL;

static void update_mode_button(void)
{
    if (!s_mode_btn_label || !s_mode_btn) return;

    if (s_app_mode == MODE_VOICE_RECOGNITION) {
        lv_label_set_text(s_mode_btn_label, "Voice Mode");
        lv_obj_set_style_bg_color(s_mode_btn, lv_color_hex(0x2196F3), 0);
        lv_obj_set_style_bg_grad_color(s_mode_btn, lv_color_hex(0x1976D2), 0);
    } else {
        lv_label_set_text(s_mode_btn_label, "Sign Mode");
        lv_obj_set_style_bg_color(s_mode_btn, lv_color_hex(0xFF9800), 0);
        lv_obj_set_style_bg_grad_color(s_mode_btn, lv_color_hex(0xF57C00), 0);
    }
}

static void switch_to_mode(app_mode_t new_mode)
{
    if (s_app_mode == new_mode) return;

    if (new_mode == MODE_SIGN_LANGUAGE) {
        /* Sign mode: suspend audio_sr, enable UART gestures */
        audio_sr_suspend();
        s_app_mode = MODE_SIGN_LANGUAGE;
        ESP_LOGI(TAG, ">>> Sign Mode (audio_sr paused, UART active)");
    } else {
        /* Voice mode: resume audio_sr */
        s_app_mode = MODE_VOICE_RECOGNITION;
        audio_sr_resume();
        ESP_LOGI(TAG, ">>> Voice Mode (audio_sr running)");
    }

    update_mode_button();
}

static void mode_btn_click_cb(lv_event_t *e)
{
    LV_UNUSED(e);

    if (s_app_mode == MODE_VOICE_RECOGNITION) {
        switch_to_mode(MODE_SIGN_LANGUAGE);
    } else {
        switch_to_mode(MODE_VOICE_RECOGNITION);
    }
}

/* ========== WiFi Event Handler ========== */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/* ========== WiFi Station Init ========== */
#define WIFI_RETURN_ON_ERROR(x, msg) do {         \
    esp_err_t err = (x);                           \
    if (err != ESP_OK) {                           \
        ESP_LOGE(TAG, "%s: %s (0x%x)", msg, esp_err_to_name(err), err); \
        return;                                    \
    }                                              \
} while(0)

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    WIFI_RETURN_ON_ERROR(esp_netif_init(), "esp_netif_init failed");
    WIFI_RETURN_ON_ERROR(esp_event_loop_create_default(), "esp_event_loop_create_default failed");
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    WIFI_RETURN_ON_ERROR(esp_wifi_init(&cfg), "esp_wifi_init failed");

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    WIFI_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID,
                    &wifi_event_handler,
                    NULL,
                    &instance_any_id),
                    "register WIFI_EVENT handler failed");
    WIFI_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP,
                    &wifi_event_handler,
                    NULL,
                    &instance_got_ip),
                    "register IP_EVENT handler failed");

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
        },
    };
    WIFI_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), "esp_wifi_set_mode failed");
    WIFI_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &wifi_config), "esp_wifi_set_config failed");
    WIFI_RETURN_ON_ERROR(esp_wifi_start(), "esp_wifi_start failed");

    ESP_LOGI(TAG, "wifi_init_sta finished, waiting for connection...");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to AP SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s", EXAMPLE_ESP_WIFI_SSID);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

/* ========== WiFi Task ========== */
static void wifi_task(void *pvParameters)
{
    ESP_LOGI(TAG, "WiFi task started");
    wifi_init_sta();
    ESP_LOGI(TAG, "WiFi task finished");
    vTaskDelete(NULL);
}

/* ========== UART Gesture → MP3 Playback ========== */

static const char *s_gesture_files[] = {
    "a_little.mp3",   /* GESTURE_A_LITTLE      = 0 */
    "good.mp3",       /* GESTURE_GOOD          = 1 */
    "help.mp3",       /* GESTURE_HELP          = 2 */
    "me.mp3",         /* GESTURE_ME            = 3 */
    "name.mp3",       /* GESTURE_NAME          = 4 */
    "sign.mp3",       /* GESTURE_SIGN_LANGUAGE = 5 */
    "you.mp3",        /* GESTURE_YOU           = 6 */
};

static void on_gesture_event(const uart_gesture_event_t *event, void *user_data)
{
    /* 仅在手语识别模式下响应 UART 手势 */
    if (s_app_mode != MODE_SIGN_LANGUAGE) {
        ESP_LOGI(TAG, "[UART] gesture ignored (not in sign language mode)");
        return;
    }

    if (event->gesture_id >= sizeof(s_gesture_files) / sizeof(s_gesture_files[0])) {
        ESP_LOGW(TAG, "Unknown gesture_id: %d", event->gesture_id);
        return;
    }

    ESP_LOGI(TAG, "Gesture → play '%s' (id=%d, conf=%d%%, seq=%d)",
             s_gesture_files[event->gesture_id],
             event->gesture_id, event->confidence, event->seq);

    music_player_play_by_name(s_gesture_files[event->gesture_id]);
}

/* ========== Mode Switch Button ========== */

static lv_obj_t *mode_button_create(lv_obj_t *parent)
{
    s_mode_btn = lv_btn_create(parent);
    lv_obj_set_size(s_mode_btn, 200, 60);
    lv_obj_set_pos(s_mode_btn, 50, 280);
    lv_obj_set_style_radius(s_mode_btn, 12, 0);
    lv_obj_set_style_shadow_width(s_mode_btn, 4, 0);
    lv_obj_set_style_shadow_color(s_mode_btn, lv_color_hex(0x333333), 0);
    lv_obj_set_style_shadow_opa(s_mode_btn, LV_OPA_30, 0);
    lv_obj_add_event_cb(s_mode_btn, mode_btn_click_cb, LV_EVENT_CLICKED, NULL);

    s_mode_btn_label = lv_label_create(s_mode_btn);
    lv_obj_center(s_mode_btn_label);
    lv_obj_set_style_text_color(s_mode_btn_label, lv_color_white(), 0);
    lv_obj_set_style_text_font(s_mode_btn_label, &lv_font_montserrat_16, 0);

    /* 默认: 语音识别模式 (蓝色) */
    s_app_mode = MODE_VOICE_RECOGNITION;
    update_mode_button();

    return s_mode_btn;
}

/* ========== Main Entry ========== */
void app_main(void)
{
    /* ---- NVS init ---- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized");

    /* ---- Mount SPIFFS storage partition ---- */
    ESP_LOGI(TAG, "Mounting SPIFFS storage partition...");
    {
        esp_vfs_spiffs_conf_t spiffs_conf = {
            .base_path = BSP_SPIFFS_MOUNT_POINT,
            .partition_label = "storage",
            .max_files = 5,
            .format_if_mount_failed = false,
        };
        ret = esp_vfs_spiffs_register(&spiffs_conf);
        if (ret != ESP_OK && ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "SPIFFS mount failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SPIFFS mounted OK");
        }
    }

    /* ---- Init BSP speaker (I2S_NUM_0, I2C, OUT codec handle) ---- */
    ESP_LOGI(TAG, "[INIT-1] bsp_audio_codec_speaker_init()...");
    esp_codec_dev_handle_t bsp_play_handle = bsp_audio_codec_speaker_init();
    assert(bsp_play_handle);
    ESP_LOGI(TAG, "[INIT-1] BSP speaker handle OK: %p", bsp_play_handle);

    /* ---- Start Audio Speech Recognition (I2S_NUM_1 only) ---- */
    ESP_LOGI(TAG, "[INIT-2] audio_sr_init(true)...");
    ret = audio_sr_init(true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Audio SR init failed: %s", esp_err_to_name(ret));
    }

    /* ---- Init audio player (BSP OUT handle) ---- */
    ESP_LOGI(TAG, "[INIT-3] music_player_audio_init()...");
    ESP_ERROR_CHECK(music_player_audio_init(bsp_play_handle));
    ESP_LOGI(TAG, "[INIT-3] Audio player OK");

    /* ---- UART Gesture Receiver ---- */
    ESP_LOGI(TAG, "[INIT-4] uart_gesture_init(115200)...");
    ret = uart_gesture_init(115200, on_gesture_event, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART gesture init failed: %s", esp_err_to_name(ret));
    }

    /* ---- LVGL display + touch init ---- */
    bsp_display_cfg_t cfg = {
        .hw_cfg = {
            .hdmi_resolution = BSP_HDMI_RES_NONE,
            .dsi_bus = {
                .lane_bit_rate_mbps = BSP_LCD_MIPI_DSI_LANE_BITRATE_MBPS,
            },
        },
    };
    lv_display_t *disp = lvgl_adapter_init(&cfg);
    assert(disp != NULL && "Failed to init LVGL adapter");
    bsp_display_backlight_on();

    ESP_ERROR_CHECK(esp_lv_adapter_lock(-1));
    email_ui_create(lv_scr_act());
    ui_speech_create(lv_scr_act());
    mode_button_create(lv_scr_act());   /* 模式切换按钮 (替代原 Play All) */
    esp_lv_adapter_unlock();

    /* ---- Create WiFi task (non-blocking) ---- */
    xTaskCreate(wifi_task, "wifi_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "WiFi task created");

}
