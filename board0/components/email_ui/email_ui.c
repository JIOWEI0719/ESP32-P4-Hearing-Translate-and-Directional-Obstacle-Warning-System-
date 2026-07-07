#include "email_ui.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
/* smtp_send_email() defined in main component */
extern esp_err_t smtp_send_email(void);

static const char *TAG = "email_ui";

/* Pre-allocated SMTP task stack & TCB (avoids heap fragmentation) */
#define SMTP_STACK_BYTES 16384
static StaticTask_t s_smtp_tcb;
static StackType_t  s_smtp_stack[SMTP_STACK_BYTES / sizeof(StackType_t)];
static TaskHandle_t s_smtp_handle = NULL;

/* Reference to WiFi event group defined in main.c */
extern EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

/* ========== SMTP Status ========== */
typedef enum {
    SMTP_IDLE,
    SMTP_SENDING,
    SMTP_SUCCESS,
    SMTP_FAILED,
} smtp_status_t;

static volatile smtp_status_t g_smtp_status = SMTP_IDLE;
static lv_obj_t *g_status_label = NULL;
static lv_obj_t *g_send_btn     = NULL;

/* ========== SMTP Task ========== */
static void smtp_task(void *pvParameters)
{
    ESP_LOGI(TAG, "SMTP task started");
    g_smtp_status = SMTP_SENDING;

    esp_err_t ret = smtp_send_email();

    g_smtp_status = (ret == ESP_OK) ? SMTP_SUCCESS : SMTP_FAILED;
    ESP_LOGI(TAG, "SMTP task finished: %s", (ret == ESP_OK) ? "OK" : "FAIL");
    vTaskDelete(NULL);
}

/* ========== Status Poll Timer ========== */
static void status_timer_cb(lv_timer_t *timer)
{
    (void)timer;
    switch (g_smtp_status) {
    case SMTP_IDLE:
        lv_label_set_text(g_status_label, "Ready");
        if (g_send_btn) lv_obj_clear_state(g_send_btn, LV_STATE_DISABLED);
        break;
    case SMTP_SENDING:
        lv_label_set_text(g_status_label, "Sending...");
        if (g_send_btn) lv_obj_add_state(g_send_btn, LV_STATE_DISABLED);
        break;
    case SMTP_SUCCESS:
        lv_label_set_text(g_status_label, "Email sent!");
        if (g_send_btn) lv_obj_clear_state(g_send_btn, LV_STATE_DISABLED);
        break;
    case SMTP_FAILED:
        lv_label_set_text(g_status_label, "Send failed");
        if (g_send_btn) lv_obj_clear_state(g_send_btn, LV_STATE_DISABLED);
        break;
    }
}

/* ========== Button Click Handler ========== */
static void on_send_click(lv_event_t *e)
{
    (void)e;

    /* Check WiFi status */
    if (s_wifi_event_group == NULL) {
        lv_label_set_text(g_status_label, "WiFi not ready");
        g_smtp_status = SMTP_FAILED;
        return;
    }
    EventBits_t bits = xEventGroupGetBits(s_wifi_event_group);
    if (!(bits & WIFI_CONNECTED_BIT)) {
        lv_label_set_text(g_status_label, "WiFi not connected");
        g_smtp_status = SMTP_FAILED;
        return;
    }

    /* Guard: don't start a new task while one is already running */
    if (g_smtp_status == SMTP_SENDING) {
        ESP_LOGW(TAG, "SMTP task already running");
        return;
    }

    /* Create SMTP task (static allocation — no heap needed) */
    s_smtp_handle = xTaskCreateStatic(smtp_task, "smtp_task",
                                      SMTP_STACK_BYTES / sizeof(StackType_t),
                                      NULL, 5, s_smtp_stack, &s_smtp_tcb);
    if (s_smtp_handle == NULL) {
        ESP_LOGE(TAG, "Failed to create SMTP task");
        lv_label_set_text(g_status_label, "Task create failed");
        g_smtp_status = SMTP_FAILED;
    }
}

/* ========== UI Creation ========== */
lv_obj_t *email_ui_create(lv_obj_t *parent)
{
    lv_obj_t *cont = lv_obj_create(parent);
    lv_obj_set_size(cont, 300, 260);
    lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 0, 0);
    lv_obj_set_style_border_width(cont, 0, 0);
    lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);

    /* Styled send button (plain LVGL button, no image dependency) */
    g_send_btn = lv_button_create(cont);
    lv_obj_set_size(g_send_btn, 258, 160);
    lv_obj_align(g_send_btn, LV_ALIGN_TOP_MID, 0, 20);
    lv_obj_set_style_radius(g_send_btn, 12, 0);
    lv_obj_set_style_bg_color(g_send_btn, lv_color_hex(0x2196F3), 0);
    lv_obj_set_style_bg_grad_color(g_send_btn, lv_color_hex(0x1565C0), 0);
    lv_obj_set_style_bg_grad_dir(g_send_btn, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_shadow_width(g_send_btn, 8, 0);
    lv_obj_set_style_shadow_color(g_send_btn, lv_color_hex(0x1565C0), 0);
    lv_obj_set_style_shadow_opa(g_send_btn, LV_OPA_50, 0);

    lv_obj_t *btn_label = lv_label_create(g_send_btn);
    lv_label_set_text(btn_label, "Send Email");
    lv_obj_center(btn_label);
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(btn_label, lv_color_white(), 0);

    lv_obj_add_event_cb(g_send_btn, on_send_click, LV_EVENT_CLICKED, NULL);

    /* Status label below the button */
    g_status_label = lv_label_create(cont);
    lv_label_set_text(g_status_label, "Ready");
    lv_obj_align(g_status_label, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_text_align(g_status_label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(g_status_label, &lv_font_montserrat_18, 0);

    /* Poll timer: updates status label every 200ms */
    lv_timer_create(status_timer_cb, 200, NULL);

    ESP_LOGI(TAG, "Email UI created");
    return cont;
}
