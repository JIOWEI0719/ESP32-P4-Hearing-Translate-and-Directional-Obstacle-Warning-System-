/**
 * @file uart_gesture.c
 * @brief UART 手势识别结果接收与解析
 *
 * 协议格式 (9 字节):
 *   Byte0: 0xAA (帧头1)
 *   Byte1: 0x55 (帧头2)
 *   Byte2: 协议版本 (0x01)
 *   Byte3: 帧类型   (0x01 = 手势识别结果)
 *   Byte4: 序号     (0~255 循环递增)
 *   Byte5: 手势 ID  (0~6)
 *   Byte6: 置信度   (0~100)
 *   Byte7: 状态位   (bit0=有效, bit1=低置信度, bit2=保持中)
 *   Byte8: CRC8     (Byte2~Byte7 的校验, 多项式 0x31)
 */

#include "uart_gesture.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "uart_gesture";

/* ── UART 配置 ── */
#define UART_PORT              UART_NUM_0
#define UART_RX_PIN            38
#define UART_TX_PIN            37
#define UART_BUF_SIZE          1024
#define UART_QUEUE_SIZE        10

/* ── 协议常量 ── */
#define FRAME_HEAD_0           0xAA
#define FRAME_HEAD_1           0x55
#define FRAME_PAYLOAD_LEN      7     /* Byte2 ~ Byte8 (含 CRC) */
#define FRAME_TOTAL_LEN        9

/* ── 帧解析状态机 ── */
typedef enum {
    PARSER_WAIT_AA,
    PARSER_WAIT_55,
    PARSER_READ_PAYLOAD,
} uart_parser_state_t;

/* ── 全局状态 ── */
static uart_gesture_cb_t  s_callback      = NULL;
static void              *s_user_data     = NULL;
static float              s_debounce_sec  = 2.0f;
static uint8_t            s_last_gesture  = 0xFF;
static int64_t            s_last_trigger_us = 0;
static bool               s_initialized   = false;

/* ── CRC8 表 (多项式 0x07, 无反转, init=0x00, xorout=0x00) ── */
static const uint8_t s_crc8_table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
};

/* ── 日志辅助 ── */
static void log_frame_hex(const uint8_t *data, size_t len)
{
    char hex[64] = {0};
    size_t pos = 0;
    for (size_t i = 0; i < len && pos < sizeof(hex) - 3; i++) {
        pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", data[i]);
    }
    ESP_LOGI(TAG, "RAW: %s", hex);
}

/* ── CRC8 计算 ── */
static uint8_t crc8_calc(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = s_crc8_table[crc ^ data[i]];
    }
    return crc;
}

/* ── 手势文件名映射 ── */
static const char *s_gesture_name(uint8_t id)
{
    switch (id) {
        case 0:  return "A_LITTLE";
        case 1:  return "GOOD";
        case 2:  return "HELP";
        case 3:  return "ME";
        case 4:  return "NAME";
        case 5:  return "SIGN_LANGUAGE";
        case 6:  return "YOU";
        default: return "UNKNOWN";
    }
}

/* ── 手势事件触发 ── */
static void gesture_trigger(const uart_gesture_event_t *event)
{
    /* 去抖检查: 同一手势在间隔内忽略 */
    if (s_last_gesture == event->gesture_id) {
        int64_t now = esp_timer_get_time();
        int64_t elapsed_us = now - s_last_trigger_us;
        float elapsed_sec = (float)elapsed_us / 1000000.0f;
        if (elapsed_sec < s_debounce_sec) {
            ESP_LOGI(TAG, "[DEBOUNCE] %s (id=%d) skipped, %.1fs < %.1fs",
                     s_gesture_name(event->gesture_id),
                     event->gesture_id, elapsed_sec, s_debounce_sec);
            return;
        }
    }

    s_last_gesture     = event->gesture_id;
    s_last_trigger_us  = esp_timer_get_time();

    ESP_LOGI(TAG, "[TRIGGER] %s (id=%d, conf=%d%%, seq=%d, status=0x%02X)",
             s_gesture_name(event->gesture_id),
             event->gesture_id, event->confidence, event->seq, event->status);

    if (s_callback) {
        s_callback(event, s_user_data);
    }
}

/* ── 协议解析 (在接收任务中逐字节调用) ── */

typedef struct {
    uart_parser_state_t state;
    uint8_t             payload[FRAME_PAYLOAD_LEN];
    size_t              payload_pos;
} uart_parser_t;

static void parser_reset(uart_parser_t *p)
{
    p->state        = PARSER_WAIT_AA;
    p->payload_pos  = 0;
    memset(p->payload, 0, sizeof(p->payload));
}

static void parser_feed(uart_parser_t *p, uint8_t byte)
{
    switch (p->state) {

    case PARSER_WAIT_AA:
        if (byte == FRAME_HEAD_0) {
            p->state = PARSER_WAIT_55;
        }
        /* 非 0xAA 的字节静默忽略 (可能是上一帧残余或噪声) */
        break;

    case PARSER_WAIT_55:
        if (byte == FRAME_HEAD_1) {
            p->state       = PARSER_READ_PAYLOAD;
            p->payload_pos = 0;
        } else if (byte == FRAME_HEAD_0) {
            /* 连续 AA AA 的情况，保持在 WAIT_55 */
        } else {
            /* 第二个字节不是 0x55，重新开始 */
            parser_reset(p);
            /* 这个字节可能是新的 0xAA，递归处理 */
            parser_feed(p, byte);
        }
        break;

    case PARSER_READ_PAYLOAD:
        p->payload[p->payload_pos++] = byte;

        if (p->payload_pos >= FRAME_PAYLOAD_LEN) {
            /* 收到完整一帧 (header + payload 共 9 字节) */
            log_frame_hex(p->payload, FRAME_PAYLOAD_LEN);

            /* CRC8 校验: Byte2~Byte6 → 共 5 字节，与 Byte8 比较 */
            uint8_t crc_calc = crc8_calc(p->payload, FRAME_PAYLOAD_LEN - 1); /* 前 6 字节 */
            uint8_t crc_recv = p->payload[FRAME_PAYLOAD_LEN - 1];            /* 最后 1 字节 */

            if (crc_calc != crc_recv) {
                ESP_LOGW(TAG, "[CRC] mismatch: calc=0x%02X recv=0x%02X — frame dropped",
                         crc_calc, crc_recv);
                parser_reset(p);
                return;
            }

            /* 提取字段 */
            uint8_t version    = p->payload[0];   /* Byte2 */
            uint8_t frame_type = p->payload[1];   /* Byte3 */
            uint8_t seq        = p->payload[2];   /* Byte4 */
            uint8_t gesture_id = p->payload[3];   /* Byte5 */
            uint8_t confidence = p->payload[4];   /* Byte6 */
            uint8_t status     = p->payload[5];   /* Byte7 */

            ESP_LOGI(TAG, "[PARSE] ver=%d type=%d seq=%d id=%d(%s) conf=%d%% status=0x%02X",
                     version, frame_type, seq,
                     gesture_id, s_gesture_name(gesture_id),
                     confidence, status);

            /* ── 状态判断 ── */
            if (!(status & 0x01)) {
                /* bit0 = 0: 当前结果无效 */
                ESP_LOGI(TAG, "[STATUS] bit0=0 → invalid, ignored");
                parser_reset(p);
                return;
            }

            if (status & 0x02) {
                /* bit1 = 1: 低置信度，仅记录日志 */
                ESP_LOGI(TAG, "[STATUS] bit1=1 → low confidence, logged only");
                parser_reset(p);
                return;
            }

            /* ── 有效手势 ── */
            if (gesture_id == 0xFF) {
                ESP_LOGI(TAG, "[STATUS] GESTURE_UNKNOWN → no playback");
                parser_reset(p);
                return;
            }

            if (gesture_id > 6) {
                ESP_LOGW(TAG, "[STATUS] out-of-range gesture_id=%d → ignored",
                         gesture_id);
                parser_reset(p);
                return;
            }

            /* 触发回调 */
            uart_gesture_event_t event = {
                .gesture_id = gesture_id,
                .confidence = confidence,
                .seq        = seq,
                .status     = status,
            };
            gesture_trigger(&event);

            parser_reset(p);
        }
        break;
    }
}

/* ── 接收任务 ── */
static void uart_gesture_task(void *arg)
{
    ESP_LOGI(TAG, "Task started, waiting for frames on UART%d...", UART_PORT);

    uart_parser_t parser;
    parser_reset(&parser);

    uint8_t buf[64];

    while (1) {
        int len = uart_read_bytes(UART_PORT, buf, sizeof(buf),
                                  pdMS_TO_TICKS(100));
        if (len > 0) {
            /* 监控日志：打印本次收到的原始字节 */
            char hex[128];
            size_t pos = 0;
            for (int i = 0; i < len && pos < sizeof(hex) - 3; i++) {
                pos += snprintf(hex + pos, sizeof(hex) - pos, "%02X ", buf[i]);
            }
            ESP_LOGI(TAG, "[RX] %d bytes: %s", len, hex);

            for (int i = 0; i < len; i++) {
                parser_feed(&parser, buf[i]);
            }
        }
        /* 如果没有数据则自然 yield，不占用 CPU */
    }
}

/* ── 公开 API ── */

esp_err_t uart_gesture_init(int baud_rate, uart_gesture_cb_t callback, void *user_data)
{
    if (s_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_ERR_INVALID_STATE;
    }

    s_callback  = callback;
    s_user_data = user_data;

    /* ── 配置 UART 驱动 ── */
    uart_config_t uart_config = {
        .baud_rate           = baud_rate,
        .data_bits           = UART_DATA_8_BITS,
        .parity              = UART_PARITY_DISABLE,
        .stop_bits           = UART_STOP_BITS_1,
        .flow_ctrl           = UART_HW_FLOWCTRL_DISABLE,
        .source_clk          = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_param_config(UART_PORT, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_driver_install(UART_PORT, UART_BUF_SIZE, UART_BUF_SIZE,
                              UART_QUEUE_SIZE, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install: %s", esp_err_to_name(ret));
        return ret;
    }

    /* ── 创建接收任务 ── */
    BaseType_t task_ret = xTaskCreate(uart_gesture_task, "uart_gesture",
                                      4096, NULL, 5, NULL);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed: %d", task_ret);
        uart_driver_delete(UART_PORT);
        return ESP_ERR_NO_MEM;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "Initialized: UART%d baud=%d pins RX=%d TX=%d",
             UART_PORT, baud_rate, UART_RX_PIN, UART_TX_PIN);
    return ESP_OK;
}

void uart_gesture_set_debounce(float sec)
{
    s_debounce_sec = sec;
    ESP_LOGI(TAG, "Debounce set to %.1fs", sec);
}
