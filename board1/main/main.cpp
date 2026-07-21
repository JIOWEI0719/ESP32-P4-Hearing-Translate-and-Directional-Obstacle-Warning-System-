/* ESP32-P4 standalone camera capture with ESP-DL gesture inferencing. */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <algorithm>
#include <map>
#include <vector>

#include "app_ethernet.h"
#include "camera_capture.h"
#include "dl_define.hpp"
#include "dl_model_base.hpp"
#include "dl_tensor_base.hpp"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "lwip/apps/netbiosns.h"
#include "mdns.h"
#include "rtsp_service.h"
#include "video_dev.h"

#define EXAMPLE_MDNS_HOST_NAME "esp32p4-rtsp"
#define ENABLE_GESTURE_INFERENCE 1
#define ENABLE_RTSP_PREVIEW 0
#define INFERENCE_INTERVAL_MS 125
#define INFERENCE_TASK_STACK_SIZE (12 * 1024)
#define INFERENCE_TASK_PRIORITY 2
#define INFERENCE_IDLE_DELAY_MS 10
#define CAMERA_CAPTURE_TASK_STACK_SIZE (6 * 1024)
#define CAMERA_CAPTURE_TASK_PRIORITY 3
#define DEBUG_PREPROCESS_VARIANT_TEST 0
#define DEBUG_DUMP_FIRST_FEATURE_PGM 0
#define DEBUG_MODEL_INPUT_HTTP 0

static const char *TAG = "CAM_GESTURE";

static constexpr int GESTURE_INPUT_WIDTH = 224;
static constexpr int GESTURE_INPUT_HEIGHT = 224;
static constexpr size_t GESTURE_INPUT_FRAME_SIZE =
    (size_t)GESTURE_INPUT_WIDTH * (size_t)GESTURE_INPUT_HEIGHT;
static constexpr int GESTURE_LABEL_COUNT = 7;
static constexpr int GESTURE_MODEL_INPUT_EXPONENT = -7;
static constexpr int GESTURE_MODEL_OUTPUT_EXPONENT = -4;
static constexpr uart_port_t GESTURE_UART_NUM = UART_NUM_1;
static constexpr gpio_num_t GESTURE_UART_TX_GPIO = GPIO_NUM_36;
static constexpr int GESTURE_UART_BAUD_RATE = 115200;
static constexpr int GESTURE_UART_RX_BUFFER_SIZE = 256;
static constexpr float GESTURE_DEFAULT_CONFIDENCE_THRESHOLD = 0.90f;
static constexpr float GESTURE_A_LITTLE_CONFIDENCE_THRESHOLD = 0.65f;
static constexpr float GESTURE_YOU_CONFIDENCE_THRESHOLD = 0.55f;
static constexpr float GESTURE_GOOD_CONFIDENCE_THRESHOLD = 0.92f;
static constexpr float GESTURE_MID_CONFIDENCE_THRESHOLD = 0.85f;
static constexpr uint8_t GESTURE_UART_FRAME_HEAD_0 = 0xAA;
static constexpr uint8_t GESTURE_UART_FRAME_HEAD_1 = 0x55;
static constexpr uint8_t GESTURE_UART_PROTOCOL_VERSION = 0x01;
static constexpr uint8_t GESTURE_UART_FRAME_TYPE_RESULT = 0x01;
static constexpr uint8_t GESTURE_UART_STATUS_VALID = 0x01;
static constexpr uint8_t GESTURE_UART_STATUS_LOW_CONFIDENCE = 0x02;

static const char *GESTURE_LABELS[GESTURE_LABEL_COUNT] = {
    "a_little",
    "good",
    "help",
    "me",
    "name",
    "sign_language",
    "you",
};

enum gesture_id_t {
    GESTURE_ID_A_LITTLE = 0,
    GESTURE_ID_GOOD = 1,
    GESTURE_ID_HELP = 2,
    GESTURE_ID_ME = 3,
    GESTURE_ID_NAME = 4,
    GESTURE_ID_SIGN_LANGUAGE = 5,
    GESTURE_ID_YOU = 6,
};

extern const uint8_t gesture_cnn_224_gray_esp32p4_espdl_start[] asm("_binary_gesture_cnn_224_gray_esp32p4_espdl_start");
extern const uint8_t gesture_cnn_224_gray_esp32p4_espdl_end[] asm("_binary_gesture_cnn_224_gray_esp32p4_espdl_end");

static camera_capture_t g_camera_capture;
#if ENABLE_RTSP_PREVIEW
static camera_context g_camera_context;
static esp_rtsp_handle_t g_rtsp;
#endif

static SemaphoreHandle_t g_features_mutex;
static SemaphoreHandle_t g_frame_ready_sem;
static SemaphoreHandle_t g_debug_image_mutex;
static volatile bool g_inference_pending;
static int64_t g_last_capture_us;

static float *g_latest_features;
static float *g_base_features;
static float *g_inference_features;
static uint8_t g_gesture_uart_seq;

static bool allocate_feature_buffers()
{
    const size_t bytes = GESTURE_INPUT_FRAME_SIZE * sizeof(float);
    const uint32_t caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;

    g_latest_features = (float *)heap_caps_malloc(bytes, caps);
    g_base_features = (float *)heap_caps_malloc(bytes, caps);
    g_inference_features = (float *)heap_caps_malloc(bytes, caps);
    if (!g_latest_features || !g_base_features || !g_inference_features) {
        ESP_LOGE(TAG,
                 "failed to allocate 224x224 feature buffers in PSRAM (%u bytes each)",
                 (unsigned)bytes);
        return false;
    }

    ESP_LOGI(TAG,
             "allocated three 224x224 feature buffers in PSRAM (%u bytes total)",
             (unsigned)(bytes * 3));
    return true;
}

#if DEBUG_MODEL_INPUT_HTTP
static uint8_t g_latest_model_input_u8[GESTURE_INPUT_FRAME_SIZE];
static bool g_latest_model_input_ready;
static uint8_t g_debug_layout_y_800x640_s800[GESTURE_INPUT_FRAME_SIZE];
static uint8_t g_debug_layout_y_800x640_s960[GESTURE_INPUT_FRAME_SIZE];
static uint8_t g_debug_layout_y_800x640_s1024[GESTURE_INPUT_FRAME_SIZE];
static uint8_t g_debug_layout_y_640x800_s640[GESTURE_INPUT_FRAME_SIZE];
static uint8_t g_debug_layout_y_640x800_s800[GESTURE_INPUT_FRAME_SIZE];
static uint8_t g_debug_layout_o_uyy_e_vyy[GESTURE_INPUT_FRAME_SIZE];
static bool g_debug_layout_ready[6];
#endif

static float y_limited_to_full_range(float y)
{
    float value = (y - 16.0f) * (255.0f / 219.0f);
    if (value < 0.0f) {
        return 0.0f;
    }
    if (value > 255.0f) {
        return 255.0f;
    }
    return value;
}

#if ENABLE_RTSP_PREVIEW
static void net_connect()
{
    ESP_LOGI(TAG, "net_connect start");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    netbiosns_init();
    netbiosns_set_name(EXAMPLE_MDNS_HOST_NAME);

    ESP_ERROR_CHECK(mdns_init());
    ESP_ERROR_CHECK(mdns_hostname_set(EXAMPLE_MDNS_HOST_NAME));
    ESP_ERROR_CHECK(mdns_instance_name_set("esp32p4-rtsp"));

    mdns_txt_item_t service_txt[] = {
        {"stream", "h264"},
        {"mode", "gesture"},
    };
    ESP_ERROR_CHECK(mdns_service_add(EXAMPLE_MDNS_HOST_NAME,
                                     "_rtsp",
                                     "_tcp",
                                     RTSP_SERVER_PORT,
                                     service_txt,
                                     sizeof(service_txt) / sizeof(service_txt[0])));

    ESP_LOGI(TAG, "connecting ethernet");
    if (app_ethernet_connect() == nullptr) {
        ESP_LOGE(TAG, "ethernet connect failed");
        return;
    }
    ESP_LOGI(TAG, "network connected");
}
#endif

static void resize_center_crop_y_to_features(const uint8_t *y_plane,
                                             int src_width,
                                             int src_height,
                                             int src_stride,
                                             float *features)
{
    const int dst_width = GESTURE_INPUT_WIDTH;
    const int dst_height = GESTURE_INPUT_HEIGHT;
    const int crop_size = src_width < src_height ? src_width : src_height;
    const int crop_x = (src_width - crop_size) / 2;
    const int crop_y = (src_height - crop_size) / 2;

    for (int y = 0; y < dst_height; y++) {
        int y0 = crop_y + (y * crop_size) / dst_height;
        int y1 = crop_y + ((y + 1) * crop_size) / dst_height;
        if (y1 <= y0) {
            y1 = y0 + 1;
        }
        if (y1 > crop_y + crop_size) {
            y1 = crop_y + crop_size;
        }

        for (int x = 0; x < dst_width; x++) {
            int x0 = crop_x + (x * crop_size) / dst_width;
            int x1 = crop_x + ((x + 1) * crop_size) / dst_width;
            if (x1 <= x0) {
                x1 = x0 + 1;
            }
            if (x1 > crop_x + crop_size) {
                x1 = crop_x + crop_size;
            }

            uint32_t sum = 0;
            uint32_t count = 0;
            for (int sy = y0; sy < y1; sy++) {
                const uint8_t *row = y_plane + (sy * src_stride);
                for (int sx = x0; sx < x1; sx++) {
                    sum += row[sx];
                    count++;
                }
            }

            features[(y * dst_width) + x] = count ? ((float)sum / (float)count) : 0.0f;
        }
    }
}

static uint8_t get_o_uyy_e_vyy_luma(const uint8_t *frame, int row_stride, int x, int y)
{
    const uint8_t *row = frame + ((size_t)y * (size_t)row_stride);
    const int pair = x / 2;
    const int pair_offset = pair * 3;
    return row[pair_offset + 1 + (x & 1)];
}

static void resize_center_crop_o_uyy_e_vyy_to_features(const uint8_t *frame,
                                                       int src_width,
                                                       int src_height,
                                                       int row_stride,
                                                       float *features)
{
    const int dst_width = GESTURE_INPUT_WIDTH;
    const int dst_height = GESTURE_INPUT_HEIGHT;
    const int crop_size = src_width < src_height ? src_width : src_height;
    const int crop_x = (src_width - crop_size) / 2;
    const int crop_y = (src_height - crop_size) / 2;

    for (int y = 0; y < dst_height; y++) {
        int y0 = crop_y + (y * crop_size) / dst_height;
        int y1 = crop_y + ((y + 1) * crop_size) / dst_height;
        if (y1 <= y0) {
            y1 = y0 + 1;
        }
        if (y1 > crop_y + crop_size) {
            y1 = crop_y + crop_size;
        }

        for (int x = 0; x < dst_width; x++) {
            int x0 = crop_x + (x * crop_size) / dst_width;
            int x1 = crop_x + ((x + 1) * crop_size) / dst_width;
            if (x1 <= x0) {
                x1 = x0 + 1;
            }
            if (x1 > crop_x + crop_size) {
                x1 = crop_x + crop_size;
            }

            uint32_t sum = 0;
            uint32_t count = 0;
            for (int sy = y0; sy < y1; sy++) {
                for (int sx = x0; sx < x1; sx++) {
                    sum += get_o_uyy_e_vyy_luma(frame, row_stride, sx, sy);
                    count++;
                }
            }

            features[(y * dst_width) + x] = count ? ((float)sum / (float)count) : 0.0f;
        }
    }
}

#if DEBUG_MODEL_INPUT_HTTP
static void resize_center_crop_y_to_u8(const uint8_t *y_plane,
                                       int src_width,
                                       int src_height,
                                       int src_stride,
                                       uint8_t *image)
{
    const int dst_width = GESTURE_INPUT_WIDTH;
    const int dst_height = GESTURE_INPUT_HEIGHT;
    const int crop_size = src_width < src_height ? src_width : src_height;
    const int crop_x = (src_width - crop_size) / 2;
    const int crop_y = (src_height - crop_size) / 2;

    for (int y = 0; y < dst_height; y++) {
        int y0 = crop_y + (y * crop_size) / dst_height;
        int y1 = crop_y + ((y + 1) * crop_size) / dst_height;
        if (y1 <= y0) {
            y1 = y0 + 1;
        }
        if (y1 > crop_y + crop_size) {
            y1 = crop_y + crop_size;
        }

        for (int x = 0; x < dst_width; x++) {
            int x0 = crop_x + (x * crop_size) / dst_width;
            int x1 = crop_x + ((x + 1) * crop_size) / dst_width;
            if (x1 <= x0) {
                x1 = x0 + 1;
            }
            if (x1 > crop_x + crop_size) {
                x1 = crop_x + crop_size;
            }

            uint32_t sum = 0;
            uint32_t count = 0;
            for (int sy = y0; sy < y1; sy++) {
                const uint8_t *row = y_plane + (sy * src_stride);
                for (int sx = x0; sx < x1; sx++) {
                    sum += row[sx];
                    count++;
                }
            }

            image[(y * dst_width) + x] = count ? (uint8_t)((sum + (count / 2)) / count) : 0;
        }
    }
}

static void resize_center_crop_o_uyy_e_vyy_to_u8(const uint8_t *frame,
                                                 int src_width,
                                                 int src_height,
                                                 int row_stride,
                                                 uint8_t *image)
{
    const int dst_width = GESTURE_INPUT_WIDTH;
    const int dst_height = GESTURE_INPUT_HEIGHT;
    const int crop_size = src_width < src_height ? src_width : src_height;
    const int crop_x = (src_width - crop_size) / 2;
    const int crop_y = (src_height - crop_size) / 2;

    for (int y = 0; y < dst_height; y++) {
        int y0 = crop_y + (y * crop_size) / dst_height;
        int y1 = crop_y + ((y + 1) * crop_size) / dst_height;
        if (y1 <= y0) {
            y1 = y0 + 1;
        }
        if (y1 > crop_y + crop_size) {
            y1 = crop_y + crop_size;
        }

        for (int x = 0; x < dst_width; x++) {
            int x0 = crop_x + (x * crop_size) / dst_width;
            int x1 = crop_x + ((x + 1) * crop_size) / dst_width;
            if (x1 <= x0) {
                x1 = x0 + 1;
            }
            if (x1 > crop_x + crop_size) {
                x1 = crop_x + crop_size;
            }

            uint32_t sum = 0;
            uint32_t count = 0;
            for (int sy = y0; sy < y1; sy++) {
                for (int sx = x0; sx < x1; sx++) {
                    sum += get_o_uyy_e_vyy_luma(frame, row_stride, sx, sy);
                    count++;
                }
            }

            image[(y * dst_width) + x] = count ? (uint8_t)((sum + (count / 2)) / count) : 0;
        }
    }
}
#endif

static void prepare_features_raw_y(const float *src, float *dst, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        dst[i] = src[i];
    }
}

static void prepare_features_full_range_y(const float *src, float *dst, size_t length)
{
    for (size_t i = 0; i < length; i++) {
        dst[i] = y_limited_to_full_range(src[i]);
    }
}

static void prepare_features_flip_x(const float *src, float *dst, size_t length)
{
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const size_t expected_length = (size_t)width * (size_t)height;
    if (length < expected_length) {
        prepare_features_raw_y(src, dst, length);
        return;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[(y * width) + x] = src[(y * width) + (width - 1 - x)];
        }
    }
}

static void prepare_features_flip_y(const float *src, float *dst, size_t length)
{
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const size_t expected_length = (size_t)width * (size_t)height;
    if (length < expected_length) {
        prepare_features_raw_y(src, dst, length);
        return;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[(y * width) + x] = src[((height - 1 - y) * width) + x];
        }
    }
}

static void prepare_features_rotate_180(const float *src, float *dst, size_t length)
{
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const size_t expected_length = (size_t)width * (size_t)height;
    if (length < expected_length) {
        prepare_features_raw_y(src, dst, length);
        return;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[(y * width) + x] = src[((height - 1 - y) * width) + (width - 1 - x)];
        }
    }
}

static void prepare_features_rotate_90_cw(const float *src, float *dst, size_t length)
{
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const size_t expected_length = (size_t)width * (size_t)height;
    if (width != height || length < expected_length) {
        prepare_features_raw_y(src, dst, length);
        return;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[(y * width) + x] = src[((height - 1 - x) * width) + y];
        }
    }
}

static void prepare_features_rotate_90_ccw(const float *src, float *dst, size_t length)
{
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const size_t expected_length = (size_t)width * (size_t)height;
    if (width != height || length < expected_length) {
        prepare_features_raw_y(src, dst, length);
        return;
    }

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            dst[(y * width) + x] = src[(x * width) + (width - 1 - y)];
        }
    }
}

static void print_feature_stats(const float *features, size_t length)
{
    if (!features || length == 0) {
        return;
    }

    float min_value = features[0];
    float max_value = features[0];
    double sum = 0.0;

    for (size_t i = 0; i < length; i++) {
        float value = features[i];
        if (value < min_value) {
            min_value = value;
        }
        if (value > max_value) {
            max_value = value;
        }
        sum += value;
    }

    printf("Input feature stats: min=%.1f avg=%.1f max=%.1f\r\n",
           min_value,
           (float)(sum / (double)length),
           max_value);
}

static bool feature_frame_is_valid_for_dump(const float *features, size_t length)
{
    if (!features || length == 0) {
        return false;
    }

    float min_value = features[0];
    float max_value = features[0];
    double sum = 0.0;

    for (size_t i = 0; i < length; i++) {
        float value = features[i];
        if (value < min_value) {
            min_value = value;
        }
        if (value > max_value) {
            max_value = value;
        }
        sum += value;
    }

    const float avg_value = (float)(sum / (double)length);
    return avg_value >= 70.0f && (max_value - min_value) >= 20.0f;
}

static void dump_feature_pgm_once(const char *name, const float *features, size_t length)
{
#if DEBUG_DUMP_FIRST_FEATURE_PGM
    const size_t expected_length = (size_t)GESTURE_INPUT_WIDTH * (size_t)GESTURE_INPUT_HEIGHT;
    if (!features || length < expected_length) {
        return;
    }

    printf("FEATURE_PGM_BEGIN %s\r\n", name);
    printf("P2\r\n%d %d\r\n255\r\n", GESTURE_INPUT_WIDTH, GESTURE_INPUT_HEIGHT);
    for (int y = 0; y < GESTURE_INPUT_HEIGHT; y++) {
        for (int x = 0; x < GESTURE_INPUT_WIDTH; x++) {
            float value = features[(y * GESTURE_INPUT_WIDTH) + x];
            if (value < 0.0f) {
                value = 0.0f;
            }
            if (value > 255.0f) {
                value = 255.0f;
            }
            printf("%d", (int)(value + 0.5f));
            if (x + 1 < GESTURE_INPUT_WIDTH) {
                printf(" ");
            }
        }
        printf("\r\n");
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    printf("FEATURE_PGM_END %s\r\n", name);
#else
    (void)name;
    (void)features;
    (void)length;
#endif
}

#if DEBUG_MODEL_INPUT_HTTP
typedef struct {
    const uint8_t *image;
    const bool *ready;
    const char *name;
} debug_bmp_source_t;

static uint8_t clamp_feature_to_u8(float value)
{
    if (value < 0.0f) {
        return 0;
    }
    if (value > 255.0f) {
        return 255;
    }
    return (uint8_t)(value + 0.5f);
}

static bool update_debug_layout_image(const uint8_t *buf,
                                      size_t len,
                                      int src_width,
                                      int src_height,
                                      int src_stride,
                                      uint8_t *image)
{
    if (!buf || !image || src_width <= 0 || src_height <= 0 || src_stride < src_width) {
        return false;
    }

    const size_t y_plane_len = (size_t)src_stride * (size_t)src_height;
    if (len < y_plane_len) {
        return false;
    }

    resize_center_crop_y_to_u8(buf, src_width, src_height, src_stride, image);
    return true;
}

static void update_debug_layout_images(const uint8_t *buf, size_t len)
{
    if (!g_debug_image_mutex ||
        xSemaphoreTake(g_debug_image_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    g_debug_layout_ready[0] = update_debug_layout_image(buf, len, 800, 640, 800,
                                                       g_debug_layout_y_800x640_s800);
    g_debug_layout_ready[1] = update_debug_layout_image(buf, len, 800, 640, 960,
                                                       g_debug_layout_y_800x640_s960);
    g_debug_layout_ready[2] = update_debug_layout_image(buf, len, 800, 640, 1024,
                                                       g_debug_layout_y_800x640_s1024);
    g_debug_layout_ready[3] = update_debug_layout_image(buf, len, 640, 800, 640,
                                                       g_debug_layout_y_640x800_s640);
    g_debug_layout_ready[4] = update_debug_layout_image(buf, len, 640, 800, 800,
                                                       g_debug_layout_y_640x800_s800);
    const int packed_stride = 800 * 3 / 2;
    if (len >= ((size_t)packed_stride * 640U)) {
        resize_center_crop_o_uyy_e_vyy_to_u8(buf,
                                             800,
                                             640,
                                             packed_stride,
                                             g_debug_layout_o_uyy_e_vyy);
        g_debug_layout_ready[5] = true;
    } else {
        g_debug_layout_ready[5] = false;
    }

    xSemaphoreGive(g_debug_image_mutex);
}

static void update_debug_model_input_image(const float *features, size_t length)
{
    const size_t expected_length = (size_t)GESTURE_INPUT_WIDTH *
                                   (size_t)GESTURE_INPUT_HEIGHT;
    if (!features || length < expected_length || !g_debug_image_mutex) {
        return;
    }

    if (xSemaphoreTake(g_debug_image_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    for (size_t i = 0; i < expected_length; i++) {
        g_latest_model_input_u8[i] = clamp_feature_to_u8(features[i]);
    }
    g_latest_model_input_ready = true;

    xSemaphoreGive(g_debug_image_mutex);
}

static esp_err_t send_le16(httpd_req_t *req, uint16_t value)
{
    const uint8_t bytes[] = {
        (uint8_t)(value & 0xff),
        (uint8_t)((value >> 8) & 0xff),
    };
    return httpd_resp_send_chunk(req, (const char *)bytes, sizeof(bytes));
}

static esp_err_t send_le32(httpd_req_t *req, uint32_t value)
{
    const uint8_t bytes[] = {
        (uint8_t)(value & 0xff),
        (uint8_t)((value >> 8) & 0xff),
        (uint8_t)((value >> 16) & 0xff),
        (uint8_t)((value >> 24) & 0xff),
    };
    return httpd_resp_send_chunk(req, (const char *)bytes, sizeof(bytes));
}

static esp_err_t debug_bmp_handler(httpd_req_t *req)
{
    const debug_bmp_source_t *source = (const debug_bmp_source_t *)req->user_ctx;
    const int width = GESTURE_INPUT_WIDTH;
    const int height = GESTURE_INPUT_HEIGHT;
    const int row_bytes = width * 3;
    const uint32_t image_bytes = (uint32_t)(row_bytes * height);
    const uint32_t file_bytes = 54 + image_bytes;
    uint8_t row[row_bytes];

    if (!g_debug_image_mutex ||
        xSemaphoreTake(g_debug_image_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "image mutex unavailable");
        return ESP_FAIL;
    }

    if (!source || !source->image || !source->ready || !(*source->ready)) {
        xSemaphoreGive(g_debug_image_mutex);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "debug image is not ready yet");
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/bmp");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    httpd_resp_set_hdr(req, "X-Debug-Image", source->name ? source->name : "unknown");

    const uint8_t magic[] = {'B', 'M'};
    esp_err_t err = httpd_resp_send_chunk(req, (const char *)magic, sizeof(magic));
    if (err == ESP_OK) {
        err = send_le32(req, file_bytes);
    }
    if (err == ESP_OK) {
        err = send_le16(req, 0);
    }
    if (err == ESP_OK) {
        err = send_le16(req, 0);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 54);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 40);
    }
    if (err == ESP_OK) {
        err = send_le32(req, (uint32_t)width);
    }
    if (err == ESP_OK) {
        err = send_le32(req, (uint32_t)height);
    }
    if (err == ESP_OK) {
        err = send_le16(req, 1);
    }
    if (err == ESP_OK) {
        err = send_le16(req, 24);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 0);
    }
    if (err == ESP_OK) {
        err = send_le32(req, image_bytes);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 2835);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 2835);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 0);
    }
    if (err == ESP_OK) {
        err = send_le32(req, 0);
    }

    for (int y = height - 1; err == ESP_OK && y >= 0; y--) {
        for (int x = 0; x < width; x++) {
            uint8_t value = source->image[(y * width) + x];
            row[(x * 3) + 0] = value;
            row[(x * 3) + 1] = value;
            row[(x * 3) + 2] = value;
        }
        err = httpd_resp_send_chunk(req, (const char *)row, sizeof(row));
    }

    xSemaphoreGive(g_debug_image_mutex);

    if (err == ESP_OK) {
        err = httpd_resp_send_chunk(req, nullptr, 0);
    }
    return err;
}

static void start_debug_model_input_http_server()
{
    static const debug_bmp_source_t model_source = {
        .image = g_latest_model_input_u8,
        .ready = &g_latest_model_input_ready,
        .name = "model_input",
    };
    static const debug_bmp_source_t layout_sources[] = {
        {
            .image = g_debug_layout_y_800x640_s800,
            .ready = &g_debug_layout_ready[0],
            .name = "y_800x640_s800",
        },
        {
            .image = g_debug_layout_y_800x640_s960,
            .ready = &g_debug_layout_ready[1],
            .name = "y_800x640_s960",
        },
        {
            .image = g_debug_layout_y_800x640_s1024,
            .ready = &g_debug_layout_ready[2],
            .name = "y_800x640_s1024",
        },
        {
            .image = g_debug_layout_y_640x800_s640,
            .ready = &g_debug_layout_ready[3],
            .name = "y_640x800_s640",
        },
        {
            .image = g_debug_layout_y_640x800_s800,
            .ready = &g_debug_layout_ready[4],
            .name = "y_640x800_s800",
        },
        {
            .image = g_debug_layout_o_uyy_e_vyy,
            .ready = &g_debug_layout_ready[5],
            .name = "o_uyy_e_vyy",
        },
    };
    static const char *layout_uris[] = {
        "/debug_y_800x640_s800.bmp",
        "/debug_y_800x640_s960.bmp",
        "/debug_y_800x640_s1024.bmp",
        "/debug_y_640x800_s640.bmp",
        "/debug_y_640x800_s800.bmp",
        "/debug_o_uyy_e_vyy.bmp",
    };

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.ctrl_port = 32768;
    config.max_uri_handlers = 8;

    httpd_handle_t server = nullptr;
    esp_err_t err = httpd_start(&server, &config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "failed to start model input HTTP server: %s", esp_err_to_name(err));
        return;
    }

    httpd_uri_t input_image_uri = {
        .uri = "/ei_input.bmp",
        .method = HTTP_GET,
        .handler = debug_bmp_handler,
        .user_ctx = (void *)&model_source,
    };
    ESP_ERROR_CHECK(httpd_register_uri_handler(server, &input_image_uri));

    for (size_t i = 0; i < sizeof(layout_sources) / sizeof(layout_sources[0]); i++) {
        httpd_uri_t layout_uri = {
            .uri = layout_uris[i],
            .method = HTTP_GET,
            .handler = debug_bmp_handler,
            .user_ctx = (void *)&layout_sources[i],
        };
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &layout_uri));
    }

    ESP_LOGI(TAG, "model input image: http://%s.local/ei_input.bmp", EXAMPLE_MDNS_HOST_NAME);
    ESP_LOGI(TAG, "layout debug images: /debug_y_800x640_s800.bmp /debug_y_800x640_s960.bmp /debug_y_800x640_s1024.bmp /debug_y_640x800_s640.bmp /debug_y_640x800_s800.bmp /debug_o_uyy_e_vyy.bmp");
}
#endif

static void camera_raw_frame_cb(const uint8_t *buf,
                                size_t len,
                                int width,
                                int height,
                                int stride,
                                void *user_ctx)
{
#if ENABLE_GESTURE_INFERENCE
    (void)user_ctx;
    static bool raw_frame_info_logged = false;

    if (!buf || width <= 0 || height <= 0 ||
        !g_features_mutex || !g_frame_ready_sem) {
        return;
    }

    if (!raw_frame_info_logged) {
        ESP_LOGI(TAG,
                 "raw inference frame: width=%d height=%d stride=%d len=%u",
                 width,
                 height,
                 stride,
                 (unsigned)len);
        raw_frame_info_logged = true;
    }

    int64_t now_us = esp_timer_get_time();
    if ((now_us - g_last_capture_us) < (INFERENCE_INTERVAL_MS * 1000LL)) {
        return;
    }

    if (g_inference_pending) {
        return;
    }

    if (xSemaphoreTake(g_features_mutex, 0) != pdTRUE) {
        return;
    }

#if DEBUG_MODEL_INPUT_HTTP
    update_debug_layout_images(buf, len);
#endif

    // The ESP32-P4 video pipeline exposes YUV420 using the packed
    // O_UYY_E_VYY layout in both RTSP and standalone capture modes.
    const int packed_stride = (width * 3) / 2;
    const size_t packed_len = (size_t)packed_stride * (size_t)height;
    if (len < packed_len) {
        ESP_LOGW(TAG, "raw frame too short: len=%u expected at least %u for O_UYY_E_VYY",
                 (unsigned)len, (unsigned)packed_len);
        xSemaphoreGive(g_features_mutex);
        return;
    }
    resize_center_crop_o_uyy_e_vyy_to_features(buf,
                                               width,
                                               height,
                                               packed_stride,
                                               g_latest_features);
    g_inference_pending = true;
    g_last_capture_us = now_us;

    xSemaphoreGive(g_features_mutex);
    xSemaphoreGive(g_frame_ready_sem);
#else
    (void)buf;
    (void)len;
    (void)width;
    (void)height;
    (void)stride;
    (void)user_ctx;
#endif
}

#if ENABLE_GESTURE_INFERENCE
static void camera_capture_task(void *arg)
{
    camera_capture_t *camera = (camera_capture_t *)arg;

    while (true) {
        camera_frame_t frame = {};
        esp_err_t err = camera_capture_get_frame(camera, &frame);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera frame capture failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        camera_raw_frame_cb(frame.buf,
                            frame.len,
                            frame.width,
                            frame.height,
                            frame.width,
                            nullptr);

        err = camera_capture_return_frame(camera, &frame);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "camera frame return failed: %s", esp_err_to_name(err));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
#endif

#if ENABLE_GESTURE_INFERENCE
static uint8_t gesture_uart_crc8(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void gesture_uart_init()
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = GESTURE_UART_BAUD_RATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.rx_flow_ctrl_thresh = 0;
    uart_config.source_clk = UART_SCLK_DEFAULT;

    ESP_ERROR_CHECK(uart_driver_install(GESTURE_UART_NUM,
                                        GESTURE_UART_RX_BUFFER_SIZE,
                                        0,
                                        0,
                                        nullptr,
                                        0));
    ESP_ERROR_CHECK(uart_param_config(GESTURE_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GESTURE_UART_NUM,
                                 GESTURE_UART_TX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG,
             "gesture UART enabled: UART%d TX=GPIO%d baud=%d 8N1",
             (int)GESTURE_UART_NUM,
             (int)GESTURE_UART_TX_GPIO,
             GESTURE_UART_BAUD_RATE);
}

static float gesture_confidence_threshold_for_id(int gesture_id)
{
    if (gesture_id == GESTURE_ID_A_LITTLE) {
        return GESTURE_A_LITTLE_CONFIDENCE_THRESHOLD;
    }
    if (gesture_id == GESTURE_ID_YOU) {
        return GESTURE_YOU_CONFIDENCE_THRESHOLD;
    }
    if (gesture_id == GESTURE_ID_GOOD) {
        return GESTURE_GOOD_CONFIDENCE_THRESHOLD;
    }
    if (gesture_id == GESTURE_ID_HELP ||
        gesture_id == GESTURE_ID_SIGN_LANGUAGE) {
        return GESTURE_MID_CONFIDENCE_THRESHOLD;
    }
    return GESTURE_DEFAULT_CONFIDENCE_THRESHOLD;
}

static void send_gesture_uart_frame(int gesture_id, float confidence)
{
    if (gesture_id < 0 || gesture_id >= GESTURE_LABEL_COUNT) {
        return;
    }

    int confidence_percent = (int)lroundf(confidence * 100.0f);
    confidence_percent = std::clamp(confidence_percent, 0, 100);

    const float threshold = gesture_confidence_threshold_for_id(gesture_id);
    uint8_t status = 0;
    if (confidence >= threshold) {
        status |= GESTURE_UART_STATUS_VALID;
    } else {
        status |= GESTURE_UART_STATUS_LOW_CONFIDENCE;
    }

    uint8_t frame[9] = {
        GESTURE_UART_FRAME_HEAD_0,
        GESTURE_UART_FRAME_HEAD_1,
        GESTURE_UART_PROTOCOL_VERSION,
        GESTURE_UART_FRAME_TYPE_RESULT,
        g_gesture_uart_seq++,
        (uint8_t)gesture_id,
        (uint8_t)confidence_percent,
        status,
        0,
    };
    frame[8] = gesture_uart_crc8(&frame[2], 6);

    const int written = uart_write_bytes(GESTURE_UART_NUM,
                                         (const char *)frame,
                                         sizeof(frame));
    if (written != (int)sizeof(frame)) {
        ESP_LOGW(TAG, "gesture UART write incomplete: %d/%u", written, (unsigned)sizeof(frame));
    }
}

static int8_t quantize_model_input_pixel(float gray)
{
    if (gray < 0.0f) {
        gray = 0.0f;
    }
    if (gray > 255.0f) {
        gray = 255.0f;
    }

    const float normalized = (gray * (2.0f / 255.0f)) - 1.0f;
    int value = (int)lroundf(normalized * (float)(1 << -GESTURE_MODEL_INPUT_EXPONENT));
    if (value < -128) {
        value = -128;
    }
    if (value > 127) {
        value = 127;
    }
    return (int8_t)value;
}

static bool fill_model_input(dl::TensorBase *input, const float *features, size_t length)
{
    if (!input || !features || length < GESTURE_INPUT_FRAME_SIZE) {
        return false;
    }

    if (input->get_dtype() != dl::DATA_TYPE_INT8 || input->get_size() < (int)GESTURE_INPUT_FRAME_SIZE) {
        ESP_LOGE(TAG,
                 "unexpected ESP-DL input tensor: dtype=%s size=%d exponent=%d",
                 input->get_dtype_string(),
                 input->get_size(),
                 input->get_exponent());
        return false;
    }

    if (input->get_exponent() != GESTURE_MODEL_INPUT_EXPONENT) {
        ESP_LOGW(TAG,
                 "input exponent is %d, expected %d",
                 input->get_exponent(),
                 GESTURE_MODEL_INPUT_EXPONENT);
    }

    int8_t *input_data = input->get_element_ptr<int8_t>();
    for (size_t i = 0; i < GESTURE_INPUT_FRAME_SIZE; i++) {
        input_data[i] = quantize_model_input_pixel(features[i]);
    }
    return true;
}

static void softmax_logits(const float *logits, float *scores, int count)
{
    float max_value = logits[0];
    for (int i = 1; i < count; i++) {
        if (logits[i] > max_value) {
            max_value = logits[i];
        }
    }

    float sum = 0.0f;
    for (int i = 0; i < count; i++) {
        scores[i] = expf(logits[i] - max_value);
        sum += scores[i];
    }

    if (sum <= 0.0f) {
        return;
    }

    for (int i = 0; i < count; i++) {
        scores[i] /= sum;
    }
}

static bool read_model_scores(dl::TensorBase *output, float *scores, int count)
{
    if (!output || !scores || count <= 0) {
        return false;
    }

    if (output->get_dtype() != dl::DATA_TYPE_INT8 || output->get_size() < count) {
        ESP_LOGE(TAG,
                 "unexpected ESP-DL output tensor: dtype=%s size=%d exponent=%d",
                 output->get_dtype_string(),
                 output->get_size(),
                 output->get_exponent());
        return false;
    }

    const float scale = DL_SCALE(output->get_exponent());
    const int8_t *output_data = output->get_element_ptr<int8_t>();
    float logits[GESTURE_LABEL_COUNT] = {};
    for (int i = 0; i < count; i++) {
        logits[i] = (float)output_data[i] * scale;
    }
    softmax_logits(logits, scores, count);
    return true;
}

static int print_gesture_scores(const float *scores, int count)
{
    int top_index = 0;
    float top_value = scores[0];

    printf("Classification scores:\r\n");
    for (int i = 0; i < count; i++) {
        if (scores[i] > top_value) {
            top_value = scores[i];
            top_index = i;
        }
        printf("  %s: %.4f\r\n", GESTURE_LABELS[i], scores[i]);
    }

    printf("Top result: %s (%.4f)\r\n", GESTURE_LABELS[top_index], top_value);
    return top_index;
}

static void log_model_tensor_info(dl::Model *model)
{
    if (!model) {
        return;
    }

    dl::TensorBase *input = model->get_input();
    dl::TensorBase *output = model->get_output();
    if (input) {
        ESP_LOGI(TAG,
                 "ESP-DL input: shape=%s dtype=%s exponent=%d size=%d",
                 dl::vector_to_string(input->get_shape()).c_str(),
                 input->get_dtype_string(),
                 input->get_exponent(),
                 input->get_size());
    }
    if (output) {
        ESP_LOGI(TAG,
                 "ESP-DL output: shape=%s dtype=%s exponent=%d size=%d",
                 dl::vector_to_string(output->get_shape()).c_str(),
                 output->get_dtype_string(),
                 output->get_exponent(),
                 output->get_size());
    }
}

static void inference_task(void *arg)
{
    (void)arg;
    bool feature_pgm_dumped = false;

    ESP_LOGI(TAG,
             "loading ESP-DL model from rodata, size=%u bytes",
             (unsigned)(gesture_cnn_224_gray_esp32p4_espdl_end -
                        gesture_cnn_224_gray_esp32p4_espdl_start));
    dl::Model model((const char *)gesture_cnn_224_gray_esp32p4_espdl_start,
                    fbs::MODEL_LOCATION_IN_FLASH_RODATA);
    log_model_tensor_info(&model);

    dl::TensorBase *model_input = model.get_input();
    dl::TensorBase *model_output = model.get_output();
    if (!model_input || !model_output) {
        ESP_LOGE(TAG, "ESP-DL model input/output unavailable");
        vTaskDelete(nullptr);
        return;
    }

    while (true) {
        if (xSemaphoreTake(g_frame_ready_sem, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        if (xSemaphoreTake(g_features_mutex, portMAX_DELAY) == pdTRUE) {
            memcpy(g_base_features,
                   g_latest_features,
                   GESTURE_INPUT_FRAME_SIZE * sizeof(float));
            g_inference_pending = false;
            xSemaphoreGive(g_features_mutex);
        }

        prepare_features_full_range_y(g_base_features,
                                      g_inference_features,
                                      GESTURE_INPUT_FRAME_SIZE);
        print_feature_stats(g_inference_features, GESTURE_INPUT_FRAME_SIZE);
#if DEBUG_MODEL_INPUT_HTTP
        update_debug_model_input_image(g_inference_features, GESTURE_INPUT_FRAME_SIZE);
#endif
        if (!feature_pgm_dumped &&
            feature_frame_is_valid_for_dump(g_base_features, GESTURE_INPUT_FRAME_SIZE)) {
            dump_feature_pgm_once("full_range_y",
                                  g_inference_features,
                                  GESTURE_INPUT_FRAME_SIZE);
            feature_pgm_dumped = true;
        }

        if (!fill_model_input(model_input, g_inference_features, GESTURE_INPUT_FRAME_SIZE)) {
            vTaskDelay(pdMS_TO_TICKS(INFERENCE_IDLE_DELAY_MS));
            continue;
        }

        const int64_t inference_start_us = esp_timer_get_time();
        model.run(dl::RUNTIME_MODE_MULTI_CORE);
        const int64_t inference_time_us = esp_timer_get_time() - inference_start_us;

        float scores[GESTURE_LABEL_COUNT] = {};
        if (!read_model_scores(model_output, scores, GESTURE_LABEL_COUNT)) {
            vTaskDelay(pdMS_TO_TICKS(INFERENCE_IDLE_DELAY_MS));
            continue;
        }

        printf("Timing: DSP 0 ms, inference %d ms, anomaly 0 ms\r\n",
               (int)((inference_time_us + 500) / 1000));
        const int top_index = print_gesture_scores(scores, GESTURE_LABEL_COUNT);
        send_gesture_uart_frame(top_index, scores[top_index]);
        vTaskDelay(pdMS_TO_TICKS(INFERENCE_IDLE_DELAY_MS));
    }
}
#endif

extern "C" int app_main()
{
    printf("ESP32-P4 standalone camera + ESP-DL gesture inferencing\r\n");
    printf("Model input: %dx%d grayscale, labels=%d\r\n",
           GESTURE_INPUT_WIDTH,
           GESTURE_INPUT_HEIGHT,
           GESTURE_LABEL_COUNT);

#if ENABLE_GESTURE_INFERENCE
    if (!allocate_feature_buffers()) {
        return 1;
    }
    g_features_mutex = xSemaphoreCreateMutex();
    g_frame_ready_sem = xSemaphoreCreateBinary();
#if DEBUG_MODEL_INPUT_HTTP
    g_debug_image_mutex = xSemaphoreCreateMutex();
#endif
    if (!g_features_mutex || !g_frame_ready_sem
#if DEBUG_MODEL_INPUT_HTTP
        || !g_debug_image_mutex
#endif
    ) {
        ESP_LOGE(TAG, "failed to create inference synchronization objects");
        return 1;
    }
    gesture_uart_init();
#else
    printf("Gesture inference disabled for dataset capture\r\n");
#endif

#if ENABLE_RTSP_PREVIEW
    net_connect();

#if DEBUG_MODEL_INPUT_HTTP
    start_debug_model_input_http_server();
#endif

    ESP_LOGI(TAG, "initialize camera/video devices");
    video_dev_init(&g_camera_context);
#if ENABLE_GESTURE_INFERENCE
    g_camera_context.raw_frame_cb = camera_raw_frame_cb;
    g_camera_context.raw_frame_user_ctx = nullptr;

    ESP_LOGI(TAG, "start inference task");
    xTaskCreatePinnedToCore(inference_task,
                            "gesture_infer",
                            INFERENCE_TASK_STACK_SIZE,
                            nullptr,
                            INFERENCE_TASK_PRIORITY,
                            nullptr,
                            1);
#else
    g_camera_context.raw_frame_cb = nullptr;
    g_camera_context.raw_frame_user_ctx = nullptr;
#endif

    ESP_LOGI(TAG, "start RTSP service");
    g_rtsp = rtsp_service_start(&g_camera_context);

    printf("Preview with VLC/ffplay: rtsp://esp32p4-rtsp.local:%d\r\n", RTSP_SERVER_PORT);
    printf("Browser via webrtc-streamer: http://127.0.0.1:8000/webrtcstreamer.html?video=rtsp://esp32p4-rtsp.local:%d\r\n",
           RTSP_SERVER_PORT);
    printf("Dataset capture example: ffmpeg -i rtsp://192.168.137.2:%d -vf fps=1 D:\\camera_dataset\\raw\\frame_%%05d.jpg\r\n",
           RTSP_SERVER_PORT);
#else
    ESP_LOGI(TAG, "initialize standalone camera capture");
    ESP_ERROR_CHECK(camera_capture_init(&g_camera_capture,
                                        CAMERA_CAPTURE_WIDTH,
                                        CAMERA_CAPTURE_HEIGHT));

#if ENABLE_GESTURE_INFERENCE
    ESP_LOGI(TAG, "start inference task");
    xTaskCreatePinnedToCore(inference_task,
                            "gesture_infer",
                            INFERENCE_TASK_STACK_SIZE,
                            nullptr,
                            INFERENCE_TASK_PRIORITY,
                            nullptr,
                            1);

    ESP_LOGI(TAG, "start standalone camera capture task");
    xTaskCreatePinnedToCore(camera_capture_task,
                            "camera_capture",
                            CAMERA_CAPTURE_TASK_STACK_SIZE,
                            &g_camera_capture,
                            CAMERA_CAPTURE_TASK_PRIORITY,
                            nullptr,
                            0);
#endif

    printf("Standalone mode: Ethernet/RTSP disabled, camera starts automatically.\r\n");
#endif

    return 0;
}
