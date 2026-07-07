#ifndef VIDEO_DEV_H
#define VIDEO_DEV_H

#include <esp_cam_sensor_types.h>
#include <stddef.h>
#include <stdint.h>
#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C" {
#endif
// csi摄像头和h264 codec
#define CAM_DEV_PATH        ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define ENCODE_DEV_PATH     ESP_VIDEO_H264_DEVICE_NAME

#define EXAMPLE_VIDEO_BUFFER_COUNT   2
#define BUFFER_COUNT        EXAMPLE_VIDEO_BUFFER_COUNT

typedef struct {
    uint8_t *buf;
    size_t len;
    size_t width;
    size_t height;
    struct timeval timestamp;
} frame_buffer_t;

typedef void (*camera_raw_frame_cb_t)(const uint8_t *buf,
                                      size_t len,
                                      int width,
                                      int height,
                                      int stride,
                                      void *user_ctx);

typedef struct {
    int cap_fd;
    uint32_t format;
    uint8_t *cap_buffer[BUFFER_COUNT];
    int m2m_fd;
    uint8_t *m2m_cap_buffer;
    frame_buffer_t fb;
    int width;
    int height;
    int capture_stride;
    int capture_sizeimage;
    camera_raw_frame_cb_t raw_frame_cb;
    void *raw_frame_user_ctx;
} camera_context;

// 摄像头初始化
int video_dev_init(camera_context *context);


esp_err_t video_start(int width, int height, camera_context *cb_ctx);

IRAM_ATTR frame_buffer_t *video_fb_get(camera_context *cb_ctx);

/**
 *
 * 源于dvp摄像头需要归还buffer兼容写法，这里调用拍照后调用，但实际上不用归还，故没有传frame_buffer_t参数
 * @param cb_ctx 摄像头上下文
 */
void video_after_take(const camera_context *cb_ctx);

void video_stop(camera_context *cb_ctx);

void enumerate_camera_capabilities(int fd);

#ifdef __cplusplus
}
#endif

#endif
