#include "camera_capture.h"

#include <assert.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "esp_cache.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"
#include "sdkconfig.h"

#define CAM_DEV_PATH ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define CAMERA_CAPTURE_FORMAT V4L2_PIX_FMT_YUV420

static const char *TAG = "camera_capture";

static const esp_video_init_csi_config_t csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_PORT,
                .scl_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SCL_PIN,
                .sda_pin = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_SDA_PIN,
            },
            .freq = CONFIG_EXAMPLE_MIPI_CSI_SCCB_I2C_FREQ,
        },
        .reset_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_RESET_PIN,
        .pwdn_pin = CONFIG_EXAMPLE_MIPI_CSI_CAM_SENSOR_PWDN_PIN,
    },
};

static const esp_video_init_config_t camera_config = {
    .csi = csi_config,
};

static void print_video_device_info(const struct v4l2_capability *capability)
{
    ESP_LOGI(TAG, "version: %d.%d.%d",
             (uint16_t)(capability->version >> 16),
             (uint8_t)(capability->version >> 8),
             (uint8_t)capability->version);
    ESP_LOGI(TAG, "driver: %s", capability->driver);
    ESP_LOGI(TAG, "card: %s", capability->card);
    ESP_LOGI(TAG, "bus: %s", capability->bus_info);
}

esp_err_t camera_capture_init(camera_capture_t *camera, int width, int height)
{
    if (!camera) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(camera, 0, sizeof(*camera));
    camera->fd = -1;

    ESP_RETURN_ON_ERROR(esp_video_init(&camera_config), TAG, "esp_video_init failed");

    int fd = open(CAM_DEV_PATH, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open %s", CAM_DEV_PATH);
        return ESP_FAIL;
    }

    struct v4l2_capability capability = {0};
    if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QUERYCAP failed");
        close(fd);
        return ESP_FAIL;
    }
    print_video_device_info(&capability);

    struct v4l2_format format = {0};
    format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    format.fmt.pix.width = width;
    format.fmt.pix.height = height;
    format.fmt.pix.pixelformat = CAMERA_CAPTURE_FORMAT;
    if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT %dx%d YUV420 failed", width, height);
        close(fd);
        return ESP_FAIL;
    }

    struct v4l2_requestbuffers req = {0};
    req.count = CAMERA_CAPTURE_BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        close(fd);
        return ESP_FAIL;
    }

    for (int i = 0; i < CAMERA_CAPTURE_BUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QUERYBUF %d failed", i);
            close(fd);
            return ESP_FAIL;
        }

        camera->buffers[i] = (uint8_t *)mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                             MAP_SHARED, fd, buf.m.offset);
        if (!camera->buffers[i]) {
            ESP_LOGE(TAG, "mmap %d failed", i);
            close(fd);
            return ESP_FAIL;
        }
        camera->buffer_lengths[i] = buf.length;

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF %d failed", i);
            close(fd);
            return ESP_FAIL;
        }
    }

    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        close(fd);
        return ESP_FAIL;
    }

    camera->fd = fd;
    camera->width = format.fmt.pix.width;
    camera->height = format.fmt.pix.height;
    camera->streaming = true;

    ESP_LOGI(TAG, "camera capture started: %dx%d YUV420", camera->width, camera->height);
    return ESP_OK;
}

esp_err_t camera_capture_get_frame(camera_capture_t *camera, camera_frame_t *frame)
{
    if (!camera || !frame || camera->fd < 0 || !camera->streaming) {
        return ESP_ERR_INVALID_ARG;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(camera->fd, VIDIOC_DQBUF, &buf) != 0) {
        ESP_LOGE(TAG, "VIDIOC_DQBUF failed");
        return ESP_FAIL;
    }

    esp_err_t cache_ret = esp_cache_msync(camera->buffers[buf.index],
                                          buf.bytesused,
                                          ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (cache_ret != ESP_OK) {
        ESP_LOGW(TAG, "capture buffer cache sync failed: %s", esp_err_to_name(cache_ret));
    }

    frame->buf = camera->buffers[buf.index];
    frame->len = buf.bytesused;
    frame->width = camera->width;
    frame->height = camera->height;
    frame->index = buf.index;
    return ESP_OK;
}

esp_err_t camera_capture_return_frame(camera_capture_t *camera, const camera_frame_t *frame)
{
    if (!camera || !frame || camera->fd < 0 || frame->index < 0 ||
        frame->index >= CAMERA_CAPTURE_BUFFER_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }

    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = frame->index;

    if (ioctl(camera->fd, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QBUF return failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

void camera_capture_deinit(camera_capture_t *camera)
{
    if (!camera || camera->fd < 0) {
        return;
    }

    if (camera->streaming) {
        int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        ioctl(camera->fd, VIDIOC_STREAMOFF, &type);
        camera->streaming = false;
    }

    for (int i = 0; i < CAMERA_CAPTURE_BUFFER_COUNT; i++) {
        if (camera->buffers[i]) {
            munmap(camera->buffers[i], camera->buffer_lengths[i]);
            camera->buffers[i] = NULL;
            camera->buffer_lengths[i] = 0;
        }
    }

    close(camera->fd);
    camera->fd = -1;
}
