#ifndef CAMERA_CAPTURE_H
#define CAMERA_CAPTURE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_CAPTURE_WIDTH 800
#define CAMERA_CAPTURE_HEIGHT 800
#define CAMERA_CAPTURE_BUFFER_COUNT 2

typedef struct {
    uint8_t *buf;
    size_t len;
    int width;
    int height;
    int index;
} camera_frame_t;

typedef struct {
    int fd;
    uint8_t *buffers[CAMERA_CAPTURE_BUFFER_COUNT];
    size_t buffer_lengths[CAMERA_CAPTURE_BUFFER_COUNT];
    int width;
    int height;
    bool streaming;
} camera_capture_t;

esp_err_t camera_capture_init(camera_capture_t *camera, int width, int height);
esp_err_t camera_capture_get_frame(camera_capture_t *camera, camera_frame_t *frame);
esp_err_t camera_capture_return_frame(camera_capture_t *camera, const camera_frame_t *frame);
void camera_capture_deinit(camera_capture_t *camera);

#ifdef __cplusplus
}
#endif

#endif
