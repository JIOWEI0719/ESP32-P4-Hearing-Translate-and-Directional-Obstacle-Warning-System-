#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 手势事件（仅当有效识别且置信度足够时触发）
 */
typedef struct {
    uint8_t gesture_id;   /**< 手势 ID (0~6) */
    uint8_t confidence;   /**< 置信度 (0~100, 对应 0.00~1.00) */
    uint8_t seq;          /**< 帧序号 */
    uint8_t status;       /**< 原始状态字节 (bit0=有效, bit1=低置信度, bit2=保持中) */
} uart_gesture_event_t;

/**
 * @brief 手势事件回调函数类型
 *
 * @param event     解析到的手势事件
 * @param user_data 用户注册时传入的上下文指针
 */
typedef void (*uart_gesture_cb_t)(const uart_gesture_event_t *event, void *user_data);

/**
 * @brief 初始化 UART 手势接收器
 *
 * 配置 UART0 (GPIO38=RX, GPIO37=TX)，启动 FreeRTOS 接收任务，
 * 开始解析 AA 55 协议帧。解析到有效手势后通过回调通知。
 *
 * @param baud_rate     波特率 (如 115200)
 * @param callback      有效手势回调 (bit0=1 且 bit1=0)
 * @param user_data     回调透传参数 (可为 NULL)
 * @return ESP_OK 成功
 */
esp_err_t uart_gesture_init(int baud_rate, uart_gesture_cb_t callback, void *user_data);

/**
 * @brief 设置去抖间隔
 *
 * 同一手势 ID 在去抖间隔内不会重复触发回调。
 *
 * @param sec  秒数 (默认 2.0f)
 */
void uart_gesture_set_debounce(float sec);

#ifdef __cplusplus
}
#endif
