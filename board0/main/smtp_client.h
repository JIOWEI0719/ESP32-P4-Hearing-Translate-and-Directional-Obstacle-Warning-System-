/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Send an email via SMTP (blocking, runs in its own task).
 *
 * Uses hardcoded SMTP credentials (smtp.qq.com:465).
 * This function should be called from a FreeRTOS task with sufficient stack (>= 8KB).
 *
 * @return ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t smtp_send_email(void);

#ifdef __cplusplus
}
#endif
