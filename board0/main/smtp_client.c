/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "smtp_client.h"
#include "audio_sr.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_tls.h"
#include "esp_timer.h"
#include "esp_crt_bundle.h"
#include "mbedtls/base64.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "smtp";

/* ========== Hardcoded SMTP Credentials ========== */
#define SMTP_SERVER        "smtp.qq.com"
#define SMTP_PORT          465
#define SMTP_SENDER_EMAIL  "yourmail@qq.com"
#define SMTP_SENDER_AUTH   "yourauthcode"
#define SMTP_RCPT_EMAIL    "recipient@qq.com"
#define SMTP_SUBJECT       "Hello from ESP32-P4"
#define SMTP_BODY          "I need your help! This email was sent from ESP32-P4 by clicking the button."

/* ========== I/O Helpers ========== */

static esp_err_t smtp_read_line(esp_tls_t *tls, char *buf, size_t buf_size, int timeout_ms)
{
    size_t len = 0;
    int64_t deadline = esp_timer_get_time() + timeout_ms * 1000LL;

    while (len < buf_size - 1) {
        size_t bytes_read = 0;
        int ret = esp_tls_conn_read(tls, buf + len, 1);
        if (ret == 0) {
            continue; /* no data yet */
        } else if (ret < 0) {
            ESP_LOGE(TAG, "TLS read error: %d", ret);
            return ESP_FAIL;
        }
        bytes_read = (size_t)ret;
        len += bytes_read;
        if (buf[len - 1] == '\n') {
            break;
        }
        /* Check timeout */
        if (esp_timer_get_time() > deadline) {
            ESP_LOGE(TAG, "SMTP read timeout");
            return ESP_ERR_TIMEOUT;
        }
    }
    buf[len] = '\0';
    ESP_LOGD(TAG, "S: %s", buf);
    return ESP_OK;
}

static esp_err_t smtp_write(esp_tls_t *tls, const char *str)
{
    size_t len = strlen(str);
    ESP_LOGD(TAG, "C: %s", str);
    int ret = esp_tls_conn_write(tls, str, len);
    if (ret < 0) {
        ESP_LOGE(TAG, "TLS write error: %d", ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static int smtp_get_response_code(const char *response)
{
    int code = 0;
    if (strlen(response) >= 3) {
        code = (response[0] - '0') * 100 + (response[1] - '0') * 10 + (response[2] - '0');
    }
    return code;
}

/* Multi-line SMTP responses end with a line where the 4th char is ' ' not '-' */
static esp_err_t smtp_read_full_response(esp_tls_t *tls, char *last_line, size_t buf_size)
{
    char line[256];
    int retry = 0;
    do {
        esp_err_t err = smtp_read_line(tls, line, sizeof(line), 10000);
        if (err != ESP_OK) return err;
        strncpy(last_line, line, buf_size - 1);
        last_line[buf_size - 1] = '\0';
        retry++;
    } while (strlen(line) >= 4 && line[3] == '-' && retry < 20);
    return ESP_OK;
}

/* ========== Base64 Helpers ========== */

static esp_err_t base64_encode_str(const char *input, char **output_out, size_t *out_len)
{
    size_t input_len = strlen(input);
    size_t olen = 0;
    mbedtls_base64_encode(NULL, 0, &olen, (const unsigned char *)input, input_len);
    *output_out = malloc(olen + 1);
    if (!*output_out) return ESP_ERR_NO_MEM;
    int ret = mbedtls_base64_encode((unsigned char *)*output_out, olen + 1, &olen,
                                     (const unsigned char *)input, input_len);
    if (ret != 0) {
        free(*output_out);
        return ESP_FAIL;
    }
    (*output_out)[olen] = '\0';
    *out_len = olen;
    return ESP_OK;
}

/* ========== Main Send Function ========== */

esp_err_t smtp_send_email(void)
{
    esp_err_t ret = ESP_FAIL;
    esp_tls_t *tls = NULL;
    char response[256];
    char *b64 = NULL;
    size_t b64_len = 0;
    int code;

    /* Step 0: Pause audio_sr to free CPU + memory for TLS handshake */
    ESP_LOGI(TAG, "Pausing audio_sr before TLS...");
    audio_sr_suspend();

    /* Log free heap before TLS */
    ESP_LOGI(TAG, "Free heap: internal=%u PSRAM=%u",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    /* Step 1: TLS connection to SMTP server */
    esp_tls_cfg_t tls_cfg = {
        .is_plain_tcp = false,
        .timeout_ms = 15000,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .skip_common_name = true,
    };
    ESP_LOGI(TAG, "Connecting to %s:%d...", SMTP_SERVER, SMTP_PORT);
    tls = esp_tls_init();
    if (!tls) {
        ESP_LOGE(TAG, "esp_tls_init failed");
        goto cleanup;
    }
    int rc = esp_tls_conn_new_sync(SMTP_SERVER, strlen(SMTP_SERVER), SMTP_PORT, &tls_cfg, tls);
    if (rc != 1) {
        ESP_LOGE(TAG, "TLS connection failed (rc=%d). Check WiFi and server address.", rc);
        esp_tls_conn_destroy(tls);
        tls = NULL;
        goto cleanup;
    }
    ESP_LOGI(TAG, "TLS connected");

    /* Step 2: Read server greeting (220) */
    ret = smtp_read_full_response(tls, response, sizeof(response));
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 220) { ESP_LOGE(TAG, "Expected 220, got %d", code); goto cleanup; }

    /* Step 3: EHLO */
    ret = smtp_write(tls, "EHLO esp32\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_full_response(tls, response, sizeof(response));
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 250) { ESP_LOGE(TAG, "EHLO expected 250, got %d", code); goto cleanup; }

    /* Step 4: AUTH LOGIN */
    ret = smtp_write(tls, "AUTH LOGIN\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 334) { ESP_LOGE(TAG, "AUTH LOGIN expected 334, got %d", code); goto cleanup; }

    /* Step 5: Send base64 username */
    ret = base64_encode_str(SMTP_SENDER_EMAIL, &b64, &b64_len);
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_write(tls, b64);
    free(b64); b64 = NULL;
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_write(tls, "\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 334) { ESP_LOGE(TAG, "Username expected 334, got %d", code); goto cleanup; }

    /* Step 6: Send base64 auth code */
    ret = base64_encode_str(SMTP_SENDER_AUTH, &b64, &b64_len);
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_write(tls, b64);
    free(b64); b64 = NULL;
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_write(tls, "\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 235) { ESP_LOGE(TAG, "Auth code expected 235, got %d", code); goto cleanup; }

    ESP_LOGI(TAG, "Authentication OK");

    /* Step 7: MAIL FROM */
    ret = smtp_write(tls, "MAIL FROM: <" SMTP_SENDER_EMAIL ">\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 250) { ESP_LOGE(TAG, "MAIL FROM expected 250, got %d", code); goto cleanup; }

    /* Step 8: RCPT TO */
    ret = smtp_write(tls, "RCPT TO: <" SMTP_RCPT_EMAIL ">\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 250) { ESP_LOGE(TAG, "RCPT TO expected 250, got %d", code); goto cleanup; }

    /* Step 9: DATA */
    ret = smtp_write(tls, "DATA\r\n");
    if (ret != ESP_OK) goto cleanup;
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 354) { ESP_LOGE(TAG, "DATA expected 354, got %d", code); goto cleanup; }

    /* Step 10: Send email content */
    {
        char body_buf[1024];
        snprintf(body_buf, sizeof(body_buf),
                 "From: <%s>\r\n"
                 "To: <%s>\r\n"
                 "Subject: %s\r\n"
                 "Content-Type: text/plain; charset=UTF-8\r\n"
                 "\r\n"
                 "%s\r\n"
                 ".\r\n",
                 SMTP_SENDER_EMAIL, SMTP_RCPT_EMAIL, SMTP_SUBJECT, SMTP_BODY);
        ESP_LOGI(TAG, "Sending email body...");
        ret = smtp_write(tls, body_buf);
        if (ret != ESP_OK) goto cleanup;
    }
    ret = smtp_read_line(tls, response, sizeof(response), 10000);
    if (ret != ESP_OK) goto cleanup;
    code = smtp_get_response_code(response);
    if (code != 250) { ESP_LOGE(TAG, "Send body expected 250, got %d", code); goto cleanup; }

    ESP_LOGI(TAG, "Email sent successfully!");

    /* Step 11: QUIT */
    smtp_write(tls, "QUIT\r\n");
    ret = ESP_OK;

cleanup:
    /* Resume audio_sr regardless of outcome */
    audio_sr_resume();
    ESP_LOGI(TAG, "audio_sr resumed");

    if (b64) free(b64);
    if (tls) esp_tls_conn_destroy(tls);
    return ret;
}
