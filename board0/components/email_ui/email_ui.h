/*
 * SPDX-FileCopyrightText: 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create the email send button UI on the given parent.
 *
 * Displays the emailButton0.png image as a clickable button
 * and a status label below it.
 *
 * @param parent  Parent LVGL object (typically lv_scr_act()).
 * @return        The container object.
 */
lv_obj_t *email_ui_create(lv_obj_t *parent);

#ifdef __cplusplus
}
#endif
