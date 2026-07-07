#pragma once
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Create speech recognition + environment sound UI on the given parent.
 *
 * Displays speech status label, DOA direction indicator,
 * command result label, and environment sound alert label.
 * Polls g_sr_queue and g_ei_queue via LVGL timer.
 *
 * @param parent  Parent LVGL object (typically lv_scr_act()).
 * @return        The container object.
 */
lv_obj_t *ui_speech_create(lv_obj_t *parent);

#ifdef __cplusplus
}
#endif
