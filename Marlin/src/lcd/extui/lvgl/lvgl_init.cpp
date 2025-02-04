/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(HAS_LVGL)

#include "lvgl_init.h"
#include "lv_drv_conf.h"

#include <lvgl.h>

#ifndef LVGL_H_BUFFER_LINES
  #define LVGL_H_BUFFER_LINES 10 // min 10
#endif

// Reserve LVGL display buffer
static lv_color_t primary_display_buffer[LV_HOR_RES_MAX * LVGL_H_BUFFER_LINES];
#if ENABLED(LVGL_DOUBLE_BUFFERING)
  static lv_color_t secondary_display_buffer[LV_HOR_RES_MAX * LVGL_H_BUFFER_LINES];
#endif

static lv_disp_buf_t lv_disp_buf;

void lvgl_init() {
  lv_init();

  lv_disp_buf_init(&lv_disp_buf, primary_display_buffer, TERN(LVGL_DOUBLE_BUFFERING, secondary_display_buffer, nullptr), LV_HOR_RES_MAX * LVGL_H_BUFFER_LINES);

  // Init LVGL display driver
  DISPLAY_INIT();
  lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.flush_cb = DISPLAY_FLUSH;
  disp_drv.buffer = &lv_disp_buf;
  lv_disp_drv_register(&disp_drv);

  // Init LVGL touch input driver
  #ifdef POINTER_READ
    #ifdef POINTER_INIT
      POINTER_INIT();
    #endif
    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = POINTER_READ;
    lv_indev_drv_register(&indev_drv);
  #endif

  // Init LVGL rotary input driver
  #ifdef ROTARY_READ
    #ifdef ROTARY_INIT
      ROTARY_INIT();
    #endif
    lv_indev_drv_t enc_drv;
    lv_indev_drv_init(&enc_drv);
    enc_drv.type = LV_INDEV_TYPE_ENCODER;
    enc_drv.read_cb = ROTARY_READ;
    lv_indev_drv_register(&enc_drv);
  #endif

  // Init LVGL keyboard input driver
  #ifdef KEYBOARD_READ
    #ifdef KEYBOARD_INIT
      KEYBOARD_INIT();
    #endif
    lv_indev_drv_t kbd_drv;
    lv_indev_drv_init(&kbd_drv);
    kbd_drv.type = LV_INDEV_TYPE_KEYPAD;
    kbd_drv.read_cb = KEYBOARD_READ;
    lv_indev_drv_register(&kbd_drv);
  #endif
}

#endif // HAS_LVGL
