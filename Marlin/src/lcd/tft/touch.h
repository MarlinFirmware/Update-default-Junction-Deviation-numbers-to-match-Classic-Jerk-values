/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#pragma once

#include "../../inc/MarlinConfigPre.h"
#include "../tft_io/touch_calibration.h"

#if ENABLED(TFT_TOUCH_DEVICE_GT911)
  #include HAL_PATH(../.., tft/gt911.h)
  #define TOUCH_DRIVER_CLASS GT911
#elif ENABLED(TFT_TOUCH_DEVICE_XPT2046)
  #include HAL_PATH(../.., tft/xpt2046.h)
  #define TOUCH_DRIVER_CLASS XPT2046
#else
  #error "Unknown Touch Screen Type."
#endif

// Menu Navigation
extern int8_t encoderTopLine, encoderLine, screen_items;

enum TouchControlType : uint16_t {
  NONE = 0x0000,
  CALIBRATE,
  MENU_SCREEN, MENU_ITEM,
  BACK,
  PAGE_UP, PAGE_DOWN,
  CLICK, MENU_CLICK,
  RESUME_CONTINUE,
  SLIDER,
  INCREASE, DECREASE,
  CANCEL, CONFIRM,
  HEATER, FAN,
  FEEDRATE, FLOWRATE,
  UBL,
  STOP,
  BUTTON,
  CALLBACK
};

typedef struct __attribute__((__packed__)) {
  TouchControlType type;
  uint16_t x, y;
  uint16_t width, height;
  intptr_t data;
  int32_t index;
} touch_control_t;

typedef struct {
  uint16_t x, y;
  int32_t index;
} touch_event_t;

typedef void (*touch_handler_t)(touch_event_t*);

#define MAX_CONTROLS        16
#define MINIMUM_HOLD_TIME   15
#define TOUCH_REPEAT_DELAY  75
#define MIN_REPEAT_DELAY    25
#define UBL_REPEAT_DELAY    125
#define FREE_MOVE_RANGE     32

#define TSLP_SLEEPING 1

class Touch {
  private:
    static TOUCH_DRIVER_CLASS io;
    static int16_t x, y;
    static bool enabled;

    static touch_control_t controls[MAX_CONTROLS];
    static touch_control_t *current_control;
    static uint16_t controls_count;

    static millis_t next_touch_ms, time_to_hold, repeat_delay, touch_time;
    static TouchControlType touch_control_type;

    static bool get_point(int16_t * const x, int16_t * const y);
    static void touch(touch_control_t *control);
    static void hold(touch_control_t *control, millis_t delay=0);

  public:
    static touch_event_t touch_event;
    static void init();
    static void reset() { controls_count = 0; touch_time = 0; current_control = nullptr; }
    static void clear() { controls_count = 0; }
    static void idle();
    static bool is_clicked() {
      if (touch_control_type == CLICK) {
        touch_control_type = NONE;
        return true;
      }
      return false;
    }
    static void disable() { enabled = false; }
    static void enable() { enabled = true; }
    #if HAS_DISPLAY_SLEEP
      static millis_t next_sleep_ms;
      static bool isSleeping() { return next_sleep_ms == TSLP_SLEEPING; }
      static void sleepTimeout();
      static void wakeUp();
    #endif
    static void add_control(TouchControlType type, uint16_t x, uint16_t y, uint16_t width, uint16_t height, intptr_t data=0, int32_t index=0);
    static void add_control(TouchControlType type, uint16_t x, uint16_t y, uint16_t width, uint16_t height, void (*handler)()) {
      add_control(type, x, y, width, height, intptr_t(handler));
    }
};

extern Touch touch;
