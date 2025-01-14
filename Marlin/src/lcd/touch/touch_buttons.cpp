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

#include "../../inc/MarlinConfig.h"

#if HAS_TOUCH_BUTTONS

#include "touch_buttons.h"
#include "../scaled_tft.h"

#if ENABLED(TFT_TOUCH_DEVICE_GT911)
  #include HAL_PATH(../.., tft/gt911.h)
  GT911 touchIO;
#elif ENABLED(TFT_TOUCH_DEVICE_XPT2046)
  #include HAL_PATH(../.., tft/xpt2046.h)
  XPT2046 touchIO;
#else
  #error "Unknown Touch Screen Type."
#endif

#if HAS_DISPLAY_SLEEP
  millis_t TouchButtons::next_sleep_ms;
#endif

#include "../buttons.h" // For EN_C bit mask
#include "../marlinui.h" // For ui.refresh
#include "../tft_io/tft_io.h"
#include "../tft_io/touch_calibration.h"

#define DOGM_AREA_LEFT   TFT_PIXEL_OFFSET_X
#define DOGM_AREA_TOP    TFT_PIXEL_OFFSET_Y
#define DOGM_AREA_WIDTH  (GRAPHICAL_TFT_UPSCALE) * (LCD_PIXEL_WIDTH)
#define DOGM_AREA_HEIGHT (GRAPHICAL_TFT_UPSCALE) * (LCD_PIXEL_HEIGHT)

#define BUTTON_AREA_TOP BUTTON_Y_LO
#define BUTTON_AREA_BOT BUTTON_Y_HI

TouchButtons touchBt;

void TouchButtons::init() {
  touchIO.init();
  #if HAS_DISPLAY_SLEEP
    next_sleep_ms = ui.sleep_timeout_minutes ? millis() + MIN_TO_MS(ui.sleep_timeout_minutes) : 0;
  #endif
}

uint8_t TouchButtons::read_buttons() {
  #if HAS_WIRED_LCD
    int16_t x, y;

    #if ENABLED(TFT_TOUCH_DEVICE_XPT2046)

      const bool is_touched = TOUCH_PORTRAIT == _TOUCH_ORIENTATION
                                ? touchIO.getRawPoint(&y, &x)
                                : touchIO.getRawPoint(&x, &y);
      #if HAS_DISPLAY_SLEEP
        if (is_touched)
          wakeUp();
        else if (next_sleep_ms && !isSleeping() && ELAPSED(millis(), next_sleep_ms) && ui.on_status_screen())
          sleepTimeout();
      #endif

      #if ENABLED(TOUCH_SCREEN_CALIBRATION)
        static bool no_touch = false;
      #endif

      if (!is_touched) {
        TERN_(TOUCH_SCREEN_CALIBRATION, no_touch = false);
        return 0;
      }

      #if ENABLED(TOUCH_SCREEN_CALIBRATION)
        const calibrationState state = touch_calibration.get_calibration_state();
        if (WITHIN(state, CALIBRATION_TOP_LEFT, CALIBRATION_BOTTOM_LEFT)) {
          if (!no_touch && touch_calibration.handleTouch(x, y)) ui.refresh();
          no_touch = true;
          return 0;
        }
        x = int16_t((int32_t(x) * _TOUCH_CALIBRATION_X) >> 16) + _TOUCH_OFFSET_X;
        y = int16_t((int32_t(y) * _TOUCH_CALIBRATION_Y) >> 16) + _TOUCH_OFFSET_Y;
      #else
        x = uint16_t((uint32_t(x) * _TOUCH_CALIBRATION_X) >> 16) + _TOUCH_OFFSET_X;
        y = uint16_t((uint32_t(y) * _TOUCH_CALIBRATION_Y) >> 16) + _TOUCH_OFFSET_Y;
      #endif

    #elif ENABLED(TFT_TOUCH_DEVICE_GT911)

      const bool is_touched = TOUCH_PORTRAIT == _TOUCH_ORIENTATION ? touchIO.getRawPoint(&y, &x) : touchIO.getRawPoint(&x, &y);
      if (!is_touched) return 0;

    #endif

    // Touch within the button area simulates an encoder button
    if (y > BUTTON_AREA_TOP && y < BUTTON_AREA_BOT)
      return WITHIN(x, BUTTOND_X_LO, BUTTOND_X_HI) ? EN_D
           : WITHIN(x, BUTTONA_X_LO, BUTTONA_X_HI) ? EN_A
           : WITHIN(x, BUTTONB_X_LO, BUTTONB_X_HI) ? EN_B
           : WITHIN(x, BUTTONC_X_LO, BUTTONC_X_HI) ? EN_C
           : 0;

    if ( !WITHIN(x, DOGM_AREA_LEFT, DOGM_AREA_LEFT + DOGM_AREA_WIDTH)
      || !WITHIN(y, DOGM_AREA_TOP,  DOGM_AREA_TOP  + DOGM_AREA_HEIGHT)
    ) return 0;

    // Column and row above BUTTON_AREA_TOP
    int8_t col = (x - (DOGM_AREA_LEFT)) * (LCD_WIDTH)  / (DOGM_AREA_WIDTH),
           row = (y - (DOGM_AREA_TOP))  * (LCD_HEIGHT) / (DOGM_AREA_HEIGHT);

    // Send the touch to the UI (which will simulate the encoder wheel)
    MarlinUI::screen_click(row, col, x, y);
  #endif
  return 0;
}

#if HAS_DISPLAY_SLEEP

  void TouchButtons::sleepTimeout() {
    #if HAS_LCD_BRIGHTNESS
      ui.set_brightness(0);
    #elif PIN_EXISTS(TFT_BACKLIGHT)
      WRITE(TFT_BACKLIGHT_PIN, LOW);
    #endif
    next_sleep_ms = TSLP_SLEEPING;
  }

  void TouchButtons::wakeUp() {
    if (isSleeping()) {
      #if HAS_LCD_BRIGHTNESS
        ui.set_brightness(ui.brightness);
      #elif PIN_EXISTS(TFT_BACKLIGHT)
        WRITE(TFT_BACKLIGHT_PIN, HIGH);
      #endif
    }
    next_sleep_ms = ui.sleep_timeout_minutes ? millis() + MIN_TO_MS(ui.sleep_timeout_minutes) : 0;
  }

  bool MarlinUI::display_is_asleep() {
    return touchBt.isSleeping();
  }
  void MarlinUI::sleep_display(const bool sleep/*=true*/) {
    if (!sleep) touchBt.wakeUp();
  }

#endif // HAS_DISPLAY_SLEEP

#endif // HAS_TOUCH_BUTTONS
