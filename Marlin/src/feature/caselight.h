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

/**
 * feature/caselight.h
 */

#include "../inc/MarlinConfig.h"

#if CASE_LIGHT_IS_COLOR_LED
  #include "leds/leds.h" // For LEDColor
#endif

class CaseLight {
public:
  static bool on;
  #if CASELIGHT_USES_BRIGHTNESS
    static uint8_t brightness;
  #endif

  static bool pin_is_pwm() { return TERN0(NEED_CASE_LIGHT_PIN, PWM_PIN(CASE_LIGHT_PIN)); }
  static bool has_brightness() { return TERN0(CASELIGHT_USES_BRIGHTNESS, TERN(CASE_LIGHT_USE_NEOPIXEL, true, pin_is_pwm())); }

  static void init() {
    #if NEED_CASE_LIGHT_PIN
      if (pin_is_pwm()) SET_PWM(CASE_LIGHT_PIN); else SET_OUTPUT(CASE_LIGHT_PIN);
    #endif
    update_brightness();
  }

  static void update(const bool sflag);
  static void update_brightness() { update(false); }
  static void update_enabled()    { update(true);  }

  #if ENABLED(CASE_LIGHT_IS_COLOR_LED)
    private:
      static LEDColor color;
  #endif
};

extern CaseLight caselight;
