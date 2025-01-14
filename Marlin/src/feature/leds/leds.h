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
 * leds.h - Marlin general RGB LED support
 */

#include "../../inc/MarlinConfigPre.h"

#include <string.h>

// A white component can be passed
#if ANY(RGBW_LED, PCA9632_RGBW)
  #define HAS_WHITE_LED 1
#endif

#if ENABLED(NEOPIXEL_LED)
  #define _NEOPIXEL_INCLUDE_
  #include "neopixel.h"
  #undef _NEOPIXEL_INCLUDE_
#endif

#if ENABLED(BLINKM)
  #include "blinkm.h"
#endif

#if ENABLED(PCA9533)
  #include "pca9533.h"
#endif

#if ENABLED(PCA9632)
  #include "pca9632.h"
#endif

/**
 * LEDcolor type for use with leds.set_color
 */
typedef struct LEDColor {
  uint8_t r, g, b
    OPTARG(HAS_WHITE_LED, w)
    OPTARG(NEOPIXEL_LED, i)
  ;

  LEDColor() : r(255), g(255), b(255)
    OPTARG(HAS_WHITE_LED, w(255))
    OPTARG(NEOPIXEL_LED, i(NEOPIXEL_BRIGHTNESS))
  {}

  LEDColor(const LEDColor&) = default;

  LEDColor(uint8_t r, uint8_t g, uint8_t b OPTARG(HAS_WHITE_LED, uint8_t w=0) OPTARG(NEOPIXEL_LED, uint8_t i=NEOPIXEL_BRIGHTNESS))
    : r(r), g(g), b(b) OPTARG(HAS_WHITE_LED, w(w)) OPTARG(NEOPIXEL_LED, i(i)) {}

  LEDColor(const uint8_t (&rgbw)[4]) : r(rgbw[0]), g(rgbw[1]), b(rgbw[2])
    OPTARG(HAS_WHITE_LED, w(rgbw[3]))
    OPTARG(NEOPIXEL_LED, i(NEOPIXEL_BRIGHTNESS))
  {}

  LEDColor& operator=(const uint8_t (&rgbw)[4]) {
    r = rgbw[0]; g = rgbw[1]; b = rgbw[2];
    TERN_(HAS_WHITE_LED, w = rgbw[3]);
    return *this;
  }

  bool operator==(const LEDColor &right) {
    if (this == &right) return true;
    return 0 == memcmp(this, &right, sizeof(LEDColor));
  }

  bool operator!=(const LEDColor &right) { return !operator==(right); }

  bool is_off() const {
    return 3 > r + g + b + TERN0(HAS_WHITE_LED, w);
  }
} LEDColor;

/**
 * Color presets
 */

#define LEDColorOff()             LEDColor(  0,   0,   0)
#define LEDColorRed()             LEDColor(255,   0,   0)
#if ENABLED(LED_COLORS_REDUCE_GREEN)
  #define LEDColorOrange()        LEDColor(255,  25,   0)
  #define LEDColorYellow()        LEDColor(255,  75,   0)
#else
  #define LEDColorOrange()        LEDColor(255,  80,   0)
  #define LEDColorYellow()        LEDColor(255, 255,   0)
#endif
#define LEDColorGreen()           LEDColor(  0, 255,   0)
#define LEDColorBlue()            LEDColor(  0,   0, 255)
#define LEDColorIndigo()          LEDColor(  0, 255, 255)
#define LEDColorViolet()          LEDColor(255,   0, 255)
#if HAS_WHITE_LED && DISABLED(RGB_LED)
  #define LEDColorWhite()         LEDColor(  0,   0,   0, 255)
#else
  #define LEDColorWhite()         LEDColor(255, 255, 255)
#endif

class LEDLights {
public:
  #if ANY(LED_CONTROL_MENU, PRINTER_EVENT_LEDS, CASE_LIGHT_IS_COLOR_LED)
    static LEDColor color; // last non-off color
    static bool lights_on; // the last set color was "on"
  #else
    static constexpr bool lights_on = true;
  #endif

  LEDLights() {} // ctor

  static void setup(); // init()

  static void set_color(const LEDColor &color
    OPTARG(NEOPIXEL_IS_SEQUENTIAL, bool isSequence=false)
  );

  static void set_color(uint8_t r, uint8_t g, uint8_t b
    OPTARG(HAS_WHITE_LED, uint8_t w=0)
    OPTARG(NEOPIXEL_LED, uint8_t i=NEOPIXEL_BRIGHTNESS)
    OPTARG(NEOPIXEL_IS_SEQUENTIAL, bool isSequence=false)
  ) {
    set_color(LEDColor(r, g, b OPTARG(HAS_WHITE_LED, w) OPTARG(NEOPIXEL_LED, i)) OPTARG(NEOPIXEL_IS_SEQUENTIAL, isSequence));
  }

  static void set_off()   { set_color(LEDColorOff()); }
  static void set_green() { set_color(LEDColorGreen()); }
  static void set_white() { set_color(LEDColorWhite()); }

  #if ENABLED(LED_COLOR_PRESETS)
    static const LEDColor defaultLEDColor;
    static void set_default()  { set_color(defaultLEDColor); }
    static void set_red()      { set_color(LEDColorRed()); }
    static void set_orange()   { set_color(LEDColorOrange()); }
    static void set_yellow()   { set_color(LEDColorYellow()); }
    static void set_blue()     { set_color(LEDColorBlue()); }
    static void set_indigo()   { set_color(LEDColorIndigo()); }
    static void set_violet()   { set_color(LEDColorViolet()); }
  #endif

  #if ENABLED(PRINTER_EVENT_LEDS)
    static LEDColor get_color() { return lights_on ? color : LEDColorOff(); }
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    static void toggle();  // swap "off" with color
  #endif
  #if ANY(LED_CONTROL_MENU, CASE_LIGHT_USE_RGB_LED, HAS_LED_POWEROFF_TIMEOUT)
    static void update() { set_color(color); }
  #endif

  #if HAS_LED_POWEROFF_TIMEOUT
    private:
      static millis_t led_off_time;
    public:
      static void reset_timeout(const millis_t &ms) {
        led_off_time = ms + LED_POWEROFF_TIMEOUT;
        if (!lights_on) update();
      }
      static void update_timeout(const bool power_on);
  #endif
};

extern LEDLights leds;

#if ENABLED(NEOPIXEL2_SEPARATE)

  class LEDLights2 {
  public:
    LEDLights2() {}

    static void setup(); // init()

    static void set_color(const LEDColor &color);

    static void set_color(uint8_t r, uint8_t g, uint8_t b
      OPTARG(HAS_WHITE_LED, uint8_t w=0)
      OPTARG(NEOPIXEL_LED, uint8_t i=NEOPIXEL_BRIGHTNESS)
    ) {
      set_color(LEDColor(r, g, b
        OPTARG(HAS_WHITE_LED, w)
        OPTARG(NEOPIXEL_LED, i)
      ));
    }

    static void set_off()   { set_color(LEDColorOff()); }
    static void set_green() { set_color(LEDColorGreen()); }
    static void set_white() { set_color(LEDColorWhite()); }

    #if ENABLED(NEO2_COLOR_PRESETS)
      static const LEDColor defaultLEDColor;
      static void set_default()  { set_color(defaultLEDColor); }
      static void set_red()      { set_color(LEDColorRed()); }
      static void set_orange()   { set_color(LEDColorOrange()); }
      static void set_yellow()   { set_color(LEDColorYellow()); }
      static void set_blue()     { set_color(LEDColorBlue()); }
      static void set_indigo()   { set_color(LEDColorIndigo()); }
      static void set_violet()   { set_color(LEDColorViolet()); }
    #endif

    #if ENABLED(NEOPIXEL2_SEPARATE)
      static LEDColor color; // last non-off color
      static bool lights_on; // the last set color was "on"
      static void toggle();  // swap "off" with color
      static void update() { set_color(color); }
    #endif
  };

  extern LEDLights2 leds2;

#endif // NEOPIXEL2_SEPARATE
