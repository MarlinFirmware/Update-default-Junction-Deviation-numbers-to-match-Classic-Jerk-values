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

/**
 * leds.cpp - Marlin RGB LED general support
 */

#include "../../inc/MarlinConfig.h"

#if HAS_COLOR_LEDS

#include "leds.h"

#if ANY(CASE_LIGHT_USE_RGB_LED, CASE_LIGHT_USE_NEOPIXEL)
  #include "../caselight.h"
#endif

#if ENABLED(LED_COLOR_PRESETS)
  const LEDColor LEDLights::defaultLEDColor = LEDColor(
    LED_USER_PRESET_RED, LED_USER_PRESET_GREEN, LED_USER_PRESET_BLUE
    OPTARG(HAS_WHITE_LED, LED_USER_PRESET_WHITE)
    OPTARG(NEOPIXEL_LED, LED_USER_PRESET_BRIGHTNESS)
  );
#endif

#if ANY(LED_CONTROL_MENU, PRINTER_EVENT_LEDS, CASE_LIGHT_IS_COLOR_LED)
  LEDColor LEDLights::color;
  bool LEDLights::lights_on;
#endif

LEDLights leds;

void LEDLights::setup() {
  #if ANY(RGB_LED, RGBW_LED)
    if (PWM_PIN(RGB_LED_R_PIN)) SET_PWM(RGB_LED_R_PIN); else SET_OUTPUT(RGB_LED_R_PIN);
    if (PWM_PIN(RGB_LED_G_PIN)) SET_PWM(RGB_LED_G_PIN); else SET_OUTPUT(RGB_LED_G_PIN);
    if (PWM_PIN(RGB_LED_B_PIN)) SET_PWM(RGB_LED_B_PIN); else SET_OUTPUT(RGB_LED_B_PIN);
    #if ENABLED(RGBW_LED)
      if (PWM_PIN(RGB_LED_W_PIN)) SET_PWM(RGB_LED_W_PIN); else SET_OUTPUT(RGB_LED_W_PIN);
    #endif

    #if ENABLED(RGB_STARTUP_TEST)
      int8_t led_pin_count = 0;
      if (PWM_PIN(RGB_LED_R_PIN) && PWM_PIN(RGB_LED_G_PIN) && PWM_PIN(RGB_LED_B_PIN)) led_pin_count = 3;
      #if ENABLED(RGBW_LED)
        if (PWM_PIN(RGB_LED_W_PIN) && led_pin_count) led_pin_count++;
      #endif
      // Startup animation
      if (led_pin_count) {
        // blackout
        if (PWM_PIN(RGB_LED_R_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_R_PIN), 0); else WRITE(RGB_LED_R_PIN, LOW);
        if (PWM_PIN(RGB_LED_G_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_G_PIN), 0); else WRITE(RGB_LED_G_PIN, LOW);
        if (PWM_PIN(RGB_LED_B_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_B_PIN), 0); else WRITE(RGB_LED_B_PIN, LOW);
        #if ENABLED(RGBW_LED)
          if (PWM_PIN(RGB_LED_W_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_W_PIN), 0);
          else WRITE(RGB_LED_W_PIN, LOW);
        #endif
        delay(200);

        for (uint8_t i = 0; i < led_pin_count; ++i) {
          for (uint8_t b = 0; b <= 200; ++b) {
            const uint16_t led_pwm = b <= 100 ? b : 200 - b;
            if (i == 0 && PWM_PIN(RGB_LED_R_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_R_PIN), led_pwm); else WRITE(RGB_LED_R_PIN, b < 100 ? HIGH : LOW);
            if (i == 1 && PWM_PIN(RGB_LED_G_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_G_PIN), led_pwm); else WRITE(RGB_LED_G_PIN, b < 100 ? HIGH : LOW);
            if (i == 2 && PWM_PIN(RGB_LED_B_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_B_PIN), led_pwm); else WRITE(RGB_LED_B_PIN, b < 100 ? HIGH : LOW);
            #if ENABLED(RGBW_LED)
              if (i == 3) {
                if (PWM_PIN(RGB_LED_W_PIN)) hal.set_pwm_duty(pin_t(RGB_LED_W_PIN), led_pwm);
                else WRITE(RGB_LED_W_PIN, b < 100 ? HIGH : LOW);
                delay(RGB_STARTUP_TEST_INNER_MS);//More slowing for ending
              }
            #endif
            delay(RGB_STARTUP_TEST_INNER_MS);
          }
        }
        delay(500);
      }
    #endif // RGB_STARTUP_TEST

  #elif ALL(PCA9632, RGB_STARTUP_TEST)   // PCA9632 RGB_STARTUP_TEST

    constexpr int8_t led_pin_count = TERN(HAS_WHITE_LED, 4, 3);

    // Startup animation
    LEDColor curColor = LEDColorOff();
    PCA9632_set_led_color(curColor);      // blackout
    delay(200);

    /**
     * LED Pin Counter steps -> events
     * | 0-100 | 100-200 | 200-300 | 300-400 |
     *  fade in   steady |           fade out
     *  start next pin fade in
     */

    uint16_t led_pin_counters[led_pin_count] = { 1, 0, 0 };

    bool canEnd = false;
    while (led_pin_counters[0] != 99 || !canEnd) {
      if (led_pin_counters[0] == 99)        // End loop next time pin0 counter is 99
        canEnd = true;
      for (uint8_t i = 0; i < led_pin_count; ++i) {
        if (led_pin_counters[i] > 0) {
          if (++led_pin_counters[i] == 400) // turn off current pin counter in led_pin_counters
            led_pin_counters[i] = 0;
          else if (led_pin_counters[i] == 201) { // start next pin pwm
            led_pin_counters[i + 1 == led_pin_count ? 0 : i + 1] = 1;
            i++; // skip next pin in this loop so it doesn't increment twice
          }
        }
      }
      uint16_t r, g, b;
      r = led_pin_counters[0]; curColor.r = r <= 100 ? r : r <= 300 ? 100 : 400 - r;
      g = led_pin_counters[1]; curColor.g = g <= 100 ? g : g <= 300 ? 100 : 400 - g;
      b = led_pin_counters[2]; curColor.b = b <= 100 ? b : b <= 300 ? 100 : 400 - b;
      #if HAS_WHITE_LED
        const uint16_t w = led_pin_counters[3]; curColor.w = w <= 100 ? w : w <= 300 ? 100 : 400 - w;
      #endif
      PCA9632_set_led_color(curColor);
      delay(RGB_STARTUP_TEST_INNER_MS);
    }

    // Fade to white
    for (uint8_t led_pwm = 0; led_pwm <= 100; ++led_pwm) {
      NOLESS(curColor.r, led_pwm);
      NOLESS(curColor.g, led_pwm);
      NOLESS(curColor.b, led_pwm);
      TERN_(HAS_WHITE_LED, NOLESS(curColor.w, led_pwm));
      PCA9632_set_led_color(curColor);
      delay(RGB_STARTUP_TEST_INNER_MS);
    }

  #endif // PCA9632 && RGB_STARTUP_TEST

  TERN_(NEOPIXEL_LED, neo.init());
  TERN_(PCA9533, PCA9533_init());
  TERN_(LED_USER_PRESET_STARTUP, set_default());
}

void LEDLights::set_color(const LEDColor &incol
  OPTARG(NEOPIXEL_IS_SEQUENTIAL, bool isSequence/*=false*/)
) {

  #if ENABLED(NEOPIXEL_LED)

    const uint32_t neocolor = LEDColorWhite() == incol
                            ? neo.Color(NEO_WHITE)
                            : neo.Color(incol.r, incol.g, incol.b OPTARG(HAS_WHITE_LED, incol.w));

    #if ENABLED(NEOPIXEL_IS_SEQUENTIAL)
      static uint16_t nextLed = 0;
      #ifdef NEOPIXEL_BKGD_INDEX_FIRST
        while (WITHIN(nextLed, NEOPIXEL_BKGD_INDEX_FIRST, NEOPIXEL_BKGD_INDEX_LAST)) {
          neo.reset_background_color();
          if (++nextLed >= neo.pixels()) { nextLed = 0; return; }
        }
      #endif
    #endif

    #if ALL(CASE_LIGHT_MENU, CASE_LIGHT_USE_NEOPIXEL)
      // Update brightness only if caselight is ON or switching leds off
      if (caselight.on || incol.is_off())
    #endif
    neo.set_brightness(incol.i);

    #if ENABLED(NEOPIXEL_IS_SEQUENTIAL)
      if (isSequence) {
        neo.set_pixel_color(nextLed, neocolor);
        neo.show();
        if (++nextLed >= neo.pixels()) nextLed = 0;
        return;
      }
    #endif

    #if ALL(CASE_LIGHT_MENU, CASE_LIGHT_USE_NEOPIXEL)
      // Update color only if caselight is ON or switching leds off
      if (caselight.on || incol.is_off())
    #endif
    neo.set_color(neocolor);

  #endif

  #if ENABLED(BLINKM)

    // This variant uses i2c to send the RGB components to the device.
    blinkm_set_led_color(incol);

  #endif

  #if ANY(RGB_LED, RGBW_LED)

    // This variant uses 3-4 separate pins for the RGB(W) components.
    // If the pins can do PWM then their intensity will be set.
    #define _UPDATE_RGBW(C,c) do {                     \
      if (PWM_PIN(RGB_LED_##C##_PIN))                  \
        hal.set_pwm_duty(pin_t(RGB_LED_##C##_PIN), c); \
      else                                             \
        WRITE(RGB_LED_##C##_PIN, c ? HIGH : LOW);      \
    }while(0)
    #define UPDATE_RGBW(C,c) _UPDATE_RGBW(C, TERN1(CASE_LIGHT_USE_RGB_LED, caselight.on) ? incol.c : 0)
    UPDATE_RGBW(R,r); UPDATE_RGBW(G,g); UPDATE_RGBW(B,b);
    #if ENABLED(RGBW_LED)
      UPDATE_RGBW(W,w);
    #endif

  #endif

  // Update I2C LED driver
  TERN_(PCA9632, PCA9632_set_led_color(incol));
  TERN_(PCA9533, PCA9533_set_rgb(incol.r, incol.g, incol.b));

  #if ANY(LED_CONTROL_MENU, PRINTER_EVENT_LEDS)
    // Don't update the color when OFF
    lights_on = !incol.is_off();
    if (lights_on) color = incol;
  #endif
}

#if ENABLED(LED_CONTROL_MENU)
  void LEDLights::toggle() { if (lights_on) set_off(); else update(); }
#endif

#if HAS_LED_POWEROFF_TIMEOUT

  millis_t LEDLights::led_off_time; // = 0

  void LEDLights::update_timeout(const bool power_on) {
    if (lights_on) {
      const millis_t ms = millis();
      if (power_on)
        reset_timeout(ms);
      else if (ELAPSED(ms, led_off_time))
        set_off();
    }
  }

#endif

#if ENABLED(NEOPIXEL2_SEPARATE)

  #if ENABLED(NEO2_COLOR_PRESETS)
    const LEDColor LEDLights2::defaultLEDColor = LEDColor(
      NEO2_USER_PRESET_RED, NEO2_USER_PRESET_GREEN, NEO2_USER_PRESET_BLUE
      OPTARG(HAS_WHITE_LED2, NEO2_USER_PRESET_WHITE)
      OPTARG(NEOPIXEL_LED, NEO2_USER_PRESET_BRIGHTNESS)
    );
  #endif

  #if ENABLED(LED_CONTROL_MENU)
    LEDColor LEDLights2::color;
    bool LEDLights2::lights_on;
  #endif

  LEDLights2 leds2;

  void LEDLights2::setup() {
    neo2.init();
    TERN_(NEO2_USER_PRESET_STARTUP, set_default());
  }

  void LEDLights2::set_color(const LEDColor &incol) {
    const uint32_t neocolor = LEDColorWhite() == incol
                            ? neo2.Color(NEO2_WHITE)
                            : neo2.Color(incol.r, incol.g, incol.b OPTARG(HAS_WHITE_LED2, incol.w));
    neo2.set_brightness(incol.i);
    neo2.set_color(neocolor);

    #if ENABLED(LED_CONTROL_MENU)
      // Don't update the color when OFF
      lights_on = !incol.is_off();
      if (lights_on) color = incol;
    #endif
  }

  #if ENABLED(LED_CONTROL_MENU)
    void LEDLights2::toggle() { if (lights_on) set_off(); else update(); }
  #endif

#endif  // NEOPIXEL2_SEPARATE

#endif  // HAS_COLOR_LEDS
