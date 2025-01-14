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

//
// LED Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU && ANY(LED_CONTROL_MENU, CASE_LIGHT_MENU)

#include "menu_item.h"

#if ENABLED(PSU_CONTROL)
  #include "../../feature/power.h"
#endif

#if ALL(CASE_LIGHT_MENU, CASELIGHT_USES_BRIGHTNESS)
  #include "../../feature/caselight.h"
  void menu_case_light() {
    START_MENU();
    BACK_ITEM(MSG_CONFIGURATION);
    EDIT_ITEM(percent, MSG_CASE_LIGHT_BRIGHTNESS, &caselight.brightness, 0, 255, caselight.update_brightness, true);
    EDIT_ITEM(bool, MSG_CASE_LIGHT, &caselight.on, caselight.update_enabled);
    END_MENU();
  }
#endif

#if ENABLED(LED_CONTROL_MENU)

  #include "../../feature/leds/leds.h"

  #define MSG_LIGHT2_PRESETS TERN(BIQU_BX_TFT70, MSG_LIGHT_ENCODER_PRESETS, MSG_NEO2_PRESETS)

  #if ENABLED(LED_COLOR_PRESETS)

    void menu_led_presets() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_LED_PRESETS, SS_DEFAULT|SS_INVERT);
      #endif
      BACK_ITEM(MSG_LED_CONTROL);
      ACTION_ITEM(MSG_SET_LEDS_WHITE,  leds.set_white);
      ACTION_ITEM(MSG_SET_LEDS_RED,    leds.set_red);
      ACTION_ITEM(MSG_SET_LEDS_ORANGE, leds.set_orange);
      ACTION_ITEM(MSG_SET_LEDS_YELLOW, leds.set_yellow);
      ACTION_ITEM(MSG_SET_LEDS_GREEN,  leds.set_green);
      ACTION_ITEM(MSG_SET_LEDS_BLUE,   leds.set_blue);
      ACTION_ITEM(MSG_SET_LEDS_INDIGO, leds.set_indigo);
      ACTION_ITEM(MSG_SET_LEDS_VIOLET, leds.set_violet);
      END_MENU();
    }

  #endif // LED_COLOR_PRESETS

  #if ENABLED(NEO2_COLOR_PRESETS)

    void menu_leds2_presets() {
      START_MENU();
      #if LCD_HEIGHT > 2
        STATIC_ITEM(MSG_LIGHT2_PRESETS, SS_DEFAULT|SS_INVERT);
      #endif
      BACK_ITEM(MSG_LED_CONTROL);
      ACTION_ITEM(MSG_SET_LEDS_WHITE,  leds2.set_white);
      ACTION_ITEM(MSG_SET_LEDS_RED,    leds2.set_red);
      ACTION_ITEM(MSG_SET_LEDS_ORANGE, leds2.set_orange);
      ACTION_ITEM(MSG_SET_LEDS_YELLOW, leds2.set_yellow);
      ACTION_ITEM(MSG_SET_LEDS_GREEN,  leds2.set_green);
      ACTION_ITEM(MSG_SET_LEDS_BLUE,   leds2.set_blue);
      ACTION_ITEM(MSG_SET_LEDS_INDIGO, leds2.set_indigo);
      ACTION_ITEM(MSG_SET_LEDS_VIOLET, leds2.set_violet);
      END_MENU();
    }

  #endif // NEO2_COLOR_PRESETS

  void menu_led_custom() {
    START_MENU();
    BACK_ITEM(MSG_LED_CONTROL);

    #if ENABLED(NEOPIXEL2_SEPARATE)
      STATIC_ITEM_N(1, MSG_LED_CHANNEL_N, SS_DEFAULT|SS_INVERT);
    #endif
    EDIT_ITEM(uint8, MSG_INTENSITY_R, &leds.color.r, 0, 255, leds.update, true);
    EDIT_ITEM(uint8, MSG_INTENSITY_G, &leds.color.g, 0, 255, leds.update, true);
    EDIT_ITEM(uint8, MSG_INTENSITY_B, &leds.color.b, 0, 255, leds.update, true);
    #if HAS_WHITE_LED
      EDIT_ITEM(uint8, MSG_INTENSITY_W, &leds.color.w, 0, 255, leds.update, true);
    #endif
    #if ENABLED(NEOPIXEL_LED)
      EDIT_ITEM(uint8, MSG_LED_BRIGHTNESS, &leds.color.i, 0, 255, leds.update, true);
    #endif

    #if ENABLED(NEOPIXEL2_SEPARATE)
      STATIC_ITEM_N(2, MSG_LED_CHANNEL_N, SS_DEFAULT|SS_INVERT);
      EDIT_ITEM(uint8, MSG_INTENSITY_R, &leds2.color.r, 0, 255, leds2.update, true);
      EDIT_ITEM(uint8, MSG_INTENSITY_G, &leds2.color.g, 0, 255, leds2.update, true);
      EDIT_ITEM(uint8, MSG_INTENSITY_B, &leds2.color.b, 0, 255, leds2.update, true);
      #if HAS_WHITE_LED2
        EDIT_ITEM(uint8, MSG_INTENSITY_W, &leds2.color.w, 0, 255, leds2.update, true);
      #endif
      EDIT_ITEM(uint8, MSG_NEO2_BRIGHTNESS, &leds2.color.i, 0, 255, leds2.update, true);
    #endif

    END_MENU();
  }

  void menu_led() {
    #if ENABLED(CASE_LIGHT_MENU)
      const bool has_bright = TERN0(CASELIGHT_USES_BRIGHTNESS, caselight.has_brightness());
    #endif

    START_MENU();
    BACK_ITEM(MSG_MAIN_MENU);

    if (TERN1(PSU_CONTROL, powerManager.psu_on)) {
      editable.state = leds.lights_on;
      #if ENABLED(NEOPIXEL2_SEPARATE) && DISABLED(BIQU_BX_TFT70)
        EDIT_ITEM_N(bool, 1, MSG_LIGHT_N, &editable.state, leds.toggle);
      #else
        EDIT_ITEM(bool, MSG_LIGHTS, &editable.state, leds.toggle);
      #endif
    }

    #if ENABLED(LED_COLOR_PRESETS)
      ACTION_ITEM(MSG_SET_LEDS_DEFAULT, [] { leds.set_default(); ui.refresh(); } );
      SUBMENU(MSG_LED_PRESETS, menu_led_presets);
    #endif

    #if ENABLED(NEOPIXEL2_SEPARATE)
      editable.state = leds2.lights_on;
      #if ENABLED(BIQU_BX_TFT70)
        EDIT_ITEM(bool, MSG_LIGHT_ENCODER, &editable.state, leds2.toggle);
      #else
        EDIT_ITEM_N(bool, 2, MSG_LIGHT_N, &editable.state, leds2.toggle);
      #endif
      #if ENABLED(NEO2_COLOR_PRESETS)
        ACTION_ITEM(MSG_SET_LEDS_DEFAULT, leds2.set_default);
        SUBMENU(MSG_LIGHT2_PRESETS, menu_leds2_presets);
      #endif
    #endif

    //
    // Directly set RGBW and Brightness
    //
    SUBMENU(MSG_CUSTOM_LEDS, menu_led_custom);

    //
    // Set Case light on/off/brightness
    //
    #if ENABLED(CASE_LIGHT_MENU)
      if (has_bright) {
        #if CASELIGHT_USES_BRIGHTNESS
          SUBMENU(MSG_CASE_LIGHT, menu_case_light);
        #endif
      }
      else
        EDIT_ITEM(bool, MSG_CASE_LIGHT, &caselight.on, caselight.update_enabled);
    #endif

    END_MENU();
  }

#endif // LED_CONTROL_MENU

#endif // HAS_MARLINUI_MENU && LED_CONTROL_MENU
