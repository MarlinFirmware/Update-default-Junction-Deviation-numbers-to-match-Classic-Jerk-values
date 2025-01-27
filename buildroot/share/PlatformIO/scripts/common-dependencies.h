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
 * The purpose of this file is just include Marlin Configuration files,
 * to discover which FEATURES are enabled, without any HAL include.
 * Used by common-dependencies.py
 */

#include "../../../../Marlin/src/inc/MarlinConfig.h"

//
// Conditionals only used for [features]
//
#if ENABLED(SR_LCD_3W_NL)
  // Feature checks for SR_LCD_3W_NL
#elif ANY(LCD_I2C_TYPE_MCP23017, LCD_I2C_TYPE_MCP23008)
  #define USES_LIQUIDTWI2
#elif ENABLED(LCD_I2C_TYPE_PCA8574)
  #define USES_LIQUIDCRYSTAL_I2C
#elif ANY(HAS_MARLINUI_HD44780, LCD_I2C_TYPE_PCF8575, SR_LCD_2W_NL, LCM1602)
  #define USES_LIQUIDCRYSTAL
#endif

#if SAVED_POSITIONS
  #define HAS_SAVED_POSITIONS
#endif

#if ENABLED(DUET_SMART_EFFECTOR) && PIN_EXISTS(SMART_EFFECTOR_MOD)
  #define HAS_SMART_EFF_MOD
#endif

#if HAS_MARLINUI_MENU
  #if ENABLED(BACKLASH_GCODE)
    #define HAS_MENU_BACKLASH
  #endif
  #if ENABLED(LCD_BED_TRAMMING)
    #define HAS_MENU_BED_TRAMMING
  #endif
  #if ENABLED(CANCEL_OBJECTS)
    #define HAS_MENU_CANCELOBJECT
  #endif
  #if ANY(DELTA_CALIBRATION_MENU, DELTA_AUTO_CALIBRATION)
    #define HAS_MENU_DELTA_CALIBRATE
  #endif
  #if ANY(LED_CONTROL_MENU, CASE_LIGHT_MENU)
    #define HAS_MENU_LED
  #endif
  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    #define HAS_MENU_FILAMENT
  #endif
  #if HAS_MEDIA
    #define HAS_MENU_MEDIA
  #endif
  #if ENABLED(MIXING_EXTRUDER)
    #define HAS_MENU_MIXER
  #endif
  #if ENABLED(POWER_LOSS_RECOVERY)
    #define HAS_MENU_JOB_RECOVERY
  #endif
  #if HAS_POWER_MONITOR
    #define HAS_MENU_POWER_MONITOR
  #endif
  #if HAS_CUTTER
    #define HAS_MENU_CUTTER
  #endif
  #if HAS_TEMPERATURE
    #define HAS_MENU_TEMPERATURE
  #endif
  #if ENABLED(MMU_MENUS)
    #define HAS_MENU_MMU
  #endif
  #if ENABLED(PASSWORD_FEATURE)
    #define HAS_MENU_PASSWORD
  #endif
  #if HAS_TRINAMIC_CONFIG
    #define HAS_MENU_TMC
  #endif
  #if ENABLED(TOUCH_SCREEN_CALIBRATION)
    #define HAS_MENU_TOUCH_SCREEN
  #endif
  #if ANY(HAS_LEVELING, HAS_BED_PROBE, ASSISTED_TRAMMING_WIZARD, LCD_BED_TRAMMING)
    #define HAS_MENU_PROBE_LEVEL
  #endif
  #if ENABLED(ASSISTED_TRAMMING_WIZARD)
    #define HAS_MENU_TRAMMING_WIZARD
  #endif
  #if ENABLED(AUTO_BED_LEVELING_UBL)
    #define HAS_MENU_UBL
  #endif
  #if ENABLED(ONE_CLICK_PRINT)
    #define HAS_MENU_ONE_CLICK_PRINT
  #endif
#endif

#if HAS_GRAPHICAL_TFT
  #include "../../../../Marlin/src/lcd/tft/fontdata/fontdata.h"
  #define UI_INCL_(W, H) STRINGIFY_(../../../../Marlin/src/lcd/tft/ui_##W##x##H.h)
  #define UI_INCL(W, H) UI_INCL_(W, H)
  #include UI_INCL(TFT_WIDTH, TFT_HEIGHT)

  #define Latin_Extended_A 1
  #define Cyrillic 2
  #define Greek 3
  #define Katakana 4
  #define Korean 5
  #define Vietnamese 6
  #define Simplified_Chinese 7
  #define Traditional_Chinese 8

  #if TFT_FONT == NOTOSANS
    #if FONT_SIZE == 14
      #define TFT_FONT_NOTOSANS_14
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_14_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_14_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_14_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_14_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_14_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_14_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_14_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_14_ZH_TW
      #endif
    #elif FONT_SIZE == 16
      #define TFT_FONT_NOTOSANS_16
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_16_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_16_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_16_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_16_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_16_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_16_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_16_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_16_ZH_TW
      #endif
    #elif FONT_SIZE == 19
      #define TFT_FONT_NOTOSANS_19
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_19_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_19_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_19_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_19_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_19_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_19_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_19_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_19_ZH_TW
      #endif
    #elif FONT_SIZE == 26
      #define TFT_FONT_NOTOSANS_26
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_26_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_26_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_26_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_26_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_26_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_26_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_26_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_26_ZH_TW
      #endif
    #elif FONT_SIZE == 27
      #define TFT_FONT_NOTOSANS_27
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_27_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_27_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_27_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_27_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_27_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_27_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_27_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_27_ZH_TW
      #endif
    #elif FONT_SIZE == 28
      #define TFT_FONT_NOTOSANS_28
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_28_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_28_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_28_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_28_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_28_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_28_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_28_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_28_ZH_TW
      #endif
    #elif FONT_SIZE == 29
      #define TFT_FONT_NOTOSANS_29
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_NOTOSANS_29_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_NOTOSANS_29_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_NOTOSANS_29_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_NOTOSANS_29_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_NOTOSANS_29_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_NOTOSANS_29_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_NOTOSANS_29_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_NOTOSANS_29_ZH_TW
      #endif
    #endif
  #elif TFT_FONT == UNIFONT
    #if FONT_SIZE == 10
      #define TFT_FONT_UNIFONT_10
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_UNIFONT_10_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_UNIFONT_10_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_UNIFONT_10_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_UNIFONT_10_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_UNIFONT_10_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_UNIFONT_10_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_UNIFONT_10_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_UNIFONT_10_ZH_TW
      #endif
    #elif FONT_SIZE == 20
      #define TFT_FONT_UNIFONT_20
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_UNIFONT_20_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_UNIFONT_20_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_UNIFONT_20_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_UNIFONT_20_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_UNIFONT_20_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_UNIFONT_20_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_UNIFONT_20_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_UNIFONT_20_ZH_TW
      #endif
    #elif FONT_SIZE == 30
      #define TFT_FONT_UNIFONT_30
      #if FONT_EXTRA == Latin_Extended_A
        #define TFT_FONT_UNIFONT_30_LATIN
      #elif FONT_EXTRA == Cyrillic
        #define TFT_FONT_UNIFONT_30_CYRIL
      #elif FONT_EXTRA == Greek
        #define TFT_FONT_UNIFONT_30_GREEK
      #elif FONT_EXTRA == Katakana
        #define TFT_FONT_UNIFONT_30_KATA
      #elif FONT_EXTRA == Korean
        #define TFT_FONT_UNIFONT_30_KO
      #elif FONT_EXTRA == Vietnamese
        #define TFT_FONT_UNIFONT_30_VI
      #elif FONT_EXTRA == Simplified_Chinese
        #define TFT_FONT_UNIFONT_30_ZH_CN
      #elif FONT_EXTRA == Traditional_Chinese
        #define TFT_FONT_UNIFONT_30_ZH_TW
      #endif
    #endif
  #elif TFT_FONT == HELVETICA
    #if FONT_SIZE == 14
      #define TFT_FONT_HELVETICA_14
    #elif FONT_SIZE == 19
      #define TFT_FONT_HELVETICA_19
    #endif
  #endif
#endif
