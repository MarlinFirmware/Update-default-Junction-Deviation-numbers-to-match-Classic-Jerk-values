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
#pragma once

/**
 * pins/lcd/ANET_FULL_GRAPHICS_LCD.h
 */

// IS_RRD_FG_SC
//  DOGLCD
//   HAS_MARLINUI_U8GLIB > (128x64)
//  IS_U8GLIB_ST7920
//   IS_ULTIPANEL
//     HAS_WIRED_LCD
//       HAS_DISPLAY
//         HAS_STATUS_MESSAGE
//     IS_NEWPANEL
//     HAS_MARLINUI_MENU
//       HAS_MANUAL_MOVE_MENU
//  IS_RRD_SC

#if ENABLED(ANET_FULL_GRAPHICS_LCD)

  CONTROLLER_WARNING("ANET_FULL_GRAPHICS_LCD.h", "ANET_FULL_GRAPHICS_LCD")

  /**
   * 1. Cut the tab off the LCD connector so it can be plugged into the "EXP1" connector the other way.
   * 2. Swap the LCD's +5V (Pin2) and GND (Pin1) wires. (This is the critical part!)
   * 3. Rewire the CLK Signal (LCD Pin9) to LCD Pin7. (LCD Pin9 remains open because it is open drain.)
   * 4. A wire is needed to connect the Reset switch at J3 (LCD Pin7) to EXP2 (Pin3) on the board.
   *
   * !!! If you are unsure, ask for help! Your motherboard may be damaged in some circumstances !!!
   *
   * The ANET_FULL_GRAPHICS_LCD connector plug:
   *
   *            BEFORE                     AFTER
   *            ------                     ------
   *  (BEEPER) | 1  2 | (CLK)    (BEEPER) |10  9 | --
   * (BTN_ENC) | 3  4 | --      (BTN_ENC) | 8  7 | (CLK)
   * (BTN_EN1)   5  6 | (SID)   (BTN_EN1)   6  5 | (SID)
   * (BTN_EN2) | 7  8 | (CS)    (BTN_EN2) | 4  3 | (CS)
   *        5V | 9 10 | GND           GND | 2  1 | 5V
   *            ------                     ------
   *             LCD                        LCD
   */
  #ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
    #error "CAUTION! ANET_FULL_GRAPHICS_LCD requires wiring modifications. See 'pins_lcd.h' for details." \
           "(Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
  #endif

  #define BEEPER_PIN      LCD1_01_PIN

  #define BTN_ENC         LCD1_03_PIN
  #define BTN_EN1         LCD1_05_PIN
  #define BTN_EN2         LCD1_07_PIN

  #define LCD_PINS_D4     LCD1_04_PIN
  #define LCD_PINS_EN     LCD1_06_PIN
  #define LCD_PINS_RS     LCD1_08_PIN

#else // CTC_A10S_A13

  /**
   * 1. Cut the tab off the LCD connector so it can be plugged into the "EXP1" connector the other way.
   * 2. Swap the LCD's +5V (Pin2) and GND (Pin1) wires. (This is the critical part!)
   *
   * !!! If you are unsure, ask for help! Your motherboard may be damaged in some circumstances !!!
   *
   * The CTC_A10S_A13 connector plug:
   *
   *                BEFORE                     AFTER
   *                ------                     ------
   *      (BEEPER) | 1  2 | (CLK)    (BEEPER) |10  9 | (CLK)
   *     (BTN_ENC) | 3  4 | --      (BTN_ENC) | 8  7 | --
   *     (BTN_EN1)   5  6 | (SID)   (BTN_EN1)   6  5 | (SID)
   *     (BTN_EN2) | 7  8 | (CS)    (BTN_EN2) | 4  3 | (CS)
   *            5V | 9 10 | GND           GND | 2  1 | 5V
   *                ------                     ------
   *                 LCD                        LCD
   */
  #ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
    #error "CAUTION! CTC_A10S_A13 requires wiring modifications. See 'pins_lcd.h' for details." \
           "(Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)"
  #endif

  #define BEEPER_PIN      LCD1_08_PIN

  #define BTN_ENC         LCD1_01_PIN
  #define BTN_EN1         LCD1_06_PIN
  #define BTN_EN2         LCD1_07_PIN

  #define LCD_PINS_EN     LCD1_03_PIN
  #define LCD_PINS_RS     LCD1_04_PIN
  #define LCD_PINS_D4     LCD1_05_PIN

#endif
