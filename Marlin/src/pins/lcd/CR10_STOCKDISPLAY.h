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
 * pins/lcd/CR10_STOCKDISPLAY.h
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

/**          ------
 *   BEEPER | 1  2 | ENC
 *      EN1 | 3  4 | (RESET/KILL)
 *      EN2   5  6 | LCD_D4
 *   LCD_RS | 7  8 | LCD_EN
 *      GND | 9 10 | 5V
 *           ------
 */
#ifndef BEEPER_PIN
  #define BEEPER_PIN      LCD1_01_PIN
#endif

#define BTN_ENC           LCD1_02_PIN
#define BTN_EN1           LCD1_03_PIN
#define BTN_EN2           LCD1_05_PIN

#define LCD_PINS_D4       LCD1_06_PIN // ST9720 CLK
#define LCD_PINS_RS       LCD1_07_PIN // ST9720 CS
#define LCD_PINS_EN       LCD1_08_PIN // ST9720 DAT

#ifndef KILL_PIN
  #define KILL_PIN        LCD1_04_PIN
#endif
