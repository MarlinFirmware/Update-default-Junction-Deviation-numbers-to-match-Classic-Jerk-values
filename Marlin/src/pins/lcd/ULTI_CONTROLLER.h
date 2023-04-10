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
 * pins/lcd/ULTI_CONTROLLER.h
 * http://www.thingiverse.com/thing:15081
 */

// IS_ULTIPANEL
//   HAS_WIRED_LCD
//     HAS_DISPLAY
//       HAS_STATUS_MESSAGE
//   IS_NEWPANEL
//   HAS_MARLINUI_MENU
//     HAS_MANUAL_MOVE_MENU

/**
 *        1     3    (5)    7     9
 * LCD1:  BEEP  LCDE  LCD4  LCD6 GND
 *        ENC   LCDRS LCD5  LCD7 VCC
 *        2     4     6     8     10
 *
 *        1     3    (5)    7     9
 * LCD2: <MISO  EN2   EN1  <SDDET NC
 *        SCK  <SDSS  MOSI  3.3V  NC
 *        2     4     6     8     10
 */

// This controller needs a reset pin
#ifdef LCD_PINS_D6
  #define LCD_RESET_PIN   LCD_PINS_D6
#else
  #define LCD_RESET_PIN   LCD1_07_PIN
#endif
