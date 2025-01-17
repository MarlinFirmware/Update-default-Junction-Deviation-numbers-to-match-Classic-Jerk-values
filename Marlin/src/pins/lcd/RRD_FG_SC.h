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
 * pins/lcd/RRD_FG_SC.h
 */

/**
 *  RepRap Discount Full Graphics Smart Controller
 *           ------                    ------
 *   BEEPER | 1  2 | ENC          --- | 1  2 | ---
 *   LCD_EN | 3  4 | LCD_RS       EN2 | 3  4 | (SDSS)
 *       D4   5  6 | D5           EN1   5  6 | ---
 *       D6 | 7  8 | D7        SD_DET | 7  8 | ---
 *      GND | 9 10 | 5V           GND | 9 10 | 5V
 *           ------                    ------
 *            LCD1                      LCD2
 */

#define BEEPER_PIN        LCD1_01_PIN

#define BTN_ENC           LCD1_02_PIN
#define BTN_EN1           LCD2_05_PIN  // EN1/2 should be swapped for default encoder direction CW+
#define BTN_EN2           LCD2_03_PIN

#if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define BTN_ENC_EN      LCD1_08_PIN  // Detect the presence of the encoder
#endif

#define LCD_PINS_EN       LCD1_03_PIN
#define LCD_PINS_RS       LCD1_04_PIN
#define LCD_PINS_D4       LCD1_05_PIN

#ifndef SDSS
  #define SDSS            LCD2_04_PIN
#endif
#ifndef SD_DETECT_PIN
  #define SD_DETECT_PIN   LCD2_07_PIN
#endif
