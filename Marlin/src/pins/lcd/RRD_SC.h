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
 * pins/lcd/RRD_SC.h
 *
 * HD44780 Character Display with SD Card,
 *         Beeper, and optional Click Encoder
 */

/**          ------                    ------
 *   BEEPER | 1  2 | ENC       (MISO) | 1  2 | (SCK)
 *   LCD_EN | 3  4 | LCD_RS       EN1 | 3  4 | (SDSS)
 *       D4   5  6 | D5           EN2   5  6 | (MOSI)
 *       D6 | 7  8 | D7      (SD_DET) | 7  8 | (KILL)
 *      GND | 9 10 | 5V           GND | 9 10 | 5V
 *           ------                    ------
 *            LCD1                      LCD2
 */
#ifndef BEEPER_PIN
  #define BEEPER_PIN      LCD1_01_PIN
#endif

#define LCD_PINS_EN       LCD1_03_PIN
#define LCD_PINS_RS       LCD1_04_PIN
#define LCD_PINS_D4       LCD1_05_PIN
#define LCD_PINS_D5       LCD1_06_PIN
#define LCD_PINS_D6       LCD1_07_PIN
#define LCD_PINS_D7       LCD1_08_PIN

#if IS_NEWPANEL
  #define BTN_ENC         LCD1_02_PIN
  #define BTN_EN1         LCD2_03_PIN
  #define BTN_EN2         LCD2_05_PIN
#endif
