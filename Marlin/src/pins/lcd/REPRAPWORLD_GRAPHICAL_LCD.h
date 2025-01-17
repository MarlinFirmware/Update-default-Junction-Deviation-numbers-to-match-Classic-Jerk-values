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
 * pins/lcd/REPRAPWORLD_GRAPHICAL_LCD.h
 */

/**
 * https://reprapworld.com/electronics/autonomous-printing/smart-controller-graphical-lcd-12864-lcd/
 *
 *  RRW GLCD Headers      UT 2  4  6    PWR  -  +
 *                           1  3  5
 *
 * AUX1 1  2    MTICSP 2  4  6  8 10  SDOUT  2  4  6  8
 *      3  4           1  3  5  7  9         1  3  5  7
 *      5  6
 *      7  8
 *             KEYPAD  2  4  6  8 10    SPI  2  4  6  8
 *                     1  3  5  7  9         1  3  5  7
 */

// DOGLCD IS_U8GLIB_ST7920 IS_ULTIPANEL HAS_WIRED_LCD IS_NEWPANEL HAS_MARLINUI_MENU

// REPRAPWORLD_GRAPHICAL_LCD connects to AUX2 and AUX3 on RAMPS. Other boards vary.
#ifndef LCD_PINS_EN
  #error "REPRAPWORLD_GRAPHICAL_LCD connection is not defined for your MOTHERBOARD."
#endif
