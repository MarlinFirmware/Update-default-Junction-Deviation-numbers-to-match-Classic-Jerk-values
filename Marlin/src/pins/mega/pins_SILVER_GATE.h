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

// ATmega2561

#if NOT_TARGET(__AVR_ATmega1281__, __AVR_ATmega2561__)
  #error "Oops! Select 'Silvergate' in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Silver Gate"

#define X_STEP_PIN                            43
#define X_DIR_PIN                             44
#define X_ENABLE_PIN                          42
#define X_MIN_PIN                             31
#define X_MAX_PIN                             34

#define Y_STEP_PIN                            40
#define Y_DIR_PIN                             41
#define Y_ENABLE_PIN                          39
#define Y_MIN_PIN                             32
#define Y_MAX_PIN                             35

#define Z_STEP_PIN                            13
#define Z_DIR_PIN                             38
#define Z_ENABLE_PIN                          14
#define Z_MIN_PIN                             33
#define Z_MAX_PIN                             36

#define E0_STEP_PIN                           27
#define E0_DIR_PIN                            37
#define E0_ENABLE_PIN                         45

#define SD_SS_PIN                             16

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      34  // X_MAX unless overridden
#endif

#ifndef FAN0_PIN
  #define FAN0_PIN                             5
#endif

#define HEATER_0_PIN                           7

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN                      3
#endif

#define CONTROLLER_FAN_PIN                     2

#define TEMP_0_PIN                             7  // Analog Input

#define HEATER_BED_PIN                         8
#define TEMP_BED_PIN                           6

#if HAS_WIRED_LCD
  #if IS_U8GLIB_ST7920                            // SPI GLCD 12864 ST7920
    #define LCD_PINS_RS                       30
    #define LCD_PINS_EN                       20
    #define LCD_PINS_D4                       25
    #define BEEPER_PIN                        29
    #define BTN_EN1                           19
    #define BTN_EN2                           22
    #define BTN_ENC                           24
    #define LCD_BACKLIGHT_PIN                  6
    #if ENABLED(SILVER_GATE_GLCD_CONTROLLER)
      #define KILL_PIN                        21
      #define HOME_PIN                        28
    #endif
    #define BOARD_ST7920_DELAY_1               0
    #define BOARD_ST7920_DELAY_2             250
    #define BOARD_ST7920_DELAY_3               0
  #endif
#endif

#define SD_DETECT_PIN                         15

#define STAT_LED_RED_PIN                      23
#define STAT_LED_BLUE_PIN                     26

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                      51
#endif
