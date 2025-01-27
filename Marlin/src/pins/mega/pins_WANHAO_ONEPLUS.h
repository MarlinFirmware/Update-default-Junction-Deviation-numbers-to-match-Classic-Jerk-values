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

/**
 * Wanhao 0ne+ pin assignments
 * ATmega2560
 */

#include "env_validate.h"

#define BOARD_INFO_NAME      "Wanhao i3 Mini 0ne+"
#define DEFAULT_MACHINE_NAME "i3 Mini"
#define BOARD_WEBSITE_URL    "tinyurl.com/yyxw7se7"

//
// Limit Switches
//
#define X_STOP_PIN                            19
#define Y_STOP_PIN                            18
#define Z_STOP_PIN                            38

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     38
#endif

//
// Steppers
//
#define X_STEP_PIN                            22
#define X_DIR_PIN                             23
#define X_ENABLE_PIN                          57

#define Y_STEP_PIN                            25
#define Y_DIR_PIN                             26
#define Y_ENABLE_PIN                          24

#define Z_STEP_PIN                            29
#define Z_DIR_PIN                             39
#define Z_ENABLE_PIN                          28

#define E0_STEP_PIN                           55
#define E0_DIR_PIN                            56
#define E0_ENABLE_PIN                         54

//
// Temperature Sensors
//
#define TEMP_0_PIN                            13
#define TEMP_BED_PIN                          14

//
// Heaters / Fans
//
#define HEATER_0_PIN                           4
#define HEATER_BED_PIN                        44
#define FAN0_PIN                              12  // IO pin. Buffer needed

//
// SD Card
//
#define SD_DETECT_PIN                         83
#define SD_SS_PIN                             53

//
// Misc. Functions
//
#define BEEPER_PIN                            37
#define KILL_PIN                              64

//
// LCD / Controller (Integrated MINIPANEL)
//
#if ENABLED(MINIPANEL)
  #define DOGLCD_A0                           40
  #define DOGLCD_CS                           41
  #define LCD_BACKLIGHT_PIN                   65  // Backlight LED on A11/D65
  #define LCD_RESET_PIN                       27

  #define BTN_EN1                              2
  #define BTN_EN2                              3
  #define BTN_ENC                              5

  // This display has adjustable contrast
  #define LCD_CONTRAST_MIN                     0
  #define LCD_CONTRAST_MAX                   255
  #define LCD_CONTRAST_INIT     LCD_CONTRAST_MAX
#endif
