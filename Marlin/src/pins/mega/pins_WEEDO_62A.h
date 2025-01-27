/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Based on WEEDO 62A pin configuration
 * Copyright (c) 2019 WEEDO3D Perron
 * ATmega2560
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "WEEDO 62A"
#endif

//
// Limit Switches
//
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             40
#define Y_MAX_PIN                             41
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

//
// Steppers
//
#define X_STEP_PIN                            26
#define X_DIR_PIN                             28
#define X_ENABLE_PIN                          24

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62

#define E0_STEP_PIN                           54
#define E0_DIR_PIN                            55
#define E0_ENABLE_PIN                         38

//
// Temperature Sensors
//
#define TEMP_0_PIN                            13  // ANALOG NUMBERING
#define TEMP_BED_PIN                          14  // ANALOG NUMBERING

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10  // EXTRUDER 1
#define HEATER_BED_PIN                         8  // BED
#define FAN0_PIN                               4  // IO pin. Buffer needed

//
// Misc. Functions
//
#define PS_ON_PIN                             12
#define LED_PIN                               13

//
// SD Support
//
#if HAS_MEDIA
  #define SD_SS_PIN                           53
  #define SD_DETECT_PIN                       49
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                          37

  #define DOGLCD_A0                           27
  #define DOGLCD_CS                           29
  #define LCD_RESET_PIN                       25
  #define LCD_CONTRAST_INIT                  255

  #define BTN_EN1                             33
  #define BTN_EN2                             31
  #define BTN_ENC                             35
#endif
