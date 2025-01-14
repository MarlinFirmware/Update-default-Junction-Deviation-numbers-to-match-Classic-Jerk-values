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
 * Velleman K8800 (Vertex)
 * Schematic: https://cdn.velleman.eu/downloads/files/vertex-delta/schematics/K8800-schematic-V1.4.pdf
 * ATmega2560, ATmega1280
 */

#include "env_validate.h"

#define BOARD_INFO_NAME      "K8800"
#define DEFAULT_MACHINE_NAME "Vertex Delta"

//
// Limit Switches
//
#define X_STOP_PIN                             3
#define Y_STOP_PIN                            14
#define Z_STOP_PIN                            66

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     68
#endif

#define FIL_RUNOUT_PIN                        69  // PK7

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          63

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24

//
// Temperature Sensors
//
#define TEMP_0_PIN                            13

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#define FAN0_PIN                               8
#define CONTROLLER_FAN_PIN                     9

//
// Misc. Functions
//
#define KILL_PIN                              20  // PD1
#define CASE_LIGHT_PIN                         7

//
// SD Card
//
#define SDSS                                  25
#define SD_DETECT_PIN                         21  // PD0

//
// LCD / Controller
//
#define BEEPER_PIN                             6

#if HAS_WIRED_LCD

  #define LCD_SDSS                            53

  #define DOGLCD_CS                           29
  #define DOGLCD_A0                           27

  #define LCD_PINS_RS                         27
  #define LCD_PINS_EN                         29
  #define LCD_PINS_D4                         37
  #define LCD_PINS_D5                         35
  #define LCD_PINS_D6                         33
  #define LCD_PINS_D7                         31

  //#define LCD_SCREEN_ROTATE                180  // 0, 90, 180, 270

  #if IS_NEWPANEL
    #define BTN_EN1                           17
    #define BTN_EN2                           16
    #define BTN_ENC                           23
  #endif

#endif // HAS_WIRED_LCD
