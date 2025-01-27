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
 * CNControls V15 for HMS434 pin assignments
 * ATmega2560, ATmega1280
 */

#define ALLOW_MEGA1280
#include "env_validate.h"

#define BOARD_INFO_NAME "CN Controls V15"

//
// Servos
//
#define SERVO0_PIN                             6

//
// Limit Switches
//
#define X_STOP_PIN                            34
#define Y_STOP_PIN                            39
#define Z_STOP_PIN                            62

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     49
#endif

//
// Steppers
//
#define X_STEP_PIN                            14
#define X_DIR_PIN                             25
#define X_ENABLE_PIN                          26

#define Y_STEP_PIN                            11
#define Y_DIR_PIN                             12
#define Y_ENABLE_PIN                          15

#define Z_STEP_PIN                            24
#define Z_DIR_PIN                             27
#define Z_ENABLE_PIN                          28

#define E0_STEP_PIN                           64
#define E0_DIR_PIN                            65
#define E0_ENABLE_PIN                         63

//
// Temperature Sensors
// Analog Inputs
//
#define TEMP_0_PIN                             2  // Analog Input
#define TEMP_BED_PIN                           4  // Analog Input

#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                     5  // Analog Input
#endif

//
// Heaters
//
#define HEATER_0_PIN                           4
#define HEATER_BED_PIN                        32
#define HEATER_CHAMBER_PIN                    33

//
// Fans
//
#define FAN0_PIN                               8

//
// Auto fans
//
#define AUTO_FAN_PIN                          30
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E1_AUTO_FAN_PIN
  #define E1_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E2_AUTO_FAN_PIN
  #define E2_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef E3_AUTO_FAN_PIN
  #define E3_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif
#ifndef CHAMBER_AUTO_FAN_PIN
  //#define CHAMBER_AUTO_FAN_PIN              10
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             53
#define SD_DETECT_PIN                         40

// Common I/O

#define FIL_RUNOUT_PIN                         9
//#define FIL_RUNOUT_PIN                      29  // encoder sensor
//#define PWM_1_PIN                           12
//#define PWM_2_PIN                           13
//#define SPARE_IO                            17
#define BEEPER_PIN                            13
#define STAT_LED_BLUE_PIN                     -1
#define STAT_LED_RED_PIN                      10  // 31
