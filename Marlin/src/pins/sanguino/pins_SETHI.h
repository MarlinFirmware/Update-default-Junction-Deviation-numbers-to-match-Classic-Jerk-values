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
 * Sethi 3D_1 pin assignments - www.sethi3d.com.br
 */

/**
 * Requires this Arduino IDE extension for Boards Manager:
 * https://github.com/Lauszus/Sanguino
 *
 * Follow the installation instructions at https://learn.sparkfun.com/pages/CustomBoardsArduino
 * Just use this JSON URL instead of Sparkfun's:
 * https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
 *
 * Once installed select the SANGUINO board and then select the CPU.
 */

#define ALLOW_MEGA644
#include "env_validate.h"

#define BOARD_INFO_NAME "Sethi 3D_1"

#ifndef GEN7_VERSION
  #define GEN7_VERSION                        12  // v1.x
#endif

//
// Limit Switches
//
#define X_STOP_PIN                             2
#define Y_STOP_PIN                             0
#define Z_MIN_PIN                              1
#define Z_MAX_PIN                              0

//
// Steppers
//
#define X_STEP_PIN                            19
#define X_DIR_PIN                             18
#define X_ENABLE_PIN                          24

#define Y_STEP_PIN                            23
#define Y_DIR_PIN                             22
#define Y_ENABLE_PIN                          24

#define Z_STEP_PIN                            26
#define Z_DIR_PIN                             25
#define Z_ENABLE_PIN                          24

#define E0_STEP_PIN                           28
#define E0_DIR_PIN                            27
#define E0_ENABLE_PIN                         24

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input
#define TEMP_BED_PIN                           2  // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                           4
#define HEATER_BED_PIN                         3

#if !defined(FAN0_PIN) && GEN7_VERSION < 13       // Gen7 v1.3 removed the fan pin
  #define FAN0_PIN                            31
#endif

//
// Misc. Functions
//
#define PS_ON_PIN                             15

// All these generations of Gen7 supply thermistor power
// via PS_ON, so ignore bad thermistor readings
//#define BOGUS_TEMPERATURE_GRACE_PERIOD     2000

// our pin for debugging.
#define DEBUG_PIN                              0

// our RS485 pins
#define TX_ENABLE_PIN                         12
#define RX_ENABLE_PIN                         13
