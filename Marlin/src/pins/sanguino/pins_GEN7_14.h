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
 * Gen7 v1.4 pin assignments
 * Schematic (1.4): https://github.com/Traumflug/Generation_7_Electronics/blob/Gen7Board-1.4/release%20documents/Gen7Board%201.4%20Schematic.pdf
 * Schematic (1.4.1): https://github.com/Traumflug/Generation_7_Electronics/blob/Gen7Board-1.4.1/release%20documents/Gen7Board%201.4.1%20Schematic.pdf
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

#define BOARD_INFO_NAME "Gen7 v1.4"

#define GEN7_VERSION                          14  // v1.4

//
// Limit switches
//
#define X_STOP_PIN                             0
#define Y_STOP_PIN                             1
#define Z_STOP_PIN                             2

//
// Steppers
//
#define X_STEP_PIN                            29
#define X_DIR_PIN                             28
#define X_ENABLE_PIN                          25

#define Y_STEP_PIN                            27
#define Y_DIR_PIN                             26
#define Y_ENABLE_PIN                          25

#define Z_STEP_PIN                            23
#define Z_DIR_PIN                             22
#define Z_ENABLE_PIN                          25

#define E0_STEP_PIN                           19
#define E0_DIR_PIN                            18
#define E0_ENABLE_PIN                         25

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input
#define TEMP_BED_PIN                           0  // Analog Input

//
// Heaters
//
#define HEATER_0_PIN                           4
#define HEATER_BED_PIN                         3

//
// Misc. Functions
//
#define PS_ON_PIN                             15

#ifndef CASE_LIGHT_PIN
  #define CASE_LIGHT_PIN                      15  // Hardware PWM
#endif

// A pin for debugging
#define DEBUG_PIN                              0

// RS485 pins
#define TX_ENABLE_PIN                         12
#define RX_ENABLE_PIN                         13

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #define SPINDLE_LASER_PWM_PIN               16  // Hardware PWM
  #define SPINDLE_LASER_ENA_PIN               20  // Pullup or pulldown!
  #define SPINDLE_DIR_PIN                     21
#endif
