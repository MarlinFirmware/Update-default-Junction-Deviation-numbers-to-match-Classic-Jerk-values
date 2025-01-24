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
 * Pxmalion Core i3 - https://github.com/Pxmalion
 * ATmega2560
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Core i3"
#endif

//
// Servos
//
#define SERVO0_PIN                            51
#define SERVO1_PIN                            -1
#define SERVO2_PIN                            -1
#define SERVO3_PIN                            -1

//
// Limit Switches
//
#define X_STOP_PIN                             3
#define Y_STOP_PIN                             2
#define Z_MIN_PIN                             19
#define Z_MAX_PIN                             18

// TODO: Filament Runout Sensor
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      -1
#endif

//
// Steppers
//
#define X_CS_PIN                              -1
#define Y_CS_PIN                              -1
#define Z_CS_PIN                              -1
#define E0_CS_PIN                             -1
#define E1_CS_PIN                             -1

//
// Heaters / Fans
//
#define FET_ORDER_EFB
#ifndef MOSFET_A_PIN
  #define MOSFET_A_PIN                         8
#endif
#ifndef MOSFET_B_PIN
  #define MOSFET_B_PIN                         7
#endif
#ifndef MOSFET_C_PIN
  #define MOSFET_C_PIN                         9
#endif

//
// Misc. Functions
//
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                        -1  // Analog Input
#endif

#define PS_ON_PIN                             11

#include "pins_RAMPS.h"
