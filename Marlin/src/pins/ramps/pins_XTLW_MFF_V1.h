/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * XTLW MFF V1.0 pin assignments
 */

#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "XTLW MFF V1.0"
#endif

//
// Steppers
//
#ifndef E2_STEP_PIN
  #define E2_STEP_PIN                         42
#endif
#ifndef E2_DIR_PIN
  #define E2_DIR_PIN                          40
#endif
#ifndef E2_ENABLE_PIN
  #define E2_ENABLE_PIN                       44
#endif

//
// Heaters / Fans
//

#define FET_ORDER_EFB
//#define HEATER_0_PIN                        10
#define MOSFET_D_PIN                           7
//#define HEATER_BED_PIN                       8

// Filament sensor
#if Y_MIN_PIN != 14 && !PIN_EXISTS(FIL_RUNOUT)
  #define FIL_RUNOUT_PIN                      14  // Use Y_MIN limit switch interface
#endif
#if Z_MAX_PIN != 19 && !PIN_EXISTS(FIL_RUNOUT2)
  #define FIL_RUNOUT2_PIN                     19  // Use Z_MAX limit switch interface
#endif

#include "pins_RAMPS.h"
