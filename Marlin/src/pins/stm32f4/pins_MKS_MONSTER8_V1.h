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

#define BOARD_INFO_NAME "MKS Monster8 V1"

//
// Limit Switches
//
#define X_MIN_PIN                           PA14
#define X_MAX_PIN                           PA13
#define Y_MIN_PIN                           PA15
#define Y_MAX_PIN                           PC5

//
// Steppers
//
#define E4_ENABLE_PIN                       PD14  // Driver7

//
// Misc. Functions
//
#define PW_DET                              PC5   // Y+
#define PW_OFF                              PB12  // Z+
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PC5   // Y+
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                   PB12  // Z+
#endif

#include "pins_MKS_MONSTER8_common.h"
