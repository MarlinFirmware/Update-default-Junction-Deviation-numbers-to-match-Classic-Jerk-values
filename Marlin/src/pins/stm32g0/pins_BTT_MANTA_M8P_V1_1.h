/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

// Include common Manta M8P pins
#include "pins_BTT_MANTA_M8P_common.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT Manta M8P V1.1"
#endif

//
// Steppers
//
#define E1_STEP_PIN                         PA10  // M6
#define E1_DIR_PIN                          PA14
#define E1_ENABLE_PIN                       PA15
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PF8
#endif

#define E2_STEP_PIN                         PD11  // M7
#define E2_DIR_PIN                          PD9
#define E2_ENABLE_PIN                       PD15
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD14
#endif

#define E3_STEP_PIN                         PD8   // M8
#define E3_DIR_PIN                          PC6
#define E3_ENABLE_PIN                       PC7
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD10
#endif

#if HAS_TMC_UART
  #define E1_SERIAL_TX_PIN                  PF8
  #define E2_SERIAL_TX_PIN                  PD14
  #define E3_SERIAL_TX_PIN                  PD10

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PA9
#endif

#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PB15
#endif
