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
 * MKS SGen pin assignments
 * Schematic: https://github.com/makerbase-mks/MKS-SGen/blob/master/Hardware/MKS%20SGEN%20V1.0_001/MKS%20SGEN%20V1.0_001%20SCH.pdf
 *
 * Pins diagram:
 * https://github.com/makerbase-mks/MKS-SGen/blob/master/Hardware/MKS%20SGEN%20V1.0_001/MKS%20SGEN%20V1.0_001%20PIN.pdf
 */

#define BOARD_INFO_NAME   "MKS SGen"
#define BOARD_WEBSITE_URL "github.com/makerbase-mks/MKS-SGEN"

#define REQUIRE_LPC1769
#include "../lpc1768/pins_MKS_SBASE.h"

#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 stepper drivers
  //
  #define X_SERIAL_TX_PIN                  P1_22  // J8-2
  #define Y_SERIAL_TX_PIN                  P1_23  // J8-3
  #define Z_SERIAL_TX_PIN                  P2_12  // J8-4
  #define E0_SERIAL_TX_PIN                 P2_11  // J8-5
  #define E1_SERIAL_TX_PIN                 P4_28  // J8-6

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART
