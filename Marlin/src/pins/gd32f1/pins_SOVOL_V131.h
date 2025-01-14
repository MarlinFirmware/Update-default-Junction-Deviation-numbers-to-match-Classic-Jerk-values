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

/**
 * Sovol 1.3.1 (GD32F103RET6) board pin assignments
 */

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "SOVOL V131 only supports 1 hotend / E-stepper."
  #define E_ERROR 1
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "Sovol V131"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Sovol SV06"
#endif

#include "../stm32f1/pins_CREALITY_V4.h"

#if HAS_TMC_UART

  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */

  #define X_SERIAL_TX_PIN                   PC1
  #define Y_SERIAL_TX_PIN                   PC0
  #define Z_SERIAL_TX_PIN                   PA15
  #define E0_SERIAL_TX_PIN                  PC14

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    3
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    3
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    3
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 3, "X_SLAVE_ADDRESS must be 3 for BOARD_SOVOL_V131.");
  static_assert(Y_SLAVE_ADDRESS == 3, "Y_SLAVE_ADDRESS must be 3 for BOARD_SOVOL_V131.");
  static_assert(Z_SLAVE_ADDRESS == 3, "Z_SLAVE_ADDRESS must be 3 for BOARD_SOVOL_V131.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_SOVOL_V131.");

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// SD Card
//
#define ONBOARD_SPI_DEVICE                     1
#define ONBOARD_SD_CS_PIN                   PA4   // SDSS
