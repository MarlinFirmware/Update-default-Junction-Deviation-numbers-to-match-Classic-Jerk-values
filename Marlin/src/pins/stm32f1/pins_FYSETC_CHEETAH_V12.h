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

#include "env_validate.h"

#include "pins_FYSETC_CHEETAH.h"

#undef X_HARDWARE_SERIAL
#undef Y_HARDWARE_SERIAL
#undef Z_HARDWARE_SERIAL
#undef E0_HARDWARE_SERIAL

#undef RGB_LED_R_PIN
#undef RGB_LED_G_PIN
#undef RGB_LED_B_PIN

#define FAN1_PIN                            PB0   // Fan1

#if HAS_TMC_UART

  //
  // TMC2208/TMC2209 stepper drivers
  //

  #define X_SERIAL_TX_PIN                   PA11
  #define X_SERIAL_RX_PIN                   PA12

  #define Y_SERIAL_TX_PIN                   PB6
  #define Y_SERIAL_RX_PIN                   PB7

  #define Z_SERIAL_TX_PIN                   PB10
  #define Z_SERIAL_RX_PIN                   PB11

  #define E0_SERIAL_TX_PIN                  PA2
  #define E0_SERIAL_RX_PIN                  PA3

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART
