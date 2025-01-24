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
 * Pin assignments for the MKS RUMBA32
 *
 * https://github.com/makerbase-mks/MKS-RUMBA32
 *
 * The MKS and Aus3D versions have the same pinout but the MKS version
 * has some added resistors and LEDs.  The resistors needed for the
 * TMC2208/9  UART interface are among the additions.  Also added were
 * connectors and resistors dedicated to the TMC2130 sensorless homing
 * interface.
 */

#define BOARD_INFO_NAME "MKS RUMBA32"

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4K
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#endif

#include "pins_RUMBA32_common.h"

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  #define X_SERIAL_TX_PIN                   PA3
  #define X_SERIAL_RX_PIN                   PC14

  #define Y_SERIAL_TX_PIN                   PA4
  #define Y_SERIAL_RX_PIN                   PE4

  #define Z_SERIAL_TX_PIN                   PD13
  #define Z_SERIAL_RX_PIN                   PE0

  #define E0_SERIAL_TX_PIN                  PD14
  #define E0_SERIAL_RX_PIN                  PC13

  #define E1_SERIAL_TX_PIN                  PD15
  #define E1_SERIAL_RX_PIN                  PD5

  #define E2_SERIAL_TX_PIN                  PD12
  #define E2_SERIAL_RX_PIN                  PD1
#endif
