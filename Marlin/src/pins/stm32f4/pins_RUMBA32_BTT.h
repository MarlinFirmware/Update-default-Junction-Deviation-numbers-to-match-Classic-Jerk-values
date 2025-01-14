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
 * No official schematics have been found.
 * But these differences where noted in https://github.com/bigtreetech/Rumba32/issues/1
 */

#define BOARD_INFO_NAME "RUMBA32 (BTT)"

#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE            0x1000  // 4K (24LC32AT-I/OT)
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#endif

#include "pins_RUMBA32_common.h"

#define SERVO0_PIN                          PA15  // Pin is not broken out, is a test point only.

#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 Software Serial
  //
  #define X_SERIAL_TX_PIN                   PC14  // BTT Rumba32 only uses 1 pin for UART
  #define Y_SERIAL_TX_PIN                   PE4
  #define Z_SERIAL_TX_PIN                   PE0
  #define E0_SERIAL_TX_PIN                  PC13
  #define E1_SERIAL_TX_PIN                  PD5
  #define E2_SERIAL_TX_PIN                  PD1
#endif
