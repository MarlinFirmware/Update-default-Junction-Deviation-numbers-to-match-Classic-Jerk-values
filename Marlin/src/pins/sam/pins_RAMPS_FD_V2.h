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
 * RAMPS-FD v2
 *
 * EEPROM supported
 * Use 1k thermistor tables
 */

#define BOARD_INFO_NAME "RAMPS-FD v2"

#ifndef E0_CS_PIN
  #define E0_CS_PIN                           69  // moved from A13 to A15 on v2.2, if not earlier
#endif

#include "pins_RAMPS_FD_V1.h"

#undef INVERTED_HEATER_PINS
#undef INVERTED_BED_PINS
#undef INVERTED_FAN_PINS

#define I2C_EEPROM
#define MARLIN_EEPROM_SIZE              0x10000U  // 64K in a 24C512

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12
#endif

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         5  // Analog Input on AUX2
#endif
