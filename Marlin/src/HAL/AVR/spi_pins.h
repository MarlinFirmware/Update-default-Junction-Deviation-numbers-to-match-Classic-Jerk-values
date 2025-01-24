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
 * Define SPI Pins: SCK, MISO, MOSI, SS
 * Platform pins have parentheses, e.g., "(53)", so we cannot use them.
 */
#if defined(__AVR_ATmega168__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__)
  #define _PIN_SPI_SCK  13
  #define _PIN_SPI_MISO 12
  #define _PIN_SPI_MOSI 11
  #define _PIN_SPI_SS   10
#elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__)
  #define _PIN_SPI_SCK  7
  #define _PIN_SPI_MISO 6
  #define _PIN_SPI_MOSI 5
  #define _PIN_SPI_SS   4
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define _PIN_SPI_SCK  52
  #define _PIN_SPI_MISO 50
  #define _PIN_SPI_MOSI 51
  #define _PIN_SPI_SS   53
#elif defined(__AVR_AT90USB1287__) || defined(__AVR_AT90USB1286__) || defined(__AVR_AT90USB646__) || defined(__AVR_AT90USB647__)
  #define _PIN_SPI_SCK  21
  #define _PIN_SPI_MISO 23
  #define _PIN_SPI_MOSI 22
  #define _PIN_SPI_SS   20
#elif defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2561__)
  #define _PIN_SPI_SCK  10
  #define _PIN_SPI_MISO 12
  #define _PIN_SPI_MOSI 11
  #define _PIN_SPI_SS   16
#endif

#ifndef SD_SCK_PIN
  #define SD_SCK_PIN  _PIN_SPI_SCK
#endif
#ifndef SD_MISO_PIN
  #define SD_MISO_PIN _PIN_SPI_MISO
#endif
#ifndef SD_MOSI_PIN
  #define SD_MOSI_PIN _PIN_SPI_MOSI
#endif
