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
 *
 * Available chip select pins for HW SPI are 4 10 52 77 87
 */
#if SD_SS_PIN == 4 || SD_SS_PIN == 10 || SD_SS_PIN == 52 || SD_SS_PIN == 77 || SD_SS_PIN == 87
  #define SD_SCK_PIN      76
  #define SD_MISO_PIN     74
  #define SD_MOSI_PIN     75
#endif

#if SD_SS_PIN == 4
  #define SPI_PIN         87
  #define SPI_CHAN         1
#elif SD_SS_PIN == 10
  #define SPI_PIN         77
  #define SPI_CHAN         0
#elif SD_SS_PIN == 52
  #define SPI_PIN         86
  #define SPI_CHAN         2
#elif SD_SS_PIN == 77
  #define SPI_PIN         77
  #define SPI_CHAN         0
#elif SD_SS_PIN == 87
  #define SPI_PIN         87
  #define SPI_CHAN         1
#else
  #define SOFTWARE_SPI
  #ifndef SD_SCK_PIN
    #define SD_SCK_PIN    52
  #endif
  #ifndef SD_MISO_PIN
    #define SD_MISO_PIN   50
  #endif
  #ifndef SD_MOSI_PIN
    #define SD_MOSI_PIN   51
  #endif
#endif
