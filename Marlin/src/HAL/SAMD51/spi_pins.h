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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */

#ifdef ADAFRUIT_GRAND_CENTRAL_M4

 /**
  * AGCM4 Default SPI Pins
  *
  *         SS    SCK   MISO   MOSI
  *       +-------------------------+
  *  SPI  | 53    52     50     51  |
  *  SPI1 | 83    81     80     82  |
  *       +-------------------------+
  * Any pin can be used for Chip Select (SD_SS_PIN)
  */
  #ifndef SD_SCK_PIN
    #define SD_SCK_PIN    52
  #endif
  #ifndef SD_MISO_PIN
    #define SD_MISO_PIN   50
  #endif
  #ifndef SD_MOSI_PIN
    #define SD_MOSI_PIN   51
  #endif

#else

  #error "Unsupported board!"

#endif
