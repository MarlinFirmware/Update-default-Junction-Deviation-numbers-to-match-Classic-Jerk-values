/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ALL(HAS_MARLINUI_U8GLIB, HAS_MEDIA) && (LCD_PINS_D4 == SD_SCK_PIN || LCD_PINS_EN == SD_MOSI_PIN || DOGLCD_SCK == SD_SCK_PIN || DOGLCD_MOSI == SD_MOSI_PIN)
  #define SOFTWARE_SPI      // If the SD card and LCD adapter share the same SPI pins, then software SPI is currently
                            // needed due to the speed and mode required for communicating with each device being different.
                            // This requirement can be removed if the SPI access to these devices is updated to use
                            // spiBeginTransaction.
#endif

// Onboard SD
//#define SD_SCK_PIN     P0_07
//#define SD_MISO_PIN    P0_08
//#define SD_MOSI_PIN    P0_09

// External SD
#ifndef SD_SCK_PIN
  #define SD_SCK_PIN        50
#endif
#ifndef SD_MISO_PIN
  #define SD_MISO_PIN       51
#endif
#ifndef SD_MOSI_PIN
  #define SD_MOSI_PIN       52
#endif
