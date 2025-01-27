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
 * sd/SdFatConfig.h
 *
 * Arduino SdFat Library
 * Copyright (c) 2009 by William Greiman
 *
 * This file is part of the Arduino Sd2Card Library
 */

#include "../inc/MarlinConfig.h"

/**
 * To use multiple SD cards set USE_MULTIPLE_CARDS nonzero.
 *
 * Using multiple cards costs 400 - 500  bytes of flash.
 *
 * Each card requires about 550 bytes of SRAM so use of a Mega is recommended.
 */
#define USE_MULTIPLE_CARDS 0 // TODO? HAS_MULTI_VOLUME

/**
 * Call flush for endl if ENDL_CALLS_FLUSH is nonzero
 *
 * The standard for iostreams is to call flush.  This is very costly for
 * SdFat.  Each call to flush causes 2048 bytes of I/O to the SD.
 *
 * SdFat has a single 512 byte buffer for SD I/O so it must write the current
 * data block to the SD, read the directory block from the SD, update the
 * directory entry, write the directory block to the SD and read the data
 * block back into the buffer.
 *
 * The SD flash memory controller is not designed for this many rewrites
 * so performance may be reduced by more than a factor of 100.
 *
 * If ENDL_CALLS_FLUSH is zero, you must call flush and/or close to force
 * all data to be written to the SD.
 */
#define ENDL_CALLS_FLUSH 0

/**
 * Allow FAT12 volumes if FAT12_SUPPORT is nonzero.
 * FAT12 has not been well tested.
 */
#define FAT12_SUPPORT 0

/**
 * SPI init rate for SD initialization commands. Must be 5 (F_CPU/64)
 * or 6 (F_CPU/128).
 */
#define SPI_SD_INIT_RATE 5

/**
 * Set the SS pin high for hardware SPI.  If SS is chip select for another SPI
 * device this will disable that device during the SD init phase.
 */
#define SET_SPI_SS_HIGH 1

/**
 * Define MEGA_SOFT_SPI nonzero to use software SPI on Mega Arduinos.
 * Pins used are SS 10, MOSI 11, MISO 12, and SCK 13.
 *
 * MEGA_SOFT_SPI allows an unmodified Adafruit GPS Shield to be used
 * on Mega Arduinos.  Software SPI works well with GPS Shield V1.1
 * but many SD cards will fail with GPS Shield V1.0.
 */
#define MEGA_SOFT_SPI 0

// Set SDFAT_USE_SOFTWARE_SPI nonzero to ALWAYS use Software SPI.
#define SDFAT_USE_SOFTWARE_SPI 0

/**
 * The __cxa_pure_virtual function is an error handler that is invoked when
 * a pure virtual function is called.
 */
#define USE_CXA_PURE_VIRTUAL 1

/**
 * Defines for 8.3 and long (vfat) filenames
 */

#define FILENAME_LENGTH 13 // Number of UTF-16 characters per entry

// UTF-8 may use up to 3 bytes to represent single UTF-16 code point.
// We discard 3-byte characters allowing only 2-bytes
// or 1-byte if UTF_FILENAME_SUPPORT disabled.
#define LONG_FILENAME_CHARSIZE TERN(UTF_FILENAME_SUPPORT, 2, 1)

// Total bytes needed to store a single long filename
#define LONG_FILENAME_LENGTH (FILENAME_LENGTH * LONG_FILENAME_CHARSIZE * VFAT_ENTRIES_LIMIT + 1)
