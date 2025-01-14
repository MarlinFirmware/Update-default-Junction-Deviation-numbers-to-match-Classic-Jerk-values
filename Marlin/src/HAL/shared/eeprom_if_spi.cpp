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

/**
 * Platform-independent Arduino functions for SPI EEPROM.
 * Enable USE_SHARED_EEPROM if not supplied by the framework.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SPI_EEPROM)

#include "eeprom_if.h"

void eeprom_init() {}

#if ENABLED(USE_SHARED_EEPROM)

#define CMD_WREN  6   // WREN
#define CMD_READ  2   // WRITE
#define CMD_WRITE 2   // WRITE

#ifndef EEPROM_WRITE_DELAY
  #define EEPROM_WRITE_DELAY    7
#endif

static void _eeprom_begin(uint8_t * const pos, const uint8_t cmd) {
  const uint8_t eeprom_temp[3] = {
    cmd,
    (unsigned(pos) >> 8) & 0xFF,  // Address High
     unsigned(pos)       & 0xFF   // Address Low
  };
  WRITE(SPI_EEPROM1_CS_PIN, HIGH);    // Usually free already
  WRITE(SPI_EEPROM1_CS_PIN, LOW);     // Activate the Bus
  spiSend(SPI_CHAN_EEPROM1, eeprom_temp, 3);
                                  // Leave the Bus in-use
}

uint8_t eeprom_read_byte(uint8_t *pos) {
  _eeprom_begin(pos, CMD_READ);   // Set read location and begin transmission

  const uint8_t v = spiRec(SPI_CHAN_EEPROM1); // After READ a value sits on the Bus

  WRITE(SPI_EEPROM1_CS_PIN, HIGH);    // Done with device

  return v;
}

void eeprom_write_byte(uint8_t *pos, uint8_t value) {
  const uint8_t eeprom_temp = CMD_WREN;
  WRITE(SPI_EEPROM1_CS_PIN, LOW);
  spiSend(SPI_CHAN_EEPROM1, &eeprom_temp, 1); // Write Enable

  WRITE(SPI_EEPROM1_CS_PIN, HIGH);      // Done with the Bus
  delay(1);                         // For a small amount of time

  _eeprom_begin(pos, CMD_WRITE);    // Set write address and begin transmission

  spiSend(SPI_CHAN_EEPROM1, value); // Send the value to be written
  WRITE(SPI_EEPROM1_CS_PIN, HIGH);      // Done with the Bus
  delay(EEPROM_WRITE_DELAY);        // Give page write time to complete
}

#endif // USE_SHARED_EEPROM
#endif // SPI_EEPROM
