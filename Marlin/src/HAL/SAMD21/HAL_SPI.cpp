/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * SAMD21 HAL developed by Bart Meijer (brupje)
 * Based on SAMD51 HAL by Giuliano Zaro (AKA GMagician)
 */

/**
 * Hardware and software SPI implementations are included in this file.
 *
 * Control of the slave select pin(s) is handled by the calling routines and
 * SAMD21 let hardware SPI handling to remove SS from its logic.
 */

#ifdef __SAMD21__

// --------------------------------------------------------------------------
// Includes
// --------------------------------------------------------------------------

#include "../../inc/MarlinConfig.h"
#include <SPI.h>

// --------------------------------------------------------------------------
// Public functions
// --------------------------------------------------------------------------

#if ANY(SOFTWARE_SPI, FORCE_SOFT_SPI)

  // ------------------------
  // Software SPI
  // ------------------------
  #error "Software SPI not supported for SAMD21. Use Hardware SPI."

#else // !SOFTWARE_SPI

  static SPISettings spiConfig;

  // ------------------------
  // Hardware SPI
  // ------------------------
  void spiBegin() {
    spiInit(SPI_HALF_SPEED);
  }

  void spiInit(uint8_t spiRate) {
    // Use Marlin datarates
    uint32_t clock;
    switch (spiRate) {
      case SPI_FULL_SPEED:      clock = 8000000; break;
      case SPI_HALF_SPEED:      clock = 4000000; break;
      case SPI_QUARTER_SPEED:   clock = 2000000; break;
      case SPI_EIGHTH_SPEED:    clock = 1000000; break;
      case SPI_SIXTEENTH_SPEED: clock =  500000; break;
      case SPI_SPEED_5:         clock =  250000; break;
      case SPI_SPEED_6:         clock =  125000; break;
      default:                  clock = 4000000; break; // Default from the SPI library
    }
    spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);
    SPI.begin();
  }

  /**
   * @brief  Receives a single byte from the SPI port.
   *
   * @return Byte received
   *
   * @details
   */
  uint8_t spiRec() {
    SPI.beginTransaction(spiConfig);
    uint8_t returnByte = SPI.transfer(0xFF);
    SPI.endTransaction();
    return returnByte;
  }

  /**
   * @brief  Receives a number of bytes from the SPI port to a buffer
   *
   * @param  buf   Pointer to starting address of buffer to write to.
   * @param  nbyte Number of bytes to receive.
   * @return Nothing
   */
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    memset(buf, 0xFF, nbyte);

    SPI.beginTransaction(spiConfig);
    SPI.transfer(buf, nbyte);
    SPI.endTransaction();
  }

  /**
   * @brief  Sends a single byte on SPI port
   *
   * @param  b Byte to send
   *
   * @details
   */
  void spiSend(uint8_t b) {
    SPI.beginTransaction(spiConfig);
    SPI.transfer(b);
    SPI.endTransaction();
  }

  /**
   * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
   *
   * @param  buf   Pointer with buffer start address
   * @return Nothing
   *
   * @details Uses DMA
   */
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    SPI.beginTransaction(spiConfig);
    SPI.transfer(token);
    SPI.transfer((uint8_t*)buf, 512);
    SPI.endTransaction();
  }

  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) {
    spiConfig = SPISettings(spiClock, (BitOrder)bitOrder, dataMode);
    SPI.beginTransaction(spiConfig);
  }
#endif // !SOFTWARE_SPI

#endif // __SAMD21__
