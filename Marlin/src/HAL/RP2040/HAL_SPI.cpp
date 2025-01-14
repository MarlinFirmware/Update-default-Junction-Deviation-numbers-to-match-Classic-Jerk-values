/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "../../inc/MarlinConfig.h"

#include <SPI.h>

// ------------------------
// Public Variables
// ------------------------

static SPISettings spiConfig;

// ------------------------
// Public functions
// ------------------------

#if ENABLED(SOFTWARE_SPI)

  // ------------------------
  // Software SPI
  // ------------------------

  #include "../shared/Delay.h"

  void spiBegin(void) {
    OUT_WRITE(SD_SS_PIN, HIGH);
    OUT_WRITE(SD_SCK_PIN, HIGH);
    SET_INPUT(SD_MISO_PIN);
    OUT_WRITE(SD_MOSI_PIN, HIGH);
  }

  // Use function with compile-time value so we can actually reach the desired frequency
  // Need to adjust this a little bit: on a 72MHz clock, we have 14ns/clock
  // and we'll use ~3 cycles to jump to the method and going back, so it'll take ~40ns from the given clock here
  #define CALLING_COST_NS  (3U * 1000000000U) / (F_CPU)
  void (*delaySPIFunc)();
  void delaySPI_125()  { DELAY_NS(125 - CALLING_COST_NS); }
  void delaySPI_250()  { DELAY_NS(250 - CALLING_COST_NS); }
  void delaySPI_500()  { DELAY_NS(500 - CALLING_COST_NS); }
  void delaySPI_1000() { DELAY_NS(1000 - CALLING_COST_NS); }
  void delaySPI_2000() { DELAY_NS(2000 - CALLING_COST_NS); }
  void delaySPI_4000() { DELAY_NS(4000 - CALLING_COST_NS); }

  void spiInit(uint8_t spiRate) {
    // Use datarates Marlin uses
    switch (spiRate) {
      case SPI_FULL_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 8,000,000  actual: ~1.1M
      case SPI_HALF_SPEED:   delaySPIFunc =  &delaySPI_125; break;  // desired: 4,000,000  actual: ~1.1M
      case SPI_QUARTER_SPEED:delaySPIFunc =  &delaySPI_250; break;  // desired: 2,000,000  actual: ~890K
      case SPI_EIGHTH_SPEED: delaySPIFunc =  &delaySPI_500; break;  // desired: 1,000,000  actual: ~590K
      case SPI_SPEED_5:      delaySPIFunc = &delaySPI_1000; break;  // desired:   500,000  actual: ~360K
      case SPI_SPEED_6:      delaySPIFunc = &delaySPI_2000; break;  // desired:   250,000  actual: ~210K
      default:               delaySPIFunc = &delaySPI_4000; break;  // desired:   125,000  actual: ~123K
    }
    SPI.begin();
  }

  // Begin SPI transaction, set clock, bit order, data mode
  void spiBeginTransaction(uint32_t spiClock, uint8_t bitOrder, uint8_t dataMode) { /* do nothing */ }

  uint8_t HAL_SPI_RP2040_SpiTransfer_Mode_3(uint8_t b) { // using Mode 3
    for (uint8_t bits = 8; bits--;) {
      WRITE(SD_SCK_PIN, LOW);
      WRITE(SD_MOSI_PIN, b & 0x80);

      delaySPIFunc();
      WRITE(SD_SCK_PIN, HIGH);
      delaySPIFunc();

      b <<= 1;        // little setup time
      b |= (READ(SD_MISO_PIN) != 0);
    }
    DELAY_NS(125);
    return b;
  }

  // Soft SPI receive byte
  uint8_t spiRec() {
    hal.isr_off();  // No interrupts during byte receive
    const uint8_t data = HAL_SPI_RP2040_SpiTransfer_Mode_3(0xFF);
    hal.isr_on();   // Enable interrupts
    return data;
  }

  // Soft SPI read data
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    for (uint16_t i = 0; i < nbyte; i++)
      buf[i] = spiRec();
  }

  // Soft SPI send byte
  void spiSend(uint8_t data) {
    hal.isr_off();                            // No interrupts during byte send
    HAL_SPI_RP2040_SpiTransfer_Mode_3(data);  // Don't care what is received
    hal.isr_on();                             // Enable interrupts
  }

  // Soft SPI send block
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    spiSend(token);
    for (uint16_t i = 0; i < 512; i++)
      spiSend(buf[i]);
  }

#else

  // ------------------------
  // Hardware SPI
  // ------------------------

  /**
   * VGPV SPI speed start and PCLK2/2, by default 108/2 = 54Mhz
   */

  /**
   * @brief  Begin SPI port setup
   *
   * @return Nothing
   *
   * @details Only configures SS pin since stm32duino creates and initialize the SPI object
   */
  void spiBegin() {
    #if PIN_EXISTS(SD_SS)
      OUT_WRITE(SD_SS_PIN, HIGH);
    #endif
  }

  // Configure SPI for specified SPI speed
  void spiInit(uint8_t spiRate) {
    // Use datarates Marlin uses
    uint32_t clock;
    switch (spiRate) {
      case SPI_FULL_SPEED:    clock = 20000000; break; // 13.9mhz=20000000  6.75mhz=10000000  3.38mhz=5000000  .833mhz=1000000
      case SPI_HALF_SPEED:    clock =  5000000; break;
      case SPI_QUARTER_SPEED: clock =  2500000; break;
      case SPI_EIGHTH_SPEED:  clock =  1250000; break;
      case SPI_SPEED_5:       clock =   625000; break;
      case SPI_SPEED_6:       clock =   300000; break;
      default:
        clock = 4000000; // Default from the SPI library
    }
    spiConfig = SPISettings(clock, MSBFIRST, SPI_MODE0);

    //SPI.setMISO(SD_MISO_PIN); //todo: implement? bad interface
    //SPI.setMOSI(SD_MOSI_PIN);
    //SPI.setSCLK(SD_SCK_PIN);

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
    uint8_t returnByte = SPI.transfer(0xFF);
    return returnByte;
  }

  /**
   * @brief  Receive a number of bytes from the SPI port to a buffer
   *
   * @param  buf   Pointer to starting address of buffer to write to.
   * @param  nbyte Number of bytes to receive.
   * @return Nothing
   *
   * @details Uses DMA
   */
  void spiRead(uint8_t *buf, uint16_t nbyte) {
    if (nbyte == 0) return;
    memset(buf, 0xFF, nbyte);
    SPI.transfer(buf, nbyte);
  }

  /**
   * @brief  Send a single byte on SPI port
   *
   * @param  b Byte to send
   *
   * @details
   */
  void spiSend(uint8_t b) {
    SPI.transfer(b);
  }

  /**
   * @brief  Write token and then write from 512 byte buffer to SPI (for SD card)
   *
   * @param  buf   Pointer with buffer start address
   * @return Nothing
   *
   * @details Use DMA
   */
  void spiSendBlock(uint8_t token, const uint8_t *buf) {
    //uint8_t rxBuf[512];
    //SPI.transfer(token);
    SPI.transfer((uint8_t*)buf, 512); //implement? bad interface
  }

#endif // SOFTWARE_SPI

#endif // __PLAT_RP2040__
