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

#ifdef __STM32F1__

#include "../../../inc/MarlinConfig.h"

#if HAS_SPI_TFT

#include "tft_spi.h"

SPIClass TFT_SPI::SPIx(TFT_SPI_DEVICE);

void TFT_SPI::init() {
  #if PIN_EXISTS(TFT_RESET)
    OUT_WRITE(TFT_RESET_PIN, HIGH);
    delay(100);
  #endif

  #if PIN_EXISTS(TFT_BACKLIGHT)
    OUT_WRITE(TFT_BACKLIGHT_PIN, HIGH);
  #endif

  OUT_WRITE(TFT_DC_PIN, HIGH);
  OUT_WRITE(TFT_CS_PIN, HIGH);

  /**
   * STM32F1 APB2 = 72MHz, APB1 = 36MHz, max SPI speed of this MCU if 18Mhz
   * STM32F1 has 3 SPI ports, SPI1 in APB2, SPI2/SPI3 in APB1
   * so the minimum prescale of SPI1 is DIV4, SPI2/SPI3 is DIV2
   */
  #if TFT_SPI_DEVICE == 1
    #define SPI_CLOCK_MAX SPI_CLOCK_DIV4
  #else
    #define SPI_CLOCK_MAX SPI_CLOCK_DIV2
  #endif
  uint8_t  clock;
  uint8_t spiRate = SPI_FULL_SPEED;
  switch (spiRate) {
    case SPI_FULL_SPEED:    clock = SPI_CLOCK_MAX ;  break;
    case SPI_HALF_SPEED:    clock = SPI_CLOCK_DIV4 ; break;
    case SPI_QUARTER_SPEED: clock = SPI_CLOCK_DIV8 ; break;
    case SPI_EIGHTH_SPEED:  clock = SPI_CLOCK_DIV16; break;
    case SPI_SPEED_5:       clock = SPI_CLOCK_DIV32; break;
    case SPI_SPEED_6:       clock = SPI_CLOCK_DIV64; break;
    default:                clock = SPI_CLOCK_DIV2;  // Default from the SPI library
  }
  SPIx.setModule(TFT_SPI_DEVICE);
  SPIx.setClockDivider(clock);
  SPIx.setBitOrder(MSBFIRST);
  SPIx.setDataMode(SPI_MODE0);
}

void TFT_SPI::dataTransferBegin(uint16_t dataSize) {
  SPIx.setDataSize(dataSize);
  SPIx.begin();
  WRITE(TFT_CS_PIN, LOW);
}

#ifdef TFT_DEFAULT_DRIVER
  #include "../../../lcd/tft_io/tft_ids.h"
#endif

uint32_t TFT_SPI::getID() {
  uint32_t id;
  id = readID(LCD_READ_ID);
  if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) {
    id = readID(LCD_READ_ID4);
    #ifdef TFT_DEFAULT_DRIVER
      if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF)
        id = TFT_DEFAULT_DRIVER;
    #endif
   }
  return id;
}

uint32_t TFT_SPI::readID(const uint16_t inReg) {
  uint32_t data = 0;

  #if PIN_EXISTS(TFT_MISO)
    SPIx.setClockDivider(SPI_CLOCK_DIV16);
    dataTransferBegin(DATASIZE_8BIT);
    writeReg(inReg);

    for (uint_fast8_t i = 0; i < 4; ++i) {
      uint8_t d;
      SPIx.read(&d, 1);
      data = (data << 8) | d;
    }

    dataTransferEnd();
    SPIx.setClockDivider(SPI_CLOCK_MAX);
  #endif

  return data >> 7;
}

bool TFT_SPI::isBusy() {
  #define __IS_DMA_CONFIGURED(__DMAx__, __CHx__)   (dma_channel_regs(__DMAx__, __CHx__)->CPAR != 0)

  if (!__IS_DMA_CONFIGURED(DMAx, DMA_CHx)) return false;

  if (dma_get_isr_bits(DMAx, DMA_CHx) & DMA_ISR_TEIF) {
    // You should not be here - DMA transfer error flag is set
    // Abort DMA transfer and release SPI
  }
  else {
    // Check if DMA transfer completed flag is set
    if (!(dma_get_isr_bits(DMAx, DMA_CHx) & DMA_ISR_TCIF)) return true;
    // Check if SPI TX butter is empty and SPI is idle
    if (!(SPIdev->regs->SR & SPI_SR_TXE) || (SPIdev->regs->SR & SPI_SR_BSY)) return true;
  }

  abort();
  return false;
}

void TFT_SPI::abort() {
  dma_channel_reg_map *channel_regs = dma_channel_regs(DMAx, DMA_CHx);

  dma_disable(DMAx, DMA_CHx); // Abort DMA transfer if any
  spi_tx_dma_disable(SPIdev);

  // Deconfigure DMA
  channel_regs->CCR   = 0U;
  channel_regs->CNDTR = 0U;
  channel_regs->CMAR  = 0U;
  channel_regs->CPAR  = 0U;

  dataTransferEnd();
}

void TFT_SPI::transmit(uint16_t data) { SPIx.send(data); }

void TFT_SPI::transmitDMA(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  dataTransferBegin();
  SPIx.dmaSendAsync(data, count, memoryIncrease == DMA_MINC_ENABLE);

  TERN_(TFT_SHARED_IO, while (isBusy()));
}

void TFT_SPI::transmit(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  WRITE(TFT_DC_PIN, HIGH);
  dataTransferBegin();
  SPIx.dmaSend(data, count, memoryIncrease == DMA_MINC_ENABLE);
  dataTransferEnd();
}

#endif // HAS_SPI_TFT

#endif // __STM32F1__
