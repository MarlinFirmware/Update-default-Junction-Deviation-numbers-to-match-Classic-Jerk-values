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

#include "../../platforms.h"

#ifdef HAL_STM32

#include "../../../inc/MarlinConfig.h"

#if HAS_SPI_TFT

#include "tft_spi.h"
#include "pinconfig.h"

//#define DEBUG_TFT_IO
#define DEBUG_OUT ENABLED(DEBUG_TFT_IO)
#include "../../../core/debug_out.h"

SPI_HandleTypeDef TFT_SPI::SPIx;
DMA_HandleTypeDef TFT_SPI::DMAtx;

void TFT_SPI::init() {
  SPI_TypeDef *spiInstance;

  OUT_WRITE(TFT_A0_PIN, HIGH);
  OUT_WRITE(TFT_CS_PIN, HIGH);

  if ((spiInstance = (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(TFT_SCK_PIN),  PinMap_SPI_SCLK)) == NP) return;
  if (spiInstance != (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(TFT_MOSI_PIN), PinMap_SPI_MOSI)) return;

  #if PIN_EXISTS(TFT_MISO)
    // Check these pins in code because they are sometimes defined as analog pin references
    if ((TFT_MISO_PIN != TFT_MOSI_PIN) && (spiInstance != (SPI_TypeDef *)pinmap_peripheral(digitalPinToPinName(TFT_MISO_PIN), PinMap_SPI_MISO))) return;
  #endif

  SPIx.Instance                = spiInstance;
  SPIx.State                   = HAL_SPI_STATE_RESET;
  SPIx.Init.NSS                = SPI_NSS_SOFT;
  SPIx.Init.Mode               = SPI_MODE_MASTER;
  SPIx.Init.Direction          = (TFT_MISO_PIN == TFT_MOSI_PIN) ? SPI_DIRECTION_1LINE : SPI_DIRECTION_2LINES;
  SPIx.Init.CLKPhase           = SPI_PHASE_1EDGE;
  SPIx.Init.CLKPolarity        = SPI_POLARITY_LOW;
  SPIx.Init.DataSize           = SPI_DATASIZE_8BIT;
  SPIx.Init.FirstBit           = SPI_FIRSTBIT_MSB;
  SPIx.Init.TIMode             = SPI_TIMODE_DISABLE;
  SPIx.Init.CRCCalculation     = SPI_CRCCALCULATION_DISABLE;
  SPIx.Init.CRCPolynomial      = 10;

  #ifndef STM32H7xx
    SPIx.Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_2; // 18 MBit/s for F103, 21 MBit/s for F407, 25 MBit/s for F411
  #else
    SPIx.Init.BaudRatePrescaler       = SPI_BAUDRATEPRESCALER_4; // 20 MBit/s for H743
    SPIx.Init.NSSPMode                = SPI_NSS_PULSE_ENABLE;
    SPIx.Init.NSSPolarity             = SPI_NSS_POLARITY_LOW;
    SPIx.Init.FifoThreshold           = SPI_FIFO_THRESHOLD_01DATA;
    SPIx.Init.MasterSSIdleness        = SPI_MASTER_SS_IDLENESS_00CYCLE;
    SPIx.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    SPIx.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    SPIx.Init.MasterKeepIOState       = SPI_MASTER_KEEP_IO_STATE_ENABLE;
    SPIx.Init.IOSwap                  = SPI_IO_SWAP_DISABLE;
  #endif

  pinmap_pinout(digitalPinToPinName(TFT_SCK_PIN), PinMap_SPI_SCLK);
  pinmap_pinout(digitalPinToPinName(TFT_MOSI_PIN), PinMap_SPI_MOSI);
  #if PIN_EXISTS(TFT_MISO)
    // Check these pins in code because they are sometimes defined as analog pin references
    if (TFT_MISO_PIN != TFT_MOSI_PIN) pinmap_pinout(digitalPinToPinName(TFT_MISO_PIN), PinMap_SPI_MISO);
  #endif

  //pin_PullConfig(get_GPIO_Port(STM_PORT(digitalPinToPinName(TFT_SCK_PIN))), STM_LL_GPIO_PIN(digitalPinToPinName(TFT_SCK_PIN)), GPIO_PULLDOWN);

  #ifdef SPI1_BASE
    if (SPIx.Instance == SPI1) {
      __HAL_RCC_SPI1_CLK_ENABLE();
      #ifdef STM32F1xx
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Channel3;
        SPIx.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_4; // SPI1 clock on F1 and F4 is two times faster than SPI2 and SPI3 clock
      #elif defined(STM32F4xx)
        __HAL_RCC_DMA2_CLK_ENABLE();
        DMAtx.Instance = DMA2_Stream3;
        DMAtx.Init.Channel = DMA_CHANNEL_3;
        SPIx.Init.BaudRatePrescaler  = SPI_BAUDRATEPRESCALER_4; // SPI1 clock on F1 and F4 is two times faster than SPI2 and SPI3 clock
      #elif defined(STM32H7xx)
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Stream4;
        DMAtx.Init.Request = DMA_REQUEST_SPI1_TX;
      #endif
    }
  #endif
  #ifdef SPI2_BASE
    if (SPIx.Instance == SPI2) {
      __HAL_RCC_SPI2_CLK_ENABLE();
      #ifdef STM32F1xx
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Channel5;
      #elif defined(STM32F4xx)
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Stream4;
        DMAtx.Init.Channel = DMA_CHANNEL_0;
      #elif defined(STM32H7xx)
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Stream4;
        DMAtx.Init.Request = DMA_REQUEST_SPI2_TX;
      #endif
    }
  #endif
  #ifdef SPI3_BASE
    if (SPIx.Instance == SPI3) {
      __HAL_RCC_SPI3_CLK_ENABLE();
      #ifdef STM32F1xx
        __HAL_RCC_DMA2_CLK_ENABLE();
        DMAtx.Instance = DMA2_Channel2;
      #elif defined(STM32F4xx)
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Stream5;
        DMAtx.Init.Channel = DMA_CHANNEL_0;
      #elif defined(STM32H7xx)
        __HAL_RCC_DMA1_CLK_ENABLE();
        DMAtx.Instance = DMA1_Stream4;
        DMAtx.Init.Request = DMA_REQUEST_SPI3_TX;
      #endif
    }
  #endif

  DMAtx.Init.Direction = DMA_MEMORY_TO_PERIPH;
  DMAtx.Init.PeriphInc = DMA_PINC_DISABLE;
  DMAtx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
  DMAtx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
  DMAtx.Init.Mode = DMA_NORMAL;
  DMAtx.Init.Priority = DMA_PRIORITY_LOW;
  #if ANY(STM32F4xx, STM32H7xx)
    DMAtx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  #endif
}

void TFT_SPI::dataTransferBegin(uint16_t dataSize) {
  SPIx.Init.DataSize = dataSize;
  HAL_SPI_Init(&SPIx);
  WRITE(TFT_CS_PIN, LOW);
}

#include "../../../lcd/tft_io/tft_ids.h"

uint32_t TFT_SPI::getID() {
  DEBUG_ECHOLNPGM("TFT_SPI::getID()");

  uint32_t id = readID(LCD_READ_ID);
  #if ENABLED(DEBUG_TFT_IO)
    char debug_register[3], debug_value[5];
    sprintf_P(debug_register, PSTR("%02X"), LCD_READ_ID);
    sprintf_P(debug_value, PSTR("%04X"), uint16_t(id));
    DEBUG_ECHOLNPGM("  readID(0x", debug_register, ") : 0x", debug_value);
  #endif

  if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) {
    id = readID(LCD_READ_ID4);
    #if ENABLED(DEBUG_TFT_IO)
      sprintf_P(debug_register, PSTR("%02X"), LCD_READ_ID4);
      sprintf_P(debug_value, PSTR("%04X"), uint16_t(id));
      DEBUG_ECHOLNPGM("  readID(0x", debug_register, ") : 0x", debug_value);
    #endif
  }

  #ifdef TFT_DEFAULT_DRIVER
    if ((id & 0xFFFF) == 0 || (id & 0xFFFF) == 0xFFFF) {
      id = TFT_DEFAULT_DRIVER;
      #if ENABLED(DEBUG_TFT_IO)
        sprintf_P(debug_value, PSTR("%04X"), uint16_t(id));
        DEBUG_ECHOLNPGM("  Fallback to TFT_DEFAULT_DRIVER : 0x", debug_value);
      #endif
    }
  #endif

  return id;
}

uint32_t TFT_SPI::readID(const uint16_t inReg) {
  uint32_t data = 0;
  #if PIN_EXISTS(TFT_MISO)
    uint32_t BaudRatePrescaler = SPIx.Init.BaudRatePrescaler;
    SPIx.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;

    dataTransferBegin(DATASIZE_8BIT);
    writeReg(inReg);

    if (SPIx.Init.Direction == SPI_DIRECTION_1LINE) SPI_1LINE_RX(&SPIx);

    #ifdef STM32H7xx
      for (uint32_t i = 0; i < 4; i++) {
        MODIFY_REG(SPIx.Instance->CR2, SPI_CR2_TSIZE, 1);
        __HAL_SPI_ENABLE(&SPIx);
        SET_BIT(SPIx.Instance->CR1, SPI_CR1_CSTART);

        if (SPIx.Init.Direction == SPI_DIRECTION_2LINES) SPIx.Instance->TXDR = 0;
        while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_EOT)) { /* nada */ }
        data = (data << 8) | SPIx.Instance->RXDR;
        __HAL_SPI_DISABLE(&SPIx);
        __HAL_SPI_CLEAR_EOTFLAG(&SPIx);
        __HAL_SPI_CLEAR_TXTFFLAG(&SPIx);
      }
    #else
      __HAL_SPI_ENABLE(&SPIx);
      for (uint32_t i = 0; i < 4; i++) {
        if (SPIx.Init.Direction == SPI_DIRECTION_2LINES) {
          while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_TXE)) { /* nada */ }
          SPIx.Instance->DR = 0;
        }
        while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_RXNE)) { /* nada */ }
        data = (data << 8) | SPIx.Instance->DR;
      }
    #endif

    dataTransferEnd();
    #if DISABLED(DEBUG_TFT_IO)
      SPIx.Init.BaudRatePrescaler = BaudRatePrescaler;
    #endif
  #endif

  DEBUG_ECHOLNPGM("  raw data : ", data);
  return data >> 7;
}

bool TFT_SPI::isBusy() {
  #ifdef STM32F1xx
    #define __IS_DMA_ENABLED(__HANDLE__)      ((__HANDLE__)->Instance->CCR & DMA_CCR_EN)
    #define __IS_DMA_CONFIGURED(__HANDLE__)   ((__HANDLE__)->Instance->CPAR != 0)
  #elif defined(STM32F4xx)
    #define __IS_DMA_ENABLED(__HANDLE__)      ((__HANDLE__)->Instance->CR & DMA_SxCR_EN)
    #define __IS_DMA_CONFIGURED(__HANDLE__)   ((__HANDLE__)->Instance->PAR != 0)
  #elif defined(STM32H7xx)
    #define __IS_DMA_ENABLED(__HANDLE__)      (((DMA_Stream_TypeDef *)((__HANDLE__)->Instance))->CR & DMA_SxCR_EN)
    #define __IS_DMA_CONFIGURED(__HANDLE__)   (((DMA_Stream_TypeDef *)((__HANDLE__)->Instance))->PAR != 0)
  #endif

  if (!__IS_DMA_CONFIGURED(&DMAtx)) return false;

  if (__HAL_DMA_GET_FLAG(&DMAtx, __HAL_DMA_GET_TE_FLAG_INDEX(&DMAtx))) {
    // You should not be here - DMA transfer error flag is set
    // Abort DMA transfer and release SPI
  }
  else {
    // Check if DMA transfer completed flag is set
    if (__HAL_DMA_GET_FLAG(&DMAtx, __HAL_DMA_GET_TC_FLAG_INDEX(&DMAtx)) == 0) return true;
    #ifdef STM32H7xx
      // Check if SPI data transfer is completed
      if (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_EOT)) return true;
    #else
      // Check if SPI transmit butter is empty and SPI is idle
      if ((!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_TXE)) || (__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_BSY))) return true;
    #endif
  }

  abort();
  return true;
}

void TFT_SPI::abort() {
  HAL_DMA_Abort(&DMAtx);  // Abort DMA transfer if any
  HAL_DMA_DeInit(&DMAtx);

  #ifdef STM32H7xx
    CLEAR_BIT(SPIx.Instance->CFG1, SPI_CFG1_TXDMAEN);
    __HAL_SPI_CLEAR_EOTFLAG(&SPIx);
    __HAL_SPI_CLEAR_TXTFFLAG(&SPIx);
  #else
    CLEAR_BIT(SPIx.Instance->CR2, SPI_CR2_TXDMAEN);
  #endif

  dataTransferEnd();  // Stop SPI and deselect CS
}

void TFT_SPI::transmit(uint16_t data) {
  if (SPIx.Init.Direction == SPI_DIRECTION_1LINE) SPI_1LINE_TX(&SPIx);

  #ifdef STM32H7xx
    MODIFY_REG(SPIx.Instance->CR2, SPI_CR2_TSIZE, 1);
    __HAL_SPI_ENABLE(&SPIx);
    SET_BIT(SPIx.Instance->CR1, SPI_CR1_CSTART);

    SPIx.Instance->TXDR = data;

    while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_SR_EOT)) { /* nada */ }

    __HAL_SPI_CLEAR_EOTFLAG(&SPIx);
    __HAL_SPI_CLEAR_TXTFFLAG(&SPIx);
    __HAL_SPI_DISABLE(&SPIx);
  #else
    __HAL_SPI_ENABLE(&SPIx);
    SPIx.Instance->DR = data;
    while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_TXE)) { /* nada */ } // Wait for data transfer to actually start
    while ( __HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_BSY)) { /* nada */ } // Wait until SPI is idle
  #endif

  if (SPIx.Init.Direction == SPI_DIRECTION_2LINES) __HAL_SPI_CLEAR_OVRFLAG(&SPIx);  // Clear overrun flag in 2 Lines communication mode because received data is not read
}

void TFT_SPI::transmitDMA(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  DMAtx.Init.MemInc = memoryIncrease;
  HAL_DMA_Init(&DMAtx);

  if (SPIx.Init.Direction == SPI_DIRECTION_1LINE) SPI_1LINE_TX(&SPIx);

  dataTransferBegin();

  #ifdef STM32H7xx
    HAL_DMA_Start(&DMAtx, (uint32_t)data, (uint32_t)&(SPIx.Instance->TXDR), count);

    CLEAR_BIT(SPIx.Instance->CFG1, SPI_CFG1_TXDMAEN);
    MODIFY_REG(SPIx.Instance->CR2, SPI_CR2_TSIZE, count);
    SET_BIT(SPIx.Instance->CFG1, SPI_CFG1_TXDMAEN);   // Enable Tx DMA Request
    __HAL_SPI_ENABLE(&SPIx);
    SET_BIT(SPIx.Instance->CR1, SPI_CR1_CSTART);
  #else
    HAL_DMA_Start(&DMAtx, (uint32_t)data, (uint32_t)&(SPIx.Instance->DR), count);

    __HAL_SPI_ENABLE(&SPIx);
    SET_BIT(SPIx.Instance->CR2, SPI_CR2_TXDMAEN);   // Enable Tx DMA Request
  #endif

  TERN_(TFT_SHARED_IO, while (isBusy()));
}

void TFT_SPI::transmit(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {
  transmitDMA(memoryIncrease, data, count);

  HAL_DMA_PollForTransfer(&DMAtx, HAL_DMA_FULL_TRANSFER, HAL_MAX_DELAY);
  #ifdef STM32H7xx
    while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_SR_EOT)) { /* nada */ }
  #else
    while (!__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_TXE)) { /* nada */ }
    while (__HAL_SPI_GET_FLAG(&SPIx, SPI_FLAG_BSY)) { /* nada */ }
  #endif
  abort();
}

#if ENABLED(USE_SPI_DMA_TC)
  void TFT_SPI::transmitDMA_IT(uint32_t memoryIncrease, uint16_t *data, uint16_t count) {

    DMAtx.Init.MemInc = memoryIncrease;
    HAL_DMA_Init(&DMAtx);

    if (SPIx.Init.Direction == SPI_DIRECTION_1LINE) SPI_1LINE_TX(&SPIx);

    dataTransferBegin();

    HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    HAL_DMA_Start_IT(&DMAtx, (uint32_t)data, (uint32_t)&(SPIx.Instance->DR), count);
    __HAL_SPI_ENABLE(&SPIx);

    SET_BIT(SPIx.Instance->CR2, SPI_CR2_TXDMAEN);   // Enable Tx DMA Request
  }

  extern "C" void DMA2_Stream3_IRQHandler(void) { TFT_SPI::DMA_IRQHandler(); }
#endif

#endif // HAS_SPI_TFT
#endif // HAL_STM32
