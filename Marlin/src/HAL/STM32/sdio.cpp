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

#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"

#if ENABLED(ONBOARD_SDIO)

#include "sdio.h"

#include <stdint.h>
#include <stdbool.h>

#if defined(STM32F103xE) || defined(STM32F103xG)
  #include <stm32f1xx_hal_rcc_ex.h>
  #include <stm32f1xx_hal_sd.h>
#elif defined(STM32F4xx)
  #include <stm32f4xx_hal_rcc.h>
  #include <stm32f4xx_hal_dma.h>
  #include <stm32f4xx_hal_gpio.h>
  #include <stm32f4xx_hal_sd.h>
#elif defined(STM32F7xx)
  #include <stm32f7xx_hal_rcc.h>
  #include <stm32f7xx_hal_dma.h>
  #include <stm32f7xx_hal_gpio.h>
  #include <stm32f7xx_hal_sd.h>
#elif defined(STM32H7xx)
  #define SDIO_FOR_STM32H7
  #include <stm32h7xx_hal_rcc.h>
  #include <stm32h7xx_hal_dma.h>
  #include <stm32h7xx_hal_gpio.h>
  #include <stm32h7xx_hal_sd.h>
#else
  #error "SDIO is only supported with STM32F103xE, STM32F103xG, STM32F4xx, STM32F7xx, and STM32H7xx."
#endif

// SDIO Max Clock (naming from STM Manual, don't change)
#define SDIOCLK 48000000

// Target Clock, configurable. Default is 18MHz, from STM32F1
#ifndef SDIO_CLOCK
  #define SDIO_CLOCK 18000000 // 18 MHz
#endif

SD_HandleTypeDef hsd;  // SDIO structure

static uint32_t clock_to_divider(uint32_t clk) {
  #ifdef SDIO_FOR_STM32H7
    // SDMMC_CK frequency = sdmmc_ker_ck / [2 * CLKDIV].
    uint32_t sdmmc_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC);
    return sdmmc_clk / (2U * SDIO_CLOCK) + (sdmmc_clk % (2U * SDIO_CLOCK) != 0);
  #else
    // limit the SDIO master clock to 8/3 of PCLK2. See STM32 Manuals
    // Also limited to no more than 48Mhz (SDIOCLK).
    const uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    clk = min(clk, (uint32_t)(pclk2 * 8 / 3));
    clk = min(clk, (uint32_t)SDIOCLK);
    // Round up divider, so we don't run the card over the speed supported,
    // and subtract by 2, because STM32 will add 2, as written in the manual:
    // SDIO_CK frequency = SDIOCLK / [CLKDIV + 2]
    return pclk2 / clk + (pclk2 % clk != 0) - 2;
  #endif
}

// Start the SDIO clock
void HAL_SD_MspInit(SD_HandleTypeDef *hsd) {
  UNUSED(hsd);
  #ifdef SDIO_FOR_STM32H7
    pinmap_pinout(PC_12, PinMap_SD);
    pinmap_pinout(PD_2,  PinMap_SD);
    pinmap_pinout(PC_8,  PinMap_SD);
    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3)  // Define D1-D3 only for 4-bit wide SDIO bus
      pinmap_pinout(PC_9,  PinMap_SD);
      pinmap_pinout(PC_10, PinMap_SD);
      pinmap_pinout(PC_11, PinMap_SD);
    #endif
    __HAL_RCC_SDMMC1_CLK_ENABLE();
    HAL_NVIC_EnableIRQ(SDMMC1_IRQn);
  #else
    __HAL_RCC_SDIO_CLK_ENABLE();
  #endif
}

#ifdef SDIO_FOR_STM32H7

  #define SD_TIMEOUT              1000 // ms

  extern "C" void SDMMC1_IRQHandler(void) { HAL_SD_IRQHandler(&hsd); }

  uint8_t waitingRxCplt = 0, waitingTxCplt = 0;
  void HAL_SD_TxCpltCallback(SD_HandleTypeDef *hsdio) { waitingTxCplt = 0; }
  void HAL_SD_RxCpltCallback(SD_HandleTypeDef *hsdio) { waitingRxCplt = 0; }

  void HAL_SD_MspDeInit(SD_HandleTypeDef *hsd) {
    __HAL_RCC_SDMMC1_FORCE_RESET();   delay(10);
    __HAL_RCC_SDMMC1_RELEASE_RESET(); delay(10);
  }

  bool SDIO_Init() {
    HAL_StatusTypeDef sd_state = HAL_OK;
    if (hsd.Instance == SDMMC1) HAL_SD_DeInit(&hsd);

    // HAL SD initialization
    hsd.Instance = SDMMC1;
    hsd.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
    hsd.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
    hsd.Init.BusWide = SDMMC_BUS_WIDE_1B;
    hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
    hsd.Init.ClockDiv = clock_to_divider(SDIO_CLOCK);
    sd_state = HAL_SD_Init(&hsd);

    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3)
      if (sd_state == HAL_OK)
        sd_state = HAL_SD_ConfigWideBusOperation(&hsd, SDMMC_BUS_WIDE_4B);
    #endif

    return (sd_state == HAL_OK);
  }

#else // !SDIO_FOR_STM32H7

  #define SD_TIMEOUT               500 // ms

  // SDIO retries, configurable. Default is 3, from STM32F1
  #ifndef SDIO_READ_RETRIES
    #define SDIO_READ_RETRIES 3
  #endif

  // F4 supports one DMA for RX and another for TX, but Marlin will never
  // do read and write at same time, so we use the same DMA for both.
  DMA_HandleTypeDef hdma_sdio;

  #ifdef STM32F1xx
    #define DMA_IRQ_HANDLER DMA2_Channel4_5_IRQHandler
  #elif defined(STM32F4xx)
    #define DMA_IRQ_HANDLER DMA2_Stream3_IRQHandler
  #else
    #error "Unknown STM32 architecture."
  #endif

  extern "C" void SDIO_IRQHandler(void) { HAL_SD_IRQHandler(&hsd); }
  extern "C" void DMA_IRQ_HANDLER(void) { HAL_DMA_IRQHandler(&hdma_sdio); }

  /*
    SDIO_INIT_CLK_DIV is 118
    SDIO clock frequency is 48MHz / (TRANSFER_CLOCK_DIV + 2)
    SDIO init clock frequency should not exceed 400kHz = 48MHz / (118 + 2)

    Default TRANSFER_CLOCK_DIV is 2 (118 / 40)
    Default SDIO clock frequency is 48MHz / (2 + 2) = 12 MHz
    This might be too fast for stable SDIO operations

    MKS Robin SDIO seems stable with BusWide 1bit and ClockDiv 8 (i.e., 4.8MHz SDIO clock frequency)
    More testing is required as there are clearly some 4bit init problems.
  */

  void go_to_transfer_speed() {
    /* Default SDIO peripheral configuration for SD card initialization */
    hsd.Init.ClockEdge           = hsd.Init.ClockEdge;
    hsd.Init.ClockBypass         = hsd.Init.ClockBypass;
    hsd.Init.ClockPowerSave      = hsd.Init.ClockPowerSave;
    hsd.Init.BusWide             = hsd.Init.BusWide;
    hsd.Init.HardwareFlowControl = hsd.Init.HardwareFlowControl;
    hsd.Init.ClockDiv            = clock_to_divider(SDIO_CLOCK);

    /* Initialize SDIO peripheral interface with default configuration */
    SDIO_Init(hsd.Instance, hsd.Init);
  }

  void SD_LowLevel_Init() {
    uint32_t tempreg;

    // Enable GPIO clocks
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef  GPIO_InitStruct;

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = 1;  // GPIO_NOPULL
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    #if DISABLED(STM32F1xx)
      GPIO_InitStruct.Alternate = GPIO_AF12_SDIO;
    #endif

    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_12;  // D0 & SCK
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3)  // define D1-D3 only if have a four bit wide SDIO bus
      GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11;  // D1-D3
      HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    #endif

    // Configure PD.02 CMD line
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    // Setup DMA
    #ifdef STM32F1xx
      hdma_sdio.Init.Mode = DMA_NORMAL;
      hdma_sdio.Instance = DMA2_Channel4;
      HAL_NVIC_EnableIRQ(DMA2_Channel4_5_IRQn);
    #elif defined(STM32F4xx)
      hdma_sdio.Init.Mode = DMA_PFCTRL;
      hdma_sdio.Instance = DMA2_Stream3;
      hdma_sdio.Init.Channel = DMA_CHANNEL_4;
      hdma_sdio.Init.FIFOMode = DMA_FIFOMODE_ENABLE;
      hdma_sdio.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
      hdma_sdio.Init.MemBurst = DMA_MBURST_INC4;
      hdma_sdio.Init.PeriphBurst = DMA_PBURST_INC4;
      HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
    #endif
    HAL_NVIC_EnableIRQ(SDIO_IRQn);
    hdma_sdio.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdio.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_sdio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_sdio.Init.Priority = DMA_PRIORITY_MEDIUM;
    __HAL_LINKDMA(&hsd, hdmarx, hdma_sdio);
    __HAL_LINKDMA(&hsd, hdmatx, hdma_sdio);

    #ifdef STM32F1xx
      __HAL_RCC_SDIO_CLK_ENABLE();
      __HAL_RCC_DMA2_CLK_ENABLE();
    #else
      __HAL_RCC_SDIO_FORCE_RESET();   delay(2);
      __HAL_RCC_SDIO_RELEASE_RESET(); delay(2);
      __HAL_RCC_SDIO_CLK_ENABLE();

      __HAL_RCC_DMA2_FORCE_RESET();   delay(2);
      __HAL_RCC_DMA2_RELEASE_RESET(); delay(2);
      __HAL_RCC_DMA2_CLK_ENABLE();
    #endif

    // Initialize the SDIO (with initial <400Khz Clock)
    tempreg = 0                   // Reset value
            | SDIO_CLKCR_CLKEN    // Clock enabled
            | SDIO_INIT_CLK_DIV;  // Clock Divider. Clock = 48000 / (118 + 2) = 400Khz
                                  // Keep the rest at 0 => HW_Flow Disabled, Rising Clock Edge, Disable CLK ByPass, Bus Width = 0, Power save Disable
    SDIO->CLKCR = tempreg;

    // Power up the SDIO
    SDIO_PowerState_ON(SDIO);
    hsd.Instance = SDIO;
  }

  bool SDIO_Init() {
    uint8_t retryCnt = SDIO_READ_RETRIES;

    bool status;
    hsd.Instance = SDIO;
    hsd.State = HAL_SD_STATE_RESET;

    SD_LowLevel_Init();

    uint8_t retry_Cnt = retryCnt;
    for (;;) {
      hal.watchdog_refresh();
      status = (bool) HAL_SD_Init(&hsd);
      if (!status) break;
      if (!--retry_Cnt) return false;   // return failing status if retries are exhausted
    }

    go_to_transfer_speed();

    hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_ENABLE;
    hsd.Init.ClockDiv = 8;

    #if PINS_EXIST(SDIO_D1, SDIO_D2, SDIO_D3) // go to 4 bit wide mode if pins are defined
      retry_Cnt = retryCnt;
      for (;;) {
        hal.watchdog_refresh();
        if (!HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B)) break;  // some cards are only 1 bit wide so a pass here is not required
        if (!--retry_Cnt) break;
      }
      if (!retry_Cnt) {  // wide bus failed, go back to one bit wide mode
        hsd.State = (HAL_SD_StateTypeDef) 0;  // HAL_SD_STATE_RESET
        SD_LowLevel_Init();
        retry_Cnt = retryCnt;
        for (;;) {
          hal.watchdog_refresh();
          status = (bool) HAL_SD_Init(&hsd);
          if (!status) break;
          if (!--retry_Cnt) return false;   // return failing status if retries are exhausted
        }
        go_to_transfer_speed();
      }
    #endif

    return true;
  }

  /**
   * @brief Read or Write a block
   * @details Read or Write a block with SDIO
   *
   * @param block The block index
   * @param src The data buffer source for a write
   * @param dst The data buffer destination for a read
   *
   * @return true on success
   */
  static bool SDIO_ReadWriteBlock_DMA(uint32_t block, const uint8_t *src, uint8_t *dst) {
    if (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) return false;

    hal.watchdog_refresh();

    HAL_StatusTypeDef ret;
    if (src) {
      hdma_sdio.Init.Direction = DMA_MEMORY_TO_PERIPH;
      HAL_DMA_Init(&hdma_sdio);
      ret = HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t*)src, block, 1);
    }
    else {
      hdma_sdio.Init.Direction = DMA_PERIPH_TO_MEMORY;
      HAL_DMA_Init(&hdma_sdio);
      ret = HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)dst, block, 1);
    }

    if (ret != HAL_OK) {
      HAL_DMA_Abort_IT(&hdma_sdio);
      HAL_DMA_DeInit(&hdma_sdio);
      return false;
    }

    millis_t timeout = millis() + SD_TIMEOUT;
    // Wait the transfer
    while (hsd.State != HAL_SD_STATE_READY) {
      if (ELAPSED(millis(), timeout)) {
        HAL_DMA_Abort_IT(&hdma_sdio);
        HAL_DMA_DeInit(&hdma_sdio);
        return false;
      }
    }

    while (__HAL_DMA_GET_FLAG(&hdma_sdio, __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_sdio)) != 0
        || __HAL_DMA_GET_FLAG(&hdma_sdio, __HAL_DMA_GET_TE_FLAG_INDEX(&hdma_sdio)) != 0) { /* nada */ }

    HAL_DMA_Abort_IT(&hdma_sdio);
    HAL_DMA_DeInit(&hdma_sdio);

    timeout = millis() + SD_TIMEOUT;
    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER) if (ELAPSED(millis(), timeout)) return false;

    return true;
  }

#endif // !SDIO_FOR_STM32H7

/**
 * @brief Read a block
 * @details Read a block from media with SDIO
 *
 * @param block The block index
 * @param src The block buffer
 *
 * @return true on success
 */
bool SDIO_ReadBlock(uint32_t block, uint8_t *dst) {
  #ifdef SDIO_FOR_STM32H7

    uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;

    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
      if (HAL_GetTick() >= timeout) return false;

    waitingRxCplt = 1;
    if (HAL_SD_ReadBlocks_DMA(&hsd, (uint8_t*)dst, block, 1) != HAL_OK)
      return false;

    timeout = HAL_GetTick() + SD_TIMEOUT;
    while (waitingRxCplt)
      if (HAL_GetTick() >= timeout) return false;

    return true;

  #else

    uint8_t retries = SDIO_READ_RETRIES;
    while (retries--) if (SDIO_ReadWriteBlock_DMA(block, nullptr, dst)) return true;
    return false;

  #endif
}

/**
 * @brief Write a block
 * @details Write a block to media with SDIO
 *
 * @param block The block index
 * @param src The block data
 *
 * @return true on success
 */
bool SDIO_WriteBlock(uint32_t block, const uint8_t *src) {
  #ifdef SDIO_FOR_STM32H7

    uint32_t timeout = HAL_GetTick() + SD_TIMEOUT;

    while (HAL_SD_GetCardState(&hsd) != HAL_SD_CARD_TRANSFER)
      if (HAL_GetTick() >= timeout) return false;

    waitingTxCplt = 1;
    if (HAL_SD_WriteBlocks_DMA(&hsd, (uint8_t*)src, block, 1) != HAL_OK)
      return false;

    timeout = HAL_GetTick() + SD_TIMEOUT;
    while (waitingTxCplt)
      if (HAL_GetTick() >= timeout) return false;

    return true;

  #else

    uint8_t retries = SDIO_READ_RETRIES;
    while (retries--) {
      if (SDIO_ReadWriteBlock_DMA(block, src, nullptr)) return true;
      delay(10);
    }
    return false;

  #endif
}

bool SDIO_IsReady() {
  return hsd.State == HAL_SD_STATE_READY;
}

uint32_t SDIO_GetCardSize() {
  return (uint32_t)(hsd.SdCard.BlockNbr) * (hsd.SdCard.BlockSize);
}

#endif // ONBOARD_SDIO
#endif // HAL_STM32
