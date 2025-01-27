/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../core/serial_hook.h"
#include <drivers/usart/Usart.h>

#define SERIAL_INDEX_MIN 1
#define SERIAL_INDEX_MAX 4
#include "../shared/serial_ports.h"

#if defined(LCD_SERIAL_PORT) && ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
  #define LCD_SERIAL_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
#endif

// Optionally set uart IRQ priority to reduce overflow errors
//#define UART_RX_IRQ_PRIO 1
//#define UART_TX_IRQ_PRIO 1
//#define UART_RX_DMA_IRQ_PRIO 1

struct MarlinSerial : public Usart {
  MarlinSerial(
    struct usart_config_t *usart_device,
    gpio_pin_t tx_pin,
    gpio_pin_t rx_pin
    #if ENABLED(SERIAL_DMA)
    , M4_DMA_TypeDef *dma_unit = nullptr,
    en_dma_channel_t rx_dma_channel = DmaCh0
    #endif
  ) : Usart(usart_device, tx_pin, rx_pin) {
    #if ENABLED(SERIAL_DMA)
      if (dma_unit != nullptr) {
        enableRxDma(dma_unit, rx_dma_channel);
      }
    #endif
  }

  #if defined(UART_RX_IRQ_PRIO) || defined(UART_TX_IRQ_PRIO) || defined(UART_RX_DMA_IRQ_PRIO)
    void setPriority() {
      #if defined(UART_RX_IRQ_PRIO)
        NVIC_SetPriority(c_dev()->interrupts.rx_data_available.interrupt_number, UART_RX_IRQ_PRIO);
        NVIC_SetPriority(c_dev()->interrupts.rx_error.interrupt_number, UART_RX_IRQ_PRIO);
      #endif

      #if defined(UART_TX_IRQ_PRIO)
        NVIC_SetPriority(c_dev()->interrupts.tx_buffer_empty.interrupt_number, UART_TX_IRQ_PRIO);
        NVIC_SetPriority(c_dev()->interrupts.tx_complete.interrupt_number, UART_TX_IRQ_PRIO);
      #endif

      #if defined(UART_RX_DMA_IRQ_PRIO) && ENABLED(SERIAL_DMA)
        NVIC_SetPriority(c_dev()->dma.rx.rx_data_available_dma_btc.interrupt_number, UART_RX_DMA_IRQ_PRIO);
      #endif
    }

    void begin(uint32_t baud) {
      Usart::begin(baud);
      setPriority();
    }

    void begin(uint32_t baud, uint8_t config) {
      Usart::begin(baud, config);
      setPriority();
    }

    void begin(uint32_t baud, const stc_usart_uart_init_t *config, const bool rxNoiseFilter = true) {
      Usart::begin(baud, config, rxNoiseFilter);
      setPriority();
    }
  #endif // UART_RX_IRQ_PRIO || UART_TX_IRQ_PRIO || UART_RX_DMA_IRQ_PRIO
};

typedef Serial1Class<MarlinSerial> MSerialT;

extern MSerialT MSerial1;
extern MSerialT MSerial2;
