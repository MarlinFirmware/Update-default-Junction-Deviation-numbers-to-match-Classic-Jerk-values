/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 * Copyright (c) 2017 Victor Perez
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

//
// HAL_HardwareSerial Class. Adapted from Arduino HardwareSerial.
//

#if RX_BUFFER_SIZE == 0
  #undef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif

#if TX_BUFFER_SIZE == 0
  #undef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

typedef struct {
  USART_TypeDef * uart;
  uint32_t dma_rcc;
  #if ANY(STM32F0xx, STM32F1xx)           // F0 / F1
    DMA_TypeDef * dma_controller;
    DMA_Channel_TypeDef * dma_channelRX;
  #else                                   // F2 / F4 / F7
    uint32_t dma_channel;
    DMA_Stream_TypeDef * dma_streamRX;
  #endif
} DMA_CFG;

class HAL_HardwareSerial : public Stream {
  protected:
    // Don't put any members after these buffers, since only the first
    // 32 bytes of this struct can be accessed quickly using the ldd instruction.
    unsigned char _rx_buffer[RX_BUFFER_SIZE];
    unsigned char _tx_buffer[TX_BUFFER_SIZE];

    serial_t _serial;

  public:
    HAL_HardwareSerial(void *peripheral);
    void begin(unsigned long, uint8_t);
    void end();
    virtual int available();
    virtual int read();
    virtual int peek();
    virtual size_t write(uint8_t);
    virtual void flush();
    operator bool() { return true; }

    void setRx(uint32_t _rx);
    void setTx(uint32_t _tx);

    static int _tx_complete_irq(serial_t *obj); // Interrupt handler

  private:
    uint8_t _uart_index;
    bool    _rx_enabled;
    uint8_t _config;
    unsigned long _baud;
    void init(PinName _rx, PinName _tx);
    void update_rx_head();
    DMA_CFG RX_DMA;
    void Serial_DMA_Read_Enable();
};
