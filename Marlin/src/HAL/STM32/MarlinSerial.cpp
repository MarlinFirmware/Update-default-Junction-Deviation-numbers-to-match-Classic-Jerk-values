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
#include "MarlinSerial.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#ifndef USART4
  #define USART4 UART4
#endif
#ifndef USART5
  #define USART5 UART5
#endif
#ifndef USART6
  #define USART6 UART6
#endif
#ifndef USART7
  #define USART7 UART7
#endif
#ifndef USART8
  #define USART8 UART8
#endif
#ifndef USART9
  #define USART9 UART9
#endif

#if ENABLED(SERIAL_DMA)
  #define DECLARE_SERIAL_PORT(ser_num) \
    MSerialT MSerial ## ser_num (true, USART ## ser_num);
#else
  #define DECLARE_SERIAL_PORT(ser_num) \
    void _rx_complete_irq_ ## ser_num (serial_t * obj); \
    MSerialT MSerial ## ser_num (true, USART ## ser_num, &_rx_complete_irq_ ## ser_num); \
    void _rx_complete_irq_ ## ser_num (serial_t * obj) { MSerial ## ser_num ._rx_complete_irq(obj); }
#endif

#if USING_HW_SERIAL1
  DECLARE_SERIAL_PORT(1)
#endif
#if USING_HW_SERIAL2
  DECLARE_SERIAL_PORT(2)
#endif
#if USING_HW_SERIAL3
  DECLARE_SERIAL_PORT(3)
#endif
#if USING_HW_SERIAL4
  DECLARE_SERIAL_PORT(4)
#endif
#if USING_HW_SERIAL5
  DECLARE_SERIAL_PORT(5)
#endif
#if USING_HW_SERIAL6
  DECLARE_SERIAL_PORT(6)
#endif
#if USING_HW_SERIAL7
  DECLARE_SERIAL_PORT(7)
#endif
#if USING_HW_SERIAL8
  DECLARE_SERIAL_PORT(8)
#endif
#if USING_HW_SERIAL9
  DECLARE_SERIAL_PORT(9)
#endif
#if USING_HW_SERIAL10
  DECLARE_SERIAL_PORT(10)
#endif
#if USING_HW_SERIALLP1
  DECLARE_SERIAL_PORT(LP1)
#endif

void MarlinSerial::begin(unsigned long baud, uint8_t config) {
  #if ENABLED(SERIAL_DMA)
    HAL_HardwareSerial::begin(baud, config);
  #else
    HardwareSerial::begin(baud, config);
    // Replace the IRQ callback with the one we have defined
    TERN_(EMERGENCY_PARSER, _serial.rx_callback = _rx_callback);
  #endif
}

#if DISABLED(SERIAL_DMA)

  // This function Copyright (c) 2006 Nicholas Zambetti.
  void MarlinSerial::_rx_complete_irq(serial_t *obj) {
    // No Parity error, read byte and store it in the buffer if there is room
    unsigned char c;
    if (uart_getc(obj, &c) == 0) {

      rx_buffer_index_t i = (unsigned int)(obj->rx_head + 1) % SERIAL_RX_BUFFER_SIZE;

      // If tail overlaps head the buffer is overflowed
      // so don't write the character or advance the head.
      if (i != obj->rx_tail) {
        obj->rx_buff[obj->rx_head] = c;
        obj->rx_head = i;
      }

      #if ENABLED(EMERGENCY_PARSER)
        emergency_parser.update(static_cast<MSerialT*>(this)->emergency_state, c);
      #endif
    }
  }

#endif // !SERIAL_DMA

#endif // HAL_STM32
