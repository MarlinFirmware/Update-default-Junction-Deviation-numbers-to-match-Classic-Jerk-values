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
 * MarlinSerial_Due.h - Hardware serial library for Arduino DUE
 * Copyright (c) 2017 Eduardo José Tagle. All right reserved
 * Based on MarlinSerial for AVR, copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 */

#include <WString.h>

#include "../../inc/MarlinConfigPre.h"
#include "../../core/types.h"
#include "../../core/serial_hook.h"

typedef ForwardSerial1Class< decltype(Serial) > DefaultSerial1;
typedef ForwardSerial1Class< decltype(Serial1) > DefaultSerial2;
typedef ForwardSerial1Class< decltype(Serial2) > DefaultSerial3;
typedef ForwardSerial1Class< decltype(Serial3) > DefaultSerial4;
extern DefaultSerial1 MSerial0;
extern DefaultSerial2 MSerial1;
extern DefaultSerial3 MSerial2;
extern DefaultSerial4 MSerial3;

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 3
#define EP_SERIAL_PORT(N) customizedSerial##N
#define USB_SERIAL_PORT(N) customizedSerial##N
#include "../shared/serial_ports.h"

// Define constants and variables for buffering incoming serial data.  We're
// using a ring buffer (I think), in which rx_buffer_head is the index of the
// location to which to write the next incoming character and rx_buffer_tail
// is the index of the location from which to read.
// 256 is the max limit due to uint8_t head and tail. Use only powers of 2. (...,16,32,64,128,256)
#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 32
#endif

//#if ENABLED(SERIAL_XON_XOFF) && RX_BUFFER_SIZE < 1024
//  #error "SERIAL_XON_XOFF requires RX_BUFFER_SIZE >= 1024 for reliable transfers without drops."
//#elif RX_BUFFER_SIZE && (RX_BUFFER_SIZE < 2 || !IS_POWER_OF_2(RX_BUFFER_SIZE))
//  #error "RX_BUFFER_SIZE must be a power of 2 greater than 1."
//#elif TX_BUFFER_SIZE && (TX_BUFFER_SIZE < 2 || TX_BUFFER_SIZE > 256 || !IS_POWER_OF_2(TX_BUFFER_SIZE))
//  #error "TX_BUFFER_SIZE must be 0, a power of 2 greater than 1, and no greater than 256."
//#endif

// Templated structure wrapper
template<typename S, unsigned int addr> struct StructWrapper {
  constexpr StructWrapper(int) {}
  FORCE_INLINE S* operator->() const { return (S*)addr; }
};

template<typename Cfg>
class MarlinSerial {
protected:
  // Information for all supported UARTs
  static constexpr uint32_t BASES[] = {0x400E0800U, 0x40098000U, 0x4009C000U, 0x400A0000U, 0x400A4000U};
  static constexpr IRQn_Type IRQS[] = {  UART_IRQn, USART0_IRQn, USART1_IRQn, USART2_IRQn, USART3_IRQn};
  static constexpr int    IRQ_IDS[] = {    ID_UART,   ID_USART0,   ID_USART1,   ID_USART2,   ID_USART3};

  // Alias for shorter code
  static constexpr StructWrapper<Uart,BASES[Cfg::PORT]> HWUART = 0;
  static constexpr IRQn_Type HWUART_IRQ = IRQS[Cfg::PORT];
  static constexpr int HWUART_IRQ_ID = IRQ_IDS[Cfg::PORT];

  // Base size of type on buffer size
  typedef uvalue_t(Cfg::RX_SIZE - 1) ring_buffer_pos_t;

  struct ring_buffer_r {
    volatile ring_buffer_pos_t head, tail;
    unsigned char buffer[Cfg::RX_SIZE];
  };

  struct ring_buffer_t {
    volatile uint8_t head, tail;
    unsigned char buffer[Cfg::TX_SIZE];
  };

  static ring_buffer_r rx_buffer;
  static ring_buffer_t tx_buffer;
  static bool _written;

  static constexpr uint8_t XON_XOFF_CHAR_SENT = 0x80,  // XON / XOFF Character was sent
                           XON_XOFF_CHAR_MASK = 0x1F;  // XON / XOFF character to send

  // XON / XOFF character definitions
  static constexpr uint8_t XON_CHAR  = 17, XOFF_CHAR = 19;
  static uint8_t xon_xoff_state,
                 rx_dropped_bytes,
                 rx_buffer_overruns,
                 rx_framing_errors;
  static ring_buffer_pos_t rx_max_enqueued;

  FORCE_INLINE static void store_rxd_char();
  FORCE_INLINE static void _tx_thr_empty_irq();
  static void UART_ISR();

public:
  MarlinSerial() {};
  static void begin(const long);
  static void end();
  static int peek();
  static int read();
  static void flush();
  static ring_buffer_pos_t available();
  static size_t write(const uint8_t c);
  static void flushTX();

  static bool emergency_parser_enabled() { return Cfg::EMERGENCYPARSER; }

  FORCE_INLINE static uint8_t dropped() { return Cfg::DROPPED_RX ? rx_dropped_bytes : 0; }
  FORCE_INLINE static uint8_t buffer_overruns() { return Cfg::RX_OVERRUNS ? rx_buffer_overruns : 0; }
  FORCE_INLINE static uint8_t framing_errors() { return Cfg::RX_FRAMING_ERRORS ? rx_framing_errors : 0; }
  FORCE_INLINE static ring_buffer_pos_t rxMaxEnqueued() { return Cfg::MAX_RX_QUEUED ? rx_max_enqueued : 0; }
};

// Serial port configuration
template <uint8_t serial>
struct MarlinSerialCfg {
  static constexpr int PORT               = serial;
  static constexpr unsigned int RX_SIZE   = RX_BUFFER_SIZE;
  static constexpr unsigned int TX_SIZE   = TX_BUFFER_SIZE;
  static constexpr bool XONOFF            = ENABLED(SERIAL_XON_XOFF);
  static constexpr bool EMERGENCYPARSER   = ENABLED(EMERGENCY_PARSER);
  static constexpr bool DROPPED_RX        = ENABLED(SERIAL_STATS_DROPPED_RX);
  static constexpr bool RX_OVERRUNS       = ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS);
  static constexpr bool RX_FRAMING_ERRORS = ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS);
  static constexpr bool MAX_RX_QUEUED     = ENABLED(SERIAL_STATS_MAX_RX_QUEUED);
};

#if defined(SERIAL_PORT) && SERIAL_PORT >= 0
  typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT> > > MSerialT1;
  extern MSerialT1 customizedSerial1;
#endif

#if defined(SERIAL_PORT_2) && SERIAL_PORT_2 >= 0
  typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT_2> > > MSerialT2;
  extern MSerialT2 customizedSerial2;
#endif

#if defined(SERIAL_PORT_3) && SERIAL_PORT_3 >= 0
  typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT_3> > > MSerialT3;
  extern MSerialT3 customizedSerial3;
#endif
