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
 * MarlinSerial.h - Hardware serial library for Wiring
 * Copyright (c) 2006 Nicholas Zambetti.  All right reserved.
 *
 * Modified 28 September 2010 by Mark Sproul
 * Modified 14 February 2016 by Andreas Hardtung (added tx buffer)
 * Modified 01 October 2017 by Eduardo José Tagle (added XON/XOFF)
 * Templatized 01 October 2018 by Eduardo José Tagle to allow multiple instances
 */

#include <WString.h>

#include "../../inc/MarlinConfigPre.h"
#include "../../core/types.h"
#include "../../core/serial_hook.h"

#ifndef USBCON

  // The presence of the UBRRH register is used to detect a UART.
  #define UART_PRESENT(port) ((port == 0 && (defined(UBRRH) || defined(UBRR0H))) || \
                              (port == 1 && defined(UBRR1H)) || (port == 2 && defined(UBRR2H)) || \
                              (port == 3 && defined(UBRR3H)))

  // These are macros to build serial port register names for the selected SERIAL_PORT (C preprocessor
  // requires two levels of indirection to expand macro values properly)
  #define SERIAL_REGNAME(registerbase,number,suffix) _SERIAL_REGNAME(registerbase,number,suffix)
  #if SERIAL_PORT == 0 && (!defined(UBRR0H) || !defined(UDR0)) // use un-numbered registers if necessary
    #define _SERIAL_REGNAME(registerbase,number,suffix) registerbase##suffix
  #else
    #define _SERIAL_REGNAME(registerbase,number,suffix) registerbase##number##suffix
  #endif

  // Registers used by MarlinSerial class (expanded depending on selected serial port)

  // Templated 8bit register (generic)
  #define UART_REGISTER_DECL_BASE(registerbase, suffix) \
    template<int portNr> struct R_##registerbase##x##suffix {}

  // Templated 8bit register (specialization for each port)
  #define UART_REGISTER_DECL(port, registerbase, suffix) \
    template<> struct R_##registerbase##x##suffix<port> { \
      constexpr R_##registerbase##x##suffix(int) {} \
      FORCE_INLINE void operator=(uint8_t newVal) const { SERIAL_REGNAME(registerbase,port,suffix) = newVal; } \
      FORCE_INLINE operator uint8_t() const { return SERIAL_REGNAME(registerbase,port,suffix); } \
    }

  // Templated 1bit register (generic)
  #define UART_BIT_DECL_BASE(registerbase, suffix, bit) \
    template<int portNr>struct B_##bit##x {}

  // Templated 1bit register (specialization for each port)
  #define UART_BIT_DECL(port, registerbase, suffix, bit) \
    template<> struct B_##bit##x<port> { \
      constexpr B_##bit##x(int) {} \
      FORCE_INLINE void operator=(int newVal) const { \
        if (newVal) \
          SBI(SERIAL_REGNAME(registerbase,port,suffix),SERIAL_REGNAME(bit,port,)); \
        else \
          CBI(SERIAL_REGNAME(registerbase,port,suffix),SERIAL_REGNAME(bit,port,)); \
      } \
      FORCE_INLINE operator bool() const { return TEST(SERIAL_REGNAME(registerbase,port,suffix),SERIAL_REGNAME(bit,port,)); } \
    }

  #define UART_DECL_BASE() \
    UART_REGISTER_DECL_BASE(UCSR,A);\
    UART_REGISTER_DECL_BASE(UDR,);\
    UART_REGISTER_DECL_BASE(UBRR,H);\
    UART_REGISTER_DECL_BASE(UBRR,L);\
    UART_BIT_DECL_BASE(UCSR,B,RXEN);\
    UART_BIT_DECL_BASE(UCSR,B,TXEN);\
    UART_BIT_DECL_BASE(UCSR,A,TXC);\
    UART_BIT_DECL_BASE(UCSR,B,RXCIE);\
    UART_BIT_DECL_BASE(UCSR,A,UDRE);\
    UART_BIT_DECL_BASE(UCSR,A,FE);\
    UART_BIT_DECL_BASE(UCSR,A,DOR);\
    UART_BIT_DECL_BASE(UCSR,B,UDRIE);\
    UART_BIT_DECL_BASE(UCSR,A,RXC);\
    UART_BIT_DECL_BASE(UCSR,A,U2X)

  #define UART_DECL(port) \
    UART_REGISTER_DECL(port,UCSR,A);\
    UART_REGISTER_DECL(port,UDR,);\
    UART_REGISTER_DECL(port,UBRR,H);\
    UART_REGISTER_DECL(port,UBRR,L);\
    UART_BIT_DECL(port,UCSR,B,RXEN);\
    UART_BIT_DECL(port,UCSR,B,TXEN);\
    UART_BIT_DECL(port,UCSR,A,TXC);\
    UART_BIT_DECL(port,UCSR,B,RXCIE);\
    UART_BIT_DECL(port,UCSR,A,UDRE);\
    UART_BIT_DECL(port,UCSR,A,FE);\
    UART_BIT_DECL(port,UCSR,A,DOR);\
    UART_BIT_DECL(port,UCSR,B,UDRIE);\
    UART_BIT_DECL(port,UCSR,A,RXC);\
    UART_BIT_DECL(port,UCSR,A,U2X)

  // Declare empty templates
  UART_DECL_BASE();

  // And all the specializations for each possible serial port
  #if UART_PRESENT(0)
    UART_DECL(0);
  #endif
  #if UART_PRESENT(1)
    UART_DECL(1);
  #endif
  #if UART_PRESENT(2)
    UART_DECL(2);
  #endif
  #if UART_PRESENT(3)
    UART_DECL(3);
  #endif

  #define BYTE 0

  template<typename Cfg>
  class MarlinSerial {
  protected:
    // Registers
    static constexpr R_UCSRxA<Cfg::PORT> R_UCSRA = 0;
    static constexpr R_UDRx<Cfg::PORT>   R_UDR   = 0;
    static constexpr R_UBRRxH<Cfg::PORT> R_UBRRH = 0;
    static constexpr R_UBRRxL<Cfg::PORT> R_UBRRL = 0;

    // Bits
    static constexpr B_RXENx<Cfg::PORT>  B_RXEN  = 0;
    static constexpr B_TXENx<Cfg::PORT>  B_TXEN  = 0;
    static constexpr B_TXCx<Cfg::PORT>   B_TXC   = 0;
    static constexpr B_RXCIEx<Cfg::PORT> B_RXCIE = 0;
    static constexpr B_UDREx<Cfg::PORT>  B_UDRE  = 0;
    static constexpr B_FEx<Cfg::PORT>    B_FE    = 0;
    static constexpr B_DORx<Cfg::PORT>   B_DOR   = 0;
    static constexpr B_UDRIEx<Cfg::PORT> B_UDRIE = 0;
    static constexpr B_RXCx<Cfg::PORT>   B_RXC   = 0;
    static constexpr B_U2Xx<Cfg::PORT>   B_U2X   = 0;

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

    FORCE_INLINE static ring_buffer_pos_t atomic_read_rx_head();

    static volatile bool rx_tail_value_not_stable;
    static volatile uint16_t rx_tail_value_backup;

    FORCE_INLINE static void atomic_set_rx_tail(ring_buffer_pos_t value);
    FORCE_INLINE static ring_buffer_pos_t atomic_read_rx_tail();

  public:
    FORCE_INLINE static void store_rxd_char();
    FORCE_INLINE static void _tx_udr_empty_irq();

  public:
    static void begin(const long);
    static void end();
    static int peek();
    static int read();
    static void flush();
    static ring_buffer_pos_t available();
    static void write(const uint8_t c);
    static void flushTX();
    #if ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
      static ring_buffer_pos_t get_tx_buffer_free();
    #endif

    enum { HasEmergencyParser = Cfg::EMERGENCYPARSER };
    static bool emergency_parser_enabled() { return Cfg::EMERGENCYPARSER; }

    FORCE_INLINE static uint8_t dropped() { return Cfg::DROPPED_RX ? rx_dropped_bytes : 0; }
    FORCE_INLINE static uint8_t buffer_overruns() { return Cfg::RX_OVERRUNS ? rx_buffer_overruns : 0; }
    FORCE_INLINE static uint8_t framing_errors() { return Cfg::RX_FRAMING_ERRORS ? rx_framing_errors : 0; }
    FORCE_INLINE static ring_buffer_pos_t rxMaxEnqueued() { return Cfg::MAX_RX_QUEUED ? rx_max_enqueued : 0; }
  };

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

  typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT> > > MSerialT1;
  extern MSerialT1 customizedSerial1;

  #ifdef SERIAL_PORT_2
    typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT_2> > > MSerialT2;
    extern MSerialT2 customizedSerial2;
  #endif

  #ifdef SERIAL_PORT_3
    typedef Serial1Class< MarlinSerial< MarlinSerialCfg<SERIAL_PORT_3> > > MSerialT3;
    extern MSerialT3 customizedSerial3;
  #endif

#endif // !USBCON

#ifdef MMU_SERIAL_PORT
  template <uint8_t serial>
  struct MMU2SerialCfg {
    static constexpr int PORT               = serial;
    static constexpr unsigned int RX_SIZE   = 32;
    static constexpr unsigned int TX_SIZE   = 32;
    static constexpr bool XONOFF            = false;
    static constexpr bool EMERGENCYPARSER   = false;
    static constexpr bool DROPPED_RX        = false;
    static constexpr bool RX_FRAMING_ERRORS = false;
    static constexpr bool MAX_RX_QUEUED     = false;
    static constexpr bool RX_OVERRUNS       = false;
  };

  typedef Serial1Class< MarlinSerial< MMU2SerialCfg<MMU_SERIAL_PORT> > > MSerialMMU2;
  extern MSerialMMU2 mmuSerial;
#endif

#ifdef LCD_SERIAL_PORT

  template <uint8_t serial>
  struct LCDSerialCfg {
    static constexpr int PORT               = serial;
    static constexpr unsigned int RX_SIZE   = TERN(HAS_DGUS_LCD, DGUS_RX_BUFFER_SIZE,  64);
    static constexpr unsigned int TX_SIZE   = TERN(HAS_DGUS_LCD, DGUS_TX_BUFFER_SIZE, 128);
    static constexpr bool XONOFF            = false;
    static constexpr bool EMERGENCYPARSER   = ENABLED(EMERGENCY_PARSER);
    static constexpr bool DROPPED_RX        = false;
    static constexpr bool RX_FRAMING_ERRORS = false;
    static constexpr bool MAX_RX_QUEUED     = false;
    static constexpr bool RX_OVERRUNS       = ALL(HAS_DGUS_LCD, SERIAL_STATS_RX_BUFFER_OVERRUNS);
  };

  typedef Serial1Class< MarlinSerial< LCDSerialCfg<LCD_SERIAL_PORT> > > MSerialLCD;
  extern MSerialLCD lcdSerial;
#endif

// Use the UART for Bluetooth in AT90USB configurations
#if defined(USBCON) && ENABLED(BLUETOOTH)
  typedef Serial1Class<HardwareSerial> MSerialBT;
  extern MSerialBT bluetoothSerial;
#endif
