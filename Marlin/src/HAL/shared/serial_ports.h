/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * serial_ports.h - Define Marlin serial port macros and optionally declare ports
 *
 * This header defines one or more of the serial ports...
 *   - MYSERIAL1/2/3 ..... for host devices
 *   - MMU_SERIAL ........ for Multi-Material color changers
 *   - LCD_SERIAL ........ for serial LCDs
 *   - RS485_SERIAL ...... for CAN bus devices
 *
 * Before including this header define the following macros:
 *   - SERIAL_INDEX_MIN, SERIAL_INDEX_MAX to provide the valid range of serial port indexes.
 *   - _MSERIAL(X) and MSERIAL(X) to provide the instance name of Serial Port X. (Default: MSerial##X)
 *   - EP_SERIAL_PORT(X) to provide the instance name of Emergency Parser serial port X (if it is special).
 *   - USB_SERIAL_PORT(X) to provide the instance name of the USB serial port (if any).
 *   - ETH_SERIAL_PORT(X) to provide the instance name of the Ethernet serial port (if any).
 *   - DECLARE_SERIAL(X) to declare standard a serial port with the given index.
 */

#ifndef _MSERIAL
  #define _MSERIAL(X) MSerial##X
#endif
#ifndef MSERIAL
  #define MSERIAL(X) _MSERIAL(X)
#endif

#define INDEX_RANGE_MSG " must be from " STRINGIFY(SERIAL_INDEX_MIN) " to " STRINGIFY(SERIAL_INDEX_MAX)
#if defined(USB_SERIAL_PORT) && defined(ETH_SERIAL_PORT)
  #define MORE_INDEXES_MSG ", -1 for Native USB, or -2 for Ethernet"
#elif defined(USB_SERIAL_PORT)
  #define MORE_INDEXES_MSG ", or -1 for Native USB"
#elif defined(ETH_SERIAL_PORT)
  #define MORE_INDEXES_MSG ", or -2 for Ethernet"
#else
  #define MORE_INDEXES_MSG
#endif

//
// SERIAL_PORT => MYSERIAL1
//

#if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
  #define MYSERIAL1 EP_SERIAL_PORT(1)
#elif WITHIN(SERIAL_PORT, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
  #define MYSERIAL1 MSERIAL(SERIAL_PORT)
  #ifdef DECLARE_SERIAL
    DECLARE_SERIAL(SERIAL_PORT);
  #endif
#elif SERIAL_PORT == -1 && defined(USB_SERIAL_PORT)
  #define MYSERIAL1 USB_SERIAL_PORT(1)
#elif SERIAL_PORT == -2 && defined(ETH_SERIAL_PORT)
  #define MYSERIAL1 ETH_SERIAL_PORT(1)
#endif
#ifndef MYSERIAL1
  static_assert(false, "SERIAL_PORT" INDEX_RANGE_MSG MORE_INDEXES_MSG ".");
  #define MYSERIAL1 _MSERIAL(1) // Dummy port
#endif

//
// SERIAL_PORT_2 => MYSERIAL2
//

#ifdef SERIAL_PORT_2
  #if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
    #define MYSERIAL2 EP_SERIAL_PORT(2)
  #elif WITHIN(SERIAL_PORT_2, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define MYSERIAL2 MSERIAL(SERIAL_PORT_2)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(SERIAL_PORT_2);
    #endif
  #elif SERIAL_PORT_2 == -1 && defined(USB_SERIAL_PORT)
    #define MYSERIAL2 USB_SERIAL_PORT(2)
  #elif SERIAL_PORT_2 == -2 && defined(ETH_SERIAL_PORT)
    #define MYSERIAL2 ETH_SERIAL_PORT(2)
  #endif
  #ifndef MYSERIAL2
    static_assert(false, "SERIAL_PORT_2" INDEX_RANGE_MSG MORE_INDEXES_MSG ".");
    #define MYSERIAL2 _MSERIAL(1) // Dummy port
  #endif
#endif

//
// SERIAL_PORT_3 => MYSERIAL3
//

#ifdef SERIAL_PORT_3
  #if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
    #define MYSERIAL3 EP_SERIAL_PORT(3)
  #elif WITHIN(SERIAL_PORT_3, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define MYSERIAL3 MSERIAL(SERIAL_PORT_3)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(SERIAL_PORT_3);
    #endif
  #elif SERIAL_PORT_3 == -1 && defined(USB_SERIAL_PORT)
    #define MYSERIAL3 USB_SERIAL_PORT(3)
  #elif SERIAL_PORT_3 == -2 && defined(ETH_SERIAL_PORT)
    #define MYSERIAL3 ETH_SERIAL_PORT(3)
  #endif
  #ifndef MYSERIAL3
    static_assert(false, "SERIAL_PORT_3" INDEX_RANGE_MSG MORE_INDEXES_MSG ".");
    #define MYSERIAL3 _MSERIAL(1) // Dummy port
  #endif
#endif

//
// MMU_SERIAL_PORT => MMU_SERIAL
//

#ifdef MMU_SERIAL_PORT
  #if WITHIN(MMU_SERIAL_PORT, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define MMU_SERIAL MSERIAL(MMU_SERIAL_PORT)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(MMU_SERIAL_PORT);
    #endif
  #else
    static_assert(false, "MMU_SERIAL_PORT" INDEX_RANGE_MSG ".");
    #define MMU_SERIAL _MSERIAL(1) // Dummy port
  #endif
#endif

//
// LCD_SERIAL_PORT => LCD_SERIAL
//

#ifdef LCD_SERIAL_PORT
  #if WITHIN(LCD_SERIAL_PORT, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define LCD_SERIAL MSERIAL(LCD_SERIAL_PORT)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(LCD_SERIAL_PORT);
    #endif
  #else
    static_assert(false, "LCD_SERIAL_PORT" INDEX_RANGE_MSG ".");
    #define LCD_SERIAL _MSERIAL(1) // Dummy port
  #endif
#endif

//
// RS485_SERIAL_PORT => RS485_SERIAL
//

#ifdef RS485_SERIAL_PORT
  #if WITHIN(RS485_SERIAL_PORT, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define RS485_SERIAL MSERIAL(RS485_SERIAL_PORT)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(RS485_SERIAL_PORT);
    #endif
  #else
    static_assert(false, "RS485_SERIAL_PORT" INDEX_RANGE_MSG ".");
    #define RS485_SERIAL _MSERIAL(1) // Dummy port
  #endif
#endif

#undef DECLARE_SERIAL
#undef SERIAL_INDEX_MIN
#undef SERIAL_INDEX_MAX
#undef INDEX_RANGE_MSG
