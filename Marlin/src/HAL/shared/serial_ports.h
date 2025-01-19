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
 * serial_ports.h - Define Marlin serial port macros
 */

#ifndef _MSERIAL
  #define _MSERIAL(X) MSerial##X
#endif
#ifndef MSERIAL
  #define MSERIAL(X) _MSERIAL(X)
#endif

#define INDEX_RANGE_MSG " must be from " STRINGIFY(SERIAL_INDEX_MIN) " to " STRINGIFY(SERIAL_INDEX_MAX)

#if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
  #define MYSERIAL1 EP_SERIAL_PORT(1)
#elif WITHIN(SERIAL_PORT, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
  #define MYSERIAL1 MSERIAL(SERIAL_PORT)
  #ifdef DECLARE_SERIAL
    DECLARE_SERIAL(SERIAL_PORT);
  #endif
#elif !defined(USB_SERIAL_PORT) && !defined(ETH_SERIAL_PORT)
  static_assert(false, "SERIAL_PORT" INDEX_RANGE_MSG ".");
#elif SERIAL_PORT == -1
  #define MYSERIAL1 USB_SERIAL_PORT(1)
#elif SERIAL_PORT == -2
  #define MYSERIAL1 ETH_SERIAL_PORT(1)
#else
  static_assert(false, "SERIAL_PORT" INDEX_RANGE_MSG ", or -1 for Native USB.");
#endif
#ifndef MYSERIAL1
  #define MYSERIAL1 _MSERIAL(1) // Dummy port
#endif

#ifdef SERIAL_PORT_2
  #if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
    #define MYSERIAL2 EP_SERIAL_PORT(2)
  #elif WITHIN(SERIAL_PORT_2, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define MYSERIAL2 MSERIAL(SERIAL_PORT_2)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(SERIAL_PORT_2);
    #endif
  #elif !defined(USB_SERIAL_PORT) && !defined(ETH_SERIAL_PORT)
    static_assert(false, "SERIAL_PORT_2" INDEX_RANGE_MSG ".");
  #elif SERIAL_PORT_2 == -1
    #define MYSERIAL2 USB_SERIAL_PORT(2)
  #elif SERIAL_PORT_2 == -2
    #define MYSERIAL2 ETH_SERIAL_PORT(2)
  #else
    static_assert(false, "SERIAL_PORT_2" INDEX_RANGE_MSG ", or -1 for Native USB.");
  #endif
  #ifndef MYSERIAL2
    #define MYSERIAL2 _MSERIAL(1) // Dummy port
  #endif
#endif

#ifdef SERIAL_PORT_3
  #if defined(EP_SERIAL_PORT) && ENABLED(EMERGENCY_PARSER)
    #define MYSERIAL3 EP_SERIAL_PORT(3)
  #elif WITHIN(SERIAL_PORT_3, SERIAL_INDEX_MIN, SERIAL_INDEX_MAX)
    #define MYSERIAL3 MSERIAL(SERIAL_PORT_3)
    #ifdef DECLARE_SERIAL
      DECLARE_SERIAL(SERIAL_PORT_3);
    #endif
  #elif !defined(USB_SERIAL_PORT) && !defined(ETH_SERIAL_PORT)
    static_assert(false, "SERIAL_PORT_3" INDEX_RANGE_MSG ".");
  #elif SERIAL_PORT_3 == -1
    #define MYSERIAL3 USB_SERIAL_PORT(3)
  #elif SERIAL_PORT_3 == -2
    #define MYSERIAL3 ETH_SERIAL_PORT(3)
  #else
    static_assert(false, "SERIAL_PORT_3" INDEX_RANGE_MSG ", or -1 for Native USB.");
  #endif
  #ifndef MYSERIAL3
    #define MYSERIAL3 _MSERIAL(1) // Dummy port
  #endif
#endif

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
