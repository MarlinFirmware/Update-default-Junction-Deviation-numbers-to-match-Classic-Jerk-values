/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(EMERGENCY_PARSER)
  #include "../../feature/e_parser.h"
#endif

#include "../../core/serial_hook.h"

typedef ForwardSerial1Class< decltype(Serial) > DefaultSerial1;
extern DefaultSerial1 MSerial0;
typedef ForwardSerial1Class<decltype(SerialUSB)> USBSerialType;
extern USBSerialType USBSerial;

#define Serial0 Serial
#define _DECLARE_SERIAL(X) \
  typedef ForwardSerial1Class<decltype(Serial##X)> DefaultSerial##X; \
  extern DefaultSerial##X MSerial##X
#define DECLARE_SERIAL(X) _DECLARE_SERIAL(X)

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 6
#define USB_SERIAL_PORT(...) MSerial0
#include "../shared/serial_ports.h"

#if defined(LCD_SERIAL_PORT) && ANY(HAS_DGUS_LCD, EXTENSIBLE_UI)
  #define SERIAL_GET_TX_BUFFER_FREE() LCD_SERIAL.availableForWrite()
#endif
