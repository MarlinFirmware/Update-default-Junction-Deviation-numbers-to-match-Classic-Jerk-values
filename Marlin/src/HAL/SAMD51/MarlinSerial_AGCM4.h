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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */

#include "../../core/serial_hook.h"

typedef Serial1Class<Uart> UartT;

extern UartT Serial2;
extern UartT Serial3;
extern UartT Serial4;

typedef ForwardSerial1Class< decltype(Serial) > DefaultSerial1;
typedef ForwardSerial1Class< decltype(Serial1) > DefaultSerial2;
typedef ForwardSerial1Class< decltype(Serial2) > DefaultSerial3;
typedef ForwardSerial1Class< decltype(Serial3) > DefaultSerial4;
typedef ForwardSerial1Class< decltype(Serial4) > DefaultSerial5;

extern DefaultSerial1 MSerial0;
extern DefaultSerial2 MSerial1;
extern DefaultSerial3 MSerial2;
extern DefaultSerial4 MSerial3;
extern DefaultSerial5 MSerial4;

#define __MSERIAL(X) MSerial##X
#define _MSERIAL(X) __MSERIAL(X)
#define MSERIAL(X) _MSERIAL(INCREMENT(X))

#define SERIAL_INDEX_MIN 0
#define SERIAL_INDEX_MAX 3
#define USB_SERIAL_PORT(...) MSerial0
#include "../shared/serial_ports.h"
