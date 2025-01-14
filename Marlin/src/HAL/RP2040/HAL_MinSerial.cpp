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
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(POSTMORTEM_DEBUGGING)

#include "../shared/HAL_MinSerial.h"


static void TXBegin() {
  #if !WITHIN(SERIAL_PORT, -1, 2)
    #warning "Using POSTMORTEM_DEBUGGING requires a physical U(S)ART hardware in case of severe error."
    #warning "Disabling the severe error reporting feature currently because the used serial port is not a HW port."
  #else
    #if SERIAL_PORT == -1
      USBSerial.begin(BAUDRATE);
    #elif SERIAL_PORT == 0
      USBSerial.begin(BAUDRATE);
    #elif SERIAL_PORT == 1
      Serial1.begin(BAUDRATE);
    #endif
  #endif
}

static void TX(char b){
    #if SERIAL_PORT == -1
      USBSerial
    #elif SERIAL_PORT == 0
      USBSerial
    #elif SERIAL_PORT == 1
      Serial1
    #endif
    .write(b);
}

// A SW memory barrier, to ensure GCC does not overoptimize loops
#define sw_barrier() __asm__ volatile("": : :"memory");


void install_min_serial() {
  HAL_min_serial_init = &TXBegin;
  HAL_min_serial_out = &TX;
}

#endif // POSTMORTEM_DEBUGGING
#endif // __PLAT_RP2040__
