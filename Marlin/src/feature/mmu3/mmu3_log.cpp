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

/**
 * mmu2_log.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "mmu3_log.h"

namespace MMU3 {

  void LogEchoEvent_P(PGM_P const pstr) {
    SERIAL_ECHO_START(); // @@TODO Decide MMU errors on serial line
    SERIAL_MMU2();
    SERIAL_ECHOLN_P(pstr);
  }

  void LogErrorEvent_P(PGM_P const pstr) {
    LogEchoEvent_P(pstr);
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
