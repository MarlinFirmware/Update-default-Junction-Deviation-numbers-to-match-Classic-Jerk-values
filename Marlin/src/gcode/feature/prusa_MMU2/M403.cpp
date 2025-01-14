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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3

#include "../../gcode.h"

#if HAS_PRUSA_MMU3
  #include "../../../feature/mmu3/mmu3.h"
#elif HAS_PRUSA_MMU2
  #include "../../../feature/mmu/mmu2.h"
#endif

/**
 * M403: Set filament type for MMU2
 *
 * Valid filament type values:
 *
 *  0   Default
 *  1   Flexible
 *  2   PVA
 */
void GcodeSuite::M403() {
  const int8_t index = parser.intval('E', -1),
                type = parser.intval('F', -1);

  if (WITHIN(index, 0, EXTRUDERS - 1) && WITHIN(type, 0, 2))
    #if HAS_PRUSA_MMU3
      mmu3.set_filament_type(index, type);
    #else
      mmu2.set_filament_type(index, type);
    #endif
  else
    SERIAL_ECHO_MSG("M403 - bad arguments.");
}

#endif // HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3
