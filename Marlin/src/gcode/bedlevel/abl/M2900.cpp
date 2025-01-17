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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(CREALITY_RTS)

#include "../../gcode.h"
#include "../../../feature/bedlevel/bedlevel.h"
#include "../../../lcd/rts/lcd_rts.h"

#define DIMLFAC1 (0.8f)
#define DIMLFAC2 ((1.0f - DIMLFAC1) / 2.0f) // 0.1

/**
 * M2900: Report Bed Leveling Grid and test result
 */
void GcodeSuite::M2900() {
  bedlevel.print_leveling_grid();
  const float z_valuesTest = DIMLFAC1 * bedlevel.z_values[0][0]
                           + DIMLFAC2 * (  (bedlevel.z_values[0][1] + bedlevel.z_values[1][0]) * 2.0f
                                         - (bedlevel.z_values[0][2] + bedlevel.z_values[2][0]) );
  SERIAL_ECHOLNPGM("z_valuesTest = ", z_valuesTest, " algorithm");
}

#endif // CREALITY_RTS
