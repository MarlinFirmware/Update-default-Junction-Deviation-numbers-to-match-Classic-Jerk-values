/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfig.h"

#if ENABLED(CNC_5X)

#include "../gcode.h"
#include "../../module/planner.h"

//#define DEBUG_CNC_5X
#define DEBUG_OUT ENABLED(DEBUG_CNC_5X)
#include "../../core/debug_out.h"

/**
 * M168 - 5 Axis Simple Conversion
 *
 *  X Y Z - position of cartesians axis
 *  E A B - position of rotary axis
 */
void GcodeSuite::M168() {
  //float IX_5, IY_5, IZ_5,
  //      OX_5, OY_5, OZ_5,
  //      IA_5, IB_5, IC_5,
  //      OA_5, OB_5, OC_5, E_5;

  const float X_5 = parser.floatval('X'),
              Y_5 = parser.floatval('Y'),
              Z_5 = parser.floatval('Z'),
              A_5 = parser.floatval('A'),
              B_5 = parser.floatval('B'),
              C_5 = parser.floatval('C');

  DEBUG_ECHOLNPGM("X", X_5, " Y", Y_5, " Z", Z_5, " A", A_5, " B", B_5, " C", C_5);

  #if ALL(A_5x, B_5x)
    const float I = A_5, J = B_5;
  #elif ALL(A_5x, C_5x)
    const float I = A_5, J = C_5;
  #elif ALL(B_5x, C_5x)
    const float I = B_5, J = C_5;
  #endif

  destination.set(X_5, Y_5, Z_5 /*, I, J */);
  prepare_line_to_destination();
  planner.synchronize();

  //current_position = current_position5;
  DEBUG_ECHOLNPGM("I", current_position.i, " J", current_position.j);
}

#endif // CNC_5X
