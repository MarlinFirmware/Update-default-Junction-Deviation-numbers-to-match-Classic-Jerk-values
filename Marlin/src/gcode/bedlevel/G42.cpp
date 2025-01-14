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

#include "../../inc/MarlinConfig.h"

#if HAS_MESH

#include "../gcode.h"
#include "../../MarlinCore.h" // for IsRunning()
#include "../../module/motion.h"
#include "../../feature/bedlevel/bedlevel.h"

#if HAS_PROBE_XY_OFFSET
  #include "../../module/probe.h" // for probe.offset
#endif

/**
 * G42: Move X & Y axes to mesh coordinates (I & J)
 *
 * Parameters:
 *   F<feedrate> : Feedrate in mm/min
 *   I<index>    : X axis point index
 *   J<index>    : Y axis point index
 *   P           : Flag to put the probe at the given point
 */
void GcodeSuite::G42() {
  if (!MOTION_CONDITIONS) return;

  const bool hasI = parser.seenval('I');
  const int8_t ix = hasI ? parser.value_int() : 0;
  const bool hasJ = parser.seenval('J');
  const int8_t iy = hasJ ? parser.value_int() : 0;

  if ((hasI && !WITHIN(ix, 0, GRID_MAX_POINTS_X - 1)) || (hasJ && !WITHIN(iy, 0, GRID_MAX_POINTS_Y - 1))) {
    SERIAL_ECHOLNPGM(STR_ERR_MESH_XY);
    return;
  }

  // Move to current_position, as modified by I, J, P parameters
  destination = current_position;

  if (hasI) destination.x = bedlevel.get_mesh_x(ix);
  if (hasJ) destination.y = bedlevel.get_mesh_y(iy);

  #if HAS_PROBE_XY_OFFSET
    if (parser.seen_test('P')) {
      if (hasI) destination.x -= probe.offset_xy.x;
      if (hasJ) destination.y -= probe.offset_xy.y;
    }
  #endif

  const feedRate_t fval = parser.linearval('F'),
                   fr_mm_s = MMM_TO_MMS(fval > 0 ? fval : 0.0f);

  // SCARA kinematic has "safe" XY raw moves
  #if IS_SCARA
    prepare_internal_fast_move_to_destination(fr_mm_s);
  #else
    prepare_internal_move_to_destination(fr_mm_s);
  #endif
}

#endif // HAS_MESH
