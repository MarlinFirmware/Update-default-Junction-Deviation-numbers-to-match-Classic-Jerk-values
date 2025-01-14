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

#if ENABLED(BEZIER_CURVE_SUPPORT)

#if AXIS_COLLISION('I') || AXIS_COLLISION('J')
  #error "G5 parameter 'I' or 'J' collision with axis name."
#endif

#include "../../module/motion.h"
#include "../../module/planner_bezier.h"

/**
 * Parameters interpreted according to:
 * https://linuxcnc.org/docs/2.7/html/gcode/g-code.html#gcode:g5
 * However I, J omission is not supported at this point; all
 * parameters can be omitted and default to zero.
 */

#include "../gcode.h"
#include "../../MarlinCore.h" // for IsRunning()

/**
 * G5: Cubic B-spline
 */
void GcodeSuite::G5() {
  if (!MOTION_CONDITIONS) return;

  #if ENABLED(CNC_WORKSPACE_PLANES)
    if (workspace_plane != PLANE_XY) {
      SERIAL_ERROR_MSG(STR_ERR_BAD_PLANE_MODE);
      return;
    }
  #endif

  get_destination_from_command();

  const xy_pos_t offsets[2] = {
    { parser.linearval('I'), parser.linearval('J') },
    { parser.linearval('P'), parser.linearval('Q') }
  };

  cubic_b_spline(current_position, destination, offsets, MMS_SCALED(feedrate_mm_s), active_extruder);
  current_position = destination;
}

#endif // BEZIER_CURVE_SUPPORT
