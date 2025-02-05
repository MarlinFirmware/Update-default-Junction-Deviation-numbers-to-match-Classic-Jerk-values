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

#if HAS_HOTEND_OFFSET

#include "../gcode.h"
#include "../../module/motion.h"

#if ENABLED(DELTA)
  #include "../../module/planner.h"
#endif

/**
 * M218 - set hotend offset (in linear units)
 *
 *   T<tool>
 *   X<xoffset>
 *   Y<yoffset>
 *   Z<zoffset>
 */
void GcodeSuite::M218() {

  if (!parser.seen_any()) return M218_report();

  const int8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  #if HAS_X_AXIS
    if (parser.seenval('X')) hotend_offset[target_extruder].x = parser.value_linear_units();
  #endif
  #if HAS_Y_AXIS
    if (parser.seenval('Y')) hotend_offset[target_extruder].y = parser.value_linear_units();
  #endif
  #if HAS_Z_AXIS
    if (parser.seenval('Z')) hotend_offset[target_extruder].z = parser.value_linear_units();
  #endif

  #if ENABLED(DELTA)
    if (target_extruder == active_extruder)
      do_blocking_move_to_xy(current_position, planner.settings.max_feedrate_mm_s[X_AXIS]);
  #endif
}

void GcodeSuite::M218_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading(forReplay, F(STR_HOTEND_OFFSETS));
  for (uint8_t e = 1; e < HOTENDS; ++e) {
    report_echo_start(forReplay);
    SERIAL_ECHOLNPGM_P(
      PSTR("  M218 T"), e,
      SP_X_STR, LINEAR_UNIT(hotend_offset[e].x),
      SP_Y_STR, LINEAR_UNIT(hotend_offset[e].y),
      SP_Z_STR, p_float_t(LINEAR_UNIT(hotend_offset[e].z), 3)
    );
  }
}

#endif // HAS_HOTEND_OFFSET
