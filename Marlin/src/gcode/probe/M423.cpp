/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * M423.cpp - X-Axis Twist Compensation
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(X_AXIS_TWIST_COMPENSATION)

#include "../gcode.h"
#include "../../feature/x_twist.h"
#include "../../module/probe.h"

/**
 * M423: Set a Z offset for X-Twist (added to the mesh on future G29).
 *  M423 [R] [A<startx>] [I<interval>] [X<index> Z<offset>]
 *
 *    R         - Reset the twist compensation data
 *    A<linear> - Set the X twist starting X position
 *    E<linear> - Set the X twist ending X position
 *    I<linear> - Set the X twist X-spacing directly
 *    X<index>  - Index of a Z value in the list
 *    Z<linear> - A Z value to set
 */
void GcodeSuite::M423() {

  bool do_report = true;
  float new_spacing = 0;

  if (parser.seen_test('R')) {
    do_report = false;
    xatc.reset();
  }
  if (parser.seenval('A')) {
    do_report = false;
    xatc.start = parser.value_float();
    new_spacing = (probe.max_x() - xatc.start) / (XATC_MAX_POINTS - 1);
  }
  if (parser.seenval('E')) {
    do_report = false;
    new_spacing = (parser.value_float() - xatc.start) / (XATC_MAX_POINTS - 1);
  }
  else if (parser.seenval('I')) {
    do_report = false;
    new_spacing = parser.value_float();
  }

  if (new_spacing) xatc.spacing = new_spacing;

  if (parser.seenval('X')) {
    do_report = false;
    const int8_t x = parser.value_int();
    if (!WITHIN(x, 0, XATC_MAX_POINTS - 1))
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(X) out of range (0..", XATC_MAX_POINTS - 1, ")."));
    else {
      if (parser.seenval('Z'))
        xatc.z_offset[x] = parser.value_linear_units();
      else
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(Z) required."));
    }
  }

  if (do_report) M423_report();

}

void GcodeSuite::M423_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading(forReplay, F("X-Twist Correction"));
  SERIAL_ECHOLNPGM("  M423 A", xatc.start, " I", xatc.spacing);
  for (uint_fast8_t x = 0; x < XATC_MAX_POINTS; ++x) {
    const float z = xatc.z_offset[x];
    SERIAL_ECHOPGM("  M423 X", x, " Z");
    serial_offset(isnan(z) ? 0 : z);
    SERIAL_EOL();
  }
}

#endif // X_AXIS_TWIST_COMPENSATION
