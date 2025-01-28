/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(CNC_COORDINATE_ROTATION)

#include "../gcode.h"
#include "../../module/motion.h"

void GcodeSuite::G68_report() {
  SERIAL_ECHOLNPGM("Rotation: R", DEGREES(rotation.rad), " X", rotation.x, " Y", rotation.y);
}

/**
 * G68: Set a center and angle for coordinate rotation
 *   X<linear> - X center for rotation, workspace-relative
 *   Y<linear> - Y center for rotation, workspace-relative
 *   R<angle>  - Coordinate Rotation in Degrees
 */
void GcodeSuite::G68() {
  if (!parser.seen_any()) return G68_report();
  if (parser.seenval('X') || parser.seenval('A')) rotation.x = RAW_X_POSITION(parser.value_linear_units());
  if (parser.seenval('Y') || parser.seenval('B')) rotation.y = RAW_Y_POSITION(parser.value_linear_units());
  if (parser.seenval('R')) rotation.set_angle(RADIANS(parser.value_float()));
}

/**
 * G69: Cancel coordinate rotation
 */
void GcodeSuite::G69() {

  rotation.reset();

}

#endif // CNC_COORDINATE_ROTATION
