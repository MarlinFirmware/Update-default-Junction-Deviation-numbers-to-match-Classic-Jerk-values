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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(NOZZLE_PARK_FEATURE)

#include "../../gcode.h"
#include "../../../libs/nozzle.h"
#include "../../../module/motion.h"
#if ENABLED(SOVOL_SV06_RTS)
  #include "../../../lcd/sovol_rts/sovol_rts.h"
#endif

/**
 * G27: Park the nozzle according with the given style
 *
 *  P<style> - Parking style:
 *             0 = (Default) Relative raise by NOZZLE_PARK_Z_RAISE_MIN (>= NOZZLE_PARK_POINT.z) before XY parking.
 *             1 = Absolute move to NOZZLE_PARK_POINT.z before XY parking. (USE WITH CAUTION!)
 *             2 = Relative raise by NOZZLE_PARK_POINT.z before XY parking.
 *             3 = Relative raise by NOZZLE_PARK_Z_RAISE_MIN, skip XY parking.
 *             4 = No Z raise. Just XY parking.
 */
void GcodeSuite::G27() {
  // Don't allow nozzle parking without homing first
  if (homing_needed_error()) return;
  nozzle.park(parser.ushortval('P'));
  TERN_(SOVOL_SV06_RTS, RTS_MoveAxisHoming());
}

#endif // NOZZLE_PARK_FEATURE
