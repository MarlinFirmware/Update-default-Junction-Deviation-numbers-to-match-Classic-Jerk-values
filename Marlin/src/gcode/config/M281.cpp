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

#if ENABLED(EDITABLE_SERVO_ANGLES)

#include "../gcode.h"
#include "../../module/servo.h"

/**
 * M281 - Edit / Report Servo Angles
 *
 *  P<index> - Servo to update
 *  L<angle> - Deploy Angle
 *  U<angle> - Stowed Angle
 */
void GcodeSuite::M281() {
  if (!parser.seen_any()) return M281_report();

  if (!parser.seenval('P')) return;

  const int servo_index = parser.value_int();
  if (WITHIN(servo_index, 0, NUM_SERVOS - 1)) {
    #if ENABLED(BLTOUCH)
      if (servo_index == Z_PROBE_SERVO_NR) {
        SERIAL_ERROR_MSG("BLTouch angles can't be changed.");
        return;
      }
    #endif
    if (parser.seenval('L')) servo_angles[servo_index][0] = parser.value_int();
    if (parser.seenval('U')) servo_angles[servo_index][1] = parser.value_int();
  }
  else
    SERIAL_ERROR_MSG("Servo ", servo_index, " out of range");
}

void GcodeSuite::M281_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_SERVO_ANGLES));
  for (uint_fast8_t i = 0; i < NUM_SERVOS; ++i) {
    switch (i) {
      default: break;
      #if ENABLED(SWITCHING_EXTRUDER)
        case SWITCHING_EXTRUDER_SERVO_NR:
        #if EXTRUDERS > 3
          case SWITCHING_EXTRUDER_E23_SERVO_NR:
        #endif
      #elif ENABLED(SWITCHING_NOZZLE)
        case SWITCHING_NOZZLE_SERVO_NR:
        #if ENABLED(SWITCHING_NOZZLE_TWO_SERVOS)
          case SWITCHING_NOZZLE_E1_SERVO_NR:
        #endif
      #elif ENABLED(BLTOUCH) || (HAS_Z_SERVO_PROBE && defined(Z_SERVO_ANGLES))
        case Z_PROBE_SERVO_NR:
      #endif
          report_echo_start(forReplay);
          SERIAL_ECHOLNPGM("  M281 P", i, " L", servo_angles[i][0], " U", servo_angles[i][1]);
    }
  }
}

#endif // EDITABLE_SERVO_ANGLES
