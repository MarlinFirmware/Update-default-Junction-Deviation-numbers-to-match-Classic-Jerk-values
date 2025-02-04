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

#if HAS_HOME_OFFSET

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../lcd/marlinui.h"
#include "../../libs/buzzer.h"
#include "../../MarlinCore.h"

/**
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 *
 * *** TODO: Deprecate M206 for SCARA in favor of M665.
 */
void GcodeSuite::M206() {
  if (!parser.seen_any()) return M206_report();
  LOOP_NUM_AXES(a)
    if (parser.seenval(AXIS_CHAR(a))) set_home_offset((AxisEnum)a, parser.value_axis_units((AxisEnum)a));
  #if ENABLED(MORGAN_SCARA)
    if (parser.seenval('T')) set_home_offset(A_AXIS, parser.value_float()); // Theta
    if (parser.seenval('P')) set_home_offset(B_AXIS, parser.value_float()); // Psi
  #endif

  report_current_position();
}

void GcodeSuite::M206_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_HOME_OFFSET));
  SERIAL_ECHOLNPGM_P(
    #if IS_CARTESIAN
      LIST_N(DOUBLE(NUM_AXES),
        PSTR("  M206 X"), LINEAR_UNIT(home_offset.x),
        SP_Y_STR, LINEAR_UNIT(home_offset.y),
        SP_Z_STR, LINEAR_UNIT(home_offset.z),
        SP_I_STR, I_AXIS_UNIT(home_offset.i),
        SP_J_STR, J_AXIS_UNIT(home_offset.j),
        SP_K_STR, K_AXIS_UNIT(home_offset.k),
        SP_U_STR, U_AXIS_UNIT(home_offset.u),
        SP_V_STR, V_AXIS_UNIT(home_offset.v),
        SP_W_STR, W_AXIS_UNIT(home_offset.w)
      )
    #else
      PSTR("  M206 Z"), LINEAR_UNIT(home_offset.z)
    #endif
  );
}

/**
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 *
 * Parameters:
 *   X, Y, Z, ...  - Flags to set the home offset for only the specified axes
 *               P - Flag to add the distance to the existing home offset
 */
void GcodeSuite::M428() {
  if (homing_needed_error()) return;

  const bool no_axes = !parser.seen(STR_AXES_LOGICAL);
  xyz_float_t diff;
  LOOP_NUM_AXES(i) {
    if (no_axes || parser.seen_test(AXIS_CHAR(i))) {
      diff[i] = base_home_pos((AxisEnum)i) - current_position[i];
      // If the current position is too far, for min homing just set to the negative current position
      if (!WITHIN(diff[i], -20, 20) && home_dir((AxisEnum)i) > 0) diff[i] = -current_position[i];
      // If still too far report an error
      if (!WITHIN(diff[i], -20, 20)) {
        SERIAL_ERROR_MSG(STR_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGE(MSG_ERR_M428_TOO_FAR);
        ERR_BUZZ();
        return;
      }
    }
    else
      diff[i] = 0;
  }

  const bool adding = parser.seen_test('P');
  LOOP_NUM_AXES(i) set_home_offset((AxisEnum)i, diff[i] + (adding ? home_offset[i] : 0));

  report_current_position();
  LCD_MESSAGE(MSG_HOME_OFFSETS_APPLIED);
  OKAY_BUZZ();
}

#endif // HAS_HOME_OFFSET
