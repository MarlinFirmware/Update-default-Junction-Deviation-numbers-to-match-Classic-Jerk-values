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

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"

#if ENABLED(M114_DETAIL)

  void report_all_axis_pos(const xyze_pos_t &pos, const uint8_t n=LOGICAL_AXES, const uint8_t precision=3) {
    for (uint_fast8_t a = 0; a < n; ++a) {
      SERIAL_ECHOPGM_P((PGM_P)pgm_read_ptr(&SP_AXIS_LBL[a]));
      if (pos[a] >= 0) SERIAL_CHAR(' ');
      SERIAL_ECHO(p_float_t(pos[a], precision));
    }
    SERIAL_EOL();
  }
  inline void report_linear_axis_pos(const xyze_pos_t &pos) { report_all_axis_pos(pos, XYZ); }

  void report_linear_axis_pos(const xyz_pos_t &pos, const uint8_t precision=3) {
    LOOP_NUM_AXES(a) SERIAL_ECHO(FPSTR(pgm_read_ptr(&SP_AXIS_LBL[a])), p_float_t(pos[a], precision));
    SERIAL_EOL();
  }

  void report_current_position_detail() {
    // Position as sent by G-code
    SERIAL_ECHOPGM("\nLogical:");
    report_linear_axis_pos(current_position.asLogical());

    // Cartesian position in native machine space
    SERIAL_ECHOPGM("Raw:    ");
    report_linear_axis_pos(current_position);

    xyze_pos_t leveled = current_position;

    #if HAS_LEVELING
      // Current position with leveling applied
      SERIAL_ECHOPGM("Leveled:");
      planner.apply_leveling(leveled);
      report_linear_axis_pos(leveled);

      // Test planner un-leveling. This should match the Raw result.
      SERIAL_ECHOPGM("UnLevel:");
      xyze_pos_t unleveled = leveled;
      planner.unapply_leveling(unleveled);
      report_linear_axis_pos(unleveled);
    #endif

    #if IS_KINEMATIC
      // Kinematics applied to the leveled position
      SERIAL_ECHOPGM(TERN(POLAR, "Polar", TERN(IS_SCARA, "Scara", "Delta")) "K: " );
      inverse_kinematics(leveled);  // writes delta[]
      report_linear_axis_pos(delta);
    #endif

    planner.synchronize();

    SERIAL_ECHOPGM("Stepper:");
    LOOP_LOGICAL_AXES(i) {
      SERIAL_ECHOPGM_P((PGM_P)pgm_read_ptr(&SP_AXIS_LBL[i]), stepper.position((AxisEnum)i));
    }
    SERIAL_EOL();

    #if IS_SCARA
      const xy_float_t deg = {
        planner.get_axis_position_degrees(A_AXIS),
        planner.get_axis_position_degrees(B_AXIS)
      };
      SERIAL_ECHOPGM("Degrees:");
      report_all_axis_pos(deg, 2);
    #endif

    SERIAL_ECHOPGM("FromStp:");
    get_cartesian_from_steppers();  // writes 'cartes' (with forward kinematics)
    xyze_pos_t from_steppers = LOGICAL_AXIS_ARRAY(
      planner.get_axis_position_mm(E_AXIS),
      cartes.x, cartes.y, cartes.z,
      planner.get_axis_position_mm(I_AXIS),
      planner.get_axis_position_mm(J_AXIS),
      planner.get_axis_position_mm(K_AXIS),
      planner.get_axis_position_mm(U_AXIS),
      planner.get_axis_position_mm(V_AXIS),
      planner.get_axis_position_mm(W_AXIS)
    );
    report_all_axis_pos(from_steppers);

    const xyze_float_t diff = from_steppers - leveled;
    SERIAL_ECHOPGM("Diff:   ");
    report_all_axis_pos(diff);

    TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
  }

#endif // M114_DETAIL

/**
 * M114: Report the current position to host.
 *       Since steppers are moving, the count positions are
 *       projected by using planner calculations.
 *   D - Report more detail. This syncs the planner. (Requires M114_DETAIL)
 *   E - Report E stepper position (Requires M114_DETAIL)
 *   R - Report the realtime position instead of projected.
 */
void GcodeSuite::M114() {

  #if ENABLED(M114_DETAIL)
    if (parser.seen_test('D')) {
      IF_DISABLED(M114_LEGACY, planner.synchronize());
      report_current_position();
      report_current_position_detail();
      return;
    }
    #if HAS_EXTRUDERS
      if (parser.seen_test('E')) {
        SERIAL_ECHOLNPGM("Count E:", stepper.position(E_AXIS));
        return;
      }
    #endif
  #endif

  TERN_(M114_REALTIME, if (parser.seen_test('R')) return report_real_position());

  TERN_(M114_LEGACY, planner.synchronize());
  report_current_position_projected();

  TERN_(FULL_REPORT_TO_HOST_FEATURE, report_current_grblstate_moving());
}
