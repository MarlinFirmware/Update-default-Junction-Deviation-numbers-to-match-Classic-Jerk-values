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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

#include "../gcode.h"
#include "../../module/planner.h"

/**
 * M92: Set axis steps-per-unit for one or more axes, X, Y, Z, [I, [J, [K, [U, [V, [W,]]]]]] and E.
 *      (Follows the same syntax as G92)
 *
 *      With multiple extruders use T to specify which one.
 *
 *      If no argument is given print the current values.
 *
 * With MAGIC_NUMBERS_GCODE:
 *
 *   Use 'H' and/or 'L' to get ideal layer-height information.
 *   H<microsteps> - Specify micro-steps to use. Best guess if not supplied.
 *   L<linear>     - Desired layer height in current units. Nearest good heights are shown.
 */

void GcodeSuite::M92() {

  const int_fast8_t target_extruder = get_target_extruder_from_command();
  if (target_extruder < 0) return;

  // No arguments? Show M92 report.
  if (!parser.seen(STR_AXES_LOGICAL TERN_(MAGIC_NUMBERS_GCODE, "HL")))
    return M92_report(true, target_extruder);

  LOOP_LOGICAL_AXES(i) {
    if (parser.seenval(AXIS_CHAR(i))) {
      if (TERN1(HAS_EXTRUDERS, i != E_AXIS))
        planner.settings.axis_steps_per_mm[i] = parser.value_per_axis_units((AxisEnum)i);
      else {
        #if HAS_EXTRUDERS
          const float value = parser.value_per_axis_units((AxisEnum)(E_AXIS_N(target_extruder)));
          if (value < 20) {
            float factor = planner.settings.axis_steps_per_mm[E_AXIS_N(target_extruder)] / value; // increase e constants if M92 E14 is given for netfab.
            #if ALL(CLASSIC_JERK, HAS_CLASSIC_E_JERK)
              planner.max_jerk.e *= factor;
            #endif
            planner.settings.max_feedrate_mm_s[E_AXIS_N(target_extruder)] *= factor;
            planner.max_acceleration_steps_per_s2[E_AXIS_N(target_extruder)] *= factor;
          }
          planner.settings.axis_steps_per_mm[E_AXIS_N(target_extruder)] = value;
        #endif
      }
    }
  }
  planner.refresh_positioning();

  #if ENABLED(MAGIC_NUMBERS_GCODE)
    #ifndef Z_MICROSTEPS
      #define Z_MICROSTEPS 16
    #endif
    const float wanted = parser.linearval('L');
    if (parser.seen('H') || wanted) {
      const uint16_t argH = parser.ushortval('H'),
                     micro_steps = argH ?: Z_MICROSTEPS;
      const float z_full_step_mm = micro_steps * planner.mm_per_step[Z_AXIS];
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM("{ micro_steps:", micro_steps, ", z_full_step_mm:", z_full_step_mm);
      if (wanted) {
        const float best = uint16_t(wanted / z_full_step_mm) * z_full_step_mm;
        SERIAL_ECHOPGM(", best:[", best);
        if (best != wanted) { SERIAL_ECHO(C(','), best + z_full_step_mm); }
        SERIAL_CHAR(']');
      }
      SERIAL_ECHOLNPGM(" }");
    }
  #endif
}

void GcodeSuite::M92_report(const bool forReplay/*=true*/, const int8_t e/*=-1*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_STEPS_PER_UNIT));
  #if NUM_AXES
    #define PRINT_EOL
    SERIAL_ECHOPGM_P(LIST_N(DOUBLE(NUM_AXES),
      PSTR("  M92 X"), LINEAR_UNIT(planner.settings.axis_steps_per_mm[X_AXIS]),
      SP_Y_STR, LINEAR_UNIT(planner.settings.axis_steps_per_mm[Y_AXIS]),
      SP_Z_STR, LINEAR_UNIT(planner.settings.axis_steps_per_mm[Z_AXIS]),
      SP_I_STR, I_AXIS_UNIT(planner.settings.axis_steps_per_mm[I_AXIS]),
      SP_J_STR, J_AXIS_UNIT(planner.settings.axis_steps_per_mm[J_AXIS]),
      SP_K_STR, K_AXIS_UNIT(planner.settings.axis_steps_per_mm[K_AXIS]),
      SP_U_STR, U_AXIS_UNIT(planner.settings.axis_steps_per_mm[U_AXIS]),
      SP_V_STR, V_AXIS_UNIT(planner.settings.axis_steps_per_mm[V_AXIS]),
      SP_W_STR, W_AXIS_UNIT(planner.settings.axis_steps_per_mm[W_AXIS])
    ));
  #endif

  #if HAS_EXTRUDERS && DISABLED(DISTINCT_E_FACTORS)
    #define PRINT_EOL
    SERIAL_ECHOPGM_P(SP_E_STR, VOLUMETRIC_UNIT(planner.settings.axis_steps_per_mm[E_AXIS]));
  #endif

  if (ENABLED(PRINT_EOL)) SERIAL_EOL();

  #if ENABLED(DISTINCT_E_FACTORS)
    for (uint_fast8_t i = 0; i < E_STEPPERS; ++i) {
      if (e >= 0 && i != e) continue;
      report_echo_start(forReplay);
      SERIAL_ECHOLNPGM_P(
        PSTR("  M92 T"), i,
        SP_E_STR, VOLUMETRIC_UNIT(planner.settings.axis_steps_per_mm[E_AXIS_N(i)])
      );
    }
  #else
    UNUSED(e);
  #endif
}

#endif // EDITABLE_STEPS_PER_UNIT
