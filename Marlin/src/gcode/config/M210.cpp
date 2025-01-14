/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(EDITABLE_HOMING_FEEDRATE)

#include "../gcode.h"
#include "../../module/motion.h"

/**
 * M210 - Set homing feedrate for one or more axes
 *        in current units (in/mm) per minute
 *
 *  X[feedrate]  Set X axis homing feedrate
 *  Y[feedrate]  Set Y axis homing feedrate
 *  Z[feedrate]  Set Z axis homing feedrate
 *  A[feedrate]  Set I axis homing feedrate (configured axis name applies)
 *  B[feedrate]  Set J axis homing feedrate (configured axis name applies)
 *  C[feedrate]  Set K axis homing feedrate (configured axis name applies)
 *  U[feedrate]  Set U axis homing feedrate (configured axis name applies)
 *  V[feedrate]  Set V axis homing feedrate (configured axis name applies)
 *  W[feedrate]  Set W axis homing feedrate (configured axis name applies)
 *
 * With no arguments, report the current offsets.
 */
void GcodeSuite::M210() {
  if (!parser.seen_any())
    return M210_report();

  #if HAS_X_AXIS
    if (parser.floatval('X') > 0)        homing_feedrate_mm_m.x = parser.value_axis_units(X_AXIS);
  #endif
  #if HAS_Y_AXIS
    if (parser.floatval('Y') > 0)        homing_feedrate_mm_m.y = parser.value_axis_units(Y_AXIS);
  #endif
  #if HAS_Z_AXIS
    if (parser.floatval('Z') > 0)        homing_feedrate_mm_m.z = parser.value_axis_units(Z_AXIS);
  #endif
  #if HAS_I_AXIS
    if (parser.floatval(AXIS4_NAME) > 0) homing_feedrate_mm_m.i = parser.value_axis_units(I_AXIS);
  #endif
  #if HAS_J_AXIS
    if (parser.floatval(AXIS5_NAME) > 0) homing_feedrate_mm_m.j = parser.value_axis_units(J_AXIS);
  #endif
  #if HAS_K_AXIS
    if (parser.floatval(AXIS6_NAME) > 0) homing_feedrate_mm_m.k = parser.value_axis_units(K_AXIS);
  #endif
  #if HAS_U_AXIS
    if (parser.floatval(AXIS7_NAME) > 0) homing_feedrate_mm_m.u = parser.value_axis_units(U_AXIS);
  #endif
  #if HAS_V_AXIS
    if (parser.floatval(AXIS8_NAME) > 0) homing_feedrate_mm_m.v = parser.value_axis_units(V_AXIS);
  #endif
  #if HAS_W_AXIS
    if (parser.floatval(AXIS9_NAME) > 0) homing_feedrate_mm_m.w = parser.value_axis_units(W_AXIS);
  #endif
}

void GcodeSuite::M210_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_HOMING_FEEDRATE));

  SERIAL_ECHOPGM("  M210");
  SERIAL_ECHOLNPGM_P(
    LIST_N(DOUBLE(NUM_AXES)
      , SP_X_STR, X_AXIS_UNIT(homing_feedrate_mm_m.x)
      , SP_Y_STR, Y_AXIS_UNIT(homing_feedrate_mm_m.y)
      , SP_Z_STR, Z_AXIS_UNIT(homing_feedrate_mm_m.z)
      , SP_I_STR, I_AXIS_UNIT(homing_feedrate_mm_m.i)
      , SP_J_STR, J_AXIS_UNIT(homing_feedrate_mm_m.j)
      , SP_K_STR, K_AXIS_UNIT(homing_feedrate_mm_m.k)
      , SP_U_STR, U_AXIS_UNIT(homing_feedrate_mm_m.u)
      , SP_V_STR, V_AXIS_UNIT(homing_feedrate_mm_m.v)
      , SP_W_STR, W_AXIS_UNIT(homing_feedrate_mm_m.w)
    )
  );
}

#endif // EDITABLE_HOMING_FEEDRATE
