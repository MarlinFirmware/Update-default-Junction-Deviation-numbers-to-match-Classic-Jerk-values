/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(CNC_DRILLING_CYCLE)

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../MarlinCore.h"
#include "../../module/planner.h"

//#define DEBUG_CNC_DRILLING_CYCLE
#define DEBUG_OUT ENABLED(DEBUG_CNC_DRILLING_CYCLE)
#include "../../core/debug_out.h"

#if ENABLED(VARIABLE_G0_FEEDRATE)
  extern feedRate_t fast_move_feedrate;
#endif

#if 0

void GcodeSuite::G81() {
  if (!IsRunning()
    #if ENABLED(NO_MOTION_BEFORE_HOMING)
      || homing_needed_error(
          (parser.seen_test('X') ? _BV(X_AXIS) : 0)
        | (parser.seen_test('Y') ? _BV(Y_AXIS) : 0)
        | (parser.seen_test('Z') ? _BV(Z_AXIS) : 0) )
    #endif
  ) return;

  get_destination_from_command();

  const float slow_feedrate = feedrate_mm_s,
              fast_feedrate = (
                #if ENABLED(VARIABLE_G0_FEEDRATE)
                  saved_g0_feedrate_mm_s
                #elif defined(G0_FEEDRATE)
                  MMM_TO_MMS(G0_FEEDRATE)
                #else
                  feedrate_mm_s
                #endif
              );

  auto line_to_z = [](const float &z, const feedRate_t fr_mm_s) {
    REMEMBER(fr, feedrate_mm_s, fr_mm_s);
    destination.z = z;
    prepare_line_to_destination();
  };

  // Store current Z and drill depth Z
  const float initial_z = current_position.z,
              depth_z = destination.z;
  float retract_z = initial_z;

  // Rapid move XY plane only at current Z height
  line_to_z(initial_z, fast_feedrate);

  // Rapid move to Z retract height, if provided
  if (parser.seenval('R')) {
    retract_z = LOGICAL_TO_NATIVE(parser.value_float(), Z_AXIS);
    line_to_z(retract_z, fast_feedrate);
  }

  // Move Z only to target depth at defined feedrate
  line_to_z(depth_z, slow_feedrate);

  // Retract to specified height or starting height
  line_to_z(retract_z, fast_feedrate);
}

#endif

void do_synchronized_move() {
  prepare_line_to_destination();
  planner.synchronize();
}

void line_to_z(const float &z, const feedRate_t fr_mm_s) {
  REMEMBER(fr, feedrate_mm_s, fr_mm_s);
  destination.z = z;
  do_synchronized_move();
}

inline feedRate_t get_fast_feedrate() {
  return (
    #if ENABLED(VARIABLE_G0_FEEDRATE)
      fast_move_feedrate
    #elif defined(G0_FEEDRATE)
      MMM_TO_MMS(G0_FEEDRATE)
    #else
      feedrate_mm_s
    #endif
  );
}

/**
 * G81 Drill Canned Cycle
 *
 *  X Y - Hole position
 *    Z - Hole Depth - Relative to the Retraction plan R
 *    R - Position on Retraction plan
 *    Q - Max Depth for each Cut/Peck
 *    P - Dwell time on bottom of each Cut/Peck
 *    F - Cutting feedrate
 */
void GcodeSuite::G81() {
  if (!IsRunning()) return;

  constexpr float qval = 0;
  constexpr millis_t pval = 0;

  const float slow_feedrate = feedrate_mm_s,
              fast_feedrate = get_fast_feedrate();

  float zval = 0;
  if (parser.seenval('Z')) {
    zval = parser.value_float();
    DEBUG_ECHOLNPGM("Z", zval);
  }

  float rval = 0;
  if (parser.seenval('R')) {
    rval = parser.value_float();
    DEBUG_ECHOLNPGM("R", rval);
  }

  feedRate_t fval = slow_feedrate;
  if (parser.seenval('F')) {
    fval = parser.value_feedrate();
    DEBUG_ECHOLNPGM("F", fval);
  }

  const float z_zero = current_position.z - rval, z_final = z_zero + zval;

  destination.set(current_position.x, current_position.y);
  line_to_z(z_zero + rval, fval);

  float n_d = qval ? z_zero + rval - qval : z_final;

  for (; z_final < n_d; n_d -= qval) {
    line_to_z(n_d, fval);
    if (pval) dwell(pval);
    line_to_z(z_zero + rval, fast_feedrate);
  }

  line_to_z(z_zero + zval, feedrate_mm_s);
  if (pval) dwell(pval);
  line_to_z(z_zero + rval, fast_feedrate);
}

/**
 * G82 Spot Drill Canned Cycle
 *
 *  X Y - Hole position
 *    Z - Hole Depth - Relative to the Retraction plan R
 *    R - Position on Retraction plan
 *    Q - Max Depth for each Cut/Peck
 *    P - Dwell time on bottom of each Cut/Peck
 *    F - Cutting feedrate
 */
void GcodeSuite::G82() {
  if (!IsRunning()) return;

  static constexpr float qval = 0;
  const float slow_feedrate = feedrate_mm_s,
              fast_feedrate = get_fast_feedrate();

  float zval = 0;
  if (parser.seenval('Z')) {
    zval = parser.value_float();
    DEBUG_ECHOLNPGM("Z", zval);
  }

  float rval = 0;
  if (parser.seenval('R')) {
    rval = parser.value_float();
    DEBUG_ECHOLNPGM("R", rval);
  }

  millis_t pval = 0;
  if (parser.seenval('P')) {
    pval = parser.value_millis();
    DEBUG_ECHOLNPGM("P", pval);
  }

  feedRate_t fval = slow_feedrate;
  if (parser.seenval('F')) {
    fval = parser.value_feedrate();
    DEBUG_ECHOLNPGM("F", fval);
  }

  const float z_zero = current_position.z - rval, z_final = z_zero + zval;

  destination.set(current_position.x, current_position.y);
  line_to_z(z_zero + rval, fval);

  float n_d = qval ? z_zero + rval - qval : z_final;

  for (; z_final < n_d; n_d -= qval) {
    line_to_z(n_d, fval);
    if (pval) dwell(pval);
    line_to_z(z_zero + rval, fast_feedrate);
  }

  line_to_z(z_zero + zval, fval);

  if (pval) dwell(pval);

  line_to_z(z_zero + rval, fast_feedrate);
}

/**
 * G83 Pecking Drill Canned Cycle
 *
 *  X Y - Hole position
 *    Z - Hole Depth - Relative to the Retraction plan R
 *    R - Position on Retraction plan
 *    Q - Max Depth for each Cut/Peck
 *    P - Dwell time on bottom of each Cut/Peck
 *    F - Cutting feedrate
 */
void GcodeSuite::G83() {
  if (!IsRunning()) return;

  const float slow_feedrate = feedrate_mm_s,
              fast_feedrate = get_fast_feedrate();

  float zval = 0;
  if (parser.seenval('Z')) {
    zval = parser.value_float();
    DEBUG_ECHOLNPGM("Z", zval);
  }

  float rval = 0;
  if (parser.seenval('R')) {
    rval = parser.value_float();
    DEBUG_ECHOLNPGM("R", rval);
  }

  float qval = 0;
  if (parser.seenval('Q')) {
    qval = parser.value_float();
    DEBUG_ECHOLNPGM("Q", qval);
  }

  millis_t pval = 0;
  if (parser.seenval('P')) {
    pval = parser.value_millis();
    DEBUG_ECHOLNPGM("P", pval);
  }

  feedRate_t fval = slow_feedrate;
  if (parser.seenval('F')) {
    fval = parser.value_feedrate();
    DEBUG_ECHOLNPGM("F", fval);
  }

  const float z_zero = current_position.z - rval, z_final = z_zero + zval;

  destination.set(current_position.x, current_position.y);
  line_to_z(z_zero + rval, fval);

  float n_d = qval ? z_zero + rval - qval : z_final;

  for (; z_final < n_d; n_d -= qval) {
    line_to_z(n_d, fval);
    if (pval) dwell(pval);
    line_to_z(z_zero + rval, fast_feedrate);
  }

  line_to_z(z_zero + zval, fval);
  if (pval) dwell(pval);
  line_to_z(z_zero + rval, fast_feedrate);
}

#endif // CNC_DRILLING_CYCLE
