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

#if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN)

#include "../../feature/z_stepper_align.h"

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/stepper.h"
#include "../../module/planner.h"
#include "../../module/probe.h"
#include "../../lcd/marlinui.h" // for LCD_MESSAGE

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_Z_STEPPER_ALIGN_STEPPER_XY
  #include "../../libs/least_squares_fit.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../../feature/bltouch.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if NUM_Z_STEPPERS >= 3
  #define TRIPLE_Z 1
  #if NUM_Z_STEPPERS >= 4
    #define QUAD_Z 1
  #endif
#endif

/**
 * G34: Z-Stepper automatic alignment
 *
 * Manual stepper lock controls (reset by G28):
 *   L                 Unlock all steppers
 *   Z<1-4>            Z stepper to lock / unlock
 *   S<state>          0=UNLOCKED 1=LOCKED. If omitted, assume LOCKED.
 *
 *   Examples:
 *     G34 Z1     ; Lock Z1
 *     G34 L Z2   ; Unlock all, then lock Z2
 *     G34 Z2 S0  ; Unlock Z2
 *
 * With Z_STEPPER_AUTO_ALIGN:
 *   I<iterations>     Number of tests. If omitted, Z_STEPPER_ALIGN_ITERATIONS.
 *   T<accuracy>       Target Accuracy factor. If omitted, Z_STEPPER_ALIGN_ACC.
 *   A<amplification>  Provide an Amplification value. If omitted, Z_STEPPER_ALIGN_AMP.
 *   R                 Flag to recalculate points based on current probe offsets
 */
void GcodeSuite::G34() {

  DEBUG_SECTION(log_G34, "G34", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) log_machine_info();

  planner.synchronize();  // Prevent damage

  const bool seenL = parser.seen('L');
  if (seenL) stepper.set_all_z_lock(false);

  const bool seenZ = parser.seenval('Z');
  if (seenZ) {
    const bool state = parser.boolval('S', true);
    switch (parser.intval('Z')) {
      case 1: stepper.set_z1_lock(state); break;
      case 2: stepper.set_z2_lock(state); break;
      #if TRIPLE_Z
        case 3: stepper.set_z3_lock(state); break;
        #if QUAD_Z
          case 4: stepper.set_z4_lock(state); break;
        #endif
      #endif
    }
  }

  if (seenL || seenZ) {
    stepper.set_separate_multi_axis(seenZ);
    return;
  }

  #if ENABLED(Z_STEPPER_AUTO_ALIGN)

    do { // break out on error

      const int8_t z_auto_align_iterations = parser.intval('I', Z_STEPPER_ALIGN_ITERATIONS);
      if (!WITHIN(z_auto_align_iterations, 1, 30)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(I)teration out of bounds (1-30)."));
        break;
      }

      const float z_auto_align_accuracy = parser.floatval('T', Z_STEPPER_ALIGN_ACC);
      if (!WITHIN(z_auto_align_accuracy, 0.001f, 1.0f)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(T)arget accuracy out of bounds (0.001-1.0)."));
        break;
      }

      const float z_auto_align_amplification = TERN(HAS_Z_STEPPER_ALIGN_STEPPER_XY, Z_STEPPER_ALIGN_AMP, parser.floatval('A', Z_STEPPER_ALIGN_AMP));
      if (!WITHIN(ABS(z_auto_align_amplification), 0.5f, 2.0f)) {
        SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(A)mplification out of bounds (0.5-2.0)."));
        break;
      }

      if (parser.seen('R')) z_stepper_align.reset_to_default();

      const ProbePtRaise raise_after = parser.boolval('E') ? PROBE_PT_STOW : PROBE_PT_RAISE;

      // Disable the leveling matrix before auto-aligning
      #if HAS_LEVELING
        #if ENABLED(RESTORE_LEVELING_AFTER_G34)
          const bool leveling_was_active = planner.leveling_active;
        #endif
        set_bed_leveling_enabled(false);
      #endif

      TERN_(CNC_WORKSPACE_PLANES, workspace_plane = PLANE_XY);

      probe.use_probing_tool();

      TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

      // Compute a worst-case clearance height to probe from. After the first
      // iteration this will be re-calculated based on the actual bed position
      auto magnitude2 = [&](const uint8_t i, const uint8_t j) {
        const xy_pos_t diff = z_stepper_align.xy[i] - z_stepper_align.xy[j];
        return HYPOT2(diff.x, diff.y);
      };
      const float zoffs = (probe.offset.z < 0) ? -probe.offset.z : 0.0f;
      float z_probe = (Z_TWEEN_SAFE_CLEARANCE + zoffs) + (G34_MAX_GRADE) * 0.01f * SQRT(_MAX(0, magnitude2(0, 1)
        #if TRIPLE_Z
          , magnitude2(2, 1), magnitude2(2, 0)
          #if QUAD_Z
            , magnitude2(3, 2), magnitude2(3, 1), magnitude2(3, 0)
          #endif
        #endif
      ));

      // Home before the alignment procedure
      home_if_needed();

      #if !HAS_Z_STEPPER_ALIGN_STEPPER_XY
        float last_z_align_move[NUM_Z_STEPPERS] = ARRAY_N_1(NUM_Z_STEPPERS, 10000.0f);
      #else
        float last_z_align_level_indicator = 10000.0f;
      #endif
      float z_measured[NUM_Z_STEPPERS] = { 0 },
            z_maxdiff = 0.0f,
            amplification = z_auto_align_amplification;

      #if !HAS_Z_STEPPER_ALIGN_STEPPER_XY
        bool adjustment_reverse = false;
      #endif

      #if HAS_STATUS_MESSAGE
        PGM_P const msg_iteration = GET_TEXT(MSG_ITERATION);
        const uint8_t iter_str_len = strlen_P(msg_iteration);
      #endif

      // Final z and iteration values will be used after breaking the loop
      float z_measured_min;
      uint8_t iteration = 0;
      bool err_break = false; // To break out of nested loops
      while (iteration < z_auto_align_iterations) {
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> probing all positions.");

        const int iter = iteration + 1;
        SERIAL_ECHOLNPGM("\nG34 Iteration: ", iter);
        #if HAS_STATUS_MESSAGE
          char str[iter_str_len + 2 + 1];
          sprintf_P(str, msg_iteration, iter);
          ui.set_status(str);
        #endif

        // Initialize minimum value
        z_measured_min =  100000.0f;
        float z_measured_max = -100000.0f;

        // Probe all positions (one per Z-Stepper)
        for (uint8_t i = 0; i < NUM_Z_STEPPERS; ++i) {
          // iteration odd/even --> downward / upward stepper sequence
          const uint8_t iprobe = (iteration & 1) ? NUM_Z_STEPPERS - 1 - i : i;

          xy_pos_t &ppos = z_stepper_align.xy[iprobe];

          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM_P(PSTR("Probing X"), ppos.x, SP_Y_STR, ppos.y);

          // Probe a Z height for each stepper.
          // Probing sanity check is disabled, as it would trigger even in normal cases because
          // current_position.z has been manually altered in the "dirty trick" above.

          if (DEBUGGING(LEVELING))
            DEBUG_ECHOLNPGM(
              "Z_PROBE_LOW_POINT: ", p_float_t(Z_PROBE_LOW_POINT, 2),
              "z_probe: ", p_float_t(z_probe, 2),
              "Probe Tgt: ", p_float_t((Z_PROBE_LOW_POINT) - z_probe * 0.5f, 2)
            );

          const float z_probed_height = probe.probe_at_point(
            DIFF_TERN(HAS_HOME_OFFSET, ppos, xy_pos_t(home_offset)),   // xy
            raise_after,                                               // raise_after
            (DEBUGGING(LEVELING) || DEBUGGING(INFO)) ? 3 : 0,          // verbose_level
            true, false,                                               // probe_relative, sanity_check
            (Z_PROBE_LOW_POINT) - (z_probe * 0.5f),                    // z_min_point
            Z_TWEEN_SAFE_CLEARANCE                                     // z_clearance
          );

          if (DEBUGGING(LEVELING)) {
            DEBUG_ECHOLNPGM_P(PSTR("Probing X"), ppos.x, SP_Y_STR, ppos.y);
            DEBUG_ECHOLNPGM("Height = ", z_probed_height);
          }

          if (isnan(z_probed_height)) {
            SERIAL_ECHOLNPGM(STR_ERR_PROBING_FAILED);
            LCD_MESSAGE(MSG_LCD_PROBING_FAILED);
            err_break = true;
            break;
          }

          // Add height to each value, to provide a more useful target height for
          // the next iteration of probing. This allows adjustments to be made away from the bed.
          z_measured[iprobe] = z_probed_height + (Z_TWEEN_SAFE_CLEARANCE + zoffs); //do we need to add the clearance to this?

          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Z", iprobe + 1, " measured position is ", z_measured[iprobe]);

          // Remember the minimum measurement to calculate the correction later on
          z_measured_min = _MIN(z_measured_min, z_measured[iprobe]);
          z_measured_max = _MAX(z_measured_max, z_measured[iprobe]);
        } // for (i)

        if (err_break) break;

        // Adapt the next probe clearance height based on the new measurements.
        // Safe_height = lowest distance to bed (= highest measurement) plus highest measured misalignment.
        z_maxdiff = z_measured_max - z_measured_min;

        // The intent of the line below seems to be to clamp the probe depth on successive iterations of G34, but in reality if the amplification
        // factor is not completely accurate, this was causing probing to fail as the probe stopped fractions of a mm from the trigger point
        // on the second iteration very reliably. This may be restored with an uncertainty factor at some point, however its usefulness after
        // all probe points have seen a successful probe is questionable.
        //z_probe = (Z_TWEEN_SAFE_CLEARANCE + zoffs) + z_measured_max + z_maxdiff; // Not sure we need z_maxdiff, but leaving it in for safety.

        #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
          // Replace the initial values in z_measured with calculated heights at
          // each stepper position. This allows the adjustment algorithm to be
          // shared between both possible probing mechanisms.

          // This must be done after the next z_probe height is calculated, so that
          // the height is calculated from actual print area positions, and not
          // extrapolated motor movements.

          // Compute the least-squares fit for all probed points.
          // Calculate the Z position of each stepper and store it in z_measured.
          // This allows the actual adjustment logic to be shared by both algorithms.
          linear_fit_data lfd;
          incremental_LSF_reset(&lfd);
          for (uint8_t i = 0; i < NUM_Z_STEPPERS; ++i) {
            SERIAL_ECHOLNPGM("PROBEPT_", i, ": ", z_measured[i]);
            incremental_LSF(&lfd, z_stepper_align.xy[i], z_measured[i]);
          }
          finish_incremental_LSF(&lfd);

          z_measured_min = 100000.0f;
          for (uint8_t i = 0; i < NUM_Z_STEPPERS; ++i) {
            z_measured[i] = -(lfd.A * z_stepper_align.stepper_xy[i].x + lfd.B * z_stepper_align.stepper_xy[i].y + lfd.D);
            z_measured_min = _MIN(z_measured_min, z_measured[i]);
          }

          SERIAL_ECHOLNPGM(
            LIST_N(DOUBLE(NUM_Z_STEPPERS),
              "Calculated Z1=", z_measured[0],
                        " Z2=", z_measured[1],
                        " Z3=", z_measured[2],
                        " Z4=", z_measured[3]
            )
          );
        #endif

        SERIAL_EOL();

        SString<15 + TERN0(TRIPLE_Z, 30) + TERN0(QUAD_Z, 45)> msg(F("1:2="), p_float_t(ABS(z_measured[1] - z_measured[0]), 3));
        #if TRIPLE_Z
          msg.append(F(" 3-2="), p_float_t(ABS(z_measured[2] - z_measured[1]), 3))
             .append(F(" 3-1="), p_float_t(ABS(z_measured[2] - z_measured[0]), 3));
        #endif
        #if QUAD_Z
          msg.append(F(" 4-3="), p_float_t(ABS(z_measured[3] - z_measured[2]), 3))
             .append(F(" 4-2="), p_float_t(ABS(z_measured[3] - z_measured[1]), 3))
             .append(F(" 4-1="), p_float_t(ABS(z_measured[3] - z_measured[0]), 3));
        #endif

        msg.echoln();
        ui.set_status(msg);

        auto decreasing_accuracy = [](const_float_t v1, const_float_t v2) {
          if (v1 < v2 * 0.7f) {
            SERIAL_ECHOLNPGM("Decreasing Accuracy Detected.");
            LCD_MESSAGE(MSG_DECREASING_ACCURACY);
            return true;
          }
          return false;
        };

        #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
          // Check if the applied corrections go in the correct direction.
          // Calculate the sum of the absolute deviations from the mean of the probe measurements.
          // Compare to the last iteration to ensure it's getting better.

          // Calculate mean value as a reference
          float z_measured_mean = 0.0f;
          for (uint8_t zstepper = 0; zstepper < NUM_Z_STEPPERS; ++zstepper) z_measured_mean += z_measured[zstepper];
          z_measured_mean /= NUM_Z_STEPPERS;

          // Calculate the sum of the absolute deviations from the mean value
          float z_align_level_indicator = 0.0f;
          for (uint8_t zstepper = 0; zstepper < NUM_Z_STEPPERS; ++zstepper)
            z_align_level_indicator += ABS(z_measured[zstepper] - z_measured_mean);

          // If it's getting worse, stop and throw an error
          err_break = decreasing_accuracy(last_z_align_level_indicator, z_align_level_indicator);
          if (err_break) break;

          last_z_align_level_indicator = z_align_level_indicator;
        #endif

        // The following correction actions are to be enabled for select Z-steppers only
        stepper.set_separate_multi_axis(true);

        bool success_break = true;
        // Correct the individual stepper offsets
        for (uint8_t zstepper = 0; zstepper < NUM_Z_STEPPERS; ++zstepper) {
          // Calculate current stepper move
          float z_align_move = z_measured[zstepper] - z_measured_min;
          const float z_align_abs = ABS(z_align_move);

          #if !HAS_Z_STEPPER_ALIGN_STEPPER_XY
            // Optimize one iteration's correction based on the first measurements
            if (z_align_abs) amplification = (iteration == 1) ? _MIN(last_z_align_move[zstepper] / z_align_abs, 2.0f) : z_auto_align_amplification;

            // Check for less accuracy compared to last move
            if (decreasing_accuracy(last_z_align_move[zstepper], z_align_abs)) {
              if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Z", zstepper + 1, " last_z_align_move = ", last_z_align_move[zstepper]);
              if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Z", zstepper + 1, " z_align_abs = ", z_align_abs);
              FLIP(adjustment_reverse);
            }

            // Remember the alignment for the next iteration, but only if steppers move,
            // otherwise it would be just zero (in case this stepper was at z_measured_min already)
            if (z_align_abs > 0) last_z_align_move[zstepper] = z_align_abs;
          #endif

          // Stop early if all measured points achieve accuracy target
          if (z_align_abs > z_auto_align_accuracy) success_break = false;

          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Z", zstepper + 1, " corrected by ", z_align_move);

          // Lock all steppers except one
          stepper.set_all_z_lock(true, zstepper);

          #if !HAS_Z_STEPPER_ALIGN_STEPPER_XY
            // Decreasing accuracy was detected so move was inverted.
            // Will match reversed Z steppers on dual steppers. Triple will need more work to map.
            if (adjustment_reverse) {
              z_align_move = -z_align_move;
              if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> Z", zstepper + 1, " correction reversed to ", z_align_move);
            }
          #endif

          // Do a move to correct part of the misalignment for the current stepper
          do_blocking_move_to_z(amplification * z_align_move + current_position.z);
        } // for (zstepper)

        // Back to normal stepper operations
        stepper.set_all_z_lock(false);
        stepper.set_separate_multi_axis(false);

        if (err_break) break;

        if (success_break) {
          SERIAL_ECHOLNPGM("Target accuracy achieved.");
          LCD_MESSAGE(MSG_ACCURACY_ACHIEVED);
          break;
        }

        iteration++;
      } // while (iteration < z_auto_align_iterations)

      if (err_break)
        SERIAL_ECHOLNPGM("G34 aborted.");
      else {
        SERIAL_ECHOLNPGM("Did ", iteration + (iteration != z_auto_align_iterations), " of ", z_auto_align_iterations);
        SERIAL_ECHOLNPGM("Accuracy: ", p_float_t(z_maxdiff, 2));
      }

      // Stow the probe because the last call to probe.probe_at_point(...)
      // leaves the probe deployed when it's successful.
      IF_DISABLED(TOUCH_MI_PROBE, probe.stow());

      #if ENABLED(HOME_AFTER_G34)
        // Home Z after the alignment procedure
        process_subcommands_now(F("G28Z"));
      #else
        // Use the probed height from the last iteration to determine the Z height.
        // z_measured_min is used, because all steppers are aligned to z_measured_min.
        // Ideally, this would be equal to the 'z_probe * 0.5f' which was added earlier.
        if (DEBUGGING(LEVELING))
          DEBUG_ECHOLNPGM(
            "z_measured_min: ", p_float_t(z_measured_min, 2),
            "Z_TWEEN_SAFE_CLEARANCE: ", p_float_t(Z_TWEEN_SAFE_CLEARANCE, 2),
            "zoffs: ", p_float_t(zoffs, 2)
          );

        if (!err_break)
          current_position.z -= z_measured_min - (Z_TWEEN_SAFE_CLEARANCE + zoffs); // We shouldn't want to subtract the clearance from here right? (Depends if we added it further up)
        sync_plan_position();
      #endif

      probe.use_probing_tool(false);

      #if ALL(HAS_LEVELING, RESTORE_LEVELING_AFTER_G34)
        set_bed_leveling_enabled(leveling_was_active);
      #endif

    }while(0);

    probe.use_probing_tool(false);

  #endif // Z_STEPPER_AUTO_ALIGN
}

#endif // Z_MULTI_ENDSTOPS || Z_STEPPER_AUTO_ALIGN

#if ENABLED(Z_STEPPER_AUTO_ALIGN)

/**
 * M422: Set a Z-Stepper automatic alignment XY point.
 *       Use repeatedly to set multiple points.
 *
 *   S<index> : Index of the probe point to set
 *
 * With Z_STEPPER_ALIGN_STEPPER_XY:
 *   W<index> : Index of the Z stepper position to set
 *              The W and S parameters may not be combined.
 *
 * S and W require an X and/or Y parameter
 *   X<pos>   : X position to set (Unchanged if omitted)
 *   Y<pos>   : Y position to set (Unchanged if omitted)
 *
 * R : Recalculate points based on current probe offsets
 */
void GcodeSuite::M422() {

  if (!parser.seen_any()) return M422_report();

  if (parser.seen('R')) {
    z_stepper_align.reset_to_default();
    return;
  }

  const bool is_probe_point = parser.seen_test('S');

  if (TERN0(HAS_Z_STEPPER_ALIGN_STEPPER_XY, is_probe_point && parser.seen_test('W'))) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(S) and (W) may not be combined."));
    return;
  }

  xy_pos_t * const pos_dest = (
    TERN_(HAS_Z_STEPPER_ALIGN_STEPPER_XY, !is_probe_point ? z_stepper_align.stepper_xy :)
    z_stepper_align.xy
  );

  if (!is_probe_point && TERN1(HAS_Z_STEPPER_ALIGN_STEPPER_XY, !parser.seen_test('W'))) {
    SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(S)" TERN_(HAS_Z_STEPPER_ALIGN_STEPPER_XY, " or (W)") " is required."));
    return;
  }

  // Get the Probe Position Index or Z Stepper Index
  int8_t position_index = 1;
  FSTR_P err_string = F("?(S) Probe-position");
  if (is_probe_point)
    position_index = parser.intval('S');
  else {
    #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
      err_string = F("?(W) Z-stepper");
      position_index = parser.intval('W');
    #endif
  }

  if (!WITHIN(position_index, 1, NUM_Z_STEPPERS)) {
    SERIAL_ECHOLN(err_string, F(" index invalid (1.." STRINGIFY(NUM_Z_STEPPERS) ")."));
    return;
  }

  --position_index;

  const xy_pos_t pos = {
    parser.floatval('X', pos_dest[position_index].x),
    parser.floatval('Y', pos_dest[position_index].y)
  };

  if (is_probe_point) {
    if (!probe.can_reach(pos.x, Y_CENTER)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(X) out of bounds."));
      return;
    }
    if (!probe.can_reach(pos)) {
      SERIAL_ECHOLNPGM(GCODE_ERR_MSG("(Y) out of bounds."));
      return;
    }
  }

  pos_dest[position_index] = pos;
}

void GcodeSuite::M422_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading(forReplay, F(STR_Z_AUTO_ALIGN));
  for (uint8_t i = 0; i < NUM_Z_STEPPERS; ++i) {
    report_echo_start(forReplay);
    SERIAL_ECHOLNPGM_P(
      PSTR("  M422 S"), i + 1,
      SP_X_STR, z_stepper_align.xy[i].x,
      SP_Y_STR, z_stepper_align.xy[i].y
    );
  }
  #if HAS_Z_STEPPER_ALIGN_STEPPER_XY
    for (uint8_t i = 0; i < NUM_Z_STEPPERS; ++i) {
      report_echo_start(forReplay);
      SERIAL_ECHOLNPGM_P(
        PSTR("  M422 W"), i + 1,
        SP_X_STR, z_stepper_align.stepper_xy[i].x,
        SP_Y_STR, z_stepper_align.stepper_xy[i].y
      );
    }
  #endif
}

#endif // Z_STEPPER_AUTO_ALIGN
