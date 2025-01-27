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

#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/stepper.h" // for various

#if HAS_HOMING_CURRENT
  #include "../../module/motion.h" // for set/restore_homing_current
#endif

#if HAS_MULTI_HOTEND
  #include "../../module/tool_change.h"
#endif

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../../feature/tmc_util.h"
#endif

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../../feature/bltouch.h"
#endif

#if FT_MOTION_DISABLE_FOR_PROBING
  #include "../../module/ft_motion.h"
#endif

#include "../../lcd/marlinui.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../lcd/extui/ui_api.h"
#elif ENABLED(DWIN_CREALITY_LCD)
  #include "../../lcd/e3v2/creality/dwin.h"
#elif ENABLED(SOVOL_SV06_RTS)
  #include "../../lcd/sovol_rts/sovol_rts.h"
#endif

#if ENABLED(LASER_FEATURE)
  #include "../../feature/spindle_laser.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../core/debug_out.h"

#if ENABLED(QUICK_HOME)

  static void quick_home_xy() {

    // Pretend the current position is 0,0
    current_position.set(0.0, 0.0);
    sync_plan_position();

    const int x_axis_home_dir = TOOL_X_HOME_DIR(active_extruder);

    // Use a higher diagonal feedrate so axes move at homing speed
    const float minfr = _MIN(homing_feedrate(X_AXIS), homing_feedrate(Y_AXIS)),
                fr_mm_s = HYPOT(minfr, minfr);

    // Set homing current to X and Y axis if defined
    #if HAS_CURRENT_HOME(X)
      set_homing_current(X_AXIS);
    #endif
    #if HAS_CURRENT_HOME(Y) && NONE(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
      set_homing_current(Y_AXIS);
    #endif

    #if ENABLED(SENSORLESS_HOMING)
      sensorless_t stealth_states {
        NUM_AXIS_LIST(
          TERN0(X_SENSORLESS, tmc_enable_stallguard(stepperX)),
          TERN0(Y_SENSORLESS, tmc_enable_stallguard(stepperY)),
          false, false, false, false, false, false, false
        )
        , TERN0(X2_SENSORLESS, tmc_enable_stallguard(stepperX2))
        , TERN0(Y2_SENSORLESS, tmc_enable_stallguard(stepperY2))
      };
    #endif

    do_blocking_move_to_xy(1.5 * max_length(X_AXIS) * x_axis_home_dir, 1.5 * max_length(Y_AXIS) * Y_HOME_DIR, fr_mm_s);

    endstops.validate_homing_move();

    current_position.set(0.0, 0.0);

    #if HAS_CURRENT_HOME(X)
      restore_homing_current(X_AXIS);
    #endif
    #if HAS_CURRENT_HOME(Y) && NONE(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)
      restore_homing_current(Y_AXIS);
    #endif

    #if ENABLED(SENSORLESS_HOMING) && DISABLED(ENDSTOPS_ALWAYS_ON_DEFAULT)
      TERN_(X_SENSORLESS, tmc_disable_stallguard(stepperX, stealth_states.x));
      TERN_(X2_SENSORLESS, tmc_disable_stallguard(stepperX2, stealth_states.x2));
      TERN_(Y_SENSORLESS, tmc_disable_stallguard(stepperY, stealth_states.y));
      TERN_(Y2_SENSORLESS, tmc_disable_stallguard(stepperY2, stealth_states.y2));
    #endif
  }

#endif // QUICK_HOME

#if ENABLED(Z_SAFE_HOMING)

  inline void home_z_safely() {

    #if FT_MOTION_DISABLE_FOR_PROBING
      FTMotionDisableInScope FT_Disabler; // Disable Fixed-Time Motion for homing
    #endif

    DEBUG_SECTION(log_G28, "home_z_safely", DEBUGGING(LEVELING));

    // Disallow Z homing if X or Y homing is needed
    if (homing_needed_error(_BV(X_AXIS) | _BV(Y_AXIS))) return;

    sync_plan_position();

    /**
     * Move the Z probe (or just the nozzle) to the safe homing point
     * (Z is already at the right height)
     */
    constexpr xy_float_t safe_homing_xy = { Z_SAFE_HOMING_X_POINT, Z_SAFE_HOMING_Y_POINT };
    destination.set(safe_homing_xy, current_position.z);

    TERN_(HOMING_Z_WITH_PROBE, destination -= probe.offset_xy);

    if (position_is_reachable(destination)) {

      if (DEBUGGING(LEVELING)) DEBUG_POS("home_z_safely", destination);

      // Free the active extruder for movement
      TERN_(DUAL_X_CARRIAGE, idex_set_parked(false));

      TERN_(SENSORLESS_HOMING, safe_delay(500)); // Short delay needed to settle

      do_blocking_move_to_xy(destination);
      homeaxis(Z_AXIS);
    }
    else {
      LCD_MESSAGE(MSG_ZPROBE_OUT);
      SERIAL_ECHO_MSG(STR_ZPROBE_OUT_SER);
    }
  }

#endif // Z_SAFE_HOMING

#if ENABLED(IMPROVE_HOMING_RELIABILITY)

  motion_state_t begin_slow_homing() {
    motion_state_t motion_state{0};
    motion_state.acceleration.set(planner.settings.max_acceleration_mm_per_s2[X_AXIS],
                                 planner.settings.max_acceleration_mm_per_s2[Y_AXIS]
                                 OPTARG(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS])
                               );
    planner.settings.max_acceleration_mm_per_s2[X_AXIS] = 100;
    planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = 100;
    TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = 100);
    #if ENABLED(CLASSIC_JERK)
      motion_state.jerk_state = planner.max_jerk;
      planner.max_jerk.set(0, 0 OPTARG(DELTA, 0));
    #endif
    planner.refresh_acceleration_rates();
    return motion_state;
  }

  void end_slow_homing(const motion_state_t &motion_state) {
    planner.settings.max_acceleration_mm_per_s2[X_AXIS] = motion_state.acceleration.x;
    planner.settings.max_acceleration_mm_per_s2[Y_AXIS] = motion_state.acceleration.y;
    TERN_(DELTA, planner.settings.max_acceleration_mm_per_s2[Z_AXIS] = motion_state.acceleration.z);
    TERN_(CLASSIC_JERK, planner.max_jerk = motion_state.jerk_state);
    planner.refresh_acceleration_rates();
  }

#endif // IMPROVE_HOMING_RELIABILITY

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 *  L<bool>   Force leveling state ON (if possible) or OFF after homing (Requires RESTORE_LEVELING_AFTER_G28 or ENABLE_LEVELING_AFTER_G28)
 *  O         Home only if the position is not known and trusted
 *  R<linear> Raise by n mm/inches before homing
 *  H         Hold the current X/Y position when executing a home Z, or if
 *            multiple axes are homed, the position when Z home is executed.
 *            When using a probe for Z Home, positions close to the edge may
 *            fail with position unreachable due to probe/nozzle offset.  This
 *            can be used to avoid a model.
 *
 * Cartesian/SCARA parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 */
void GcodeSuite::G28() {
  DEBUG_SECTION(log_G28, "G28", DEBUGGING(LEVELING));
  if (DEBUGGING(LEVELING)) log_machine_info();

  #if ENABLED(MARLIN_DEV_MODE)
    if (parser.seen_test('S')) {
      LOOP_NUM_AXES(a) set_axis_is_at_home((AxisEnum)a);
      sync_plan_position();
      SERIAL_ECHOLNPGM("Simulated Homing");
      report_current_position();
      return;
    }
  #endif

  /**
   * Set the laser power to false to stop the planner from processing the current power setting.
   */
  #if ENABLED(LASER_FEATURE)
    planner.laser_inline.status.isPowered = false;
  #endif

  // Home (O)nly if position is unknown
  if (!axes_should_home() && parser.seen_test('O')) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("> homing not needed, skip");
    return;
  }

  #if ENABLED(FULL_REPORT_TO_HOST_FEATURE)
    const M_StateEnum old_grblstate = M_State_grbl;
    set_and_report_grblstate(M_HOMING);
  #endif

  TERN_(DWIN_CREALITY_LCD, dwinHomingStart());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingStart());

  planner.synchronize();          // Wait for planner moves to finish!

  // Count this command as movement / activity
  reset_stepper_timeout();

  #if NUM_AXES

    #if ENABLED(DUAL_X_CARRIAGE)
      bool IDEX_saved_duplication_state = extruder_duplication_enabled;
      DualXMode IDEX_saved_mode = dual_x_carriage_mode;
    #endif

    SET_SOFT_ENDSTOP_LOOSE(false);  // Reset a leftover 'loose' motion state

    // Disable the leveling matrix before homing
    #if CAN_SET_LEVELING_AFTER_G28
      const bool leveling_restore_state = parser.boolval('L', TERN1(RESTORE_LEVELING_AFTER_G28, planner.leveling_active));
    #endif

    // Cancel any prior G29 session
    TERN_(PROBE_MANUALLY, g29_in_progress = false);

    // Disable leveling before homing
    TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

    // Reset to the XY plane
    TERN_(CNC_WORKSPACE_PLANES, workspace_plane = PLANE_XY);

    #if ENABLED(IMPROVE_HOMING_RELIABILITY)
      motion_state_t saved_motion_state = begin_slow_homing();
    #endif

    #if FT_MOTION_DISABLE_FOR_PROBING
      FTMotionDisableInScope FT_Disabler; // Disable Fixed-Time Motion for homing
    #endif

    // Always home with tool 0 active
    #if HAS_MULTI_HOTEND
      #if DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE)
        const uint8_t old_tool_index = active_extruder;
      #endif
      // PARKING_EXTRUDER homing requires different handling of movement / solenoid activation, depending on the side of homing
      #if ENABLED(PARKING_EXTRUDER)
        const bool pe_final_change_must_unpark = parking_extruder_unpark_after_homing(old_tool_index, X_HOME_DIR + 1 == old_tool_index * 2);
      #endif
      tool_change(0, true);
    #endif

    TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));

    remember_feedrate_scaling_off();

    endstops.enable(true); // Enable endstops for next homing move

    #if HAS_Z_AXIS
      bool finalRaiseZ = false;
    #endif

    #if ENABLED(DELTA)

      constexpr bool doZ = true; // for NANODLP_Z_SYNC if your DLP is on a DELTA

      home_delta();

      TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));

    #elif ENABLED(AXEL_TPARA)

      constexpr bool doZ = true; // for NANODLP_Z_SYNC if your DLP is on a TPARA

      home_TPARA();

    #else // !DELTA && !AXEL_TPARA

      #define _UNSAFE(A) TERN0(Z_SAFE_HOMING, homeZZ && axis_should_home(_AXIS(A)))

      const bool homeZZ = TERN0(HAS_Z_AXIS, parser.seen_test('Z')),
                 NUM_AXIS_LIST_(             // Other axes should be homed before Z safe-homing
                   needX = _UNSAFE(X), needY = _UNSAFE(Y), needZ = false, // UNUSED
                   needI = _UNSAFE(I), needJ = _UNSAFE(J), needK = _UNSAFE(K),
                   needU = _UNSAFE(U), needV = _UNSAFE(V), needW = _UNSAFE(W)
                 )
                 NUM_AXIS_LIST_(             // Home each axis if needed or flagged
                   homeX = needX || parser.seen_test('X'),
                   homeY = needY || parser.seen_test('Y'),
                   homeZ = homeZZ,
                   homeI = needI || parser.seen_test(AXIS4_NAME), homeJ = needJ || parser.seen_test(AXIS5_NAME),
                   homeK = needK || parser.seen_test(AXIS6_NAME), homeU = needU || parser.seen_test(AXIS7_NAME),
                   homeV = needV || parser.seen_test(AXIS8_NAME), homeW = needW || parser.seen_test(AXIS9_NAME)
                 )
                 home_all = NUM_AXIS_GANG_(  // Home-all if all or none are flagged
                      homeX == homeX, && homeY == homeX, && homeZ == homeX,
                   && homeI == homeX, && homeJ == homeX, && homeK == homeX,
                   && homeU == homeX, && homeV == homeX, && homeW == homeX
                 )
                 NUM_AXIS_LIST(
                   doX = home_all || homeX, doY = home_all || homeY, doZ = home_all || homeZ,
                   doI = home_all || homeI, doJ = home_all || homeJ, doK = home_all || homeK,
                   doU = home_all || homeU, doV = home_all || homeV, doW = home_all || homeW
                 );

      #if !HAS_Y_AXIS
        constexpr bool doY = false;
      #endif

      #if HAS_Z_AXIS

        UNUSED(needZ);

        // Z may home first, e.g., when homing away from the bed.
        // This is also permitted when homing with a Z endstop.
        if (TERN0(HOME_Z_FIRST, doZ)) homeaxis(Z_AXIS);

        // 'R' to specify a specific raise. 'R0' indicates no raise, e.g., for recovery.resume
        // When 'R0' is used, there should already be adequate clearance, e.g., from homing Z to max.
        const bool seenR = parser.seenval('R');

        // Use raise given by 'R' or Z_CLEARANCE_FOR_HOMING (above the probe trigger point)
        float z_homing_height = seenR ? parser.value_linear_units() : Z_CLEARANCE_FOR_HOMING;

        // Check for any lateral motion that might require clearance
        const bool may_skate = seenR NUM_AXIS_GANG(|| doX, || doY, || TERN0(Z_SAFE_HOMING, doZ), || doI, || doJ, || doK, || doU, || doV, || doW);

        if (seenR && z_homing_height == 0) {
          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("R0 = No Z raise");
        }
        else {
          bool with_probe = ENABLED(HOMING_Z_WITH_PROBE);
          // Raise above the current Z (which should be synced in the planner)
          // The "height" for Z is a coordinate. But if Z is not trusted/homed make it relative.
          if (seenR || !(z_min_trusted || axis_should_home(Z_AXIS))) {
            z_homing_height += current_position.z;
            with_probe = false;
          }

          if (may_skate) {
            // Apply Z clearance before doing any lateral motion
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Raise Z before homing:");
            do_z_clearance(z_homing_height, with_probe);
          }
        }

        // Init BLTouch ahead of any lateral motion, even if not homing with the probe
        TERN_(BLTOUCH, if (may_skate) bltouch.init());

      #endif // HAS_Z_AXIS

      // Diagonal move first if both are homing
      TERN_(QUICK_HOME, if (doX && doY) quick_home_xy());

      #if HAS_Y_AXIS
        // Home Y (before X)
        if (ENABLED(HOME_Y_BEFORE_X) && (doY || TERN0(CODEPENDENT_XY_HOMING, doX)))
          homeaxis(Y_AXIS);
      #endif

      // Home X
      #if HAS_X_AXIS
        if (doX || (doY && ENABLED(CODEPENDENT_XY_HOMING) && DISABLED(HOME_Y_BEFORE_X))) {

          #if ENABLED(DUAL_X_CARRIAGE)

            // Always home the 2nd (right) extruder first
            active_extruder = 1;
            homeaxis(X_AXIS);

            // Remember this extruder's position for later tool change
            inactive_extruder_x = current_position.x;

            // Home the 1st (left) extruder
            active_extruder = 0;
            homeaxis(X_AXIS);

            // Consider the active extruder to be in its "parked" position
            idex_set_parked();

          #else

            homeaxis(X_AXIS);

          #endif
        }
      #endif // HAS_X_AXIS

      #if ALL(FOAMCUTTER_XYUV, HAS_I_AXIS)
        // Home I (after X)
        if (doI) homeaxis(I_AXIS);
      #endif

      #if HAS_Y_AXIS
        // Home Y (after X)
        if (DISABLED(HOME_Y_BEFORE_X) && doY) homeaxis(Y_AXIS);
      #endif

      #if ALL(FOAMCUTTER_XYUV, HAS_J_AXIS)
        // Home J (after Y)
        if (doJ) homeaxis(J_AXIS);
      #endif

      TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));

      #if ENABLED(FOAMCUTTER_XYUV)

        // Skip homing of unused Z axis for foamcutters
        if (doZ) set_axis_is_at_home(Z_AXIS);

      #elif HAS_Z_AXIS

        // Home Z last if homing towards the bed
        #if DISABLED(HOME_Z_FIRST)
          if (doZ) {
            #if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN)
              stepper.set_all_z_lock(false);
              stepper.set_separate_multi_axis(false);
            #endif

            #if ENABLED(Z_SAFE_HOMING)
              // H means hold the current X/Y position when probing.
              // Otherwise move to the define safe X/Y position before homing Z.
              if (!parser.seen_test('H'))
                home_z_safely();
              else
                homeaxis(Z_AXIS);
            #else
              homeaxis(Z_AXIS);
            #endif

            #if ANY(Z_HOME_TO_MIN, ALLOW_Z_AFTER_HOMING)
              finalRaiseZ = true;
            #endif
          }
        #endif

        SECONDARY_AXIS_CODE(
          if (doI) homeaxis(I_AXIS),
          if (doJ) homeaxis(J_AXIS),
          if (doK) homeaxis(K_AXIS),
          if (doU) homeaxis(U_AXIS),
          if (doV) homeaxis(V_AXIS),
          if (doW) homeaxis(W_AXIS)
        );

      #endif // HAS_Z_AXIS

      sync_plan_position();

    #endif

    /**
     * Preserve DXC mode across a G28 for IDEX printers in DXC_DUPLICATION_MODE.
     * This is important because it lets a user use the LCD Panel to set an IDEX Duplication mode, and
     * then print a standard G-Code file that contains a single print that does a G28 and has no other
     * IDEX specific commands in it.
     */
    #if ENABLED(DUAL_X_CARRIAGE)

      if (idex_is_duplicating()) {

        TERN_(IMPROVE_HOMING_RELIABILITY, saved_motion_state = begin_slow_homing());

        // Always home the 2nd (right) extruder first
        active_extruder = 1;
        homeaxis(X_AXIS);

        // Remember this extruder's position for later tool change
        inactive_extruder_x = current_position.x;

        // Home the 1st (left) extruder
        active_extruder = 0;
        homeaxis(X_AXIS);

        // Consider the active extruder to be parked
        idex_set_parked();

        dual_x_carriage_mode = IDEX_saved_mode;
        set_duplication_enabled(IDEX_saved_duplication_state);

        TERN_(IMPROVE_HOMING_RELIABILITY, end_slow_homing(saved_motion_state));
      }

    #endif // DUAL_X_CARRIAGE

    endstops.not_homing();

    // Clear endstop state for polled stallGuard endstops
    TERN_(SPI_ENDSTOPS, endstops.clear_endstop_state());

    // Move to a height where we can use the full xy-area
    TERN_(DELTA_HOME_TO_SAFE_ZONE, do_blocking_move_to_z(delta_clip_start_height));

    #if HAS_Z_AXIS
      // Move to the configured Z only if Z was homed to MIN, because machines that
      // home to MAX historically expect 'G28 Z' to be safe to use at the end of a
      // print, and do_move_after_z_homing is not very nuanced.
      if (finalRaiseZ) do_move_after_z_homing();
    #endif

    TERN_(CAN_SET_LEVELING_AFTER_G28, if (leveling_restore_state) set_bed_leveling_enabled());

    // Restore the active tool after homing
    #if HAS_MULTI_HOTEND && (DISABLED(DELTA) || ENABLED(DELTA_HOME_TO_SAFE_ZONE))
      tool_change(old_tool_index, TERN(PARKING_EXTRUDER, !pe_final_change_must_unpark, DISABLED(DUAL_X_CARRIAGE)));   // Do move if one of these
    #endif

    #ifdef XY_AFTER_HOMING
      if (!axes_should_home(_BV(X_AXIS) | _BV(Y_AXIS)))
        do_blocking_move_to(xy_pos_t(XY_AFTER_HOMING));
    #endif

    restore_feedrate_and_scaling();

    if (ENABLED(NANODLP_Z_SYNC) && (ENABLED(NANODLP_ALL_AXIS) || TERN0(HAS_Z_AXIS, doZ)))
      SERIAL_ECHOLNPGM(STR_Z_MOVE_COMP);

  #endif // NUM_AXES

  ui.refresh();

  TERN_(SOVOL_SV06_RTS, RTS_MoveAxisHoming());
  TERN_(DWIN_CREALITY_LCD, dwinHomingDone());
  TERN_(EXTENSIBLE_UI, ExtUI::onHomingDone());

  report_current_position();

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(old_grblstate));

  #ifdef EVENT_GCODE_AFTER_HOMING
    gcode.process_subcommands_now(F(EVENT_GCODE_AFTER_HOMING));
  #endif

}
