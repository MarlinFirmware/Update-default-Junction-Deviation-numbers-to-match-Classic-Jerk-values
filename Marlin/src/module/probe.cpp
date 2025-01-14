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

/**
 * module/probe.cpp
 */

#include "../inc/MarlinConfig.h"

#if HAS_BED_PROBE

#include "probe.h"

#include "../libs/buzzer.h"
#include "motion.h"
#include "temperature.h"
#include "endstops.h"

#include "../gcode/gcode.h"
#include "../lcd/marlinui.h"

#include "../MarlinCore.h" // for stop(), disable_e_steppers(), wait_for_user_response()

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BD_SENSOR)
  #include "../feature/bedlevel/bdl/bdl.h"
#endif

#if ENABLED(DELTA)
  #include "delta.h"
#endif

#if ENABLED(SENSORLESS_PROBING)
  abc_float_t offset_sensorless_adj{0};
  float largest_sensorless_adj = 0;
#endif

#if ANY(HAS_QUIET_PROBING, USE_SENSORLESS)
  #include "stepper/indirection.h"
  #if ALL(HAS_QUIET_PROBING, PROBING_ESTEPPERS_OFF)
    #include "stepper.h"
  #endif
  #if USE_SENSORLESS
    #include "../feature/tmc_util.h"
    #if ENABLED(IMPROVE_HOMING_RELIABILITY)
      #include "planner.h"
    #endif
  #endif
#endif

#if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
  #include "../feature/backlash.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h" // for PROMPT_USER_CONTINUE
#endif

#if HAS_Z_SERVO_PROBE
  #include "servo.h"
#endif

#if HAS_PTC
  #include "../feature/probe_temp_comp.h"
#endif

#if ENABLED(X_AXIS_TWIST_COMPENSATION)
  #include "../feature/x_twist.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

Probe probe;

xyz_pos_t Probe::offset; // Initialized by settings.load

#if HAS_PROBE_XY_OFFSET
  const xy_pos_t &Probe::offset_xy = Probe::offset;
#endif

#if ENABLED(SENSORLESS_PROBING)
  Probe::sense_bool_t Probe::test_sensitivity = { true, true, true };
#endif

#if ENABLED(Z_PROBE_SLED)

  #ifndef SLED_DOCKING_OFFSET
    #define SLED_DOCKING_OFFSET 0
  #endif

  /**
   * Method to dock/undock a sled designed by Charles Bell.
   *
   * stow[in]     If false, move to MAX_X and engage the solenoid
   *              If true, move to MAX_X and release the solenoid
   */
  static void dock_sled(const bool stow) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("dock_sled(", stow, ")");

    // Dock sled a bit closer to ensure proper capturing
    do_blocking_move_to_x(X_MAX_POS + SLED_DOCKING_OFFSET - ((stow) ? 1 : 0));

    #if HAS_SOLENOID_1 && DISABLED(EXT_SOLENOID)
      WRITE(SOL1_PIN, !stow); // switch solenoid
    #endif
  }

#elif ENABLED(MAGLEV4)

  // Write trigger pin to release the probe
  inline void maglev_deploy() {
    WRITE(MAGLEV_TRIGGER_PIN, HIGH);
    delay(MAGLEV_TRIGGER_DELAY);
    WRITE(MAGLEV_TRIGGER_PIN, LOW);
  }

  inline void maglev_idle() { do_z_clearance(10); }

#elif ENABLED(TOUCH_MI_PROBE)

  // Move to the magnet to unlock the probe
  inline void run_deploy_moves() {
    #ifndef TOUCH_MI_DEPLOY_XPOS
      #define TOUCH_MI_DEPLOY_XPOS X_MIN_POS
    #elif TOUCH_MI_DEPLOY_XPOS > X_MAX_BED
      TemporaryGlobalEndstopsState unlock_x(false);
    #endif
    #if TOUCH_MI_DEPLOY_YPOS > Y_MAX_BED
      TemporaryGlobalEndstopsState unlock_y(false);
    #endif

    #if ENABLED(TOUCH_MI_MANUAL_DEPLOY)

      const screenFunc_t prev_screen = ui.currentScreen;
      LCD_MESSAGE(MSG_MANUAL_DEPLOY_TOUCHMI);
      ui.return_to_status();

      TERN_(HOST_PROMPT_SUPPORT, hostui.continue_prompt(F("Deploy TouchMI")));
      TERN_(HAS_RESUME_CONTINUE, wait_for_user_response());
      ui.reset_status();
      ui.goto_screen(prev_screen);

    #elif defined(TOUCH_MI_DEPLOY_XPOS) && defined(TOUCH_MI_DEPLOY_YPOS)
      do_blocking_move_to_xy(TOUCH_MI_DEPLOY_XPOS, TOUCH_MI_DEPLOY_YPOS);
    #elif defined(TOUCH_MI_DEPLOY_XPOS)
      do_blocking_move_to_x(TOUCH_MI_DEPLOY_XPOS);
    #elif defined(TOUCH_MI_DEPLOY_YPOS)
      do_blocking_move_to_y(TOUCH_MI_DEPLOY_YPOS);
    #endif
  }

  // Move down to the bed to stow the probe
  // TODO: Handle cases where it would be a bad idea to move down.
  inline void run_stow_moves() {
    const float oldz = current_position.z;
    endstops.enable_z_probe(false);
    do_blocking_move_to_z(TOUCH_MI_RETRACT_Z, homing_feedrate(Z_AXIS));
    do_blocking_move_to_z(oldz, homing_feedrate(Z_AXIS));
  }

#elif ENABLED(Z_PROBE_ALLEN_KEY)

  inline void run_deploy_moves() {
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_1
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t deploy_1 = Z_PROBE_ALLEN_KEY_DEPLOY_1;
      do_blocking_move_to(deploy_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_1_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_2
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t deploy_2 = Z_PROBE_ALLEN_KEY_DEPLOY_2;
      do_blocking_move_to(deploy_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_2_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_3
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t deploy_3 = Z_PROBE_ALLEN_KEY_DEPLOY_3;
      do_blocking_move_to(deploy_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_3_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_4
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t deploy_4 = Z_PROBE_ALLEN_KEY_DEPLOY_4;
      do_blocking_move_to(deploy_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_4_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_DEPLOY_5
      #ifndef Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t deploy_5 = Z_PROBE_ALLEN_KEY_DEPLOY_5;
      do_blocking_move_to(deploy_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_DEPLOY_5_FEEDRATE));
    #endif
  }

  inline void run_stow_moves() {
    #ifdef Z_PROBE_ALLEN_KEY_STOW_1
      #ifndef Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t stow_1 = Z_PROBE_ALLEN_KEY_STOW_1;
      do_blocking_move_to(stow_1, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_1_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_2
      #ifndef Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t stow_2 = Z_PROBE_ALLEN_KEY_STOW_2;
      do_blocking_move_to(stow_2, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_2_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_3
      #ifndef Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t stow_3 = Z_PROBE_ALLEN_KEY_STOW_3;
      do_blocking_move_to(stow_3, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_3_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_4
      #ifndef Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t stow_4 = Z_PROBE_ALLEN_KEY_STOW_4;
      do_blocking_move_to(stow_4, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_4_FEEDRATE));
    #endif
    #ifdef Z_PROBE_ALLEN_KEY_STOW_5
      #ifndef Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE
        #define Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE 0.0f
      #endif
      constexpr xyz_pos_t stow_5 = Z_PROBE_ALLEN_KEY_STOW_5;
      do_blocking_move_to(stow_5, MMM_TO_MMS(Z_PROBE_ALLEN_KEY_STOW_5_FEEDRATE));
    #endif
  }

#elif ENABLED(MAG_MOUNTED_PROBE)

  typedef struct { float fr_mm_min; xyz_pos_t where; } mag_probe_move_t;

  inline void run_deploy_moves() {
    #ifdef MAG_MOUNTED_DEPLOY_1
      constexpr mag_probe_move_t deploy_1 = MAG_MOUNTED_DEPLOY_1;
      do_blocking_move_to(deploy_1.where, MMM_TO_MMS(deploy_1.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_DEPLOY_2
      constexpr mag_probe_move_t deploy_2 = MAG_MOUNTED_DEPLOY_2;
      do_blocking_move_to(deploy_2.where, MMM_TO_MMS(deploy_2.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_DEPLOY_3
      constexpr mag_probe_move_t deploy_3 = MAG_MOUNTED_DEPLOY_3;
      do_blocking_move_to(deploy_3.where, MMM_TO_MMS(deploy_3.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_DEPLOY_4
      constexpr mag_probe_move_t deploy_4 = MAG_MOUNTED_DEPLOY_4;
      do_blocking_move_to(deploy_4.where, MMM_TO_MMS(deploy_4.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_DEPLOY_5
      constexpr mag_probe_move_t deploy_5 = MAG_MOUNTED_DEPLOY_5;
      do_blocking_move_to(deploy_5.where, MMM_TO_MMS(deploy_5.fr_mm_min));
    #endif
  }

  inline void run_stow_moves() {
    #ifdef MAG_MOUNTED_STOW_1
      constexpr mag_probe_move_t stow_1 = MAG_MOUNTED_STOW_1;
      do_blocking_move_to(stow_1.where, MMM_TO_MMS(stow_1.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_STOW_2
      constexpr mag_probe_move_t stow_2 = MAG_MOUNTED_STOW_2;
      do_blocking_move_to(stow_2.where, MMM_TO_MMS(stow_2.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_STOW_3
      constexpr mag_probe_move_t stow_3 = MAG_MOUNTED_STOW_3;
      do_blocking_move_to(stow_3.where, MMM_TO_MMS(stow_3.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_STOW_4
      constexpr mag_probe_move_t stow_4 = MAG_MOUNTED_STOW_4;
      do_blocking_move_to(stow_4.where, MMM_TO_MMS(stow_4.fr_mm_min));
    #endif
    #ifdef MAG_MOUNTED_STOW_5
      constexpr mag_probe_move_t stow_5 = MAG_MOUNTED_STOW_5;
      do_blocking_move_to(stow_5.where, MMM_TO_MMS(stow_5.fr_mm_min));
    #endif
  }

#endif // MAG_MOUNTED_PROBE

#if HAS_QUIET_PROBING

  #ifndef DELAY_BEFORE_PROBING
    #define DELAY_BEFORE_PROBING 25
  #endif

  void Probe::set_probing_paused(const bool dopause) {
    TERN_(PROBING_HEATERS_OFF, thermalManager.pause_heaters(dopause));
    TERN_(PROBING_FANS_OFF, thermalManager.set_fans_paused(dopause));
    TERN_(PROBING_ESTEPPERS_OFF, if (dopause) stepper.disable_e_steppers());
    #if ENABLED(PROBING_STEPPERS_OFF) && DISABLED(DELTA)
      static uint8_t old_trusted;
      if (dopause) {
        old_trusted = axes_trusted;
        stepper.disable_axis(X_AXIS);
        stepper.disable_axis(Y_AXIS);
      }
      else {
        if (TEST(old_trusted, X_AXIS)) stepper.enable_axis(X_AXIS);
        if (TEST(old_trusted, Y_AXIS)) stepper.enable_axis(Y_AXIS);
        axes_trusted = old_trusted;
      }
    #endif
    if (dopause) safe_delay(_MAX(DELAY_BEFORE_PROBING, 25));
  }

#endif // HAS_QUIET_PROBING

FORCE_INLINE void probe_specific_action(const bool deploy) {
  DEBUG_SECTION(log_psa, "Probe::probe_specific_action", DEBUGGING(LEVELING));
  #if ENABLED(PAUSE_BEFORE_DEPLOY_STOW)

    // Start preheating before waiting for user confirmation that the probe is ready.
    TERN_(PREHEAT_BEFORE_PROBING, if (deploy) probe.preheat_for_probing(0, PROBING_BED_TEMP, true));

    FSTR_P const ds_fstr = deploy ? GET_TEXT_F(MSG_MANUAL_DEPLOY) : GET_TEXT_F(MSG_MANUAL_STOW);
    ui.return_to_status();       // To display the new status message
    ui.set_max_status(ds_fstr);  // Set a status message that won't be overwritten by the host
    SERIAL_ECHOLN(deploy ? GET_EN_TEXT_F(MSG_MANUAL_DEPLOY) : GET_EN_TEXT_F(MSG_MANUAL_STOW));

    OKAY_BUZZ();

    #if ENABLED(PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED)
    {
      // Wait for the probe to be attached or detached before asking for explicit user confirmation
      // Allow the user to interrupt
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      TERN_(HAS_RESUME_CONTINUE, wait_for_user = true);
      while (deploy == PROBE_TRIGGERED() && TERN1(HAS_RESUME_CONTINUE, wait_for_user)) idle_no_sleep();
      TERN_(HAS_RESUME_CONTINUE, wait_for_user = false);
      OKAY_BUZZ();
    }
    #endif

    TERN_(HOST_PROMPT_SUPPORT, hostui.continue_prompt(ds_fstr));
    #if ENABLED(DWIN_LCD_PROUI)
      ExtUI::onUserConfirmRequired(ICON_BLTouch, ds_fstr, FPSTR(CONTINUE_STR));
    #elif ENABLED(EXTENSIBLE_UI)
      ExtUI::onUserConfirmRequired(ds_fstr);
    #endif
    TERN_(HAS_RESUME_CONTINUE, wait_for_user_response());

    ui.reset_alert_level();
    //ui.reset_status();

  #endif // PAUSE_BEFORE_DEPLOY_STOW

  #if ENABLED(SOLENOID_PROBE)

    #if HAS_SOLENOID_1
      WRITE(SOL1_PIN, deploy);
    #endif

  #elif ENABLED(MAGLEV4)

    deploy ? maglev_deploy() : maglev_idle();

  #elif ENABLED(Z_PROBE_SLED)

    dock_sled(!deploy);

  #elif ENABLED(BLTOUCH)

    deploy ? bltouch.deploy() : bltouch.stow();

  #elif HAS_Z_SERVO_PROBE

    // i.e., deploy ? DEPLOY_Z_SERVO() : STOW_Z_SERVO();
    servo[Z_PROBE_SERVO_NR].move(servo_angles[Z_PROBE_SERVO_NR][deploy ? 0 : 1]);

    #ifdef Z_SERVO_MEASURE_ANGLE
      // After deploy move back to the measure angle...
      if (deploy) servo[Z_PROBE_SERVO_NR].move(Z_SERVO_MEASURE_ANGLE);
    #endif

    if (TERN0(Z_SERVO_DEACTIVATE_AFTER_STOW, !deploy)) servo[Z_PROBE_SERVO_NR].detach();

  #elif ANY(TOUCH_MI_PROBE, Z_PROBE_ALLEN_KEY, MAG_MOUNTED_PROBE)

    deploy ? run_deploy_moves() : run_stow_moves();

  #elif ENABLED(RACK_AND_PINION_PROBE)

    do_blocking_move_to_x(deploy ? Z_PROBE_DEPLOY_X : Z_PROBE_RETRACT_X);

  #elif DISABLED(PAUSE_BEFORE_DEPLOY_STOW)

    UNUSED(deploy);

  #endif
}

#if ANY(PREHEAT_BEFORE_PROBING, PREHEAT_BEFORE_LEVELING)

  #if ENABLED(PREHEAT_BEFORE_PROBING)
    #ifndef PROBING_NOZZLE_TEMP
      #define PROBING_NOZZLE_TEMP 0
    #endif
    #ifndef PROBING_BED_TEMP
      #define PROBING_BED_TEMP 0
    #endif
  #endif

  /**
   * Do preheating as required before leveling or probing.
   *  - If a preheat input is higher than the current target, raise the target temperature.
   *  - If a preheat input is higher than the current temperature, wait for stabilization.
   */
  void Probe::preheat_for_probing(const celsius_t hotend_temp, const celsius_t bed_temp, const bool early/*=false*/) {
    #if HAS_HOTEND && (PROBING_NOZZLE_TEMP || LEVELING_NOZZLE_TEMP)
      #define WAIT_FOR_NOZZLE_HEAT
    #endif
    #if HAS_HEATED_BED && (PROBING_BED_TEMP || LEVELING_BED_TEMP)
      #define WAIT_FOR_BED_HEAT
    #endif

    if (!early) LCD_MESSAGE(MSG_PREHEATING);

    DEBUG_ECHOPGM("Preheating ");

    #if ENABLED(WAIT_FOR_NOZZLE_HEAT)
      const celsius_t hotendPreheat = hotend_temp > thermalManager.degTargetHotend(0) ? hotend_temp : 0;
      if (hotendPreheat) {
        DEBUG_ECHOPGM("hotend (", hotendPreheat, ")");
        thermalManager.setTargetHotend(hotendPreheat, 0);
      }
    #endif

    #if ENABLED(WAIT_FOR_BED_HEAT)
      const celsius_t bedPreheat = bed_temp > thermalManager.degTargetBed() ? bed_temp : 0;
      if (bedPreheat) {
        if (TERN0(WAIT_FOR_NOZZLE_HEAT, hotendPreheat)) DEBUG_ECHOPGM(" and ");
        DEBUG_ECHOPGM("bed (", bedPreheat, ")");
        thermalManager.setTargetBed(bedPreheat);
      }
    #endif

    DEBUG_EOL();

    if (!early) {
      TERN_(WAIT_FOR_NOZZLE_HEAT, if (hotend_temp > thermalManager.wholeDegHotend(0) + (TEMP_WINDOW)) thermalManager.wait_for_hotend(0));
      TERN_(WAIT_FOR_BED_HEAT,    if (bed_temp    > thermalManager.wholeDegBed() + (TEMP_BED_WINDOW)) thermalManager.wait_for_bed_heating());
    }
  }

#endif

/**
 * Print an error and stop()
 */
void Probe::probe_error_stop() {
  SERIAL_ERROR_START();
  SERIAL_ECHOPGM(STR_STOP_PRE);
  #if ANY(Z_PROBE_SLED, Z_PROBE_ALLEN_KEY)
    SERIAL_ECHOPGM(STR_STOP_UNHOMED);
  #elif ENABLED(BLTOUCH)
    SERIAL_ECHOPGM(STR_STOP_BLTOUCH);
  #endif
  SERIAL_ECHOLNPGM(STR_STOP_POST);
  stop();
}

/**
 * Attempt to deploy or stow the probe
 *
 * Return TRUE if the probe could not be deployed/stowed
 */
bool Probe::set_deployed(const bool deploy, const bool no_return/*=false*/) {
  if (DEBUGGING(LEVELING)) {
    DEBUG_POS("Probe::set_deployed", current_position);
    DEBUG_ECHOLNPGM("deploy=", deploy, " no_return=", no_return);
  }

  if (endstops.z_probe_enabled == deploy) return false;

  // Make room for probe to deploy (or stow)
  // Fix-mounted probe should only raise for deploy
  // unless PAUSE_BEFORE_DEPLOY_STOW is enabled
  #if ANY(FIX_MOUNTED_PROBE, NOZZLE_AS_PROBE) && DISABLED(PAUSE_BEFORE_DEPLOY_STOW)
    const bool z_raise_wanted = deploy;
  #else
    constexpr bool z_raise_wanted = true;
  #endif

  if (z_raise_wanted) {
    const float zdest = DIFF_TERN(HAS_HOTEND_OFFSET, Z_CLEARANCE_DEPLOY_PROBE, hotend_offset[active_extruder].z);
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Raise Z to ", zdest);
    do_z_clearance(zdest);
  }

  #if ANY(Z_PROBE_SLED, Z_PROBE_ALLEN_KEY)
    if (homing_needed_error(TERN_(Z_PROBE_SLED, _BV(X_AXIS)))) {
      probe_error_stop();
      return true;
    }
  #endif

  const xy_pos_t old_xy = current_position;        // Remember location before probe deployment

  #if ENABLED(PROBE_TRIGGERED_WHEN_STOWED_TEST)

    // Only deploy/stow if needed
    if (PROBE_TRIGGERED() == deploy) {
      if (!deploy) endstops.enable_z_probe(false); // Switch off triggered when stowed probes early
                                                   // otherwise an Allen-Key probe can't be stowed.
      probe_specific_action(deploy);
    }

    if (PROBE_TRIGGERED() == deploy) {             // Unchanged after deploy/stow action?
      if (IsRunning()) {
        SERIAL_ERROR_MSG("Z-Probe failed");
        LCD_ALERTMESSAGE_F("Err: ZPROBE");
      }
      stop();
      return true;
    }

  #else

    probe_specific_action(deploy);

  #endif

  // If preheating is required before any probing...
  // TODO: Consider skipping this for things like M401, G34, etc.
  TERN_(PREHEAT_BEFORE_PROBING, if (deploy) preheat_for_probing(PROBING_NOZZLE_TEMP, PROBING_BED_TEMP));

  if (!no_return) do_blocking_move_to(old_xy); // Return to the original location unless handled externally

  endstops.enable_z_probe(deploy);
  return false;
}

/**
 * @brief Move down until the probe triggers or the low limit is reached
 *        Used by run_z_probe to do a single Z probe move.
 *
 * @param  z        Z destination
 * @param  fr_mm_s  Feedrate in mm/s
 * @return true to indicate an error
 *
 * @details Used by run_z_probe to get each bed Z height measurement.
 *          Sets current_position.z to the height where the probe triggered
 *          (according to the Z stepper count). The float Z is propagated
 *          back to the planner.position to preempt any rounding error.
 *
 * @return TRUE if the probe failed to trigger.
 */
bool Probe::probe_down_to_z(const_float_t z, const_feedRate_t fr_mm_s) {
  DEBUG_SECTION(log_probe, "Probe::probe_down_to_z", DEBUGGING(LEVELING));

  #if ALL(HAS_HEATED_BED, WAIT_FOR_BED_HEATER)
    thermalManager.wait_for_bed_heating();
  #endif

  #if ALL(HAS_TEMP_HOTEND, WAIT_FOR_HOTEND)
    thermalManager.wait_for_hotend_heating(active_extruder);
  #endif

  #if ENABLED(BLTOUCH)
    // Ensure the BLTouch is deployed. (Does nothing if already deployed.)
    // Don't deploy with high_speed_mode enabled. The probe already re-deploys itself.
    if (TERN(MEASURE_BACKLASH_WHEN_PROBING, true, !bltouch.high_speed_mode) && bltouch.deploy())
      return true;
  #endif

  #if HAS_Z_SERVO_PROBE && (ENABLED(Z_SERVO_INTERMEDIATE_STOW) || defined(Z_SERVO_MEASURE_ANGLE))
    probe_specific_action(true);  // Always re-deploy in this case
  #endif

  // Disable stealthChop if used. Enable diag1 pin on driver.
  #if ENABLED(SENSORLESS_PROBING)
    sensorless_t stealth_states { false };
    #if HAS_DELTA_SENSORLESS_PROBING
      if (test_sensitivity.x) stealth_states.x = tmc_enable_stallguard(stepperX); // Delta watches all DIAG pins for a stall
      if (test_sensitivity.y) stealth_states.y = tmc_enable_stallguard(stepperY);
    #endif
    if (test_sensitivity.z) {
      stealth_states.z = tmc_enable_stallguard(stepperZ);                         // All machines will check Z-DIAG for stall
      #if ENABLED(Z_MULTI_ENDSTOPS)
        stealth_states.z2 = tmc_enable_stallguard(stepperZ2);
        #if NUM_Z_STEPPERS >= 3
          stealth_states.z3 = tmc_enable_stallguard(stepperZ3);
          #if NUM_Z_STEPPERS >= 4
            stealth_states.z4 = tmc_enable_stallguard(stepperZ4);
          #endif
        #endif
      #endif
    }
    TERN_(IMPROVE_HOMING_RELIABILITY, planner.enable_stall_prevention(true));

    endstops.enable(true);
  #endif // SENSORLESS_PROBING

  TERN_(HAS_QUIET_PROBING, set_probing_paused(true));

  // Move down until the probe is triggered
  do_blocking_move_to_z(z, fr_mm_s);

  // Check to see if the probe was triggered
  const bool probe_triggered = (
    #if HAS_DELTA_SENSORLESS_PROBING
      endstops.trigger_state() & (_BV(X_MAX) | _BV(Y_MAX) | _BV(Z_MAX))
    #else
      TEST(endstops.trigger_state(), Z_MIN_PROBE)
    #endif
  );

  // Offset sensorless probing
  #if HAS_DELTA_SENSORLESS_PROBING
    if (probe_triggered) refresh_largest_sensorless_adj();
  #endif

  TERN_(HAS_QUIET_PROBING, set_probing_paused(false));

  // Re-enable stealthChop if used. Disable diag1 pin on driver.
  #if ENABLED(SENSORLESS_PROBING)
    endstops.not_homing();
    #if HAS_DELTA_SENSORLESS_PROBING
      if (test_sensitivity.x) tmc_disable_stallguard(stepperX, stealth_states.x);
      if (test_sensitivity.y) tmc_disable_stallguard(stepperY, stealth_states.y);
    #endif
    if (test_sensitivity.z) {
      tmc_disable_stallguard(stepperZ, stealth_states.z);
      #if ENABLED(Z_MULTI_ENDSTOPS)
        tmc_disable_stallguard(stepperZ2, stealth_states.z2);
        #if NUM_Z_STEPPERS >= 3
          tmc_disable_stallguard(stepperZ3, stealth_states.z3);
          #if NUM_Z_STEPPERS >= 4
            tmc_disable_stallguard(stepperZ4, stealth_states.z4);
          #endif
        #endif
      #endif
    }
    TERN_(IMPROVE_HOMING_RELIABILITY, planner.enable_stall_prevention(false));
  #endif // SENSORLESS_PROBING

  #if ENABLED(BLTOUCH)
    if (probe_triggered && !bltouch.high_speed_mode && bltouch.stow())
      return true; // Stow in LOW SPEED MODE on every trigger
  #endif

  #if ALL(HAS_Z_SERVO_PROBE, Z_SERVO_INTERMEDIATE_STOW)
    probe_specific_action(false);  //  Always stow
  #endif

  // Clear endstop flags
  endstops.hit_on_purpose();

  // Get Z where the steppers were interrupted
  set_current_from_steppers_for_axis(Z_AXIS);

  // Tell the planner where we actually are
  sync_plan_position();

  return !probe_triggered;
}

#if ENABLED(PROBE_TARE)

  /**
   * @brief Init the tare pin
   *
   * @details Init tare pin to ON state for a strain gauge, otherwise OFF
   */
  void Probe::tare_init() {
    OUT_WRITE(PROBE_TARE_PIN, !PROBE_TARE_STATE);
  }

  /**
   * @brief Tare the Z probe
   *
   * @details Signal to the probe to tare itself
   *
   * @return TRUE if the tare cold not be completed
   */
  bool Probe::tare() {
    #if ALL(PROBE_ACTIVATION_SWITCH, PROBE_TARE_ONLY_WHILE_INACTIVE)
      if (endstops.probe_switch_activated()) {
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Cannot tare an active probe");
        return true;
      }
    #endif

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Taring probe");
    WRITE(PROBE_TARE_PIN, PROBE_TARE_STATE);
    delay(PROBE_TARE_TIME);
    WRITE(PROBE_TARE_PIN, !PROBE_TARE_STATE);
    delay(PROBE_TARE_DELAY);

    endstops.hit_on_purpose();
    return false;
  }
#endif

/**
 * @brief Probe at the current XY (possibly more than once) to find the bed Z.
 *
 * @details Used by probe_at_point to get the bed Z height at the current XY.
 *          Leaves current_position.z at the height where the probe triggered.
 *
 * @param sanity_check Flag to compare the probe result with the expected result
 *                     based on the probe Z offset. If the result is too far away
 *                     (more than Z_PROBE_ERROR_TOLERANCE too early) then throw an error.
 * @param z_min_point Override the minimum probing height (-2mm), to allow deeper probing.
 * @param z_clearance Z clearance to apply on probe failure.
 *
 * @return The Z position of the bed at the current XY or NAN on error.
 */
float Probe::run_z_probe(const bool sanity_check/*=true*/, const_float_t z_min_point/*=Z_PROBE_LOW_POINT*/, const_float_t z_clearance/*=Z_TWEEN_SAFE_CLEARANCE*/) {
  DEBUG_SECTION(log_probe, "Probe::run_z_probe", DEBUGGING(LEVELING));

  const float zoffs = SUM_TERN(HAS_HOTEND_OFFSET, -offset.z, hotend_offset[active_extruder].z);

  auto try_to_probe = [&](PGM_P const plbl, const_float_t z_probe_low_point, const feedRate_t fr_mm_s, const bool scheck) -> bool {
    constexpr float error_tolerance = Z_PROBE_ERROR_TOLERANCE;
    if (DEBUGGING(LEVELING)) {
      DEBUG_ECHOPGM_P(plbl);
      DEBUG_ECHOLNPGM("> try_to_probe(..., ", z_probe_low_point, ", ", fr_mm_s, ", ...)");
    }

    // Tare the probe, if supported
    if (TERN0(PROBE_TARE, tare())) return true;

    // Do a first probe at the fast speed
    const bool probe_fail = probe_down_to_z(z_probe_low_point, fr_mm_s),              // No probe trigger?
               early_fail = (scheck && current_position.z > zoffs + error_tolerance); // Probe triggered too high?
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING) && (probe_fail || early_fail)) {
        DEBUG_ECHOPGM(" Probe fail! - ");
        if (probe_fail) DEBUG_ECHOLNPGM("No trigger.");
        if (early_fail) DEBUG_ECHOLNPGM("Triggered early (above ", zoffs + error_tolerance, "mm)");
      }
    #else
      UNUSED(plbl);
    #endif
    return probe_fail || early_fail;
  };

  // Stop the probe before it goes too low to prevent damage.
  // For known Z probe below the expected trigger point, otherwise -10mm lower.
  const float z_probe_low_point = zoffs + z_min_point -float((!axis_is_trusted(Z_AXIS)) * 10);
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Probe Low Point: ", z_probe_low_point);

  // Double-probing does a fast probe followed by a slow probe
  #if TOTAL_PROBING == 2

    // Attempt to tare the probe
    if (TERN0(PROBE_TARE, tare())) return NAN;

    // Do a first probe at the fast speed
    if (try_to_probe(PSTR("FAST"), z_probe_low_point, z_probe_fast_mm_s, sanity_check)) return NAN;

    const float z1 = DIFF_TERN(HAS_DELTA_SENSORLESS_PROBING, current_position.z, largest_sensorless_adj);
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("1st Probe Z:", z1);

    // Raise to give the probe clearance
    do_z_clearance(z1 + (Z_CLEARANCE_MULTI_PROBE), false);

  #elif Z_PROBE_FEEDRATE_FAST != Z_PROBE_FEEDRATE_SLOW

    // If the nozzle is well over the travel height then
    // move down quickly before doing the slow probe
    const float z = (Z_CLEARANCE_DEPLOY_PROBE) + 5.0f + _MAX(zoffs, 0.0f);
    if (current_position.z > z) {
      // Probe down fast. If the probe never triggered, raise for probe clearance
      if (!probe_down_to_z(z, z_probe_fast_mm_s))
        do_z_clearance(z_clearance);
    }
  #endif

  #if EXTRA_PROBING > 0
    float probes[TOTAL_PROBING];
  #endif

  #if TOTAL_PROBING > 2
    float probes_z_sum = 0;
    for (
      #if EXTRA_PROBING > 0
        uint8_t p = 0; p < TOTAL_PROBING; p++
      #else
        uint8_t p = TOTAL_PROBING; p--;
      #endif
    )
  #endif
    {
      // If the probe won't tare, return
      if (TERN0(PROBE_TARE, tare())) return true;

      // Probe downward slowly to find the bed
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Slow Probe:");
      if (try_to_probe(PSTR("SLOW"), z_probe_low_point, MMM_TO_MMS(Z_PROBE_FEEDRATE_SLOW), sanity_check)) return NAN;

      TERN_(MEASURE_BACKLASH_WHEN_PROBING, backlash.measure_with_probe());

      const float z = DIFF_TERN(HAS_DELTA_SENSORLESS_PROBING, current_position.z, largest_sensorless_adj);

      #if EXTRA_PROBING > 0
        // Insert Z measurement into probes[]. Keep it sorted ascending.
        for (uint8_t i = 0; i <= p; ++i) {                            // Iterate the saved Zs to insert the new Z
          if (i == p || probes[i] > z) {                              // Last index or new Z is smaller than this Z
            for (int8_t m = p; --m >= i;) probes[m + 1] = probes[m];  // Shift items down after the insertion point
            probes[i] = z;                                            // Insert the new Z measurement
            break;                                                    // Only one to insert. Done!
          }
        }
      #elif TOTAL_PROBING > 2
        probes_z_sum += z;
      #else
        UNUSED(z);
      #endif

      #if TOTAL_PROBING > 2
        // Small Z raise after all but the last probe
        if (p
          #if EXTRA_PROBING > 0
            < TOTAL_PROBING - 1
          #endif
        ) do_z_clearance(z + (Z_CLEARANCE_MULTI_PROBE), false);
      #endif
    }

  #if TOTAL_PROBING > 2

    #if EXTRA_PROBING > 0
      // Take the center value (or average the two middle values) as the median
      static constexpr int PHALF = (TOTAL_PROBING - 1) / 2;
      const float middle = probes[PHALF],
                  median = ((TOTAL_PROBING) & 1) ? middle : (middle + probes[PHALF + 1]) * 0.5f;

      // Remove values farthest from the median
      uint8_t min_avg_idx = 0, max_avg_idx = TOTAL_PROBING - 1;
      for (uint8_t i = EXTRA_PROBING; i--;)
        if (ABS(probes[max_avg_idx] - median) > ABS(probes[min_avg_idx] - median))
          max_avg_idx--; else min_avg_idx++;

      // Return the average value of all remaining probes.
      for (uint8_t i = min_avg_idx; i <= max_avg_idx; ++i)
        probes_z_sum += probes[i];

    #endif

    const float measured_z = probes_z_sum * RECIPROCAL(MULTIPLE_PROBING);

  #elif TOTAL_PROBING == 2

    const float z2 = DIFF_TERN(HAS_DELTA_SENSORLESS_PROBING, current_position.z, largest_sensorless_adj);

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("2nd Probe Z:", z2, " Discrepancy:", z1 - z2);

    // Return a weighted average of the fast and slow probes
    const float measured_z = (z2 * 3.0f + z1 * 2.0f) * 0.2f;

  #else

    // Return the single probe result
    const float measured_z = current_position.z;

  #endif

  return DIFF_TERN(HAS_HOTEND_OFFSET, measured_z, hotend_offset[active_extruder].z);
}

#if DO_TOOLCHANGE_FOR_PROBING

  #include "tool_change.h"

  /**
   * Switches to the appropriate tool (PROBING_TOOL) for probing (probing = true), and switches
   * back to the old tool when probing = false. Uses statics to avoid unnecessary checks and to
   * cache the previous tool, so always call with false after calling with true.
   */
  void Probe::use_probing_tool(const bool probing/*=true*/) {
    static uint8_t old_tool;
    static bool old_state = false;
    if (probing == old_state) return;
    old_state = probing;
    if (probing) old_tool = active_extruder;
    const uint8_t tool = probing ? PROBING_TOOL : old_tool;
    if (tool != active_extruder)
      tool_change(tool, ENABLED(PROBE_TOOLCHANGE_NO_MOVE));
  }

#endif

/**
 * - Move to the given XY
 * - Deploy the probe, if not already deployed
 * - Probe the bed, get the Z position
 * - Depending on the 'stow' flag
 *   - Stow the probe, or
 *   - Raise to the BETWEEN height
 * - Return the probed Z position
 * - Revert to previous tool
 *
 * A batch of multiple probing operations should always be preceded by use_probing_tool() invocation
 * and succeeded by use_probing_tool(false), in order to avoid multiple tool changes and to end up
 * with the previously active tool.
 *
 */
float Probe::probe_at_point(
  const_float_t rx, const_float_t ry,
  const ProbePtRaise raise_after,     // = PROBE_PT_NONE
  const uint8_t verbose_level,        // = 0
  const bool probe_relative,          // = true
  const bool sanity_check,            // = true
  const_float_t z_min_point,          // = Z_PROBE_LOW_POINT
  const_float_t z_clearance,          // = Z_TWEEN_SAFE_CLEARANCE
  const bool raise_after_is_rel       // = false
) {
  DEBUG_SECTION(log_probe, "Probe::probe_at_point", DEBUGGING(LEVELING));

  if (DEBUGGING(LEVELING)) {
    DEBUG_ECHOLNPGM(
      "...(", LOGICAL_X_POSITION(rx), ", ", LOGICAL_Y_POSITION(ry),
      ", ", raise_after == PROBE_PT_RAISE ? "raise" : raise_after == PROBE_PT_LAST_STOW ? "stow (last)" : raise_after == PROBE_PT_STOW ? "stow" : "none",
      ", ", verbose_level,
      ", ", probe_relative ? "probe" : "nozzle", "_relative)"
    );
    DEBUG_POS("", current_position);
  }

  // Use a safe Z height for the XY move
  const float safe_z = _MAX(current_position.z, z_clearance);

  // On delta keep Z below clip height or do_blocking_move_to will abort
  xyz_pos_t npos = NUM_AXIS_ARRAY(
    rx, ry, TERN(DELTA, _MIN(delta_clip_start_height, safe_z), safe_z),
    current_position.i, current_position.j, current_position.k,
    current_position.u, current_position.v, current_position.w
  );
  if (!can_reach(npos, probe_relative)) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Not Reachable");
    return NAN;
  }

  if (DEBUGGING(LEVELING)) DEBUG_ECHOPGM("Move to probe");
  if (probe_relative) { // Get the nozzle position, adjust for active hotend if not 0
    if (DEBUGGING(LEVELING)) DEBUG_ECHOPGM("-relative");
    npos -= DIFF_TERN(HAS_HOTEND_OFFSET, offset_xy, xy_pos_t(hotend_offset[active_extruder]));
  }
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM(" point");

  // Move the probe to the starting XYZ
  do_blocking_move_to(npos, feedRate_t(XY_PROBE_FEEDRATE_MM_S));

  // Change Z motor current to homing current
  TERN_(PROBING_USE_CURRENT_HOME, set_homing_current(Z_AXIS));

  float measured_z;

  #if ENABLED(BD_SENSOR)

    safe_delay(4);

    measured_z = current_position.z - bdl.read(); // Difference between Z-home-relative Z and sensor reading

  #else // !BD_SENSOR

    #if ENABLED(BLTOUCH)
      // Now at the safe_z if it is still triggered it may be in an alarm
      // condition.  Reset to clear alarm has a side effect of stowing the probe,
      // which the following deploy will handle.
      if (bltouch.triggered()) bltouch._reset();
    #endif

    measured_z = deploy() ? NAN : run_z_probe(sanity_check, z_min_point, z_clearance) + offset.z;

    // Deploy succeeded and a successful measurement was done.
    // Raise and/or stow the probe depending on 'raise_after' and settings.
    if (!isnan(measured_z)) {
      switch (raise_after) {
        default: break;
        case PROBE_PT_RAISE:
          if (raise_after_is_rel)
            do_z_clearance_by(z_clearance);
          else
            do_z_clearance(z_clearance);
          break;
        case PROBE_PT_STOW: case PROBE_PT_LAST_STOW:
          if (stow()) measured_z = NAN;   // Error on stow?
          break;
      }
    }

    // If any error occurred stow the probe and set an alert
    if (isnan(measured_z)) {
      // TODO: Disable steppers (unless G29_RETRY_AND_RECOVER or G29_HALT_ON_FAILURE are set).
      // Something definitely went wrong at this point, so it might be a good idea to release the steppers.
      // The user may want to quickly move the carriage or bed by hand to avoid bed damage from the (hot) nozzle.
      // This would also benefit from the contemplated "Audio Alerts" feature.
      stow();
      LCD_MESSAGE(MSG_LCD_PROBING_FAILED);
      #if DISABLED(G29_RETRY_AND_RECOVER)
        SERIAL_ERROR_MSG(STR_ERR_PROBING_FAILED);
      #endif
    }
    else {
      TERN_(HAS_PTC, ptc.apply_compensation(measured_z));
      TERN_(X_AXIS_TWIST_COMPENSATION, measured_z += xatc.compensation(npos + offset_xy));
      if (verbose_level > 2 || DEBUGGING(LEVELING))
        SERIAL_ECHOLNPGM("Bed X: ", LOGICAL_X_POSITION(rx), " Y: ", LOGICAL_Y_POSITION(ry), " Z: ", measured_z);
    }

  #endif // !BD_SENSOR

  // Restore the Z homing current
  TERN_(PROBING_USE_CURRENT_HOME, restore_homing_current(Z_AXIS));

  return measured_z;
}

#if HAS_Z_SERVO_PROBE

  void Probe::servo_probe_init() {
    /**
     * Set position of Z Servo Endstop
     *
     * The servo might be deployed and positioned too low to stow
     * when starting up the machine or rebooting the board.
     * There's no way to know where the nozzle is positioned until
     * homing has been done - no homing with z-probe without init!
     */
    STOW_Z_SERVO();

    TERN_(Z_SERVO_DEACTIVATE_AFTER_STOW, servo[Z_PROBE_SERVO_NR].detach());
  }

#endif // HAS_Z_SERVO_PROBE

#if HAS_DELTA_SENSORLESS_PROBING

  /**
   * Set the sensorless Z offset
   */
  void Probe::set_offset_sensorless_adj(const_float_t sz) {
    DEBUG_SECTION(pso, "Probe::set_offset_sensorless_adj", true);
    if (test_sensitivity.x) offset_sensorless_adj.a = sz;
    if (test_sensitivity.y) offset_sensorless_adj.b = sz;
    if (test_sensitivity.z) offset_sensorless_adj.c = sz;
  }

  /**
   * Refresh largest_sensorless_adj based on triggered endstops
   */
  void Probe::refresh_largest_sensorless_adj() {
    DEBUG_SECTION(rso, "Probe::refresh_largest_sensorless_adj", true);
    largest_sensorless_adj = -3;  // A reference away from any real probe height
    const Endstops::endstop_mask_t state = endstops.state();
    if (TEST(state, X_MAX)) {
      NOLESS(largest_sensorless_adj, offset_sensorless_adj.a);
      DEBUG_ECHOLNPGM("Endstop_X: ", largest_sensorless_adj, " TowerX");
    }
    if (TEST(state, Y_MAX)) {
      NOLESS(largest_sensorless_adj, offset_sensorless_adj.b);
      DEBUG_ECHOLNPGM("Endstop_Y: ", largest_sensorless_adj, " TowerY");
    }
    if (TEST(state, Z_MAX)) {
      NOLESS(largest_sensorless_adj, offset_sensorless_adj.c);
      DEBUG_ECHOLNPGM("Endstop_Z: ", largest_sensorless_adj, " TowerZ");
    }
  }

#endif

#endif // HAS_BED_PROBE
