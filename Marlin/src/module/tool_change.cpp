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

#include "../inc/MarlinConfigPre.h"

#if HAS_TOOLCHANGE

#include "tool_change.h"

#include "motion.h"
#include "planner.h"
#include "temperature.h"

#include "../MarlinCore.h"
#include "../gcode/gcode.h"

//#define DEBUG_TOOL_CHANGE
//#define DEBUG_TOOLCHANGE_FILAMENT_SWAP

#if HAS_MULTI_EXTRUDER
  toolchange_settings_t toolchange_settings;  // Initialized by settings.load
#endif

#if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)
  migration_settings_t migration = migration_defaults;
#endif

#if ENABLED(TOOLCHANGE_FS_INIT_BEFORE_SWAP)
  Flags<EXTRUDERS> toolchange_extruder_ready;
#endif

#if ENABLED(TOOL_SENSOR)
  #include "../lcd/marlinui.h"
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  #include "stepper.h"
#endif

#if ANY(SWITCHING_EXTRUDER, SWITCHING_NOZZLE, SWITCHING_TOOLHEAD)
  #include "servo.h"
#endif

#if ENABLED(EXT_SOLENOID) && DISABLED(PARKING_EXTRUDER)
  #include "../feature/solenoid.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if HAS_FANMUX
  #include "../feature/fanmux.h"
#endif

#if HAS_PRUSA_MMU3
  #include "../feature/mmu3/mmu3.h"
#elif HAS_PRUSA_MMU2
  #include "../feature/mmu/mmu2.h"
#elif HAS_PRUSA_MMU1
  #include "../feature/mmu/mmu.h"
#endif

#if HAS_MARLINUI_MENU
  #include "../lcd/marlinui.h"
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)
  #include "../feature/pause.h"
#endif

#if HAS_BED_PROBE
  #include "probe.h"
#endif

#if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
  #if TOOLCHANGE_FS_WIPE_RETRACT <= 0
    #undef TOOLCHANGE_FS_WIPE_RETRACT
    #define TOOLCHANGE_FS_WIPE_RETRACT 0
  #endif
#endif

#if DO_SWITCH_EXTRUDER

  #if EXTRUDERS > 3
    #define _SERVO_NR(E) ((E) < 2 ? SWITCHING_EXTRUDER_SERVO_NR : SWITCHING_EXTRUDER_E23_SERVO_NR)
  #else
    #define _SERVO_NR(E) SWITCHING_EXTRUDER_SERVO_NR
  #endif

  void move_extruder_servo(const uint8_t e) {
    planner.synchronize();
    constexpr bool evenExtruders = !(EXTRUDERS & 1);
    if (evenExtruders || e < EXTRUDERS - 1) {
      servo[_SERVO_NR(e)].move(servo_angles[_SERVO_NR(e)][e & 1]);
      safe_delay(500);
    }
  }

#endif // DO_SWITCH_EXTRUDER

#if ENABLED(SWITCHING_NOZZLE)

  #if SWITCHING_NOZZLE_TWO_SERVOS

    inline void _move_nozzle_servo(const uint8_t e, const uint8_t angle_index) {
      constexpr int8_t  sns_index[2] = { SWITCHING_NOZZLE_SERVO_NR, SWITCHING_NOZZLE_E1_SERVO_NR };
      planner.synchronize();
      servo[sns_index[e]].move(servo_angles[sns_index[e]][angle_index]);
      safe_delay(SWITCHING_NOZZLE_SERVO_DWELL);
    }

    void lower_nozzle(const uint8_t e) { _move_nozzle_servo(e, 0); }
    void raise_nozzle(const uint8_t e) { _move_nozzle_servo(e, 1); }

  #else

    void move_nozzle_servo(const uint8_t angle_index) {
      planner.synchronize();
      servo[SWITCHING_NOZZLE_SERVO_NR].move(servo_angles[SWITCHING_NOZZLE_SERVO_NR][angle_index]);
      safe_delay(SWITCHING_NOZZLE_SERVO_DWELL);
    }

  #endif

#endif // SWITCHING_NOZZLE

// Move to position routines
void _line_to_current(const AxisEnum fr_axis, const float fscale=1) {
  line_to_current_position(planner.settings.max_feedrate_mm_s[fr_axis] * fscale);
}
void slow_line_to_current(const AxisEnum fr_axis) { _line_to_current(fr_axis, 0.2f); }
void fast_line_to_current(const AxisEnum fr_axis) { _line_to_current(fr_axis, 0.5f); }

#define DEBUG_OUT ENABLED(DEBUG_TOOL_CHANGE)
#include "../core/debug_out.h"

#if ENABLED(MAGNETIC_PARKING_EXTRUDER)

  float parkingposx[2],           // M951 R L
        parkinggrabdistance,      // M951 I
        parkingslowspeed,         // M951 J
        parkinghighspeed,         // M951 H
        parkingtraveldistance,    // M951 D
        compensationmultiplier;

  inline void magnetic_parking_extruder_tool_change(const uint8_t new_tool) {

    const float oldx = current_position.x,
                grabpos = mpe_settings.parking_xpos[new_tool] + (new_tool ? mpe_settings.grab_distance : -mpe_settings.grab_distance),
                offsetcompensation = TERN0(HAS_HOTEND_OFFSET, hotend_offset[active_extruder].x * mpe_settings.compensation_factor);

    if (homing_needed_error(_BV(X_AXIS))) return;

    /**
     * Z Lift and Nozzle Offset shift ar defined in caller method to work equal with any Multi Hotend realization
     *
     * Steps:
     *   1. Move high speed to park position of new extruder
     *   2. Move to couple position of new extruder (this also discouple the old extruder)
     *   3. Move to park position of new extruder
     *   4. Move high speed to approach park position of old extruder
     *   5. Move to park position of old extruder
     *   6. Move to starting position
     */

    // STEP 1

    current_position.x = mpe_settings.parking_xpos[new_tool] + offsetcompensation;

    DEBUG_ECHOPGM("(1) Move extruder ", new_tool);
    DEBUG_POS(" to new extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    // STEP 2

    current_position.x = grabpos + offsetcompensation;

    DEBUG_ECHOPGM("(2) Couple extruder ", new_tool);
    DEBUG_POS(" to new extruder GrabPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // Delay before moving tool, to allow magnetic coupling
    gcode.dwell(150);

    // STEP 3

    current_position.x = mpe_settings.parking_xpos[new_tool] + offsetcompensation;

    DEBUG_ECHOPGM("(3) Move extruder ", new_tool);
    DEBUG_POS(" back to new extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // STEP 4

    current_position.x = mpe_settings.parking_xpos[active_extruder] + (active_extruder == 0 ? MPE_TRAVEL_DISTANCE : -MPE_TRAVEL_DISTANCE) + offsetcompensation;

    DEBUG_ECHOPGM("(4) Move extruder ", new_tool);
    DEBUG_POS(" close to old extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    // STEP 5

    current_position.x = mpe_settings.parking_xpos[active_extruder] + offsetcompensation;

    DEBUG_ECHOPGM("(5) Park extruder ", new_tool);
    DEBUG_POS(" at old extruder ParkPos", current_position);

    planner.buffer_line(current_position, mpe_settings.slow_feedrate, new_tool);
    planner.synchronize();

    // STEP 6

    current_position.x = oldx;

    DEBUG_ECHOPGM("(6) Move extruder ", new_tool);
    DEBUG_POS(" to starting position", current_position);

    planner.buffer_line(current_position, mpe_settings.fast_feedrate, new_tool);
    planner.synchronize();

    DEBUG_ECHOLNPGM("Autopark done.");
  }

#elif ENABLED(PARKING_EXTRUDER)

  void pe_solenoid_init() {
    for (uint8_t n = 0; n <= 1; ++n) pe_solenoid_set_pin_state(n, !PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
  }

  void pe_solenoid_set_pin_state(const uint8_t extruder_num, const uint8_t state) {
    switch (extruder_num) {
      case 1: OUT_WRITE(SOL1_PIN, state); break;
      default: OUT_WRITE(SOL0_PIN, state); break;
    }
    #if PARKING_EXTRUDER_SOLENOIDS_DELAY > 0
      gcode.dwell(PARKING_EXTRUDER_SOLENOIDS_DELAY);
    #endif
  }

  bool extruder_parked = true, do_solenoid_activation = true;

  // Modifies tool_change() behavior based on homing side
  bool parking_extruder_unpark_after_homing(const uint8_t final_tool, bool homed_towards_final_tool) {
    do_solenoid_activation = false; // Tell parking_extruder_tool_change to skip solenoid activation

    if (!extruder_parked) return false; // nothing to do

    if (homed_towards_final_tool) {
      pe_solenoid_magnet_off(1 - final_tool);
      DEBUG_ECHOLNPGM("Disengage magnet", 1 - final_tool);
      pe_solenoid_magnet_on(final_tool);
      DEBUG_ECHOLNPGM("Engage magnet", final_tool);
      parking_extruder_set_parked(false);
      return false;
    }

    return true;
  }

  inline void parking_extruder_tool_change(const uint8_t new_tool, bool no_move) {
    if (!no_move) {

      constexpr float parkingposx[] = PARKING_EXTRUDER_PARKING_X;

      #if HAS_HOTEND_OFFSET
        const float x_offset = hotend_offset[active_extruder].x;
      #else
        constexpr float x_offset = 0;
      #endif

      const float midpos = (parkingposx[0] + parkingposx[1]) * 0.5f + x_offset,
                  grabpos = parkingposx[new_tool] + (new_tool ? PARKING_EXTRUDER_GRAB_DISTANCE : -(PARKING_EXTRUDER_GRAB_DISTANCE)) + x_offset;

      /**
       * 1. Move to park position of old extruder
       * 2. Disengage magnetic field, wait for delay
       * 3. Move near new extruder
       * 4. Engage magnetic field for new extruder
       * 5. Move to parking incl. offset of new extruder
       * 6. Lower Z-Axis
       */

      // STEP 1

      DEBUG_POS("Start PE Tool-Change", current_position);

      // Don't park the active_extruder unless unparked
      if (!extruder_parked) {
        current_position.x = parkingposx[active_extruder] + x_offset;

        DEBUG_ECHOLNPGM("(1) Park extruder ", active_extruder);
        DEBUG_POS("Moving ParkPos", current_position);

        fast_line_to_current(X_AXIS);

        // STEP 2

        planner.synchronize();
        DEBUG_ECHOLNPGM("(2) Disengage magnet");
        pe_solenoid_magnet_off(active_extruder);

        // STEP 3

        current_position.x += active_extruder ? -10 : 10; // move 10mm away from parked extruder

        DEBUG_ECHOLNPGM("(3) Move near new extruder");
        DEBUG_POS("Move away from parked extruder", current_position);

        fast_line_to_current(X_AXIS);
      }

      // STEP 4

      planner.synchronize();
      DEBUG_ECHOLNPGM("(4) Engage magnetic field");

      // Just save power for inverted magnets
      TERN_(PARKING_EXTRUDER_SOLENOIDS_INVERT, pe_solenoid_magnet_on(active_extruder));
      pe_solenoid_magnet_on(new_tool);

      // STEP 5

      current_position.x = grabpos + (new_tool ? -10 : 10);
      fast_line_to_current(X_AXIS);

      current_position.x = grabpos;

      DEBUG_SYNCHRONIZE();
      DEBUG_POS("(5) Unpark extruder", current_position);

      slow_line_to_current(X_AXIS);

      // STEP 6

      current_position.x = DIFF_TERN(HAS_HOTEND_OFFSET, midpos, hotend_offset[new_tool].x);

      DEBUG_SYNCHRONIZE();
      DEBUG_POS("(6) Move midway between hotends", current_position);

      fast_line_to_current(X_AXIS);
      planner.synchronize(); // Always sync the final move

      DEBUG_POS("PE Tool-Change done.", current_position);
      parking_extruder_set_parked(false);
    }
    else if (do_solenoid_activation) {
      // Deactivate current extruder solenoid
      pe_solenoid_set_pin_state(active_extruder, !PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
      // Engage new extruder magnetic field
      pe_solenoid_set_pin_state(new_tool, PARKING_EXTRUDER_SOLENOIDS_PINS_ACTIVE);
    }

    do_solenoid_activation = true; // Activate solenoid for subsequent tool_change()
  }

#endif // PARKING_EXTRUDER

#if ENABLED(TOOL_SENSOR)

  bool tool_sensor_disabled; // = false

  // Return a bitmask of tool sensor states
  inline uint8_t poll_tool_sensor_pins() {
    return (0
      #if PIN_EXISTS(TOOL_SENSOR1)
        | (READ(TOOL_SENSOR1_PIN) << 0)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR2)
        | (READ(TOOL_SENSOR2_PIN) << 1)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR3)
        | (READ(TOOL_SENSOR3_PIN) << 2)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR4)
        | (READ(TOOL_SENSOR4_PIN) << 3)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR5)
        | (READ(TOOL_SENSOR5_PIN) << 4)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR6)
        | (READ(TOOL_SENSOR6_PIN) << 5)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR7)
        | (READ(TOOL_SENSOR7_PIN) << 6)
      #endif
      #if PIN_EXISTS(TOOL_SENSOR8)
        | (READ(TOOL_SENSOR8_PIN) << 7)
      #endif
    );
  }

  uint8_t check_tool_sensor_stats(const uint8_t tool_index, const bool kill_on_error/*=false*/, const bool disable/*=false*/) {
    static uint8_t sensor_tries; // = 0
    for (;;) {
      if (poll_tool_sensor_pins() == _BV(tool_index)) {
        sensor_tries = 0;
        return tool_index;
      }
      else if (kill_on_error && (!tool_sensor_disabled || disable)) {
        sensor_tries++;
        if (sensor_tries > 10) kill(F("Tool Sensor error"));
        safe_delay(5);
      }
      else {
        sensor_tries++;
        if (sensor_tries > 10) return -1;
        safe_delay(5);
      }
    }
  }

#endif // TOOL_SENSOR

#if ENABLED(SWITCHING_TOOLHEAD)

  inline void switching_toolhead_lock(const bool locked) {
    #ifdef SWITCHING_TOOLHEAD_SERVO_ANGLES
      const uint16_t swt_angles[2] = SWITCHING_TOOLHEAD_SERVO_ANGLES;
      servo[SWITCHING_TOOLHEAD_SERVO_NR].move(swt_angles[locked ? 0 : 1]);
    #elif PIN_EXISTS(SWT_SOLENOID)
      OUT_WRITE(SWT_SOLENOID_PIN, locked);
      gcode.dwell(10);
    #else
      #error "No toolhead locking mechanism configured."
    #endif
  }

  void swt_init() {
    switching_toolhead_lock(true);

    #if ENABLED(TOOL_SENSOR)
      // Init tool sensors
      #if PIN_EXISTS(TOOL_SENSOR1)
        SET_INPUT_PULLUP(TOOL_SENSOR1_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR2)
        SET_INPUT_PULLUP(TOOL_SENSOR2_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR3)
        SET_INPUT_PULLUP(TOOL_SENSOR3_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR4)
        SET_INPUT_PULLUP(TOOL_SENSOR4_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR5)
        SET_INPUT_PULLUP(TOOL_SENSOR5_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR6)
        SET_INPUT_PULLUP(TOOL_SENSOR6_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR7)
        SET_INPUT_PULLUP(TOOL_SENSOR7_PIN);
      #endif
      #if PIN_EXISTS(TOOL_SENSOR8)
        SET_INPUT_PULLUP(TOOL_SENSOR8_PIN);
      #endif

      if (check_tool_sensor_stats(0)) {
        LCD_MESSAGE_F("TC error");
        switching_toolhead_lock(false);
        while (check_tool_sensor_stats(0)) { /* nada */ }
        switching_toolhead_lock(true);
      }
      LCD_MESSAGE_F("TC Success");
    #endif // TOOL_SENSOR
  }

  inline void switching_toolhead_tool_change(const uint8_t new_tool, bool no_move/*=false*/) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS;
    const float placexpos = toolheadposx[active_extruder],
                grabxpos = toolheadposx[new_tool];

    (void)check_tool_sensor_stats(active_extruder, true);

    /**
     * 1. Move to switch position of current toolhead
     * 2. Unlock tool and drop it in the dock
     * 3. Move to the new toolhead
     * 4. Grab and lock the new toolhead
     */

    // 1. Move to switch position of current toolhead

    DEBUG_POS("Start ST Tool-Change", current_position);

    current_position.x = placexpos;

    DEBUG_ECHOLNPGM("(1) Place old tool ", active_extruder);
    DEBUG_POS("Move X SwitchPos", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS - (SWITCHING_TOOLHEAD_Y_SECURITY);

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    slow_line_to_current(Y_AXIS);

    // 2. Unlock tool and drop it in the dock
    TERN_(TOOL_SENSOR, tool_sensor_disabled = true);

    planner.synchronize();
    DEBUG_ECHOLNPGM("(2) Unlock and Place Toolhead");
    switching_toolhead_lock(false);
    safe_delay(500);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;
    DEBUG_POS("Move Y SwitchPos", current_position);
    slow_line_to_current(Y_AXIS);

    // Wait for move to complete, then another 0.2s
    planner.synchronize();
    safe_delay(200);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    slow_line_to_current(Y_AXIS); // move away from docked toolhead

    (void)check_tool_sensor_stats(active_extruder);

    // 3. Move to the new toolhead

    current_position.x = grabxpos;

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(3) Move to new toolhead position");
    DEBUG_POS("Move to new toolhead X", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS - (SWITCHING_TOOLHEAD_Y_SECURITY);

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    slow_line_to_current(Y_AXIS);

    // 4. Grab and lock the new toolhead

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(4) Grab and lock new toolhead");
    DEBUG_POS("Move Y SwitchPos", current_position);

    slow_line_to_current(Y_AXIS);

    // Wait for move to finish, pause 0.2s, move servo, pause 0.5s
    planner.synchronize();
    safe_delay(200);

    (void)check_tool_sensor_stats(new_tool, true, true);

    switching_toolhead_lock(true);
    safe_delay(500);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    slow_line_to_current(Y_AXIS); // Move away from docked toolhead
    planner.synchronize();        // Always sync the final move

    (void)check_tool_sensor_stats(new_tool, true, true);

    DEBUG_POS("ST Tool-Change done.", current_position);
  }

#elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)

  inline void magnetic_switching_toolhead_tool_change(const uint8_t new_tool, bool no_move/*=false*/) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS,
                    toolheadclearx[] = SWITCHING_TOOLHEAD_X_SECURITY;

    const float placexpos = toolheadposx[active_extruder],
                placexclear = toolheadclearx[active_extruder],
                grabxpos = toolheadposx[new_tool],
                grabxclear = toolheadclearx[new_tool];

    /**
     * 1. Move to switch position of current toolhead
     * 2. Release and place toolhead in the dock
     * 3. Move to the new toolhead
     * 4. Grab the new toolhead and move to security position
     */

    DEBUG_POS("Start MST Tool-Change", current_position);

    // 1. Move to switch position current toolhead

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR;

    SERIAL_ECHOLNPGM("(1) Place old tool ", active_extruder);
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    fast_line_to_current(Y_AXIS);

    current_position.x = placexclear;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move X SwitchPos + Security", current_position);

    fast_line_to_current(X_AXIS);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos", current_position);

    fast_line_to_current(Y_AXIS);

    current_position.x = placexpos;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move X SwitchPos", current_position);

    line_to_current_position(planner.settings.max_feedrate_mm_s[X_AXIS] * 0.25f);

    // 2. Release and place toolhead in the dock

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(2) Release and Place Toolhead");

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_RELEASE;
    DEBUG_POS("Move Y SwitchPos + Release", current_position);
    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS] * 0.1f);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_SECURITY;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos + Security", current_position);

    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS]);

    // 3. Move to new toolhead position

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(3) Move to new toolhead position");

    current_position.x = grabxpos;
    DEBUG_POS("Move to new toolhead X", current_position);
    fast_line_to_current(X_AXIS);

    // 4. Grab the new toolhead and move to security position

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(4) Grab new toolhead, move to security position");

    current_position.y = SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_RELEASE;
    DEBUG_POS("Move Y SwitchPos + Release", current_position);
    line_to_current_position(planner.settings.max_feedrate_mm_s[Y_AXIS]);

    current_position.y = SWITCHING_TOOLHEAD_Y_POS;

    DEBUG_SYNCHRONIZE();
    DEBUG_POS("Move Y SwitchPos", current_position);

    _line_to_current(Y_AXIS, 0.2f);

    #if ENABLED(PRIME_BEFORE_REMOVE) && (SWITCHING_TOOLHEAD_PRIME_MM || SWITCHING_TOOLHEAD_RETRACT_MM)
      #if SWITCHING_TOOLHEAD_PRIME_MM
        current_position.e += SWITCHING_TOOLHEAD_PRIME_MM;
        planner.buffer_line(current_position, MMM_TO_MMS(SWITCHING_TOOLHEAD_PRIME_FEEDRATE), new_tool);
      #endif
      #if SWITCHING_TOOLHEAD_RETRACT_MM
        current_position.e -= SWITCHING_TOOLHEAD_RETRACT_MM;
        planner.buffer_line(current_position, MMM_TO_MMS(SWITCHING_TOOLHEAD_RETRACT_FEEDRATE), new_tool);
      #endif
    #else
      planner.synchronize();
      safe_delay(100); // Give switch time to settle
    #endif

    current_position.x = grabxclear;
    DEBUG_POS("Move to new toolhead X + Security", current_position);
    _line_to_current(X_AXIS, 0.1f);
    planner.synchronize();
    safe_delay(100); // Give switch time to settle

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_POS("Move back Y clear", current_position);
    fast_line_to_current(Y_AXIS); // move away from docked toolhead
    planner.synchronize(); // Always sync last tool-change move

    DEBUG_POS("MST Tool-Change done.", current_position);
  }

#elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)

  inline void est_activate_solenoid()   { OUT_WRITE(SOL0_PIN, HIGH); }
  inline void est_deactivate_solenoid() { OUT_WRITE(SOL0_PIN, LOW); }
  void est_init() { est_activate_solenoid(); }

  inline void em_switching_toolhead_tool_change(const uint8_t new_tool, bool no_move) {
    if (no_move) return;

    constexpr float toolheadposx[] = SWITCHING_TOOLHEAD_X_POS;
    const float placexpos = toolheadposx[active_extruder],
                grabxpos = toolheadposx[new_tool];
    const xyz_pos_t &hoffs = hotend_offset[active_extruder];

    /**
     * 1. Raise Z-Axis to give enough clearance
     * 2. Move to position near active extruder parking
     * 3. Move gently to park position of active extruder
     * 4. Disengage magnetic field, wait for delay
     * 5. Leave extruder and move to position near new extruder parking
     * 6. Move gently to park position of new extruder
     * 7. Engage magnetic field for new extruder parking
     * 8. Unpark extruder
     * 9. Apply Z hotend offset to current position
     */

    DEBUG_POS("Start EMST Tool-Change", current_position);

    // 1. Raise Z-Axis to give enough clearance

    current_position.z += SWITCHING_TOOLHEAD_Z_HOP;
    DEBUG_POS("(1) Raise Z-Axis ", current_position);
    fast_line_to_current(Z_AXIS);

    // 2. Move to position near active extruder parking

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(2) Move near active extruder parking", active_extruder);
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.set(hoffs.x + placexpos,
                         hoffs.y + SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR);
    fast_line_to_current(X_AXIS);

    // 3. Move gently to park position of active extruder

    DEBUG_SYNCHRONIZE();
    SERIAL_ECHOLNPGM("(3) Move gently to park position of active extruder", active_extruder);
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    slow_line_to_current(Y_AXIS);

    // 4. Disengage magnetic field, wait for delay

    planner.synchronize();
    DEBUG_ECHOLNPGM("(4) Disengage magnet");
    est_deactivate_solenoid();

    // 5. Leave extruder and move to position near new extruder parking

    DEBUG_ECHOLNPGM("(5) Move near new extruder parking");
    DEBUG_POS("Moving ParkPos", current_position);

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    slow_line_to_current(Y_AXIS);
    current_position.set(hoffs.x + grabxpos,
                         hoffs.y + SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR);
    fast_line_to_current(X_AXIS);

    // 6. Move gently to park position of new extruder

    current_position.y -= SWITCHING_TOOLHEAD_Y_CLEAR;
    if (DEBUGGING(LEVELING)) {
      planner.synchronize();
      DEBUG_ECHOLNPGM("(6) Move near new extruder");
    }
    slow_line_to_current(Y_AXIS);

    // 7. Engage magnetic field for new extruder parking

    DEBUG_SYNCHRONIZE();
    DEBUG_ECHOLNPGM("(7) Engage magnetic field");
    est_activate_solenoid();

    // 8. Unpark extruder

    current_position.y += SWITCHING_TOOLHEAD_Y_CLEAR;
    DEBUG_ECHOLNPGM("(8) Unpark extruder");
    slow_line_to_current(X_AXIS);
    planner.synchronize(); // Always sync the final move

    // 9. Apply Z hotend offset to current position

    DEBUG_POS("(9) Applying Z-offset", current_position);
    current_position.z += hoffs.z - hotend_offset[new_tool].z;

    DEBUG_POS("EMST Tool-Change done.", current_position);
  }

#endif // ELECTROMAGNETIC_SWITCHING_TOOLHEAD

#if HAS_EXTRUDERS
  inline void invalid_extruder_error(const uint8_t e) {
    SERIAL_ECHO_START();
    SERIAL_CHAR('T'); SERIAL_ECHO(e);
    SERIAL_CHAR(' '); SERIAL_ECHOLNPGM(STR_INVALID_EXTRUDER);
  }
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * @brief Dual X Tool Change
   * @details Change tools, with extra behavior based on current mode
   *
   * @param new_tool Tool index to activate
   * @param no_move Flag indicating no moves should take place
   */
  inline void dualx_tool_change(const uint8_t new_tool, bool &no_move) {

    DEBUG_ECHOPGM("Dual X Carriage Mode ");
    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE: DEBUG_ECHOLNPGM("FULL_CONTROL"); break;
      case DXC_AUTO_PARK_MODE:    DEBUG_ECHOLNPGM("AUTO_PARK");    break;
      case DXC_DUPLICATION_MODE:  DEBUG_ECHOLNPGM("DUPLICATION");  break;
      case DXC_MIRRORED_MODE:     DEBUG_ECHOLNPGM("MIRRORED");     break;
    }

    // Get the home position of the currently-active tool
    const float xhome = x_home_pos(active_extruder);

    if (dual_x_carriage_mode == DXC_AUTO_PARK_MODE                  // If Auto-Park mode is enabled
        && IsRunning() && !no_move                                  // ...and movement is permitted
        && (delayed_move_time || current_position.x != xhome)       // ...and delayed_move_time is set OR not "already parked"...
    ) {
      DEBUG_ECHOLNPGM("MoveX to ", xhome);
      current_position.x = xhome;
      line_to_current_position(planner.settings.max_feedrate_mm_s[X_AXIS]);   // Park the current head
      planner.synchronize();
    }

    // Activate the new extruder ahead of calling set_axis_is_at_home!
    active_extruder = new_tool;

    // This function resets the max/min values - the current position may be overwritten below.
    set_axis_is_at_home(X_AXIS);

    DEBUG_POS("New Extruder", current_position);

    switch (dual_x_carriage_mode) {
      case DXC_FULL_CONTROL_MODE:
        // New current position is the position of the activated extruder
        current_position.x = inactive_extruder_x;
        // Save the inactive extruder's position (from the old current_position)
        inactive_extruder_x = destination.x;
        DEBUG_ECHOLNPGM("DXC Full Control curr.x=", current_position.x, " dest.x=", destination.x);
        break;
      case DXC_AUTO_PARK_MODE:
        idex_set_parked();
        break;
      default:
        break;
    }

    // Ensure X axis DIR pertains to the correct carriage
    stepper.apply_directions();

    DEBUG_ECHOLNPGM("Active extruder parked: ", active_extruder_parked ? "yes" : "no");
    DEBUG_POS("New extruder (parked)", current_position);
  }

#endif // DUAL_X_CARRIAGE

/**
 * Prime active tool using TOOLCHANGE_FILAMENT_SWAP settings
 */
#if ENABLED(TOOLCHANGE_FILAMENT_SWAP)

  #define DEBUG_OUT ENABLED(DEBUG_TOOLCHANGE_FILAMENT_SWAP)
  #include "../core/debug_out.h"

  // Define any variables required
  static Flags<EXTRUDERS> extruder_was_primed; // Extruders primed status

  #if ENABLED(TOOLCHANGE_FS_PRIME_FIRST_USED)
    bool enable_first_prime; // As set by M217 V
  #endif

  // Cool down with fan
  inline void filament_swap_cooling() {
    #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
      thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = toolchange_settings.fan_speed;
      gcode.dwell(SEC_TO_MS(toolchange_settings.fan_time));
      thermalManager.fan_speed[TOOLCHANGE_FS_FAN] = FAN_OFF_PWM;
    #endif
  }

  /**
   * Check if too cold to move the specified tool
   *
   * Returns TRUE if too cold to move (also echos message: STR_ERR_HOTEND_TOO_COLD)
   * Returns FALSE if able to  move.
   */
  bool too_cold(uint8_t toolID){
    if (!DEBUGGING(DRYRUN) && thermalManager.targetTooColdToExtrude(toolID)) {
      SERIAL_ECHO_MSG(STR_ERR_HOTEND_TOO_COLD);
      return true;
    }
    return false;
  }

  /**
   * Cutting recovery -- Recover from cutting retraction that occurs at the end of nozzle priming
   *
   * If the active_extruder is up to temp (!too_cold):
   *  Extrude filament distance = toolchange_settings.extra_resume + toolchange_settings.wipe_retract
   *  current_position.e = e;
   *  sync_plan_position_e();
   */
  void extruder_cutting_recover(const_float_t e) {
    if (too_cold(active_extruder)) return;
    const float dist = toolchange_settings.extra_resume + toolchange_settings.wipe_retract;
    DEBUG_ECHOLNPGM("Performing Cutting Recover | Distance: ", dist, " | Speed: ", MMM_TO_MMS(toolchange_settings.unretract_speed), "mm/s");
    unscaled_e_move(dist, MMM_TO_MMS(toolchange_settings.unretract_speed));

    DEBUG_ECHOLNPGM("Set E position: ", e);
    current_position.e = e;
    sync_plan_position_e(); // Resume new E Position
  }

  /**
   * Prime the currently selected extruder (Filament loading only)
   * Leave the E position unchanged so subsequent extrusion works properly.
   *
   * If too_cold(toolID) returns TRUE -> returns without moving extruder.
   * Extruders filament = swap_length + extra prime, then performs cutting retraction if enabled.
   * If cooling fan is enabled, calls filament_swap_cooling();
   */
  void extruder_prime() {
    if (too_cold(active_extruder)) {
      DEBUG_ECHOLNPGM("Priming Aborted -  Nozzle Too Cold!");
      return; // Extruder too cold to prime
    }

    feedRate_t fr_mm_s = MMM_TO_MMS(toolchange_settings.unretract_speed); // Set default speed for unretract

    const float resume_current_e = current_position.e;

    #if ENABLED(TOOLCHANGE_FS_SLOW_FIRST_PRIME)
      /**
       * Perform first unretract movement at the slower Prime_Speed to avoid breakage on first prime
       */
      static Flags<EXTRUDERS> extruder_did_first_prime;  // Extruders first priming status
      if (!extruder_did_first_prime[active_extruder]) {
        extruder_did_first_prime.set(active_extruder);   // Log first prime complete
        // new nozzle - prime at user-specified speed.
        const feedRate_t prime_mm_s = MMM_TO_MMS(toolchange_settings.prime_speed);
        DEBUG_ECHOLNPGM("First time priming T", active_extruder, ", reducing speed from ", fr_mm_s, " to ",  prime_mm_s, "mm/s");
        fr_mm_s = prime_mm_s;
        unscaled_e_move(0, fr_mm_s);      // Init planner with 0 length move
      }
    #endif

    // Calculate and perform the priming distance
    if (toolchange_settings.extra_prime >= 0) {
      // Positive extra_prime value
      // - Return filament at speed (fr_mm_s) then extra_prime at prime speed
      DEBUG_ECHOLNPGM("Loading Filament for T", active_extruder, " | Distance: ", toolchange_settings.swap_length, " | Speed: ", fr_mm_s, "mm/s");
      unscaled_e_move(toolchange_settings.swap_length, fr_mm_s); // Prime (Unretract) filament by extruding equal to Swap Length (Unretract)

      if (toolchange_settings.extra_prime > 0) {
        DEBUG_ECHOLNPGM("Performing Extra Priming for T", active_extruder, " | Distance: ", toolchange_settings.extra_prime, " | Speed: ", MMM_TO_MMS(toolchange_settings.prime_speed), "mm/s");
        unscaled_e_move(toolchange_settings.extra_prime, MMM_TO_MMS(toolchange_settings.prime_speed)); // Extra Prime Distance
      }
    }
    else {
      // Negative extra_prime value
      // - Unretract distance (swap length) is reduced by the value of extra_prime
      const float eswap = toolchange_settings.swap_length + toolchange_settings.extra_prime;
      DEBUG_ECHOLNPGM("Negative ExtraPrime value - Swap Return Length has been reduced from ", toolchange_settings.swap_length, " to ", eswap);
      DEBUG_ECHOLNPGM("Loading Filament for T", active_extruder, " | Distance: ", eswap, " | Speed: ", fr_mm_s, "mm/s");
      unscaled_e_move(eswap, fr_mm_s);
    }

    extruder_was_primed.set(active_extruder); // Log that this extruder has been primed

    // Cutting retraction
    #if TOOLCHANGE_FS_WIPE_RETRACT
      DEBUG_ECHOLNPGM("Performing Cutting Retraction | Distance: ", -toolchange_settings.wipe_retract, " | Speed: ", MMM_TO_MMS(toolchange_settings.retract_speed), "mm/s");
      unscaled_e_move(-toolchange_settings.wipe_retract, MMM_TO_MMS(toolchange_settings.retract_speed));
    #endif

    // Leave E unchanged when priming
    current_position.e = resume_current_e;
    sync_plan_position_e();

    // Cool down with fan
    filament_swap_cooling();
  }

  /**
   * Sequence to Prime the currently selected extruder
   * Raise Z, move the ToolChange_Park if enabled, prime the extruder, move back.
   */
  void tool_change_prime() {

    DEBUG_SECTION(tcp, "tool_change_prime", true);

    if (!too_cold(active_extruder)) {
      destination = current_position; // Remember the old position

      const bool ok = TERN0(TOOLCHANGE_PARK, all_axes_homed() && toolchange_settings.enable_park);

      #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
        // Store and stop fan. Restored on any exit.
        REMEMBER(fan, thermalManager.fan_speed[TOOLCHANGE_FS_FAN], 0);
      #endif

      // Z raise
      if (ok) {
        // Do a small lift to avoid the workpiece in the move back (below)
        current_position.z += toolchange_settings.z_raise;
        TERN_(HAS_SOFTWARE_ENDSTOPS, NOMORE(current_position.z, soft_endstop.max.z));
        fast_line_to_current(Z_AXIS);
        planner.synchronize();
      }

      // Park
      #if ENABLED(TOOLCHANGE_PARK)
        if (ok) {
          IF_DISABLED(TOOLCHANGE_PARK_Y_ONLY, current_position.x = toolchange_settings.change_point.x);
          IF_DISABLED(TOOLCHANGE_PARK_X_ONLY, current_position.y = toolchange_settings.change_point.y);
          #if NONE(TOOLCHANGE_PARK_X_ONLY, TOOLCHANGE_PARK_Y_ONLY)
            SECONDARY_AXIS_CODE(
              current_position.i = toolchange_settings.change_point.i,
              current_position.j = toolchange_settings.change_point.j,
              current_position.k = toolchange_settings.change_point.k,
              current_position.u = toolchange_settings.change_point.u,
              current_position.v = toolchange_settings.change_point.v,
              current_position.w = toolchange_settings.change_point.w
            );
          #endif
          planner.buffer_line(current_position, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE), active_extruder);
          planner.synchronize();
        }
      #endif

      // Prime without changing E
      extruder_prime();

      // Move back
      #if ENABLED(TOOLCHANGE_PARK)
        if (ok) {
          #if ENABLED(TOOLCHANGE_NO_RETURN)
            destination.x = current_position.x;
            destination.y = current_position.y;
          #endif
          do_blocking_move_to_xy(destination, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE));
          do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
          planner.synchronize();
        }
      #endif

      // Clone previous position
      extruder_cutting_recover(destination.e); // Cutting recover

      // Retract if previously retracted
      #if ENABLED(FWRETRACT)
        if (fwretract.retracted[active_extruder])
          unscaled_e_move(-fwretract.settings.retract_length, fwretract.settings.retract_feedrate_mm_s);
      #endif

      // If resume_position is negative
      if (current_position.e < 0) unscaled_e_move(current_position.e, MMM_TO_MMS(toolchange_settings.retract_speed));

      planner.synchronize();
      planner.set_e_position_mm(current_position.e); // Extruder primed and ready
    }
  }

#endif // TOOLCHANGE_FILAMENT_SWAP

/**
 * Perform a tool-change, which may result in moving the
 * previous tool out of the way and the new tool into place.
 */
void tool_change(const uint8_t new_tool, bool no_move/*=false*/) {

  if (TERN0(MAGNETIC_SWITCHING_TOOLHEAD, new_tool == active_extruder))
    return;

  #if ENABLED(MIXING_EXTRUDER)

    UNUSED(no_move);

    if (new_tool >= MIXING_VIRTUAL_TOOLS)
      return invalid_extruder_error(new_tool);

    #if MIXING_VIRTUAL_TOOLS > 1
      // T0-Tnnn: Switch virtual tool by changing the index to the mix
      mixer.T(new_tool);
    #endif

  #elif HAS_PRUSA_MMU3

    UNUSED(no_move);

    mmu3.tool_change(new_tool);

  #elif HAS_PRUSA_MMU2

    UNUSED(no_move);

    mmu2.tool_change(new_tool);

  #elif EXTRUDERS == 0

    // Nothing to do
    UNUSED(new_tool); UNUSED(no_move);

  #elif EXTRUDERS < 2

    UNUSED(no_move);

    if (new_tool) invalid_extruder_error(new_tool);
    return;

  #elif HAS_MULTI_EXTRUDER

    planner.synchronize();

    #if ENABLED(DUAL_X_CARRIAGE)  // Only T0 allowed if the Printer is in DXC_DUPLICATION_MODE or DXC_MIRRORED_MODE
      if (new_tool != 0 && idex_is_duplicating())
         return invalid_extruder_error(new_tool);
    #endif

    if (new_tool >= EXTRUDERS)
      return invalid_extruder_error(new_tool);

    if (!no_move && homing_needed()) {
      no_move = true;
      DEBUG_ECHOLNPGM("No move (not homed)");
    }

    TERN_(HAS_MARLINUI_MENU, if (!no_move) ui.update());

    #if ENABLED(DUAL_X_CARRIAGE)
      const bool idex_full_control = dual_x_carriage_mode == DXC_FULL_CONTROL_MODE;
    #else
      constexpr bool idex_full_control = false;
    #endif

    const uint8_t old_tool = active_extruder;
    const bool can_move_away = !no_move && !idex_full_control;

    #if ENABLED(AUTO_BED_LEVELING_UBL)
      // Workaround for UBL mesh boundary, possibly?
      TEMPORARY_BED_LEVELING_STATE(false);
    #endif

    // First tool priming. To prime again, reboot the machine. -- Should only occur for first T0 after powerup!
    #if ENABLED(TOOLCHANGE_FS_PRIME_FIRST_USED)
      if (enable_first_prime && old_tool == 0 && new_tool == 0 && !extruder_was_primed[0]) {
        tool_change_prime();
        TERN_(TOOLCHANGE_FS_INIT_BEFORE_SWAP, toolchange_extruder_ready.set(old_tool)); // Primed and initialized
      }
    #endif

    if (new_tool != old_tool || TERN0(PARKING_EXTRUDER, extruder_parked)) { // PARKING_EXTRUDER may need to attach old_tool when homing
      destination = current_position;

      #if ALL(TOOLCHANGE_FILAMENT_SWAP, HAS_FAN) && TOOLCHANGE_FS_FAN >= 0
        // Store and stop fan. Restored on any exit.
        REMEMBER(fan, thermalManager.fan_speed[TOOLCHANGE_FS_FAN], 0);
      #endif

      // Z raise before retraction
      #if ENABLED(TOOLCHANGE_ZRAISE_BEFORE_RETRACT) && !HAS_SWITCHING_NOZZLE
        if (can_move_away && TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park)) {
          // Do a small lift to avoid the workpiece in the move back (below)
          current_position.z += toolchange_settings.z_raise;
          TERN_(HAS_SOFTWARE_ENDSTOPS, NOMORE(current_position.z, soft_endstop.max.z));
          fast_line_to_current(Z_AXIS);
          planner.synchronize();
        }
      #endif

      // Unload / Retract
      #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
        const bool should_swap = can_move_away && toolchange_settings.swap_length;
        if (should_swap) {
          if (too_cold(old_tool)) {
            // If SingleNozzle setup is too cold, unable to perform tool_change.
            if (ENABLED(SINGLENOZZLE)) { active_extruder = new_tool; return; }
          }
          else if (extruder_was_primed[old_tool]) {
            // Retract the old extruder if it was previously primed
            // To-Do: Should SingleNozzle always retract?
            DEBUG_ECHOLNPGM("Retracting Filament for T", old_tool, ". | Distance: ", toolchange_settings.swap_length, " | Speed: ", MMM_TO_MMS(toolchange_settings.retract_speed), "mm/s");
            unscaled_e_move(-toolchange_settings.swap_length, MMM_TO_MMS(toolchange_settings.retract_speed));
          }
        }
      #endif

      REMEMBER(fr, feedrate_mm_s, XY_PROBE_FEEDRATE_MM_S);

      #if HAS_SOFTWARE_ENDSTOPS
        #if HAS_HOTEND_OFFSET
          #define _EXT_ARGS , old_tool, new_tool
        #else
          #define _EXT_ARGS
        #endif
        update_software_endstops(X_AXIS _EXT_ARGS);
        #if DISABLED(DUAL_X_CARRIAGE)
          update_software_endstops(Y_AXIS _EXT_ARGS);
          update_software_endstops(Z_AXIS _EXT_ARGS);
        #endif
      #endif

      #if NONE(TOOLCHANGE_ZRAISE_BEFORE_RETRACT, HAS_SWITCHING_NOZZLE)
        if (can_move_away && TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park)) {
          // Do a small lift to avoid the workpiece in the move back (below)
          current_position.z += toolchange_settings.z_raise;
          TERN_(HAS_SOFTWARE_ENDSTOPS, NOMORE(current_position.z, soft_endstop.max.z));
          fast_line_to_current(Z_AXIS);
        }
      #endif

      // Toolchange park
      #if ENABLED(TOOLCHANGE_PARK) && !HAS_SWITCHING_NOZZLE
        if (can_move_away && toolchange_settings.enable_park) {
          IF_DISABLED(TOOLCHANGE_PARK_Y_ONLY, current_position.x = toolchange_settings.change_point.x);
          IF_DISABLED(TOOLCHANGE_PARK_X_ONLY, current_position.y = toolchange_settings.change_point.y);
          #if NONE(TOOLCHANGE_PARK_X_ONLY, TOOLCHANGE_PARK_Y_ONLY)
            SECONDARY_AXIS_CODE(
              current_position.i = toolchange_settings.change_point.i,
              current_position.j = toolchange_settings.change_point.j,
              current_position.k = toolchange_settings.change_point.k,
              current_position.u = toolchange_settings.change_point.u,
              current_position.v = toolchange_settings.change_point.v,
              current_position.w = toolchange_settings.change_point.w
            );
          #endif
          planner.buffer_line(current_position, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE), old_tool);
          planner.synchronize();
        }
      #endif

      #if HAS_HOTEND_OFFSET
        xyz_pos_t diff = hotend_offset[new_tool] - hotend_offset[old_tool];
        TERN_(DUAL_X_CARRIAGE, diff.x = 0);
      #else
        constexpr xyz_pos_t diff{0};
      #endif

      #if ENABLED(DUAL_X_CARRIAGE)
        dualx_tool_change(new_tool, no_move);
      #elif ENABLED(PARKING_EXTRUDER)                                   // Dual Parking extruder
        parking_extruder_tool_change(new_tool, no_move);
      #elif ENABLED(MAGNETIC_PARKING_EXTRUDER)                          // Magnetic Parking extruder
        magnetic_parking_extruder_tool_change(new_tool);
      #elif ENABLED(SWITCHING_TOOLHEAD)                                 // Switching Toolhead
        switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)                        // Magnetic Switching Toolhead
        magnetic_switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(ELECTROMAGNETIC_SWITCHING_TOOLHEAD)                 // Magnetic Switching ToolChanger
        em_switching_toolhead_tool_change(new_tool, no_move);
      #elif ENABLED(SWITCHING_NOZZLE)                                   // Switching Nozzle
        // Raise by a configured distance to avoid workpiece, except with
        // SWITCHING_NOZZLE_TWO_SERVOS, as both nozzles will lift instead.
        TERN_(SWITCHING_NOZZLE_TWO_SERVOS, raise_nozzle(old_tool));
        if (!no_move) {
          const float newz = current_position.z + _MAX(-diff.z, 0.0);
          // Check if Z has space to compensate at least z_offset, and if not, just abort now
          const float maxz = _MIN(TERN(HAS_SOFTWARE_ENDSTOPS, soft_endstop.max.z, Z_MAX_POS), Z_MAX_POS);
          if (newz > maxz) return;
          current_position.z = _MIN(newz + toolchange_settings.z_raise, maxz);
          fast_line_to_current(Z_AXIS);
        }
        #if SWITCHING_NOZZLE_TWO_SERVOS                                 // Switching Nozzle with two servos
          lower_nozzle(new_tool);
        #else
          move_nozzle_servo(new_tool);
        #endif
      #elif ANY(MECHANICAL_SWITCHING_EXTRUDER, MECHANICAL_SWITCHING_NOZZLE)
        if (!no_move) {
          current_position.z = _MIN(current_position.z + toolchange_settings.z_raise, _MIN(TERN(HAS_SOFTWARE_ENDSTOPS, soft_endstop.max.z, Z_MAX_POS), Z_MAX_POS));
          fast_line_to_current(Z_AXIS);
        }
      #endif

      IF_DISABLED(DUAL_X_CARRIAGE, active_extruder = new_tool); // Set the new active extruder

      TERN_(TOOL_SENSOR, tool_sensor_disabled = false);

      (void)check_tool_sensor_stats(active_extruder, true);

      // The newly-selected extruder XYZ is actually at...
      DEBUG_ECHOLNPGM("Offset Tool XYZ by { ", diff.x, ", ", diff.y, ", ", diff.z, " }");
      current_position += diff;

      // Tell the planner the new "current position"
      sync_plan_position();

      #if ENABLED(DELTA)
        //LOOP_NUM_AXES(i) update_software_endstops(i); // or modify the constrain function
        const bool safe_to_move = current_position.z < delta_clip_start_height - 1;
      #else
        constexpr bool safe_to_move = true;
      #endif

      // Return to position and lower again
      const bool should_move = safe_to_move && !no_move && IsRunning();
      if (should_move) {

        #if ANY(SINGLENOZZLE_STANDBY_TEMP, SINGLENOZZLE_STANDBY_FAN)
          thermalManager.singlenozzle_change(old_tool, new_tool);
        #endif

        #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
          if (should_swap && !too_cold(active_extruder))
            extruder_prime(); // Prime selected Extruder
        #endif

        // Prevent a move outside physical bounds
        #if ENABLED(MAGNETIC_SWITCHING_TOOLHEAD)
          // If the original position is within tool store area, go to X origin at once
          if (destination.y < SWITCHING_TOOLHEAD_Y_POS + SWITCHING_TOOLHEAD_Y_CLEAR) {
            current_position.x = X_MIN_POS;
            planner.buffer_line(current_position, planner.settings.max_feedrate_mm_s[X_AXIS], new_tool);
            planner.synchronize();
          }
        #else
          apply_motion_limits(destination);
        #endif

        // Should the nozzle move back to the old position?
        if (can_move_away) {
          #if ENABLED(TOOLCHANGE_NO_RETURN)
            // Just move back down
            DEBUG_ECHOLNPGM("Move back Z only");

            if (TERN1(TOOLCHANGE_PARK, toolchange_settings.enable_park))
              do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);

          #else
            // Move back to the original (or adjusted) position
            DEBUG_POS("Move back", destination);

            #if ENABLED(TOOLCHANGE_PARK)
              if (toolchange_settings.enable_park) do_blocking_move_to_xy_z(destination, destination.z, MMM_TO_MMS(TOOLCHANGE_PARK_XY_FEEDRATE));
            #else
              do_blocking_move_to_xy(destination, planner.settings.max_feedrate_mm_s[X_AXIS]* 0.5f);

              // If using MECHANICAL_SWITCHING extruder/nozzle, set HOTEND_OFFSET in Z axis after running EVENT_GCODE_TOOLCHANGE below.
              #if NONE(MECHANICAL_SWITCHING_EXTRUDER, MECHANICAL_SWITCHING_NOZZLE)
                do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
                SECONDARY_AXIS_CODE(
                  do_blocking_move_to_i(destination.i, planner.settings.max_feedrate_mm_s[I_AXIS]),
                  do_blocking_move_to_j(destination.j, planner.settings.max_feedrate_mm_s[J_AXIS]),
                  do_blocking_move_to_k(destination.k, planner.settings.max_feedrate_mm_s[K_AXIS]),
                  do_blocking_move_to_u(destination.u, planner.settings.max_feedrate_mm_s[U_AXIS]),
                  do_blocking_move_to_v(destination.v, planner.settings.max_feedrate_mm_s[V_AXIS]),
                  do_blocking_move_to_w(destination.w, planner.settings.max_feedrate_mm_s[W_AXIS])
                );
              #endif
            #endif

          #endif
        }

        else DEBUG_ECHOLNPGM("Move back skipped");

        #if ENABLED(TOOLCHANGE_FILAMENT_SWAP)
          if (should_swap && !too_cold(active_extruder)) {
            extruder_cutting_recover(0); // New extruder primed and set to 0

            // Restart Fan
            #if HAS_FAN && TOOLCHANGE_FS_FAN >= 0
              RESTORE(fan);
            #endif
          }
        #endif

        TERN_(DUAL_X_CARRIAGE, idex_set_parked(false));
      } // should_move

      #if HAS_SWITCHING_NOZZLE
        // Move back down. (Including when the new tool is higher.)
        if (!should_move)
          do_blocking_move_to_z(destination.z, planner.settings.max_feedrate_mm_s[Z_AXIS]);
      #endif

    } // (new_tool != old_tool)

    planner.synchronize();

    #if ENABLED(EXT_SOLENOID) && DISABLED(PARKING_EXTRUDER)
      disable_all_solenoids();
      enable_solenoid(active_extruder);
    #endif

    #if HAS_PRUSA_MMU1
      if (new_tool >= E_STEPPERS) return invalid_extruder_error(new_tool);
      select_multiplexed_stepper(new_tool);
    #endif

    #if DO_SWITCH_EXTRUDER
      planner.synchronize();
      move_extruder_servo(active_extruder);
    #endif

    TERN_(HAS_FANMUX, fanmux_switch(active_extruder));

    if (ENABLED(EVENT_GCODE_TOOLCHANGE_ALWAYS_RUN) || !no_move) {

      #if ANY(TC_GCODE_USE_GLOBAL_X, TC_GCODE_USE_GLOBAL_Y, TC_GCODE_USE_GLOBAL_Z)
        // G0/G1/G2/G3/G5 moves are relative to the active tool.
        // Shift the workspace to make custom moves relative to T0.
        xyz_pos_t old_workspace_offset;
        if (new_tool > 0) {
          old_workspace_offset = workspace_offset;
          const xyz_pos_t &he = hotend_offset[new_tool];
          TERN_(TC_GCODE_USE_GLOBAL_X, workspace_offset.x -= he.x);
          TERN_(TC_GCODE_USE_GLOBAL_Y, workspace_offset.y -= he.y);
          TERN_(TC_GCODE_USE_GLOBAL_Z, workspace_offset.z -= he.z);
        }
      #endif

      switch (new_tool) {
        default: break;
        #ifdef EVENT_GCODE_TOOLCHANGE_T0
          case 0: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T0)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T1
          case 1: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T1)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T2
          case 2: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T2)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T3
          case 3: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T3)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T4
          case 4: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T4)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T5
          case 5: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T5)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T6
          case 6: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T6)); break;
        #endif
        #ifdef EVENT_GCODE_TOOLCHANGE_T7
          case 7: gcode.process_subcommands_now(F(EVENT_GCODE_TOOLCHANGE_T7)); break;
        #endif
      }

      #if ANY(TC_GCODE_USE_GLOBAL_X, TC_GCODE_USE_GLOBAL_Y, TC_GCODE_USE_GLOBAL_Z)
        if (new_tool > 0) workspace_offset = old_workspace_offset;
      #endif

      // If using MECHANICAL_SWITCHING extruder/nozzle, set HOTEND_OFFSET in Z axis after running EVENT_GCODE_TOOLCHANGE
      // so that nozzle does not lower below print surface if new hotend Z offset is higher than old hotend Z offset.
      #if ANY(MECHANICAL_SWITCHING_EXTRUDER, MECHANICAL_SWITCHING_NOZZLE)
        #if HAS_HOTEND_OFFSET
          xyz_pos_t diff = hotend_offset[new_tool] - hotend_offset[old_tool];
          TERN_(DUAL_X_CARRIAGE, diff.x = 0);
        #else
          constexpr xyz_pos_t diff{0};
        #endif

        if (!no_move) {
          // Move to new hotend Z offset and reverse Z_RAISE
          do_blocking_move_to_z(
            _MIN(
              _MAX((destination.z - diff.z) - toolchange_settings.z_raise,
              _MAX(TERN(HAS_SOFTWARE_ENDSTOPS, soft_endstop.min.z, Z_MIN_POS), Z_MIN_POS)
            ),
            _MIN(TERN(HAS_SOFTWARE_ENDSTOPS, soft_endstop.max.z, Z_MAX_POS), Z_MAX_POS)),
            planner.settings.max_feedrate_mm_s[Z_AXIS]
          );
        }
      #endif

      #ifdef EVENT_GCODE_AFTER_TOOLCHANGE
        if (TERN1(DUAL_X_CARRIAGE, dual_x_carriage_mode == DXC_AUTO_PARK_MODE))
          gcode.process_subcommands_now(F(EVENT_GCODE_AFTER_TOOLCHANGE));
      #endif

    } // !no_move

    SERIAL_ECHOLNPGM(STR_ACTIVE_EXTRUDER, active_extruder);

  #endif // HAS_MULTI_EXTRUDER
}

#if ENABLED(TOOLCHANGE_MIGRATION_FEATURE)

  #define DEBUG_OUT ENABLED(DEBUG_TOOLCHANGE_MIGRATION_FEATURE)
  #include "../core/debug_out.h"

  bool extruder_migration() {

    if (thermalManager.targetTooColdToExtrude(active_extruder)) {
      DEBUG_ECHOLNPGM("Migration Source Too Cold");
      return false;
    }

    // No auto-migration or specified target?
    if (!migration.target && active_extruder >= migration.last) {
      DEBUG_ECHO_MSG("No Migration Target");
      DEBUG_ECHO_MSG("Target: ", migration.target, " Last: ", migration.last, " Active: ", active_extruder);
      migration.automode = false;
      return false;
    }

    // Migrate to a target or the next extruder

    uint8_t migration_extruder = active_extruder;

    if (migration.target) {
      DEBUG_ECHOLNPGM("Migration using fixed target");
      // Specified target ok?
      const int16_t t = migration.target - 1;
      if (t != active_extruder) migration_extruder = t;
    }
    else if (migration.automode && migration_extruder < migration.last && migration_extruder < EXTRUDERS - 1)
      migration_extruder++;

    if (migration_extruder == active_extruder) {
      DEBUG_ECHOLNPGM("Migration source matches active");
      return false;
    }

    // Migration begins
    DEBUG_ECHOLNPGM("Beginning migration");

    migration.in_progress = true; // Prevent runout script
    planner.synchronize();

    // Remember position before migration
    const float resume_current_e = current_position.e;

    // Migrate the flow
    planner.set_flow(migration_extruder, planner.flow_percentage[active_extruder]);

    // Migrate the retracted state
    #if ENABLED(FWRETRACT)
      fwretract.retracted.set(migration_extruder, fwretract.retracted[active_extruder]);
    #endif

    // Migrate the temperature to the new hotend
    #if HAS_MULTI_HOTEND
      thermalManager.setTargetHotend(thermalManager.degTargetHotend(active_extruder), migration_extruder);
      TERN_(AUTOTEMP, planner.autotemp_update());
      thermalManager.set_heating_message(0);
      thermalManager.wait_for_hotend(active_extruder);
    #endif

    // Migrate Linear Advance K factor to the new extruder
    TERN_(LIN_ADVANCE, planner.extruder_advance_K[active_extruder] = planner.extruder_advance_K[migration_extruder]);

    // Temporary migration toolchange_settings restored on exit. i.e., before next tool_change().
    #if defined(MIGRATION_FS_EXTRA_PRIME) \
     || defined(MIGRATION_FS_WIPE_RETRACT) \
     || defined(MIGRATION_FS_FAN_SPEED) \
     || defined(MIGRATION_FS_FAN_TIME) \
     || defined(MIGRATION_ZRAISE) \
     || defined(TOOLCHANGE_MIGRATION_DO_PARK)
      REMEMBER(tmp_mig_settings, toolchange_settings);
      #ifdef MIGRATION_FS_EXTRA_PRIME
        toolchange_settings.extra_prime = MIGRATION_FS_EXTRA_PRIME;
      #endif
      #ifdef MIGRATION_FS_WIPE_RETRACT
        toolchange_settings.wipe_retract = MIGRATION_FS_WIPE_RETRACT;
      #endif
      #ifdef MIGRATION_FS_FAN_SPEED
        toolchange_settings.fan_speed = MIGRATION_FS_FAN_SPEED;
      #endif
      #ifdef MIGRATION_FS_FAN_TIME
        toolchange_settings.fan_time = MIGRATION_FS_FAN_TIME;
      #endif
      #ifdef MIGRATION_ZRAISE
        toolchange_settings.z_raise = MIGRATION_ZRAISE;
      #endif
      #ifdef TOOLCHANGE_MIGRATION_DO_PARK
        toolchange_settings.enable_park = ENABLED(TOOLCHANGE_MIGRATION_DO_PARK);
      #endif
    #endif

    // Perform the tool change
    tool_change(migration_extruder);

    // Retract if previously retracted
    #if ENABLED(FWRETRACT)
      if (fwretract.retracted[active_extruder])
        unscaled_e_move(-fwretract.settings.retract_length, fwretract.settings.retract_feedrate_mm_s);
    #endif

    // If resume_position is negative
    if (resume_current_e < 0) unscaled_e_move(resume_current_e, MMM_TO_MMS(toolchange_settings.retract_speed));

    // If no available extruder
    if (EXTRUDERS < 2 || active_extruder >= EXTRUDERS - 2 || active_extruder == migration.last)
      migration.automode = false;

    migration.in_progress = false;

    current_position.e = resume_current_e;

    planner.synchronize();
    planner.set_e_position_mm(current_position.e); // New extruder primed and ready

    DEBUG_ECHOLNPGM("Migration Complete");
    return true;
  }

#endif // TOOLCHANGE_MIGRATION_FEATURE

#endif // HAS_TOOLCHANGE
