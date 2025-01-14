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
 * motion.cpp
 */

#include "motion.h"
#include "endstops.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "../gcode/gcode.h"
#include "../lcd/marlinui.h"
#include "../inc/MarlinConfig.h"

#if IS_SCARA
  #include "../libs/buzzer.h"
  #include "../lcd/marlinui.h"
#endif

#if ENABLED(POLAR)
  #include "polar.h"
#endif

#if HAS_BED_PROBE
  #include "probe.h"
#endif

#if HAS_LEVELING
  #include "../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../feature/bltouch.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../feature/runout.h"
#endif

#if ENABLED(SENSORLESS_HOMING)
  #include "../feature/tmc_util.h"
#endif

#if ENABLED(FWRETRACT)
  #include "../feature/fwretract.h"
#endif

#if ENABLED(BABYSTEP_DISPLAY_TOTAL)
  #include "../feature/babystep.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../core/debug_out.h"

#if ENABLED(BD_SENSOR)
  #include "../feature/bedlevel/bdl/bdl.h"
#endif

// Relative Mode. Enable with G91, disable with G90.
bool relative_mode; // = false

#if HAS_Z_AXIS
  // If Z has been powered on trust that the real Z is >= current_position.z
  bool z_min_trusted; // = false
#endif

/**
 * Cartesian Current Position
 *   Used to track the native machine position as moves are queued.
 *   Used by 'line_to_current_position' to do a move after changing it.
 *   Used by 'sync_plan_position' to update 'planner.position'.
 */
#ifdef Z_IDLE_HEIGHT
  #define Z_INIT_POS Z_IDLE_HEIGHT
#else
  #define Z_INIT_POS Z_HOME_POS
#endif

xyze_pos_t current_position = LOGICAL_AXIS_ARRAY(0, X_HOME_POS, Y_HOME_POS, Z_INIT_POS, I_HOME_POS, J_HOME_POS, K_HOME_POS, U_HOME_POS, V_HOME_POS, W_HOME_POS);

/**
 * Cartesian Destination
 *   The destination for a move, filled in by G-code movement commands,
 *   and expected by functions like 'prepare_line_to_destination'.
 *   G-codes can set destination using 'get_destination_from_command'
 */
xyze_pos_t destination; // {0}

// G60/G61 Position Save and Return
#if SAVED_POSITIONS
  Flags<SAVED_POSITIONS> did_save_position;
  xyze_pos_t stored_position[SAVED_POSITIONS];
#endif

// The active extruder (tool). Set with T<extruder> command.
#if HAS_MULTI_EXTRUDER
  uint8_t active_extruder = 0; // = 0
#endif

#if ENABLED(LCD_SHOW_E_TOTAL)
  float e_move_accumulator; // = 0
#endif

// Extruder offsets
#if HAS_HOTEND_OFFSET
  xyz_pos_t hotend_offset[HOTENDS]; // Initialized by settings.load
  void reset_hotend_offsets() {
    constexpr float tmp[XYZ][HOTENDS] = { HOTEND_OFFSET_X, HOTEND_OFFSET_Y, HOTEND_OFFSET_Z };
    static_assert(
      !tmp[X_AXIS][0] && !tmp[Y_AXIS][0] && !tmp[Z_AXIS][0],
      "Offsets for the first hotend must be 0.0."
    );
    // Transpose from [XYZ][HOTENDS] to [HOTENDS][XYZ]
    HOTEND_LOOP() LOOP_ABC(a) hotend_offset[e][a] = tmp[a][e];
    TERN_(DUAL_X_CARRIAGE, hotend_offset[1].x = _MAX(X2_HOME_POS, X2_MAX_POS));
  }
#endif

// The feedrate for the current move, often used as the default if
// no other feedrate is specified. Overridden for special moves.
// Set by the last G0 through G5 command's "F" parameter.
// Functions that override this for custom moves *must always* restore it!
#ifndef DEFAULT_FEEDRATE_MM_M
  #define DEFAULT_FEEDRATE_MM_M 4000
#endif
feedRate_t feedrate_mm_s = MMM_TO_MMS(DEFAULT_FEEDRATE_MM_M);
int16_t feedrate_percentage = 100;
#if ENABLED(EDITABLE_HOMING_FEEDRATE)
  xyz_feedrate_t homing_feedrate_mm_m = HOMING_FEEDRATE_MM_M;
#endif

// Cartesian conversion result goes here:
xyz_pos_t cartes;

#if IS_KINEMATIC

  abce_pos_t delta;

  #if HAS_SCARA_OFFSET
    abc_pos_t scara_home_offset;
  #endif

  #if HAS_SOFTWARE_ENDSTOPS
    float delta_max_radius, delta_max_radius_2;
  #elif IS_SCARA
    constexpr float delta_max_radius = PRINTABLE_RADIUS,
                    delta_max_radius_2 = sq(PRINTABLE_RADIUS);
  #elif ENABLED(POLAR)
    constexpr float delta_max_radius = PRINTABLE_RADIUS,
                    delta_max_radius_2 = sq(PRINTABLE_RADIUS);
  #else // DELTA
    constexpr float delta_max_radius = PRINTABLE_RADIUS,
                    delta_max_radius_2 = sq(PRINTABLE_RADIUS);
  #endif

#endif

/**
 * The workspace can be offset by some commands, or
 * these offsets may be omitted to save on computation.
 */
#if HAS_HOME_OFFSET
  // This offset is added to the configured home position.
  // Set by M206, M428, or menu item. Saved to EEPROM.
  xyz_pos_t home_offset{0};
#endif
#if HAS_WORKSPACE_OFFSET
  // The above two are combined to save on computes
  xyz_pos_t workspace_offset{0};
#endif

#if HAS_ABL_NOT_UBL
  feedRate_t xy_probe_feedrate_mm_s = MMM_TO_MMS(XY_PROBE_FEEDRATE);
#endif

/**
 * Output the current position to serial
 */

inline void report_more_positions() {
  stepper.report_positions();
  TERN_(IS_SCARA, scara_report_positions());
  TERN_(POLAR, polar_report_positions());
}

// Report the logical position for a given machine position
inline void report_logical_position(const xyze_pos_t &rpos) {
  const xyze_pos_t lpos = rpos.asLogical();
  #if NUM_AXES
    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
           X_LBL, lpos.x,
        SP_Y_LBL, lpos.y,
        SP_Z_LBL, lpos.z,
        SP_I_LBL, lpos.i,
        SP_J_LBL, lpos.j,
        SP_K_LBL, lpos.k,
        SP_U_LBL, lpos.u,
        SP_V_LBL, lpos.v,
        SP_W_LBL, lpos.w
      )
    );
  #endif
  #if HAS_EXTRUDERS
    SERIAL_ECHOPGM_P(SP_E_LBL, lpos.e);
  #endif
}

// Report the real current position according to the steppers.
// Forward kinematics and un-leveling are applied.
void report_real_position() {
  get_cartesian_from_steppers();
  xyze_pos_t npos = LOGICAL_AXIS_ARRAY(
    planner.get_axis_position_mm(E_AXIS),
    cartes.x, cartes.y, cartes.z,
    cartes.i, cartes.j, cartes.k,
    cartes.u, cartes.v, cartes.w
  );

  TERN_(HAS_POSITION_MODIFIERS, planner.unapply_modifiers(npos, true));

  report_logical_position(npos);
  report_more_positions();
}

// Report the logical current position according to the most recent G-code command
void report_current_position() {
  report_logical_position(current_position);
  report_more_positions();
}

/**
 * Report the logical current position according to the most recent G-code command.
 * The planner.position always corresponds to the last G-code too. This makes M114
 * suitable for debugging kinematics and leveling while avoiding planner sync that
 * definitively interrupts the printing flow.
 */
void report_current_position_projected() {
  report_logical_position(current_position);
  stepper.report_a_position(planner.position);
}

#if HAS_HOMING_CURRENT

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    auto debug_current = [](FSTR_P const s, const int16_t a, const int16_t b) {
      if (DEBUGGING(LEVELING)) { DEBUG_ECHOLN(s, F(" current: "), a, F(" -> "), b); }
    };
  #else
    #define debug_current(...)
  #endif

  #if HAS_CURRENT_HOME(X)
    int16_t saved_current_X;
  #endif
  #if HAS_CURRENT_HOME(Y)
    int16_t saved_current_Y;
  #endif
  #if HAS_CURRENT_HOME(Z)
    int16_t saved_current_Z;
  #endif
  #if HAS_CURRENT_HOME(X2)
    int16_t saved_current_X2;
  #endif
  #if HAS_CURRENT_HOME(Y2)
    int16_t saved_current_Y2;
  #endif
  #if HAS_CURRENT_HOME(Z2)
    int16_t saved_current_Z2;
  #endif
  #if HAS_CURRENT_HOME(Z3)
    int16_t saved_current_Z3;
  #endif
  #if HAS_CURRENT_HOME(Z4)
    int16_t saved_current_Z4;
  #endif
  #if HAS_CURRENT_HOME(I)
    int16_t saved_current_I;
  #endif
  #if HAS_CURRENT_HOME(J)
    int16_t saved_current_J;
  #endif
  #if HAS_CURRENT_HOME(K)
    int16_t saved_current_K;
  #endif
  #if HAS_CURRENT_HOME(U)
    int16_t saved_current_U;
  #endif
  #if HAS_CURRENT_HOME(V)
    int16_t saved_current_V;
  #endif
  #if HAS_CURRENT_HOME(W)
    int16_t saved_current_W;
  #endif

  /**
   * Set motors to their homing / probing currents.
   * Currents are saved first so they can be restored afterward.
   */
  void set_homing_current(const AxisEnum axis) {

    // Saves the running current of the motor at the moment the function is called and sets current to CURRENT_HOME
    #define _SAVE_SET_CURRENT(A) \
      saved_current_##A = stepper##A.getMilliamps(); \
      stepper##A.rms_current(A##_CURRENT_HOME); \
      debug_current(F(STR_##A), saved_current_##A, A##_CURRENT_HOME)

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Setting homing driver current");

    #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)

      // CORE and Markforged kinematics
      switch (axis) {
        default: break;
        case X_AXIS: case Y_AXIS:
          #if HAS_CURRENT_HOME(X)
            _SAVE_SET_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(X2)
            _SAVE_SET_CURRENT(X2);
          #endif
          #if HAS_CURRENT_HOME(Y)
            _SAVE_SET_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
            _SAVE_SET_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _SAVE_SET_CURRENT(Z);
          #endif
          #if HAS_CURRENT_HOME(Z2)
            _SAVE_SET_CURRENT(Z2);
          #endif
          #if HAS_CURRENT_HOME(Z3)
            _SAVE_SET_CURRENT(Z3);
          #endif
          #if HAS_CURRENT_HOME(Z4)
            _SAVE_SET_CURRENT(Z4);
          #endif
          break;
      }

    #elif CORE_IS_XZ

      // CORE XZ / ZX
      switch (axis) {
        default: break;
        case X_AXIS: case Z_AXIS:
          #if HAS_CURRENT_HOME(X)
            _SAVE_SET_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(Z)
            _SAVE_SET_CURRENT(Z);
          #endif
          break;
        case Y_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _SAVE_SET_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
            _SAVE_SET_CURRENT(Y2);
          #endif
          break;
      }

    #elif CORE_IS_YZ

      // CORE YZ / ZY
      switch (axis) {
        default: break;
        case X_AXIS:
          #if HAS_CURRENT_HOME(X)
            _SAVE_SET_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(X2)
            _SAVE_SET_CURRENT(X2);
          #endif
          break;
        case Y_AXIS: case Z_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _SAVE_SET_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Z)
            _SAVE_SET_CURRENT(Z);
          #endif
          break;
      }

    #elif IS_SCARA

      // SCARA kinematics
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _SAVE_SET_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _SAVE_SET_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _SAVE_SET_CURRENT(Z); break;
        #endif
      }

    #elif ANY(AXEL_TPARA, DELTA)

      // TPARA or DELTA kinematics.
      // Z_AXIS is a special mode to apply homing current to all axes.
      #if HAS_CURRENT_HOME(X)
        if (axis == A_AXIS || axis == Z_AXIS) _SAVE_SET_CURRENT(X);
      #endif
      #if HAS_CURRENT_HOME(Y)
        if (axis == B_AXIS || axis == Z_AXIS) _SAVE_SET_CURRENT(Y);
      #endif
      #if HAS_CURRENT_HOME(Z)
        if (axis == C_AXIS) _SAVE_SET_CURRENT(Z);
      #endif

    #elif ANY(POLARGRAPH, POLAR)

      // POLAR kinematics
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _SAVE_SET_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _SAVE_SET_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _SAVE_SET_CURRENT(Z); break;
        #endif
      }

    #elif defined(ARTICULATED_ROBOT_ARM)

      // Articulated Robot Arm
      // Useful?
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _SAVE_SET_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _SAVE_SET_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _SAVE_SET_CURRENT(Z); break;
        #endif
      }

    #elif defined(FOAMCUTTER_XYUV)

      // Foam cutter
      switch (axis) {
        default: break;
        case X_AXIS: case I_AXIS:
          #if HAS_CURRENT_HOME(X)
            _SAVE_SET_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(I)
            _SAVE_SET_CURRENT(I);
          #endif
          break;
        case Y_AXIS: case J_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _SAVE_SET_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(J)
            _SAVE_SET_CURRENT(J);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _SAVE_SET_CURRENT(Z);
          #endif
          break;
      }

    #else

      // Cartesian kinematics
      switch (axis) {
        default: break;
        case X_AXIS:
          #if HAS_CURRENT_HOME(X)
            _SAVE_SET_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(X2)
            _SAVE_SET_CURRENT(X2);
          #endif
          break;
        case Y_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _SAVE_SET_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
            _SAVE_SET_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _SAVE_SET_CURRENT(Z);
          #endif
          #if HAS_CURRENT_HOME(Z2)
            _SAVE_SET_CURRENT(Z2);
          #endif
          #if HAS_CURRENT_HOME(Z3)
            _SAVE_SET_CURRENT(Z3);
          #endif
          #if HAS_CURRENT_HOME(Z4)
            _SAVE_SET_CURRENT(Z4);
          #endif
          break;
      }

    #endif // kinematics

    switch (axis) {
      default: break;
      #if HAS_CURRENT_HOME(I) && DISABLED(FOAMCUTTER_XYUV)
        case I_AXIS: _SAVE_SET_CURRENT(I); break;
      #endif
      #if HAS_CURRENT_HOME(J) && DISABLED(FOAMCUTTER_XYUV)
        case J_AXIS: _SAVE_SET_CURRENT(J); break;
      #endif
      #if HAS_CURRENT_HOME(K)
        case K_AXIS: _SAVE_SET_CURRENT(K); break;
      #endif
      #if HAS_CURRENT_HOME(U)
        case U_AXIS: _SAVE_SET_CURRENT(U); break;
      #endif
      #if HAS_CURRENT_HOME(V)
        case V_AXIS: _SAVE_SET_CURRENT(V); break;
      #endif
      #if HAS_CURRENT_HOME(W)
        case W_AXIS: _SAVE_SET_CURRENT(W); break;
      #endif
    }

    #if SENSORLESS_STALLGUARD_DELAY
      safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
    #endif

  } // set_homing_current()

  /**
   * Restore motors to their previously-stored currents.
   * Always call set_homing_current() first!
   */
  void restore_homing_current(const AxisEnum axis) {

    // Restore the saved current
    #define _RESTORE_CURRENT(A) \
      stepper##A.rms_current(saved_current_##A); \
      debug_current(F(STR_##A), A##_CURRENT_HOME, saved_current_##A)

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Restore driver current");

    #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX)

      // CORE and Markforged kinematics
      switch (axis) {
        default: break;
        case X_AXIS: case Y_AXIS:
          #if HAS_CURRENT_HOME(X)
            _RESTORE_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(Y)
            _RESTORE_CURRENT(Y);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _RESTORE_CURRENT(Z);
          #endif
          #if HAS_CURRENT_HOME(Z2)
            _RESTORE_CURRENT(Z2);
          #endif
          #if HAS_CURRENT_HOME(Z3)
            _RESTORE_CURRENT(Z3);
          #endif
          #if HAS_CURRENT_HOME(Z4)
            _RESTORE_CURRENT(Z4);
          #endif
          break;
      }

    #elif CORE_IS_XZ

      // CORE XZ / ZX
      switch (axis) {
        default: break;
        case X_AXIS: case Z_AXIS:
          #if HAS_CURRENT_HOME(X)
            _RESTORE_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(Z)
            _RESTORE_CURRENT(Z);
          #endif
          break;
        case Y_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _RESTORE_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
            _RESTORE_CURRENT(Y2);
          #endif
          break;
      }

    #elif CORE_IS_YZ

      // CORE YZ / ZY
      switch (axis) {
        default: break;
        case X_AXIS:
          #if HAS_CURRENT_HOME(X)
            _RESTORE_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(X2)
            _RESTORE_CURRENT(X2);
          #endif
          break;
        case Y_AXIS: case Z_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _RESTORE_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Z)
            _RESTORE_CURRENT(Z);
          #endif
          break;
      }

    #elif IS_SCARA // Unsupported for now?

      // SCARA kinematics
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _RESTORE_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _RESTORE_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _RESTORE_CURRENT(Z); break;
        #endif
      }

    #elif ANY(AXEL_TPARA, DELTA)

      // TPARA or DELTA kinematics
      // Z_AXIS is a special mode to set homing current to all axes
      #if HAS_CURRENT_HOME(X)
        if (axis == A_AXIS || axis == Z_AXIS) _RESTORE_CURRENT(X);
      #endif
      #if HAS_CURRENT_HOME(Y)
        if (axis == B_AXIS || axis == Z_AXIS) _RESTORE_CURRENT(Y);
      #endif
      #if HAS_CURRENT_HOME(Z)
        if (axis == C_AXIS) _RESTORE_CURRENT(Z);
      #endif

    #elif ANY(POLARGRAPH, POLAR)

      // POLAR kinematics
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _RESTORE_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _RESTORE_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _RESTORE_CURRENT(Z); break;
        #endif
      }

    #elif ENABLED(ARTICULATED_ROBOT_ARM)

      // Articulated Robot Arm
      // Useful?
      switch (axis) {
        default: break;
        #if HAS_CURRENT_HOME(X)
          case A_AXIS: _RESTORE_CURRENT(X); break;
        #endif
        #if HAS_CURRENT_HOME(Y)
          case B_AXIS: _RESTORE_CURRENT(Y); break;
        #endif
        #if HAS_CURRENT_HOME(Z)
          case C_AXIS: _RESTORE_CURRENT(Z); break;
        #endif
      }

    #elif ENABLED(FOAMCUTTER_XYUV)

      // Foam cutter
      switch (axis) {
        default: break;
        case X_AXIS: case I_AXIS:
          #if HAS_CURRENT_HOME(X)
            _RESTORE_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(I)
            _RESTORE_CURRENT(I);
          #endif
          break;
        case Y_AXIS: case J_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _RESTORE_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(J)
            _RESTORE_CURRENT(J);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _RESTORE_CURRENT(Z);
          #endif
          break;
      }

    #else

      // Cartesian kinematics
      switch (axis) {
        default: break;
        case X_AXIS:
          #if HAS_CURRENT_HOME(X)
            _RESTORE_CURRENT(X);
          #endif
          #if HAS_CURRENT_HOME(X2)
            _RESTORE_CURRENT(X2);
          #endif
          break;
        case Y_AXIS:
          #if HAS_CURRENT_HOME(Y)
            _RESTORE_CURRENT(Y);
          #endif
          #if HAS_CURRENT_HOME(Y2)
            _RESTORE_CURRENT(Y2);
          #endif
          break;
        case Z_AXIS:
          #if HAS_CURRENT_HOME(Z)
            _RESTORE_CURRENT(Z);
          #endif
          #if HAS_CURRENT_HOME(Z2)
            _RESTORE_CURRENT(Z2);
          #endif
          #if HAS_CURRENT_HOME(Z3)
            _RESTORE_CURRENT(Z3);
          #endif
          #if HAS_CURRENT_HOME(Z4)
            _RESTORE_CURRENT(Z4);
          #endif
          break;
      }

    #endif // kinematics

    switch (axis) {
      default: break;
      #if HAS_CURRENT_HOME(I) && DISABLED(FOAMCUTTER_XYUV)
        case I_AXIS: _RESTORE_CURRENT(I); break;
      #endif
      #if HAS_CURRENT_HOME(J) && DISABLED(FOAMCUTTER_XYUV)
        case J_AXIS: _RESTORE_CURRENT(J); break;
      #endif
      #if HAS_CURRENT_HOME(K)
        case K_AXIS: _RESTORE_CURRENT(K); break;
      #endif
      #if HAS_CURRENT_HOME(U)
        case U_AXIS: _RESTORE_CURRENT(U); break;
      #endif
      #if HAS_CURRENT_HOME(V)
        case V_AXIS: _RESTORE_CURRENT(V); break;
      #endif
      #if HAS_CURRENT_HOME(W)
        case W_AXIS: _RESTORE_CURRENT(W); break;
      #endif
    }

    #if SENSORLESS_STALLGUARD_DELAY
      safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
    #endif

  } // restore_homing_current()

#endif // HAS_HOMING_CURRENT

#if ENABLED(AUTO_REPORT_POSITION)
  AutoReporter<PositionReport> position_auto_reporter;
#endif

#if ENABLED(REALTIME_REPORTING_COMMANDS)

  M_StateEnum M_State_grbl = M_INIT;

  /**
   * Output the current grbl compatible state to serial while moving
   */
  void report_current_grblstate_moving() { SERIAL_ECHOLNPGM("S_XYZ:", int(M_State_grbl)); }

  /**
   * Output the current position (processed) to serial while moving
   */
  void report_current_position_moving() {
    get_cartesian_from_steppers();
    const xyz_pos_t lpos = cartes.asLogical();

    SERIAL_ECHOPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
           X_LBL, lpos.x,
        SP_Y_LBL, lpos.y,
        SP_Z_LBL, lpos.z,
        SP_I_LBL, lpos.i,
        SP_J_LBL, lpos.j,
        SP_K_LBL, lpos.k,
        SP_U_LBL, lpos.u,
        SP_V_LBL, lpos.v,
        SP_W_LBL, lpos.w
      )
      #if HAS_EXTRUDERS
        , SP_E_LBL, current_position.e
      #endif
    );

    report_more_positions();
    report_current_grblstate_moving();
  }

  /**
   * Set a Grbl-compatible state from the current marlin_state
   */
  M_StateEnum grbl_state_for_marlin_state() {
    switch (marlin_state) {
      case MarlinState::MF_INITIALIZING: return M_INIT;
      case MarlinState::MF_SD_COMPLETE:  return M_ALARM;
      case MarlinState::MF_WAITING:      return M_IDLE;
      case MarlinState::MF_STOPPED:      return M_END;
      case MarlinState::MF_RUNNING:      return M_RUNNING;
      case MarlinState::MF_PAUSED:       return M_HOLD;
      case MarlinState::MF_KILLED:       return M_ERROR;
      default:                           return M_IDLE;
    }
  }

#endif // REALTIME_REPORTING_COMMANDS

#if IS_KINEMATIC

  bool position_is_reachable(const_float_t rx, const_float_t ry, const float inset/*=0*/) {

    bool can_reach;

    #if ENABLED(DELTA)

      can_reach = HYPOT2(rx, ry) <= sq(PRINTABLE_RADIUS - inset + fslop);

    #elif ENABLED(AXEL_TPARA)

      const float R2 = HYPOT2(rx - TPARA_OFFSET_X, ry - TPARA_OFFSET_Y);
      can_reach = (
        R2 <= sq(L1 + L2) - inset
        #if MIDDLE_DEAD_ZONE_R > 0
          && R2 >= FLOAT_SQ(MIDDLE_DEAD_ZONE_R)
        #endif
      );

    #elif IS_SCARA

      const float R2 = HYPOT2(rx - SCARA_OFFSET_X, ry - SCARA_OFFSET_Y);
      can_reach = (
        R2 <= sq(L1 + L2) - inset
        #if MIDDLE_DEAD_ZONE_R > 0
          && R2 >= FLOAT_SQ(MIDDLE_DEAD_ZONE_R)
        #endif
      );

    #elif ENABLED(POLARGRAPH)

      const float d1 = rx - (draw_area_min.x),
                  d2 = (draw_area_max.x) - rx,
                   y = ry - (draw_area_max.y),
                   a = HYPOT(d1, y),
                   b = HYPOT(d2, y);

      can_reach = (
           a < polargraph_max_belt_len + 1
        && b < polargraph_max_belt_len + 1
      );

    #elif ENABLED(POLAR)
      can_reach = HYPOT(rx, ry) <= PRINTABLE_RADIUS;
    #endif

    return can_reach;
  }

#else // CARTESIAN

  // Return true if the given position is within the machine bounds.
  bool position_is_reachable(TERN_(HAS_X_AXIS, const_float_t rx) OPTARG(HAS_Y_AXIS, const_float_t ry)) {
    if (TERN0(HAS_Y_AXIS, !COORDINATE_OKAY(ry, Y_MIN_POS - fslop, Y_MAX_POS + fslop))) return false;
    #if ENABLED(DUAL_X_CARRIAGE)
      if (active_extruder)
        return COORDINATE_OKAY(rx, X2_MIN_POS - fslop, X2_MAX_POS + fslop);
      else
        return COORDINATE_OKAY(rx, X1_MIN_POS - fslop, X1_MAX_POS + fslop);
    #else
      if (TERN0(HAS_X_AXIS, !COORDINATE_OKAY(rx, X_MIN_POS - fslop, X_MAX_POS + fslop))) return false;
      return true;
    #endif
  }

#endif // CARTESIAN

void home_if_needed(const bool keeplev/*=false*/) {
  if (!all_axes_trusted()) gcode.home_all_axes(keeplev);
}

/**
 * Run out the planner buffer and re-sync the current
 * position from the last-updated stepper positions.
 */
void quickstop_stepper() {
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  sync_plan_position();
}

#if ENABLED(REALTIME_REPORTING_COMMANDS)

  void quickpause_stepper() {
    planner.quick_pause();
    //planner.synchronize();
  }

  void quickresume_stepper() {
    planner.quick_resume();
    //planner.synchronize();
  }

#endif

/**
 * Set the planner/stepper positions directly from current_position with
 * no kinematic translation. Used for homing axes and cartesian/core syncing.
 */
void sync_plan_position() {
  if (DEBUGGING(LEVELING)) DEBUG_POS("sync_plan_position", current_position);
  planner.set_position_mm(current_position);
}

#if HAS_EXTRUDERS
  void sync_plan_position_e() { planner.set_e_position_mm(current_position.e); }
#endif

/**
 * Get the stepper positions in the cartes[] array.
 * Forward kinematics are applied for DELTA and SCARA.
 *
 * The result is in the current coordinate space with
 * leveling applied. The coordinates need to be run through
 * unapply_leveling to obtain the "ideal" coordinates
 * suitable for current_position, etc.
 */
void get_cartesian_from_steppers() {
  #if ENABLED(DELTA)
    forward_kinematics(planner.get_axis_positions_mm());
  #elif IS_SCARA
    forward_kinematics(
      planner.get_axis_position_degrees(A_AXIS), planner.get_axis_position_degrees(B_AXIS)
      OPTARG(AXEL_TPARA, planner.get_axis_position_degrees(C_AXIS))
    );
    cartes.z = planner.get_axis_position_mm(Z_AXIS);
  #elif ENABLED(POLAR)
    forward_kinematics(planner.get_axis_position_mm(X_AXIS), planner.get_axis_position_degrees(B_AXIS));
    cartes.z = planner.get_axis_position_mm(Z_AXIS);
  #else
    NUM_AXIS_CODE(
      cartes.x = planner.get_axis_position_mm(X_AXIS),
      cartes.y = planner.get_axis_position_mm(Y_AXIS),
      cartes.z = planner.get_axis_position_mm(Z_AXIS),
      cartes.i = planner.get_axis_position_mm(I_AXIS),
      cartes.j = planner.get_axis_position_mm(J_AXIS),
      cartes.k = planner.get_axis_position_mm(K_AXIS),
      cartes.u = planner.get_axis_position_mm(U_AXIS),
      cartes.v = planner.get_axis_position_mm(V_AXIS),
      cartes.w = planner.get_axis_position_mm(W_AXIS)
    );
  #endif
}

/**
 * Set the current_position for an axis based on
 * the stepper positions, removing any leveling that
 * may have been applied.
 *
 * To prevent small shifts in axis position always call
 * sync_plan_position after updating axes with this.
 *
 * To keep hosts in sync, always call report_current_position
 * after updating the current_position.
 */
void set_current_from_steppers_for_axis(const AxisEnum axis) {
  get_cartesian_from_steppers();
  xyze_pos_t pos = cartes;

  TERN_(HAS_EXTRUDERS, pos.e = planner.get_axis_position_mm(E_AXIS));

  TERN_(HAS_POSITION_MODIFIERS, planner.unapply_modifiers(pos, true));

  if (axis == ALL_AXES_ENUM)
    current_position = pos;
  else
    current_position[axis] = pos[axis];
}

/**
 * Move the planner to the current position from wherever it last moved
 * (or from wherever it has been told it is located).
 */
void line_to_current_position(const_feedRate_t fr_mm_s/*=feedrate_mm_s*/) {
  planner.buffer_line(current_position, fr_mm_s);
}

#if HAS_EXTRUDERS
  void unscaled_e_move(const_float_t length, const_feedRate_t fr_mm_s) {
    TERN_(HAS_FILAMENT_SENSOR, runout.reset());
    current_position.e += length / planner.e_factor[active_extruder];
    line_to_current_position(fr_mm_s);
    planner.synchronize();
  }
#endif

#if IS_KINEMATIC

  /**
   * Buffer a fast move without interpolation. Set current_position to destination
   */
  void prepare_fast_move_to_destination(const_feedRate_t scaled_fr_mm_s/*=MMS_SCALED(feedrate_mm_s)*/) {
    if (DEBUGGING(LEVELING)) DEBUG_POS("prepare_fast_move_to_destination", destination);

    #if UBL_SEGMENTED
      // UBL segmented line will do Z-only moves in single segment
      bedlevel.line_to_destination_segmented(scaled_fr_mm_s);
    #else
      if (current_position == destination) return;

      planner.buffer_line(destination, scaled_fr_mm_s);
    #endif

    current_position = destination;
  }

#endif // IS_KINEMATIC

/**
 * Do a fast or normal move to 'destination' with an optional FR.
 *  - Move at normal speed regardless of feedrate percentage.
 *  - Extrude the specified length regardless of flow percentage.
 */
void _internal_move_to_destination(const_feedRate_t fr_mm_s/*=0.0f*/
  OPTARG(IS_KINEMATIC, const bool is_fast/*=false*/)
) {
  REMEMBER(fr, feedrate_mm_s);
  REMEMBER(pct, feedrate_percentage, 100);
  TERN_(HAS_EXTRUDERS, REMEMBER(fac, planner.e_factor[active_extruder], 1.0f));

  if (fr_mm_s) feedrate_mm_s = fr_mm_s;
  if (TERN0(IS_KINEMATIC, is_fast))
    TERN(IS_KINEMATIC, prepare_fast_move_to_destination(), NOOP);
  else
    prepare_line_to_destination();
}

#if SECONDARY_AXES

  void secondary_axis_moves(SECONDARY_AXIS_ARGS_LC(const_float_t), const_feedRate_t fr_mm_s) {
    auto move_one = [&](const AxisEnum a, const_float_t p) {
      const feedRate_t fr = fr_mm_s ?: homing_feedrate(a);
      current_position[a] = p; line_to_current_position(fr);
    };
    SECONDARY_AXIS_CODE(
      move_one(I_AXIS, i), move_one(J_AXIS, j), move_one(K_AXIS, k),
      move_one(U_AXIS, u), move_one(V_AXIS, v), move_one(W_AXIS, w)
    );
  }

#endif

/**
 * Plan a move to (X, Y, Z, [I, [J, [K...]]]) and set the current_position
 * Plan a move to (X, Y, Z, [I, [J, [K...]]]) with separation of Z from other components.
 *
 * - If Z is moving up, the Z move is done before XY, etc.
 * - If Z is moving down, the Z move is done after XY, etc.
 * - Delta may lower Z first to get into the free motion zone.
 * - Before returning, wait for the planner buffer to empty.
 */
void do_blocking_move_to(NUM_AXIS_ARGS_(const_float_t) const_feedRate_t fr_mm_s/*=0.0f*/) {
  DEBUG_SECTION(log_move, "do_blocking_move_to", DEBUGGING(LEVELING));
  #if NUM_AXES
    if (DEBUGGING(LEVELING)) DEBUG_XYZ("> ", NUM_AXIS_ARGS_LC());
  #endif

  const feedRate_t xy_feedrate = fr_mm_s ?: feedRate_t(XY_PROBE_FEEDRATE_MM_S);

  #if HAS_Z_AXIS
    const feedRate_t z_feedrate = fr_mm_s ?: homing_feedrate(Z_AXIS);
  #endif

  #if IS_KINEMATIC && DISABLED(POLARGRAPH)
    // kinematic machines are expected to home to a point 1.5x their range? never reachable.
    if (!position_is_reachable(x, y)) return;
    destination = current_position;          // sync destination at the start
  #endif

  #if ENABLED(DELTA)

    REMEMBER(fr, feedrate_mm_s, xy_feedrate);

    if (DEBUGGING(LEVELING)) DEBUG_POS("destination = current_position", destination);

    // when in the danger zone
    if (current_position.z > delta_clip_start_height) {
      if (z > delta_clip_start_height) {                      // staying in the danger zone
        destination.set(x, y, z);                             // move directly (uninterpolated)
        prepare_internal_fast_move_to_destination();          // set current_position from destination
        if (DEBUGGING(LEVELING)) DEBUG_POS("danger zone move", current_position);
        return;
      }
      destination.z = delta_clip_start_height;
      prepare_internal_fast_move_to_destination();            // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("zone border move", current_position);
    }

    if (z > current_position.z) {                             // raising?
      destination.z = z;
      prepare_internal_fast_move_to_destination(z_feedrate);  // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("z raise move", current_position);
    }

    destination.set(x, y);
    prepare_internal_move_to_destination();                   // set current_position from destination
    if (DEBUGGING(LEVELING)) DEBUG_POS("xy move", current_position);

    if (z < current_position.z) {                             // lowering?
      destination.z = z;
      prepare_internal_fast_move_to_destination(z_feedrate);  // set current_position from destination
      if (DEBUGGING(LEVELING)) DEBUG_POS("z lower move", current_position);
    }

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

  #elif IS_SCARA

    // If Z needs to raise, do it before moving XY
    if (destination.z < z) { destination.z = z; prepare_internal_fast_move_to_destination(z_feedrate); }

    destination.set(x, y); prepare_internal_fast_move_to_destination(xy_feedrate);

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

    // If Z needs to lower, do it after moving XY
    if (destination.z > z) { destination.z = z; prepare_internal_fast_move_to_destination(z_feedrate); }

  #else

    #if HAS_Z_AXIS  // If Z needs to raise, do it before moving XY
      if (current_position.z < z) { current_position.z = z; line_to_current_position(z_feedrate); }
    #endif

    current_position.set(TERN_(HAS_X_AXIS, x) OPTARG(HAS_Y_AXIS, y)); line_to_current_position(xy_feedrate);

    #if SECONDARY_AXES
      secondary_axis_moves(SECONDARY_AXIS_LIST(i, j, k, u, v, w), fr_mm_s);
    #endif

    #if HAS_Z_AXIS
      // If Z needs to lower, do it after moving XY
      if (current_position.z > z) { current_position.z = z; line_to_current_position(z_feedrate); }
    #endif

  #endif

  planner.synchronize();
}

void do_blocking_move_to(const xy_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_LIST_(raw.x, raw.y, current_position.z,
                                    current_position.i, current_position.j, current_position.k,
                                    current_position.u, current_position.v, current_position.w) fr_mm_s);
}
void do_blocking_move_to(const xyz_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_ELEM_(raw) fr_mm_s);
}
void do_blocking_move_to(const xyze_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
  do_blocking_move_to(NUM_AXIS_ELEM_(raw) fr_mm_s);
}

#if HAS_X_AXIS
  void do_blocking_move_to_x(const_float_t rx, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_x(", rx, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(rx, current_position.y, current_position.z,
                     current_position.i, current_position.j, current_position.k,
                     current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
#endif

#if HAS_Y_AXIS
  void do_blocking_move_to_y(const_float_t ry, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_y(", ry, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(current_position.x, ry, current_position.z,
                    current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_xy(const_float_t rx, const_float_t ry, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_xy(", rx, ", ", ry, ", ", fr_mm_s, ")");
    do_blocking_move_to(
      NUM_AXIS_LIST_(rx, ry, current_position.z,
                    current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_xy(const xy_pos_t &raw, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to_xy(raw.x, raw.y, fr_mm_s);
  }
#endif

#if HAS_Z_AXIS
  void do_blocking_move_to_z(const_float_t rz, const_feedRate_t fr_mm_s/*=0.0*/) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_blocking_move_to_z(", rz, ", ", fr_mm_s, ")");
    do_blocking_move_to_xy_z(current_position, rz, fr_mm_s);
  }
  void do_blocking_move_to_xy_z(const xy_pos_t &raw, const_float_t z, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, z,
                    current_position.i, current_position.j, current_position.k,
                    current_position.u, current_position.v, current_position.w)
      fr_mm_s
    );
  }
  /**
   * Move Z to a particular height so the nozzle or deployed probe clears the bed.
   * (Use do_z_clearance_by for clearance over the current position.)
   *  - For a probe, add clearance for the probe distance
   *  - Constrain to the Z max physical position
   *  - If lowering is not allowed then skip a downward move
   *  - Execute the move at the probing (or homing) feedrate
   */
  void do_z_clearance(const_float_t zclear, const bool with_probe/*=true*/, const bool lower_allowed/*=false*/) {
    UNUSED(with_probe);
    float zdest = zclear;
    TERN_(HAS_BED_PROBE, if (with_probe && probe.offset.z < 0) zdest -= probe.offset.z);
    NOMORE(zdest, Z_MAX_POS);
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_z_clearance(", zclear, " [", current_position.z, " to ", zdest, "], ", lower_allowed, ")");
    if ((!lower_allowed && zdest < current_position.z) || zdest == current_position.z) return;
    do_blocking_move_to_z(zdest, TERN(HAS_BED_PROBE, z_probe_fast_mm_s, homing_feedrate(Z_AXIS)));
  }
  void do_z_clearance_by(const_float_t zclear) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("do_z_clearance_by(", zclear, ")");
    do_z_clearance(current_position.z + zclear, false);
  }
  /**
   * Move Z to Z_POST_CLEARANCE,
   * The axis is allowed to move down.
   */
  void do_move_after_z_homing() {
    DEBUG_SECTION(mzah, "do_move_after_z_homing", DEBUGGING(LEVELING));
    #ifdef Z_POST_CLEARANCE
      do_z_clearance(
        Z_POST_CLEARANCE,
        ALL(HOMING_Z_WITH_PROBE, HAS_STOWABLE_PROBE) && TERN0(HAS_BED_PROBE, endstops.z_probe_enabled),
        true
      );
    #elif ENABLED(USE_PROBE_FOR_Z_HOMING)
      probe.move_z_after_probing();
    #endif
  }
#endif // HAS_Z_AXIS

#if HAS_I_AXIS
  void do_blocking_move_to_xyz_i(const xyze_pos_t &raw, const_float_t i, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, i, raw.j, raw.k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_i(const_float_t ri, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyz_i(current_position, ri, fr_mm_s);
  }
#endif

#if HAS_J_AXIS
  void do_blocking_move_to_xyzi_j(const xyze_pos_t &raw, const_float_t j, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, j, raw.k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_j(const_float_t rj, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzi_j(current_position, rj, fr_mm_s);
  }
#endif

#if HAS_K_AXIS
  void do_blocking_move_to_xyzij_k(const xyze_pos_t &raw, const_float_t k, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, k, raw.u, raw.v, raw.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_k(const_float_t rk, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzij_k(current_position, rk, fr_mm_s);
  }
#endif

#if HAS_U_AXIS
  void do_blocking_move_to_xyzijk_u(const xyze_pos_t &raw, const_float_t u, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, u, raw.v, raw.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_u(const_float_t ru, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijk_u(current_position, ru, fr_mm_s);
  }
#endif

#if HAS_V_AXIS
  void do_blocking_move_to_xyzijku_v(const xyze_pos_t &raw, const_float_t v, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, raw.u, v, raw.w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_v(const_float_t rv, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijku_v(current_position, rv, fr_mm_s);
  }
#endif

#if HAS_W_AXIS
  void do_blocking_move_to_xyzijkuv_w(const xyze_pos_t &raw, const_float_t w, const_feedRate_t fr_mm_s/*=0.0f*/) {
    do_blocking_move_to(
      NUM_AXIS_LIST_(raw.x, raw.y, raw.z, raw.i, raw.j, raw.k, raw.u, raw.v, w)
      fr_mm_s
    );
  }
  void do_blocking_move_to_w(const_float_t rw, const_feedRate_t fr_mm_s/*=0.0*/) {
    do_blocking_move_to_xyzijkuv_w(current_position, rw, fr_mm_s);
  }
#endif

//
// Prepare to do endstop or probe moves with custom feedrates.
//  - Save / restore current feedrate and multiplier
//
static float saved_feedrate_mm_s;
static int16_t saved_feedrate_percentage;
void remember_feedrate_scaling_off() {
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("remember_feedrate_scaling_off: fr=", feedrate_mm_s, " ", feedrate_percentage, "%");
  saved_feedrate_mm_s = feedrate_mm_s;
  saved_feedrate_percentage = feedrate_percentage;
  feedrate_percentage = 100;
}
void restore_feedrate_and_scaling() {
  feedrate_mm_s = saved_feedrate_mm_s;
  feedrate_percentage = saved_feedrate_percentage;
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("restore_feedrate_and_scaling: fr=", feedrate_mm_s, " ", feedrate_percentage, "%");
}

#if HAS_SOFTWARE_ENDSTOPS

  // Software Endstops are based on the configured limits.
  #define _AMIN(A) A##_MIN_POS
  #define _AMAX(A) A##_MAX_POS
  soft_endstops_t soft_endstop = {
    true, false,
    { MAPLIST(_AMIN, MAIN_AXIS_NAMES) },
    { MAPLIST(_AMAX, MAIN_AXIS_NAMES) },
  };

  /**
   * Software endstops can be used to monitor the open end of
   * an axis that has a hardware endstop on the other end. Or
   * they can prevent axes from moving past endstops and grinding.
   *
   * To keep doing their job as the coordinate system changes,
   * the software endstop positions must be refreshed to remain
   * at the same positions relative to the machine.
   */
  void update_software_endstops(const AxisEnum axis
    OPTARG(HAS_HOTEND_OFFSET, const uint8_t old_tool_index/*=0*/, const uint8_t new_tool_index/*=0*/)
  ) {

    #if ENABLED(DUAL_X_CARRIAGE)

      if (axis == X_AXIS) {

        // In Dual X mode hotend_offset[X] is T1's home position
        const float dual_max_x = _MAX(hotend_offset[1].x, X2_MAX_POS);

        if (new_tool_index != 0) {
          // T1 can move from X2_MIN_POS to X2_MAX_POS or X2 home position (whichever is larger)
          soft_endstop.min.x = X2_MIN_POS;
          soft_endstop.max.x = dual_max_x;
        }
        else if (idex_is_duplicating()) {
          // In Duplication Mode, T0 can move as far left as X1_MIN_POS
          // but not so far to the right that T1 would move past the end
          soft_endstop.min.x = X1_MIN_POS;
          soft_endstop.max.x = _MIN(X1_MAX_POS, dual_max_x - duplicate_extruder_x_offset);
        }
        else {
          // In other modes, T0 can move from X1_MIN_POS to X1_MAX_POS
          soft_endstop.min.x = X1_MIN_POS;
          soft_endstop.max.x = X1_MAX_POS;
        }

      }

    #elif ENABLED(DELTA)

      soft_endstop.min[axis] = base_min_pos(axis);
      soft_endstop.max[axis] = (axis == Z_AXIS) ? DIFF_TERN(HAS_BED_PROBE, delta_height, probe.offset.z) : base_max_pos(axis);

      switch (axis) {
        case X_AXIS:
        case Y_AXIS:
          // Get a minimum radius for clamping
          delta_max_radius = _MIN(ABS(_MAX(soft_endstop.min.x, soft_endstop.min.y)), soft_endstop.max.x, soft_endstop.max.y);
          delta_max_radius_2 = sq(delta_max_radius);
          break;
        case Z_AXIS:
          refresh_delta_clip_start_height();
        default: break;
      }

    #elif HAS_HOTEND_OFFSET

      // Software endstops are relative to the tool 0 workspace, so
      // the movement limits must be shifted by the tool offset to
      // retain the same physical limit when other tools are selected.

      if (new_tool_index == old_tool_index || axis == Z_AXIS) { // The Z axis is "special" and shouldn't be modified
        const float offs = (axis == Z_AXIS) ? 0 : hotend_offset[active_extruder][axis];
        soft_endstop.min[axis] = base_min_pos(axis) + offs;
        soft_endstop.max[axis] = base_max_pos(axis) + offs;
      }
      else {
        const float diff = hotend_offset[new_tool_index][axis] - hotend_offset[old_tool_index][axis];
        soft_endstop.min[axis] += diff;
        soft_endstop.max[axis] += diff;
      }

    #else

      soft_endstop.min[axis] = base_min_pos(axis);
      soft_endstop.max[axis] = base_max_pos(axis);

    #endif

    if (DEBUGGING(LEVELING))
      SERIAL_ECHOLNPGM("Axis ", C(AXIS_CHAR(axis)), " min:", soft_endstop.min[axis], " max:", soft_endstop.max[axis]);
  }

  /**
   * Constrain the given coordinates to the software endstops.
   *
   * For DELTA/SCARA the XY constraint is based on the smallest
   * radius within the set software endstops.
   */
  void apply_motion_limits(xyz_pos_t &target) {

    if (!soft_endstop._enabled) return;

    #if IS_KINEMATIC

      if (TERN0(DELTA, !all_axes_homed())) return;

      #if ALL(HAS_HOTEND_OFFSET, DELTA)
        // The effector center position will be the target minus the hotend offset.
        const xy_pos_t offs = hotend_offset[active_extruder];
      #elif ENABLED(POLARGRAPH)
        // POLARGRAPH uses draw_area_* below...
      #elif ENABLED(POLAR)
        // For now, we don't limit POLAR
      #else
        // SCARA needs to consider the angle of the arm through the entire move, so for now use no tool offset.
        constexpr xy_pos_t offs{0};
      #endif

      #if ENABLED(POLARGRAPH)
        LIMIT(target.x, draw_area_min.x, draw_area_max.x);
        LIMIT(target.y, draw_area_min.y, draw_area_max.y);
      #elif ENABLED(POLAR)
        // Motion limits are as same as cartesian limits.
      #else
        if (TERN1(IS_SCARA, axis_was_homed(X_AXIS) && axis_was_homed(Y_AXIS))) {
          const float dist_2 = HYPOT2(target.x - offs.x, target.y - offs.y);
          if (dist_2 > delta_max_radius_2)
            target *= float(delta_max_radius / SQRT(dist_2)); // 200 / 300 = 0.66
        }
      #endif

    #else

      #if HAS_X_AXIS
        if (axis_was_homed(X_AXIS)) {
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_X)
            NOLESS(target.x, soft_endstop.min.x);
          #endif
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_X)
            NOMORE(target.x, soft_endstop.max.x);
          #endif
        }
      #endif

      #if HAS_Y_AXIS
        if (axis_was_homed(Y_AXIS)) {
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_Y)
            NOLESS(target.y, soft_endstop.min.y);
          #endif
          #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_Y)
            NOMORE(target.y, soft_endstop.max.y);
          #endif
        }
      #endif

    #endif

    #if HAS_Z_AXIS
      if (axis_was_homed(Z_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_Z)
          NOLESS(target.z, soft_endstop.min.z);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_Z)
          NOMORE(target.z, soft_endstop.max.z);
        #endif
      }
    #endif
    #if HAS_I_AXIS
      if (axis_was_homed(I_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_I)
          NOLESS(target.i, soft_endstop.min.i);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_I)
          NOMORE(target.i, soft_endstop.max.i);
        #endif
      }
    #endif
    #if HAS_J_AXIS
      if (axis_was_homed(J_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_J)
          NOLESS(target.j, soft_endstop.min.j);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_J)
          NOMORE(target.j, soft_endstop.max.j);
        #endif
      }
    #endif
    #if HAS_K_AXIS
      if (axis_was_homed(K_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_K)
          NOLESS(target.k, soft_endstop.min.k);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_K)
          NOMORE(target.k, soft_endstop.max.k);
        #endif
      }
    #endif
    #if HAS_U_AXIS
      if (axis_was_homed(U_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_U)
          NOLESS(target.u, soft_endstop.min.u);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_U)
          NOMORE(target.u, soft_endstop.max.u);
        #endif
      }
    #endif
    #if HAS_V_AXIS
      if (axis_was_homed(V_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_V)
          NOLESS(target.v, soft_endstop.min.v);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_V)
          NOMORE(target.v, soft_endstop.max.v);
        #endif
      }
    #endif
    #if HAS_W_AXIS
      if (axis_was_homed(W_AXIS)) {
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MIN_SOFTWARE_ENDSTOP_W)
          NOLESS(target.w, soft_endstop.min.w);
        #endif
        #if !HAS_SOFTWARE_ENDSTOPS || ENABLED(MAX_SOFTWARE_ENDSTOP_W)
          NOMORE(target.w, soft_endstop.max.w);
        #endif
      }
    #endif
  }

#else // !HAS_SOFTWARE_ENDSTOPS

  soft_endstops_t soft_endstop;

#endif // !HAS_SOFTWARE_ENDSTOPS

FORCE_INLINE void segment_idle(millis_t &next_idle_ms) {
  const millis_t ms = millis();
  if (ELAPSED(ms, next_idle_ms)) {
    next_idle_ms = ms + 200UL;
    return idle();
  }
  thermalManager.task();  // Returns immediately on most calls
}

/**
 * Get distance from displacements along axes and, if required, update move type.
 */
float get_move_distance(const xyze_pos_t &diff OPTARG(HAS_ROTATIONAL_AXES, bool &is_cartesian_move)) {
  #if NUM_AXES

    if (!(NUM_AXIS_GANG(diff.x, || diff.y, /* skip z */, || diff.i, || diff.j, || diff.k, || diff.u, || diff.v, || diff.w)))
      return TERN0(HAS_Z_AXIS, ABS(diff.z));

    #if ENABLED(ARTICULATED_ROBOT_ARM)

      // For articulated robots, interpreting feedrate like LinuxCNC would require inverse kinematics. As a workaround, pretend that motors sit on n mutually orthogonal
      // axes and assume that we could think of distance as magnitude of an n-vector in an n-dimensional Euclidian space.
      const float distance_sqr = NUM_AXIS_GANG(
          sq(diff.x), + sq(diff.y), + sq(diff.z),
        + sq(diff.i), + sq(diff.j), + sq(diff.k),
        + sq(diff.u), + sq(diff.v), + sq(diff.w)
      );

    #elif ENABLED(FOAMCUTTER_XYUV)

      const float distance_sqr = (
        #if HAS_J_AXIS
          _MAX(sq(diff.x) + sq(diff.y), sq(diff.i) + sq(diff.j)) // Special 5 axis kinematics. Return the larger of plane X/Y or I/J
        #else
          sq(diff.x) + sq(diff.y) // Foamcutter with only two axes (XY)
        #endif
      );

    #else

      /**
       * Calculate distance for feedrate interpretation in accordance with NIST RS274NGC interpreter - version 3) and its default CANON_XYZ feed reference mode.
       * Assume:
       *   - X, Y, Z are the primary linear axes;
       *   - U, V, W are secondary linear axes;
       *   - A, B, C are rotational axes.
       *
       * Then:
       *   - dX, dY, dZ are the displacements of the primary linear axes;
       *   - dU, dV, dW are the displacements of linear axes;
       *   - dA, dB, dC are the displacements of rotational axes.
       *
       * The time it takes to execute a move command with feedrate F is t = D/F,
       * plus any time for acceleration and deceleration.
       * Here, D is the total distance, calculated as follows:
       *
       *   D^2 = dX^2 + dY^2 + dZ^2
       *   if D^2 == 0 (none of XYZ move but any secondary linear axes move, whether other axes are moved or not):
       *     D^2 = dU^2 + dV^2 + dW^2
       *   if D^2 == 0 (only rotational axes are moved):
       *     D^2 = dA^2 + dB^2 + dC^2
       */
      float distance_sqr = XYZ_GANG(sq(diff.x), + sq(diff.y), + sq(diff.z));

      #if SECONDARY_LINEAR_AXES
        if (UNEAR_ZERO(distance_sqr)) {
          // Move does not involve any primary linear axes (xyz) but might involve secondary linear axes
          distance_sqr = (
            SECONDARY_AXIS_GANG(
              IF_DISABLED(AXIS4_ROTATES, + sq(diff.i)),
              IF_DISABLED(AXIS5_ROTATES, + sq(diff.j)),
              IF_DISABLED(AXIS6_ROTATES, + sq(diff.k)),
              IF_DISABLED(AXIS7_ROTATES, + sq(diff.u)),
              IF_DISABLED(AXIS8_ROTATES, + sq(diff.v)),
              IF_DISABLED(AXIS9_ROTATES, + sq(diff.w))
            )
          );
        }
      #endif

      #if HAS_ROTATIONAL_AXES
        if (UNEAR_ZERO(distance_sqr)) {
          // Move involves no linear axes. Calculate angular distance in accordance with LinuxCNC
          distance_sqr = ROTATIONAL_AXIS_GANG(sq(diff.i), + sq(diff.j), + sq(diff.k), + sq(diff.u), + sq(diff.v), + sq(diff.w));
        }
        if (!UNEAR_ZERO(distance_sqr)) {
          // Move involves rotational axes, not just the extruder
          is_cartesian_move = false;
        }
      #endif

    #endif

    return SQRT(distance_sqr);

  #else

    return 0;

  #endif
}

#if IS_KINEMATIC

  #if IS_SCARA
    /**
     * Before raising this value, use M665 S[seg_per_sec] to decrease
     * the number of segments-per-second. Default is 200. Some deltas
     * do better with 160 or lower. It would be good to know how many
     * segments-per-second are actually possible for SCARA on AVR.
     *
     * Longer segments result in less kinematic overhead
     * but may produce jagged lines. Try 0.5mm, 1.0mm, and 2.0mm
     * and compare the difference.
     */
    #define SCARA_MIN_SEGMENT_LENGTH 0.5f
  #elif ENABLED(POLAR)
    #define POLAR_MIN_SEGMENT_LENGTH 0.5f
  #endif

  /**
   * Prepare a linear move in a DELTA or SCARA setup.
   *
   * Called from prepare_line_to_destination as the
   * default Delta/SCARA segmenter.
   *
   * This calls planner.buffer_line several times, adding
   * small incremental moves for DELTA or SCARA.
   *
   * For Unified Bed Leveling (Delta or Segmented Cartesian)
   * the bedlevel.line_to_destination_segmented method replaces this.
   *
   * For Auto Bed Leveling (Bilinear) with SEGMENT_LEVELED_MOVES
   * this is replaced by segmented_line_to_destination below.
   */
  inline bool line_to_destination_kinematic() {

    // Get the top feedrate of the move in the XY plane
    const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);

    const xyze_float_t diff = destination - current_position;

    // If the move is only in Z/E don't split up the move
    if (!diff.x && !diff.y) {
      planner.buffer_line(destination, scaled_fr_mm_s);
      return false; // caller will update current_position
    }

    // Fail if attempting move outside printable radius
    if (!position_is_reachable(destination)) return true;

    // Get the linear distance in XYZ
    #if HAS_ROTATIONAL_AXES
      bool cartes_move = true;
    #endif
    float cartesian_mm = get_move_distance(diff OPTARG(HAS_ROTATIONAL_AXES, cartes_move));

    // If the move is very short, check the E move distance
    TERN_(HAS_EXTRUDERS, if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(diff.e));

    // No E move either? Game over.
    if (UNEAR_ZERO(cartesian_mm)) return true;

    // Minimum number of seconds to move the given distance
    const float seconds = cartesian_mm / (
      #if ALL(HAS_ROTATIONAL_AXES, INCH_MODE_SUPPORT)
        cartes_move ? scaled_fr_mm_s : LINEAR_UNIT(scaled_fr_mm_s)
      #else
        scaled_fr_mm_s
      #endif
    );

    // The number of segments-per-second times the duration
    // gives the number of segments
    uint16_t segments = segments_per_second * seconds;

    // For SCARA enforce a minimum segment size
    #if IS_SCARA
      NOMORE(segments, cartesian_mm * RECIPROCAL(SCARA_MIN_SEGMENT_LENGTH));
    #elif ENABLED(POLAR)
      NOMORE(segments, cartesian_mm * RECIPROCAL(POLAR_MIN_SEGMENT_LENGTH));
    #endif

    // At least one segment is required
    NOLESS(segments, 1U);

    // The approximate length of each segment
    const float inv_segments = 1.0f / float(segments);
    const xyze_float_t segment_distance = diff * inv_segments;

    // Add hints to help optimize the move
    PlannerHints hints(cartesian_mm * inv_segments);
    TERN_(HAS_ROTATIONAL_AXES, hints.cartesian_move = cartes_move);
    TERN_(FEEDRATE_SCALING, hints.inv_duration = scaled_fr_mm_s / hints.millimeters);

    /*
    SERIAL_ECHOPGM("mm=", cartesian_mm);
    SERIAL_ECHOPGM(" seconds=", seconds);
    SERIAL_ECHOPGM(" segments=", segments);
    SERIAL_ECHOPGM(" segment_mm=", hints.millimeters);
    SERIAL_EOL();
    //*/

    // Get the current position as starting point
    xyze_pos_t raw = current_position;

    // Calculate and execute the segments
    millis_t next_idle_ms = millis() + 200UL;
    while (--segments) {
      segment_idle(next_idle_ms);
      raw += segment_distance;
      if (!planner.buffer_line(raw, scaled_fr_mm_s, active_extruder, hints))
        break;
    }

    // Ensure last segment arrives at target location.
    planner.buffer_line(destination, scaled_fr_mm_s, active_extruder, hints);

    return false; // caller will update current_position
  }

#else // !IS_KINEMATIC

  #if ENABLED(SEGMENT_LEVELED_MOVES) && DISABLED(AUTO_BED_LEVELING_UBL)

    /**
     * Prepare a segmented move on a CARTESIAN setup.
     *
     * This calls planner.buffer_line several times, adding
     * small incremental moves. This allows the planner to
     * apply more detailed bed leveling to the full move.
     */
    inline void segmented_line_to_destination(const_feedRate_t fr_mm_s, const float segment_size=LEVELED_SEGMENT_LENGTH) {

      const xyze_float_t diff = destination - current_position;

      // If the move is only in Z/E don't split up the move
      if (!diff.x && !diff.y) {
        planner.buffer_line(destination, fr_mm_s);
        return;
      }

      // Get the linear distance in XYZ
      #if HAS_ROTATIONAL_AXES
        bool cartes_move = true;
      #endif
      float cartesian_mm = get_move_distance(diff OPTARG(HAS_ROTATIONAL_AXES, cartes_move));

      // If the move is very short, check the E move distance
      TERN_(HAS_EXTRUDERS, if (UNEAR_ZERO(cartesian_mm)) cartesian_mm = ABS(diff.e));

      // No E move either? Game over.
      if (UNEAR_ZERO(cartesian_mm)) return;

      // The length divided by the segment size
      // At least one segment is required
      uint16_t segments = cartesian_mm / segment_size;
      NOLESS(segments, 1U);

      // The approximate length of each segment
      const float inv_segments = 1.0f / float(segments);
      const xyze_float_t segment_distance = diff * inv_segments;

      // Add hints to help optimize the move
      PlannerHints hints(cartesian_mm * inv_segments);
      TERN_(HAS_ROTATIONAL_AXES, hints.cartesian_move = cartes_move);
      TERN_(FEEDRATE_SCALING, hints.inv_duration = scaled_fr_mm_s / hints.millimeters);

      //SERIAL_ECHOPGM("mm=", cartesian_mm);
      //SERIAL_ECHOLNPGM(" segments=", segments);
      //SERIAL_ECHOLNPGM(" segment_mm=", hints.millimeters);

      // Get the raw current position as starting point
      xyze_pos_t raw = current_position;

      // Calculate and execute the segments
      millis_t next_idle_ms = millis() + 200UL;
      while (--segments) {
        segment_idle(next_idle_ms);
        raw += segment_distance;
        if (!planner.buffer_line(raw, fr_mm_s, active_extruder, hints))
          break;
      }

      // Since segment_distance is only approximate,
      // the final move must be to the exact destination.
      planner.buffer_line(destination, fr_mm_s, active_extruder, hints);
    }

  #endif // SEGMENT_LEVELED_MOVES && !AUTO_BED_LEVELING_UBL

  /**
   * Prepare a linear move in a Cartesian setup.
   *
   * When a mesh-based leveling system is active, moves are segmented
   * according to the configuration of the leveling system.
   *
   * Return true if 'current_position' was set to 'destination'
   */
  inline bool line_to_destination_cartesian() {
    const float scaled_fr_mm_s = MMS_SCALED(feedrate_mm_s);
    #if HAS_MESH
      if (planner.leveling_active && planner.leveling_active_at_z(destination.z)) {
        #if ENABLED(AUTO_BED_LEVELING_UBL)
          #if UBL_SEGMENTED
            return bedlevel.line_to_destination_segmented(scaled_fr_mm_s);
          #else
            bedlevel.line_to_destination_cartesian(scaled_fr_mm_s, active_extruder); // UBL's motion routine needs to know about
            return true;                                                             // all moves, including Z-only moves.
          #endif
        #elif ENABLED(SEGMENT_LEVELED_MOVES)
          segmented_line_to_destination(scaled_fr_mm_s);
          return false; // caller will update current_position
        #else
          /**
           * For MBL and ABL-BILINEAR only segment moves when X or Y are involved.
           * Otherwise fall through to do a direct single move.
           */
          if (xy_pos_t(current_position) != xy_pos_t(destination)) {
            #if ENABLED(MESH_BED_LEVELING)
              bedlevel.line_to_destination(scaled_fr_mm_s);
            #elif ENABLED(AUTO_BED_LEVELING_BILINEAR)
              bedlevel.line_to_destination(scaled_fr_mm_s);
            #endif
            return true;
          }
        #endif
      }
    #endif // HAS_MESH

    planner.buffer_line(destination, scaled_fr_mm_s);
    return false; // caller will update current_position
  }

#endif // !IS_KINEMATIC

#if HAS_DUPLICATION_MODE
  bool extruder_duplication_enabled;
  #if ENABLED(MULTI_NOZZLE_DUPLICATION)
    uint8_t duplication_e_mask; // = 0
  #endif
#endif

#if ENABLED(DUAL_X_CARRIAGE)

  DualXMode dual_x_carriage_mode         = DEFAULT_DUAL_X_CARRIAGE_MODE;
  float inactive_extruder_x              = X2_MAX_POS,                    // Used in mode 0 & 1
        duplicate_extruder_x_offset      = DEFAULT_DUPLICATION_X_OFFSET;  // Used in mode 2 & 3
  xyz_pos_t raised_parked_position;                                       // Used in mode 1
  bool active_extruder_parked            = false;                         // Used in mode 1, 2 & 3
  millis_t delayed_move_time             = 0;                             // Used in mode 1
  celsius_t duplicate_extruder_temp_offset = 0;                           // Used in mode 2 & 3
  bool idex_mirrored_mode                = false;                         // Used in mode 3

  float x_home_pos(const uint8_t extruder) {
    if (extruder == 0) return X_HOME_POS;

    /**
     * In dual carriage mode the extruder offset provides an override of the
     * second X-carriage position when homed - otherwise X2_HOME_POS is used.
     * This allows soft recalibration of the second extruder home position
     * (with M218 T1 Xn) without firmware reflash.
     */
    return hotend_offset[1].x > 0 ? hotend_offset[1].x : X2_HOME_POS;
  }

  void idex_set_mirrored_mode(const bool mirr) {
    idex_mirrored_mode = mirr;
    stepper.apply_directions();
  }

  void set_duplication_enabled(const bool dupe, const int8_t tool_index/*=-1*/) {
    extruder_duplication_enabled = dupe;
    if (tool_index >= 0) active_extruder = tool_index;
    stepper.apply_directions();
  }

  void idex_set_parked(const bool park/*=true*/) {
    delayed_move_time = 0;
    active_extruder_parked = park;
    if (park) raised_parked_position = current_position;  // Remember current raised toolhead position for use by unpark
  }

  /**
   * Prepare a linear move in a dual X axis setup
   *
   * Return true if current_position[] was set to destination[]
   */
  inline bool dual_x_carriage_unpark() {
    if (active_extruder_parked) {
      switch (dual_x_carriage_mode) {

        case DXC_FULL_CONTROL_MODE: break;

        case DXC_AUTO_PARK_MODE: {
          if (current_position.e == destination.e) {
            // This is a travel move (with no extrusion)
            // Skip it, but keep track of the current position
            // (so it can be used as the start of the next non-travel move)
            if (delayed_move_time != 0xFFFFFFFFUL) {
              current_position = destination;
              NOLESS(raised_parked_position.z, destination.z);
              delayed_move_time = millis() + 1000UL;
              return true;
            }
          }
          //
          // Un-park the active extruder
          //
          const feedRate_t fr_zfast = planner.settings.max_feedrate_mm_s[Z_AXIS];
          //  1. Move to the raised parked XYZ. Presumably the tool is already at XY.
          xyze_pos_t raised = raised_parked_position; raised.e = current_position.e;
          if (planner.buffer_line(raised, fr_zfast)) {
            //  2. Move to the current native XY and raised Z. Presumably this is a null move.
            xyze_pos_t curpos = current_position; curpos.z = raised_parked_position.z;
            if (planner.buffer_line(curpos, PLANNER_XY_FEEDRATE_MM_S)) {
              //  3. Lower Z back down
              line_to_current_position(fr_zfast);
            }
          }
          stepper.apply_directions();

          idex_set_parked(false);
          if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("idex_set_parked(false)");
        } break;

        case DXC_MIRRORED_MODE:
        case DXC_DUPLICATION_MODE:
          if (active_extruder == 0) {
            set_duplication_enabled(false); // Clear stale duplication state
            // Restore planner to parked head (T1) X position
            float x0_pos = current_position.x;
            xyze_pos_t pos_now = current_position;
            pos_now.x = inactive_extruder_x;
            planner.set_position_mm(pos_now);

            // Keep the same X or add the duplication X offset
            xyze_pos_t new_pos = pos_now;
            if (dual_x_carriage_mode == DXC_DUPLICATION_MODE)
              new_pos.x = x0_pos + duplicate_extruder_x_offset;
            else
              new_pos.x = _MIN(X_BED_SIZE - x0_pos, X_MAX_POS);

            // Move duplicate extruder into the correct position
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Set planner X", inactive_extruder_x, " ... Line to X", new_pos.x);
            if (!planner.buffer_line(new_pos, planner.settings.max_feedrate_mm_s[X_AXIS], 1)) break;
            planner.synchronize();

            sync_plan_position();             // Extra sync for good measure
            set_duplication_enabled(true);    // Enable Duplication
            idex_set_parked(false);           // No longer parked
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("set_duplication_enabled(true)\nidex_set_parked(false)");
          }
          else if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Active extruder not 0");
          break;
      }
    }
    return false;
  }

#endif // DUAL_X_CARRIAGE

/**
 * Prepare a single move and get ready for the next one
 *
 * This may result in several calls to planner.buffer_line to
 * do smaller moves for DELTA, SCARA, mesh moves, etc.
 *
 * Make sure current_position.e and destination.e are good
 * before calling or cold/lengthy extrusion may get missed.
 *
 * Before exit, current_position is set to destination.
 */
void prepare_line_to_destination() {
  apply_motion_limits(destination);

  #if ANY(PREVENT_COLD_EXTRUSION, PREVENT_LENGTHY_EXTRUDE)

    if (!DEBUGGING(DRYRUN) && destination.e != current_position.e) {
      bool ignore_e = thermalManager.tooColdToExtrude(active_extruder);
      if (ignore_e) SERIAL_ECHO_MSG(STR_ERR_COLD_EXTRUDE_STOP);

      #if ENABLED(PREVENT_LENGTHY_EXTRUDE)
        const float e_delta = ABS(destination.e - current_position.e) * planner.e_factor[active_extruder];
        if (e_delta > (EXTRUDE_MAXLENGTH)) {
          #if ENABLED(MIXING_EXTRUDER)
            float collector[MIXING_STEPPERS];
            mixer.refresh_collector(1.0, mixer.get_current_vtool(), collector);
            MIXER_STEPPER_LOOP(e) {
              if (e_delta * collector[e] > (EXTRUDE_MAXLENGTH)) {
                ignore_e = true;
                SERIAL_ECHO_MSG(STR_ERR_LONG_EXTRUDE_STOP);
                break;
              }
            }
          #else
            ignore_e = true;
            SERIAL_ECHO_MSG(STR_ERR_LONG_EXTRUDE_STOP);
          #endif
        }
      #endif

      if (ignore_e) {
        current_position.e = destination.e;       // Behave as if the E move really took place
        planner.set_e_position_mm(destination.e); // Prevent the planner from complaining too
      }
    }

  #endif // PREVENT_COLD_EXTRUSION || PREVENT_LENGTHY_EXTRUDE

  if (TERN0(DUAL_X_CARRIAGE, dual_x_carriage_unpark())) return;

  if (
    #if UBL_SEGMENTED
      #if IS_KINEMATIC // UBL using Kinematic / Cartesian cases as a workaround for now.
        bedlevel.line_to_destination_segmented(MMS_SCALED(feedrate_mm_s))
      #else
        line_to_destination_cartesian()
      #endif
    #elif IS_KINEMATIC
      line_to_destination_kinematic()
    #else
      line_to_destination_cartesian()
    #endif
  ) return;

  current_position = destination;
}

#if HAS_ENDSTOPS

  main_axes_bits_t axes_homed, axes_trusted; // = 0

  main_axes_bits_t axes_should_home(main_axes_bits_t axis_bits/*=main_axes_mask*/) {
    auto set_should = [](main_axes_bits_t &b, AxisEnum a) {
      if (TEST(b, a) && TERN(HOME_AFTER_DEACTIVATE, axis_is_trusted, axis_was_homed)(a))
        CBI(b, a);
    };
    // Clear test bits that are trusted
    NUM_AXIS_CODE(
      set_should(axis_bits, X_AXIS), set_should(axis_bits, Y_AXIS), set_should(axis_bits, Z_AXIS),
      set_should(axis_bits, I_AXIS), set_should(axis_bits, J_AXIS), set_should(axis_bits, K_AXIS),
      set_should(axis_bits, U_AXIS), set_should(axis_bits, V_AXIS), set_should(axis_bits, W_AXIS)
    );
    return axis_bits;
  }

  bool homing_needed_error(main_axes_bits_t axis_bits/*=main_axes_mask*/) {
    if (!(axis_bits &= axes_should_home(axis_bits))) return false;

    char all_axes[] = STR_AXES_MAIN, need[NUM_AXES + 1];
    uint8_t n = 0;
    LOOP_NUM_AXES(i) if (TEST(axis_bits, i)) need[n++] = all_axes[i];
    need[n] = '\0';

    SString<30> msg;
    msg.setf(GET_EN_TEXT_F(MSG_HOME_FIRST), need);
    SERIAL_ECHO_START();
    msg.echoln();

    ui.status_printf(0, GET_TEXT_F(MSG_HOME_FIRST), need);
    return true;
  }

  /**
   * Homing bump feedrate (mm/s)
   */
  feedRate_t get_homing_bump_feedrate(const AxisEnum axis) {
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS) return MMM_TO_MMS(Z_PROBE_FEEDRATE_SLOW);
    #endif
    static const uint8_t homing_bump_divisor[] PROGMEM = HOMING_BUMP_DIVISOR;
    uint8_t hbd = pgm_read_byte(&homing_bump_divisor[axis]);
    if (hbd < 1) {
      hbd = 10;
      SERIAL_ECHO_MSG("Warning: Homing Bump Divisor < 1");
    }
    return homing_feedrate(axis) / float(hbd);
  }

  #if ENABLED(SENSORLESS_HOMING)
    /**
     * Set sensorless homing if the axis has it, accounting for Core Kinematics.
     */
    sensorless_t start_sensorless_homing_per_axis(const AxisEnum axis) {
      sensorless_t stealth_states { false };

      switch (axis) {
        default: break;
        #if X_SENSORLESS
          case X_AXIS:
            stealth_states.x = tmc_enable_stallguard(stepperX);
            TERN_(X2_SENSORLESS, stealth_states.x2 = tmc_enable_stallguard(stepperX2));
            #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX) && Y_SENSORLESS
              stealth_states.y = tmc_enable_stallguard(stepperY);
            #elif CORE_IS_XZ && Z_SENSORLESS
              stealth_states.z = tmc_enable_stallguard(stepperZ);
            #endif
            break;
        #endif
        #if Y_SENSORLESS
          case Y_AXIS:
            stealth_states.y = tmc_enable_stallguard(stepperY);
            TERN_(Y2_SENSORLESS, stealth_states.y2 = tmc_enable_stallguard(stepperY2));
            #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX) && X_SENSORLESS
              stealth_states.x = tmc_enable_stallguard(stepperX);
            #elif CORE_IS_YZ && Z_SENSORLESS
              stealth_states.z = tmc_enable_stallguard(stepperZ);
            #endif
            break;
        #endif
        #if Z_SENSORLESS
          case Z_AXIS:
            stealth_states.z = tmc_enable_stallguard(stepperZ);
            TERN_(Z2_SENSORLESS, stealth_states.z2 = tmc_enable_stallguard(stepperZ2));
            TERN_(Z3_SENSORLESS, stealth_states.z3 = tmc_enable_stallguard(stepperZ3));
            TERN_(Z4_SENSORLESS, stealth_states.z4 = tmc_enable_stallguard(stepperZ4));
            #if CORE_IS_XZ && X_SENSORLESS
              stealth_states.x = tmc_enable_stallguard(stepperX);
            #elif CORE_IS_YZ && Y_SENSORLESS
              stealth_states.y = tmc_enable_stallguard(stepperY);
            #endif
            break;
        #endif
        #if I_SENSORLESS
          case I_AXIS: stealth_states.i = tmc_enable_stallguard(stepperI); break;
        #endif
        #if J_SENSORLESS
          case J_AXIS: stealth_states.j = tmc_enable_stallguard(stepperJ); break;
        #endif
        #if K_SENSORLESS
          case K_AXIS: stealth_states.k = tmc_enable_stallguard(stepperK); break;
        #endif
        #if U_SENSORLESS
          case U_AXIS: stealth_states.u = tmc_enable_stallguard(stepperU); break;
        #endif
        #if V_SENSORLESS
          case V_AXIS: stealth_states.v = tmc_enable_stallguard(stepperV); break;
        #endif
        #if W_SENSORLESS
          case W_AXIS: stealth_states.w = tmc_enable_stallguard(stepperW); break;
        #endif
      }

      switch (axis) {
        #if X_SPI_SENSORLESS
          case X_AXIS: endstops.tmc_spi_homing.x = true; break;
        #endif
        #if Y_SPI_SENSORLESS
          case Y_AXIS: endstops.tmc_spi_homing.y = true; break;
        #endif
        #if Z_SPI_SENSORLESS
          case Z_AXIS: endstops.tmc_spi_homing.z = true; break;
        #endif
        #if I_SPI_SENSORLESS
          case I_AXIS: endstops.tmc_spi_homing.i = true; break;
        #endif
        #if J_SPI_SENSORLESS
          case J_AXIS: endstops.tmc_spi_homing.j = true; break;
        #endif
        #if K_SPI_SENSORLESS
          case K_AXIS: endstops.tmc_spi_homing.k = true; break;
        #endif
        #if U_SPI_SENSORLESS
          case U_AXIS: endstops.tmc_spi_homing.u = true; break;
        #endif
        #if V_SPI_SENSORLESS
          case V_AXIS: endstops.tmc_spi_homing.v = true; break;
        #endif
        #if W_SPI_SENSORLESS
          case W_AXIS: endstops.tmc_spi_homing.w = true; break;
        #endif
        default: break;
      }

      TERN_(IMPROVE_HOMING_RELIABILITY, sg_guard_period = millis() + default_sg_guard_duration);

      return stealth_states;
    }

    void end_sensorless_homing_per_axis(const AxisEnum axis, sensorless_t enable_stealth) {
      switch (axis) {
        default: break;
        #if X_SENSORLESS
          case X_AXIS:
            tmc_disable_stallguard(stepperX, enable_stealth.x);
            TERN_(X2_SENSORLESS, tmc_disable_stallguard(stepperX2, enable_stealth.x2));
            #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX) && Y_SENSORLESS
              tmc_disable_stallguard(stepperY, enable_stealth.y);
            #elif CORE_IS_XZ && Z_SENSORLESS
              tmc_disable_stallguard(stepperZ, enable_stealth.z);
            #endif
            break;
        #endif
        #if Y_SENSORLESS
          case Y_AXIS:
            tmc_disable_stallguard(stepperY, enable_stealth.y);
            TERN_(Y2_SENSORLESS, tmc_disable_stallguard(stepperY2, enable_stealth.y2));
            #if ANY(CORE_IS_XY, MARKFORGED_XY, MARKFORGED_YX) && X_SENSORLESS
              tmc_disable_stallguard(stepperX, enable_stealth.x);
            #elif CORE_IS_YZ && Z_SENSORLESS
              tmc_disable_stallguard(stepperZ, enable_stealth.z);
            #endif
            break;
        #endif
        #if Z_SENSORLESS
          case Z_AXIS:
            tmc_disable_stallguard(stepperZ, enable_stealth.z);
            TERN_(Z2_SENSORLESS, tmc_disable_stallguard(stepperZ2, enable_stealth.z2));
            TERN_(Z3_SENSORLESS, tmc_disable_stallguard(stepperZ3, enable_stealth.z3));
            TERN_(Z4_SENSORLESS, tmc_disable_stallguard(stepperZ4, enable_stealth.z4));
            #if CORE_IS_XZ && X_SENSORLESS
              tmc_disable_stallguard(stepperX, enable_stealth.x);
            #elif CORE_IS_YZ && Y_SENSORLESS
              tmc_disable_stallguard(stepperY, enable_stealth.y);
            #endif
            break;
        #endif
        #if I_SENSORLESS
          case I_AXIS: tmc_disable_stallguard(stepperI, enable_stealth.i); break;
        #endif
        #if J_SENSORLESS
          case J_AXIS: tmc_disable_stallguard(stepperJ, enable_stealth.j); break;
        #endif
        #if K_SENSORLESS
          case K_AXIS: tmc_disable_stallguard(stepperK, enable_stealth.k); break;
        #endif
        #if U_SENSORLESS
          case U_AXIS: tmc_disable_stallguard(stepperU, enable_stealth.u); break;
        #endif
        #if V_SENSORLESS
          case V_AXIS: tmc_disable_stallguard(stepperV, enable_stealth.v); break;
        #endif
        #if W_SENSORLESS
          case W_AXIS: tmc_disable_stallguard(stepperW, enable_stealth.w); break;
        #endif
      }

      switch (axis) {
        #if X_SPI_SENSORLESS
          case X_AXIS: endstops.tmc_spi_homing.x = false; break;
        #endif
        #if Y_SPI_SENSORLESS
          case Y_AXIS: endstops.tmc_spi_homing.y = false; break;
        #endif
        #if Z_SPI_SENSORLESS
          case Z_AXIS: endstops.tmc_spi_homing.z = false; break;
        #endif
        #if I_SPI_SENSORLESS
          case I_AXIS: endstops.tmc_spi_homing.i = false; break;
        #endif
        #if J_SPI_SENSORLESS
          case J_AXIS: endstops.tmc_spi_homing.j = false; break;
        #endif
        #if K_SPI_SENSORLESS
          case K_AXIS: endstops.tmc_spi_homing.k = false; break;
        #endif
        #if U_SPI_SENSORLESS
          case U_AXIS: endstops.tmc_spi_homing.u = false; break;
        #endif
        #if V_SPI_SENSORLESS
          case V_AXIS: endstops.tmc_spi_homing.v = false; break;
        #endif
        #if W_SPI_SENSORLESS
          case W_AXIS: endstops.tmc_spi_homing.w = false; break;
        #endif
        default: break;
      }
    }

  #endif // SENSORLESS_HOMING

  /**
   * Home an individual linear axis
   */
  void do_homing_move(const AxisEnum axis, const float distance, const feedRate_t fr_mm_s=0.0, const bool final_approach=true) {
    DEBUG_SECTION(log_move, "do_homing_move", DEBUGGING(LEVELING));

    const feedRate_t home_fr_mm_s = fr_mm_s ?: homing_feedrate(axis);

    if (DEBUGGING(LEVELING)) {
      DEBUG_ECHOPGM("...(", C(AXIS_CHAR(axis)), ", ", distance, ", ");
      if (fr_mm_s)
        DEBUG_ECHO(fr_mm_s);
      else
        DEBUG_ECHOPGM("[", home_fr_mm_s, "]");
      DEBUG_ECHOLNPGM(")");
    }

    // Only do some things when moving towards an endstop
    const int8_t axis_home_dir = TERN0(DUAL_X_CARRIAGE, axis == X_AXIS)
                  ? TOOL_X_HOME_DIR(active_extruder) : home_dir(axis);
    const bool is_home_dir = (axis_home_dir > 0) == (distance > 0);

    #if ENABLED(SENSORLESS_HOMING)
      sensorless_t stealth_states;
    #endif

    if (is_home_dir) {

      if (TERN0(HOMING_Z_WITH_PROBE, axis == Z_AXIS)) {
        #if ALL(HAS_HEATED_BED, WAIT_FOR_BED_HEATER)
          // Wait for bed to heat back up between probing points
          thermalManager.wait_for_bed_heating();
        #endif

        #if ALL(HAS_HOTEND, WAIT_FOR_HOTEND)
          // Wait for the hotend to heat back up between probing points
          thermalManager.wait_for_hotend_heating(active_extruder);
        #endif

        TERN_(HAS_QUIET_PROBING, if (final_approach) probe.set_probing_paused(true));
      }

      // Disable stealthChop if used. Enable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        stealth_states = start_sensorless_homing_per_axis(axis);
        #if SENSORLESS_STALLGUARD_DELAY
          safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
        #endif
      #endif
    }

    #if ANY(MORGAN_SCARA, MP_SCARA)
      // Tell the planner the axis is at 0
      current_position[axis] = 0;
      sync_plan_position();
      current_position[axis] = distance;
      line_to_current_position(home_fr_mm_s);
    #else
      // Get the ABC or XYZ positions in mm
      abce_pos_t target = planner.get_axis_positions_mm();

      target[axis] = 0;                         // Set the single homing axis to 0
      planner.set_machine_position_mm(target);  // Update the machine position

      #if HAS_DIST_MM_ARG
        const xyze_float_t cart_dist_mm{0};
      #endif

      // Set delta/cartesian axes directly
      target[axis] = distance;                  // The move will be towards the endstop
      planner.buffer_segment(target OPTARG(HAS_DIST_MM_ARG, cart_dist_mm), home_fr_mm_s, active_extruder);
    #endif

    planner.synchronize();

    if (is_home_dir) {

      #if HOMING_Z_WITH_PROBE && HAS_QUIET_PROBING
        if (axis == Z_AXIS && final_approach) probe.set_probing_paused(false);
      #endif

      endstops.validate_homing_move();

      // Re-enable stealthChop if used. Disable diag1 pin on driver.
      #if ENABLED(SENSORLESS_HOMING)
        end_sensorless_homing_per_axis(axis, stealth_states);
        #if SENSORLESS_STALLGUARD_DELAY
          safe_delay(SENSORLESS_STALLGUARD_DELAY); // Short delay needed to settle
        #endif
      #endif
    }
  }

  /**
   * Set an axis to be unhomed. (Unless we are on a machine - e.g. a cheap Chinese CNC machine -
   * that has no endstops. Such machines should always be considered to be in a "known" and
   * "trusted" position).
   */
  void set_axis_never_homed(const AxisEnum axis) {
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM(">>> set_axis_never_homed(", C(AXIS_CHAR(axis)), ")");

    set_axis_untrusted(axis);
    set_axis_unhomed(axis);

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("<<< set_axis_never_homed(", C(AXIS_CHAR(axis)), ")");

    TERN_(I2C_POSITION_ENCODERS, I2CPEM.unhomed(axis));
  }

  #ifdef TMC_HOME_PHASE
    /**
     * Move the axis back to its home_phase if set and driver is capable (TMC)
     *
     * Improves homing repeatability by homing to stepper coil's nearest absolute
     * phase position. Trinamic drivers use a stepper phase table with 1024 values
     * spanning 4 full steps with 256 positions each (ergo, 1024 positions).
     */
    void backout_to_tmc_homing_phase(const AxisEnum axis) {
      const xyz_long_t home_phase = TMC_HOME_PHASE;

      // check if home phase is disabled for this axis.
      if (home_phase[axis] < 0) return;

      int16_t phasePerUStep,      // TMC µsteps(phase) per Marlin µsteps
              phaseCurrent,       // The TMC µsteps(phase) count of the current position
              effectorBackoutDir, // Direction in which the effector mm coordinates move away from endstop.
              stepperBackoutDir;  // Direction in which the TMC µstep count(phase) move away from endstop.

      #define PHASE_PER_MICROSTEP(N) (256 / _MAX(1, N##_MICROSTEPS))

      switch (axis) {
        #ifdef X_MICROSTEPS
          case X_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(X);
            phaseCurrent = stepperX.get_microstep_counter();
            effectorBackoutDir = -(X_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_X_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef Y_MICROSTEPS
          case Y_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(Y);
            phaseCurrent = stepperY.get_microstep_counter();
            effectorBackoutDir = -(Y_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_Y_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef Z_MICROSTEPS
          case Z_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(Z);
            phaseCurrent = stepperZ.get_microstep_counter();
            effectorBackoutDir = -(Z_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_Z_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef I_MICROSTEPS
          case I_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(I);
            phaseCurrent = stepperI.get_microstep_counter();
            effectorBackoutDir = -(I_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_I_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef J_MICROSTEPS
          case J_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(J);
            phaseCurrent = stepperJ.get_microstep_counter();
            effectorBackoutDir = -(J_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_J_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef K_MICROSTEPS
          case K_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(K);
            phaseCurrent = stepperK.get_microstep_counter();
            effectorBackoutDir = -(K_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_K_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef U_MICROSTEPS
          case U_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(U);
            phaseCurrent = stepperU.get_microstep_counter();
            effectorBackoutDir = -(U_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_U_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef V_MICROSTEPS
          case V_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(V);
            phaseCurrent = stepperV.get_microstep_counter();
            effectorBackoutDir = -(V_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_V_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        #ifdef W_MICROSTEPS
          case W_AXIS:
            phasePerUStep = PHASE_PER_MICROSTEP(W);
            phaseCurrent = stepperW.get_microstep_counter();
            effectorBackoutDir = -(W_HOME_DIR);
            stepperBackoutDir = TERN_(INVERT_W_DIR, -)(-effectorBackoutDir);
            break;
        #endif
        default: return;
      }

      // Phase distance to nearest home phase position when moving in the backout direction from endstop(may be negative).
      int16_t phaseDelta = (home_phase[axis] - phaseCurrent) * stepperBackoutDir;

      // Check if home distance within endstop assumed repeatability noise of .05mm and warn.
      if (ABS(phaseDelta) * planner.mm_per_step[axis] / phasePerUStep < 0.05f)
        SERIAL_ECHOLNPGM("Selected home phase ", home_phase[axis],
                         " too close to endstop trigger phase ", phaseCurrent,
                         ". Pick a different phase for ", C(AXIS_CHAR(axis)));

      // Skip to next if target position is behind current. So it only moves away from endstop.
      if (phaseDelta < 0) phaseDelta += 1024;

      // Convert TMC µsteps(phase) to whole Marlin µsteps to effector backout direction to mm
      const float mmDelta = int16_t(phaseDelta / phasePerUStep) * effectorBackoutDir * planner.mm_per_step[axis];

      // Optional debug messages
      if (DEBUGGING(LEVELING)) {
        DEBUG_ECHOLNPGM(
          "Endstop ", C(AXIS_CHAR(axis)), " hit at Phase:", phaseCurrent,
          " Delta:", phaseDelta, " Distance:", mmDelta
        );
      }

      if (mmDelta != 0) {
        // Retrace by the amount computed in mmDelta.
        do_homing_move(axis, mmDelta, get_homing_bump_feedrate(axis));
      }
    }
  #endif

  /**
   * Home an individual "raw axis" to its endstop.
   * This applies to XYZ on Cartesian and Core robots, and
   * to the individual ABC steppers on DELTA and SCARA.
   *
   * At the end of the procedure the axis is marked as
   * homed and the current position of that axis is updated.
   * Kinematic robots should wait till all axes are homed
   * before updating the current position.
   */

  void homeaxis(const AxisEnum axis) {

    #if ANY(MORGAN_SCARA, MP_SCARA)
      // Only Z homing (with probe) is permitted
      if (axis != Z_AXIS) { BUZZ(100, 880); return; }
    #else
      #define _CAN_HOME(A) (axis == _AXIS(A) && (ANY(A##_SPI_SENSORLESS, HAS_##A##_STATE) || TERN0(HOMING_Z_WITH_PROBE, _AXIS(A) == Z_AXIS)))
      #define _ANDCANT(N) && !_CAN_HOME(N)
      if (true MAIN_AXIS_MAP(_ANDCANT)) return;
    #endif

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM(">>> homeaxis(", C(AXIS_CHAR(axis)), ")");

    const int axis_home_dir = TERN0(DUAL_X_CARRIAGE, axis == X_AXIS)
                ? TOOL_X_HOME_DIR(active_extruder) : home_dir(axis);

    //
    // Homing Z with a probe? Raise Z (maybe) and deploy the Z probe.
    // Return early if probe deployment fails.
    //
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS && probe.deploy()) { probe.stow(); return; }
    #endif

    // Set flags for X, Y, Z motor locking
    #if HAS_EXTRA_ENDSTOPS
      switch (axis) {
        TERN_(X_DUAL_ENDSTOPS, case X_AXIS:)
        TERN_(Y_DUAL_ENDSTOPS, case Y_AXIS:)
        TERN_(Z_MULTI_ENDSTOPS, case Z_AXIS:)
          stepper.set_separate_multi_axis(true);
        default: break;
      }
    #endif

    //
    // Deploy BLTouch or tare the probe just before probing
    //
    #if HOMING_Z_WITH_PROBE
      if (axis == Z_AXIS) {

        #if ENABLED(BLTOUCH)
          // BLTouch was deployed above, but get the alarm state.
          // Stow and return early if there is a deploy alarm.
          if (bltouch.deploy()) { bltouch.stow(); return; }
        #endif

        // Tare the probe. Stow and return early if it fails
        if (TERN0(PROBE_TARE, probe.tare())) { probe.stow(); return; }

        // Tell the Bed Distance Sensor we're Z homing
        TERN_(BD_SENSOR, bdl.config_state = BDS_HOMING_Z);
      }
    #endif

    //
    // Set a new current for the homed axis motor(s)
    //
    TERN_(HAS_HOMING_CURRENT, set_homing_current(axis));

    //
    // Back away to prevent an early sensorless trigger
    //
    #if DISABLED(DELTA) && defined(SENSORLESS_BACKOFF_MM)
      const xyz_float_t backoff = SENSORLESS_BACKOFF_MM;
      if ((TERN0(X_SENSORLESS, axis == X_AXIS) || TERN0(Y_SENSORLESS, axis == Y_AXIS) || TERN0(Z_SENSORLESS, axis == Z_AXIS) || TERN0(I_SENSORLESS, axis == I_AXIS) || TERN0(J_SENSORLESS, axis == J_AXIS) || TERN0(K_SENSORLESS, axis == K_AXIS)) && backoff[axis]) {
        const float backoff_length = -ABS(backoff[axis]) * axis_home_dir;
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Sensorless backoff: ", backoff_length, "mm");
        do_homing_move(axis, backoff_length, homing_feedrate(axis));
      }
    #endif

    //
    // Back away to prevent opposite endstop damage
    //
    #if !defined(SENSORLESS_BACKOFF_MM) && XY_COUNTERPART_BACKOFF_MM
      if (!(axis_was_homed(X_AXIS) || axis_was_homed(Y_AXIS)) && (axis == X_AXIS || axis == Y_AXIS)) {
        const AxisEnum opposite_axis = axis == X_AXIS ? Y_AXIS : X_AXIS;
        const float backoff_length = -ABS(XY_COUNTERPART_BACKOFF_MM) * home_dir(opposite_axis);
        do_homing_move(opposite_axis, backoff_length, homing_feedrate(opposite_axis));
      }
    #endif

    // Determine if a homing bump will be done and the bumps distance
    // When homing Z with probe respect probe clearance
    const bool use_probe_bump = TERN0(HOMING_Z_WITH_PROBE, axis == Z_AXIS && home_bump_mm(axis));
    const float bump = axis_home_dir * (
      use_probe_bump ? _MAX(TERN0(HOMING_Z_WITH_PROBE, Z_CLEARANCE_BETWEEN_PROBES), home_bump_mm(axis)) : home_bump_mm(axis)
    );

    //
    // Fast move towards endstop until triggered
    //
    const float move_length = 1.5f * max_length(TERN(DELTA, Z_AXIS, axis)) * axis_home_dir;
    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Home Fast: ", move_length, "mm");
    do_homing_move(axis, move_length, 0.0, !use_probe_bump);

    // If a second homing move is configured...
    if (bump) {
      #if ALL(HOMING_Z_WITH_PROBE, BLTOUCH)
        if (axis == Z_AXIS && !bltouch.high_speed_mode) bltouch.stow(); // Intermediate STOW (in LOW SPEED MODE)
      #endif

      // Move away from the endstop by the axis HOMING_BUMP_MM
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Move Away: ", -bump, "mm");
      do_homing_move(axis, -bump, TERN0(HOMING_Z_WITH_PROBE, (axis == Z_AXIS ? z_probe_fast_mm_s : 0)), false);

      #if ENABLED(DETECT_BROKEN_ENDSTOP)

        // Check for a broken endstop
        EndstopEnum es;
        switch (axis) {
          #define _ESCASE(A) case A##_AXIS: es = A##_ENDSTOP; break;
          MAIN_AXIS_MAP(_ESCASE)
          default: break;
        }

        #if ENABLED(DUAL_X_CARRIAGE)
          if (axis == X_AXIS && axis_home_dir > 0) {
            es = X_MAX;
            if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("DUAL_X_CARRIAGE: Homing to X_MAX");
          }
        #endif

        if (TEST(endstops.state(), es)) {
          SERIAL_ECHO_MSG("Bad ", C(AXIS_CHAR(axis)), " Endstop?");
          kill(GET_TEXT_F(MSG_KILL_HOMING_FAILED));
        }

      #endif // DETECT_BROKEN_ENDSTOP

      #if ALL(HOMING_Z_WITH_PROBE, BLTOUCH)
        if (axis == Z_AXIS && !bltouch.high_speed_mode && bltouch.deploy()) {
          bltouch.stow();
          return; // Intermediate DEPLOY (in LOW SPEED MODE)
        }
      #endif

      // Slow move towards endstop until triggered
      const float rebump = bump * 2;
      if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("Re-bump: ", rebump, "mm");
      do_homing_move(axis, rebump, get_homing_bump_feedrate(axis), true);
    }

    #if ALL(HOMING_Z_WITH_PROBE, BLTOUCH)
      if (axis == Z_AXIS) bltouch.stow(); // The final STOW
    #endif

    #if HAS_EXTRA_ENDSTOPS
      const bool pos_dir = axis_home_dir > 0;
      #if ENABLED(X_DUAL_ENDSTOPS)
        if (axis == X_AXIS) {
          const float adj = ABS(endstops.x2_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.x2_endstop_adj > 0) : (endstops.x2_endstop_adj < 0)) stepper.set_x_lock(true); else stepper.set_x2_lock(true);
            do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_x_lock(false);
            stepper.set_x2_lock(false);
          }
        }
      #endif
      #if ENABLED(Y_DUAL_ENDSTOPS)
        if (axis == Y_AXIS) {
          const float adj = ABS(endstops.y2_endstop_adj);
          if (adj) {
            if (pos_dir ? (endstops.y2_endstop_adj > 0) : (endstops.y2_endstop_adj < 0)) stepper.set_y_lock(true); else stepper.set_y2_lock(true);
            do_homing_move(axis, pos_dir ? -adj : adj);
            stepper.set_y_lock(false);
            stepper.set_y2_lock(false);
          }
        }
      #endif

      #if ENABLED(Z_MULTI_ENDSTOPS)
        if (axis == Z_AXIS) {

          #if NUM_Z_STEPPERS == 2

            const float adj = ABS(endstops.z2_endstop_adj);
            if (adj) {
              if (pos_dir ? (endstops.z2_endstop_adj > 0) : (endstops.z2_endstop_adj < 0)) stepper.set_z1_lock(true); else stepper.set_z2_lock(true);
              do_homing_move(axis, pos_dir ? -adj : adj);
              stepper.set_z1_lock(false);
              stepper.set_z2_lock(false);
            }

          #else

            // Handy arrays of stepper lock function pointers

            typedef void (*adjustFunc_t)(const bool);

            adjustFunc_t lock[] = {
              stepper.set_z1_lock, stepper.set_z2_lock, stepper.set_z3_lock
              #if NUM_Z_STEPPERS >= 4
                , stepper.set_z4_lock
              #endif
            };
            float adj[] = {
              0, endstops.z2_endstop_adj, endstops.z3_endstop_adj
              #if NUM_Z_STEPPERS >= 4
                , endstops.z4_endstop_adj
              #endif
            };

            adjustFunc_t tempLock;
            float tempAdj;

            // Manual bubble sort by adjust value
            if (adj[1] < adj[0]) {
              tempLock = lock[0], tempAdj = adj[0];
              lock[0] = lock[1], adj[0] = adj[1];
              lock[1] = tempLock, adj[1] = tempAdj;
            }
            if (adj[2] < adj[1]) {
              tempLock = lock[1], tempAdj = adj[1];
              lock[1] = lock[2], adj[1] = adj[2];
              lock[2] = tempLock, adj[2] = tempAdj;
            }
            #if NUM_Z_STEPPERS >= 4
              if (adj[3] < adj[2]) {
                tempLock = lock[2], tempAdj = adj[2];
                lock[2] = lock[3], adj[2] = adj[3];
                lock[3] = tempLock, adj[3] = tempAdj;
              }
              if (adj[2] < adj[1]) {
                tempLock = lock[1], tempAdj = adj[1];
                lock[1] = lock[2], adj[1] = adj[2];
                lock[2] = tempLock, adj[2] = tempAdj;
              }
            #endif
            if (adj[1] < adj[0]) {
              tempLock = lock[0], tempAdj = adj[0];
              lock[0] = lock[1], adj[0] = adj[1];
              lock[1] = tempLock, adj[1] = tempAdj;
            }

            if (pos_dir) {
              // normalize adj to smallest value and do the first move
              (*lock[0])(true);
              do_homing_move(axis, adj[1] - adj[0]);
              // lock the second stepper for the final correction
              (*lock[1])(true);
              do_homing_move(axis, adj[2] - adj[1]);
              #if NUM_Z_STEPPERS >= 4
                // lock the third stepper for the final correction
                (*lock[2])(true);
                do_homing_move(axis, adj[3] - adj[2]);
              #endif
            }
            else {
              #if NUM_Z_STEPPERS >= 4
                (*lock[3])(true);
                do_homing_move(axis, adj[2] - adj[3]);
              #endif
              (*lock[2])(true);
              do_homing_move(axis, adj[1] - adj[2]);
              (*lock[1])(true);
              do_homing_move(axis, adj[0] - adj[1]);
            }

            stepper.set_z1_lock(false);
            stepper.set_z2_lock(false);
            stepper.set_z3_lock(false);
            #if NUM_Z_STEPPERS >= 4
              stepper.set_z4_lock(false);
            #endif

          #endif
        }
      #endif

      // Reset flags for X, Y, Z motor locking
      switch (axis) {
        default: break;
        TERN_(X_DUAL_ENDSTOPS, case X_AXIS:)
        TERN_(Y_DUAL_ENDSTOPS, case Y_AXIS:)
        TERN_(Z_MULTI_ENDSTOPS, case Z_AXIS:)
          stepper.set_separate_multi_axis(false);
      }

    #endif // HAS_EXTRA_ENDSTOPS

    #ifdef TMC_HOME_PHASE
      // move back to homing phase if configured and capable
      backout_to_tmc_homing_phase(axis);
    #endif

    #if IS_SCARA

      set_axis_is_at_home(axis);
      sync_plan_position();

    #elif ENABLED(DELTA)

      // Delta has already moved all three towers up in G28
      // so here it re-homes each tower in turn.
      // Delta homing treats the axes as normal linear axes.

      const float adjDistance = delta_endstop_adj[axis],
                  minDistance = (MIN_STEPS_PER_SEGMENT) * planner.mm_per_step[axis];

      // Retrace by the amount specified in delta_endstop_adj if more than min steps.
      if (adjDistance * (Z_HOME_DIR) < 0 && ABS(adjDistance) > minDistance) { // away from endstop, more than min distance
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("adjDistance:", adjDistance);
        do_homing_move(axis, adjDistance, get_homing_bump_feedrate(axis));
      }

    #else // CARTESIAN / CORE / MARKFORGED_XY / MARKFORGED_YX

      set_axis_is_at_home(axis);
      sync_plan_position();

      destination[axis] = current_position[axis];

      if (DEBUGGING(LEVELING)) DEBUG_POS("> AFTER set_axis_is_at_home", current_position);

    #endif

    #if ALL(BD_SENSOR, HOMING_Z_WITH_PROBE)
      if (axis == Z_AXIS) bdl.config_state = BDS_IDLE;
    #endif

    // Put away the Z probe. Return early if it fails.
    if (TERN0(HOMING_Z_WITH_PROBE, axis == Z_AXIS && probe.stow())) return;

    #if DISABLED(DELTA) && defined(HOMING_BACKOFF_POST_MM)
      const xyz_float_t endstop_backoff = HOMING_BACKOFF_POST_MM;
      if (endstop_backoff[axis]) {
        current_position[axis] -= ABS(endstop_backoff[axis]) * axis_home_dir;
        line_to_current_position(TERN_(HOMING_Z_WITH_PROBE, (axis == Z_AXIS) ? z_probe_fast_mm_s :) homing_feedrate(axis));

        #if ENABLED(SENSORLESS_HOMING)
          planner.synchronize();
          if (false
            #ifdef NORMAL_AXIS
              || axis != NORMAL_AXIS
            #endif
          ) safe_delay(200);  // Short delay to allow belts to spring back
        #endif
      }
    #endif

    // Clear retracted status if homing the Z axis
    #if ENABLED(FWRETRACT)
      if (axis == Z_AXIS) fwretract.current_hop = 0.0;
    #endif

    if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("<<< homeaxis(", C(AXIS_CHAR(axis)), ")");

    //
    // Restore axis motor(s) current after homing
    //
    TERN_(HAS_HOMING_CURRENT, restore_homing_current(axis));

  } // homeaxis()

#endif // HAS_ENDSTOPS

/**
 * Set an axis' current position to its home position (after homing).
 *
 * For Core and Cartesian robots this applies one-to-one when an
 * individual axis has been homed.
 *
 * DELTA should wait until all homing is done before setting the XYZ
 * current_position to home, because homing is a single operation.
 * In the case where the axis positions are trusted and previously
 * homed, DELTA could home to X or Y individually by moving either one
 * to the center. However, homing Z always homes XY and Z.
 *
 * SCARA should wait until all XY homing is done before setting the XY
 * current_position to home, because neither X nor Y is at home until
 * both are at home. Z can however be homed individually.
 *
 * Callers must sync the planner position after calling this!
 */
void set_axis_is_at_home(const AxisEnum axis) {
  if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM(">>> set_axis_is_at_home(", C(AXIS_CHAR(axis)), ")");

  set_axis_trusted(axis);
  set_axis_homed(axis);

  #if ENABLED(DUAL_X_CARRIAGE)
    if (axis == X_AXIS && (active_extruder == 1 || dual_x_carriage_mode == DXC_DUPLICATION_MODE)) {
      current_position.x = SUM_TERN(HAS_HOME_OFFSET, x_home_pos(active_extruder), home_offset.x);
      return;
    }
  #endif

  #if ANY(MORGAN_SCARA, AXEL_TPARA)
    scara_set_axis_is_at_home(axis);
  #elif ENABLED(DELTA)
    current_position[axis] = (axis == Z_AXIS) ? DIFF_TERN(HAS_BED_PROBE, delta_height, probe.offset.z) : base_home_pos(axis);
  #else
    current_position[axis] = SUM_TERN(HAS_HOME_OFFSET, base_home_pos(axis), home_offset[axis]);
  #endif

  /**
   * Z Probe Z Homing? Account for the probe's Z offset.
   */
  #if HAS_BED_PROBE && Z_HOME_TO_MIN
    if (axis == Z_AXIS) {
      #if HOMING_Z_WITH_PROBE
        #if ENABLED(BD_SENSOR)
          safe_delay(100);
          current_position.z = bdl.read();
        #else
          current_position.z -= probe.offset.z;
        #endif
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("*** Z homed with PROBE" TERN_(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN, " (Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)") " ***\n> (M851 Z", probe.offset.z, ")");
      #else
        if (DEBUGGING(LEVELING)) DEBUG_ECHOLNPGM("*** Z homed to ENDSTOP ***");
      #endif
    }
  #endif

  TERN_(I2C_POSITION_ENCODERS, I2CPEM.homed(axis));

  TERN_(BABYSTEP_DISPLAY_TOTAL, babystep.reset_total(axis));

  TERN_(HAS_WORKSPACE_OFFSET, workspace_offset[axis] = 0);

  if (DEBUGGING(LEVELING)) {
    #if HAS_HOME_OFFSET
      DEBUG_ECHOLNPGM("> home_offset[", C(AXIS_CHAR(axis)), "] = ", home_offset[axis]);
    #endif
    DEBUG_POS("", current_position);
    DEBUG_ECHOLNPGM("<<< set_axis_is_at_home(", C(AXIS_CHAR(axis)), ")");
  }
}

#if HAS_HOME_OFFSET
  /**
   * Set the home offset for an axis.
   */
  void set_home_offset(const AxisEnum axis, const_float_t v) {
    home_offset[axis] = v;
  }
#endif
