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
#pragma once

/**
 * Conditionals-3-etc.h
 * Conditionals that need to be set before Configuration_adv.h or pins.h
 */

//========================================================
// Get requirements for the benefit of IntelliSense, etc.
//
#include "Conditionals-2-LCD.h"
//========================================================

#ifndef STRING_CONFIG_H_AUTHOR
  #define STRING_CONFIG_H_AUTHOR "(anonymous)"
#endif

//
// Remove irrelevant Configuration.h settings
//

// Clean up unused temperature sensors and sub-options

#if !TEMP_SENSOR_0
  #undef TEMP_SENSOR_0
#endif
#if !TEMP_SENSOR_1
  #undef TEMP_SENSOR_1
#endif
#if !TEMP_SENSOR_2
  #undef TEMP_SENSOR_2
#endif
#if !TEMP_SENSOR_3
  #undef TEMP_SENSOR_3
#endif
#if !TEMP_SENSOR_4
  #undef TEMP_SENSOR_4
#endif
#if !TEMP_SENSOR_5
  #undef TEMP_SENSOR_5
#endif
#if !TEMP_SENSOR_6
  #undef TEMP_SENSOR_6
#endif
#if !TEMP_SENSOR_7
  #undef TEMP_SENSOR_7
#endif

#if !TEMP_SENSOR_BED
  #undef TEMP_SENSOR_BED
  #undef THERMAL_PROTECTION_BED
  #undef MAX_BED_POWER
  #undef PREHEAT_1_TEMP_BED
  #undef PREHEAT_2_TEMP_BED
#endif
#if !TEMP_SENSOR_CHAMBER
  #undef TEMP_SENSOR_CHAMBER
  #undef THERMAL_PROTECTION_CHAMBER
  #undef MAX_CHAMBER_POWER
  #undef PREHEAT_1_TEMP_CHAMBER
  #undef PREHEAT_2_TEMP_CHAMBER
#endif
#if !TEMP_SENSOR_COOLER
  #undef TEMP_SENSOR_COOLER
  #undef THERMAL_PROTECTION_COOLER
#endif
#if !TEMP_SENSOR_PROBE
  #undef TEMP_SENSOR_PROBE
#endif
#if !TEMP_SENSOR_REDUNDANT
  #undef TEMP_SENSOR_REDUNDANT
#endif
#if !TEMP_SENSOR_BOARD
  #undef TEMP_SENSOR_BOARD
#endif
#if !TEMP_SENSOR_SOC
  #undef TEMP_SENSOR_SOC
#endif
#if !SOFT_PWM_SCALE
  #undef SOFT_PWM_SCALE
#endif

/**
 * Disable unused SINGLENOZZLE sub-options
 */
#if DISABLED(SINGLENOZZLE)
  #undef SINGLENOZZLE_STANDBY_TEMP
#endif
#if !ALL(HAS_FAN, SINGLENOZZLE)
  #undef SINGLENOZZLE_STANDBY_FAN
#endif

// No inactive extruders with SWITCHING_NOZZLE or Průša MMU1 or just 1 E stepper exists
#if HAS_SWITCHING_NOZZLE || HAS_PRUSA_MMU1 || E_STEPPERS < 2
  #undef DISABLE_OTHER_EXTRUDERS
#endif

// Switching extruder has its own servo?
#if ENABLED(SWITCHING_EXTRUDER) && (!HAS_SWITCHING_NOZZLE || SWITCHING_EXTRUDER_SERVO_NR != SWITCHING_NOZZLE_SERVO_NR)
  #define DO_SWITCH_EXTRUDER 1
#endif

/**
 * The BLTouch Probe emulates a servo probe
 * and uses "special" angles for its state.
 */
#if ENABLED(BLTOUCH) && !defined(Z_PROBE_SERVO_NR)
  #define Z_PROBE_SERVO_NR 0
#endif

/**
 * Set a flag for a servo probe (or BLTouch)
 */
#ifdef Z_PROBE_SERVO_NR
  #define HAS_Z_SERVO_PROBE 1
#endif
#if ANY(HAS_Z_SERVO_PROBE, SWITCHING_EXTRUDER, SWITCHING_NOZZLE)
  #define HAS_SERVO_ANGLES 1
#endif
#if !HAS_SERVO_ANGLES
  #undef EDITABLE_SERVO_ANGLES
#endif

/**
 * Set flags for any form of bed probe
 */
#if ANY(TOUCH_MI_PROBE, Z_PROBE_ALLEN_KEY, HAS_Z_SERVO_PROBE, SOLENOID_PROBE, Z_PROBE_SLED, RACK_AND_PINION_PROBE, SENSORLESS_PROBING, MAGLEV4, MAG_MOUNTED_PROBE, BIQU_MICROPROBE_V1, BIQU_MICROPROBE_V2)
  #define HAS_STOWABLE_PROBE 1
#endif
#if ANY(HAS_STOWABLE_PROBE, FIX_MOUNTED_PROBE, BD_SENSOR, NOZZLE_AS_PROBE)
  #define HAS_BED_PROBE 1
#endif

// Probing tool change
#if !HAS_MULTI_EXTRUDER
  #undef PROBING_TOOL
#endif
#if HAS_BED_PROBE && defined(PROBING_TOOL)
  #define DO_TOOLCHANGE_FOR_PROBING 1
#endif

/**
 * Fill in undefined Filament Sensor options
 */
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #if NUM_RUNOUT_SENSORS >= 1
    #ifndef FIL_RUNOUT1_STATE
      #define FIL_RUNOUT1_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT1_PULLUP
      #define FIL_RUNOUT1_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT1_PULLDOWN
      #define FIL_RUNOUT1_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 2
    #ifndef FIL_RUNOUT2_STATE
      #define FIL_RUNOUT2_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT2_PULLUP
      #define FIL_RUNOUT2_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT2_PULLDOWN
      #define FIL_RUNOUT2_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 3
    #ifndef FIL_RUNOUT3_STATE
      #define FIL_RUNOUT3_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT3_PULLUP
      #define FIL_RUNOUT3_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT3_PULLDOWN
      #define FIL_RUNOUT3_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 4
    #ifndef FIL_RUNOUT4_STATE
      #define FIL_RUNOUT4_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT4_PULLUP
      #define FIL_RUNOUT4_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT4_PULLDOWN
      #define FIL_RUNOUT4_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 5
    #ifndef FIL_RUNOUT5_STATE
      #define FIL_RUNOUT5_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT5_PULLUP
      #define FIL_RUNOUT5_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT5_PULLDOWN
      #define FIL_RUNOUT5_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 6
    #ifndef FIL_RUNOUT6_STATE
      #define FIL_RUNOUT6_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT6_PULLUP
      #define FIL_RUNOUT6_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT6_PULLDOWN
      #define FIL_RUNOUT6_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 7
    #ifndef FIL_RUNOUT7_STATE
      #define FIL_RUNOUT7_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT7_PULLUP
      #define FIL_RUNOUT7_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT7_PULLDOWN
      #define FIL_RUNOUT7_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 8
    #ifndef FIL_RUNOUT8_STATE
      #define FIL_RUNOUT8_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT8_PULLUP
      #define FIL_RUNOUT8_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT8_PULLDOWN
      #define FIL_RUNOUT8_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
#endif // FILAMENT_RUNOUT_SENSOR

#if ENABLED(FILAMENT_SWITCH_AND_MOTION)
  #if NUM_MOTION_SENSORS >= 1
    #ifndef FIL_MOTION1_STATE
      #define FIL_MOTION1_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION1_PULLUP
      #define FIL_MOTION1_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION1_PULLDOWN
      #define FIL_MOTION1_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 2
    #ifndef FIL_MOTION2_STATE
      #define FIL_MOTION2_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION2_PULLUP
      #define FIL_MOTION2_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION2_PULLDOWN
      #define FIL_MOTION2_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 3
    #ifndef FIL_MOTION3_STATE
      #define FIL_MOTION3_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION3_PULLUP
      #define FIL_MOTION3_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION3_PULLDOWN
      #define FIL_MOTION3_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 4
    #ifndef FIL_MOTION4_STATE
      #define FIL_MOTION4_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION4_PULLUP
      #define FIL_MOTION4_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION4_PULLDOWN
      #define FIL_MOTION4_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 5
    #ifndef FIL_MOTION5_STATE
      #define FIL_MOTION5_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION5_PULLUP
      #define FIL_MOTION5_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION5_PULLDOWN
      #define FIL_MOTION5_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 6
    #ifndef FIL_MOTION6_STATE
      #define FIL_MOTION6_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION6_PULLUP
      #define FIL_MOTION6_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION6_PULLDOWN
      #define FIL_MOTION6_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 7
    #ifndef FIL_MOTION7_STATE
      #define FIL_MOTION7_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION7_PULLUP
      #define FIL_MOTION7_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION7_PULLDOWN
      #define FIL_MOTION7_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 8
    #ifndef FIL_MOTION8_STATE
      #define FIL_MOTION8_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION8_PULLUP
      #define FIL_MOTION8_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION8_PULLDOWN
      #define FILMOTION8_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
#endif // FILAMENT_SWITCH_AND_MOTION

// Homing to Min or Max
#if HAS_X_AXIS
  #if X_HOME_DIR > 0
    #define X_HOME_TO_MAX 1
  #elif X_HOME_DIR < 0
    #define X_HOME_TO_MIN 1
  #endif
#endif
#if HAS_Y_AXIS
  #if Y_HOME_DIR > 0
    #define Y_HOME_TO_MAX 1
  #elif Y_HOME_DIR < 0
    #define Y_HOME_TO_MIN 1
  #endif
#endif
#if HAS_Z_AXIS
  #if Z_HOME_DIR > 0
    #define Z_HOME_TO_MAX 1
  #elif Z_HOME_DIR < 0
    #define Z_HOME_TO_MIN 1
  #endif
#endif
#if HAS_I_AXIS
  #if I_HOME_DIR > 0
    #define I_HOME_TO_MAX 1
  #elif I_HOME_DIR < 0
    #define I_HOME_TO_MIN 1
  #endif
#endif
#if HAS_J_AXIS
  #if J_HOME_DIR > 0
    #define J_HOME_TO_MAX 1
  #elif J_HOME_DIR < 0
    #define J_HOME_TO_MIN 1
  #endif
#endif
#if HAS_K_AXIS
  #if K_HOME_DIR > 0
    #define K_HOME_TO_MAX 1
  #elif K_HOME_DIR < 0
    #define K_HOME_TO_MIN 1
  #endif
#endif
#if HAS_U_AXIS
  #if U_HOME_DIR > 0
    #define U_HOME_TO_MAX 1
  #elif U_HOME_DIR < 0
    #define U_HOME_TO_MIN 1
  #endif
#endif
#if HAS_V_AXIS
  #if V_HOME_DIR > 0
    #define V_HOME_TO_MAX 1
  #elif V_HOME_DIR < 0
    #define V_HOME_TO_MIN 1
  #endif
#endif
#if HAS_W_AXIS
  #if W_HOME_DIR > 0
    #define W_HOME_TO_MAX 1
  #elif W_HOME_DIR < 0
    #define W_HOME_TO_MIN 1
  #endif
#endif

/**
 * Conditionals based on the type of Bed Probe
 */
#if HAS_BED_PROBE
  #if ALL(DELTA, SENSORLESS_PROBING)
    #define HAS_DELTA_SENSORLESS_PROBING 1
  #else
    #define HAS_REAL_BED_PROBE 1
  #endif
  #if HAS_REAL_BED_PROBE && NONE(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN, Z_SPI_SENSORLESS)
    #define NEED_Z_MIN_PROBE_PIN 1
  #endif
  #if Z_HOME_TO_MIN && (!NEED_Z_MIN_PROBE_PIN || ENABLED(USE_PROBE_FOR_Z_HOMING))
    #define HOMING_Z_WITH_PROBE 1
  #endif
  #if DISABLED(NOZZLE_AS_PROBE)
    #define HAS_PROBE_XY_OFFSET 1
  #endif
  #if ANY(Z_PROBE_ALLEN_KEY, MAG_MOUNTED_PROBE)
    #define PROBE_TRIGGERED_WHEN_STOWED_TEST 1 // Extra test for Allen Key Probe
  #endif
  #ifndef Z_PROBE_ERROR_TOLERANCE
    #define Z_PROBE_ERROR_TOLERANCE Z_CLEARANCE_MULTI_PROBE
  #endif
  #if MULTIPLE_PROBING > 1
    #if EXTRA_PROBING > 0
      #define TOTAL_PROBING (MULTIPLE_PROBING + EXTRA_PROBING)
    #else
      #define TOTAL_PROBING MULTIPLE_PROBING
    #endif
  #endif
#else
  // Clear probe pin settings when no probe is selected
  #undef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #undef Z_MIN_PROBE_ENDSTOP_HIT_STATE
  #undef USE_PROBE_FOR_Z_HOMING
  #undef Z_MIN_PROBE_REPEATABILITY_TEST
  #undef HOMING_Z_WITH_PROBE
  #undef Z_CLEARANCE_MULTI_PROBE
  #undef Z_PROBE_ERROR_TOLERANCE
  #undef Z_AFTER_PROBING
  #undef Z_PROBE_LOW_POINT
  #undef MULTIPLE_PROBING
  #undef EXTRA_PROBING
  #undef PROBE_OFFSET_ZMIN
  #undef PROBE_OFFSET_ZMAX
  #undef PAUSE_BEFORE_DEPLOY_STOW
  #undef PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED
  #undef PROBING_HEATERS_OFF
  #undef WAIT_FOR_BED_HEATER
  #undef WAIT_FOR_HOTEND
  #undef PROBING_FANS_OFF
  #undef PROBING_ESTEPPERS_OFF
  #undef PROBING_STEPPERS_OFF
  #undef DELAY_BEFORE_PROBING
  #undef PREHEAT_BEFORE_PROBING
  #undef PROBING_NOZZLE_TEMP
  #undef PROBING_BED_TEMP
  #undef NOZZLE_TO_PROBE_OFFSET
#endif

#if ENABLED(BELTPRINTER) && !defined(HOME_Y_BEFORE_X)
  #define HOME_Y_BEFORE_X
#endif

#if Z_HOME_TO_MAX && DISABLED(Z_SAFE_HOMING)
  #define HOME_Z_FIRST // If homing away from BED do Z first
#endif

/**
 * Conditionals based on the type of Bed Leveling
 */
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #undef LCD_BED_LEVELING
  #if ANY(DELTA, SEGMENT_LEVELED_MOVES)
    #define UBL_SEGMENTED 1
  #endif
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT)
  #define ABL_PLANAR 1
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)
  #define ABL_USES_GRID 1
  #ifndef XY_PROBE_FEEDRATE_MIN
    #define XY_PROBE_FEEDRATE_MIN 60 // Minimum mm/min value for 'G29 S<feedrate>'
  #endif
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_3POINT)
  #define HAS_ABL_NOT_UBL 1
#endif
#if ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL, MESH_BED_LEVELING)
  #define HAS_MESH 1
#endif
#if ANY(AUTO_BED_LEVELING_UBL, AUTO_BED_LEVELING_3POINT)
  #define NEEDS_THREE_PROBE_POINTS 1
#endif
#if ANY(HAS_ABL_NOT_UBL, AUTO_BED_LEVELING_UBL)
  #define HAS_ABL_OR_UBL 1
  #if DISABLED(PROBE_MANUALLY)
    #define HAS_AUTOLEVEL 1
  #endif
#endif
#if ANY(HAS_ABL_OR_UBL, MESH_BED_LEVELING)
  #define HAS_LEVELING 1
  #if DISABLED(AUTO_BED_LEVELING_UBL)
    #define PLANNER_LEVELING 1
  #endif
#endif
#if !HAS_LEVELING
  #undef RESTORE_LEVELING_AFTER_G28
  #undef ENABLE_LEVELING_AFTER_G28
  #undef G29_RETRY_AND_RECOVER
#endif
#if !HAS_LEVELING || ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)
  #undef PROBE_MANUALLY
#endif
#if ANY(HAS_BED_PROBE, PROBE_MANUALLY, MESH_BED_LEVELING)
  #define PROBE_SELECTED 1
#endif

#if !HAS_MESH
  #undef MESH_INSET
#endif

#if NONE(PROBE_SELECTED, AUTO_BED_LEVELING_UBL)
  #undef Z_CLEARANCE_BETWEEN_PROBES
#endif

#if NONE(PROBE_SELECTED, DELTA_AUTO_CALIBRATION, DELTA_CALIBRATION_MENU)
  #undef PROBING_MARGIN
  #undef PROBING_MARGIN_LEFT
  #undef PROBING_MARGIN_RIGHT
  #undef PROBING_MARGIN_FRONT
  #undef PROBING_MARGIN_BACK
  #undef XY_PROBE_FEEDRATE
#endif

#if !(ANY(HAS_BED_PROBE, HAS_LEVELING) || ALL(AUTO_BED_LEVELING_UBL, HAS_MARLINUI_MENU))
  #undef Z_CLEARANCE_DEPLOY_PROBE
#endif

#if !(ANY(HAS_BED_PROBE, BACKLASH_GCODE) || (ENABLED(EXTENSIBLE_UI) && ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)))
  #undef Z_PROBE_FEEDRATE_FAST
  #undef Z_PROBE_FEEDRATE_SLOW
#endif

/**
 * Z_CLEARANCE_FOR_HOMING
 */
#ifndef Z_CLEARANCE_FOR_HOMING
  #ifdef Z_CLEARANCE_BETWEEN_PROBES
    #define Z_CLEARANCE_FOR_HOMING Z_CLEARANCE_BETWEEN_PROBES
  #else
    #define Z_CLEARANCE_FOR_HOMING 5
  #endif
#endif

/**
 * Z_CLEARANCE_BETWEEN_PROBES
 */
#if PROBE_SELECTED || (HAS_MARLINUI_MENU && ANY(DELTA_CALIBRATION_MENU, DELTA_AUTO_CALIBRATION))
  #ifndef Z_CLEARANCE_BETWEEN_PROBES
    #define Z_CLEARANCE_BETWEEN_PROBES Z_CLEARANCE_FOR_HOMING
  #endif
#endif

/**
 * Z_CLEARANCE_BETWEEN_MANUAL_PROBES / Z_CLEARANCE_MULTI_PROBE
 */
#if PROBE_SELECTED
  #if ANY(MESH_BED_LEVELING, PROBE_MANUALLY)
    #if Z_CLEARANCE_BETWEEN_PROBES > Z_CLEARANCE_FOR_HOMING
      #define Z_CLEARANCE_BETWEEN_MANUAL_PROBES Z_CLEARANCE_BETWEEN_PROBES
    #else
      #define Z_CLEARANCE_BETWEEN_MANUAL_PROBES Z_CLEARANCE_FOR_HOMING
    #endif
  #endif
  #ifndef Z_CLEARANCE_MULTI_PROBE
    #ifdef Z_CLEARANCE_BETWEEN_PROBES
      #define Z_CLEARANCE_MULTI_PROBE Z_CLEARANCE_BETWEEN_PROBES
    #else
      #define Z_CLEARANCE_MULTI_PROBE 5
    #endif
  #endif
#endif
#if TOTAL_PROBING < 2
  #undef Z_CLEARANCE_MULTI_PROBE
#endif

#if DISABLED(ENABLE_LEVELING_FADE_HEIGHT)
  #undef DEFAULT_LEVELING_FADE_HEIGHT
#endif

#ifdef GRID_MAX_POINTS_X
  #define GRID_MAX_POINTS ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))
  #define GRID_LOOP(A,B) for (uint8_t A = 0; A < GRID_MAX_POINTS_X; ++A) for (uint8_t B = 0; B < GRID_MAX_POINTS_Y; ++B)
#endif

/**
 * CoreXY, CoreXZ, and CoreYZ - and their reverse
 */
#if ANY(COREXY, COREYX)
  #define CORE_IS_XY 1
#endif
#if ANY(COREXZ, COREZX)
  #define CORE_IS_XZ 1
#endif
#if ANY(COREYZ, COREZY)
  #define CORE_IS_YZ 1
#endif
#if CORE_IS_XY || CORE_IS_XZ || CORE_IS_YZ
  #define IS_CORE 1
  #if CORE_IS_XY
    #define CORE_AXIS_1 A_AXIS
    #define CORE_AXIS_2 B_AXIS
    #define NORMAL_AXIS Z_AXIS
  #elif CORE_IS_XZ
    #define CORE_AXIS_1 A_AXIS
    #define NORMAL_AXIS Y_AXIS
    #define CORE_AXIS_2 C_AXIS
  #elif CORE_IS_YZ
    #define NORMAL_AXIS X_AXIS
    #define CORE_AXIS_1 B_AXIS
    #define CORE_AXIS_2 C_AXIS
  #endif
  #define CORESIGN(n) (ANY(COREYX, COREZX, COREZY) ? (-(n)) : (n))
#elif ANY(MARKFORGED_XY, MARKFORGED_YX)
  // Markforged kinematics
  #define CORE_AXIS_1 A_AXIS
  #define CORE_AXIS_2 B_AXIS
  #define NORMAL_AXIS Z_AXIS
#endif

#if ANY(MORGAN_SCARA, MP_SCARA, AXEL_TPARA)
  #define IS_SCARA 1
  #define IS_KINEMATIC 1
#elif ANY(DELTA, POLARGRAPH, POLAR)
  #define IS_KINEMATIC 1
#else
  #define IS_CARTESIAN 1
  #if !IS_CORE
    #define IS_FULL_CARTESIAN 1
  #endif
#endif

#if DISABLED(DELTA)
  #undef DELTA_HOME_TO_SAFE_ZONE
#endif
#if ANY(DELTA, AXEL_TPARA)
  #define Z_CAN_FALL_DOWN
#endif

/**
 * This setting is also used by M109 when trying to calculate
 * a ballpark safe margin to prevent wait-forever situation.
 */
#ifndef EXTRUDE_MINTEMP
  #define EXTRUDE_MINTEMP 170
#endif

#if ANY(PID_DEBUG, PID_BED_DEBUG, PID_CHAMBER_DEBUG)
  #define HAS_PID_DEBUG 1
#endif

#if ALL(SPI_FLASH, HAS_MEDIA, MARLIN_DEV_MODE)
  #define SPI_FLASH_BACKUP 1
#endif
