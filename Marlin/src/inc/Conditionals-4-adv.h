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
#pragma once

/**
 * Conditionals-4-adv.h
 * Conditionals set before pins.h and which depend on Configuration_adv.h.
 */

//========================================================
// Get requirements for the benefit of IntelliSense, etc.
//
#include "MarlinConfigPre-4-adv.h"
//========================================================

#if ENABLED(MARLIN_SMALL_BUILD)
  #undef EEPROM_CHITCHAT
  #undef CAPABILITIES_REPORT
  #define DISABLE_M503
#endif

#ifndef AXIS_RELATIVE_MODES
  #define AXIS_RELATIVE_MODES {}
#endif

#if defined(SWITCHING_NOZZLE_E1_SERVO_NR) && DISABLED(MECHANICAL_SWITCHING_NOZZLE)
  #define SWITCHING_NOZZLE_TWO_SERVOS 1
#endif

// Determine NUM_SERVOS if none was supplied
#ifndef NUM_SERVOS
  #define NUM_SERVOS 0
  #if HAS_Z_SERVO_PROBE && NUM_SERVOS <= Z_PROBE_SERVO_NR
    #undef NUM_SERVOS
    #define NUM_SERVOS INCREMENT(Z_PROBE_SERVO_NR)
  #endif
  #if ENABLED(CHAMBER_VENT) && NUM_SERVOS <= CHAMBER_VENT_SERVO_NR
    #undef NUM_SERVOS
    #define NUM_SERVOS INCREMENT(CHAMBER_VENT_SERVO_NR)
  #endif
  #if ENABLED(SWITCHING_TOOLHEAD) && NUM_SERVOS <= SWITCHING_TOOLHEAD_SERVO_NR
    #undef NUM_SERVOS
    #define NUM_SERVOS INCREMENT(SWITCHING_TOOLHEAD_SERVO_NR)
  #endif
  #if ENABLED(SWITCHING_NOZZLE)
    #if NUM_SERVOS <= SWITCHING_NOZZLE_SERVO_NR
      #undef NUM_SERVOS
      #define NUM_SERVOS INCREMENT(SWITCHING_NOZZLE_SERVO_NR)
    #endif
    #if NUM_SERVOS <= SWITCHING_NOZZLE_E1_SERVO_NR
      #undef NUM_SERVOS
      #define NUM_SERVOS INCREMENT(SWITCHING_NOZZLE_E1_SERVO_NR)
    #endif
  #endif
  #if ENABLED(SWITCHING_EXTRUDER)
    #if NUM_SERVOS <= SWITCHING_EXTRUDER_SERVO_NR
      #undef NUM_SERVOS
      #define NUM_SERVOS INCREMENT(SWITCHING_EXTRUDER_SERVO_NR)
    #endif
    #if NUM_SERVOS <= SWITCHING_EXTRUDER_E23_SERVO_NR
      #undef NUM_SERVOS
      #define NUM_SERVOS INCREMENT(SWITCHING_EXTRUDER_E23_SERVO_NR)
    #endif
  #endif
  #if ENABLED(SPINDLE_SERVO) && NUM_SERVOS <= SPINDLE_SERVO_NR
    #undef NUM_SERVOS
    #define NUM_SERVOS INCREMENT(SPINDLE_SERVO_NR)
  #endif
#endif // !defined(NUM_SERVOS)

// Convenience override for a BLTouch alone
#if ENABLED(BLTOUCH)
  #ifndef BLTOUCH_DELAY
    #define BLTOUCH_DELAY 500
  #endif
  #ifdef BLTOUCH_HS_MODE
    #define HAS_BLTOUCH_HS_MODE 1
  #endif
  #if NUM_SERVOS == 1
    #undef SERVO_DELAY
    #define SERVO_DELAY { 50 }
  #endif
#endif

#if !HAS_BED_PROBE
  #undef BABYSTEP_ZPROBE_OFFSET
  #undef PROBING_USE_CURRENT_HOME
#endif
#if !HAS_STOWABLE_PROBE
  #undef PROBE_DEPLOY_STOW_MENU
#endif

// Some options are disallowed without required axes
#if !HAS_X_AXIS
  //#define LCD_SHOW_E_TOTAL
  #define NO_WORKSPACE_OFFSETS
  #define NO_HOME_OFFSETS
  #undef CALIBRATION_MEASURE_LEFT
  #undef CALIBRATION_MEASURE_RIGHT
  #undef CALIBRATION_MEASURE_XMAX
  #undef CALIBRATION_MEASURE_XMIN
  #undef DISABLE_IDLE_X
  #undef INPUT_SHAPING_X
  #undef SAFE_BED_LEVELING_START_X
  #undef SHAPING_FREQ_X
  #undef SHAPING_ZETA_X
  #undef STEALTHCHOP_X
#endif

#if !HAS_Y_AXIS
  #undef ARC_SUPPORT
  #undef CALIBRATION_MEASURE_BACK
  #undef CALIBRATION_MEASURE_FRONT
  #undef CALIBRATION_MEASURE_YMAX
  #undef CALIBRATION_MEASURE_YMIN
  #undef DISABLE_IDLE_Y
  #undef HOME_Y_BEFORE_X
  #undef INPUT_SHAPING_Y
  #undef QUICK_HOME
  #undef SAFE_BED_LEVELING_START_Y
  #undef SHAPING_FREQ_Y
  #undef SHAPING_ZETA_Y
  #undef STEALTHCHOP_Y
  #undef STEP_STATE_Y
#endif

#if !HAS_Z_AXIS
  #undef CALIBRATION_MEASURE_ZMAX
  #undef CALIBRATION_MEASURE_ZMIN
  #undef CNC_WORKSPACE_PLANES
  #undef DISABLE_IDLE_Z
  #undef ENABLE_LEVELING_FADE_HEIGHT
  #undef HOME_Z_FIRST
  #undef HOMING_Z_WITH_PROBE
  #undef INPUT_SHAPING_Z
  #undef NUM_Z_STEPPERS
  #undef SAFE_BED_LEVELING_START_Z
  #undef SHAPING_FREQ_Z
  #undef SHAPING_ZETA_Z
  #undef STEALTHCHOP_Z
  #undef STEP_STATE_Z
  #undef Z_IDLE_HEIGHT
  #undef Z_PROBE_SLED
  #undef Z_SAFE_HOMING
#endif

#if !HAS_I_AXIS
  #undef CALIBRATION_MEASURE_IMAX
  #undef CALIBRATION_MEASURE_IMIN
  #undef DISABLE_IDLE_I
  #undef SAFE_BED_LEVELING_START_I
  #undef STEALTHCHOP_I
  #undef STEP_STATE_I
#endif

#if !HAS_J_AXIS
  #undef CALIBRATION_MEASURE_JMAX
  #undef CALIBRATION_MEASURE_JMIN
  #undef DISABLE_IDLE_J
  #undef SAFE_BED_LEVELING_START_J
  #undef STEALTHCHOP_J
  #undef STEP_STATE_J
#endif

#if !HAS_K_AXIS
  #undef CALIBRATION_MEASURE_KMAX
  #undef CALIBRATION_MEASURE_KMIN
  #undef DISABLE_IDLE_K
  #undef SAFE_BED_LEVELING_START_K
  #undef STEALTHCHOP_K
  #undef STEP_STATE_K
#endif

#if !HAS_U_AXIS
  #undef CALIBRATION_MEASURE_UMAX
  #undef CALIBRATION_MEASURE_UMIN
  #undef DISABLE_IDLE_U
  #undef SAFE_BED_LEVELING_START_U
  #undef STEALTHCHOP_U
  #undef STEP_STATE_U
#endif

#if !HAS_V_AXIS
  #undef CALIBRATION_MEASURE_VMAX
  #undef CALIBRATION_MEASURE_VMIN
  #undef DISABLE_IDLE_V
  #undef SAFE_BED_LEVELING_START_V
  #undef STEALTHCHOP_V
  #undef STEP_STATE_V
#endif

#if !HAS_W_AXIS
  #undef CALIBRATION_MEASURE_WMAX
  #undef CALIBRATION_MEASURE_WMIN
  #undef DISABLE_IDLE_W
  #undef SAFE_BED_LEVELING_START_W
  #undef STEALTHCHOP_W
  #undef STEP_STATE_W
#endif

// Disallowed with no extruders
#if !HAS_EXTRUDERS
  #define NO_VOLUMETRICS
  #undef ADVANCED_PAUSE_FEATURE
  #undef DISABLE_IDLE_E
  #undef EXTRUDER_RUNOUT_PREVENT
  #undef FILAMENT_LOAD_UNLOAD_GCODES
  #undef FWRETRACT
  #undef LCD_SHOW_E_TOTAL
  #undef LIN_ADVANCE
  #undef MANUAL_E_MOVES_RELATIVE
  #undef PID_EXTRUSION_SCALING
  #undef SHOW_TEMP_ADC_VALUES
  #undef STEALTHCHOP_E
#endif

#if !TEMP_SENSOR_CHAMBER
  #undef CHAMBER_CHECK_INTERVAL
  #undef CHAMBER_AUTO_FAN_PIN
#endif
#if !TEMP_SENSOR_COOLER
  #undef COOLER_AUTO_FAN_PIN
#endif

#if ENABLED(DISABLE_X) && !defined(DISABLE_IDLE_X)
  #define DISABLE_IDLE_X
#endif
#if ENABLED(DISABLE_Y) && !defined(DISABLE_IDLE_Y)
  #define DISABLE_IDLE_Y
#endif
#if ENABLED(DISABLE_Z) && !defined(DISABLE_IDLE_Z)
  #define DISABLE_IDLE_Z
#endif
#if ENABLED(DISABLE_I) && !defined(DISABLE_IDLE_I)
  #define DISABLE_IDLE_I
#endif
#if ENABLED(DISABLE_J) && !defined(DISABLE_IDLE_J)
  #define DISABLE_IDLE_J
#endif
#if ENABLED(DISABLE_K) && !defined(DISABLE_IDLE_K)
  #define DISABLE_IDLE_K
#endif
#if ENABLED(DISABLE_U) && !defined(DISABLE_IDLE_U)
  #define DISABLE_IDLE_U
#endif
#if ENABLED(DISABLE_V) && !defined(DISABLE_IDLE_V)
  #define DISABLE_IDLE_V
#endif
#if ENABLED(DISABLE_W) && !defined(DISABLE_IDLE_W)
  #define DISABLE_IDLE_W
#endif
#if ENABLED(DISABLE_E) && !defined(DISABLE_IDLE_E)
  #define DISABLE_IDLE_E
#endif

#define _OR_HAS_DI(A) || ALL(HAS_##A##_AXIS, DISABLE_IDLE_##A)
#if ALL(HAS_EXTRUDERS, DISABLE_IDLE_E) MAP(_OR_HAS_DI, X, Y, Z, I, J, K, U, V, W)
  #define HAS_DISABLE_IDLE_AXES 1
#endif
#undef _OR_HAS_DI

// Remove hotend-dependent settings
#if HOTENDS < 8
  #undef E7_AUTO_FAN_PIN
  #undef HEATER_7_MAXTEMP
  #undef HEATER_7_MINTEMP
  #if HOTENDS < 7
    #undef E6_AUTO_FAN_PIN
    #undef HEATER_6_MAXTEMP
    #undef HEATER_6_MINTEMP
    #if HOTENDS < 6
      #undef E5_AUTO_FAN_PIN
      #undef HEATER_5_MAXTEMP
      #undef HEATER_5_MINTEMP
      #if HOTENDS < 5
        #undef E4_AUTO_FAN_PIN
        #undef HEATER_4_MAXTEMP
        #undef HEATER_4_MINTEMP
        #if HOTENDS < 4
          #undef E3_AUTO_FAN_PIN
          #undef HEATER_3_MAXTEMP
          #undef HEATER_3_MINTEMP
          #if HOTENDS < 3
            #undef E2_AUTO_FAN_PIN
            #undef HEATER_2_MAXTEMP
            #undef HEATER_2_MINTEMP
            #if HOTENDS < 2
              #undef E1_AUTO_FAN_PIN
              #undef HEATER_1_MAXTEMP
              #undef HEATER_1_MINTEMP
              #if HOTENDS < 1
                #undef E0_AUTO_FAN_PIN
                #undef HEATER_0_MAXTEMP
                #undef HEATER_0_MINTEMP
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

// Use Junction Deviation for motion if Jerk is disabled
#if DISABLED(CLASSIC_JERK)
  #define HAS_JUNCTION_DEVIATION 1
#endif

// E jerk exists with JD disabled (of course) but also when Linear Advance is disabled on Delta/SCARA
#if HAS_EXTRUDERS && (ENABLED(CLASSIC_JERK) || (IS_KINEMATIC && DISABLED(LIN_ADVANCE)))
  #define HAS_CLASSIC_E_JERK 1
#endif

// Linear advance uses Jerk since E is an isolated axis
#if ALL(HAS_JUNCTION_DEVIATION, LIN_ADVANCE)
  #define HAS_LINEAR_E_JERK 1
#endif

// Some displays can toggle Adaptive Step Smoothing.
// The state is saved to EEPROM.
// In future this may be added to a G-code such as M205 A.
#if ALL(ADAPTIVE_STEP_SMOOTHING, DWIN_LCD_PROUI)
  #define ADAPTIVE_STEP_SMOOTHING_TOGGLE
#endif

/**
 * Temperature Sensors; define what sensor(s) we have.
 */

// Temperature sensor IDs
#define H_NONE    -128
#define H_REDUNDANT -7
#define H_SOC       -6
#define H_BOARD     -5
#define H_COOLER    -4
#define H_PROBE     -3
#define H_CHAMBER   -2
#define H_BED       -1
#define H_E0         0
#define H_E1         1
#define H_E2         2
#define H_E3         3
#define H_E4         4
#define H_E5         5
#define H_E6         6
#define H_E7         7

#define _SENSOR_IS(I,N) || (TEMP_SENSOR(N) == I)
#define _E_SENSOR_IS(I,N) _SENSOR_IS(N,I)
#define ANY_E_SENSOR_IS(N) (0 REPEAT2(HOTENDS, _E_SENSOR_IS, N))
#define ANY_THERMISTOR_IS(N) ( ANY_E_SENSOR_IS(N) _SENSOR_IS(N,REDUNDANT) \
  _SENSOR_IS(N,BED) _SENSOR_IS(N,PROBE) _SENSOR_IS(N,CHAMBER) _SENSOR_IS(N,COOLER) _SENSOR_IS(N,BOARD) )

#if ANY_THERMISTOR_IS(1000)
  #define HAS_USER_THERMISTORS 1
#endif

#if TEMP_SENSOR_REDUNDANT
  #define _HEATER_ID(M) H_##M
  #define HEATER_ID(M)  _HEATER_ID(M)
  #define REDUNDANT_TEMP_MATCH(M,N) (HEATER_ID(TEMP_SENSOR_REDUNDANT_##M) == _HEATER_ID(N))
#else
  #define REDUNDANT_TEMP_MATCH(...) 0
#endif

#if TEMP_SENSOR_IS_MAX_TC(0)
  #if TEMP_SENSOR_0 == -5
    #define TEMP_SENSOR_0_IS_MAX31865 1
    #define TEMP_SENSOR_0_MAX_TC_TMIN    0
    #define TEMP_SENSOR_0_MAX_TC_TMAX 1024
    #ifndef MAX31865_SENSOR_WIRES_0
      #define MAX31865_SENSOR_WIRES_0 2
    #endif
    #ifndef MAX31865_WIRE_OHMS_0
      #define MAX31865_WIRE_OHMS_0 0.0f
    #endif
  #elif TEMP_SENSOR_0 == -3
    #define TEMP_SENSOR_0_IS_MAX31855 1
    #define TEMP_SENSOR_0_MAX_TC_TMIN -270
    #define TEMP_SENSOR_0_MAX_TC_TMAX 1800
  #elif TEMP_SENSOR_0 == -2
    #define TEMP_SENSOR_0_IS_MAX6675 1
    #define TEMP_SENSOR_0_MAX_TC_TMIN    0
    #define TEMP_SENSOR_0_MAX_TC_TMAX 1024
  #endif
#elif TEMP_SENSOR_0 == -4
  #define TEMP_SENSOR_0_IS_AD8495 1
#elif TEMP_SENSOR_0 == -1
  #define TEMP_SENSOR_0_IS_AD595 1
#elif TEMP_SENSOR_0 > 0
  #define TEMP_SENSOR_0_IS_THERMISTOR 1
  #if TEMP_SENSOR_0 == 1000
    #define TEMP_SENSOR_0_IS_CUSTOM 1
  #elif TEMP_SENSOR_0 == 998 || TEMP_SENSOR_0 == 999
    #define TEMP_SENSOR_0_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(1)
  #if TEMP_SENSOR_1 == -5
    #define TEMP_SENSOR_1_IS_MAX31865 1
    #define TEMP_SENSOR_1_MAX_TC_TMIN    0
    #define TEMP_SENSOR_1_MAX_TC_TMAX 1024
    #ifndef MAX31865_SENSOR_WIRES_1
      #define MAX31865_SENSOR_WIRES_1 2
    #endif
    #ifndef MAX31865_WIRE_OHMS_1
      #define MAX31865_WIRE_OHMS_1 0.0f
    #endif
  #elif TEMP_SENSOR_1 == -3
    #define TEMP_SENSOR_1_IS_MAX31855 1
    #define TEMP_SENSOR_1_MAX_TC_TMIN -270
    #define TEMP_SENSOR_1_MAX_TC_TMAX 1800
  #elif TEMP_SENSOR_1 == -2
    #define TEMP_SENSOR_1_IS_MAX6675 1
    #define TEMP_SENSOR_1_MAX_TC_TMIN    0
    #define TEMP_SENSOR_1_MAX_TC_TMAX 1024
  #endif

  #if TEMP_SENSOR_1 != TEMP_SENSOR_0
    #if   TEMP_SENSOR_1 == -5
      #error "If MAX31865 Thermocouple (-5) is used for TEMP_SENSOR_1 then TEMP_SENSOR_0 must match."
    #elif TEMP_SENSOR_1 == -3
      #error "If MAX31855 Thermocouple (-3) is used for TEMP_SENSOR_1 then TEMP_SENSOR_0 must match."
    #elif TEMP_SENSOR_1 == -2
      #error "If MAX6675 Thermocouple (-2) is used for TEMP_SENSOR_1 then TEMP_SENSOR_0 must match."
    #endif
  #endif
#elif TEMP_SENSOR_1 == -4
  #define TEMP_SENSOR_1_IS_AD8495 1
#elif TEMP_SENSOR_1 == -1
  #define TEMP_SENSOR_1_IS_AD595 1
#elif TEMP_SENSOR_1 > 0
  #define TEMP_SENSOR_1_IS_THERMISTOR 1
  #if TEMP_SENSOR_1 == 1000
    #define TEMP_SENSOR_1_IS_CUSTOM 1
  #elif TEMP_SENSOR_1 == 998 || TEMP_SENSOR_1 == 999
    #define TEMP_SENSOR_1_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(2)
  #if TEMP_SENSOR_2 == -5
    #define TEMP_SENSOR_2_IS_MAX31865 1
    #define TEMP_SENSOR_2_MAX_TC_TMIN    0
    #define TEMP_SENSOR_2_MAX_TC_TMAX 1024
    #ifndef MAX31865_SENSOR_WIRES_2
      #define MAX31865_SENSOR_WIRES_2 2
    #endif
    #ifndef MAX31865_WIRE_OHMS_2
      #define MAX31865_WIRE_OHMS_2 0.0f
    #endif
  #elif TEMP_SENSOR_2 == -3
    #define TEMP_SENSOR_2_IS_MAX31855 1
    #define TEMP_SENSOR_2_MAX_TC_TMIN -270
    #define TEMP_SENSOR_2_MAX_TC_TMAX 1800
  #elif TEMP_SENSOR_2 == -2
    #define TEMP_SENSOR_2_IS_MAX6675 1
    #define TEMP_SENSOR_2_MAX_TC_TMIN    0
    #define TEMP_SENSOR_2_MAX_TC_TMAX 1024
  #endif

  #if TEMP_SENSOR_2 != TEMP_SENSOR_0
    #if   TEMP_SENSOR_2 == -5
      #error "If MAX31865 Thermocouple (-5) is used for TEMP_SENSOR_2 then TEMP_SENSOR_0 must match."
    #elif TEMP_SENSOR_2 == -3
      #error "If MAX31855 Thermocouple (-3) is used for TEMP_SENSOR_2 then TEMP_SENSOR_0 must match."
    #elif TEMP_SENSOR_2 == -2
      #error "If MAX6675 Thermocouple (-2) is used for TEMP_SENSOR_2 then TEMP_SENSOR_0 must match."
    #endif
  #endif
#elif TEMP_SENSOR_2 == -4
  #define TEMP_SENSOR_2_IS_AD8495 1
#elif TEMP_SENSOR_2 == -1
  #define TEMP_SENSOR_2_IS_AD595 1
#elif TEMP_SENSOR_2 > 0
  #define TEMP_SENSOR_2_IS_THERMISTOR 1
  #if TEMP_SENSOR_2 == 1000
    #define TEMP_SENSOR_2_IS_CUSTOM 1
  #elif TEMP_SENSOR_2 == 998 || TEMP_SENSOR_2 == 999
    #define TEMP_SENSOR_2_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_3 > 0
  #define TEMP_SENSOR_3_IS_THERMISTOR 1
  #if TEMP_SENSOR_3 == 1000
    #define TEMP_SENSOR_3_IS_CUSTOM 1
  #elif TEMP_SENSOR_3 == 998 || TEMP_SENSOR_3 == 999
    #define TEMP_SENSOR_3_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_4 > 0
  #define TEMP_SENSOR_4_IS_THERMISTOR 1
  #if TEMP_SENSOR_4 == 1000
    #define TEMP_SENSOR_4_IS_CUSTOM 1
  #elif TEMP_SENSOR_4 == 998 || TEMP_SENSOR_4 == 999
    #define TEMP_SENSOR_4_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_5 > 0
  #define TEMP_SENSOR_5_IS_THERMISTOR 1
  #if TEMP_SENSOR_5 == 1000
    #define TEMP_SENSOR_5_IS_CUSTOM 1
  #elif TEMP_SENSOR_5 == 998 || TEMP_SENSOR_5 == 999
    #define TEMP_SENSOR_5_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_6 > 0
  #define TEMP_SENSOR_6_IS_THERMISTOR 1
  #if TEMP_SENSOR_6 == 1000
    #define TEMP_SENSOR_6_IS_CUSTOM 1
  #elif TEMP_SENSOR_6 == 998 || TEMP_SENSOR_6 == 999
    #define TEMP_SENSOR_6_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_7 > 0
  #define TEMP_SENSOR_7_IS_THERMISTOR 1
  #if TEMP_SENSOR_7 == 1000
    #define TEMP_SENSOR_7_IS_CUSTOM 1
  #elif TEMP_SENSOR_7 == 998 || TEMP_SENSOR_7 == 999
    #define TEMP_SENSOR_7_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
  #define _REDUNDANT_E (REDUNDANT_TEMP_MATCH(SOURCE, E0) || REDUNDANT_TEMP_MATCH(SOURCE, E1) || REDUNDANT_TEMP_MATCH(SOURCE, E2))
  #if TEMP_SENSOR_REDUNDANT == -5
    #if !_REDUNDANT_E
      #error "MAX31865 Thermocouples (-5) not supported for TEMP_SENSOR_REDUNDANT_SOURCE other than TEMP_SENSOR_[0-2]."
    #endif

    #define TEMP_SENSOR_REDUNDANT_IS_MAX31865    1
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN    0
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX 1024
  #elif TEMP_SENSOR_REDUNDANT == -3
    #if !_REDUNDANT_E
      #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_REDUNDANT_SOURCE other than TEMP_SENSOR_[0-2]."
    #endif

    #define TEMP_SENSOR_REDUNDANT_IS_MAX31855    1
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN -270
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX 1800
  #elif TEMP_SENSOR_REDUNDANT == -2
    #if !_REDUNDANT_E
      #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_REDUNDANT_SOURCE other than TEMP_SENSOR_[0-2]."
    #endif

    #define TEMP_SENSOR_REDUNDANT_IS_MAX6675     1
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN    0
    #define TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX 1024
  #endif
  #undef _REDUNDANT_E

  // Mimic setting up the source TEMP_SENSOR
  #if REDUNDANT_TEMP_MATCH(SOURCE, E0)
    #define TEMP_SENSOR_0_MAX_TC_TMIN TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN
    #define TEMP_SENSOR_0_MAX_TC_TMAX TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX
    #ifndef MAX31865_SENSOR_WIRES_0
      #define MAX31865_SENSOR_WIRES_0 2
    #endif
  #elif REDUNDANT_TEMP_MATCH(SOURCE, E1)
    #define TEMP_SENSOR_1_MAX_TC_TMIN TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN
    #define TEMP_SENSOR_1_MAX_TC_TMAX TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX
    #ifndef MAX31865_SENSOR_WIRES_1
      #define MAX31865_SENSOR_WIRES_1 2
    #endif
  #elif REDUNDANT_TEMP_MATCH(SOURCE, E2)
    #define TEMP_SENSOR_2_MAX_TC_TMIN TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN
    #define TEMP_SENSOR_2_MAX_TC_TMAX TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX
    #ifndef MAX31865_SENSOR_WIRES_2
      #define MAX31865_SENSOR_WIRES_2 2
    #endif
  #endif

  #if (TEMP_SENSOR_IS_MAX_TC(0) && TEMP_SENSOR_REDUNDANT != TEMP_SENSOR_0) || (TEMP_SENSOR_IS_MAX_TC(1) && TEMP_SENSOR_REDUNDANT != TEMP_SENSOR_1) || (TEMP_SENSOR_IS_MAX_TC(2) && TEMP_SENSOR_REDUNDANT != TEMP_SENSOR_2)
    #if   TEMP_SENSOR_REDUNDANT == -5
      #error "If MAX31865 Thermocouple (-5) is used for TEMP_SENSOR_[0-2] then TEMP_SENSOR_REDUNDANT must match."
    #elif TEMP_SENSOR_REDUNDANT == -3
      #error "If MAX31855 Thermocouple (-3) is used for TEMP_SENSOR_[0-2] then TEMP_SENSOR_REDUNDANT must match."
    #elif TEMP_SENSOR_REDUNDANT == -2
      #error "If MAX6675 Thermocouple (-2) is used for TEMP_SENSOR_[0-2] then TEMP_SENSOR_REDUNDANT must match."
    #endif
  #endif
#elif TEMP_SENSOR_REDUNDANT == -4
  #define TEMP_SENSOR_REDUNDANT_IS_AD8495 1
#elif TEMP_SENSOR_REDUNDANT == -1
  #define TEMP_SENSOR_REDUNDANT_IS_AD595 1
#elif TEMP_SENSOR_REDUNDANT == 998 || TEMP_SENSOR_REDUNDANT == 999
  #error "Dummy sensors are not supported for TEMP_SENSOR_REDUNDANT."
#elif TEMP_SENSOR_REDUNDANT > 0
  #define TEMP_SENSOR_REDUNDANT_IS_THERMISTOR 1
  #if TEMP_SENSOR_REDUNDANT == 1000
    #define TEMP_SENSOR_REDUNDANT_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_3 == -4
  #define TEMP_SENSOR_3_IS_AD8495 1
#elif TEMP_SENSOR_3 == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_3."
#elif TEMP_SENSOR_3 == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_3."
#elif TEMP_SENSOR_3 == -1
  #define TEMP_SENSOR_3_IS_AD595 1
#elif TEMP_SENSOR_3 == 998 || TEMP_SENSOR_3 == 999
  #define TEMP_SENSOR_3_IS_DUMMY 1
#elif TEMP_SENSOR_3 > 0
  #define TEMP_SENSOR_3_IS_THERMISTOR 1
  #if TEMP_SENSOR_3 == 1000
    #define TEMP_SENSOR_3_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_4 == -4
  #define TEMP_SENSOR_4_IS_AD8495 1
#elif TEMP_SENSOR_4 == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_4."
#elif TEMP_SENSOR_4 == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_4."
#elif TEMP_SENSOR_4 == -1
  #define TEMP_SENSOR_4_IS_AD595 1
#elif TEMP_SENSOR_4 == 998 || TEMP_SENSOR_4 == 999
  #define TEMP_SENSOR_4_IS_DUMMY 1
#elif TEMP_SENSOR_4 > 0
  #define TEMP_SENSOR_4_IS_THERMISTOR 1
  #if TEMP_SENSOR_4 == 1000
    #define TEMP_SENSOR_4_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_5 == -4
  #define TEMP_SENSOR_5_IS_AD8495 1
#elif TEMP_SENSOR_5 == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_5."
#elif TEMP_SENSOR_5 == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_5."
#elif TEMP_SENSOR_5 == -1
  #define TEMP_SENSOR_5_IS_AD595 1
#elif TEMP_SENSOR_5 == 998 || TEMP_SENSOR_5 == 999
  #define TEMP_SENSOR_5_IS_DUMMY 1
#elif TEMP_SENSOR_5 > 0
  #define TEMP_SENSOR_5_IS_THERMISTOR 1
  #if TEMP_SENSOR_5 == 1000
    #define TEMP_SENSOR_5_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_6 == -4
  #define TEMP_SENSOR_6_IS_AD8495 1
#elif TEMP_SENSOR_6 == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_6."
#elif TEMP_SENSOR_6 == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_6."
#elif TEMP_SENSOR_6 == -1
  #define TEMP_SENSOR_6_IS_AD595 1
#elif TEMP_SENSOR_6 == 998 || TEMP_SENSOR_6 == 999
  #define TEMP_SENSOR_6_IS_DUMMY 1
#elif TEMP_SENSOR_6 > 0
  #define TEMP_SENSOR_6_IS_THERMISTOR 1
  #if TEMP_SENSOR_6 == 1000
    #define TEMP_SENSOR_6_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_7 == -4
  #define TEMP_SENSOR_7_IS_AD8495 1
#elif TEMP_SENSOR_7 == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_7."
#elif TEMP_SENSOR_7 == -2
  #error "MAX7775 Thermocouples (-2) not supported for TEMP_SENSOR_7."
#elif TEMP_SENSOR_7 == -1
  #define TEMP_SENSOR_7_IS_AD595 1
#elif TEMP_SENSOR_7 == 998 || TEMP_SENSOR_7 == 999
  #define TEMP_SENSOR_7_IS_DUMMY 1
#elif TEMP_SENSOR_7 > 0
  #define TEMP_SENSOR_7_IS_THERMISTOR 1
  #if TEMP_SENSOR_7 == 1000
    #define TEMP_SENSOR_7_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(BED)
  #if TEMP_SENSOR_BED == -5
    #define TEMP_SENSOR_BED_IS_MAX31865 1
    #define TEMP_SENSOR_BED_MAX_TC_TMIN    0
    #define TEMP_SENSOR_BED_MAX_TC_TMAX 1024
    #ifndef MAX31865_SENSOR_WIRES_BED
      #define MAX31865_SENSOR_WIRES_BED 2
    #endif
    #ifndef MAX31865_WIRE_OHMS_BED
      #define MAX31865_WIRE_OHMS_BED 0.0f
    #endif
  #elif TEMP_SENSOR_BED == -3
    #define TEMP_SENSOR_BED_IS_MAX31855 1
    #define TEMP_SENSOR_BED_MAX_TC_TMIN -270
    #define TEMP_SENSOR_BED_MAX_TC_TMAX 1800
  #elif TEMP_SENSOR_BED == -2
    #define TEMP_SENSOR_BED_IS_MAX6675 1
    #define TEMP_SENSOR_BED_MAX_TC_TMIN    0
    #define TEMP_SENSOR_BED_MAX_TC_TMAX 1024
  #endif
#elif TEMP_SENSOR_BED == -4
  #define TEMP_SENSOR_BED_IS_AD8495 1
#elif TEMP_SENSOR_BED == -1
  #define TEMP_SENSOR_BED_IS_AD595 1
#elif TEMP_SENSOR_BED > 0
  #define TEMP_SENSOR_BED_IS_THERMISTOR 1
  #if TEMP_SENSOR_BED == 1000
    #define TEMP_SENSOR_BED_IS_CUSTOM 1
  #elif TEMP_SENSOR_BED == 998 || TEMP_SENSOR_BED == 999
    #define TEMP_SENSOR_BED_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(0) || TEMP_SENSOR_IS_MAX_TC(1) || TEMP_SENSOR_IS_MAX_TC(2) || TEMP_SENSOR_IS_MAX_TC(BED) || TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
  #define HAS_MAX_TC 1
#endif
#if TEMP_SENSOR_0_IS_MAX6675 || TEMP_SENSOR_1_IS_MAX6675 || TEMP_SENSOR_2_IS_MAX6675 || TEMP_SENSOR_BED_IS_MAX6675 || TEMP_SENSOR_REDUNDANT_IS_MAX6675
  #define HAS_MAX6675 1
#endif
#if TEMP_SENSOR_0_IS_MAX31855 || TEMP_SENSOR_1_IS_MAX31855 || TEMP_SENSOR_2_IS_MAX31855 || TEMP_SENSOR_BED_IS_MAX31855 || TEMP_SENSOR_REDUNDANT_IS_MAX31855
  #define HAS_MAX31855 1
#endif
#if TEMP_SENSOR_0_IS_MAX31865 || TEMP_SENSOR_1_IS_MAX31865 || TEMP_SENSOR_2_IS_MAX31865 || TEMP_SENSOR_BED_IS_MAX31865 || TEMP_SENSOR_REDUNDANT_IS_MAX31865
  #define HAS_MAX31865 1
#endif

#if !HAS_MAX_TC
  #undef THERMOCOUPLE_MAX_ERRORS
#endif

#if TEMP_SENSOR_CHAMBER == -4
  #define TEMP_SENSOR_CHAMBER_IS_AD8495 1
#elif TEMP_SENSOR_CHAMBER == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_CHAMBER."
#elif TEMP_SENSOR_CHAMBER == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_CHAMBER."
#elif TEMP_SENSOR_CHAMBER == -1
  #define TEMP_SENSOR_CHAMBER_IS_AD595 1
#elif TEMP_SENSOR_CHAMBER > 0
  #define TEMP_SENSOR_CHAMBER_IS_THERMISTOR 1
  #if TEMP_SENSOR_CHAMBER == 1000
    #define TEMP_SENSOR_CHAMBER_IS_CUSTOM 1
  #elif TEMP_SENSOR_CHAMBER == 998 || TEMP_SENSOR_CHAMBER == 999
    #define TEMP_SENSOR_CHAMBER_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_COOLER == -4
  #define TEMP_SENSOR_COOLER_IS_AD8495 1
#elif TEMP_SENSOR_COOLER == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_COOLER."
#elif TEMP_SENSOR_COOLER == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_COOLER."
#elif TEMP_SENSOR_COOLER == -1
  #define TEMP_SENSOR_COOLER_IS_AD595 1
#elif TEMP_SENSOR_COOLER == 998 || TEMP_SENSOR_COOLER == 999
  #define TEMP_SENSOR_COOLER_IS_DUMMY 1
#elif TEMP_SENSOR_COOLER > 0
  #define TEMP_SENSOR_COOLER_IS_THERMISTOR 1
  #if TEMP_SENSOR_COOLER == 1000
    #define TEMP_SENSOR_COOLER_IS_CUSTOM 1
  #endif
#endif

#if TEMP_SENSOR_PROBE == -4
  #define TEMP_SENSOR_PROBE_IS_AD8495 1
#elif TEMP_SENSOR_PROBE == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_PROBE."
#elif TEMP_SENSOR_PROBE == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_PROBE."
#elif TEMP_SENSOR_PROBE == -1
  #define TEMP_SENSOR_PROBE_IS_AD595 1
#elif TEMP_SENSOR_PROBE > 0
  #define TEMP_SENSOR_PROBE_IS_THERMISTOR 1
  #if TEMP_SENSOR_PROBE == 1000
    #define TEMP_SENSOR_PROBE_IS_CUSTOM 1
  #elif TEMP_SENSOR_PROBE == 998 || TEMP_SENSOR_PROBE == 999
    #define TEMP_SENSOR_PROBE_IS_DUMMY 1
  #endif
#endif

#if TEMP_SENSOR_BOARD == -4
  #define TEMP_SENSOR_BOARD_IS_AD8495 1
#elif TEMP_SENSOR_BOARD == -3
  #error "MAX31855 Thermocouples (-3) not supported for TEMP_SENSOR_BOARD."
#elif TEMP_SENSOR_BOARD == -2
  #error "MAX6675 Thermocouples (-2) not supported for TEMP_SENSOR_BOARD."
#elif TEMP_SENSOR_BOARD == -1
  #define TEMP_SENSOR_BOARD_IS_AD595 1
#elif TEMP_SENSOR_BOARD > 0
  #define TEMP_SENSOR_BOARD_IS_THERMISTOR 1
  #if TEMP_SENSOR_BOARD == 1000
    #define TEMP_SENSOR_BOARD_IS_CUSTOM 1
  #elif TEMP_SENSOR_BOARD == 998 || TEMP_SENSOR_BOARD == 999
    #define TEMP_SENSOR_BOARD_IS_DUMMY 1
  #endif
#endif

#if !ANY_THERMISTOR_IS(-1)
  #undef TEMP_SENSOR_AD595_GAIN
  #undef TEMP_SENSOR_AD595_OFFSET
#endif
#if !ANY_THERMISTOR_IS(-4)
  #undef TEMP_SENSOR_AD8495_GAIN
  #undef TEMP_SENSOR_AD8495_OFFSET
#endif
#if !ANY_THERMISTOR_IS(998)
  #undef DUMMY_THERMISTOR_998_VALUE
#endif
#if !ANY_THERMISTOR_IS(999)
  #undef DUMMY_THERMISTOR_999_VALUE
#endif
#if !ALL(HAS_X_AXIS, HAS_HOTEND)
  #undef AUTOTEMP
#endif
#if DISABLED(THERMAL_PROTECTION_HOTENDS)
  #undef THERMAL_PROTECTION_PERIOD
  #undef THERMAL_PROTECTION_HYSTERESIS
  #undef WATCH_TEMP_PERIOD
  #undef WATCH_TEMP_INCREASE
#endif
#if !ALL(HAS_HOTEND, PIDTEMP)
  #undef PID_PARAMS_PER_HOTEND
#endif

#if !TEMP_SENSOR_BED
  #undef BED_MINTEMP
  #undef BED_MAXTEMP
  #undef BED_OVERSHOOT
  #undef CONTROLLER_FAN_BED_HEATING
#endif
#if DISABLED(THERMAL_PROTECTION_BED)
  #undef THERMAL_PROTECTION_BED_PERIOD
  #undef THERMAL_PROTECTION_BED_HYSTERESIS
  #undef WATCH_BED_TEMP_PERIOD
  #undef WATCH_BED_TEMP_INCREASE
#endif

#if !TEMP_SENSOR_CHAMBER
  #undef CHAMBER_MINTEMP
  #undef CHAMBER_MAXTEMP
#endif
#if DISABLED(THERMAL_PROTECTION_CHAMBER)
  #undef THERMAL_PROTECTION_CHAMBER_PERIOD
  #undef THERMAL_PROTECTION_CHAMBER_HYSTERESIS
  #undef WATCH_CHAMBER_TEMP_PERIOD
  #undef WATCH_CHAMBER_TEMP_INCREASE
#endif

#if !TEMP_SENSOR_COOLER
  #undef COOLER_MINTEMP
  #undef COOLER_MAXTEMP
  #undef COOLER_OVERSHOOT
#endif
#if DISABLED(THERMAL_PROTECTION_COOLER)
  #undef THERMAL_PROTECTION_COOLER_PERIOD
  #undef THERMAL_PROTECTION_COOLER_HYSTERESIS
  #undef WATCH_COOLER_TEMP_PERIOD
  #undef WATCH_COOLER_TEMP_INCREASE
#endif

#if HAS_MULTI_EXTRUDER || HAS_MULTI_HOTEND || HAS_PRUSA_MMU2 || HAS_PRUSA_MMU3 || (ENABLED(MIXING_EXTRUDER) && MIXING_VIRTUAL_TOOLS > 1)
  #define HAS_TOOLCHANGE 1
#endif

#if ENABLED(MIXING_EXTRUDER) && (ENABLED(RETRACT_SYNC_MIXING) || ALL(FILAMENT_LOAD_UNLOAD_GCODES, FILAMENT_UNLOAD_ALL_EXTRUDERS))
  #define HAS_MIXER_SYNC_CHANNEL 1
#endif

#if ANY(DUAL_X_CARRIAGE, MULTI_NOZZLE_DUPLICATION)
  #define HAS_DUPLICATION_MODE 1
#endif

#if ENABLED(PRINTCOUNTER) && (SERVICE_INTERVAL_1 > 0 || SERVICE_INTERVAL_2 > 0 || SERVICE_INTERVAL_3 > 0)
  #define HAS_SERVICE_INTERVALS 1
#endif

#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #define HAS_FILAMENT_SENSOR 1
  #if NUM_RUNOUT_SENSORS > 1
    #define MULTI_FILAMENT_SENSOR 1
  #endif
  #ifdef FILAMENT_RUNOUT_DISTANCE_MM
    #define HAS_FILAMENT_RUNOUT_DISTANCE 1
  #endif
  #if ENABLED(MIXING_EXTRUDER)
    #define WATCH_ALL_RUNOUT_SENSORS
  #endif
#endif

#if ANY(PTC_PROBE, PTC_BED, PTC_HOTEND)
  #define HAS_PTC 1
#endif

// Let SD_FINISHED_RELEASECOMMAND stand in for SD_FINISHED_STEPPERRELEASE
#if ENABLED(SD_FINISHED_STEPPERRELEASE)
  #ifndef SD_FINISHED_RELEASECOMMAND
    #define SD_FINISHED_RELEASECOMMAND "M84" // planner.finish_and_disable()
  #endif
#else
  #undef SD_FINISHED_RELEASECOMMAND
#endif

#if ENABLED(NO_SD_AUTOSTART)
  #undef MENU_ADDAUTOSTART
#endif

#if ANY(HAS_MEDIA, SET_PROGRESS_MANUALLY)
  #define HAS_PRINT_PROGRESS 1
#endif

#if !ALL(HAS_MARLINUI_MENU, HAS_MEDIA)
  #undef SD_MENU_CONFIRM_START
#endif

#if DISABLED(SET_PROGRESS_MANUALLY)
  #undef SET_REMAINING_TIME
  #undef SET_INTERACTION_TIME
  #undef M73_REPORT
#endif

#if ANY(HAS_MARLINUI_MENU, ULTIPANEL_FEEDMULTIPLY, SOFT_RESET_ON_KILL)
  #define HAS_ENCODER_ACTION 1
#endif

#if ENABLED(ENCODER_RATE_MULTIPLIER)
  #ifndef ENCODER_5X_STEPS_PER_SEC
    #define ENCODER_5X_STEPS_PER_SEC 0
  #endif
  #ifndef ENCODER_10X_STEPS_PER_SEC
    #define ENCODER_10X_STEPS_PER_SEC 0
  #endif
  #ifndef ENCODER_100X_STEPS_PER_SEC
    #define ENCODER_100X_STEPS_PER_SEC 0
  #endif
  #if !((HAS_MARLINUI_MENU || HAS_DWIN_E3V2) && (ENCODER_5X_STEPS_PER_SEC || ENCODER_10X_STEPS_PER_SEC || ENCODER_100X_STEPS_PER_SEC))
    #undef ENCODER_RATE_MULTIPLIER
    #undef ENCODER_5X_STEPS_PER_SEC
    #undef ENCODER_10X_STEPS_PER_SEC
    #undef ENCODER_100X_STEPS_PER_SEC
  #endif
#endif

#if STATUS_MESSAGE_TIMEOUT_SEC > 0
  #define HAS_STATUS_MESSAGE_TIMEOUT 1
#endif

#if HAS_MEDIA && SD_PROCEDURE_DEPTH
  #define HAS_MEDIA_SUBCALLS 1
#endif

#if ANY(SHOW_ELAPSED_TIME, SHOW_REMAINING_TIME, SHOW_INTERACTION_TIME)
  #define HAS_TIME_DISPLAY 1
#endif

#if ANY(SHOW_PROGRESS_PERCENT, HAS_TIME_DISPLAY) && !HAS_GRAPHICAL_TFT
  #define HAS_EXTRA_PROGRESS 1
#endif

#if HAS_PRINT_PROGRESS && ANY(PRINT_PROGRESS_SHOW_DECIMALS, SHOW_REMAINING_TIME)
  #define HAS_PRINT_PROGRESS_PERMYRIAD 1
#endif

#if ANY(MARLIN_BRICKOUT, MARLIN_INVADERS, MARLIN_SNAKE, MARLIN_MAZE)
  #define HAS_GAMES 1
  #if MANY(MARLIN_BRICKOUT, MARLIN_INVADERS, MARLIN_SNAKE, MARLIN_MAZE)
    #define HAS_GAME_MENU 1
  #endif
#endif

#if ANY(FWRETRACT, HAS_LEVELING, SKEW_CORRECTION)
  #define HAS_POSITION_MODIFIERS 1
#endif

#if ANY(X_DUAL_ENDSTOPS, Y_DUAL_ENDSTOPS, Z_MULTI_ENDSTOPS)
  #define HAS_EXTRA_ENDSTOPS 1
#endif
#if ANY(MIN_SOFTWARE_ENDSTOPS, MAX_SOFTWARE_ENDSTOPS)
  #define HAS_SOFTWARE_ENDSTOPS 1
#endif
#if ANY(EXTENSIBLE_UI, IS_NEWPANEL, EMERGENCY_PARSER, HAS_ADC_BUTTONS, HAS_DWIN_E3V2)
  #define HAS_RESUME_CONTINUE 1
#endif

#if ANY(BLINKM, RGB_LED, RGBW_LED, PCA9632, PCA9533, NEOPIXEL_LED)
  #define HAS_COLOR_LEDS 1
#else
  #undef LED_POWEROFF_TIMEOUT
#endif
#if ALL(HAS_RESUME_CONTINUE, PRINTER_EVENT_LEDS, HAS_MEDIA)
  #define HAS_LEDS_OFF_FLAG 1
#endif
#ifdef DISPLAY_SLEEP_MINUTES
  #define HAS_DISPLAY_SLEEP 1
#endif
#ifdef LCD_BACKLIGHT_TIMEOUT_MINS
  #define HAS_BACKLIGHT_TIMEOUT 1
#endif

#if ANY(DIGIPOT_MCP4018, DIGIPOT_MCP4451)
  #define HAS_MOTOR_CURRENT_I2C 1
#endif

#if ENABLED(DUAL_X_CARRIAGE)
  #ifndef INVERT_X2_DIR
    #define INVERT_X2_DIR INVERT_X_DIR
  #endif
#endif

// X2 but not IDEX => Dual Synchronized X Steppers
#if defined(X2_DRIVER_TYPE) && DISABLED(DUAL_X_CARRIAGE)
  #define HAS_SYNCED_X_STEPPERS 1
#endif

// Y2 Stepper => Dual Synchronized Y Steppers
#ifdef Y2_DRIVER_TYPE
  #define HAS_SYNCED_Y_STEPPERS 1
#endif

// Multiple Z steppers
#ifdef INVERT_Z_DIR
  #if NUM_Z_STEPPERS >= 2 && !defined(INVERT_Z2_DIR)
    #define INVERT_Z2_DIR INVERT_Z_DIR
    #if NUM_Z_STEPPERS >= 3 && !defined(INVERT_Z3_DIR)
      #define INVERT_Z3_DIR INVERT_Z_DIR
      #if NUM_Z_STEPPERS >= 4 && !defined(INVERT_Z4_DIR)
        #define INVERT_Z4_DIR INVERT_Z_DIR
      #endif
    #endif
  #endif
#endif
#if NUM_Z_STEPPERS < 4
  #undef INVERT_Z4_VS_Z_DIR
  #if NUM_Z_STEPPERS < 3
    #undef INVERT_Z3_VS_Z_DIR
    #if NUM_Z_STEPPERS < 2
      #undef INVERT_Z2_VS_Z_DIR
    #endif
  #endif
#endif

// Z Stepper Auto-align
#if ENABLED(Z_STEPPER_AUTO_ALIGN)
  #ifdef Z_STEPPER_ALIGN_STEPPER_XY
    #define HAS_Z_STEPPER_ALIGN_STEPPER_XY 1
    #undef Z_STEPPER_ALIGN_AMP
  #endif
  #ifndef Z_STEPPER_ALIGN_AMP
    #define Z_STEPPER_ALIGN_AMP 1.0
  #endif
#endif

//
// Spindle/Laser power display types
// Defined here so sanity checks can use them
//
#if ANY(SPINDLE_FEATURE, LASER_FEATURE)
  #define HAS_CUTTER 1
  #define _CUTTER_POWER_PWM255  1
  #define _CUTTER_POWER_PERCENT 2
  #define _CUTTER_POWER_RPM     3
  #define _CUTTER_POWER(V)      _CAT(_CUTTER_POWER_, V)
  #define CUTTER_UNIT_IS(V)    (_CUTTER_POWER(CUTTER_POWER_UNIT) == _CUTTER_POWER(V))
#endif

#if !defined(__AVR__) || !defined(USBCON)
  // Define constants and variables for buffering serial data.
  // Use only 0 or powers of 2 greater than 1
  // : [0, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, ...]
  #ifndef RX_BUFFER_SIZE
    #define RX_BUFFER_SIZE 128
  #endif
  // 256 is the max TX buffer limit due to uint8_t head and tail
  // : [0, 4, 8, 16, 32, 64, 128, 256]
  #ifndef TX_BUFFER_SIZE
    #define TX_BUFFER_SIZE 32
  #endif
#else
  // SERIAL_XON_XOFF not supported on USB-native devices
  #undef SERIAL_XON_XOFF
#endif

#if ENABLED(HOST_PROMPT_SUPPORT) && DISABLED(EMERGENCY_PARSER)
  #define HAS_GCODE_M876 1
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #ifndef ACTION_ON_PAUSE
    #define ACTION_ON_PAUSE   "pause"
  #endif
  #ifndef ACTION_ON_PAUSED
    #define ACTION_ON_PAUSED  "paused"
  #endif
  #ifndef ACTION_ON_RESUME
    #define ACTION_ON_RESUME  "resume"
  #endif
  #ifndef ACTION_ON_RESUMED
    #define ACTION_ON_RESUMED "resumed"
  #endif
  #ifndef ACTION_ON_CANCEL
    #define ACTION_ON_CANCEL  "cancel"
  #endif
  #ifndef ACTION_ON_START
    #define ACTION_ON_START   "start"
  #endif
  #ifndef ACTION_ON_KILL
    #define ACTION_ON_KILL    "poweroff"
  #endif
  #ifndef SHUTDOWN_ACTION
    #define SHUTDOWN_ACTION   "shutdown"
  #endif
  #if HAS_FILAMENT_SENSOR
    #ifndef ACTION_ON_FILAMENT_RUNOUT
      #define ACTION_ON_FILAMENT_RUNOUT "filament_runout"
    #endif
    #ifndef ACTION_REASON_ON_FILAMENT_RUNOUT
      #define ACTION_REASON_ON_FILAMENT_RUNOUT "filament_runout"
    #endif
  #endif
  #if ENABLED(G29_RETRY_AND_RECOVER)
    #ifndef ACTION_ON_G29_RECOVER
      #define ACTION_ON_G29_RECOVER "probe_rewipe"
    #endif
    #ifndef ACTION_ON_G29_FAILURE
      #define ACTION_ON_G29_FAILURE "probe_failed"
    #endif
  #endif
#endif

#if ANY(FYSETC_MINI_12864_2_1, FYSETC_242_OLED_12864)
  #ifndef LED_USER_PRESET_GREEN
    #define LED_USER_PRESET_GREEN      128
  #endif
  #ifndef LED_USER_PRESET_BLUE
    #define LED_USER_PRESET_BLUE         0
  #endif
  #ifndef LED_USER_PRESET_BRIGHTNESS
    #define LED_USER_PRESET_BRIGHTNESS 255
  #endif
#endif

// Set defaults for unspecified LED user colors
#if ENABLED(LED_CONTROL_MENU)
  #ifndef LED_USER_PRESET_RED
    #define LED_USER_PRESET_RED       255
  #endif
  #ifndef LED_USER_PRESET_GREEN
    #define LED_USER_PRESET_GREEN     255
  #endif
  #ifndef LED_USER_PRESET_BLUE
    #define LED_USER_PRESET_BLUE      255
  #endif
  #ifndef LED_USER_PRESET_WHITE
    #define LED_USER_PRESET_WHITE     0
  #endif
  #ifndef LED_USER_PRESET_BRIGHTNESS
    #ifdef NEOPIXEL_BRIGHTNESS
      #define LED_USER_PRESET_BRIGHTNESS NEOPIXEL_BRIGHTNESS
    #else
      #define LED_USER_PRESET_BRIGHTNESS 255
    #endif
  #endif
#endif

#if ALL(LED_CONTROL_MENU, NEOPIXEL2_SEPARATE)
  #ifndef LED2_USER_PRESET_RED
    #define LED2_USER_PRESET_RED       255
  #endif
  #ifndef LED2_USER_PRESET_GREEN
    #define LED2_USER_PRESET_GREEN     255
  #endif
  #ifndef LED2_USER_PRESET_BLUE
    #define LED2_USER_PRESET_BLUE      255
  #endif
  #ifndef LED2_USER_PRESET_WHITE
    #define LED2_USER_PRESET_WHITE     0
  #endif
  #ifndef LED2_USER_PRESET_BRIGHTNESS
    #ifdef NEOPIXEL2_BRIGHTNESS
      #define LED2_USER_PRESET_BRIGHTNESS NEOPIXEL2_BRIGHTNESS
    #else
      #define LED2_USER_PRESET_BRIGHTNESS 255
    #endif
  #endif
#endif

// Full Touch Screen needs 'tft/xpt2046'
#if ANY(TFT_TOUCH_DEVICE_XPT2046, HAS_TFT_LVGL_UI)
  #define HAS_TFT_XPT2046 1
#endif

// Touch Screen or "Touch Buttons" need XPT2046 pins
// but they use different components
#if HAS_TFT_XPT2046 || HAS_RES_TOUCH_BUTTONS
  #define NEED_TOUCH_PINS 1
#endif

// Extensible UI pin mapping for RepRapDiscount
#if ENABLED(TOUCH_UI_FTDI_EVE) && ANY(AO_EXP1_PINMAP, AO_EXP2_PINMAP, CR10_TFT_PINMAP)
  #define TOUCH_UI_ULTIPANEL 1
#endif

// Poll-based jogging for joystick and other devices
#if ENABLED(JOYSTICK)
  #define POLL_JOG
#endif

#ifndef HOMING_BUMP_MM
  #define HOMING_BUMP_MM { 0, 0, 0 }
#endif

#if HAS_USB_FLASH_DRIVE && NONE(USE_OTG_USB_HOST, USE_UHS3_USB)
  #define USE_UHS2_USB
#endif

/**
 * Driver Timings (in nanoseconds)
 * NOTE: Driver timing order is longest-to-shortest duration.
 *       Preserve this ordering when adding new drivers.
 */
#ifndef MINIMUM_STEPPER_POST_DIR_DELAY
  #if HAS_DRIVER(TB6560)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 15000
  #elif HAS_DRIVER(TB6600)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 1500
  #elif HAS_DRIVER(DRV8825)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 650
  #elif HAS_DRIVER(LV8729)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 500
  #elif HAS_DRIVER(A5984)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 400
  #elif HAS_DRIVER(A4988)
    #define MINIMUM_STEPPER_POST_DIR_DELAY 200
  #elif HAS_TRINAMIC_CONFIG || HAS_TRINAMIC_STANDALONE
    #define MINIMUM_STEPPER_POST_DIR_DELAY 100
  #else
    #define MINIMUM_STEPPER_POST_DIR_DELAY 0   // Expect at least 10µS since one Stepper ISR must transpire
  #endif
#endif

#ifndef MINIMUM_STEPPER_PRE_DIR_DELAY
  #define MINIMUM_STEPPER_PRE_DIR_DELAY MINIMUM_STEPPER_POST_DIR_DELAY
#endif

#ifndef MINIMUM_STEPPER_PULSE_NS
  #if HAS_DRIVER(TB6560)
    #define MINIMUM_STEPPER_PULSE_NS 30000
  #elif HAS_DRIVER(TB6600)
    #define MINIMUM_STEPPER_PULSE_NS 3000
  #elif HAS_DRIVER(DRV8825)
    #define MINIMUM_STEPPER_PULSE_NS 2000
  #elif HAS_DRIVER(A4988) || HAS_DRIVER(A5984)
    #define MINIMUM_STEPPER_PULSE_NS 1000
  #elif HAS_DRIVER(LV8729)
    #define MINIMUM_STEPPER_PULSE_NS 500
  #elif HAS_TRINAMIC_CONFIG || HAS_TRINAMIC_STANDALONE
    #define MINIMUM_STEPPER_PULSE_NS 100
  #else
    // Expecting MAXIMUM_STEPPER_RATE to be defined
  #endif
#endif

#ifndef MAXIMUM_STEPPER_RATE
  #if HAS_DRIVER(TB6560)
    #define MAXIMUM_STEPPER_RATE 15000
  #elif HAS_DRIVER(TB6600)
    #define MAXIMUM_STEPPER_RATE 150000
  #elif HAS_DRIVER(DRV8825)
    #define MAXIMUM_STEPPER_RATE 250000
  #elif HAS_DRIVER(A4988)
    #define MAXIMUM_STEPPER_RATE 500000
  #elif HAS_DRIVER(LV8729)
    #define MAXIMUM_STEPPER_RATE 1000000
  #elif HAS_TRINAMIC_CONFIG || HAS_TRINAMIC_STANDALONE
    #define MAXIMUM_STEPPER_RATE 5000000
  #else
    // Expecting MINIMUM_STEPPER_PULSE_NS to be defined
  #endif
#endif

// Test for edge stepping on any axis
#define AXIS_HAS_DEDGE(A) (ENABLED(EDGE_STEPPING) && AXIS_IS_TMC(A))

#if ENABLED(DIRECT_STEPPING)
  #ifndef STEPPER_PAGES
    #define STEPPER_PAGES 16
  #endif
  #ifndef STEPPER_PAGE_FORMAT
    #define STEPPER_PAGE_FORMAT SP_4x2_256
  #endif
  #ifndef PAGE_MANAGER
    #define PAGE_MANAGER SerialPageManager
  #endif
#endif

#if    defined(SAFE_BED_LEVELING_START_X) || defined(SAFE_BED_LEVELING_START_Y) || defined(SAFE_BED_LEVELING_START_Z) \
    || defined(SAFE_BED_LEVELING_START_I) || defined(SAFE_BED_LEVELING_START_J) || defined(SAFE_BED_LEVELING_START_K) \
    || defined(SAFE_BED_LEVELING_START_U) || defined(SAFE_BED_LEVELING_START_V) || defined(SAFE_BED_LEVELING_START_W)
  #define HAS_SAFE_BED_LEVELING 1
#endif

//
// SD Card connection methods
// Defined here so pins and sanity checks can use them
//
#if HAS_MEDIA
  #define _SDCARD_LCD          1
  #define _SDCARD_ONBOARD      2
  #define _SDCARD_CUSTOM_CABLE 3
  #define _SDCARD_ID(V) _CAT(_SDCARD_, V)
  #define SD_CONNECTION_IS(V) (_SDCARD_ID(SDCARD_CONNECTION) == _SDCARD_ID(V))
#else
  #define SD_CONNECTION_IS(...) 0
  #undef SD_ABORT_ON_ENDSTOP_HIT
#endif

// Power Monitor sensors
#if ANY(POWER_MONITOR_CURRENT, POWER_MONITOR_VOLTAGE)
  #define HAS_POWER_MONITOR 1
  #if ENABLED(POWER_MONITOR_CURRENT) && (ENABLED(POWER_MONITOR_VOLTAGE) || defined(POWER_MONITOR_FIXED_VOLTAGE))
    #define HAS_POWER_MONITOR_WATTS 1
  #endif
#endif

// Flag if an EEPROM type is pre-selected
#if ENABLED(EEPROM_SETTINGS) && NONE(I2C_EEPROM, SPI_EEPROM, QSPI_EEPROM, FLASH_EEPROM_EMULATION, SRAM_EEPROM_EMULATION, SDCARD_EEPROM_EMULATION)
  #define NO_EEPROM_SELECTED 1
#endif

// Flags for Case Light having a color property or a single pin
#if ENABLED(CASE_LIGHT_ENABLE)
  #if ANY(CASE_LIGHT_USE_NEOPIXEL, CASE_LIGHT_USE_RGB_LED)
    #define CASE_LIGHT_IS_COLOR_LED 1
  #else
    #define NEED_CASE_LIGHT_PIN 1
  #endif
#endif

// Flags for Case Light having a brightness property
#if ENABLED(CASE_LIGHT_ENABLE) && (NONE(CASE_LIGHT_NO_BRIGHTNESS, CASE_LIGHT_IS_COLOR_LED) || ENABLED(CASE_LIGHT_USE_NEOPIXEL))
  #define CASELIGHT_USES_BRIGHTNESS 1
#endif

// Flag whether least_squares_fit.cpp is used
#if ANY(AUTO_BED_LEVELING_UBL, AUTO_BED_LEVELING_LINEAR, HAS_Z_STEPPER_ALIGN_STEPPER_XY)
  #define NEED_LSF 1
#endif

#if ALL(HAS_TFT_LVGL_UI, CUSTOM_MENU_MAIN)
  #define _HAS_1(N) (defined(MAIN_MENU_ITEM_##N##_DESC) && defined(MAIN_MENU_ITEM_##N##_GCODE))
  #define HAS_USER_ITEM(V...) DO(HAS,||,V)
#else
  #define HAS_USER_ITEM(...) 0
#endif

/**
 * LCD_SERIAL_PORT must be defined ahead of HAL.h and
 * currently HAL.h must be included ahead of pins.h.
 */
#if LCD_IS_SERIAL_HOST && !defined(LCD_SERIAL_PORT)
  #if MB(MKS_MONSTER8_V1, BTT_SKR_MINI_E3_V1_0, BTT_SKR_MINI_E3_V1_2, BTT_SKR_MINI_E3_V2_0, BTT_SKR_MINI_E3_V3_0, BTT_SKR_MINI_E3_V3_0_1, BTT_SKR_E3_TURBO, BTT_OCTOPUS_V1_1, BTT_SKR_V3_0, BTT_SKR_V3_0_EZ, AQUILA_V101)

    #define LCD_SERIAL_PORT 1
  #elif MB(CREALITY_V24S1_301, CREALITY_V24S1_301F4, CREALITY_F401RE, CREALITY_V423, CREALITY_CR4NTXXC10, CREALITY_CR4NS, MKS_ROBIN, PANOWIN_CUTLASS, KODAMA_BARDO)
    #define LCD_SERIAL_PORT 2
  #else
    #define LCD_SERIAL_PORT 3
  #endif
  #ifdef LCD_SERIAL_PORT
    #define AUTO_ASSIGNED_LCD_SERIAL 1
  #endif
#endif

#if !HAS_MULTI_SERIAL
  #undef MEATPACK_ON_SERIAL_PORT_2
#endif
#if ANY(MEATPACK_ON_SERIAL_PORT_1, MEATPACK_ON_SERIAL_PORT_2)
  #define HAS_MEATPACK 1
#endif

// AVR are (usually) too limited in resources to store the configuration into the binary
#if ENABLED(CONFIGURATION_EMBEDDING) && !defined(FORCE_CONFIG_EMBED) && (defined(__AVR__) || !HAS_MEDIA || ANY(SDCARD_READONLY, DISABLE_M503))
  #undef CONFIGURATION_EMBEDDING
  #define CANNOT_EMBED_CONFIGURATION defined(__AVR__)
#endif

// Input shaping
#if ANY(INPUT_SHAPING_X, INPUT_SHAPING_Y, INPUT_SHAPING_Z)
  #define HAS_ZV_SHAPING 1
#endif

// FT Motion unified window and batch size
#if ENABLED(FT_MOTION)
  #if HAS_X_AXIS
    #define HAS_FTM_SHAPING 1
  #endif
  #if ENABLED(FTM_UNIFIED_BWS)
    #define FTM_WINDOW_SIZE FTM_BW_SIZE
    #define FTM_BATCH_SIZE  FTM_BW_SIZE
  #endif
  #if ANY(BIQU_MICROPROBE_V1, BIQU_MICROPROBE_V2)
    #define FT_MOTION_DISABLE_FOR_PROBING 1
  #endif
#endif

// Multi-Stepping Limit
#ifndef MULTISTEPPING_LIMIT
  #define MULTISTEPPING_LIMIT 128
  #define MULTISTEPPING_LIMIT_WARNING 1
#endif

// One redundant cooling fan by default
#if defined(REDUNDANT_PART_COOLING_FAN) && !defined(NUM_REDUNDANT_FANS)
  #define NUM_REDUNDANT_FANS 1
#endif

// Power-Loss Recovery
#if ENABLED(POWER_LOSS_RECOVERY)
  #ifdef PLR_BED_THRESHOLD
    #define HAS_PLR_BED_THRESHOLD 1
  #endif
  #if ANY(DWIN_CREALITY_LCD, DWIN_LCD_PROUI)
    #define HAS_PLR_UI_FLAG 1   // recovery.ui_flag_resume
  #endif
#endif

// Toolchange Event G-code
#if !HAS_MULTI_EXTRUDER || !(defined(EVENT_GCODE_TOOLCHANGE_T0) || defined(EVENT_GCODE_TOOLCHANGE_T1) || defined(EVENT_GCODE_TOOLCHANGE_T2) || defined(EVENT_GCODE_TOOLCHANGE_T3) || defined(EVENT_GCODE_TOOLCHANGE_T4) || defined(EVENT_GCODE_TOOLCHANGE_T5) || defined(EVENT_GCODE_TOOLCHANGE_T6) || defined(EVENT_GCODE_TOOLCHANGE_T7))
  #undef TC_GCODE_USE_GLOBAL_X
  #undef TC_GCODE_USE_GLOBAL_Y
  #undef TC_GCODE_USE_GLOBAL_Z
#endif

// Clean up for TOOLCHANGE_MIGRATION_FEATURE
#if DISABLED(TOOLCHANGE_MIGRATION_FEATURE)
  #undef MIGRATION_ZRAISE
  #undef MIGRATION_FS_EXTRA_PRIME
  #undef MIGRATION_FS_WIPE_RETRACT
  #undef MIGRATION_FS_FAN_SPEED
  #undef MIGRATION_FS_FAN_TIME
  #undef TOOLCHANGE_MIGRATION_DO_PARK
#endif

// Clean up for TOOLCHANGE_PARK
#if DISABLED(TOOLCHANGE_PARK)
  #undef TOOLCHANGE_PARK_XY
  #undef TOOLCHANGE_PARK_XY_FEEDRATE
  #undef TOOLCHANGE_PARK_X_ONLY
  #undef TOOLCHANGE_PARK_Y_ONLY
  #undef TOOLCHANGE_MIGRATION_DO_PARK
#endif

// Clean up if only mm units are used
#if DISABLED(INCH_MODE_SUPPORT)
  #undef MANUAL_MOVE_DISTANCE_IN
#endif

// Clean up if no rotational axes exist
#if !HAS_ROTATIONAL_AXES
  #undef MANUAL_MOVE_DISTANCE_DEG
#endif

// Only report "Not SD printing" when the state changes
// To get legacy behavior define AUTO_REPORT_SD_STATUS 2
#ifdef AUTO_REPORT_SD_STATUS
  #if ENABLED(AUTO_REPORT_SD_STATUS) // Not blank, 1, or true
    #define QUIETER_AUTO_REPORT_SD_STATUS
  #endif
  #undef AUTO_REPORT_SD_STATUS
  #define AUTO_REPORT_SD_STATUS
#endif
