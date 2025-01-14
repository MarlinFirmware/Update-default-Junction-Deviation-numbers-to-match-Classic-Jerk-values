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

/**
 * Warnings.cpp
 * Test configuration values and give warnings at compile-time.
 */

#include "MarlinConfig.h"

//
// Warnings! Located here so they will appear just once in the build output.
//

// static_warning works like a static_assert but only emits a (messy) warning.
#ifdef __GNUC__
  namespace mfwarn {
    struct true_type {};
    struct false_type {};
    template <int test> struct converter : public true_type {};
    template <> struct converter<0> : public false_type {};
  }
  #define static_warning(cond, msg) \
    struct CAT(static_warning, __LINE__) { \
      void _(::mfwarn::false_type const&) __attribute__((deprecated(msg))) {}; \
      void _(::mfwarn::true_type const&) {}; \
      CAT(static_warning, __LINE__)() {_(::mfwarn::converter<(cond)>());} \
    }
#else
  #define static_warning(...)
#endif

#if ENABLED(MARLIN_DEV_MODE)
  #warning "WARNING! Disable MARLIN_DEV_MODE for the final build!"
  #ifdef __LONG_MAX__
    #if __LONG_MAX__ > __INT_MAX__
      #warning "The 'long' type is larger than the 'int' type on this platform."
    #else
      #warning "The 'long' type is the same as the 'int' type on this platform."
    #endif
  #endif
#endif

#if !USE_STD_CONFIGS
  #if __has_include("../../Configuration.h")
    #define HAS_IGNORED_CONFIGS
  #elif __has_include("../../Configuration_adv.h")
    #define HAS_IGNORED_CONFIGS
  #endif
  #ifdef HAS_IGNORED_CONFIGS
    #warning "Configuration.h and Configuration_adv.h are being ignored, overridden by Config.h."
  #endif
#endif

#if CONFIG_EXPORT % 100 == 5
  #warning "Rename 'Config-export.h' to 'Config.h' to override Configuration.h and Configuration_adv.h."
#endif

#if DISABLED(DEBUG_FLAGS_GCODE)
  #warning "DEBUG_FLAGS_GCODE is recommended if you have space. Some hosts rely on it."
#endif

#if DISABLED(CAPABILITIES_REPORT)
  #warning "CAPABILITIES_REPORT is recommended if you have space. Some hosts rely on it."
#endif

#if ENABLED(LA_DEBUG)
  #warning "WARNING! Disable LA_DEBUG for the final build!"
#endif

#if NUM_AXES_WARNING
  #warning "Note: NUM_AXES is now based on the *_DRIVER_TYPE settings so you can remove NUM_AXES from Configuration.h."
#endif

// Safety Features
#if DISABLED(USE_WATCHDOG)
  #warning "Safety Alert! Enable USE_WATCHDOG for the final build!"
#endif
#if HAS_HOTEND && DISABLED(THERMAL_PROTECTION_HOTENDS)
  #warning "Safety Alert! Enable THERMAL_PROTECTION_HOTENDS for the final build!"
#endif
#if HAS_HEATED_BED && DISABLED(THERMAL_PROTECTION_BED)
  #warning "Safety Alert! Enable THERMAL_PROTECTION_BED for the final build!"
#endif
#if HAS_HEATED_CHAMBER && DISABLED(THERMAL_PROTECTION_CHAMBER)
  #warning "Safety Alert! Enable THERMAL_PROTECTION_CHAMBER for the final build!"
#endif
#if HAS_COOLER && DISABLED(THERMAL_PROTECTION_COOLER)
  #warning "Safety Alert! Enable THERMAL_PROTECTION_COOLER for the final build!"
#endif
#if ENABLED(IGNORE_THERMOCOUPLE_ERRORS)
  #warning "Safety Alert! Disable IGNORE_THERMOCOUPLE_ERRORS for the final build!"
#endif
#if ANY_THERMISTOR_IS(998) || ANY_THERMISTOR_IS(999)
  #warning "Warning! Don't use dummy thermistors (998/999) for final build!"
#endif

#if NONE(HAS_RESUME_CONTINUE, HOST_PROMPT_SUPPORT, UNIT_TEST, NO_USER_FEEDBACK_WARNING)
  #warning "Your Configuration provides no method to acquire user feedback! (Define NO_USER_FEEDBACK_WARNING to suppress this warning.)"
#endif

#if MB(DUE3DOM_MINI) && PIN_EXISTS(TEMP_2) && !TEMP_SENSOR_BOARD
  #warning "Onboard temperature sensor for BOARD_DUE3DOM_MINI has moved from TEMP_SENSOR_2 (TEMP_2_PIN) to TEMP_SENSOR_BOARD (TEMP_BOARD_PIN)."
#elif MB(BTT_SKR_E3_TURBO) && PIN_EXISTS(TEMP_2) && !TEMP_SENSOR_BOARD
  #warning "Onboard temperature sensor for BOARD_BTT_SKR_E3_TURBO has moved from TEMP_SENSOR_2 (TEMP_2_PIN) to TEMP_SENSOR_BOARD (TEMP_BOARD_PIN)."
#endif

#ifndef NO_AUTO_ASSIGN_WARNING

  #if AUTO_ASSIGNED_LCD_SERIAL
    #warning "Note: Auto-assigned LCD_SERIAL_PORT. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif

  #if AUTO_ASSIGNED_X2_STEPPER
    #warning "Note: Auto-assigned X2 STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_X2_MS1
    #warning "Note: Auto-assigned X2_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_X2_MS2
    #warning "Note: Auto-assigned X2_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_X2_MS3
    #warning "Note: Auto-assigned X2_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_X2_CS
    #warning "Note: Auto-assigned X2_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_X2_DIAG
    #if X2_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif X2_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned X2 sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_Y2_STEPPER
    #warning "Note: Auto-assigned Y2 STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Y2_MS1
    #warning "Note: Auto-assigned Y2_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Y2_MS2
    #warning "Note: Auto-assigned Y2_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Y2_MS3
    #warning "Note: Auto-assigned Y2_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Y2_CS
    #warning "Note: Auto-assigned Y2_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Y2_DIAG
    #if Y2_USE_ENDSTOP == X_MIN_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_USE_ENDSTOP == Y_MIN_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_USE_ENDSTOP == Z_MIN_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Y2_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned Y2 sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_Z2_STEPPER
    #warning "Note: Auto-assigned Z2 STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z2_MS1
    #warning "Note: Auto-assigned Z2_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z2_MS2
    #warning "Note: Auto-assigned Z2_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z2_MS3
    #warning "Note: Auto-assigned Z2_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z2_CS
    #warning "Note: Auto-assigned Z2_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z2_DIAG
    #if Z2_USE_ENDSTOP == X_MIN_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_USE_ENDSTOP == Y_MIN_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_USE_ENDSTOP == Z_MIN_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z2_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned Z2 sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_Z3_STEPPER
    #warning "Note: Auto-assigned Z3 STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z3_CS
    #warning "Note: Auto-assigned Z3_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z3_MS1
    #warning "Note: Auto-assigned Z3_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z3_MS2
    #warning "Note: Auto-assigned Z3_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z3_MS3
    #warning "Note: Auto-assigned Z3_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z3_DIAG
    #if Z3_USE_ENDSTOP == X_MIN_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_USE_ENDSTOP == Y_MIN_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_USE_ENDSTOP == Z_MIN_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z3_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned Z3 sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_Z4_STEPPER
    #warning "Note: Auto-assigned Z4 STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z4_CS
    #warning "Note: Auto-assigned Z4_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z4_MS1
    #warning "Note: Auto-assigned Z4_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z4_MS2
    #warning "Note: Auto-assigned Z4_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z4_MS3
    #warning "Note: Auto-assigned Z4_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_Z4_DIAG
    #if Z4_USE_ENDSTOP == X_MIN_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_USE_ENDSTOP == Y_MIN_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_USE_ENDSTOP == Z_MIN_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif Z4_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned Z4 sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_I_STEPPER
    #warning "Note: Auto-assigned I STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_I_CS
    #warning "Note: Auto-assigned I_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_I_MS1
    #warning "Note: Auto-assigned I_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_I_MS2
    #warning "Note: Auto-assigned I_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_I_MS3
    #warning "Note: Auto-assigned I_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_I_DIAG
    #if I_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned I sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned I sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned I sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned I sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif I_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned I sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_J_STEPPER
    #warning "Note: Auto-assigned J STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_J_CS
    #warning "Note: Auto-assigned J_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_J_MS1
    #warning "Note: Auto-assigned J_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_J_MS2
    #warning "Note: Auto-assigned J_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_J_MS3
    #warning "Note: Auto-assigned J_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_J_DIAG
    #if J_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned J sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned J sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned J sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned J sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif J_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned J sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_K_STEPPER
    #warning "Note: Auto-assigned K STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_K_CS
    #warning "Note: Auto-assigned K_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_K_MS1
    #warning "Note: Auto-assigned K_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_K_MS2
    #warning "Note: Auto-assigned K_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_K_MS3
    #warning "Note: Auto-assigned K_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_K_DIAG
    #if K_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned K sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned K sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == X_STOP_PIN
      #warning "Note: Auto-assigned K sensorless endstop to X_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Y_STOP_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Y_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Z_STOP_PIN
      #warning "Note: Auto-assigned K sensorless endstop to Z_STOP_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned K sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif

  #if AUTO_ASSIGNED_U_STEPPER
    #warning "Note: Auto-assigned U STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_U_CS
    #warning "Note: Auto-assigned U_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_U_MS1
    #warning "Note: Auto-assigned U_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_U_MS2
    #warning "Note: Auto-assigned U_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_U_MS3
    #warning "Note: Auto-assigned U_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_U_DIAG
    #if U_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned U sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned U sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif K_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned U sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned U sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned U sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned U sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif U_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned U sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_V_STEPPER
    #warning "Note: Auto-assigned V STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_V_CS
    #warning "Note: Auto-assigned V_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_V_MS1
    #warning "Note: Auto-assigned V_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_V_MS2
    #warning "Note: Auto-assigned V_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_V_MS3
    #warning "Note: Auto-assigned V_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_V_DIAG
    #if V_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned V sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned V sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned V sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned V sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned V sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned V sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif V_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned V sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif
  #if AUTO_ASSIGNED_W_STEPPER
    #warning "Note: Auto-assigned W STEP/DIR/ENABLE_PINs to unused En_STEP/DIR/ENABLE_PINs. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_W_CS
    #warning "Note: Auto-assigned W_CS_PIN to an unused En_CS_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_W_MS1
    #warning "Note: Auto-assigned W_MS1_PIN to an unused En_MS1_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_W_MS2
    #warning "Note: Auto-assigned W_MS2_PIN to an unused En_MS2_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_W_MS3
    #warning "Note: Auto-assigned W_MS3_PIN to an unused En_MS3_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif
  #if AUTO_ASSIGNED_W_DIAG
    #if W_STOP_PIN == X_MIN_PIN
      #warning "Note: Auto-assigned W sensorless endstop to X_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == X_MAX_PIN
      #warning "Note: Auto-assigned W sensorless endstop to X_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == Y_MIN_PIN
      #warning "Note: Auto-assigned W sensorless endstop to Y_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == Y_MAX_PIN
      #warning "Note: Auto-assigned W sensorless endstop to Y_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == Z_MIN_PIN
      #warning "Note: Auto-assigned W sensorless endstop to Z_MIN_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == Z_MAX_PIN
      #warning "Note: Auto-assigned W sensorless endstop to Z_MAX_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E0_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E0_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E1_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E1_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E2_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E2_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E3_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E3_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E4_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E4_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E5_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E5_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E6_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E6_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #elif W_STOP_PIN == E7_DIAG_PIN
      #warning "Note: Auto-assigned W sensorless endstop to E7_DIAG_PIN. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
    #endif
  #endif

  #if ENABLED(CHAMBER_FAN) && !defined(CHAMBER_FAN_INDEX)
    #warning "Note: Auto-assigned CHAMBER_FAN_INDEX to the first free FAN pin. (Define NO_AUTO_ASSIGN_WARNING to suppress this warning.)"
  #endif

#endif // !NO_AUTO_ASSIGN_WARNING

#if IS_LEGACY_TFT
  #warning "Don't forget to update your TFT settings in Configuration.h."
#endif

#if ENABLED(EMIT_CREALITY_422_WARNING) && DISABLED(NO_CREALITY_422_DRIVER_WARNING)
  // Driver labels: A=TMC2208, B=TMC2209, C=HR4988, E=A4988, H=TMC2225, H8=HR4988
  #warning "Creality 4.2.2 boards come with a variety of stepper drivers. Check the board label (typically on SD Card module) and set the correct *_DRIVER_TYPE! (A/H: TMC2208_STANDALONE  B: TMC2209_STANDALONE  C/E/H8: A4988). (Define NO_CREALITY_422_DRIVER_WARNING to suppress this warning.)"
#endif

#if ENABLED(PRINTCOUNTER_SYNC)
  #warning "To prevent step loss, motion will pause for PRINTCOUNTER auto-save."
#endif

#if HOMING_Z_WITH_PROBE && IS_CARTESIAN && NONE(Z_SAFE_HOMING, NO_Z_SAFE_HOMING_WARNING)
  #error "Z_SAFE_HOMING is recommended when homing with a probe. (Enable Z_SAFE_HOMING or define NO_Z_SAFE_HOMING_WARNING to suppress this warning.)"
#endif

#if HAS_TRINAMIC_CONFIG && NONE(EDGE_STEPPING, NO_EDGE_STEPPING_WARNING)
  #error "EDGE_STEPPING is strongly recommended with Trinamic stepper drivers. (Enable EDGE_STEPPING or define NO_EDGE_STEPPING_WARNING to suppress this warning.)"
#endif

#if ENABLED(BIQU_MICROPROBE_V2) && NONE(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN, NO_MICROPROBE_WARNING)
  #warning "BIQU MicroProbe V2 detect signal requires a strong pull-up. Some processors have weak internal pull-up capabilities, so we recommended connecting MicroProbe SIGNAL / GND to Z-MIN / Z-STOP instead of the dedicated PROBE port. (Define NO_MICROPROBE_WARNING to suppress this warning.)"
#endif

//
// Warn users of potential endstop/DIAG pin conflicts to prevent homing issues when not using sensorless homing
//
#if !USE_SENSORLESS && HAS_DIAG_PINS
  #if ENABLED(USES_DIAG_JUMPERS) && DISABLED(DIAG_JUMPERS_REMOVED)
    #warning "Motherboard DIAG jumpers must be removed when SENSORLESS_HOMING is disabled. (Define DIAG_JUMPERS_REMOVED to suppress this warning.)"
  #endif
  #if ENABLED(USES_DIAG_PINS) && DISABLED(DIAG_PINS_REMOVED)
    #warning "Driver DIAG pins must be physically removed unless SENSORLESS_HOMING is enabled. (See https://bit.ly/2ZPRlt0) (Define DIAG_PINS_REMOVED to suppress this warning.)"
  #endif
#endif

#if ENABLED(QUICK_HOME)
  #if X_SPI_SENSORLESS || Y_SPI_SENSORLESS
    #warning "If SPI_ENDSTOPS are unreliable with QUICK_HOME try adjusting SENSORLESS_BACKOFF_MM, Travel Acceleration (M204 T), Homing Feedrate (M210 XY), or disable QUICK_HOME."
  #elif X_SENSORLESS || Y_SENSORLESS
    #warning "If SENSORLESS_HOMING is unreliable with QUICK_HOME try adjusting SENSORLESS_BACKOFF_MM, Travel Acceleration (M204 T), Homing Feedrate (M210 XY), or disable QUICK_HOME."
  #endif
#endif

#if HIGHER_CURRENT_HOME_WARNING
  #warning "High homing currents can lead to damage if a sensor fails or is set up incorrectly."
#endif

#if USE_SENSORLESS && DISABLED(NO_HOMING_CURRENT_WARNING)
  #if ENABLED(X_SENSORLESS) && defined(X_CURRENT_HOME) && !HAS_CURRENT_HOME(X)
    #warning "It's recommended to set X_CURRENT_HOME lower than X_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #elif ENABLED(X2_SENSORLESS) && defined(X2_CURRENT_HOME) && !HAS_CURRENT_HOME(X2)
    #warning "It's recommended to set X2_CURRENT_HOME lower than X2_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #endif
  #if ENABLED(Y_SENSORLESS) && defined(Y_CURRENT_HOME) && !HAS_CURRENT_HOME(Y)
    #warning "It's recommended to set Y_CURRENT_HOME lower than Y_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #elif ENABLED(Y2_SENSORLESS) && defined(Y2_CURRENT_HOME) && !HAS_CURRENT_HOME(Y2)
    #warning "It's recommended to set Y2_CURRENT_HOME lower than Y2_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #endif
  #if ENABLED(Z_SENSORLESS) && defined(Z_CURRENT_HOME) && !HAS_CURRENT_HOME(Z)
    #warning "It's recommended to set Z_CURRENT_HOME lower than Z_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #elif ENABLED(Z2_SENSORLESS) && defined(Z2_CURRENT_HOME) && !HAS_CURRENT_HOME(Z2)
    #warning "It's recommended to set Z2_CURRENT_HOME lower than Z2_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #elif ENABLED(Z3_SENSORLESS) && defined(Z3_CURRENT_HOME) && !HAS_CURRENT_HOME(Z3)
    #warning "It's recommended to set Z3_CURRENT_HOME lower than Z3_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #elif ENABLED(Z4_SENSORLESS) && defined(Z4_CURRENT_HOME) && !HAS_CURRENT_HOME(Z4)
    #warning "It's recommended to set Z4_CURRENT_HOME lower than Z4_CURRENT with SENSORLESS_HOMING. (Define NO_HOMING_CURRENT_WARNING to suppress this warning.)"
  #endif
#endif

#if CANNOT_EMBED_CONFIGURATION
  #warning "Disabled CONFIGURATION_EMBEDDING because the target usually has less flash storage. Define FORCE_CONFIG_EMBED to override."
#endif

#if HAS_LCD_CONTRAST && LCD_CONTRAST_MIN >= LCD_CONTRAST_MAX && DISABLED(NO_LCD_CONTRAST_WARNING)
  #warning "Contrast cannot be changed when LCD_CONTRAST_MIN >= LCD_CONTRAST_MAX. (Define NO_LCD_CONTRAST_WARNING to suppress this warning.)"
#endif

#if PROGRESS_MSG_EXPIRE > 0 && HAS_STATUS_MESSAGE_TIMEOUT
  #warning "It is recommended not to combine PROGRESS_MSG_EXPIRE with STATUS_MESSAGE_TIMEOUT_SEC."
#endif

/**
 * FYSETC/MKS/BTT/BEEZ Mini Panel backlighting
 */
#if ANY(FYSETC_242_OLED_12864, FYSETC_MINI_12864_2_1) && !ALL(NEOPIXEL_LED, LED_CONTROL_MENU, LED_USER_PRESET_STARTUP, LED_COLOR_PRESETS)
  #warning "Your FYSETC/MKS/BTT/BEEZ Mini Panel works best with NEOPIXEL_LED, LED_CONTROL_MENU, LED_USER_PRESET_STARTUP, and LED_COLOR_PRESETS."
#endif

#if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0) && DISABLED(RGB_LED)
  #warning "Your FYSETC Mini Panel works best with RGB_LED."
#elif ANY(FYSETC_MINI_12864_2_0, FYSETC_MINI_12864_2_1) && DISABLED(LED_USER_PRESET_STARTUP)
  #warning "Your FYSETC/MKS/BTT/BEEZ Mini Panel works best with LED_USER_PRESET_STARTUP."
#endif

#if ANY(FYSETC_242_OLED_12864, FYSETC_MINI_12864) && ALL(PSU_CONTROL, HAS_COLOR_LEDS) && !LED_POWEROFF_TIMEOUT
  #warning "Your FYSETC display with PSU_CONTROL works best with LED_POWEROFF_TIMEOUT."
#endif

/**
 * Maple environment
 */
#if defined(__STM32F1__) && DISABLED(NO_MAPLE_WARNING)
  #warning "Maple build environments are deprecated. Please use a non-Maple build environment. Report issues to the Marlin Firmware project."
#endif

/**
 * Průša MK3/S/+ fan pin reassignment
 */
#if MB(BTT_BTT002_V1_0, EINSY_RAMBO) && DISABLED(NO_MK3_FAN_PINS_WARNING)
  #warning "Define MK3_FAN_PINS to swap hotend and part cooling fan pins. (Define NO_MK3_FAN_PINS_WARNING to suppress this warning.)"
#endif

/**
 * GD32 is not exactly like STM32
 */
#if MB(SOVOL_V131, TRIGORILLA_V006)
  #warning "GD32 based controllers may not be fully compatible with STM32 platforms. Please report any issues."
#endif

/**
 * BD Sensor should always include BABYSTEPPING
 */
#if ENABLED(BD_SENSOR) && DISABLED(BABYSTEPPING)
  #warning "BABYSTEPPING is recommended with BD_SENSOR."
#endif

/**
 * EP Babystepping works best with EMERGENCY_PARSER
 */
#if ENABLED(EP_BABYSTEPPING) && DISABLED(EMERGENCY_PARSER)
  #warning "EMERGENCY_PARSER is recommended for EP_BABYSTEPPING."
#endif

/**
 * POLAR warnings
 */
#if ALL(POLAR, S_CURVE_ACCELERATION)
  #warning "POLAR kinematics may not work well with S_CURVE_ACCELERATION."
#endif

/**
 * Input Shaping
 */
#if HAS_ZV_SHAPING
  #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
    #warning "Input Shaping for CORE / MARKFORGED kinematic axes is still experimental."
  #endif
  #if ENABLED(I2S_STEPPER_STREAM)
    #warning "Input Shaping has not been tested with I2S_STEPPER_STREAM."
  #endif
#endif

/**
 * Automatic Multistepping Limit
 */
#if MULTISTEPPING_LIMIT_WARNING
  #warning "MULTISTEPPING_LIMIT has been automatically set to 128. Use a lower value if the machine is slow to respond."
#endif

/**
 * SD Card extras
 */
#if SDSORT_CACHE_VFATS_WARNING
  #warning "SDSORT_CACHE_VFATS has been reduced to VFAT_ENTRIES_LIMIT."
#endif
#if SDSORT_CACHE_LPC1768_WARNING
  #warning "SDCARD_SORT_ALPHA sub-options overridden for LPC1768 with DOGM LCD SCK overlap."
#endif

/**
 * Ender-5 S1 bootloader
 */
#ifdef STM32F4_UPDATE_FOLDER
  #warning "Place the firmware bin file in a folder named 'STM32F4_UPDATE' on the SD card. Install with 'M936 V2'."
#endif

/**
 * Voxelab N32 bootloader
 */
#ifdef SDCARD_FLASH_LIMIT_256K
  #warning "This board has 512K but the bootloader can only flash firmware.bin <= 256K. ICSP required for full 512K capacity."
#endif

/**
 * ProUI Extras
 */
#if ENABLED(DWIN_LCD_PROUI)
  #if BOOTSCREEN_TIMEOUT > 2000
    #warning "For ProUI the original BOOTSCREEN_TIMEOUT of 1100 is recommended."
  #endif
  #if HAS_PID_HEATING && NONE(PID_AUTOTUNE_MENU, PID_EDIT_MENU)
    #warning "For ProUI PID_AUTOTUNE_MENU and PID_EDIT_MENU is recommended for PID tuning."
  #elif ENABLED(MPCTEMP) && NONE(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
    #warning "For ProUI MPC_EDIT_MENU and MPC_AUTOTUNE_MENU is recommended for MPC tuning."
  #endif
#endif

/**
 * AD595 Minimum Voltage
 */
#if HAL_ADC_VREF_MV < 5000 && ANY_THERMISTOR_IS(-1) && DISABLED(ALLOW_AD595_3V3_VREF)
  #warning "The (-1) AD595 Thermocouple Amplifier requires 5V input supply! Use AD8495 for 3.3V ADC."
#endif

/**
 * No PWM on the Piezo Beeper?
 */
#if PIN_EXISTS(BEEPER) && ALL(SPEAKER, NO_SPEAKER)
  #warning "The BEEPER cannot produce tones so you can disable SPEAKER."
#endif

/**
 * Fixed-Time Motion
 */
#if ALL(FT_MOTION, I2S_STEPPER_STREAM)
  #warning "FT_MOTION has not been tested with I2S_STEPPER_STREAM."
#endif

/**
 * User doesn't have or disabled G92?
 */
#if DISABLED(EDITABLE_STEPS_PER_UNIT)
  #warning "EDITABLE_STEPS_PER_UNIT is required to enable G92 runtime configuration of steps-per-unit."
#endif

/**
 * Peltier with PIDTEMPBED
 */
#if ALL(PELTIER_BED, PIDTEMPBED)
  #warning "PELTIER_BED with PIDTEMPBED requires extra circuitry. Use with caution."
#endif
