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
 * stepper.cpp - A singleton object to execute motion plans using stepper motors
 * Marlin Firmware
 *
 * Derived from Grbl
 * Copyright (c) 2009-2011 Simen Svale Skogsrud
 *
 * Grbl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Grbl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Grbl.  If not, see <https://www.gnu.org/licenses/>.
 */

/**
 * Timer calculations informed by the 'RepRap cartesian firmware' by Zack Smith
 * and Philipp Tiefenbacher.
 */

/**
 *         __________________________
 *        /|                        |\     _________________         ^
 *       / |                        | \   /|               |\        |
 *      /  |                        |  \ / |               | \       s
 *     /   |                        |   |  |               |  \      p
 *    /    |                        |   |  |               |   \     e
 *   +-----+------------------------+---+--+---------------+----+    e
 *   |               BLOCK 1            |      BLOCK 2          |    d
 *
 *                           time ----->
 *
 *  The speed over time graph forms a TRAPEZOID. The slope of acceleration is calculated by
 *    v = u + t
 *  where 't' is the accumulated timer values of the steps so far.
 *
 *  The Stepper ISR dynamically executes acceleration, deceleration, and cruising according to the block parameters.
 *    - Start at block->initial_rate.
 *    - Accelerate while step_events_completed < block->accelerate_before.
 *    - Cruise while step_events_completed < block->decelerate_start.
 *    - Decelerate after that, until all steps are completed.
 *    - Reset the trapezoid generator.
 */

/**
 * Marlin uses the Bresenham algorithm. For a detailed explanation of theory and
 * method see https://www.cs.helsinki.fi/group/goa/mallinnus/lines/bresenh.html
 */

/**
 * Jerk controlled movements planner added Apr 2018 by Eduardo José Tagle.
 * Equations based on Synthethos TinyG2 sources, but the fixed-point
 * implementation is new, as we are running the ISR with a variable period.
 * Also implemented the Bézier velocity curve evaluation in ARM assembler,
 * to avoid impacting ISR speed.
 */

#include "stepper.h"

Stepper stepper; // Singleton

#define BABYSTEPPING_EXTRA_DIR_WAIT

#include "stepper/cycles.h"
#ifdef __AVR__
  #include "stepper/speed_lookuptable.h"
#endif

#include "endstops.h"
#include "planner.h"
#include "motion.h"

#if ENABLED(FT_MOTION)
  #include "ft_motion.h"
#endif

#include "../lcd/marlinui.h"
#include "../gcode/queue.h"
#include "../sd/cardreader.h"
#include "../MarlinCore.h"
#include "../HAL/shared/Delay.h"

#if ENABLED(BD_SENSOR)
  #include "../feature/bedlevel/bdl/bdl.h"
#endif

#if ENABLED(BABYSTEPPING)
  #include "../feature/babystep.h"
#endif

#if MB(ALLIGATOR)
  #include "../feature/dac/dac_dac084s085.h"
#endif

#if HAS_MOTOR_CURRENT_SPI
  #include <SPI.h>
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../feature/mixing.h"
#endif

#if HAS_FILAMENT_RUNOUT_DISTANCE
  #include "../feature/runout.h"
#endif

#if ENABLED(AUTO_POWER_CONTROL)
  #include "../feature/power.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../feature/powerloss.h"
#endif

#if HAS_CUTTER
  #include "../feature/spindle_laser.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#if ENABLED(I2S_STEPPER_STREAM)
  #include "../HAL/ESP32/i2s.h"
#endif

// public:

#if ANY(HAS_EXTRA_ENDSTOPS, Z_STEPPER_AUTO_ALIGN)
  bool Stepper::separate_multi_axis = false;
#endif

#if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
  bool Stepper::initialized; // = false
  uint32_t Stepper::motor_current_setting[MOTOR_CURRENT_COUNT]; // Initialized by settings.load
  #if HAS_MOTOR_CURRENT_SPI
    constexpr uint32_t Stepper::digipot_count[];
  #endif
#endif

stepper_flags_t Stepper::axis_enabled; // {0}

// private:

block_t* Stepper::current_block; // (= nullptr) A pointer to the block currently being traced

AxisBits Stepper::last_direction_bits, // = 0
         Stepper::axis_did_move; // = 0

bool Stepper::abort_current_block;

#if DISABLED(MIXING_EXTRUDER) && HAS_MULTI_EXTRUDER
  uint8_t Stepper::last_moved_extruder = 0xFF;
#endif

#if ENABLED(X_DUAL_ENDSTOPS)
  bool Stepper::locked_X_motor = false, Stepper::locked_X2_motor = false;
#endif
#if ENABLED(Y_DUAL_ENDSTOPS)
  bool Stepper::locked_Y_motor = false, Stepper::locked_Y2_motor = false;
#endif

#if ANY(Z_MULTI_ENDSTOPS, Z_STEPPER_AUTO_ALIGN)
  bool Stepper::locked_Z_motor = false, Stepper::locked_Z2_motor = false
    #if NUM_Z_STEPPERS >= 3
      , Stepper::locked_Z3_motor = false
      #if NUM_Z_STEPPERS >= 4
        , Stepper::locked_Z4_motor = false
      #endif
    #endif
  ;
#endif

// In timer_ticks
uint32_t Stepper::acceleration_time, Stepper::deceleration_time;

#if MULTISTEPPING_LIMIT > 1
  uint8_t Stepper::steps_per_isr = 1; // Count of steps to perform per Stepper ISR call
#endif

#if DISABLED(OLD_ADAPTIVE_MULTISTEPPING)
  hal_timer_t Stepper::time_spent_in_isr = 0, Stepper::time_spent_out_isr = 0;
#endif

#if ENABLED(ADAPTIVE_STEP_SMOOTHING)
  #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
    bool Stepper::adaptive_step_smoothing_enabled; // Initialized by settings.load
  #else
    constexpr bool Stepper::adaptive_step_smoothing_enabled; // = true
  #endif
  // Oversampling factor (log2(multiplier)) to increase temporal resolution of axis
  uint8_t Stepper::oversampling_factor;
#else
  constexpr uint8_t Stepper::oversampling_factor; // = 0
#endif

#if ENABLED(FREEZE_FEATURE)
  bool Stepper::frozen; // = false
#endif

xyze_long_t Stepper::delta_error{0};

xyze_long_t Stepper::advance_dividend{0};
uint32_t Stepper::advance_divisor = 0,
         Stepper::step_events_completed = 0, // The number of step events executed in the current block
         Stepper::accelerate_before,         // The count at which to start cruising
         Stepper::decelerate_start,          // The count at which to start decelerating
         Stepper::step_event_count;          // The total event count for the current block

#if ANY(HAS_MULTI_EXTRUDER, MIXING_EXTRUDER)
  uint8_t Stepper::stepper_extruder;
#else
  constexpr uint8_t Stepper::stepper_extruder;
#endif

#if ENABLED(S_CURVE_ACCELERATION)
  int32_t __attribute__((used)) Stepper::bezier_A __asm__("bezier_A");    // A coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) Stepper::bezier_B __asm__("bezier_B");    // B coefficient in Bézier speed curve with alias for assembler
  int32_t __attribute__((used)) Stepper::bezier_C __asm__("bezier_C");    // C coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) Stepper::bezier_F __asm__("bezier_F");   // F coefficient in Bézier speed curve with alias for assembler
  uint32_t __attribute__((used)) Stepper::bezier_AV __asm__("bezier_AV"); // AV coefficient in Bézier speed curve with alias for assembler
  #ifdef __AVR__
    bool __attribute__((used)) Stepper::A_negative __asm__("A_negative"); // If A coefficient was negative
  #endif
  bool Stepper::bezier_2nd_half;    // =false If Bézier curve has been initialized or not
#endif

#if ENABLED(LIN_ADVANCE)
  hal_timer_t Stepper::nextAdvanceISR = LA_ADV_NEVER,
              Stepper::la_interval = LA_ADV_NEVER;
  int32_t     Stepper::la_delta_error = 0,
              Stepper::la_dividend = 0,
              Stepper::la_advance_steps = 0;
  bool        Stepper::la_active = false;
#endif

#if ENABLED(NONLINEAR_EXTRUSION)
  ne_coeff_t Stepper::ne;
  ne_fix_t Stepper::ne_fix;
  int32_t Stepper::ne_edividend;
  uint32_t Stepper::ne_scale;
#endif

#if HAS_ZV_SHAPING
  shaping_time_t      ShapingQueue::now = 0;
  #if ANY(MCU_LPC1768, MCU_LPC1769) && DISABLED(NO_LPC_ETHERNET_BUFFER)
    // Use the 16K LPC Ethernet buffer: https://github.com/MarlinFirmware/Marlin/issues/25432#issuecomment-1450420638
    #define _ATTR_BUFFER __attribute__((section("AHBSRAM1"),aligned))
  #else
    #define _ATTR_BUFFER
  #endif
  shaping_time_t      ShapingQueue::times[shaping_echoes] _ATTR_BUFFER;
  shaping_echo_axis_t ShapingQueue::echo_axes[shaping_echoes];
  uint16_t            ShapingQueue::tail = 0;

  #define SHAPING_VAR_DEFS(AXIS)                                           \
    shaping_time_t  ShapingQueue::delay_##AXIS;                            \
    shaping_time_t  ShapingQueue::_peek_##AXIS = shaping_time_t(-1);       \
    uint16_t        ShapingQueue::head_##AXIS = 0;                         \
    uint16_t        ShapingQueue::_free_count_##AXIS = shaping_echoes - 1; \
    ShapeParams     Stepper::shaping_##AXIS;

  TERN_(INPUT_SHAPING_X, SHAPING_VAR_DEFS(x))
  TERN_(INPUT_SHAPING_Y, SHAPING_VAR_DEFS(y))
  TERN_(INPUT_SHAPING_Z, SHAPING_VAR_DEFS(z))
#endif

#if ENABLED(BABYSTEPPING)
  hal_timer_t Stepper::nextBabystepISR = BABYSTEP_NEVER;
#endif

#if ENABLED(DIRECT_STEPPING)
  page_step_state_t Stepper::page_step_state;
#endif

hal_timer_t Stepper::ticks_nominal = 0;
#if DISABLED(S_CURVE_ACCELERATION)
  uint32_t Stepper::acc_step_rate; // needed for deceleration start point
#endif

xyz_long_t Stepper::endstops_trigsteps;
xyze_long_t Stepper::count_position{0};
xyze_int8_t Stepper::count_direction{0};

#define MINDIR(A) (count_direction[_AXIS(A)] < 0)
#define MAXDIR(A) (count_direction[_AXIS(A)] > 0)

#define STEPTEST(A,M,I) TERN0(USE_##A##I##_##M, !(TEST(endstops.state(), A##I##_##M) && M## DIR(A)) && !locked_ ##A##I##_motor)
#define _STEP_WRITE(A,I,V) A##I##_STEP_WRITE(V)

#define DUAL_ENDSTOP_APPLY_STEP(A,V)             \
  if (separate_multi_axis) {                     \
    if (ENABLED(A##_HOME_TO_MIN)) {              \
      if (STEPTEST(A,MIN, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MIN,2)) _STEP_WRITE(A,2,V); \
    }                                            \
    else if (ENABLED(A##_HOME_TO_MAX)) {         \
      if (STEPTEST(A,MAX, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MAX,2)) _STEP_WRITE(A,2,V); \
    }                                            \
  }                                              \
  else {                                         \
    _STEP_WRITE(A, ,V);                          \
    _STEP_WRITE(A,2,V);                          \
  }

#define DUAL_SEPARATE_APPLY_STEP(A,V)             \
  if (separate_multi_axis) {                      \
    if (!locked_##A## _motor) _STEP_WRITE(A, ,V); \
    if (!locked_##A##2_motor) _STEP_WRITE(A,2,V); \
  }                                               \
  else {                                          \
    _STEP_WRITE(A, ,V);                           \
    _STEP_WRITE(A,2,V);                           \
  }

#define TRIPLE_ENDSTOP_APPLY_STEP(A,V)           \
  if (separate_multi_axis) {                     \
    if (ENABLED(A##_HOME_TO_MIN)) {              \
      if (STEPTEST(A,MIN, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MIN,2)) _STEP_WRITE(A,2,V); \
      if (STEPTEST(A,MIN,3)) _STEP_WRITE(A,3,V); \
    }                                            \
    else if (ENABLED(A##_HOME_TO_MAX)) {         \
      if (STEPTEST(A,MAX, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MAX,2)) _STEP_WRITE(A,2,V); \
      if (STEPTEST(A,MAX,3)) _STEP_WRITE(A,3,V); \
    }                                            \
  }                                              \
  else {                                         \
    _STEP_WRITE(A, ,V);                          \
    _STEP_WRITE(A,2,V);                          \
    _STEP_WRITE(A,3,V);                          \
  }

#define TRIPLE_SEPARATE_APPLY_STEP(A,V)           \
  if (separate_multi_axis) {                      \
    if (!locked_##A## _motor) _STEP_WRITE(A, ,V); \
    if (!locked_##A##2_motor) _STEP_WRITE(A,2,V); \
    if (!locked_##A##3_motor) _STEP_WRITE(A,3,V); \
  }                                               \
  else {                                          \
    _STEP_WRITE(A, ,V);                           \
    _STEP_WRITE(A,2,V);                           \
    _STEP_WRITE(A,3,V);                           \
  }

#define QUAD_ENDSTOP_APPLY_STEP(A,V)             \
  if (separate_multi_axis) {                     \
    if (ENABLED(A##_HOME_TO_MIN)) {              \
      if (STEPTEST(A,MIN, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MIN,2)) _STEP_WRITE(A,2,V); \
      if (STEPTEST(A,MIN,3)) _STEP_WRITE(A,3,V); \
      if (STEPTEST(A,MIN,4)) _STEP_WRITE(A,4,V); \
    }                                            \
    else if (ENABLED(A##_HOME_TO_MAX)) {         \
      if (STEPTEST(A,MAX, )) _STEP_WRITE(A, ,V); \
      if (STEPTEST(A,MAX,2)) _STEP_WRITE(A,2,V); \
      if (STEPTEST(A,MAX,3)) _STEP_WRITE(A,3,V); \
      if (STEPTEST(A,MAX,4)) _STEP_WRITE(A,4,V); \
    }                                            \
  }                                              \
  else {                                         \
    _STEP_WRITE(A, ,V);                          \
    _STEP_WRITE(A,2,V);                          \
    _STEP_WRITE(A,3,V);                          \
    _STEP_WRITE(A,4,V);                          \
  }

#define QUAD_SEPARATE_APPLY_STEP(A,V)             \
  if (separate_multi_axis) {                      \
    if (!locked_##A## _motor) _STEP_WRITE(A, ,V); \
    if (!locked_##A##2_motor) _STEP_WRITE(A,2,V); \
    if (!locked_##A##3_motor) _STEP_WRITE(A,3,V); \
    if (!locked_##A##4_motor) _STEP_WRITE(A,4,V); \
  }                                               \
  else {                                          \
    _STEP_WRITE(A, ,V);                           \
    _STEP_WRITE(A,2,V);                           \
    _STEP_WRITE(A,3,V);                           \
    _STEP_WRITE(A,4,V);                           \
  }

#if HAS_SYNCED_X_STEPPERS
  #define X_APPLY_DIR(FWD,Q) do{ X_DIR_WRITE(FWD); X2_DIR_WRITE(INVERT_DIR(X2_VS_X, FWD)); }while(0)
  #if ENABLED(X_DUAL_ENDSTOPS)
    #define X_APPLY_STEP(STATE,Q) DUAL_ENDSTOP_APPLY_STEP(X,STATE)
  #else
    #define X_APPLY_STEP(STATE,Q) do{ X_STEP_WRITE(STATE); X2_STEP_WRITE(STATE); }while(0)
  #endif
#elif ENABLED(DUAL_X_CARRIAGE)
  #define X_APPLY_DIR(FWD,ALWAYS) do{ \
    if (extruder_duplication_enabled || ALWAYS) { X_DIR_WRITE(FWD); X2_DIR_WRITE((FWD) ^ idex_mirrored_mode); } \
    else if (last_moved_extruder) X2_DIR_WRITE(FWD); else X_DIR_WRITE(FWD); \
  }while(0)
  #define X_APPLY_STEP(STATE,ALWAYS) do{ \
    if (extruder_duplication_enabled || ALWAYS) { X_STEP_WRITE(STATE); X2_STEP_WRITE(STATE); } \
    else if (last_moved_extruder) X2_STEP_WRITE(STATE); else X_STEP_WRITE(STATE); \
  }while(0)
#elif HAS_X_AXIS
  #define X_APPLY_DIR(FWD,Q) X_DIR_WRITE(FWD)
  #define X_APPLY_STEP(STATE,Q) X_STEP_WRITE(STATE)
#endif

#if HAS_SYNCED_Y_STEPPERS
  #define Y_APPLY_DIR(FWD,Q) do{ Y_DIR_WRITE(FWD); Y2_DIR_WRITE(INVERT_DIR(Y2_VS_Y, FWD)); }while(0)
  #if ENABLED(Y_DUAL_ENDSTOPS)
    #define Y_APPLY_STEP(STATE,Q) DUAL_ENDSTOP_APPLY_STEP(Y,STATE)
  #else
    #define Y_APPLY_STEP(STATE,Q) do{ Y_STEP_WRITE(STATE); Y2_STEP_WRITE(STATE); }while(0)
  #endif
#elif HAS_Y_AXIS
  #define Y_APPLY_DIR(FWD,Q) Y_DIR_WRITE(FWD)
  #define Y_APPLY_STEP(STATE,Q) Y_STEP_WRITE(STATE)
#endif

#if NUM_Z_STEPPERS == 4
  #define Z_APPLY_DIR(FWD,Q) do{ \
    Z_DIR_WRITE(FWD); Z2_DIR_WRITE(INVERT_DIR(Z2_VS_Z, FWD)); \
    Z3_DIR_WRITE(INVERT_DIR(Z3_VS_Z, FWD)); Z4_DIR_WRITE(INVERT_DIR(Z4_VS_Z, FWD)); \
  }while(0)
  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z_APPLY_STEP(STATE,Q) QUAD_ENDSTOP_APPLY_STEP(Z,STATE)
  #elif ENABLED(Z_STEPPER_AUTO_ALIGN)
    #define Z_APPLY_STEP(STATE,Q) QUAD_SEPARATE_APPLY_STEP(Z,STATE)
  #else
    #define Z_APPLY_STEP(STATE,Q) do{ Z_STEP_WRITE(STATE); Z2_STEP_WRITE(STATE); Z3_STEP_WRITE(STATE); Z4_STEP_WRITE(STATE); }while(0)
  #endif
#elif NUM_Z_STEPPERS == 3
  #define Z_APPLY_DIR(FWD,Q) do{ \
    Z_DIR_WRITE(FWD); Z2_DIR_WRITE(INVERT_DIR(Z2_VS_Z, FWD)); Z3_DIR_WRITE(INVERT_DIR(Z3_VS_Z, FWD)); \
  }while(0)
  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z_APPLY_STEP(STATE,Q) TRIPLE_ENDSTOP_APPLY_STEP(Z,STATE)
  #elif ENABLED(Z_STEPPER_AUTO_ALIGN)
    #define Z_APPLY_STEP(STATE,Q) TRIPLE_SEPARATE_APPLY_STEP(Z,STATE)
  #else
    #define Z_APPLY_STEP(STATE,Q) do{ Z_STEP_WRITE(STATE); Z2_STEP_WRITE(STATE); Z3_STEP_WRITE(STATE); }while(0)
  #endif
#elif NUM_Z_STEPPERS == 2
  #define Z_APPLY_DIR(FWD,Q) do{ Z_DIR_WRITE(FWD); Z2_DIR_WRITE(INVERT_DIR(Z2_VS_Z, FWD)); }while(0)
  #if ENABLED(Z_MULTI_ENDSTOPS)
    #define Z_APPLY_STEP(STATE,Q) DUAL_ENDSTOP_APPLY_STEP(Z,STATE)
  #elif ENABLED(Z_STEPPER_AUTO_ALIGN)
    #define Z_APPLY_STEP(STATE,Q) DUAL_SEPARATE_APPLY_STEP(Z,STATE)
  #else
    #define Z_APPLY_STEP(STATE,Q) do{ Z_STEP_WRITE(STATE); Z2_STEP_WRITE(STATE); }while(0)
  #endif
#elif HAS_Z_AXIS
  #define Z_APPLY_DIR(FWD,Q) Z_DIR_WRITE(FWD)
  #define Z_APPLY_STEP(STATE,Q) Z_STEP_WRITE(STATE)
#endif

#if HAS_I_AXIS
  #define I_APPLY_DIR(FWD,Q) I_DIR_WRITE(FWD)
  #define I_APPLY_STEP(STATE,Q) I_STEP_WRITE(STATE)
#endif
#if HAS_J_AXIS
  #define J_APPLY_DIR(FWD,Q) J_DIR_WRITE(FWD)
  #define J_APPLY_STEP(STATE,Q) J_STEP_WRITE(STATE)
#endif
#if HAS_K_AXIS
  #define K_APPLY_DIR(FWD,Q) K_DIR_WRITE(FWD)
  #define K_APPLY_STEP(STATE,Q) K_STEP_WRITE(STATE)
#endif
#if HAS_U_AXIS
  #define U_APPLY_DIR(FWD,Q) U_DIR_WRITE(FWD)
  #define U_APPLY_STEP(STATE,Q) U_STEP_WRITE(STATE)
#endif
#if HAS_V_AXIS
  #define V_APPLY_DIR(FWD,Q) V_DIR_WRITE(FWD)
  #define V_APPLY_STEP(STATE,Q) V_STEP_WRITE(STATE)
#endif
#if HAS_W_AXIS
  #define W_APPLY_DIR(FWD,Q) W_DIR_WRITE(FWD)
  #define W_APPLY_STEP(STATE,Q) W_STEP_WRITE(STATE)
#endif

//#define E0_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(0) : REV_E_DIR(0); }while(0)
//#define E1_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(1) : REV_E_DIR(1); }while(0)
//#define E2_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(2) : REV_E_DIR(2); }while(0)
//#define E3_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(3) : REV_E_DIR(3); }while(0)
//#define E4_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(4) : REV_E_DIR(4); }while(0)
//#define E5_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(5) : REV_E_DIR(5); }while(0)
//#define E6_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(6) : REV_E_DIR(6); }while(0)
//#define E7_APPLY_DIR(FWD) do{ (FWD) ? FWD_E_DIR(7) : REV_E_DIR(7); }while(0)

#if ENABLED(MIXING_EXTRUDER)
  #define E_APPLY_DIR(FWD,Q) do{ if (FWD) { MIXER_STEPPER_LOOP(j) FWD_E_DIR(j); } else { MIXER_STEPPER_LOOP(j) REV_E_DIR(j); } }while(0)
#else
  #define E_APPLY_DIR(FWD,Q) do{ if (FWD) { FWD_E_DIR(stepper_extruder); } else { REV_E_DIR(stepper_extruder); } }while(0)
  #define E_APPLY_STEP(STATE,Q) E_STEP_WRITE(stepper_extruder, STATE)
#endif

constexpr uint32_t cycles_to_ns(const uint32_t CYC) { return 1000UL * (CYC) / ((F_CPU) / 1000000); }
constexpr uint32_t ns_per_pulse_timer_tick = 1000000000UL / (STEPPER_TIMER_RATE);

// Round up when converting from ns to timer ticks
constexpr hal_timer_t ns_to_pulse_timer_ticks(const uint32_t ns) { return (ns + ns_per_pulse_timer_tick / 2) / ns_per_pulse_timer_tick; }

constexpr uint32_t timer_setup_ns = cycles_to_ns(timer_read_add_and_store_cycles);
constexpr hal_timer_t PULSE_HIGH_TICK_COUNT = ns_to_pulse_timer_ticks(_min_pulse_high_ns - _MIN(_min_pulse_high_ns, timer_setup_ns));
constexpr hal_timer_t PULSE_LOW_TICK_COUNT = ns_to_pulse_timer_ticks(_min_pulse_low_ns - _MIN(_min_pulse_low_ns, timer_setup_ns));

#define USING_TIMED_PULSE() hal_timer_t start_pulse_count = 0
#define START_TIMED_PULSE() (start_pulse_count = HAL_timer_get_count(MF_TIMER_PULSE))
#define AWAIT_TIMED_PULSE(DIR) while (PULSE_##DIR##_TICK_COUNT > HAL_timer_get_count(MF_TIMER_PULSE) - start_pulse_count) { /* nada */ }
#define AWAIT_HIGH_PULSE() AWAIT_TIMED_PULSE(HIGH)
#define AWAIT_LOW_PULSE()  AWAIT_TIMED_PULSE(LOW)

#if MINIMUM_STEPPER_PRE_DIR_DELAY > 0
  #define DIR_WAIT_BEFORE() DELAY_NS(MINIMUM_STEPPER_PRE_DIR_DELAY)
#else
  #define DIR_WAIT_BEFORE()
#endif

#if MINIMUM_STEPPER_POST_DIR_DELAY > 0
  #define DIR_WAIT_AFTER() DELAY_NS(MINIMUM_STEPPER_POST_DIR_DELAY)
#else
  #define DIR_WAIT_AFTER()
#endif

void Stepper::enable_axis(const AxisEnum axis) {
  #define _CASE_ENABLE(N) case N##_AXIS: ENABLE_AXIS_##N(); break;
  switch (axis) {
    MAIN_AXIS_MAP(_CASE_ENABLE)
    default: break;
  }
  mark_axis_enabled(axis);

  TERN_(EXTENSIBLE_UI, ExtUI::onAxisEnabled(ExtUI::axis_to_axis_t(axis)));
}

/**
 * Mark an axis as disabled and power off its stepper(s).
 * If one of the axis steppers is still in use by a non-disabled axis the axis will remain powered.
 * DISCUSSION: It's basically just stepper ENA pins that are shared across axes, not whole steppers.
 *             Used on MCUs with a shortage of pins. We already track the overlap of ENA pins, so now
 *             we just need stronger logic to track which ENA pins are being set more than once.
 *
 *             It would be better to use a bit mask (i.e., Flags<NUM_DISTINCT_AXIS_ENUMS>).
 *             While the method try_to_disable in gcode/control/M17_M18_M84.cpp does use the
 *             bit mask, it is still only at the axis level.
 * TODO: Power off steppers that don't share another axis. Currently axis-based steppers turn off as a unit.
 *       So we'd need to power off the off axis, then power on the on axis (for a microsecond).
 *       A global solution would keep a usage count when enabling or disabling a stepper, but this partially
 *       defeats the purpose of an on/off mask.
 */
bool Stepper::disable_axis(const AxisEnum axis) {
  mark_axis_disabled(axis);

  // This scheme prevents shared steppers being disabled. It should consider several axes at once
  // and keep a count of how many times each ENA pin has been set.

  // If all the axes that share the enabled bit are disabled
  // toggle the ENA state that they all share.
  const bool can_disable = can_axis_disable(axis);
  if (can_disable) {
    #define _CASE_DISABLE(N) case N##_AXIS: DISABLE_AXIS_##N(); break;
    switch (axis) {
      MAIN_AXIS_MAP(_CASE_DISABLE)
      default: break;
    }
    TERN_(EXTENSIBLE_UI, ExtUI::onAxisDisabled(ExtUI::axis_to_axis_t(axis)));
  }

  return can_disable;
}

#if HAS_EXTRUDERS

  void Stepper::enable_extruder(E_TERN_(const uint8_t eindex)) {
    IF_DISABLED(HAS_MULTI_EXTRUDER, constexpr uint8_t eindex = 0);
    #define _CASE_ENA_E(N) case N: ENABLE_AXIS_E##N(); mark_axis_enabled(E_AXIS E_OPTARG(eindex)); break;
    switch (eindex) {
      REPEAT(E_STEPPERS, _CASE_ENA_E)
    }
  }

  bool Stepper::disable_extruder(E_TERN_(const uint8_t eindex/*=0*/)) {
    IF_DISABLED(HAS_MULTI_EXTRUDER, constexpr uint8_t eindex = 0);
    mark_axis_disabled(E_AXIS E_OPTARG(eindex));
    const bool can_disable = can_axis_disable(E_AXIS E_OPTARG(eindex));
    if (can_disable) {
      #define _CASE_DIS_E(N) case N: DISABLE_AXIS_E##N(); break;
      switch (eindex) { REPEAT(E_STEPPERS, _CASE_DIS_E) }
    }
    return can_disable;
  }

  void Stepper::enable_e_steppers() {
    #define _ENA_E(N) ENABLE_EXTRUDER(N);
    REPEAT(EXTRUDERS, _ENA_E)
  }

  void Stepper::disable_e_steppers() {
    #define _DIS_E(N) DISABLE_EXTRUDER(N);
    REPEAT(EXTRUDERS, _DIS_E)
  }

#endif

void Stepper::enable_all_steppers() {
  TERN_(AUTO_POWER_CONTROL, powerManager.power_on());
  NUM_AXIS_CODE(
    enable_axis(X_AXIS), enable_axis(Y_AXIS), enable_axis(Z_AXIS),
    enable_axis(I_AXIS), enable_axis(J_AXIS), enable_axis(K_AXIS),
    enable_axis(U_AXIS), enable_axis(V_AXIS), enable_axis(W_AXIS)
  );
  enable_e_steppers();

  TERN_(EXTENSIBLE_UI, ExtUI::onSteppersEnabled());
}

void Stepper::disable_all_steppers() {
  NUM_AXIS_CODE(
    disable_axis(X_AXIS), disable_axis(Y_AXIS), disable_axis(Z_AXIS),
    disable_axis(I_AXIS), disable_axis(J_AXIS), disable_axis(K_AXIS),
    disable_axis(U_AXIS), disable_axis(V_AXIS), disable_axis(W_AXIS)
  );
  disable_e_steppers();

  TERN_(EXTENSIBLE_UI, ExtUI::onSteppersDisabled());
}

#if ENABLED(FTM_OPTIMIZE_DIR_STATES)
  // We'll compare the updated DIR bits to the last set state
  static AxisBits last_set_direction;
#endif

// Set a single axis direction based on the last set flags.
// A direction bit of "1" indicates forward or positive motion.
#define SET_STEP_DIR(A) do{                     \
    const bool fwd = motor_direction(_AXIS(A)); \
    A##_APPLY_DIR(fwd, false);                  \
    count_direction[_AXIS(A)] = fwd ? 1 : -1;   \
  }while(0)

/**
 * Set the stepper direction of each axis
 *
 *   COREXY: X_AXIS=A_AXIS and Y_AXIS=B_AXIS
 *   COREXZ: X_AXIS=A_AXIS and Z_AXIS=C_AXIS
 *   COREYZ: Y_AXIS=B_AXIS and Z_AXIS=C_AXIS
 */
void Stepper::apply_directions() {
  DIR_WAIT_BEFORE();

  LOGICAL_AXIS_CODE(
    SET_STEP_DIR(E),
    SET_STEP_DIR(X), SET_STEP_DIR(Y), SET_STEP_DIR(Z), // ABC
    SET_STEP_DIR(I), SET_STEP_DIR(J), SET_STEP_DIR(K),
    SET_STEP_DIR(U), SET_STEP_DIR(V), SET_STEP_DIR(W)
  );

  TERN_(FTM_OPTIMIZE_DIR_STATES, last_set_direction = last_direction_bits);

  DIR_WAIT_AFTER();
}

#if ENABLED(S_CURVE_ACCELERATION)
  /**
   *  This uses a quintic (fifth-degree) Bézier polynomial for the velocity curve, giving
   *  a "linear pop" velocity curve; with pop being the sixth derivative of position:
   *  velocity - 1st, acceleration - 2nd, jerk - 3rd, snap - 4th, crackle - 5th, pop - 6th
   *
   *  The Bézier curve takes the form:
   *
   *  V(t) = P_0 * B_0(t) + P_1 * B_1(t) + P_2 * B_2(t) + P_3 * B_3(t) + P_4 * B_4(t) + P_5 * B_5(t)
   *
   *  Where 0 <= t <= 1, and V(t) is the velocity. P_0 through P_5 are the control points, and B_0(t)
   *  through B_5(t) are the Bernstein basis as follows:
   *
   *        B_0(t) =   (1-t)^5        =   -t^5 +  5t^4 - 10t^3 + 10t^2 -  5t   +   1
   *        B_1(t) =  5(1-t)^4 * t    =   5t^5 - 20t^4 + 30t^3 - 20t^2 +  5t
   *        B_2(t) = 10(1-t)^3 * t^2  = -10t^5 + 30t^4 - 30t^3 + 10t^2
   *        B_3(t) = 10(1-t)^2 * t^3  =  10t^5 - 20t^4 + 10t^3
   *        B_4(t) =  5(1-t)   * t^4  =  -5t^5 +  5t^4
   *        B_5(t) =             t^5  =    t^5
   *                                      ^       ^       ^       ^       ^       ^
   *                                      |       |       |       |       |       |
   *                                      A       B       C       D       E       F
   *
   *  Unfortunately, we cannot use forward-differencing to calculate each position through
   *  the curve, as Marlin uses variable timer periods. So, we require a formula of the form:
   *
   *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + D*t^2 + E*t + F
   *
   *  Looking at the above B_0(t) through B_5(t) expanded forms, if we take the coefficients of t^5
   *  through t of the Bézier form of V(t), we can determine that:
   *
   *        A =    -P_0 +  5*P_1 - 10*P_2 + 10*P_3 -  5*P_4 +  P_5
   *        B =   5*P_0 - 20*P_1 + 30*P_2 - 20*P_3 +  5*P_4
   *        C = -10*P_0 + 30*P_1 - 30*P_2 + 10*P_3
   *        D =  10*P_0 - 20*P_1 + 10*P_2
   *        E = - 5*P_0 +  5*P_1
   *        F =     P_0
   *
   *  Now, since we will (currently) *always* want the initial acceleration and jerk values to be 0,
   *  We set P_i = P_0 = P_1 = P_2 (initial velocity), and P_t = P_3 = P_4 = P_5 (target velocity),
   *  which, after simplification, resolves to:
   *
   *        A = - 6*P_i +  6*P_t =  6*(P_t - P_i)
   *        B =  15*P_i - 15*P_t = 15*(P_i - P_t)
   *        C = -10*P_i + 10*P_t = 10*(P_t - P_i)
   *        D = 0
   *        E = 0
   *        F = P_i
   *
   *  As the t is evaluated in non uniform steps here, there is no other way rather than evaluating
   *  the Bézier curve at each point:
   *
   *        V_f(t) = A*t^5 + B*t^4 + C*t^3 + F          [0 <= t <= 1]
   *
   * Floating point arithmetic execution time cost is prohibitive, so we will transform the math to
   * use fixed point values to be able to evaluate it in realtime. Assuming a maximum of 250000 steps
   * per second (driver pulses should at least be 2µS hi/2µS lo), and allocating 2 bits to avoid
   * overflows on the evaluation of the Bézier curve, means we can use
   *
   *   t: unsigned Q0.32 (0 <= t < 1) |range 0 to 0xFFFFFFFF unsigned
   *   A:   signed Q24.7 ,            |range = +/- 250000 * 6 * 128 = +/- 192000000 = 0x0B71B000 | 28 bits + sign
   *   B:   signed Q24.7 ,            |range = +/- 250000 *15 * 128 = +/- 480000000 = 0x1C9C3800 | 29 bits + sign
   *   C:   signed Q24.7 ,            |range = +/- 250000 *10 * 128 = +/- 320000000 = 0x1312D000 | 29 bits + sign
   *   F:   signed Q24.7 ,            |range = +/- 250000     * 128 =      32000000 = 0x01E84800 | 25 bits + sign
   *
   * The trapezoid generator state contains the following information, that we will use to create and evaluate
   * the Bézier curve:
   *
   *  blk->step_event_count [TS] = The total count of steps for this movement. (=distance)
   *  blk->initial_rate     [VI] = The initial steps per second (=velocity)
   *  blk->final_rate       [VF] = The ending steps per second  (=velocity)
   *  and the count of events completed (step_events_completed) [CS] (=distance until now)
   *
   *  Note the abbreviations we use in the following formulae are between []s
   *
   *  For Any 32bit CPU:
   *
   *    At the start of each trapezoid, calculate the coefficients A,B,C,F and Advance [AV], as follows:
   *
   *      A =  6*128*(VF - VI) =  768*(VF - VI)
   *      B = 15*128*(VI - VF) = 1920*(VI - VF)
   *      C = 10*128*(VF - VI) = 1280*(VF - VI)
   *      F =    128*VI        =  128*VI
   *     AV = (1<<32)/TS      ~= 0xFFFFFFFF / TS (To use ARM UDIV, that is 32 bits) (this is computed at the planner, to offload expensive calculations from the ISR)
   *
   *    And for each point, evaluate the curve with the following sequence:
   *
   *      void lsrs(uint32_t& d, uint32_t s, int cnt) {
   *        d = s >> cnt;
   *      }
   *      void lsls(uint32_t& d, uint32_t s, int cnt) {
   *        d = s << cnt;
   *      }
   *      void lsrs(int32_t& d, uint32_t s, int cnt) {
   *        d = uint32_t(s) >> cnt;
   *      }
   *      void lsls(int32_t& d, uint32_t s, int cnt) {
   *        d = uint32_t(s) << cnt;
   *      }
   *      void umull(uint32_t& rlo, uint32_t& rhi, uint32_t op1, uint32_t op2) {
   *        uint64_t res = uint64_t(op1) * op2;
   *        rlo = uint32_t(res & 0xFFFFFFFF);
   *        rhi = uint32_t((res >> 32) & 0xFFFFFFFF);
   *      }
   *      void smlal(int32_t& rlo, int32_t& rhi, int32_t op1, int32_t op2) {
   *        int64_t mul = int64_t(op1) * op2;
   *        int64_t s = int64_t(uint32_t(rlo) | ((uint64_t(uint32_t(rhi)) << 32U)));
   *        mul += s;
   *        rlo = int32_t(mul & 0xFFFFFFFF);
   *        rhi = int32_t((mul >> 32) & 0xFFFFFFFF);
   *      }
   *      int32_t _eval_bezier_curve_arm(uint32_t curr_step) {
   *        uint32_t flo = 0;
   *        uint32_t fhi = bezier_AV * curr_step;
   *        uint32_t t = fhi;
   *        int32_t alo = bezier_F;
   *        int32_t ahi = 0;
   *        int32_t A = bezier_A;
   *        int32_t B = bezier_B;
   *        int32_t C = bezier_C;
   *
   *        lsrs(ahi, alo, 1);          // a  = F << 31
   *        lsls(alo, alo, 31);         //
   *        umull(flo, fhi, fhi, t);    // f *= t
   *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
   *        lsrs(flo, fhi, 1);          //
   *        smlal(alo, ahi, flo, C);    // a+=(f>>33)*C
   *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
   *        lsrs(flo, fhi, 1);          //
   *        smlal(alo, ahi, flo, B);    // a+=(f>>33)*B
   *        umull(flo, fhi, fhi, t);    // f>>=32; f*=t
   *        lsrs(flo, fhi, 1);          // f>>=33;
   *        smlal(alo, ahi, flo, A);    // a+=(f>>33)*A;
   *        lsrs(alo, ahi, 6);          // a>>=38
   *
   *        return alo;
   *      }
   *
   *  This is rewritten in ARM assembly for optimal performance (43 cycles to execute).
   *
   *  For AVR, the precision of coefficients is scaled so the Bézier curve can be evaluated in real-time:
   *  Let's reduce precision as much as possible. After some experimentation we found that:
   *
   *    Assume t and AV with 24 bits is enough
   *       A =  6*(VF - VI)
   *       B = 15*(VI - VF)
   *       C = 10*(VF - VI)
   *       F =     VI
   *      AV = (1<<24)/TS   (this is computed at the planner, to offload expensive calculations from the ISR)
   *
   *    Instead of storing sign for each coefficient, we will store its absolute value,
   *    and flag the sign of the A coefficient, so we can save to store the sign bit.
   *    It always holds that sign(A) = - sign(B) = sign(C)
   *
   *     So, the resulting range of the coefficients are:
   *
   *       t: unsigned (0 <= t < 1) |range 0 to 0xFFFFFF unsigned
   *       A:   signed Q24 , range = 250000 * 6 = 1500000 = 0x16E360 | 21 bits
   *       B:   signed Q24 , range = 250000 *15 = 3750000 = 0x393870 | 22 bits
   *       C:   signed Q24 , range = 250000 *10 = 2500000 = 0x1312D0 | 21 bits
   *       F:   signed Q24 , range = 250000     =  250000 = 0x0ED090 | 20 bits
   *
   *    And for each curve, estimate its coefficients with:
   *
   *      void _calc_bezier_curve_coeffs(int32_t v0, int32_t v1, uint32_t av) {
   *       // Calculate the Bézier coefficients
   *       if (v1 < v0) {
   *         A_negative = true;
   *         bezier_A = 6 * (v0 - v1);
   *         bezier_B = 15 * (v0 - v1);
   *         bezier_C = 10 * (v0 - v1);
   *       }
   *       else {
   *         A_negative = false;
   *         bezier_A = 6 * (v1 - v0);
   *         bezier_B = 15 * (v1 - v0);
   *         bezier_C = 10 * (v1 - v0);
   *       }
   *       bezier_F = v0;
   *      }
   *
   *    And for each point, evaluate the curve with the following sequence:
   *
   *      // unsigned multiplication of 24 bits x 24bits, return upper 16 bits
   *      void umul24x24to16hi(uint16_t& r, uint24_t op1, uint24_t op2) {
   *        r = (uint64_t(op1) * op2) >> 8;
   *      }
   *      // unsigned multiplication of 16 bits x 16bits, return upper 16 bits
   *      void umul16x16to16hi(uint16_t& r, uint16_t op1, uint16_t op2) {
   *        r = (uint32_t(op1) * op2) >> 16;
   *      }
   *      // unsigned multiplication of 16 bits x 24bits, return upper 24 bits
   *      void umul16x24to24hi(uint24_t& r, uint16_t op1, uint24_t op2) {
   *        r = uint24_t((uint64_t(op1) * op2) >> 16);
   *      }
   *
   *      int32_t _eval_bezier_curve(uint32_t curr_step) {
   *        // To save computing, the first step is always the initial speed
   *        if (!curr_step)
   *          return bezier_F;
   *
   *        uint16_t t;
   *        umul24x24to16hi(t, bezier_AV, curr_step);   // t: Range 0 - 1^16 = 16 bits
   *        uint16_t f = t;
   *        umul16x16to16hi(f, f, t);                   // Range 16 bits (unsigned)
   *        umul16x16to16hi(f, f, t);                   // Range 16 bits : f = t^3  (unsigned)
   *        uint24_t acc = bezier_F;                    // Range 20 bits (unsigned)
   *        if (A_negative) {
   *          uint24_t v;
   *          umul16x24to24hi(v, f, bezier_C);          // Range 21bits
   *          acc -= v;
   *          umul16x16to16hi(f, f, t);                 // Range 16 bits : f = t^4  (unsigned)
   *          umul16x24to24hi(v, f, bezier_B);          // Range 22bits
   *          acc += v;
   *          umul16x16to16hi(f, f, t);                 // Range 16 bits : f = t^5  (unsigned)
   *          umul16x24to24hi(v, f, bezier_A);          // Range 21bits + 15 = 36bits (plus sign)
   *          acc -= v;
   *        }
   *        else {
   *          uint24_t v;
   *          umul16x24to24hi(v, f, bezier_C);          // Range 21bits
   *          acc += v;
   *          umul16x16to16hi(f, f, t);                 // Range 16 bits : f = t^4  (unsigned)
   *          umul16x24to24hi(v, f, bezier_B);          // Range 22bits
   *          acc -= v;
   *          umul16x16to16hi(f, f, t);                 // Range 16 bits : f = t^5  (unsigned)
   *          umul16x24to24hi(v, f, bezier_A);          // Range 21bits + 15 = 36bits (plus sign)
   *          acc += v;
   *        }
   *        return acc;
   *      }
   *    These functions are translated to assembler for optimal performance.
   *    Coefficient calculation takes 70 cycles. Bezier point evaluation takes 150 cycles.
   */

  #ifdef __AVR__

    // For AVR we use assembly to maximize speed
    void Stepper::_calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av) {

      // Store advance
      bezier_AV = av;

      // Calculate the rest of the coefficients
      uint8_t r2 = v0 & 0xFF;
      uint8_t r3 = (v0 >> 8) & 0xFF;
      uint8_t r12 = (v0 >> 16) & 0xFF;
      uint8_t r5 = v1 & 0xFF;
      uint8_t r6 = (v1 >> 8) & 0xFF;
      uint8_t r7 = (v1 >> 16) & 0xFF;
      uint8_t r4,r8,r9,r10,r11;

      __asm__ __volatile__(
        /* Calculate the Bézier coefficients */
        /*  %10:%1:%0 = v0*/
        /*  %5:%4:%3 = v1*/
        /*  %7:%6:%10 = temporary*/
        /*  %9 = val (must be high register!)*/
        /*  %10 (must be high register!)*/

        /* Store initial velocity*/
        A("sts bezier_F, %0")
        A("sts bezier_F+1, %1")
        A("sts bezier_F+2, %10")    /* bezier_F = %10:%1:%0 = v0 */

        /* Get delta speed */
        A("ldi %2,-1")              /* %2 = 0xFF, means A_negative = true */
        A("clr %8")                 /* %8 = 0 */
        A("sub %0,%3")
        A("sbc %1,%4")
        A("sbc %10,%5")             /*  v0 -= v1, C=1 if result is negative */
        A("brcc 1f")                /* branch if result is positive (C=0), that means v0 >= v1 */

        /*  Result was negative, get the absolute value*/
        A("com %10")
        A("com %1")
        A("neg %0")
        A("sbc %1,%2")
        A("sbc %10,%2")             /* %10:%1:%0 +1  -> %10:%1:%0 = -(v0 - v1) = (v1 - v0) */
        A("clr %2")                 /* %2 = 0, means A_negative = false */

        /*  Store negative flag*/
        L("1")
        A("sts A_negative, %2")     /* Store negative flag */

        /*  Compute coefficients A,B and C   [20 cycles worst case]*/
        A("ldi %9,6")               /* %9 = 6 */
        A("mul %0,%9")              /* r1:r0 = 6*LO(v0-v1) */
        A("sts bezier_A, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:r0 = 6*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 6*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 6*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 6*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 6*HI(v0-v1) << 16 */
        A("sts bezier_A+1, %6")
        A("sts bezier_A+2, %7")     /* bezier_A = %7:%6:?? = 6*(v0-v1) [35 cycles worst] */

        A("ldi %9,15")              /* %9 = 15 */
        A("mul %0,%9")              /* r1:r0 = 5*LO(v0-v1) */
        A("sts bezier_B, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:?? = 5*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 5*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 5*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 5*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 5*HI(v0-v1) << 16 */
        A("sts bezier_B+1, %6")
        A("sts bezier_B+2, %7")     /* bezier_B = %7:%6:?? = 5*(v0-v1) [50 cycles worst] */

        A("ldi %9,10")              /* %9 = 10 */
        A("mul %0,%9")              /* r1:r0 = 10*LO(v0-v1) */
        A("sts bezier_C, r0")
        A("mov %6,r1")
        A("clr %7")                 /* %7:%6:?? = 10*LO(v0-v1) */
        A("mul %1,%9")              /* r1:r0 = 10*MI(v0-v1) */
        A("add %6,r0")
        A("adc %7,r1")              /* %7:%6:?? += 10*MI(v0-v1) << 8 */
        A("mul %10,%9")             /* r1:r0 = 10*HI(v0-v1) */
        A("add %7,r0")              /* %7:%6:?? += 10*HI(v0-v1) << 16 */
        A("sts bezier_C+1, %6")
        " sts bezier_C+2, %7"       /* bezier_C = %7:%6:?? = 10*(v0-v1) [65 cycles worst] */
        : "+r" (r2),
          "+d" (r3),
          "=r" (r4),
          "+r" (r5),
          "+r" (r6),
          "+r" (r7),
          "=r" (r8),
          "=r" (r9),
          "=r" (r10),
          "=d" (r11),
          "+r" (r12)
        :
        : "r0", "r1", "cc", "memory"
      );
    }

    FORCE_INLINE int32_t Stepper::_eval_bezier_curve(const uint32_t curr_step) {

      // If dealing with the first step, save expensive computing and return the initial speed
      if (!curr_step)
        return bezier_F;

      uint8_t r0 = 0; /* Zero register */
      uint8_t r2 = (curr_step) & 0xFF;
      uint8_t r3 = (curr_step >> 8) & 0xFF;
      uint8_t r4 = (curr_step >> 16) & 0xFF;
      uint8_t r1,r5,r6,r7,r8,r9,r10,r11; /* Temporary registers */

      __asm__ __volatile(
        /* umul24x24to16hi(t, bezier_AV, curr_step);  t: Range 0 - 1^16 = 16 bits*/
        A("lds %9,bezier_AV")       /* %9 = LO(AV)*/
        A("mul %9,%2")              /* r1:r0 = LO(bezier_AV)*LO(curr_step)*/
        A("mov %7,r1")              /* %7 = LO(bezier_AV)*LO(curr_step) >> 8*/
        A("clr %8")                 /* %8:%7  = LO(bezier_AV)*LO(curr_step) >> 8*/
        A("lds %10,bezier_AV+1")    /* %10 = MI(AV)*/
        A("mul %10,%2")             /* r1:r0  = MI(bezier_AV)*LO(curr_step)*/
        A("add %7,r0")
        A("adc %8,r1")              /* %8:%7 += MI(bezier_AV)*LO(curr_step)*/
        A("lds r1,bezier_AV+2")     /* r11 = HI(AV)*/
        A("mul r1,%2")              /* r1:r0  = HI(bezier_AV)*LO(curr_step)*/
        A("add %8,r0")              /* %8:%7 += HI(bezier_AV)*LO(curr_step) << 8*/
        A("mul %9,%3")              /* r1:r0 =  LO(bezier_AV)*MI(curr_step)*/
        A("add %7,r0")
        A("adc %8,r1")              /* %8:%7 += LO(bezier_AV)*MI(curr_step)*/
        A("mul %10,%3")             /* r1:r0 =  MI(bezier_AV)*MI(curr_step)*/
        A("add %8,r0")              /* %8:%7 += LO(bezier_AV)*MI(curr_step) << 8*/
        A("mul %9,%4")              /* r1:r0 =  LO(bezier_AV)*HI(curr_step)*/
        A("add %8,r0")              /* %8:%7 += LO(bezier_AV)*HI(curr_step) << 8*/
        /* %8:%7 = t*/

        /* uint16_t f = t;*/
        A("mov %5,%7")              /* %6:%5 = f*/
        A("mov %6,%8")
        /* %6:%5 = f*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits (unsigned) [17] */
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %9,r1")              /* store MIL(LO(f) * LO(t)) in %9, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %9,r0")              /* %9 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %9,r0")              /* %9 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t)) */
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 = */
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/
        /* [15 +17*2] = [49]*/

        /* %4:%3:%2 will be acc from now on*/

        /* uint24_t acc = bezier_F; / Range 20 bits (unsigned)*/
        A("clr %9")                 /* "decimal place we get for free"*/
        A("lds %2,bezier_F")
        A("lds %3,bezier_F+1")
        A("lds %4,bezier_F+2")      /* %4:%3:%2 = acc*/

        /* if (A_negative) {*/
        A("lds r0,A_negative")
        A("or r0,%0")               /* Is flag signalling negative? */
        A("brne 3f")                /* If yes, Skip next instruction if A was negative*/
        A("rjmp 1f")                /* Otherwise, jump */

        /* uint24_t v; */
        /* umul16x24to24hi(v, f, bezier_C); / Range 21bits [29] */
        /* acc -= v; */
        L("3")
        A("lds %10, bezier_C")      /* %10 = LO(bezier_C)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_C) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_C) * LO(f))*/
        A("lds %11, bezier_C+1")    /* %11 = MI(bezier_C)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_C) * LO(f)*/
        A("lds %1, bezier_C+2")     /* %1 = HI(bezier_C)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_C) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_C) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_C) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_C) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_C) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_C) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_C) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_B); / Range 22bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_B")      /* %10 = LO(bezier_B)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_B) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_B) * LO(f))*/
        A("lds %11, bezier_B+1")    /* %11 = MI(bezier_B)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_B) * LO(f)*/
        A("lds %1, bezier_B+2")     /* %1 = HI(bezier_B)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_B) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_B) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_B) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_B) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_B) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_B) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_B) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^5  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_A); / Range 21bits [29]*/
        /* acc -= v; */
        A("lds %10, bezier_A")      /* %10 = LO(bezier_A)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_A) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_A) * LO(f))*/
        A("lds %11, bezier_A+1")    /* %11 = MI(bezier_A)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_A) * LO(f)*/
        A("lds %1, bezier_A+2")     /* %1 = HI(bezier_A)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_A) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_A) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_A) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_A) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_A) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_A) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_A) * LO(f) << 16*/
        A("jmp 2f")                 /* Done!*/

        L("1")

        /* uint24_t v; */
        /* umul16x24to24hi(v, f, bezier_C); / Range 21bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_C")      /* %10 = LO(bezier_C)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_C) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_C) * LO(f))*/
        A("lds %11, bezier_C+1")    /* %11 = MI(bezier_C)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_C) * LO(f)*/
        A("lds %1, bezier_C+2")     /* %1 = HI(bezier_C)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_C) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_C) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_C) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_C) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_C) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_C) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_C) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_C) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^3  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_B); / Range 22bits [29]*/
        /* acc -= v;*/
        A("lds %10, bezier_B")      /* %10 = LO(bezier_B)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_B) * LO(f)*/
        A("sub %9,r1")
        A("sbc %2,%0")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(LO(bezier_B) * LO(f))*/
        A("lds %11, bezier_B+1")    /* %11 = MI(bezier_B)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_B) * LO(f)*/
        A("lds %1, bezier_B+2")     /* %1 = HI(bezier_B)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_B) * LO(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= HI(bezier_B) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_B) * MI(f)*/
        A("sub %9,r0")
        A("sbc %2,r1")
        A("sbc %3,%0")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= LO(bezier_B) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_B) * MI(f)*/
        A("sub %2,r0")
        A("sbc %3,r1")
        A("sbc %4,%0")              /* %4:%3:%2:%9 -= MI(bezier_B) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_B) * LO(f)*/
        A("sub %3,r0")
        A("sbc %4,r1")              /* %4:%3:%2:%9 -= HI(bezier_B) * LO(f) << 16*/

        /* umul16x16to16hi(f, f, t); / Range 16 bits : f = t^5  (unsigned) [17]*/
        A("mul %5,%7")              /* r1:r0 = LO(f) * LO(t)*/
        A("mov %1,r1")              /* store MIL(LO(f) * LO(t)) in %1, we need it for rounding*/
        A("clr %10")                /* %10 = 0*/
        A("clr %11")                /* %11 = 0*/
        A("mul %5,%8")              /* r1:r0 = LO(f) * HI(t)*/
        A("add %1,r0")              /* %1 += LO(LO(f) * HI(t))*/
        A("adc %10,r1")             /* %10 = HI(LO(f) * HI(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%7")              /* r1:r0 = HI(f) * LO(t)*/
        A("add %1,r0")              /* %1 += LO(HI(f) * LO(t))*/
        A("adc %10,r1")             /* %10 += HI(HI(f) * LO(t))*/
        A("adc %11,%0")             /* %11 += carry*/
        A("mul %6,%8")              /* r1:r0 = HI(f) * HI(t)*/
        A("add %10,r0")             /* %10 += LO(HI(f) * HI(t))*/
        A("adc %11,r1")             /* %11 += HI(HI(f) * HI(t))*/
        A("mov %5,%10")             /* %6:%5 =*/
        A("mov %6,%11")             /* f = %10:%11*/

        /* umul16x24to24hi(v, f, bezier_A); / Range 21bits [29]*/
        /* acc += v; */
        A("lds %10, bezier_A")      /* %10 = LO(bezier_A)*/
        A("mul %10,%5")             /* r1:r0 = LO(bezier_A) * LO(f)*/
        A("add %9,r1")
        A("adc %2,%0")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(LO(bezier_A) * LO(f))*/
        A("lds %11, bezier_A+1")    /* %11 = MI(bezier_A)*/
        A("mul %11,%5")             /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_A) * LO(f)*/
        A("lds %1, bezier_A+2")     /* %1 = HI(bezier_A)*/
        A("mul %1,%5")              /* r1:r0 = MI(bezier_A) * LO(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += HI(bezier_A) * LO(f) << 8*/
        A("mul %10,%6")             /* r1:r0 = LO(bezier_A) * MI(f)*/
        A("add %9,r0")
        A("adc %2,r1")
        A("adc %3,%0")
        A("adc %4,%0")              /* %4:%3:%2:%9 += LO(bezier_A) * MI(f)*/
        A("mul %11,%6")             /* r1:r0 = MI(bezier_A) * MI(f)*/
        A("add %2,r0")
        A("adc %3,r1")
        A("adc %4,%0")              /* %4:%3:%2:%9 += MI(bezier_A) * MI(f) << 8*/
        A("mul %1,%6")              /* r1:r0 = HI(bezier_A) * LO(f)*/
        A("add %3,r0")
        A("adc %4,r1")              /* %4:%3:%2:%9 += HI(bezier_A) * LO(f) << 16*/
        L("2")
        " clr __zero_reg__"         /* C runtime expects r1 = __zero_reg__ = 0 */
        : "+r"(r0),
          "+r"(r1),
          "+r"(r2),
          "+r"(r3),
          "+r"(r4),
          "+r"(r5),
          "+r"(r6),
          "+r"(r7),
          "+r"(r8),
          "+r"(r9),
          "+r"(r10),
          "+r"(r11)
        :
        :"cc","r0","r1"
      );
      return (r2 | (uint16_t(r3) << 8)) | (uint32_t(r4) << 16);
    }

  #else

    // For all the other 32bit CPUs
    FORCE_INLINE void Stepper::_calc_bezier_curve_coeffs(const int32_t v0, const int32_t v1, const uint32_t av) {
      // Calculate the Bézier coefficients
      bezier_A =  768 * (v1 - v0);
      bezier_B = 1920 * (v0 - v1);
      bezier_C = 1280 * (v1 - v0);
      bezier_F =  128 * v0;
      bezier_AV = av;
    }

    FORCE_INLINE int32_t Stepper::_eval_bezier_curve(const uint32_t curr_step) {
      #if (defined(__arm__) || defined(__thumb__)) && __ARM_ARCH >= 6 && !defined(STM32G0B1xx) // TODO: Test define STM32G0xx versus STM32G0B1xx

        // For ARM Cortex M3/M4 CPUs, we have the optimized assembler version, that takes 43 cycles to execute
        uint32_t flo = 0;
        uint32_t fhi = bezier_AV * curr_step;
        uint32_t t = fhi;
        int32_t alo = bezier_F;
        int32_t ahi = 0;
        int32_t A = bezier_A;
        int32_t B = bezier_B;
        int32_t C = bezier_C;

         __asm__ __volatile__(
          ".syntax unified" "\n\t"              // is to prevent CM0,CM1 non-unified syntax
          A("lsrs  %[ahi],%[alo],#1")           // a  = F << 31      1 cycles
          A("lsls  %[alo],%[alo],#31")          //                   1 cycles
          A("umull %[flo],%[fhi],%[fhi],%[t]")  // f *= t            5 cycles [fhi:flo=64bits]
          A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
          A("lsrs  %[flo],%[fhi],#1")           //                   1 cycles [31bits]
          A("smlal %[alo],%[ahi],%[flo],%[C]")  // a+=(f>>33)*C;     5 cycles
          A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
          A("lsrs  %[flo],%[fhi],#1")           //                   1 cycles [31bits]
          A("smlal %[alo],%[ahi],%[flo],%[B]")  // a+=(f>>33)*B;     5 cycles
          A("umull %[flo],%[fhi],%[fhi],%[t]")  // f>>=32; f*=t      5 cycles [fhi:flo=64bits]
          A("lsrs  %[flo],%[fhi],#1")           // f>>=33;           1 cycles [31bits]
          A("smlal %[alo],%[ahi],%[flo],%[A]")  // a+=(f>>33)*A;     5 cycles
          A("lsrs  %[alo],%[ahi],#6")           // a>>=38            1 cycles
          : [alo]"+r"( alo ) ,
            [flo]"+r"( flo ) ,
            [fhi]"+r"( fhi ) ,
            [ahi]"+r"( ahi ) ,
            [A]"+r"( A ) ,  // <== Note: Even if A, B, C, and t registers are INPUT ONLY
            [B]"+r"( B ) ,  //  GCC does bad optimizations on the code if we list them as
            [C]"+r"( C ) ,  //  such, breaking this function. So, to avoid that problem,
            [t]"+r"( t )    //  we list all registers as input-outputs.
          :
          : "cc"
        );
        return alo;

      #else

        // For non ARM targets, we provide a fallback implementation. Really doubt it
        // will be useful, unless the processor is fast and 32bit

        uint32_t t = bezier_AV * curr_step;               // t: Range 0 - 1^32 = 32 bits
        uint64_t f = t;
        f *= t;                                           // Range 32*2 = 64 bits (unsigned)
        f >>= 32;                                         // Range 32 bits  (unsigned)
        f *= t;                                           // Range 32*2 = 64 bits  (unsigned)
        f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
        int64_t acc = (int64_t) bezier_F << 31;           // Range 63 bits (signed)
        acc += ((uint32_t) f >> 1) * (int64_t) bezier_C;  // Range 29bits + 31 = 60bits (plus sign)
        f *= t;                                           // Range 32*2 = 64 bits
        f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
        acc += ((uint32_t) f >> 1) * (int64_t) bezier_B;  // Range 29bits + 31 = 60bits (plus sign)
        f *= t;                                           // Range 32*2 = 64 bits
        f >>= 32;                                         // Range 32 bits : f = t^3  (unsigned)
        acc += ((uint32_t) f >> 1) * (int64_t) bezier_A;  // Range 28bits + 31 = 59bits (plus sign)
        acc >>= (31 + 7);                                 // Range 24bits (plus sign)
        return (int32_t) acc;

      #endif
    }
  #endif
#endif // S_CURVE_ACCELERATION

/**
 * Stepper Driver Interrupt
 *
 * Directly pulses the stepper motors at high frequency.
 */

HAL_STEP_TIMER_ISR() {
  #ifndef __AVR__
    // Disable interrupts, to avoid ISR preemption while we reprogram the period
    // (AVR enters the ISR with global interrupts disabled, so no need to do it here)
    hal.isr_off();
  #endif

  HAL_timer_isr_prologue(MF_TIMER_STEP);

  Stepper::isr();

  HAL_timer_isr_epilogue(MF_TIMER_STEP);
}

#ifdef CPU_32_BIT
  #define STEP_MULTIPLY(A,B) MultiU32X24toH32(A, B)
#else
  #define STEP_MULTIPLY(A,B) MultiU24X32toH16(A, B)
#endif

void Stepper::isr() {

  static hal_timer_t nextMainISR = 0;  // Interval until the next main Stepper Pulse phase (0 = Now)

  // Program timer compare for the maximum period, so it does NOT
  // flag an interrupt while this ISR is running - So changes from small
  // periods to big periods are respected and the timer does not reset to 0
  HAL_timer_set_compare(MF_TIMER_STEP, hal_timer_t(HAL_TIMER_TYPE_MAX));

  // Count of ticks for the next ISR
  hal_timer_t next_isr_ticks = 0;

  // Limit the amount of iterations
  uint8_t max_loops = 10;

  #if ENABLED(FT_MOTION)
    static uint32_t ftMotion_nextAuxISR = 0U;  // Storage for the next ISR of the auxilliary tasks.
    const bool using_ftMotion = ftMotion.cfg.active;
  #else
    constexpr bool using_ftMotion = false;
  #endif

  // We need this variable here to be able to use it in the following loop
  hal_timer_t min_ticks;
  do {

    hal_timer_t interval = 0;

    #if ENABLED(FT_MOTION)

      if (using_ftMotion) {
        if (!nextMainISR) {               // Main ISR is ready to fire during this iteration?
          nextMainISR = FTM_MIN_TICKS;    // Set to minimum interval (a limit on the top speed)
          ftMotion_stepper();             // Run FTM Stepping
          // Define 2.5 msec task for auxilliary functions.
          if (!ftMotion_nextAuxISR) {
            TERN_(BABYSTEPPING, if (babystep.has_steps()) babystepping_isr());
            ftMotion_nextAuxISR = (STEPPER_TIMER_RATE) / 400;
          }
        }

        // Enable ISRs to reduce latency for higher priority ISRs, or all ISRs if no prioritization.
        hal.isr_on();

        interval = _MIN(nextMainISR, ftMotion_nextAuxISR);
        nextMainISR -= interval;
        ftMotion_nextAuxISR -= interval;
      }

    #endif

    if (!using_ftMotion) {

      TERN_(HAS_ZV_SHAPING, shaping_isr());               // Do Shaper stepping, if needed

      if (!nextMainISR) pulse_phase_isr();                // 0 = Do coordinated axes Stepper pulses

      #if ENABLED(LIN_ADVANCE)
        if (!nextAdvanceISR) {                            // 0 = Do Linear Advance E Stepper pulses
          advance_isr();
          nextAdvanceISR = la_interval;
        }
        else if (nextAdvanceISR > la_interval)            // Start/accelerate LA steps if necessary
          nextAdvanceISR = la_interval;
      #endif

      #if ENABLED(BABYSTEPPING)
        const bool is_babystep = (nextBabystepISR == 0);  // 0 = Do Babystepping (XY)Z pulses
        if (is_babystep) nextBabystepISR = babystepping_isr();
      #endif

      // Enable ISRs to reduce latency for higher priority ISRs, or all ISRs if no prioritization.
      hal.isr_on();

      // ^== Time critical. NOTHING besides pulse generation should be above here!!!

      if (!nextMainISR) nextMainISR = block_phase_isr();  // Manage acc/deceleration, get next block

      #if ENABLED(BABYSTEPPING)
        if (is_babystep)                                  // Avoid ANY stepping too soon after baby-stepping
          NOLESS(nextMainISR, (BABYSTEP_TICKS) / 8);      // FULL STOP for 125µs after a baby-step

        if (nextBabystepISR != BABYSTEP_NEVER)            // Avoid baby-stepping too close to axis Stepping
          NOLESS(nextBabystepISR, nextMainISR / 2);       // TODO: Only look at axes enabled for baby-stepping
      #endif

      // Get the interval to the next ISR call
      interval = _MIN(nextMainISR, uint32_t(HAL_TIMER_TYPE_MAX));         // Time until the next Pulse / Block phase
      TERN_(INPUT_SHAPING_X, NOMORE(interval, ShapingQueue::peek_x()));   // Time until next input shaping echo for X
      TERN_(INPUT_SHAPING_Y, NOMORE(interval, ShapingQueue::peek_y()));   // Time until next input shaping echo for Y
      TERN_(INPUT_SHAPING_Z, NOMORE(interval, ShapingQueue::peek_z()));   // Time until next input shaping echo for Z
      TERN_(LIN_ADVANCE, NOMORE(interval, nextAdvanceISR));               // Come back early for Linear Advance?
      TERN_(BABYSTEPPING, NOMORE(interval, nextBabystepISR));             // Come back early for Babystepping?

      //
      // Compute remaining time for each ISR phase
      //     NEVER : The phase is idle
      //      Zero : The phase will occur on the next ISR call
      //  Non-zero : The phase will occur on a future ISR call
      //

      nextMainISR -= interval;
      TERN_(HAS_ZV_SHAPING, ShapingQueue::decrement_delays(interval));
      TERN_(LIN_ADVANCE, if (nextAdvanceISR != LA_ADV_NEVER) nextAdvanceISR -= interval);
      TERN_(BABYSTEPPING, if (nextBabystepISR != BABYSTEP_NEVER) nextBabystepISR -= interval);

    } // standard motion control

    /**
     * This needs to avoid a race-condition caused by interleaving
     * of interrupts required by both the LA and Stepper algorithms.
     *
     * Assume the following tick times for stepper pulses:
     *   Stepper ISR (S):  1 1000 2000 3000 4000
     *   Linear Adv. (E): 10 1010 2010 3010 4010
     *
     * The current algorithm tries to interleave them, giving:
     *  1:S 10:E 1000:S 1010:E 2000:S 2010:E 3000:S 3010:E 4000:S 4010:E
     *
     * Ideal timing would yield these delta periods:
     *  1:S  9:E  990:S   10:E  990:S   10:E  990:S   10:E  990:S   10:E
     *
     * But, since each event must fire an ISR with a minimum duration, the
     * minimum delta might be 900, so deltas under 900 get rounded up:
     *  900:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E d990:S d900:E
     *
     * It works, but divides the speed of all motors by half, leading to a sudden
     * reduction to 1/2 speed! Such jumps in speed lead to lost steps (not even
     * accounting for double/quad stepping, which makes it even worse).
     */

    // Compute the tick count for the next ISR
    next_isr_ticks += interval;

    /**
     * The following section must be done with global interrupts disabled.
     * We want nothing to interrupt it, as that could mess the calculations
     * we do for the next value to program in the period register of the
     * stepper timer and lead to skipped ISRs (if the value we happen to program
     * is less than the current count due to something preempting between the
     * read and the write of the new period value).
     */
    hal.isr_off();

    /**
     * Get the current tick value + margin
     * Assuming at least 6µs between calls to this ISR...
     * On AVR the ISR epilogue+prologue is estimated at 100 instructions - Give 8µs as margin
     * On ARM the ISR epilogue+prologue is estimated at 20 instructions - Give 1µs as margin
     */
    min_ticks = HAL_timer_get_count(MF_TIMER_STEP) + hal_timer_t(TERN(__AVR__, 8, 1) * (STEPPER_TIMER_TICKS_PER_US));

    #if ENABLED(OLD_ADAPTIVE_MULTISTEPPING)
      /**
       * NB: If for some reason the stepper monopolizes the MPU, eventually the
       * timer will wrap around (and so will 'next_isr_ticks'). So, limit the
       * loop to 10 iterations. Beyond that, there's no way to ensure correct pulse
       * timing, since the MCU isn't fast enough.
       */
      if (!--max_loops) next_isr_ticks = min_ticks;
    #endif

    // Advance pulses if not enough time to wait for the next ISR
  } while (TERN(OLD_ADAPTIVE_MULTISTEPPING, true, --max_loops) && next_isr_ticks < min_ticks);

  #if DISABLED(OLD_ADAPTIVE_MULTISTEPPING)

    // Track the time spent in the ISR
    const hal_timer_t time_spent = HAL_timer_get_count(MF_TIMER_STEP);
    time_spent_in_isr += time_spent;

    if (next_isr_ticks < min_ticks) {
      next_isr_ticks = min_ticks;

      // When forced out of the ISR, increase multi-stepping
      #if MULTISTEPPING_LIMIT > 1
        if (steps_per_isr < MULTISTEPPING_LIMIT) {
          steps_per_isr <<= 1;
          // ticks_nominal will need to be recalculated if we are in cruise phase
          ticks_nominal = 0;
        }
      #endif
    }
    else {
      // Track the time spent voluntarily outside the ISR
      time_spent_out_isr += next_isr_ticks;
      time_spent_out_isr -= time_spent;
    }

  #endif // !OLD_ADAPTIVE_MULTISTEPPING

  // Now 'next_isr_ticks' contains the period to the next Stepper ISR - And we are
  // sure that the time has not arrived yet - Warrantied by the scheduler

  // Set the next ISR to fire at the proper time
  HAL_timer_set_compare(MF_TIMER_STEP, next_isr_ticks);

  // Don't forget to finally reenable interrupts on non-AVR.
  // AVR automatically calls sei() for us on Return-from-Interrupt.
  #ifndef __AVR__
    hal.isr_on();
  #endif
}

#if MINIMUM_STEPPER_PULSE_NS || MAXIMUM_STEPPER_RATE
  #define ISR_PULSE_CONTROL 1
#endif
#if ISR_PULSE_CONTROL && MULTISTEPPING_LIMIT > 1 && DISABLED(I2S_STEPPER_STREAM)
  #define ISR_MULTI_STEPS 1
#endif

/**
 * This phase of the ISR should ONLY create the pulses for the steppers.
 * This prevents jitter caused by the interval between the start of the
 * interrupt and the start of the pulses. DON'T add any logic ahead of the
 * call to this method that might cause variation in the timing. The aim
 * is to keep pulse timing as regular as possible.
 */
void Stepper::pulse_phase_isr() {

  // If we must abort the current block, do so!
  if (abort_current_block) {
    abort_current_block = false;
    if (current_block) {
      discard_current_block();
      #if HAS_ZV_SHAPING
        ShapingQueue::purge();
        #if ENABLED(INPUT_SHAPING_X)
          shaping_x.delta_error = 0;
          shaping_x.last_block_end_pos = count_position.x;
        #endif
        #if ENABLED(INPUT_SHAPING_Y)
          shaping_y.delta_error = 0;
          shaping_y.last_block_end_pos = count_position.y;
        #endif
        #if ENABLED(INPUT_SHAPING_Z)
          shaping_z.delta_error = 0;
          shaping_z.last_block_end_pos = count_position.z;
        #endif
      #endif
    }
  }

  // If there is no current block, do nothing
  if (!current_block || step_events_completed >= step_event_count) return;

  // Skipping step processing causes motion to freeze
  if (TERN0(FREEZE_FEATURE, frozen)) return;

  // Count of pending loops and events for this iteration
  const uint32_t pending_events = step_event_count - step_events_completed;
  uint8_t events_to_do = _MIN(pending_events, steps_per_isr);

  // Just update the value we will get at the end of the loop
  step_events_completed += events_to_do;

  TERN_(ISR_PULSE_CONTROL, USING_TIMED_PULSE());

  // Take multiple steps per interrupt. For high speed moves.
  #if ENABLED(ISR_MULTI_STEPS)
    bool firstStep = true;
  #endif

  // Direct Stepping page?
  const bool is_page = current_block->is_page();

  do {
    AxisFlags step_needed{0};

    #define _APPLY_STEP(AXIS, STATE, ALWAYS) AXIS ##_APPLY_STEP(STATE, ALWAYS)
    #define _STEP_STATE(AXIS) STEP_STATE_## AXIS

    // Determine if a pulse is needed using Bresenham
    #define PULSE_PREP(AXIS) do{ \
      int32_t de = delta_error[_AXIS(AXIS)] + advance_dividend[_AXIS(AXIS)]; \
      if (de >= 0) { \
        step_needed.set(_AXIS(AXIS)); \
        de -= advance_divisor_cached; \
      } \
      delta_error[_AXIS(AXIS)] = de; \
    }while(0)

    // With input shaping, direction changes can happen with almost only
    // AWAIT_LOW_PULSE() and  DIR_WAIT_BEFORE() between steps. To work around
    // the TMC2208 / TMC2225 shutdown bug (#16076), add a half step hysteresis
    // in each direction. This results in the position being off by half an
    // average half step during travel but correct at the end of each segment.
    #if AXIS_DRIVER_TYPE_X(TMC2208) || AXIS_DRIVER_TYPE_X(TMC2208_STANDALONE) || \
        AXIS_DRIVER_TYPE_X(TMC5160) || AXIS_DRIVER_TYPE_X(TMC5160_STANDALONE)
      #define HYSTERESIS_X 64
    #else
      #define HYSTERESIS_X 0
    #endif
    #if AXIS_DRIVER_TYPE_Y(TMC2208) || AXIS_DRIVER_TYPE_Y(TMC2208_STANDALONE) || \
        AXIS_DRIVER_TYPE_Y(TMC5160) || AXIS_DRIVER_TYPE_Y(TMC5160_STANDALONE)
      #define HYSTERESIS_Y 64
    #else
      #define HYSTERESIS_Y 0
    #endif
    #if AXIS_DRIVER_TYPE_Z(TMC2208) || AXIS_DRIVER_TYPE_Z(TMC2208_STANDALONE) || \
        AXIS_DRIVER_TYPE_Z(TMC5160) || AXIS_DRIVER_TYPE_Z(TMC5160_STANDALONE)
      #define HYSTERESIS_Z 64
    #else
      #define HYSTERESIS_Z 0
    #endif
    #define _HYSTERESIS(AXIS) HYSTERESIS_##AXIS
    #define HYSTERESIS(AXIS) _HYSTERESIS(AXIS)

    #define PULSE_PREP_SHAPING(AXIS, DELTA_ERROR, DIVIDEND) do{ \
      int16_t de = DELTA_ERROR + (DIVIDEND); \
      const bool step_fwd = de >=  (64 + HYSTERESIS(AXIS)), \
                 step_bak = de <= -(64 + HYSTERESIS(AXIS)); \
      if (step_fwd || step_bak) { \
        de += step_fwd ? -128 : 128; \
        if ((MAXDIR(AXIS) && step_bak) || (MINDIR(AXIS) && step_fwd)) { \
          { USING_TIMED_PULSE(); START_TIMED_PULSE(); AWAIT_LOW_PULSE(); } \
          last_direction_bits.toggle(_AXIS(AXIS)); \
          DIR_WAIT_BEFORE(); \
          SET_STEP_DIR(AXIS); \
          TERN_(FTM_OPTIMIZE_DIR_STATES, last_set_direction = last_direction_bits); \
          DIR_WAIT_AFTER(); \
        } \
      } \
      else \
        step_needed.clear(_AXIS(AXIS)); \
      DELTA_ERROR = de; \
    }while(0)

    // Start an active pulse if needed
    #define PULSE_START(AXIS) do{ \
      if (step_needed.test(_AXIS(AXIS))) { \
        count_position[_AXIS(AXIS)] += count_direction[_AXIS(AXIS)]; \
        _APPLY_STEP(AXIS, _STEP_STATE(AXIS), 0); \
      } \
    }while(0)

    // Stop an active pulse if needed
    #define PULSE_STOP(AXIS) do { \
      if (step_needed.test(_AXIS(AXIS))) { \
        _APPLY_STEP(AXIS, !_STEP_STATE(AXIS), 0); \
      } \
    }while(0)

    #if ENABLED(DIRECT_STEPPING)
      // Direct stepping is currently not ready for HAS_I_AXIS
      if (is_page) {

        #if STEPPER_PAGE_FORMAT == SP_4x4D_128

          #define PAGE_SEGMENT_UPDATE(AXIS, VALUE) do{      \
                 if ((VALUE) <  7) dm[_AXIS(AXIS)] = false; \
            else if ((VALUE) >  7) dm[_AXIS(AXIS)] = true;  \
            page_step_state.sd[_AXIS(AXIS)] = VALUE;        \
            page_step_state.bd[_AXIS(AXIS)] += VALUE;       \
          }while(0)

          #define PAGE_PULSE_PREP(AXIS) do{ \
            step_needed.set(_AXIS(AXIS), \
              pgm_read_byte(&segment_table[page_step_state.sd[_AXIS(AXIS)]][page_step_state.segment_steps & 0x7])); \
          }while(0)

          switch (page_step_state.segment_steps) {
            case DirectStepping::Config::SEGMENT_STEPS:
              page_step_state.segment_idx += 2;
              page_step_state.segment_steps = 0;
              // fallthru
            case 0: {
              const uint8_t low = page_step_state.page[page_step_state.segment_idx],
                           high = page_step_state.page[page_step_state.segment_idx + 1];
              const AxisBits dm = last_direction_bits;

              PAGE_SEGMENT_UPDATE(X, low >> 4);
              PAGE_SEGMENT_UPDATE(Y, low & 0xF);
              PAGE_SEGMENT_UPDATE(Z, high >> 4);
              PAGE_SEGMENT_UPDATE(E, high & 0xF);

              if (dm != last_direction_bits) set_directions(dm);

            } break;

            default: break;
          }

          PAGE_PULSE_PREP(X);
          PAGE_PULSE_PREP(Y);
          PAGE_PULSE_PREP(Z);
          TERN_(HAS_EXTRUDERS, PAGE_PULSE_PREP(E));

          page_step_state.segment_steps++;

        #elif STEPPER_PAGE_FORMAT == SP_4x2_256

          #define PAGE_SEGMENT_UPDATE(AXIS, VALUE) \
            page_step_state.sd[_AXIS(AXIS)] = VALUE; \
            page_step_state.bd[_AXIS(AXIS)] += VALUE;

          #define PAGE_PULSE_PREP(AXIS) do{ \
            step_needed.set(_AXIS(AXIS), \
              pgm_read_byte(&segment_table[page_step_state.sd[_AXIS(AXIS)]][page_step_state.segment_steps & 0x3])); \
          }while(0)

          switch (page_step_state.segment_steps) {
            case DirectStepping::Config::SEGMENT_STEPS:
              page_step_state.segment_idx++;
              page_step_state.segment_steps = 0;
              // fallthru
            case 0: {
              const uint8_t b = page_step_state.page[page_step_state.segment_idx];
              PAGE_SEGMENT_UPDATE(X, (b >> 6) & 0x3);
              PAGE_SEGMENT_UPDATE(Y, (b >> 4) & 0x3);
              PAGE_SEGMENT_UPDATE(Z, (b >> 2) & 0x3);
              PAGE_SEGMENT_UPDATE(E, (b >> 0) & 0x3);
            } break;
            default: break;
          }

          PAGE_PULSE_PREP(X);
          PAGE_PULSE_PREP(Y);
          PAGE_PULSE_PREP(Z);
          TERN_(HAS_EXTRUDERS, PAGE_PULSE_PREP(E));

          page_step_state.segment_steps++;

        #elif STEPPER_PAGE_FORMAT == SP_4x1_512

          #define PAGE_PULSE_PREP(AXIS, NBIT) do{            \
            step_needed.set(_AXIS(AXIS), TEST(steps, NBIT)); \
            if (step_needed.test(_AXIS(AXIS)))               \
              page_step_state.bd[_AXIS(AXIS)]++;             \
          }while(0)

          uint8_t steps = page_step_state.page[page_step_state.segment_idx >> 1];
          if (page_step_state.segment_idx & 0x1) steps >>= 4;

          PAGE_PULSE_PREP(X, 3);
          PAGE_PULSE_PREP(Y, 2);
          PAGE_PULSE_PREP(Z, 1);
          PAGE_PULSE_PREP(E, 0);

          page_step_state.segment_idx++;

        #else
          #error "Unknown direct stepping page format!"
        #endif
      }

    #endif // DIRECT_STEPPING

    if (!is_page) {
      // Give the compiler a clue to store advance_divisor in registers for what follows
      const uint32_t advance_divisor_cached = advance_divisor;

      // Determine if pulses are needed
      #if HAS_X_STEP
        PULSE_PREP(X);
      #endif
      #if HAS_Y_STEP
        PULSE_PREP(Y);
      #endif
      #if HAS_Z_STEP
        PULSE_PREP(Z);
      #endif
      #if HAS_I_STEP
        PULSE_PREP(I);
      #endif
      #if HAS_J_STEP
        PULSE_PREP(J);
      #endif
      #if HAS_K_STEP
        PULSE_PREP(K);
      #endif
      #if HAS_U_STEP
        PULSE_PREP(U);
      #endif
      #if HAS_V_STEP
        PULSE_PREP(V);
      #endif
      #if HAS_W_STEP
        PULSE_PREP(W);
      #endif

      #if ANY(HAS_E0_STEP, MIXING_EXTRUDER)
        PULSE_PREP(E);

        #if ENABLED(LIN_ADVANCE)
          if (la_active && step_needed.e) {
            // don't actually step here, but do subtract movements steps
            // from the linear advance step count
            step_needed.e = false;
            la_advance_steps--;
          }
        #endif
      #endif

      #if HAS_ZV_SHAPING
        // record an echo if a step is needed in the primary bresenham
        const bool x_step = TERN0(INPUT_SHAPING_X, step_needed.x && shaping_x.enabled),
                   y_step = TERN0(INPUT_SHAPING_Y, step_needed.y && shaping_y.enabled),
                   z_step = TERN0(INPUT_SHAPING_Z, step_needed.z && shaping_z.enabled);
        if (x_step || y_step || z_step)
          ShapingQueue::enqueue(x_step, TERN0(INPUT_SHAPING_X, shaping_x.forward), y_step, TERN0(INPUT_SHAPING_Y, shaping_y.forward), z_step, TERN0(INPUT_SHAPING_Z, shaping_z.forward));

        // do the first part of the secondary bresenham
        #if ENABLED(INPUT_SHAPING_X)
          if (x_step)
            PULSE_PREP_SHAPING(X, shaping_x.delta_error, shaping_x.forward ? shaping_x.factor1 : -shaping_x.factor1);
        #endif
        #if ENABLED(INPUT_SHAPING_Y)
          if (y_step)
            PULSE_PREP_SHAPING(Y, shaping_y.delta_error, shaping_y.forward ? shaping_y.factor1 : -shaping_y.factor1);
        #endif
        #if ENABLED(INPUT_SHAPING_Z)
          if (z_step)
            PULSE_PREP_SHAPING(Z, shaping_z.delta_error, shaping_z.forward ? shaping_z.factor1 : -shaping_z.factor1);
        #endif
      #endif
    }

    #if ISR_MULTI_STEPS
      if (firstStep)
        firstStep = false;
      else
        AWAIT_LOW_PULSE();
    #endif

    // Pulse start
    #if HAS_X_STEP
      PULSE_START(X);
    #endif
    #if HAS_Y_STEP
      PULSE_START(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_START(Z);
    #endif
    #if HAS_I_STEP
      PULSE_START(I);
    #endif
    #if HAS_J_STEP
      PULSE_START(J);
    #endif
    #if HAS_K_STEP
      PULSE_START(K);
    #endif
    #if HAS_U_STEP
      PULSE_START(U);
    #endif
    #if HAS_V_STEP
      PULSE_START(V);
    #endif
    #if HAS_W_STEP
      PULSE_START(W);
    #endif

    #if ENABLED(MIXING_EXTRUDER)
      if (step_needed.e) {
        count_position.e += count_direction.e;
        E_STEP_WRITE(mixer.get_next_stepper(), STEP_STATE_E);
      }
    #elif HAS_E0_STEP
      PULSE_START(E);
    #endif

    TERN_(I2S_STEPPER_STREAM, i2s_push_sample());

    // TODO: need to deal with MINIMUM_STEPPER_PULSE_NS over i2s
    #if ISR_PULSE_CONTROL
      START_TIMED_PULSE();
      AWAIT_HIGH_PULSE();
    #endif

    // Pulse stop
    #if HAS_X_STEP
      PULSE_STOP(X);
    #endif
    #if HAS_Y_STEP
      PULSE_STOP(Y);
    #endif
    #if HAS_Z_STEP
      PULSE_STOP(Z);
    #endif
    #if HAS_I_STEP
      PULSE_STOP(I);
    #endif
    #if HAS_J_STEP
      PULSE_STOP(J);
    #endif
    #if HAS_K_STEP
      PULSE_STOP(K);
    #endif
    #if HAS_U_STEP
      PULSE_STOP(U);
    #endif
    #if HAS_V_STEP
      PULSE_STOP(V);
    #endif
    #if HAS_W_STEP
      PULSE_STOP(W);
    #endif

    #if ENABLED(MIXING_EXTRUDER)
      if (step_needed.e) E_STEP_WRITE(mixer.get_stepper(), !STEP_STATE_E);
    #elif HAS_E0_STEP
      PULSE_STOP(E);
    #endif

    #if ISR_MULTI_STEPS
      if (events_to_do) START_TIMED_PULSE();
    #endif

  } while (--events_to_do);
}

#if HAS_ZV_SHAPING

  void Stepper::shaping_isr() {
    AxisFlags step_needed{0};

    // Clear the echoes that are ready to process. If the buffers are too full and risk overflow, also apply echoes early.
    TERN_(INPUT_SHAPING_X, step_needed.x = !ShapingQueue::peek_x() || ShapingQueue::free_count_x() < steps_per_isr);
    TERN_(INPUT_SHAPING_Y, step_needed.y = !ShapingQueue::peek_y() || ShapingQueue::free_count_y() < steps_per_isr);
    TERN_(INPUT_SHAPING_Z, step_needed.z = !ShapingQueue::peek_z() || ShapingQueue::free_count_z() < steps_per_isr);

    if (bool(step_needed)) while (true) {
      #if ENABLED(INPUT_SHAPING_X)
        if (step_needed.x) {
          const bool forward = ShapingQueue::dequeue_x();
          PULSE_PREP_SHAPING(X, shaping_x.delta_error, (forward ? shaping_x.factor2 : -shaping_x.factor2));
          PULSE_START(X);
        }
      #endif

      #if ENABLED(INPUT_SHAPING_Y)
        if (step_needed.y) {
          const bool forward = ShapingQueue::dequeue_y();
          PULSE_PREP_SHAPING(Y, shaping_y.delta_error, (forward ? shaping_y.factor2 : -shaping_y.factor2));
          PULSE_START(Y);
        }
      #endif

      #if ENABLED(INPUT_SHAPING_Z)
        if (step_needed.z) {
          const bool forward = ShapingQueue::dequeue_z();
          PULSE_PREP_SHAPING(Z, shaping_z.delta_error, (forward ? shaping_z.factor2 : -shaping_z.factor2));
          PULSE_START(Z);
        }
      #endif

      TERN_(I2S_STEPPER_STREAM, i2s_push_sample());

      USING_TIMED_PULSE();
      if (bool(step_needed)) {
        #if ISR_MULTI_STEPS
          START_TIMED_PULSE();
          AWAIT_HIGH_PULSE();
        #endif
        #if ENABLED(INPUT_SHAPING_X)
          PULSE_STOP(X);
        #endif
        #if ENABLED(INPUT_SHAPING_Y)
          PULSE_STOP(Y);
        #endif
        #if ENABLED(INPUT_SHAPING_Z)
          PULSE_STOP(Z);
        #endif
      }

      TERN_(INPUT_SHAPING_X, step_needed.x = !ShapingQueue::peek_x() || ShapingQueue::free_count_x() < steps_per_isr);
      TERN_(INPUT_SHAPING_Y, step_needed.y = !ShapingQueue::peek_y() || ShapingQueue::free_count_y() < steps_per_isr);
      TERN_(INPUT_SHAPING_Z, step_needed.z = !ShapingQueue::peek_z() || ShapingQueue::free_count_z() < steps_per_isr);

      if (!bool(step_needed)) break;

      START_TIMED_PULSE();
      AWAIT_LOW_PULSE();
    }
  }

#endif // HAS_ZV_SHAPING

// Calculate timer interval, with all limits applied.
hal_timer_t Stepper::calc_timer_interval(uint32_t step_rate) {

  #ifdef CPU_32_BIT

    // A fast processor can just do integer division
    return step_rate > minimal_step_rate ? uint32_t(STEPPER_TIMER_RATE) / step_rate : HAL_TIMER_TYPE_MAX;

  #else

    if (step_rate >= 0x0800) {  // higher step rate
      // AVR is able to keep up at around 65kHz Stepping ISR rate at most.
      // So values for step_rate > 65535 might as well be truncated.
      // Handle it as quickly as possible. i.e., assume highest byte is zero
      // because non-zero would represent a step rate far beyond AVR capabilities.
      if (uint8_t(step_rate >> 16))
        return uint32_t(STEPPER_TIMER_RATE) / 0x10000;

      const uintptr_t table_address = uintptr_t(&speed_lookuptable_fast[uint8_t(step_rate >> 8)]);
      const uint16_t base = uint16_t(pgm_read_word(table_address));
      const uint8_t gain = uint8_t(pgm_read_byte(table_address + 2));
      return base - MultiU8X8toH8(uint8_t(step_rate & 0x00FF), gain);
    }
    else if (step_rate > minimal_step_rate) { // lower step rates
      step_rate -= minimal_step_rate; // Correct for minimal speed
      const uintptr_t table_address = uintptr_t(&speed_lookuptable_slow[uint8_t(step_rate >> 3)]);
      return uint16_t(pgm_read_word(table_address))
             - ((uint16_t(pgm_read_word(table_address + 2)) * uint8_t(step_rate & 0x0007)) >> 3);
    }

    return uint16_t(pgm_read_word(uintptr_t(speed_lookuptable_slow)));

  #endif // !CPU_32_BIT
}

#if ENABLED(NONLINEAR_EXTRUSION)
  void Stepper::calc_nonlinear_e(uint32_t step_rate) {
    const uint32_t velocity = ne_scale * step_rate; // Scale step_rate first so all intermediate values stay in range of 8.24 fixed point math
    int32_t vd =  (((((int64_t)ne_fix.A * velocity) >> 24) * velocity) >> 24) + (((int64_t)ne_fix.B * velocity) >> 24);
    NOLESS(vd, 0);

    advance_dividend.e = (uint64_t(ne_fix.C + vd) * ne_edividend) >> 24;
  }
#endif

// Get the timer interval and the number of loops to perform per tick
hal_timer_t Stepper::calc_multistep_timer_interval(uint32_t step_rate) {

  #if ENABLED(OLD_ADAPTIVE_MULTISTEPPING)

    #if MULTISTEPPING_LIMIT == 1

      // Just make sure the step rate is doable
      NOMORE(step_rate, uint32_t(MAX_STEP_ISR_FREQUENCY_1X));

    #else

      // The stepping frequency limits for each multistepping rate
      static const uint32_t limit[] PROGMEM = {
            max_step_isr_frequency_sh(0)
          , max_step_isr_frequency_sh(1)
        #if MULTISTEPPING_LIMIT >= 4
          , max_step_isr_frequency_sh(2)
        #endif
        #if MULTISTEPPING_LIMIT >= 8
          , max_step_isr_frequency_sh(3)
        #endif
        #if MULTISTEPPING_LIMIT >= 16
          , max_step_isr_frequency_sh(4)
        #endif
        #if MULTISTEPPING_LIMIT >= 32
          , max_step_isr_frequency_sh(5)
        #endif
        #if MULTISTEPPING_LIMIT >= 64
          , max_step_isr_frequency_sh(6)
        #endif
        #if MULTISTEPPING_LIMIT >= 128
          , max_step_isr_frequency_sh(7)
        #endif
      };

      // Find a doable step rate using multistepping
      uint8_t multistep = 1;
      for (uint8_t i = 0; i < COUNT(limit) && step_rate > uint32_t(pgm_read_dword(&limit[i])); ++i) {
        step_rate >>= 1;
        multistep <<= 1;
      }
      steps_per_isr = multistep;

    #endif

  #elif MULTISTEPPING_LIMIT > 1

    uint8_t loops = steps_per_isr;
    if (MULTISTEPPING_LIMIT >= 16 && loops >= 16) { step_rate >>= 4; loops >>= 4; }
    if (MULTISTEPPING_LIMIT >=  4 && loops >=  4) { step_rate >>= 2; loops >>= 2; }
    if (MULTISTEPPING_LIMIT >=  2 && loops >=  2) { step_rate >>= 1; }

  #endif

  return calc_timer_interval(step_rate);
}

// Method to get all moving axes (for proper endstop handling)
void Stepper::set_axis_moved_for_current_block() {

  #if IS_CORE
    // Define conditions for checking endstops
    #define S_(N) current_block->steps[CORE_AXIS_##N]
    #define D_(N) current_block->direction_bits[CORE_AXIS_##N]
  #endif

  #if CORE_IS_XY || CORE_IS_XZ
    /**
     * Head direction in -X axis for CoreXY and CoreXZ bots.
     *
     * If steps differ, both axes are moving.
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z, handled below)
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X)
     */
    #if ANY(COREXY, COREXZ)
      #define X_CMP(A,B) ((A)==(B))
    #else
      #define X_CMP(A,B) ((A)!=(B))
    #endif
    #define X_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && X_CMP(D_(1),D_(2))) )
  #elif ENABLED(MARKFORGED_XY)
    #define X_MOVE_TEST (current_block->steps.a != current_block->steps.b)
  #else
    #define X_MOVE_TEST !!current_block->steps.a
  #endif

  #if CORE_IS_XY || CORE_IS_YZ
    /**
     * Head direction in -Y axis for CoreXY / CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Y or Z)
     */
    #if ANY(COREYX, COREYZ)
      #define Y_CMP(A,B) ((A)==(B))
    #else
      #define Y_CMP(A,B) ((A)!=(B))
    #endif
    #define Y_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && Y_CMP(D_(1),D_(2))) )
  #elif ENABLED(MARKFORGED_YX)
    #define Y_MOVE_TEST (current_block->steps.a != current_block->steps.b)
  #else
    #define Y_MOVE_TEST !!current_block->steps.b
  #endif

  #if CORE_IS_XZ || CORE_IS_YZ
    /**
     * Head direction in -Z axis for CoreXZ or CoreYZ bots.
     *
     * If steps differ, both axes are moving
     * If DeltaA ==  DeltaB, the movement is only in the 1st axis (X or Y, already handled above)
     * If DeltaA == -DeltaB, the movement is only in the 2nd axis (Z)
     */
    #if ANY(COREZX, COREZY)
      #define Z_CMP(A,B) ((A)==(B))
    #else
      #define Z_CMP(A,B) ((A)!=(B))
    #endif
    #define Z_MOVE_TEST ( S_(1) != S_(2) || (S_(1) > 0 && Z_CMP(D_(1),D_(2))) )
  #else
    #define Z_MOVE_TEST !!current_block->steps.c
  #endif

  // Set flags for all axes that move in this block
  // These are set per-axis, not per-stepper
  AxisBits didmove;
  NUM_AXIS_CODE(
    if (X_MOVE_TEST)              didmove.a = true, // Cartesian X or Kinematic A
    if (Y_MOVE_TEST)              didmove.b = true, // Cartesian Y or Kinematic B
    if (Z_MOVE_TEST)              didmove.c = true, // Cartesian Z or Kinematic C
    if (!!current_block->steps.i) didmove.i = true,
    if (!!current_block->steps.j) didmove.j = true,
    if (!!current_block->steps.k) didmove.k = true,
    if (!!current_block->steps.u) didmove.u = true,
    if (!!current_block->steps.v) didmove.v = true,
    if (!!current_block->steps.w) didmove.w = true
  );
  axis_did_move = didmove;
}

/**
 * This last phase of the stepper interrupt processes and properly
 * schedules planner blocks. This is executed after the step pulses
 * have been done, so it is less time critical.
 */
hal_timer_t Stepper::block_phase_isr() {
  #if DISABLED(OLD_ADAPTIVE_MULTISTEPPING)
    // If the ISR uses < 50% of MPU time, halve multi-stepping
    const hal_timer_t time_spent = HAL_timer_get_count(MF_TIMER_STEP);
    #if MULTISTEPPING_LIMIT > 1
      if (steps_per_isr > 1 && time_spent_out_isr >= time_spent_in_isr + time_spent) {
        steps_per_isr >>= 1;
        // ticks_nominal will need to be recalculated if we are in cruise phase
        ticks_nominal = 0;
      }
    #endif
    time_spent_in_isr = -time_spent;    // unsigned but guaranteed to be +ve when needed
    time_spent_out_isr = 0;
  #endif

  // If no queued movements, just wait 1ms for the next block
  hal_timer_t interval = (STEPPER_TIMER_RATE) / 1000UL;

  // If there is a current block
  if (current_block) {
    // If current block is finished, reset pointer and finalize state
    if (step_events_completed >= step_event_count) {
      #if ENABLED(DIRECT_STEPPING)
        // Direct stepping is currently not ready for HAS_I_AXIS
        #if STEPPER_PAGE_FORMAT == SP_4x4D_128
          #define PAGE_SEGMENT_UPDATE_POS(AXIS) \
            count_position[_AXIS(AXIS)] += page_step_state.bd[_AXIS(AXIS)] - 128 * 7;
        #elif STEPPER_PAGE_FORMAT == SP_4x1_512 || STEPPER_PAGE_FORMAT == SP_4x2_256
          #define PAGE_SEGMENT_UPDATE_POS(AXIS) \
            count_position[_AXIS(AXIS)] += page_step_state.bd[_AXIS(AXIS)] * count_direction[_AXIS(AXIS)];
        #endif

        if (current_block->is_page()) {
          PAGE_SEGMENT_UPDATE_POS(X);
          PAGE_SEGMENT_UPDATE_POS(Y);
          PAGE_SEGMENT_UPDATE_POS(Z);
          PAGE_SEGMENT_UPDATE_POS(E);
        }
      #endif
      TERN_(HAS_FILAMENT_RUNOUT_DISTANCE, runout.block_completed(current_block));
      discard_current_block();
    }
    else {
      // Step events not completed yet...

      // Are we in acceleration phase ?
      if (step_events_completed < accelerate_before) { // Calculate new timer value

        #if ENABLED(S_CURVE_ACCELERATION)
          // Get the next speed to use (Jerk limited!)
          uint32_t acc_step_rate = acceleration_time < current_block->acceleration_time
                                   ? _eval_bezier_curve(acceleration_time)
                                   : current_block->cruise_rate;
        #else
          acc_step_rate = STEP_MULTIPLY(acceleration_time, current_block->acceleration_rate) + current_block->initial_rate;
          NOMORE(acc_step_rate, current_block->nominal_rate);
        #endif

        // acc_step_rate is in steps/second

        // step_rate to timer interval and steps per stepper isr
        interval = calc_multistep_timer_interval(acc_step_rate << oversampling_factor);
        acceleration_time += interval;
        deceleration_time = 0; // Reset since we're doing acceleration first.

        #if ENABLED(NONLINEAR_EXTRUSION)
          calc_nonlinear_e(acc_step_rate << oversampling_factor);
        #endif

        #if ENABLED(LIN_ADVANCE)
          if (la_active) {
            const uint32_t la_step_rate = la_advance_steps < current_block->max_adv_steps ? current_block->la_advance_rate : 0;
            la_interval = calc_timer_interval((acc_step_rate + la_step_rate) >> current_block->la_scaling);
          }
        #endif

        /**
         * Adjust Laser Power - Accelerating
         *
         *  isPowered - True when a move is powered.
         *  isEnabled - laser power is active.
         *
         * Laser power variables are calulated and stored in this block by the planner code.
         *  trap_ramp_active_pwr - the active power in this block across accel or decel trap steps.
         *  trap_ramp_entry_incr - holds the precalculated value to increase the current power per accel step.
         */
        #if ENABLED(LASER_POWER_TRAP)
          if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {
            if (planner.laser_inline.status.isPowered && planner.laser_inline.status.isEnabled) {
              if (current_block->laser.trap_ramp_entry_incr > 0) {
                cutter.apply_power(current_block->laser.trap_ramp_active_pwr);
                current_block->laser.trap_ramp_active_pwr += current_block->laser.trap_ramp_entry_incr * steps_per_isr;
              }
            }
            // Not a powered move.
            else cutter.apply_power(0);
          }
        #endif
      }
      // Are we in Deceleration phase ?
      else if (step_events_completed >= decelerate_start) {
        uint32_t step_rate;

        #if ENABLED(S_CURVE_ACCELERATION)
          // If this is the 1st time we process the 2nd half of the trapezoid...
          if (!bezier_2nd_half) {
            // Initialize the Bézier speed curve
            _calc_bezier_curve_coeffs(current_block->cruise_rate, current_block->final_rate, current_block->deceleration_time_inverse);
            bezier_2nd_half = true;
          }
          // Calculate the next speed to use
          step_rate = deceleration_time < current_block->deceleration_time
            ? _eval_bezier_curve(deceleration_time)
            : current_block->final_rate;
        #else
          // Using the old trapezoidal control
          step_rate = STEP_MULTIPLY(deceleration_time, current_block->acceleration_rate);
          if (step_rate < acc_step_rate) {
            step_rate = acc_step_rate - step_rate;
            NOLESS(step_rate, current_block->final_rate);
          }
          else
            step_rate = current_block->final_rate;

        #endif

        // step_rate to timer interval and steps per stepper isr
        interval = calc_multistep_timer_interval(step_rate << oversampling_factor);
        deceleration_time += interval;

        #if ENABLED(NONLINEAR_EXTRUSION)
          calc_nonlinear_e(step_rate << oversampling_factor);
        #endif

        #if ENABLED(LIN_ADVANCE)
          if (la_active) {
            const uint32_t la_step_rate = la_advance_steps > current_block->final_adv_steps ? current_block->la_advance_rate : 0;
            if (la_step_rate != step_rate) {
              const bool forward_e = la_step_rate < step_rate;
              la_interval = calc_timer_interval((forward_e ? step_rate - la_step_rate : la_step_rate - step_rate) >> current_block->la_scaling);

              if (forward_e != motor_direction(E_AXIS)) {
                last_direction_bits.toggle(E_AXIS);
                count_direction.e = -count_direction.e;

                DIR_WAIT_BEFORE();

                E_APPLY_DIR(forward_e, false);

                TERN_(FTM_OPTIMIZE_DIR_STATES, last_set_direction = last_direction_bits);

                DIR_WAIT_AFTER();
              }
            }
            else
              la_interval = LA_ADV_NEVER;
          }
        #endif // LIN_ADVANCE

        // Adjust Laser Power - Decelerating
        #if ENABLED(LASER_POWER_TRAP)
          if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {
            if (planner.laser_inline.status.isPowered && planner.laser_inline.status.isEnabled) {
              if (current_block->laser.trap_ramp_exit_decr > 0) {
                current_block->laser.trap_ramp_active_pwr -= current_block->laser.trap_ramp_exit_decr * steps_per_isr;
                cutter.apply_power(current_block->laser.trap_ramp_active_pwr);
              }
              // Not a powered move.
              else cutter.apply_power(0);
            }
          }
        #endif

      }
      else {  // Must be in cruise phase otherwise

        // Calculate the ticks_nominal for this nominal speed, if not done yet
        if (ticks_nominal == 0) {
          // step_rate to timer interval and loops for the nominal speed
          ticks_nominal = calc_multistep_timer_interval(current_block->nominal_rate << oversampling_factor);
          // Prepare for deceleration
          IF_DISABLED(S_CURVE_ACCELERATION, acc_step_rate = current_block->nominal_rate);
          deceleration_time = ticks_nominal / 2;

          #if ENABLED(NONLINEAR_EXTRUSION)
            calc_nonlinear_e(current_block->nominal_rate << oversampling_factor);
          #endif

          #if ENABLED(LIN_ADVANCE)
            if (la_active)
              la_interval = calc_timer_interval(current_block->nominal_rate >> current_block->la_scaling);
          #endif

          // Adjust Laser Power - Cruise
          #if ENABLED(LASER_POWER_TRAP)
            if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {
              if (planner.laser_inline.status.isPowered && planner.laser_inline.status.isEnabled) {
                if (current_block->laser.trap_ramp_entry_incr > 0) {
                  current_block->laser.trap_ramp_active_pwr = current_block->laser.power;
                  cutter.apply_power(current_block->laser.power);
                }
              }
              // Not a powered move.
              else cutter.apply_power(0);
            }
          #endif
        }

        // The timer interval is just the nominal value for the nominal speed
        interval = ticks_nominal;
      }
    }

    #if ENABLED(LASER_FEATURE)
      /**
       * CUTTER_MODE_DYNAMIC is experimental and developing.
       * Super-fast method to dynamically adjust the laser power OCR value based on the input feedrate in mm-per-minute.
       * TODO: Set up Min/Max OCR offsets to allow tuning and scaling of various lasers.
       * TODO: Integrate accel/decel +-rate into the dynamic laser power calc.
       */
      if (cutter.cutter_mode == CUTTER_MODE_DYNAMIC
        && planner.laser_inline.status.isPowered                  // isPowered flag set on any parsed G1, G2, G3, or G5 move; cleared on any others.
        && current_block                                          // Block may not be available if steps completed (see discard_current_block() above)
        && cutter.last_block_power != current_block->laser.power  // Prevent constant update without change
      ) {
        cutter.apply_power(current_block->laser.power);
        cutter.last_block_power = current_block->laser.power;
      }
    #endif
  }
  else { // !current_block
    #if ENABLED(LASER_FEATURE)
      if (cutter.cutter_mode == CUTTER_MODE_DYNAMIC)
        cutter.apply_power(0);  // No movement in dynamic mode so turn Laser off
    #endif
  }

  // If there is no current block at this point, attempt to pop one from the buffer
  // and prepare its movement
  if (!current_block) {

    // Anything in the buffer?
    if ((current_block = planner.get_current_block())) {

      // Run through all sync blocks
      while (current_block->is_sync()) {

        // Set laser power
        #if ENABLED(LASER_POWER_SYNC)
          if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {
            if (current_block->is_sync_pwr()) {
              planner.laser_inline.status.isSyncPower = true;
              cutter.apply_power(current_block->laser.power);
            }
          }
        #endif

        // Set "fan speeds" for a laser module
        #if ENABLED(LASER_SYNCHRONOUS_M106_M107)
          if (current_block->is_sync_fan()) planner.sync_fan_speeds(current_block->fan_speed);
        #endif

        // Set position
        if (current_block->is_sync_pos()) _set_position(current_block->position);

        // Done with this block
        discard_current_block();

        // Try to get a new block. Exit if there are no more.
        if (!(current_block = planner.get_current_block()))
          return interval; // No more queued movements!
      }

      // For non-inline cutter, grossly apply power
      #if HAS_CUTTER
        if (cutter.cutter_mode == CUTTER_MODE_STANDARD) {
          cutter.apply_power(current_block->cutter_power);
        }
      #endif

      #if ENABLED(POWER_LOSS_RECOVERY)
        recovery.info.sdpos = current_block->sdpos;
        recovery.info.current_position = current_block->start_position;
      #endif

      #if ENABLED(DIRECT_STEPPING)
        if (current_block->is_page()) {
          page_step_state.segment_steps = 0;
          page_step_state.segment_idx = 0;
          page_step_state.page = page_manager.get_page(current_block->page_idx);
          page_step_state.bd.reset();

          if (DirectStepping::Config::DIRECTIONAL)
            current_block->direction_bits = last_direction_bits;

          if (!page_step_state.page) {
            discard_current_block();
            return interval;
          }
        }
      #endif

      // Set flags for all moving axes, accounting for kinematics
      set_axis_moved_for_current_block();

      #if ENABLED(ADAPTIVE_STEP_SMOOTHING)
        oversampling_factor = 0;

        // Decide if axis smoothing is possible
        if (stepper.adaptive_step_smoothing_enabled) {
          uint32_t max_rate = current_block->nominal_rate;  // Get the step event rate
          while (max_rate < min_step_isr_frequency) {       // As long as more ISRs are possible...
            max_rate <<= 1;                                 // Try to double the rate
            if (max_rate < min_step_isr_frequency)          // Don't exceed the estimated ISR limit
              ++oversampling_factor;                        // Increase the oversampling (used for left-shift)
          }
        }
      #endif

      // Based on the oversampling factor, do the calculations
      step_event_count = current_block->step_event_count << oversampling_factor;

      // Initialize Bresenham delta errors to 1/2
      delta_error = TERN_(LIN_ADVANCE, la_delta_error =) -int32_t(step_event_count);

      // Calculate Bresenham dividends and divisors
      advance_dividend = (current_block->steps << 1).asLong();
      advance_divisor = step_event_count << 1;

      #if ENABLED(INPUT_SHAPING_X)
        if (shaping_x.enabled) {
          const int64_t steps = current_block->direction_bits.x ? int64_t(current_block->steps.x) : -int64_t(current_block->steps.x);
          shaping_x.last_block_end_pos += steps;

          // If there are any remaining echos unprocessed, then direction change must
          // be delayed and processed in PULSE_PREP_SHAPING. This will cause half a step
          // to be missed, which will need recovering and this can be done through shaping_x.remainder.
          shaping_x.forward = current_block->direction_bits.x;
          if (!ShapingQueue::empty_x()) current_block->direction_bits.x = last_direction_bits.x;
        }
      #endif

      // Y and Z follow the same logic as X (but the comments aren't repeated)
      #if ENABLED(INPUT_SHAPING_Y)
        if (shaping_y.enabled) {
          const int64_t steps = current_block->direction_bits.y ? int64_t(current_block->steps.y) : -int64_t(current_block->steps.y);
          shaping_y.last_block_end_pos += steps;
          shaping_y.forward = current_block->direction_bits.y;
          if (!ShapingQueue::empty_y()) current_block->direction_bits.y = last_direction_bits.y;
        }
      #endif

      #if ENABLED(INPUT_SHAPING_Z)
        if (shaping_z.enabled) {
          const int64_t steps = current_block->direction_bits.z ? int64_t(current_block->steps.z) : -int64_t(current_block->steps.z);
          shaping_z.last_block_end_pos += steps;
          shaping_z.forward = current_block->direction_bits.z;
          if (!ShapingQueue::empty_z()) current_block->direction_bits.z = last_direction_bits.z;
        }
      #endif

      // No step events completed so far
      step_events_completed = 0;

      // Compute the acceleration and deceleration points
      accelerate_before = current_block->accelerate_before << oversampling_factor;
      decelerate_start = current_block->decelerate_start << oversampling_factor;

      TERN_(MIXING_EXTRUDER, mixer.stepper_setup(current_block->b_color));

      E_TERN_(stepper_extruder = current_block->extruder);

      // Initialize the trapezoid generator from the current block.
      #if ENABLED(LIN_ADVANCE)
        la_active = (current_block->la_advance_rate != 0);
        #if DISABLED(MIXING_EXTRUDER) && E_STEPPERS > 1
          // If the now active extruder wasn't in use during the last move, its pressure is most likely gone.
          if (stepper_extruder != last_moved_extruder) la_advance_steps = 0;
        #endif
        if (la_active) {
          // Apply LA scaling and discount the effect of frequency scaling
          la_dividend = (advance_dividend.e << current_block->la_scaling) << oversampling_factor;
        }
      #endif

      if ( ENABLED(DUAL_X_CARRIAGE) // TODO: Find out why this fixes "jittery" small circles
        || current_block->direction_bits != last_direction_bits
        || TERN(MIXING_EXTRUDER, false, stepper_extruder != last_moved_extruder)
      ) {
        E_TERN_(last_moved_extruder = stepper_extruder);
        set_directions(current_block->direction_bits);
      }

      #if ENABLED(LASER_FEATURE)
        if (cutter.cutter_mode == CUTTER_MODE_CONTINUOUS) {           // Planner controls the laser
          if (planner.laser_inline.status.isSyncPower)
            // If the previous block was a M3 sync power then skip the trap power init otherwise it will 0 the sync power.
            planner.laser_inline.status.isSyncPower = false;          // Clear the flag to process subsequent trap calc's.
          else if (current_block->laser.status.isEnabled) {
            #if ENABLED(LASER_POWER_TRAP)
              TERN_(DEBUG_LASER_TRAP, SERIAL_ECHO_MSG("InitTrapPwr:",current_block->laser.trap_ramp_active_pwr));
              cutter.apply_power(current_block->laser.status.isPowered ? current_block->laser.trap_ramp_active_pwr : 0);
            #else
              TERN_(DEBUG_CUTTER_POWER, SERIAL_ECHO_MSG("InlinePwr:",current_block->laser.power));
              cutter.apply_power(current_block->laser.status.isPowered ? current_block->laser.power : 0);
            #endif
          }
        }
      #endif // LASER_FEATURE

      // If the endstop is already pressed, endstop interrupts won't invoke
      // endstop_triggered and the move will grind. So check here for a
      // triggered endstop, which marks the block for discard on the next ISR.
      endstops.update();

      #if ENABLED(Z_LATE_ENABLE)
        // If delayed Z enable, enable it now. This option will severely interfere with
        // timing between pulses when chaining motion between blocks, and it could lead
        // to lost steps in both X and Y axis, so avoid using it unless strictly necessary!!
        if (current_block->steps.z) enable_axis(Z_AXIS);
      #endif

      // Mark ticks_nominal as not-yet-calculated
      ticks_nominal = 0;

      #if ENABLED(S_CURVE_ACCELERATION)
        // Initialize the Bézier speed curve
        _calc_bezier_curve_coeffs(current_block->initial_rate, current_block->cruise_rate, current_block->acceleration_time_inverse);
        // We haven't started the 2nd half of the trapezoid
        bezier_2nd_half = false;
      #else
        // Set as deceleration point the initial rate of the block
        acc_step_rate = current_block->initial_rate;
      #endif

      #if ENABLED(NONLINEAR_EXTRUSION)
        ne_edividend = advance_dividend.e;
        const float scale = (float(ne_edividend) / advance_divisor) * planner.mm_per_step[E_AXIS_N(current_block->extruder)];
        ne_scale = (1L << 24) * scale;
        if (current_block->direction_bits.e && ANY_AXIS_MOVES(current_block)) {
          ne_fix.A = (1L << 24) * ne.A;
          ne_fix.B = (1L << 24) * ne.B;
          ne_fix.C = (1L << 24) * ne.C;
        }
        else {
          ne_fix.A = ne_fix.B = 0;
          ne_fix.C = (1L << 24);
        }
      #endif

      // Calculate the initial timer interval
      interval = calc_multistep_timer_interval(current_block->initial_rate << oversampling_factor);
      // Initialize ac/deceleration time as if half the time passed.
      acceleration_time = deceleration_time = interval / 2;

      #if ENABLED(NONLINEAR_EXTRUSION)
        calc_nonlinear_e(current_block->initial_rate << oversampling_factor);
      #endif

      #if ENABLED(LIN_ADVANCE)
        if (la_active) {
          const uint32_t la_step_rate = la_advance_steps < current_block->max_adv_steps ? current_block->la_advance_rate : 0;
          la_interval = calc_timer_interval((current_block->initial_rate + la_step_rate) >> current_block->la_scaling);
        }
      #endif
    }
  } // !current_block

  // Return the interval to wait
  return interval;
}

#if ENABLED(LIN_ADVANCE)

  // Timer interrupt for E. LA_steps is set in the main routine
  void Stepper::advance_isr() {
    // Apply Bresenham algorithm so that linear advance can piggy back on
    // the acceleration and speed values calculated in block_phase_isr().
    // This helps keep LA in sync with, for example, S_CURVE_ACCELERATION.
    la_delta_error += la_dividend;
    const bool e_step_needed = la_delta_error >= 0;
    if (e_step_needed) {
      count_position.e += count_direction.e;
      la_advance_steps += count_direction.e;
      la_delta_error -= advance_divisor;

      // Set the STEP pulse ON
      E_STEP_WRITE(TERN(MIXING_EXTRUDER, mixer.get_next_stepper(), stepper_extruder), STEP_STATE_E);
    }

    TERN_(I2S_STEPPER_STREAM, i2s_push_sample());

    if (e_step_needed) {
      // Enforce a minimum duration for STEP pulse ON
      #if ISR_PULSE_CONTROL
        USING_TIMED_PULSE();
        START_TIMED_PULSE();
        AWAIT_HIGH_PULSE();
      #endif

      // Set the STEP pulse OFF
      E_STEP_WRITE(TERN(MIXING_EXTRUDER, mixer.get_stepper(), stepper_extruder), !STEP_STATE_E);
    }
  }

#endif // LIN_ADVANCE

#if ENABLED(BABYSTEPPING)

  // Timer interrupt for baby-stepping
  hal_timer_t Stepper::babystepping_isr() {
    babystep.task();
    return babystep.has_steps() ? BABYSTEP_TICKS : BABYSTEP_NEVER;
  }

#endif

// Check if the given block is busy or not - Must not be called from ISR contexts
// The current_block could change in the middle of the read by an Stepper ISR, so
// we must explicitly prevent that!
bool Stepper::is_block_busy(const block_t * const block) {
  #ifdef __AVR__
    // A SW memory barrier, to ensure GCC does not overoptimize loops
    #define sw_barrier() asm volatile("": : :"memory");

    // Keep reading until 2 consecutive reads return the same value,
    // meaning there was no update in-between caused by an interrupt.
    // This works because stepper ISRs happen at a slower rate than
    // successive reads of a variable, so 2 consecutive reads with
    // the same value means no interrupt updated it.
    block_t *vold, *vnew = current_block;
    sw_barrier();
    do {
      vold = vnew;
      vnew = current_block;
      sw_barrier();
    } while (vold != vnew);
  #else
    block_t *vnew = current_block;
  #endif

  // Return if the block is busy or not
  return block == vnew;
}

void Stepper::init() {

  #if MB(ALLIGATOR)
    const float motor_current[] = MOTOR_CURRENT;
    unsigned int digipot_motor = 0;
    for (uint8_t i = 0; i < 3 + EXTRUDERS; ++i) {
      digipot_motor = 255 * (motor_current[i] / 2.5);
      dac084s085::setValue(i, digipot_motor);
    }
  #endif

  // Init Microstepping Pins
  TERN_(HAS_MICROSTEPS, microstep_init());

  // Init Dir Pins
  TERN_(HAS_X_DIR, X_DIR_INIT());
  TERN_(HAS_X2_DIR, X2_DIR_INIT());
  #if HAS_Y_DIR
    Y_DIR_INIT();
    #if ALL(HAS_Y2_STEPPER, HAS_Y2_DIR)
      Y2_DIR_INIT();
    #endif
  #endif
  #if HAS_Z_DIR
    Z_DIR_INIT();
    #if NUM_Z_STEPPERS >= 2 && HAS_Z2_DIR
      Z2_DIR_INIT();
    #endif
    #if NUM_Z_STEPPERS >= 3 && HAS_Z3_DIR
      Z3_DIR_INIT();
    #endif
    #if NUM_Z_STEPPERS >= 4 && HAS_Z4_DIR
      Z4_DIR_INIT();
    #endif
  #endif
  SECONDARY_AXIS_CODE(
    I_DIR_INIT(), J_DIR_INIT(), K_DIR_INIT(),
    U_DIR_INIT(), V_DIR_INIT(), W_DIR_INIT()
  );
  #if HAS_E0_DIR
    E0_DIR_INIT();
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT();
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT();
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT();
  #endif
  #if HAS_E4_DIR
    E4_DIR_INIT();
  #endif
  #if HAS_E5_DIR
    E5_DIR_INIT();
  #endif
  #if HAS_E6_DIR
    E6_DIR_INIT();
  #endif
  #if HAS_E7_DIR
    E7_DIR_INIT();
  #endif

  // Init Enable Pins - steppers default to disabled.
  #if HAS_X_ENABLE
    #ifndef X_ENABLE_INIT_STATE
      #define X_ENABLE_INIT_STATE !X_ENABLE_ON
    #endif
    X_ENABLE_INIT();
    if (X_ENABLE_INIT_STATE) X_ENABLE_WRITE(X_ENABLE_INIT_STATE);
    #if ALL(HAS_X2_STEPPER, HAS_X2_ENABLE)
      X2_ENABLE_INIT();
      if (X_ENABLE_INIT_STATE) X2_ENABLE_WRITE(X_ENABLE_INIT_STATE);
    #endif
  #endif
  #if HAS_Y_ENABLE
    #ifndef Y_ENABLE_INIT_STATE
      #define Y_ENABLE_INIT_STATE !Y_ENABLE_ON
    #endif
    Y_ENABLE_INIT();
    if (Y_ENABLE_INIT_STATE) Y_ENABLE_WRITE(Y_ENABLE_INIT_STATE);
    #if ALL(HAS_Y2_STEPPER, HAS_Y2_ENABLE)
      Y2_ENABLE_INIT();
      if (Y_ENABLE_INIT_STATE) Y2_ENABLE_WRITE(Y_ENABLE_INIT_STATE);
    #endif
  #endif
  #if HAS_Z_ENABLE
    #ifndef Z_ENABLE_INIT_STATE
      #define Z_ENABLE_INIT_STATE !Z_ENABLE_ON
    #endif
    Z_ENABLE_INIT();
    if (Z_ENABLE_INIT_STATE) Z_ENABLE_WRITE(Z_ENABLE_INIT_STATE);
    #if NUM_Z_STEPPERS >= 2 && HAS_Z2_ENABLE
      Z2_ENABLE_INIT();
      if (Z_ENABLE_INIT_STATE) Z2_ENABLE_WRITE(Z_ENABLE_INIT_STATE);
    #endif
    #if NUM_Z_STEPPERS >= 3 && HAS_Z3_ENABLE
      Z3_ENABLE_INIT();
      if (Z_ENABLE_INIT_STATE) Z3_ENABLE_WRITE(Z_ENABLE_INIT_STATE);
    #endif
    #if NUM_Z_STEPPERS >= 4 && HAS_Z4_ENABLE
      Z4_ENABLE_INIT();
      if (Z_ENABLE_INIT_STATE) Z4_ENABLE_WRITE(Z_ENABLE_INIT_STATE);
    #endif
  #endif
  #if HAS_I_ENABLE
    I_ENABLE_INIT();
    if (!I_ENABLE_ON) I_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_J_ENABLE
    J_ENABLE_INIT();
    if (!J_ENABLE_ON) J_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_K_ENABLE
    K_ENABLE_INIT();
    if (!K_ENABLE_ON) K_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_U_ENABLE
    U_ENABLE_INIT();
    if (!U_ENABLE_ON) U_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_V_ENABLE
    V_ENABLE_INIT();
    if (!V_ENABLE_ON) V_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_W_ENABLE
    W_ENABLE_INIT();
    if (!W_ENABLE_ON) W_ENABLE_WRITE(HIGH);
  #endif
  #if HAS_E0_ENABLE
    #ifndef E_ENABLE_INIT_STATE
      #define E_ENABLE_INIT_STATE !E_ENABLE_ON
    #endif
    #ifndef E0_ENABLE_INIT_STATE
      #define E0_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E0_ENABLE_INIT();
    if (E0_ENABLE_INIT_STATE) E0_ENABLE_WRITE(E0_ENABLE_INIT_STATE);
  #endif
  #if HAS_E1_ENABLE
    #ifndef E1_ENABLE_INIT_STATE
      #define E1_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E1_ENABLE_INIT();
    if (E1_ENABLE_INIT_STATE) E1_ENABLE_WRITE(E1_ENABLE_INIT_STATE);
  #endif
  #if HAS_E2_ENABLE
    #ifndef E2_ENABLE_INIT_STATE
      #define E2_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E2_ENABLE_INIT();
    if (E2_ENABLE_INIT_STATE) E2_ENABLE_WRITE(E2_ENABLE_INIT_STATE);
  #endif
  #if HAS_E3_ENABLE
    #ifndef E3_ENABLE_INIT_STATE
      #define E3_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E3_ENABLE_INIT();
    if (E3_ENABLE_INIT_STATE) E3_ENABLE_WRITE(E3_ENABLE_INIT_STATE);
  #endif
  #if HAS_E4_ENABLE
    #ifndef E4_ENABLE_INIT_STATE
      #define E4_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E4_ENABLE_INIT();
    if (E4_ENABLE_INIT_STATE) E4_ENABLE_WRITE(E4_ENABLE_INIT_STATE);
  #endif
  #if HAS_E5_ENABLE
    #ifndef E5_ENABLE_INIT_STATE
      #define E5_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E5_ENABLE_INIT();
    if (E5_ENABLE_INIT_STATE) E5_ENABLE_WRITE(E5_ENABLE_INIT_STATE);
  #endif
  #if HAS_E6_ENABLE
    #ifndef E6_ENABLE_INIT_STATE
      #define E6_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E6_ENABLE_INIT();
    if (E6_ENABLE_INIT_STATE) E6_ENABLE_WRITE(E6_ENABLE_INIT_STATE);
  #endif
  #if HAS_E7_ENABLE
    #ifndef E7_ENABLE_INIT_STATE
      #define E7_ENABLE_INIT_STATE E_ENABLE_INIT_STATE
    #endif
    E7_ENABLE_INIT();
    if (E7_ENABLE_INIT_STATE) E7_ENABLE_WRITE(E7_ENABLE_INIT_STATE);
  #endif

  #define _STEP_INIT(AXIS) AXIS ##_STEP_INIT()
  #define _WRITE_STEP(AXIS, HIGHLOW) AXIS ##_STEP_WRITE(HIGHLOW)
  #define _DISABLE_AXIS(AXIS) DISABLE_AXIS_## AXIS()

  #define AXIS_INIT(AXIS, PIN) \
    _STEP_INIT(AXIS); \
    _WRITE_STEP(AXIS, !_STEP_STATE(PIN)); \
    _DISABLE_AXIS(AXIS)

  #define E_AXIS_INIT(NUM) AXIS_INIT(E## NUM, E)

  // Init Step Pins
  #if HAS_X_STEP
    #if HAS_X2_STEPPER
      X2_STEP_INIT();
      X2_STEP_WRITE(!STEP_STATE_X);
    #endif
    AXIS_INIT(X, X);
  #endif

  #if HAS_Y_STEP
    #if HAS_Y2_STEPPER
      Y2_STEP_INIT();
      Y2_STEP_WRITE(!STEP_STATE_Y);
    #endif
    AXIS_INIT(Y, Y);
  #endif

  #if HAS_Z_STEP
    #if NUM_Z_STEPPERS >= 2
      Z2_STEP_INIT();
      Z2_STEP_WRITE(!STEP_STATE_Z);
    #endif
    #if NUM_Z_STEPPERS >= 3
      Z3_STEP_INIT();
      Z3_STEP_WRITE(!STEP_STATE_Z);
    #endif
    #if NUM_Z_STEPPERS >= 4
      Z4_STEP_INIT();
      Z4_STEP_WRITE(!STEP_STATE_Z);
    #endif
    AXIS_INIT(Z, Z);
  #endif
  #if HAS_I_STEP
    AXIS_INIT(I, I);
  #endif
  #if HAS_J_STEP
    AXIS_INIT(J, J);
  #endif
  #if HAS_K_STEP
    AXIS_INIT(K, K);
  #endif
  #if HAS_U_STEP
    AXIS_INIT(U, U);
  #endif
  #if HAS_V_STEP
    AXIS_INIT(V, V);
  #endif
  #if HAS_W_STEP
    AXIS_INIT(W, W);
  #endif

  #if E_STEPPERS && HAS_E0_STEP
    E_AXIS_INIT(0);
  #endif
  #if (E_STEPPERS > 1 || ENABLED(E_DUAL_STEPPER_DRIVERS)) && HAS_E1_STEP
    E_AXIS_INIT(1);
  #endif
  #if E_STEPPERS > 2 && HAS_E2_STEP
    E_AXIS_INIT(2);
  #endif
  #if E_STEPPERS > 3 && HAS_E3_STEP
    E_AXIS_INIT(3);
  #endif
  #if E_STEPPERS > 4 && HAS_E4_STEP
    E_AXIS_INIT(4);
  #endif
  #if E_STEPPERS > 5 && HAS_E5_STEP
    E_AXIS_INIT(5);
  #endif
  #if E_STEPPERS > 6 && HAS_E6_STEP
    E_AXIS_INIT(6);
  #endif
  #if E_STEPPERS > 7 && HAS_E7_STEP
    E_AXIS_INIT(7);
  #endif

  #if DISABLED(I2S_STEPPER_STREAM)
    HAL_timer_start(MF_TIMER_STEP, 122); // Init Stepper ISR to 122 Hz for quick starting
    wake_up();
    sei();
  #endif

  // Init direction states
  apply_directions();

  #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM
    initialized = true;
    digipot_init();
  #endif
}

#if HAS_ZV_SHAPING

  /**
   * Calculate a fixed point factor to apply to the signal and its echo
   * when shaping an axis.
   */
  void Stepper::set_shaping_damping_ratio(const AxisEnum axis, const_float_t zeta) {
    // From the damping ratio, get a factor that can be applied to advance_dividend for fixed-point maths.
    // For ZV, we use amplitudes 1/(1+K) and K/(1+K) where K = exp(-zeta * π / sqrt(1.0f - zeta * zeta))
    // which can be converted to 1:7 fixed point with an excellent fit with a 3rd-order polynomial.
    float factor2;
    if (zeta <= 0.0f) factor2 = 64.0f;
    else if (zeta >= 1.0f) factor2 = 0.0f;
    else {
      factor2 = 64.44056192 + -99.02008832 * zeta;
      const float zeta2 = sq(zeta);
      factor2 += -7.58095488 * zeta2;
      const float zeta3 = zeta2 * zeta;
      factor2 += 43.073216 * zeta3;
      factor2 = FLOOR(factor2);
    }

    const bool was_on = hal.isr_state();
    hal.isr_off();
    TERN_(INPUT_SHAPING_X, if (axis == X_AXIS) { shaping_x.factor2 = factor2; shaping_x.factor1 = 128 - factor2; shaping_x.zeta = zeta; })
    TERN_(INPUT_SHAPING_Y, if (axis == Y_AXIS) { shaping_y.factor2 = factor2; shaping_y.factor1 = 128 - factor2; shaping_y.zeta = zeta; })
    TERN_(INPUT_SHAPING_Z, if (axis == Z_AXIS) { shaping_z.factor2 = factor2; shaping_z.factor1 = 128 - factor2; shaping_z.zeta = zeta; })
    if (was_on) hal.isr_on();
  }

  float Stepper::get_shaping_damping_ratio(const AxisEnum axis) {
    TERN_(INPUT_SHAPING_X, if (axis == X_AXIS) return shaping_x.zeta);
    TERN_(INPUT_SHAPING_Y, if (axis == Y_AXIS) return shaping_y.zeta);
    TERN_(INPUT_SHAPING_Z, if (axis == Z_AXIS) return shaping_z.zeta);
    return -1;
  }

  void Stepper::set_shaping_frequency(const AxisEnum axis, const_float_t freq) {
    // enabling or disabling shaping whilst moving can result in lost steps
    planner.synchronize();

    const bool was_on = hal.isr_state();
    hal.isr_off();

    const shaping_time_t delay = freq ? float(uint32_t(STEPPER_TIMER_RATE) / 2) / freq : shaping_time_t(-1);
    #define SHAPING_SET_FREQ_FOR_AXIS(AXISN, AXISL)                                 \
      if (axis == AXISN) {                                                          \
        ShapingQueue::set_delay(AXISN, delay);                                      \
        shaping_##AXISL.frequency = freq;                                           \
        shaping_##AXISL.enabled = !!freq;                                           \
        shaping_##AXISL.delta_error = 0;                                            \
        shaping_##AXISL.last_block_end_pos = count_position.AXISL;                  \
      }

    TERN_(INPUT_SHAPING_X, SHAPING_SET_FREQ_FOR_AXIS(X_AXIS, x))
    TERN_(INPUT_SHAPING_Y, SHAPING_SET_FREQ_FOR_AXIS(Y_AXIS, y))
    TERN_(INPUT_SHAPING_Z, SHAPING_SET_FREQ_FOR_AXIS(Z_AXIS, z))

    if (was_on) hal.isr_on();
  }

  float Stepper::get_shaping_frequency(const AxisEnum axis) {
    TERN_(INPUT_SHAPING_X, if (axis == X_AXIS) return shaping_x.frequency);
    TERN_(INPUT_SHAPING_Y, if (axis == Y_AXIS) return shaping_y.frequency);
    TERN_(INPUT_SHAPING_Z, if (axis == Z_AXIS) return shaping_z.frequency);
    return -1;
  }

#endif // HAS_ZV_SHAPING

/**
 * Set the stepper positions directly in steps
 *
 * The input is based on the typical per-axis XYZE steps.
 * For CORE machines XYZ needs to be translated to ABC.
 *
 * This allows get_axis_position_mm to correctly
 * derive the current XYZE position later on.
 */
void Stepper::_set_position(const abce_long_t &spos) {
  #if ENABLED(INPUT_SHAPING_X)
    const int32_t x_shaping_delta = count_position.x - shaping_x.last_block_end_pos;
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    const int32_t y_shaping_delta = count_position.y - shaping_y.last_block_end_pos;
  #endif
  #if ENABLED(INPUT_SHAPING_Z)
    const int32_t z_shaping_delta = count_position.z - shaping_z.last_block_end_pos;
  #endif

  #if ANY(IS_CORE, MARKFORGED_XY, MARKFORGED_YX)
    // Core equations follow the form of the dA and dB equations at https://www.corexy.com/theory.html
    #if CORE_IS_XY
      count_position.set(spos.a + spos.b, CORESIGN(spos.a - spos.b) OPTARG(HAS_Z_AXIS, spos.c));
    #elif CORE_IS_XZ
      count_position.set(spos.a + spos.c, spos.b, CORESIGN(spos.a - spos.c));
    #elif CORE_IS_YZ
      count_position.set(spos.a, spos.b + spos.c, CORESIGN(spos.b - spos.c));
    #elif ENABLED(MARKFORGED_XY)
      count_position.set(spos.a TERN(MARKFORGED_INVERSE, +, -) spos.b, spos.b, spos.c);
    #elif ENABLED(MARKFORGED_YX)
      count_position.set(spos.a, spos.b TERN(MARKFORGED_INVERSE, +, -) spos.a, spos.c);
    #endif
    SECONDARY_AXIS_CODE(
      count_position.i = spos.i,
      count_position.j = spos.j,
      count_position.k = spos.k,
      count_position.u = spos.u,
      count_position.v = spos.v,
      count_position.w = spos.w
    );
    TERN_(HAS_EXTRUDERS, count_position.e = spos.e);
  #else
    // default non-h-bot planning
    count_position = spos;
  #endif

  #if ENABLED(INPUT_SHAPING_X)
    if (shaping_x.enabled) {
      count_position.x += x_shaping_delta;
      shaping_x.last_block_end_pos = spos.x;
    }
  #endif
  #if ENABLED(INPUT_SHAPING_Y)
    if (shaping_y.enabled) {
      count_position.y += y_shaping_delta;
      shaping_y.last_block_end_pos = spos.y;
    }
  #endif
  #if ENABLED(INPUT_SHAPING_Z)
    if (shaping_z.enabled) {
      count_position.z += z_shaping_delta;
      shaping_z.last_block_end_pos = spos.z;
    }
  #endif
}

// AVR requires guards to ensure any atomic memory operation greater than 8 bits
#define ATOMIC_SECTION_START() const bool was_enabled = suspend()
#define ATOMIC_SECTION_END() if (was_enabled) wake_up()
#define AVR_ATOMIC_SECTION_START() TERN_(__AVR__, ATOMIC_SECTION_START())
#define AVR_ATOMIC_SECTION_END() TERN_(__AVR__, ATOMIC_SECTION_END())

/**
 * Get a stepper's position in steps.
 */
int32_t Stepper::position(const AxisEnum axis) {
  AVR_ATOMIC_SECTION_START();
  const int32_t v = count_position[axis];
  AVR_ATOMIC_SECTION_END();
  return v;
}

/**
 * Set all axis stepper positions in steps
 */
void Stepper::set_position(const xyze_long_t &spos) {
  planner.synchronize();
  ATOMIC_SECTION_START();
  _set_position(spos);
  ATOMIC_SECTION_END();
}

/**
 * Set a single axis stepper position in steps
 */
void Stepper::set_axis_position(const AxisEnum a, const int32_t &v) {
  planner.synchronize();

  #if ANY(__AVR__, INPUT_SHAPING_X, INPUT_SHAPING_Y, INPUT_SHAPING_Z)
    ATOMIC_SECTION_START();
  #endif

  count_position[a] = v;
  TERN_(INPUT_SHAPING_X, if (a == X_AXIS) shaping_x.last_block_end_pos = v);
  TERN_(INPUT_SHAPING_Y, if (a == Y_AXIS) shaping_y.last_block_end_pos = v);
  TERN_(INPUT_SHAPING_Z, if (a == Z_AXIS) shaping_z.last_block_end_pos = v);

  #if ANY(__AVR__, INPUT_SHAPING_X, INPUT_SHAPING_Y, INPUT_SHAPING_Z)
    ATOMIC_SECTION_END();
  #endif
}

#if HAS_EXTRUDERS

  void Stepper::set_e_position(const int32_t &v) {
    planner.synchronize();

    AVR_ATOMIC_SECTION_START();
    count_position.e = v;
    AVR_ATOMIC_SECTION_END();
  }

#endif // HAS_EXTRUDERS

#if ENABLED(FT_MOTION)

  void Stepper::ftMotion_syncPosition() {
    planner.synchronize();

    // Update stepper positions from the planner
    AVR_ATOMIC_SECTION_START();
    count_position = planner.position;
    AVR_ATOMIC_SECTION_END();
  }

#endif // FT_MOTION

/**
 * Record stepper positions and discard the rest of the current block
 *
 * WARNING! This function may be called from ISR context!
 * If the Stepper ISR is preempted (e.g., by the endstop ISR) we
 * must ensure the move is properly canceled before the ISR resumes.
 */
void Stepper::endstop_triggered(const AxisEnum axis) {

  ATOMIC_SECTION_START();   // Suspend the Stepper ISR on all platforms

  endstops_trigsteps[axis] = (
    #if IS_CORE
      (axis == CORE_AXIS_2
        ? CORESIGN(count_position[CORE_AXIS_1] - count_position[CORE_AXIS_2])
        : count_position[CORE_AXIS_1] + count_position[CORE_AXIS_2]
      ) * double(0.5)
    #elif ENABLED(MARKFORGED_XY)
      axis == CORE_AXIS_1
        ? count_position[CORE_AXIS_1] TERN(MARKFORGED_INVERSE, +, -) count_position[CORE_AXIS_2]
        : count_position[CORE_AXIS_2]
    #elif ENABLED(MARKFORGED_YX)
      axis == CORE_AXIS_1
        ? count_position[CORE_AXIS_1]
        : count_position[CORE_AXIS_2] TERN(MARKFORGED_INVERSE, +, -) count_position[CORE_AXIS_1]
    #else // !IS_CORE
      count_position[axis]
    #endif
  );

  // Discard the rest of the move if there is a current block
  quick_stop();

  ATOMIC_SECTION_END();     // Suspend the Stepper ISR on all platforms
}

// Return the "triggered" position for an axis (that hit an endstop)
int32_t Stepper::triggered_position(const AxisEnum axis) {
  AVR_ATOMIC_SECTION_START();
  const int32_t v = endstops_trigsteps[axis];
  AVR_ATOMIC_SECTION_END();
  return v;
}

#if ANY(CORE_IS_XY, CORE_IS_XZ, MARKFORGED_XY, MARKFORGED_YX, IS_SCARA, DELTA)
  #define SAYS_A 1
#endif
#if ANY(CORE_IS_XY, CORE_IS_YZ, MARKFORGED_XY, MARKFORGED_YX, IS_SCARA, DELTA, POLAR)
  #define SAYS_B 1
#endif
#if ANY(CORE_IS_XZ, CORE_IS_YZ, DELTA)
  #define SAYS_C 1
#endif

void Stepper::report_a_position(const xyz_long_t &pos) {
  #if NUM_AXES
    SERIAL_ECHOLNPGM_P(
      LIST_N(DOUBLE(NUM_AXES),
        TERN(SAYS_A, PSTR(STR_COUNT_A), PSTR(STR_COUNT_X)), pos.x,
        TERN(SAYS_B, PSTR("B:"), SP_Y_LBL), pos.y,
        TERN(SAYS_C, PSTR("C:"), SP_Z_LBL), pos.z,
        SP_I_LBL, pos.i, SP_J_LBL, pos.j, SP_K_LBL, pos.k,
        SP_U_LBL, pos.u, SP_V_LBL, pos.v, SP_W_LBL, pos.w
      )
    );
  #endif
}

void Stepper::report_positions() {
  AVR_ATOMIC_SECTION_START();
  const xyz_long_t pos = count_position;
  AVR_ATOMIC_SECTION_END();
  report_a_position(pos);
}

#if ENABLED(FT_MOTION)

  /**
   * Run stepping from the Stepper ISR at regular short intervals.
   *
   * - Set ftMotion.sts_stepperBusy state to reflect whether there are any commands in the circular buffer.
   * - If there are no commands in the buffer, return.
   * - Get the next command from the circular buffer ftMotion.stepperCmdBuff[].
   * - If the block is being aborted, return without processing the command.
   * - Apply STEP/DIR along with any delays required. A command may be empty, with no STEP/DIR.
   */
  void Stepper::ftMotion_stepper() {

    // Check if the buffer is empty.
    ftMotion.sts_stepperBusy = (ftMotion.stepperCmdBuff_produceIdx != ftMotion.stepperCmdBuff_consumeIdx);
    if (!ftMotion.sts_stepperBusy) return;

    // "Pop" one command from current motion buffer
    const ft_command_t command = ftMotion.stepperCmdBuff[ftMotion.stepperCmdBuff_consumeIdx];
    if (++ftMotion.stepperCmdBuff_consumeIdx == (FTM_STEPPERCMD_BUFF_SIZE))
      ftMotion.stepperCmdBuff_consumeIdx = 0;

    if (abort_current_block) return;

    USING_TIMED_PULSE();

    // Get FT Motion command flags for axis STEP / DIR
    #define _FTM_STEP(AXIS) TEST(command, FT_BIT_STEP_##AXIS)
    #define _FTM_DIR(AXIS) TEST(command, FT_BIT_DIR_##AXIS)

    /**
     * Update direction bits for steppers that were stepped by this command.
     * HX, HY, HZ direction bits were set for Core kinematics
     * when the block was fetched and are not overwritten here.
     */

    #define _FTM_SET_DIR(AXIS) if (_FTM_STEP(AXIS)) last_direction_bits.bset(_AXIS(AXIS), _FTM_DIR(AXIS));
    LOGICAL_AXIS_MAP(_FTM_SET_DIR);

    if (TERN1(FTM_OPTIMIZE_DIR_STATES, last_set_direction != last_direction_bits)) {
      // Apply directions (generally applying to the entire linear move)
      #define _FTM_APPLY_DIR(A) if (TERN1(FTM_OPTIMIZE_DIR_STATES, last_direction_bits.A != last_set_direction.A)) \
                                  SET_STEP_DIR(A);
      LOGICAL_AXIS_MAP(_FTM_APPLY_DIR);

      TERN_(FTM_OPTIMIZE_DIR_STATES, last_set_direction = last_direction_bits);

      // Any DIR change requires a wait period
      DIR_WAIT_AFTER();
    }

    // Start step pulses. Edge stepping will toggle the STEP pin.
    #define _FTM_STEP_START(A) A##_APPLY_STEP(_FTM_STEP(A), false);
    LOGICAL_AXIS_MAP(_FTM_STEP_START);

    // Apply steps via I2S
    TERN_(I2S_STEPPER_STREAM, i2s_push_sample());

    // Begin waiting for the minimum pulse duration
    START_TIMED_PULSE();

    // Update step counts
    #define _FTM_STEP_COUNT(A) if (_FTM_STEP(A)) count_position.A += last_direction_bits.A ? 1 : -1;
    LOGICAL_AXIS_MAP(_FTM_STEP_COUNT);

    // Provide EDGE flags for E stepper(s)
    #if HAS_EXTRUDERS
      #if ENABLED(E_DUAL_STEPPER_DRIVERS)
        constexpr bool e_axis_has_dedge = AXIS_HAS_DEDGE(E0) && AXIS_HAS_DEDGE(E1);
      #else
        #define _EDGE_BIT(N) | (AXIS_HAS_DEDGE(E##N) << TOOL_ESTEPPER(N))
        constexpr Flags<E_STEPPERS> e_stepper_dedge { 0 REPEAT(EXTRUDERS, _EDGE_BIT) };
        const bool e_axis_has_dedge = e_stepper_dedge[stepper_extruder];
      #endif
    #endif

    // Only wait for axes without edge stepping
    const bool any_wait = false LOGICAL_AXIS_GANG(
      || (!e_axis_has_dedge  && _FTM_STEP(E)),
      || (!AXIS_HAS_DEDGE(X) && _FTM_STEP(X)), || (!AXIS_HAS_DEDGE(Y) && _FTM_STEP(Y)), || (!AXIS_HAS_DEDGE(Z) && _FTM_STEP(Z)),
      || (!AXIS_HAS_DEDGE(I) && _FTM_STEP(I)), || (!AXIS_HAS_DEDGE(J) && _FTM_STEP(J)), || (!AXIS_HAS_DEDGE(K) && _FTM_STEP(K)),
      || (!AXIS_HAS_DEDGE(U) && _FTM_STEP(U)), || (!AXIS_HAS_DEDGE(V) && _FTM_STEP(V)), || (!AXIS_HAS_DEDGE(W) && _FTM_STEP(W))
    );

    // Allow pulses to be registered by stepper drivers
    if (any_wait) AWAIT_HIGH_PULSE();

    // Stop pulses. Axes with DEDGE will do nothing, assuming STEP_STATE_* is HIGH
    #define _FTM_STEP_STOP(AXIS) AXIS##_APPLY_STEP(!STEP_STATE_##AXIS, false);
    LOGICAL_AXIS_MAP(_FTM_STEP_STOP);

  } // Stepper::ftMotion_stepper

#endif // FT_MOTION

#if ENABLED(BABYSTEPPING)

  #define _ENABLE_AXIS(A) enable_axis(_AXIS(A))
  #define _READ_DIR(AXIS) AXIS ##_DIR_READ()
  #define _APPLY_DIR(AXIS, FWD) AXIS ##_APPLY_DIR(FWD, true)

  #if MINIMUM_STEPPER_PULSE_NS
    #define STEP_PULSE_CYCLES ((MINIMUM_STEPPER_PULSE_NS) * CYCLES_PER_MICROSECOND / 1000)
  #else
    #define STEP_PULSE_CYCLES 0
  #endif

  #if ENABLED(DELTA)
    #define CYCLES_EATEN_BABYSTEP (2 * 15)
  #else
    #define CYCLES_EATEN_BABYSTEP 0
  #endif

  #if CYCLES_EATEN_BABYSTEP < STEP_PULSE_CYCLES
    #define EXTRA_CYCLES_BABYSTEP (STEP_PULSE_CYCLES - (CYCLES_EATEN_BABYSTEP))
  #else
    #define EXTRA_CYCLES_BABYSTEP 0
  #endif

  #if EXTRA_CYCLES_BABYSTEP > 20
    #define _SAVE_START() const hal_timer_t pulse_start = HAL_timer_get_count(MF_TIMER_PULSE)
    #define _PULSE_WAIT() while (EXTRA_CYCLES_BABYSTEP > uint32_t(HAL_timer_get_count(MF_TIMER_PULSE) - pulse_start) * (PULSE_TIMER_PRESCALE)) { /* nada */ }
  #else
    #define _SAVE_START() NOOP
    #if EXTRA_CYCLES_BABYSTEP > 0
      #define _PULSE_WAIT() DELAY_CYCLES(EXTRA_CYCLES_BABYSTEP)
    #elif ENABLED(DELTA)
      #define _PULSE_WAIT() DELAY_US(2);
    #elif STEP_PULSE_CYCLES > 0
      #define _PULSE_WAIT() NOOP
    #else
      #define _PULSE_WAIT() DELAY_US(4);
    #endif
  #endif

  #if ENABLED(BABYSTEPPING_EXTRA_DIR_WAIT)
    #define EXTRA_DIR_WAIT_BEFORE DIR_WAIT_BEFORE
    #define EXTRA_DIR_WAIT_AFTER  DIR_WAIT_AFTER
  #else
    #define EXTRA_DIR_WAIT_BEFORE()
    #define EXTRA_DIR_WAIT_AFTER()
  #endif

  #if DISABLED(DELTA)

    #define BABYSTEP_AXIS(AXIS, FWD, INV) do{      \
      const bool old_fwd = _READ_DIR(AXIS);        \
      _ENABLE_AXIS(AXIS);                          \
      DIR_WAIT_BEFORE();                           \
      _APPLY_DIR(AXIS, (FWD)^(INV));               \
      DIR_WAIT_AFTER();                            \
      _SAVE_START();                               \
      _APPLY_STEP(AXIS, _STEP_STATE(AXIS), true);  \
      _PULSE_WAIT();                               \
      _APPLY_STEP(AXIS, !_STEP_STATE(AXIS), true); \
      EXTRA_DIR_WAIT_BEFORE();                     \
      _APPLY_DIR(AXIS, old_fwd);                   \
      EXTRA_DIR_WAIT_AFTER();                      \
    }while(0)

  #endif

  #if IS_CORE

    #define BABYSTEP_CORE(A, B, FWD, INV, ALT) do{              \
      const xy_byte_t old_fwd = { _READ_DIR(A), _READ_DIR(B) }; \
      _ENABLE_AXIS(A); _ENABLE_AXIS(B);                         \
      DIR_WAIT_BEFORE();                                        \
      _APPLY_DIR(A, (FWD)^(INV));                               \
      _APPLY_DIR(B, (FWD)^(INV)^(ALT));                         \
      DIR_WAIT_AFTER();                                         \
      _SAVE_START();                                            \
      _APPLY_STEP(A, _STEP_STATE(A), true);                     \
      _APPLY_STEP(B, _STEP_STATE(B), true);                     \
      _PULSE_WAIT();                                            \
      _APPLY_STEP(A, !_STEP_STATE(A), true);                    \
      _APPLY_STEP(B, !_STEP_STATE(B), true);                    \
      EXTRA_DIR_WAIT_BEFORE();                                  \
      _APPLY_DIR(A, old_fwd.a); _APPLY_DIR(B, old_fwd.b);       \
      EXTRA_DIR_WAIT_AFTER();                                   \
    }while(0)

  #endif

  // MUST ONLY BE CALLED BY AN ISR,
  // No other ISR should ever interrupt this!
  void Stepper::do_babystep(const AxisEnum axis, const bool direction) {

    IF_DISABLED(BABYSTEPPING, cli());

    switch (axis) {

      #if ENABLED(BABYSTEP_XY)

        case X_AXIS:
          #if CORE_IS_XY
            BABYSTEP_CORE(X, Y, direction, 0, 0);
          #elif CORE_IS_XZ
            BABYSTEP_CORE(X, Z, direction, 0, 0);
          #else
            BABYSTEP_AXIS(X, direction, 0);
          #endif
          break;

        case Y_AXIS:
          #if CORE_IS_XY
            BABYSTEP_CORE(X, Y, direction, 0, (CORESIGN(1)>0));
          #elif CORE_IS_YZ
            BABYSTEP_CORE(Y, Z, direction, 0, (CORESIGN(1)<0));
          #else
            BABYSTEP_AXIS(Y, direction, 0);
          #endif
          break;

      #endif

      case Z_AXIS: {

        #if CORE_IS_XZ
          BABYSTEP_CORE(X, Z, direction, ENABLED(BABYSTEP_INVERT_Z), (CORESIGN(1)>0));
        #elif CORE_IS_YZ
          BABYSTEP_CORE(Y, Z, direction, ENABLED(BABYSTEP_INVERT_Z), (CORESIGN(1)<0));
        #elif DISABLED(DELTA)
          BABYSTEP_AXIS(Z, direction, ENABLED(BABYSTEP_INVERT_Z));

        #else // DELTA

          const bool z_direction = TERN_(BABYSTEP_INVERT_Z, !) direction;

          enable_axis(A_AXIS); enable_axis(B_AXIS); enable_axis(C_AXIS);

          DIR_WAIT_BEFORE();

          const bool old_fwd[3] = { X_DIR_READ(), Y_DIR_READ(), Z_DIR_READ() };

          X_DIR_WRITE(z_direction);
          Y_DIR_WRITE(z_direction);
          Z_DIR_WRITE(z_direction);

          DIR_WAIT_AFTER();

          _SAVE_START();

          X_STEP_WRITE(STEP_STATE_X);
          Y_STEP_WRITE(STEP_STATE_Y);
          Z_STEP_WRITE(STEP_STATE_Z);

          _PULSE_WAIT();

          X_STEP_WRITE(!STEP_STATE_X);
          Y_STEP_WRITE(!STEP_STATE_Y);
          Z_STEP_WRITE(!STEP_STATE_Z);

          // Restore direction bits
          EXTRA_DIR_WAIT_BEFORE();

          X_DIR_WRITE(old_fwd[A_AXIS]);
          Y_DIR_WRITE(old_fwd[B_AXIS]);
          Z_DIR_WRITE(old_fwd[C_AXIS]);

          EXTRA_DIR_WAIT_AFTER();

        #endif

      } break;

      default: break;
    }

    IF_DISABLED(BABYSTEPPING, sei());
  }

#endif // BABYSTEPPING

/**
 * Software-controlled Stepper Motor Current
 */

#if HAS_MOTOR_CURRENT_SPI

  // From Arduino DigitalPotControl example
  void Stepper::set_digipot_value_spi(const int16_t address, const int16_t value) {
    WRITE(DIGIPOTSS_PIN, LOW);  // Take the SS pin low to select the chip
    SPI.transfer(address);      // Send the address and value via SPI
    SPI.transfer(value);
    WRITE(DIGIPOTSS_PIN, HIGH); // Take the SS pin high to de-select the chip
    //delay(10);
  }

#endif // HAS_MOTOR_CURRENT_SPI

#if HAS_MOTOR_CURRENT_PWM

  void Stepper::refresh_motor_power() {
    if (!initialized) return;
    for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i) {
      switch (i) {
        #if ANY_PIN(MOTOR_CURRENT_PWM_XY, MOTOR_CURRENT_PWM_X, MOTOR_CURRENT_PWM_Y, MOTOR_CURRENT_PWM_I, MOTOR_CURRENT_PWM_J, MOTOR_CURRENT_PWM_K, MOTOR_CURRENT_PWM_U, MOTOR_CURRENT_PWM_V, MOTOR_CURRENT_PWM_W)
          case 0:
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          case 1:
        #endif
        #if HAS_MOTOR_CURRENT_PWM_E
          case 2:
        #endif
            set_digipot_current(i, motor_current_setting[i]);
        default: break;
      }
    }
  }

#endif // HAS_MOTOR_CURRENT_PWM

#if !MB(PRINTRBOARD_G2)

  #if HAS_MOTOR_CURRENT_SPI || HAS_MOTOR_CURRENT_PWM

    void Stepper::set_digipot_current(const uint8_t driver, const int16_t current) {
      if (WITHIN(driver, 0, MOTOR_CURRENT_COUNT - 1))
        motor_current_setting[driver] = current; // update motor_current_setting

      if (!initialized) return;

      #if HAS_MOTOR_CURRENT_SPI

        //SERIAL_ECHOLNPGM("Digipotss current ", current);

        const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
        set_digipot_value_spi(digipot_ch[driver], current);

      #elif HAS_MOTOR_CURRENT_PWM

        #define _WRITE_CURRENT_PWM(P) hal.set_pwm_duty(pin_t(MOTOR_CURRENT_PWM_## P ##_PIN), 255L * current / (MOTOR_CURRENT_PWM_RANGE))
        switch (driver) {
          case 0:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_X)
              _WRITE_CURRENT_PWM(X);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_Y)
              _WRITE_CURRENT_PWM(Y);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
              _WRITE_CURRENT_PWM(XY);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_I)
              _WRITE_CURRENT_PWM(I);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_J)
              _WRITE_CURRENT_PWM(J);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_K)
              _WRITE_CURRENT_PWM(K);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_U)
              _WRITE_CURRENT_PWM(U);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_V)
              _WRITE_CURRENT_PWM(V);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_W)
              _WRITE_CURRENT_PWM(W);
            #endif
            break;
          case 1:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
              _WRITE_CURRENT_PWM(Z);
            #endif
            break;
          case 2:
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
              _WRITE_CURRENT_PWM(E);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E0)
              _WRITE_CURRENT_PWM(E0);
            #endif
            #if PIN_EXISTS(MOTOR_CURRENT_PWM_E1)
              _WRITE_CURRENT_PWM(E1);
            #endif
            break;
        }
      #endif
    }

    void Stepper::digipot_init() {

      #if HAS_MOTOR_CURRENT_SPI

        SPI.begin();
        SET_OUTPUT(DIGIPOTSS_PIN);

        for (uint8_t i = 0; i < COUNT(motor_current_setting); ++i)
          set_digipot_current(i, motor_current_setting[i]);

      #elif HAS_MOTOR_CURRENT_PWM

        #ifdef __SAM3X8E__
          #define _RESET_CURRENT_PWM_FREQ(P) NOOP
        #else
          #define _RESET_CURRENT_PWM_FREQ(P) hal.set_pwm_frequency(pin_t(P), MOTOR_CURRENT_PWM_FREQUENCY)
        #endif
        #define INIT_CURRENT_PWM(P) do{ SET_PWM(MOTOR_CURRENT_PWM_## P ##_PIN); _RESET_CURRENT_PWM_FREQ(MOTOR_CURRENT_PWM_## P ##_PIN); }while(0)

        #if PIN_EXISTS(MOTOR_CURRENT_PWM_X)
          INIT_CURRENT_PWM(X);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Y)
          INIT_CURRENT_PWM(Y);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_XY)
          INIT_CURRENT_PWM(XY);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_I)
          INIT_CURRENT_PWM(I);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_J)
          INIT_CURRENT_PWM(J);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_K)
          INIT_CURRENT_PWM(K);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_U)
          INIT_CURRENT_PWM(U);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_V)
          INIT_CURRENT_PWM(V);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_W)
          INIT_CURRENT_PWM(W);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_Z)
          INIT_CURRENT_PWM(Z);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E)
          INIT_CURRENT_PWM(E);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E0)
          INIT_CURRENT_PWM(E0);
        #endif
        #if PIN_EXISTS(MOTOR_CURRENT_PWM_E1)
          INIT_CURRENT_PWM(E1);
        #endif

        refresh_motor_power();

      #endif
    }

  #endif

#else // PRINTRBOARD_G2

  #include HAL_PATH(.., fastio/G2_PWM.h)

#endif

#if HAS_MICROSTEPS

  /**
   * Software-controlled Microstepping
   */

  void Stepper::microstep_init() {
    #if HAS_X_MS_PINS
      SET_OUTPUT(X_MS1_PIN); SET_OUTPUT(X_MS2_PIN);
      #if PIN_EXISTS(X_MS3)
        SET_OUTPUT(X_MS3_PIN);
      #endif
    #endif
    #if HAS_X2_MS_PINS
      SET_OUTPUT(X2_MS1_PIN); SET_OUTPUT(X2_MS2_PIN);
      #if PIN_EXISTS(X2_MS3)
        SET_OUTPUT(X2_MS3_PIN);
      #endif
    #endif
    #if HAS_Y_MS_PINS
      SET_OUTPUT(Y_MS1_PIN); SET_OUTPUT(Y_MS2_PIN);
      #if PIN_EXISTS(Y_MS3)
        SET_OUTPUT(Y_MS3_PIN);
      #endif
    #endif
    #if HAS_Y2_MS_PINS
      SET_OUTPUT(Y2_MS1_PIN); SET_OUTPUT(Y2_MS2_PIN);
      #if PIN_EXISTS(Y2_MS3)
        SET_OUTPUT(Y2_MS3_PIN);
      #endif
    #endif
    #if HAS_Z_MS_PINS
      SET_OUTPUT(Z_MS1_PIN); SET_OUTPUT(Z_MS2_PIN);
      #if PIN_EXISTS(Z_MS3)
        SET_OUTPUT(Z_MS3_PIN);
      #endif
    #endif
    #if HAS_Z2_MS_PINS
      SET_OUTPUT(Z2_MS1_PIN); SET_OUTPUT(Z2_MS2_PIN);
      #if PIN_EXISTS(Z2_MS3)
        SET_OUTPUT(Z2_MS3_PIN);
      #endif
    #endif
    #if HAS_Z3_MS_PINS
      SET_OUTPUT(Z3_MS1_PIN); SET_OUTPUT(Z3_MS2_PIN);
      #if PIN_EXISTS(Z3_MS3)
        SET_OUTPUT(Z3_MS3_PIN);
      #endif
    #endif
    #if HAS_Z4_MS_PINS
      SET_OUTPUT(Z4_MS1_PIN); SET_OUTPUT(Z4_MS2_PIN);
      #if PIN_EXISTS(Z4_MS3)
        SET_OUTPUT(Z4_MS3_PIN);
      #endif
    #endif
    #if HAS_I_MS_PINS
      SET_OUTPUT(I_MS1_PIN); SET_OUTPUT(I_MS2_PIN);
      #if PIN_EXISTS(I_MS3)
        SET_OUTPUT(I_MS3_PIN);
      #endif
    #endif
    #if HAS_J_MS_PINS
      SET_OUTPUT(J_MS1_PIN); SET_OUTPUT(J_MS2_PIN);
      #if PIN_EXISTS(J_MS3)
        SET_OUTPUT(J_MS3_PIN);
      #endif
    #endif
    #if HAS_K_MS_PINS
      SET_OUTPUT(K_MS1_PIN); SET_OUTPUT(K_MS2_PIN);
      #if PIN_EXISTS(K_MS3)
        SET_OUTPUT(K_MS3_PIN);
      #endif
    #endif
    #if HAS_U_MS_PINS
      SET_OUTPUT(U_MS1_PIN); SET_OUTPUT(U_MS2_PIN);
      #if PIN_EXISTS(U_MS3)
        SET_OUTPUT(U_MS3_PIN);
      #endif
    #endif
    #if HAS_V_MS_PINS
      SET_OUTPUT(V_MS1_PIN); SET_OUTPUT(V_MS2_PIN);
      #if PIN_EXISTS(V_MS3)
        SET_OUTPUT(V_MS3_PIN);
      #endif
    #endif
    #if HAS_W_MS_PINS
      SET_OUTPUT(W_MS1_PIN); SET_OUTPUT(W_MS2_PIN);
      #if PIN_EXISTS(W_MS3)
        SET_OUTPUT(W_MS3_PIN);
      #endif
    #endif
    #if HAS_E0_MS_PINS
      SET_OUTPUT(E0_MS1_PIN); SET_OUTPUT(E0_MS2_PIN);
      #if PIN_EXISTS(E0_MS3)
        SET_OUTPUT(E0_MS3_PIN);
      #endif
    #endif
    #if HAS_E1_MS_PINS
      SET_OUTPUT(E1_MS1_PIN); SET_OUTPUT(E1_MS2_PIN);
      #if PIN_EXISTS(E1_MS3)
        SET_OUTPUT(E1_MS3_PIN);
      #endif
    #endif
    #if HAS_E2_MS_PINS
      SET_OUTPUT(E2_MS1_PIN); SET_OUTPUT(E2_MS2_PIN);
      #if PIN_EXISTS(E2_MS3)
        SET_OUTPUT(E2_MS3_PIN);
      #endif
    #endif
    #if HAS_E3_MS_PINS
      SET_OUTPUT(E3_MS1_PIN); SET_OUTPUT(E3_MS2_PIN);
      #if PIN_EXISTS(E3_MS3)
        SET_OUTPUT(E3_MS3_PIN);
      #endif
    #endif
    #if HAS_E4_MS_PINS
      SET_OUTPUT(E4_MS1_PIN); SET_OUTPUT(E4_MS2_PIN);
      #if PIN_EXISTS(E4_MS3)
        SET_OUTPUT(E4_MS3_PIN);
      #endif
    #endif
    #if HAS_E5_MS_PINS
      SET_OUTPUT(E5_MS1_PIN); SET_OUTPUT(E5_MS2_PIN);
      #if PIN_EXISTS(E5_MS3)
        SET_OUTPUT(E5_MS3_PIN);
      #endif
    #endif
    #if HAS_E6_MS_PINS
      SET_OUTPUT(E6_MS1_PIN); SET_OUTPUT(E6_MS2_PIN);
      #if PIN_EXISTS(E6_MS3)
        SET_OUTPUT(E6_MS3_PIN);
      #endif
    #endif
    #if HAS_E7_MS_PINS
      SET_OUTPUT(E7_MS1_PIN); SET_OUTPUT(E7_MS2_PIN);
      #if PIN_EXISTS(E7_MS3)
        SET_OUTPUT(E7_MS3_PIN);
      #endif
    #endif

    static const uint8_t microstep_modes[] = MICROSTEP_MODES;
    for (uint16_t i = 0; i < COUNT(microstep_modes); i++)
      microstep_mode(i, microstep_modes[i]);
  }

  void Stepper::microstep_ms(const uint8_t driver, const int8_t ms1, const int8_t ms2, const int8_t ms3) {
    if (ms1 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS
            WRITE(X_MS1_PIN, ms1);
          #endif
          #if HAS_X2_MS_PINS
            WRITE(X2_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS
            WRITE(Y_MS1_PIN, ms1);
          #endif
          #if HAS_Y2_MS_PINS
            WRITE(Y2_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS
            WRITE(Z_MS1_PIN, ms1);
          #endif
          #if HAS_Z2_MS_PINS
            WRITE(Z2_MS1_PIN, ms1);
          #endif
          #if HAS_Z3_MS_PINS
            WRITE(Z3_MS1_PIN, ms1);
          #endif
          #if HAS_Z4_MS_PINS
            WRITE(Z4_MS1_PIN, ms1);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS
        case  I_AXIS: WRITE(I_MS1_PIN, ms1); break;
      #endif
      #if HAS_J_MS_PINS
        case  J_AXIS: WRITE(J_MS1_PIN, ms1); break;
      #endif
      #if HAS_K_MS_PINS
        case  K_AXIS: WRITE(K_MS1_PIN, ms1); break;
      #endif
      #if HAS_U_MS_PINS
        case  U_AXIS: WRITE(U_MS1_PIN, ms1); break;
      #endif
      #if HAS_V_MS_PINS
        case  V_AXIS: WRITE(V_MS1_PIN, ms1); break;
      #endif
      #if HAS_W_MS_PINS
        case  W_AXIS: WRITE(W_MS1_PIN, ms1); break;
      #endif
      #if HAS_E0_MS_PINS
        case  E_AXIS: WRITE(E0_MS1_PIN, ms1); break;
      #endif
      #if HAS_E1_MS_PINS
        case (E_AXIS + 1): WRITE(E1_MS1_PIN, ms1); break;
      #endif
      #if HAS_E2_MS_PINS
        case (E_AXIS + 2): WRITE(E2_MS1_PIN, ms1); break;
      #endif
      #if HAS_E3_MS_PINS
        case (E_AXIS + 3): WRITE(E3_MS1_PIN, ms1); break;
      #endif
      #if HAS_E4_MS_PINS
        case (E_AXIS + 4): WRITE(E4_MS1_PIN, ms1); break;
      #endif
      #if HAS_E5_MS_PINS
        case (E_AXIS + 5): WRITE(E5_MS1_PIN, ms1); break;
      #endif
      #if HAS_E6_MS_PINS
        case (E_AXIS + 6): WRITE(E6_MS1_PIN, ms1); break;
      #endif
      #if HAS_E7_MS_PINS
        case (E_AXIS + 7): WRITE(E7_MS1_PIN, ms1); break;
      #endif
    }
    if (ms2 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS
            WRITE(X_MS2_PIN, ms2);
          #endif
          #if HAS_X2_MS_PINS
            WRITE(X2_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS
            WRITE(Y_MS2_PIN, ms2);
          #endif
          #if HAS_Y2_MS_PINS
            WRITE(Y2_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS
            WRITE(Z_MS2_PIN, ms2);
          #endif
          #if HAS_Z2_MS_PINS
            WRITE(Z2_MS2_PIN, ms2);
          #endif
          #if HAS_Z3_MS_PINS
            WRITE(Z3_MS2_PIN, ms2);
          #endif
          #if HAS_Z4_MS_PINS
            WRITE(Z4_MS2_PIN, ms2);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS
        case  I_AXIS: WRITE(I_MS2_PIN, ms2); break;
      #endif
      #if HAS_J_MS_PINS
        case  J_AXIS: WRITE(J_MS2_PIN, ms2); break;
      #endif
      #if HAS_K_MS_PINS
        case  K_AXIS: WRITE(K_MS2_PIN, ms2); break;
      #endif
      #if HAS_U_MS_PINS
        case  U_AXIS: WRITE(U_MS2_PIN, ms2); break;
      #endif
      #if HAS_V_MS_PINS
        case  V_AXIS: WRITE(V_MS2_PIN, ms2); break;
      #endif
      #if HAS_W_MS_PINS
        case  W_AXIS: WRITE(W_MS2_PIN, ms2); break;
      #endif
      #if HAS_E0_MS_PINS
        case  E_AXIS: WRITE(E0_MS2_PIN, ms2); break;
      #endif
      #if HAS_E1_MS_PINS
        case (E_AXIS + 1): WRITE(E1_MS2_PIN, ms2); break;
      #endif
      #if HAS_E2_MS_PINS
        case (E_AXIS + 2): WRITE(E2_MS2_PIN, ms2); break;
      #endif
      #if HAS_E3_MS_PINS
        case (E_AXIS + 3): WRITE(E3_MS2_PIN, ms2); break;
      #endif
      #if HAS_E4_MS_PINS
        case (E_AXIS + 4): WRITE(E4_MS2_PIN, ms2); break;
      #endif
      #if HAS_E5_MS_PINS
        case (E_AXIS + 5): WRITE(E5_MS2_PIN, ms2); break;
      #endif
      #if HAS_E6_MS_PINS
        case (E_AXIS + 6): WRITE(E6_MS2_PIN, ms2); break;
      #endif
      #if HAS_E7_MS_PINS
        case (E_AXIS + 7): WRITE(E7_MS2_PIN, ms2); break;
      #endif
    }
    if (ms3 >= 0) switch (driver) {
      #if HAS_X_MS_PINS || HAS_X2_MS_PINS
        case X_AXIS:
          #if HAS_X_MS_PINS && PIN_EXISTS(X_MS3)
            WRITE(X_MS3_PIN, ms3);
          #endif
          #if HAS_X2_MS_PINS && PIN_EXISTS(X2_MS3)
            WRITE(X2_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_Y_MS_PINS || HAS_Y2_MS_PINS
        case Y_AXIS:
          #if HAS_Y_MS_PINS && PIN_EXISTS(Y_MS3)
            WRITE(Y_MS3_PIN, ms3);
          #endif
          #if HAS_Y2_MS_PINS && PIN_EXISTS(Y2_MS3)
            WRITE(Y2_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_SOME_Z_MS_PINS
        case Z_AXIS:
          #if HAS_Z_MS_PINS && PIN_EXISTS(Z_MS3)
            WRITE(Z_MS3_PIN, ms3);
          #endif
          #if HAS_Z2_MS_PINS && PIN_EXISTS(Z2_MS3)
            WRITE(Z2_MS3_PIN, ms3);
          #endif
          #if HAS_Z3_MS_PINS && PIN_EXISTS(Z3_MS3)
            WRITE(Z3_MS3_PIN, ms3);
          #endif
          #if HAS_Z4_MS_PINS && PIN_EXISTS(Z4_MS3)
            WRITE(Z4_MS3_PIN, ms3);
          #endif
          break;
      #endif
      #if HAS_I_MS_PINS && PIN_EXISTS(I_MS3)
        case  I_AXIS: WRITE(I_MS3_PIN, ms3); break;
      #endif
      #if HAS_J_MS_PINS && PIN_EXISTS(J_MS3)
        case  J_AXIS: WRITE(J_MS3_PIN, ms3); break;
      #endif
      #if HAS_K_MS_PINS && PIN_EXISTS(K_MS3)
        case  K_AXIS: WRITE(K_MS3_PIN, ms3); break;
      #endif
      #if HAS_U_MS_PINS && PIN_EXISTS(U_MS3)
        case  U_AXIS: WRITE(U_MS3_PIN, ms3); break;
      #endif
      #if HAS_V_MS_PINS && PIN_EXISTS(V_MS3)
        case  V_AXIS: WRITE(V_MS3_PIN, ms3); break;
      #endif
      #if HAS_W_MS_PINS && PIN_EXISTS(W_MS3)
        case  W_AXIS: WRITE(W_MS3_PIN, ms3); break;
      #endif
      #if HAS_E0_MS_PINS && PIN_EXISTS(E0_MS3)
        case  E_AXIS: WRITE(E0_MS3_PIN, ms3); break;
      #endif
      #if HAS_E1_MS_PINS && PIN_EXISTS(E1_MS3)
        case (E_AXIS + 1): WRITE(E1_MS3_PIN, ms3); break;
      #endif
      #if HAS_E2_MS_PINS && PIN_EXISTS(E2_MS3)
        case (E_AXIS + 2): WRITE(E2_MS3_PIN, ms3); break;
      #endif
      #if HAS_E3_MS_PINS && PIN_EXISTS(E3_MS3)
        case (E_AXIS + 3): WRITE(E3_MS3_PIN, ms3); break;
      #endif
      #if HAS_E4_MS_PINS && PIN_EXISTS(E4_MS3)
        case (E_AXIS + 4): WRITE(E4_MS3_PIN, ms3); break;
      #endif
      #if HAS_E5_MS_PINS && PIN_EXISTS(E5_MS3)
        case (E_AXIS + 5): WRITE(E5_MS3_PIN, ms3); break;
      #endif
      #if HAS_E6_MS_PINS && PIN_EXISTS(E6_MS3)
        case (E_AXIS + 6): WRITE(E6_MS3_PIN, ms3); break;
      #endif
      #if HAS_E7_MS_PINS && PIN_EXISTS(E7_MS3)
        case (E_AXIS + 7): WRITE(E7_MS3_PIN, ms3); break;
      #endif
    }
  }

  // MS1 MS2 MS3 Stepper Driver Microstepping mode table
  #ifndef MICROSTEP1
    #define MICROSTEP1 LOW,LOW,LOW
  #endif
  #if ENABLED(HEROIC_STEPPER_DRIVERS)
    #ifndef MICROSTEP128
      #define MICROSTEP128 LOW,HIGH,LOW
    #endif
  #else
    #ifndef MICROSTEP2
      #define MICROSTEP2 HIGH,LOW,LOW
    #endif
    #ifndef MICROSTEP4
      #define MICROSTEP4 LOW,HIGH,LOW
    #endif
  #endif
  #ifndef MICROSTEP8
    #define MICROSTEP8 HIGH,HIGH,LOW
  #endif
  #ifndef MICROSTEP16
    #define MICROSTEP16 HIGH,HIGH,LOW
  #endif

  void Stepper::microstep_mode(const uint8_t driver, const uint8_t stepping_mode) {
    switch (stepping_mode) {
      #ifdef MICROSTEP1
        case 1: microstep_ms(driver, MICROSTEP1); break;
      #endif
      #ifdef MICROSTEP2
        case 2: microstep_ms(driver, MICROSTEP2); break;
      #endif
      #ifdef MICROSTEP4
        case 4: microstep_ms(driver, MICROSTEP4); break;
      #endif
      #ifdef MICROSTEP8
        case 8: microstep_ms(driver, MICROSTEP8); break;
      #endif
      #ifdef MICROSTEP16
        case 16: microstep_ms(driver, MICROSTEP16); break;
      #endif
      #ifdef MICROSTEP32
        case 32: microstep_ms(driver, MICROSTEP32); break;
      #endif
      #ifdef MICROSTEP64
        case 64: microstep_ms(driver, MICROSTEP64); break;
      #endif
      #ifdef MICROSTEP128
        case 128: microstep_ms(driver, MICROSTEP128); break;
      #endif

      default: SERIAL_ERROR_MSG("Microsteps unavailable"); break;
    }
  }

  void Stepper::microstep_readings() {
    #define PIN_CHAR(P) SERIAL_CHAR('0' + READ(P##_PIN))
    #define MS_LINE(A)  do{ SERIAL_ECHOPGM(" " STRINGIFY(A) ":"); PIN_CHAR(A##_MS1); PIN_CHAR(A##_MS2); }while(0)
    SERIAL_ECHOPGM("MS1|2|3 Pins");
    #if HAS_X_MS_PINS
      MS_LINE(X);
      #if PIN_EXISTS(X_MS3)
        PIN_CHAR(X_MS3);
      #endif
    #endif
    #if HAS_Y_MS_PINS
      MS_LINE(Y);
      #if PIN_EXISTS(Y_MS3)
        PIN_CHAR(Y_MS3);
      #endif
    #endif
    #if HAS_Z_MS_PINS
      MS_LINE(Z);
      #if PIN_EXISTS(Z_MS3)
        PIN_CHAR(Z_MS3);
      #endif
    #endif
    #if HAS_I_MS_PINS
      MS_LINE(I);
      #if PIN_EXISTS(I_MS3)
        PIN_CHAR(I_MS3);
      #endif
    #endif
    #if HAS_J_MS_PINS
      MS_LINE(J);
      #if PIN_EXISTS(J_MS3)
        PIN_CHAR(J_MS3);
      #endif
    #endif
    #if HAS_K_MS_PINS
      MS_LINE(K);
      #if PIN_EXISTS(K_MS3)
        PIN_CHAR(K_MS3);
      #endif
    #endif
    #if HAS_U_MS_PINS
      MS_LINE(U);
      #if PIN_EXISTS(U_MS3)
        PIN_CHAR(U_MS3);
      #endif
    #endif
    #if HAS_V_MS_PINS
      MS_LINE(V);
      #if PIN_EXISTS(V_MS3)
        PIN_CHAR(V_MS3);
      #endif
    #endif
    #if HAS_W_MS_PINS
      MS_LINE(W);
      #if PIN_EXISTS(W_MS3)
        PIN_CHAR(W_MS3);
      #endif
    #endif
    #if HAS_E0_MS_PINS
      MS_LINE(E0);
      #if PIN_EXISTS(E0_MS3)
        PIN_CHAR(E0_MS3);
      #endif
    #endif
    #if HAS_E1_MS_PINS
      MS_LINE(E1);
      #if PIN_EXISTS(E1_MS3)
        PIN_CHAR(E1_MS3);
      #endif
    #endif
    #if HAS_E2_MS_PINS
      MS_LINE(E2);
      #if PIN_EXISTS(E2_MS3)
        PIN_CHAR(E2_MS3);
      #endif
    #endif
    #if HAS_E3_MS_PINS
      MS_LINE(E3);
      #if PIN_EXISTS(E3_MS3)
        PIN_CHAR(E3_MS3);
      #endif
    #endif
    #if HAS_E4_MS_PINS
      MS_LINE(E4);
      #if PIN_EXISTS(E4_MS3)
        PIN_CHAR(E4_MS3);
      #endif
    #endif
    #if HAS_E5_MS_PINS
      MS_LINE(E5);
      #if PIN_EXISTS(E5_MS3)
        PIN_CHAR(E5_MS3);
      #endif
    #endif
    #if HAS_E6_MS_PINS
      MS_LINE(E6);
      #if PIN_EXISTS(E6_MS3)
        PIN_CHAR(E6_MS3);
      #endif
    #endif
    #if HAS_E7_MS_PINS
      MS_LINE(E7);
      #if PIN_EXISTS(E7_MS3)
        PIN_CHAR(E7_MS3);
      #endif
    #endif
    SERIAL_EOL();
  }

#endif // HAS_MICROSTEPS
