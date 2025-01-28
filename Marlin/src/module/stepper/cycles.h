/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * module/stepper/cycles.h - Cycle counting for the Stepper ISR
 *
 * Estimate the amount of time the Stepper ISR will take to execute.
 *
 * These cycle counts are rough estimates used to determine whether the ISR
 * has enough time to do all its work before it should yield back to userland.
 * These constants may be updated as data is gathered from a variety of MCUs.
 */

#ifdef CPU_32_BIT
  /**
   * Duration of START_TIMED_PULSE
   *
   * ...as measured on an LPC1768 with a scope and converted to cycles.
   * Not applicable to other 32-bit processors, but as long as others
   * take longer, pulses will be longer. For example the SKR Pro
   * (stm32f407zgt6) requires ~60 cyles.
   */
  constexpr uint32_t timer_read_add_and_store_cycles = 34UL;

  // The base ISR
  constexpr uint32_t isr_base_cycles = 770UL;

  // Linear advance base time is 64 cycles
  constexpr uint32_t isr_la_base_cycles = TERN0(LIN_ADVANCE, 64UL);

  // S curve interpolation adds 40 cycles
  constexpr uint32_t isr_s_curve_cycles = TERN0(S_CURVE_ACCELERATION, TERN(STM32G0B1xx, 500UL, 40UL));

  // Input shaping base time
  constexpr uint32_t isr_shaping_base_cycles = TERN0(HAS_ZV_SHAPING, 180UL);

  // Stepper Loop base cycles
  constexpr uint32_t isr_loop_base_cycles = 4UL;

  // And each stepper (start + stop pulse) takes in worst case
  constexpr uint32_t isr_stepper_cycles = 100UL;

#else

  // Cycles to perform actions in START_TIMED_PULSE
  constexpr uint32_t timer_read_add_and_store_cycles = 13UL;

  // The base ISR
  constexpr uint32_t isr_base_cycles = 882UL;

  // Linear advance base time is 32 cycles
  constexpr uint32_t isr_la_base_cycles = TERN0(LIN_ADVANCE, 30UL);

  // S curve interpolation adds 160 cycles
  constexpr uint32_t isr_s_curve_cycles = TERN0(S_CURVE_ACCELERATION, 160UL);

  // Input shaping base time
  constexpr uint32_t isr_shaping_base_cycles = TERN0(HAS_ZV_SHAPING, 290UL);

  // Stepper Loop base cycles
  constexpr uint32_t isr_loop_base_cycles = 32UL;

  // And each stepper (start + stop pulse) takes in worst case
  constexpr uint32_t isr_stepper_cycles = 60UL;

#endif

// If linear advance is disabled, the loop also handles them
constexpr uint32_t isr_mixing_stepper_cycles = (0UL
  #if DISABLED(LIN_ADVANCE) && ENABLED(MIXING_EXTRUDER)
    + (MIXING_STEPPERS) * isr_stepper_cycles
  #endif
);

// And the total minimum loop time, for all steppers, not including the base
constexpr uint32_t min_isr_loop_cycles = isr_mixing_stepper_cycles + LOGICAL_AXES * isr_stepper_cycles;

// Calculate the minimum pulse times (high and low)
#if defined(MINIMUM_STEPPER_PULSE_NS) && defined(MAXIMUM_STEPPER_RATE)
  constexpr uint32_t _min_step_period_ns = 1000000000UL / (MAXIMUM_STEPPER_RATE),
                     _min_pulse_high_ns = MINIMUM_STEPPER_PULSE_NS,
                     _min_pulse_low_ns = _MAX(_min_step_period_ns - _MIN(_min_step_period_ns, _min_pulse_high_ns), _min_pulse_high_ns);
#elif defined(MINIMUM_STEPPER_PULSE_NS)
  // Assume equal high and low pulse durations
  constexpr uint32_t _min_pulse_high_ns = MINIMUM_STEPPER_PULSE_NS,
                     _min_pulse_low_ns = _min_pulse_high_ns;
#elif defined(MAXIMUM_STEPPER_RATE)
  // Assume equal high and low pulse durations
  constexpr uint32_t _min_pulse_high_ns = 500000000UL / MAXIMUM_STEPPER_RATE,
                     _min_pulse_low_ns = _min_pulse_high_ns;
#else
  #error "At least one of MINIMUM_STEPPER_PULSE_NS or MAXIMUM_STEPPER_RATE must be defined"
#endif

// Calculate the minimum MPU cycles needed per pulse to enforce
constexpr uint32_t min_stepper_pulse_cycles = _min_pulse_high_ns * CYCLES_PER_MICROSECOND / 1000;

// The loop takes the base time plus the time for all the bresenham logic for 1 << R pulses plus the time
// between pulses for ((1 << R) - 1) pulses. But the user could be enforcing a minimum time so the loop time is:
constexpr uint32_t isr_loop_cycles(const int R) { return ((isr_loop_base_cycles + min_isr_loop_cycles + min_stepper_pulse_cycles) * ((1UL << R) - 1) + _MAX(min_isr_loop_cycles, min_stepper_pulse_cycles)); }

// Model input shaping as an extra loop call
constexpr uint32_t isr_shaping_loop_cycles(const int R) {
  return (
    #if HAS_ZV_SHAPING
        isr_loop_base_cycles
      + isr_stepper_cycles * COUNT_ENABLED(INPUT_SHAPING_X, INPUT_SHAPING_Y, INPUT_SHAPING_Z)
    #else
      0
    #endif
  ) << R;
}

// If linear advance is enabled, then it is handled separately
#if ENABLED(LIN_ADVANCE)

  // Estimate the minimum LA loop time
  constexpr uint32_t min_isr_la_loop_cycles = (isr_stepper_cycles
    // ToDo: ???
    // HELP ME: What is what?
    // Directions are set up for MIXING_STEPPERS - like before.
    // Finding the right stepper may last up to MIXING_STEPPERS loops in get_next_stepper().
    //   These loops are a bit faster than advancing a bresenham counter.
    // Always only one E stepper is stepped.
    * TERN1(MIXING_EXTRUDER, (MIXING_STEPPERS))
  );

#endif

// The real LA loop time will be the larger minimum (pulse or loop)
constexpr uint32_t isr_la_loop_cycles = TERN0(LIN_ADVANCE, _MAX(min_stepper_pulse_cycles, min_isr_la_loop_cycles));

// Estimate the total ISR execution time in cycles given a step-per-ISR shift multiplier
constexpr uint32_t isr_execution_cycles(const int R) { return (isr_base_cycles + isr_s_curve_cycles + isr_shaping_base_cycles + isr_loop_cycles(R) + isr_shaping_loop_cycles(R) + isr_la_base_cycles + isr_la_loop_cycles) >> R; }

// The maximum allowable stepping frequency when doing 1x stepping (in Hz)
constexpr uint32_t max_step_isr_frequency_1x = (F_CPU) / isr_execution_cycles(0);
constexpr uint32_t max_step_isr_frequency_sh(const int S) { return ((F_CPU) / isr_execution_cycles(S)) >> S; }

// The minimum step ISR rate used by ADAPTIVE_STEP_SMOOTHING to target 50% CPU usage
// This does not account for the possibility of multi-stepping.
constexpr uint32_t min_step_isr_frequency = max_step_isr_frequency_1x >> 1;
