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

#include "../inc/MarlinConfigPre.h"

#define BABYSTEPS_PER_SEC 1000UL
#define BABYSTEP_TICKS ((STEPPER_TIMER_RATE) / (BABYSTEPS_PER_SEC))

#if ANY(IS_CORE, BABYSTEP_XY, I2C_POSITION_ENCODERS)
  #define BS_AXIS_IND(A) A
  #define BS_AXIS(I) AxisEnum(I)
#else
  #define BS_AXIS_IND(A) 0
  #define BS_AXIS(I) Z_AXIS
#endif

#if ENABLED(BABYSTEP_DISPLAY_TOTAL)
  #if ENABLED(BABYSTEP_XY)
    #define BS_TOTAL_IND(A) A
  #else
    #define BS_TOTAL_IND(A) 0
  #endif
#endif

class Babystep {
public:
  static volatile int16_t steps[BS_AXIS_IND(Z_AXIS) + 1];
  static int16_t accum;                                     // Total babysteps in current edit

  #if ALL(EP_BABYSTEPPING, EMERGENCY_PARSER)
    static int16_t ep_babysteps;
  #endif

  #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
    static int16_t axis_total[BS_TOTAL_IND(Z_AXIS) + 1];   // Total babysteps since G28
    static void reset_total(const AxisEnum axis) {
      if (TERN1(BABYSTEP_XY, axis == Z_AXIS))
        axis_total[BS_TOTAL_IND(axis)] = 0;
    }
  #endif

  static bool can_babystep(const AxisEnum axis);
  static void add_steps(const AxisEnum axis, const int16_t distance);
  static void add_mm(const AxisEnum axis, const_float_t mm);

  #if ENABLED(EP_BABYSTEPPING)
    // Step Z for M293 / M294
    static void z_up();
    static void z_down();
    #if ENABLED(EMERGENCY_PARSER)
      // Step Z according to steps accumulated by the EP
      FORCE_INLINE static void do_ep_steps() {
        if (ep_babysteps) {
          if (ep_babysteps > 0) { z_up();   ep_babysteps--; }
          else                  { z_down(); ep_babysteps++; }
        }
      }
    #endif
  #endif // EP_BABYSTEPPING

  #if ENABLED(BD_SENSOR)
    static void set_mm(const AxisEnum axis, const_float_t mm);
  #endif

  static bool has_steps() {
    return steps[BS_AXIS_IND(X_AXIS)] || steps[BS_AXIS_IND(Y_AXIS)] || steps[BS_AXIS_IND(Z_AXIS)];
  }

  //
  // Called by the Temperature or Stepper ISR to
  // apply accumulated babysteps to the axes.
  //
  static void task() {
    for (uint8_t i = 0; i <= BS_AXIS_IND(Z_AXIS); ++i) step_axis(BS_AXIS(i));
  }

private:
  static void step_axis(const AxisEnum axis);
};

extern Babystep babystep;
