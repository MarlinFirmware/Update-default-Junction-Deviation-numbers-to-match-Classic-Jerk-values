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

#include "../inc/MarlinConfig.h"

#if ENABLED(BABYSTEPPING)

#include "babystep.h"
#include "../MarlinCore.h"
#include "../module/motion.h"   // for axes_should_home(), BABYSTEP_ALLOWED
#include "../module/planner.h"  // for axis_steps_per_mm[]
#include "../module/stepper.h"

#if ENABLED(BABYSTEP_ALWAYS_AVAILABLE)
  #include "../gcode/gcode.h"
#endif

Babystep babystep;

volatile int16_t Babystep::steps[BS_AXIS_IND(Z_AXIS) + 1];
#if ENABLED(BABYSTEP_DISPLAY_TOTAL)
  int16_t Babystep::axis_total[BS_TOTAL_IND(Z_AXIS) + 1];
#endif
int16_t Babystep::accum;

#if ALL(EP_BABYSTEPPING, EMERGENCY_PARSER)
  int16_t Babystep::ep_babysteps;
#endif

void Babystep::step_axis(const AxisEnum axis) {
  const int16_t curTodo = steps[BS_AXIS_IND(axis)]; // get rid of volatile for performance
  if (curTodo) {
    stepper.do_babystep((AxisEnum)axis, curTodo > 0);
    if (curTodo > 0) steps[BS_AXIS_IND(axis)]--; else steps[BS_AXIS_IND(axis)]++;
  }
}

void Babystep::add_mm(const AxisEnum axis, const_float_t mm) {
  add_steps(axis, mm * planner.settings.axis_steps_per_mm[axis]);
}

#if ENABLED(BD_SENSOR)
  void Babystep::set_mm(const AxisEnum axis, const_float_t mm) {
    //if (DISABLED(BABYSTEP_WITHOUT_HOMING) && axis_should_home(axis)) return;
    const int16_t distance = mm * planner.settings.axis_steps_per_mm[axis];
    accum = distance; // Count up babysteps for the UI
    steps[BS_AXIS_IND(axis)] = distance;
    TERN_(BABYSTEP_DISPLAY_TOTAL, axis_total[BS_TOTAL_IND(axis)] = distance);
    TERN_(BABYSTEP_ALWAYS_AVAILABLE, gcode.reset_stepper_timeout());
    TERN_(BABYSTEPPING, if (has_steps()) stepper.initiateBabystepping());
  }
#endif

bool Babystep::can_babystep(const AxisEnum axis) {
  return (ENABLED(BABYSTEP_WITHOUT_HOMING) || !axis_should_home(axis));
}

void Babystep::add_steps(const AxisEnum axis, const int16_t distance) {
  if (!can_babystep(axis)) return;

  accum += distance; // Count up babysteps for the UI
  steps[BS_AXIS_IND(axis)] += distance;
  TERN_(BABYSTEP_DISPLAY_TOTAL, axis_total[BS_TOTAL_IND(axis)] += distance);
  TERN_(BABYSTEP_ALWAYS_AVAILABLE, gcode.reset_stepper_timeout());
  TERN_(BABYSTEPPING, if (has_steps()) stepper.initiateBabystepping());
}

#if ENABLED(EP_BABYSTEPPING)
  // Step Z for M293 / M294
  void Babystep::z_up()   { if (BABYSTEP_ALLOWED()) add_steps(Z_AXIS, +BABYSTEP_SIZE_Z); }
  void Babystep::z_down() { if (BABYSTEP_ALLOWED()) add_steps(Z_AXIS, -BABYSTEP_SIZE_Z); }
#endif

#endif // BABYSTEPPING
