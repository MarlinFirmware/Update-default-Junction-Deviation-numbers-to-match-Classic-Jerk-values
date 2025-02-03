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
 * feature/backlash.h
 */

#include "../inc/MarlinConfigPre.h"
#include "../module/planner.h"

class Backlash {
public:
  static constexpr uint8_t all_on = 0xFF, all_off = 0x00;

private:
  static AxisBits last_direction_bits;
  static xyz_long_t residual_error;

  #if ENABLED(BACKLASH_GCODE)
    static uint8_t correction;
    static xyz_float_t distance_mm;
    #ifdef BACKLASH_SMOOTHING_MM
      static float smoothing_mm;
    #endif
  #else
    static constexpr uint8_t correction = (BACKLASH_CORRECTION) * all_on;
    static const xyz_float_t distance_mm;
    #ifdef BACKLASH_SMOOTHING_MM
      static constexpr float smoothing_mm = BACKLASH_SMOOTHING_MM;
    #endif
  #endif

  #if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
    static xyz_float_t measured_mm;
    static xyz_uint8_t measured_count;
  #endif

  class StepAdjuster;

public:
  static float get_measurement(const AxisEnum a) {
    UNUSED(a);
    // Return the measurement averaged over all readings
    return TERN(MEASURE_BACKLASH_WHEN_PROBING
      , measured_count[a] > 0 ? measured_mm[a] / measured_count[a] : 0
      , 0
    );
  }

  static bool has_measurement(const AxisEnum a) {
    UNUSED(a);
    return TERN0(MEASURE_BACKLASH_WHEN_PROBING, measured_count[a] > 0);
  }

  static bool has_any_measurement() {
    return has_measurement(X_AXIS) || has_measurement(Y_AXIS) || has_measurement(Z_AXIS);
  }

  static void add_correction_steps(const xyze_long_t &dist, const AxisBits dm, block_t * const block);
  static int32_t get_applied_steps(const AxisEnum axis);

  #if ENABLED(BACKLASH_GCODE)
    static void set_correction_uint8(const uint8_t v);
    static uint8_t get_correction_uint8() { return correction; }
    static void set_correction(const float v) { set_correction_uint8(_MAX(0, _MIN(1.0, v)) * all_on + 0.5f); }
    static float get_correction() { return float(get_correction_uint8()) / all_on; }
    static void set_distance_mm(const AxisEnum axis, const float v);
    static float get_distance_mm(const AxisEnum axis) {return distance_mm[axis];}
    #ifdef BACKLASH_SMOOTHING_MM
      static void set_smoothing_mm(const float v);
      static float get_smoothing_mm() {return smoothing_mm;}
    #endif
  #endif

  #if ENABLED(MEASURE_BACKLASH_WHEN_PROBING)
    static void measure_with_probe();
  #endif
};

extern Backlash backlash;
