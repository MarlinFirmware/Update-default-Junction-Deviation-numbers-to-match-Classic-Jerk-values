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
 * feature/filwidth.h
 */

#include "../inc/MarlinConfig.h"
#include "../module/planner.h"
#include "../module/thermistor/thermistors.h"

class FilamentWidthSensor {
public:
  static constexpr int MMD_CM = MAX_MEASUREMENT_DELAY + 1, MMD_MM = MMD_CM * 10;
  static bool enabled;            // (M405-M406) Filament Width Sensor ON/OFF
  static uint32_t accum;          // ADC accumulator
  static uint16_t raw;            // Measured filament diameter - one extruder only
  static float nominal_mm,        // (M104) Nominal filament width
               measured_mm,       // Measured filament diameter
               e_count, delay_dist;
  static uint8_t meas_delay_cm;   // Distance delay setting
  static int8_t ratios[MMD_CM],   // Ring buffer to delay measurement. (Extruder factor minus 100)
                index_r, index_w; // Indexes into ring buffer

  FilamentWidthSensor() { init(); }
  static void init();

  static void enable(const bool ena) { enabled = ena; }

  static void set_delay_cm(const uint8_t cm) {
    meas_delay_cm = _MIN(cm, MAX_MEASUREMENT_DELAY);
  }

  /**
   * Convert Filament Width (mm) to an extrusion ratio
   * and reduce to an 8 bit value
   *
   * A nominal width of 1.75 and measured width of 1.73
   * gives (100 * 1.75 / 1.73) for a ratio of 101 and
   * a return value of 1
   */
  static int8_t sample_to_size_ratio() {
    return ABS(nominal_mm - measured_mm) <= FILWIDTH_ERROR_MARGIN
           ? int(100.0f * nominal_mm / measured_mm) - 100 : 0;
  }

  // Apply a single ADC reading to the raw value
  static void accumulate(const uint16_t adc) {
    if (adc > 102)  // Ignore ADC under 0.5 volts
      accum += (uint32_t(adc) << 7) - (accum >> 7);
  }

  // Convert raw measurement to mm
  static float raw_to_mm(const uint16_t v) { return v * (float(ADC_VREF_MV) / 1000.0f) * RECIPROCAL(float(MAX_RAW_THERMISTOR_VALUE)); }
  static float raw_to_mm() { return raw_to_mm(raw); }

  // A scaled reading is ready
  // Divide to get to 0-16384 range since we used 1/128 IIR filter approach
  static void reading_ready() { raw = accum >> 10; }

  // Update mm from the raw measurement
  static void update_measured_mm() { measured_mm = raw_to_mm(); }

  // Update ring buffer used to delay filament measurements
  static void advance_e(const_float_t e_move) {

    // Increment counters with the E distance
    e_count += e_move;
    delay_dist += e_move;

    // Only get new measurements on forward E movement
    if (!UNEAR_ZERO(e_count)) {

      // Loop the delay distance counter (modulus by the mm length)
      while (delay_dist >= MMD_MM) delay_dist -= MMD_MM;

      // Convert into an index (cm) into the measurement array
      index_r = int8_t(delay_dist * 0.1f);

      // If the ring buffer is not full...
      if (index_r != index_w) {
        e_count = 0;                            // Reset the E movement counter
        const int8_t meas_sample = sample_to_size_ratio();
        do {
          if (++index_w >= MMD_CM) index_w = 0; // The next unused slot
          ratios[index_w] = meas_sample;        // Store the measurement
        } while (index_r != index_w);           // More slots to fill?
      }
    }
  }

  // Dynamically set the volumetric multiplier based on the delayed width measurement.
  static void update_volumetric() {
    if (enabled) {
      int8_t read_index = index_r - meas_delay_cm;
      if (read_index < 0) read_index += MMD_CM; // Loop around buffer if needed
      LIMIT(read_index, 0, MAX_MEASUREMENT_DELAY);
      planner.apply_filament_width_sensor(ratios[read_index]);
    }
  }

};

extern FilamentWidthSensor filwidth;
