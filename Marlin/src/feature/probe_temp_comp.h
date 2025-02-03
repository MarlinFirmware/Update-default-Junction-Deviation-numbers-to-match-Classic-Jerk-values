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
 * feature/probe_temp_comp.h
 */

#include "../inc/MarlinConfig.h"

enum TempSensorID : uint8_t {
  #if ENABLED(PTC_PROBE)
    TSI_PROBE,
  #endif
  #if ENABLED(PTC_BED)
    TSI_BED,
  #endif
  #if ENABLED(PTC_HOTEND)
    TSI_EXT,
  #endif
  TSI_COUNT
};

typedef struct {
  uint8_t measurements;      // Max. number of measurements to be stored (35 - 80°C)
  celsius_t temp_resolution, // Resolution in °C between measurements
            start_temp;      // Base measurement; z-offset == 0
} temp_calib_t;

/**
 * Probe temperature compensation implementation.
 * Z-probes like the P.I.N.D.A V2 allow for compensation of
 * measurement errors/shifts due to changed temperature
 */

class ProbeTempComp {
  public:

    static constexpr temp_calib_t cali_info[TSI_COUNT] = {
      #if ENABLED(PTC_PROBE)
        { PTC_PROBE_COUNT, PTC_PROBE_RES, PTC_PROBE_START },    // Probe
      #endif
      #if ENABLED(PTC_BED)
        { PTC_BED_COUNT, PTC_BED_RES, PTC_BED_START },          // Bed
      #endif
      #if ENABLED(PTC_HOTEND)
        { PTC_HOTEND_COUNT, PTC_HOTEND_RES, PTC_HOTEND_START }, // Extruder
      #endif
    };

    static int16_t *sensor_z_offsets[TSI_COUNT];
    #if ENABLED(PTC_PROBE)
      static int16_t z_offsets_probe[PTC_PROBE_COUNT];   // (µm)
    #endif
    #if ENABLED(PTC_BED)
      static int16_t z_offsets_bed[PTC_BED_COUNT];       // (µm)
    #endif
    #if ENABLED(PTC_HOTEND)
      static int16_t z_offsets_hotend[PTC_HOTEND_COUNT]; // (µm)
    #endif

    static void reset_index() { calib_idx = 0; };
    static uint8_t get_index() { return calib_idx; }
    static void reset();
    static void clear_all_offsets() {
      TERN_(PTC_PROBE, clear_offsets(TSI_PROBE));
      TERN_(PTC_BED, clear_offsets(TSI_BED));
      TERN_(PTC_HOTEND, clear_offsets(TSI_EXT));
    }
    static bool set_offset(const TempSensorID tsi, const uint8_t idx, const int16_t offset);
    static void print_offsets();
    static void prepare_new_calibration(const_float_t init_meas_z);
    static void push_back_new_measurement(const TempSensorID tsi, const_float_t meas_z);
    static bool finish_calibration(const TempSensorID tsi);
    static void set_enabled(const bool ena) { enabled = ena; }

    // Apply all temperature compensation adjustments
    static void apply_compensation(float &meas_z);

  private:
    static uint8_t calib_idx;
    static bool enabled;

    static void clear_offsets(const TempSensorID tsi);

    /**
     * Base value. Temperature compensation values will be deltas
     * to this value, set at first probe
     */
    static float init_measurement;

    /**
     * Fit a linear function in measured temperature offsets
     * to allow generating values of higher temperatures
     */
    static bool linear_regression(const TempSensorID tsi, float &k, float &d);

    static void compensate_measurement(const TempSensorID tsi, const celsius_t temp, float &meas_z);
};

extern ProbeTempComp ptc;
