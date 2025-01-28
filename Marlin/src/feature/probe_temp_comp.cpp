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

#include "../inc/MarlinConfigPre.h"

#if HAS_PTC

//#define DEBUG_PTC   // Print extra debug output with 'M871'

#include "probe_temp_comp.h"
#include <math.h>
#include "../module/temperature.h"

ProbeTempComp ptc;

#if ENABLED(PTC_PROBE)
  constexpr int16_t z_offsets_probe_default[PTC_PROBE_COUNT] = PTC_PROBE_ZOFFS;
  int16_t ProbeTempComp::z_offsets_probe[PTC_PROBE_COUNT] = PTC_PROBE_ZOFFS;
#endif

#if ENABLED(PTC_BED)
  constexpr int16_t z_offsets_bed_default[PTC_BED_COUNT] = PTC_BED_ZOFFS;
  int16_t ProbeTempComp::z_offsets_bed[PTC_BED_COUNT] = PTC_BED_ZOFFS;
#endif

#if ENABLED(PTC_HOTEND)
  constexpr int16_t z_offsets_hotend_default[PTC_HOTEND_COUNT] = PTC_HOTEND_ZOFFS;
  int16_t ProbeTempComp::z_offsets_hotend[PTC_HOTEND_COUNT] = PTC_HOTEND_ZOFFS;
#endif

int16_t *ProbeTempComp::sensor_z_offsets[TSI_COUNT] = {
  #if ENABLED(PTC_PROBE)
    ProbeTempComp::z_offsets_probe,
  #endif
  #if ENABLED(PTC_BED)
    ProbeTempComp::z_offsets_bed,
  #endif
  #if ENABLED(PTC_HOTEND)
    ProbeTempComp::z_offsets_hotend,
  #endif
};

constexpr temp_calib_t ProbeTempComp::cali_info[TSI_COUNT];

uint8_t ProbeTempComp::calib_idx; // = 0
float ProbeTempComp::init_measurement; // = 0.0
bool ProbeTempComp::enabled = true;

void ProbeTempComp::reset() {
  TERN_(PTC_PROBE, for (uint_fast8_t i = 0; i < PTC_PROBE_COUNT; ++i) z_offsets_probe[i] = z_offsets_probe_default[i]);
  TERN_(PTC_BED, for (uint_fast8_t i = 0; i < PTC_BED_COUNT; ++i) z_offsets_bed[i] = z_offsets_bed_default[i]);
  TERN_(PTC_HOTEND, for (uint_fast8_t i = 0; i < PTC_HOTEND_COUNT; ++i) z_offsets_hotend[i] = z_offsets_hotend_default[i]);
}

void ProbeTempComp::clear_offsets(const TempSensorID tsi) {
  for (uint_fast8_t i = 0; i < cali_info[tsi].measurements; ++i)
    sensor_z_offsets[tsi][i] = 0;
  calib_idx = 0;
}

bool ProbeTempComp::set_offset(const TempSensorID tsi, const uint8_t idx, const int16_t offset) {
  if (idx >= cali_info[tsi].measurements) return false;
  sensor_z_offsets[tsi][idx] = offset;
  return true;
}

void ProbeTempComp::print_offsets() {
  for (uint_fast8_t s = 0; s < TSI_COUNT; ++s) {
    celsius_t temp = cali_info[s].start_temp;
    for (int16_t i = -1; i < cali_info[s].measurements; ++i) {
      SERIAL_ECHOLN(
        TERN_(PTC_BED, s == TSI_BED ? F("Bed") :) TERN_(PTC_HOTEND, s == TSI_EXT ? F("Extruder") :) F("Probe"),
        F(" temp: "), temp, F("C; Offset: "), i < 0 ? 0.0f : sensor_z_offsets[s][i], F(" um")
      );
      temp += cali_info[s].temp_resolution;
    }
  }
  #if ENABLED(DEBUG_PTC)
    float meas[4] = { 0, 0, 0, 0 };
    compensate_measurement(TSI_PROBE, 27.5, meas[0]);
    compensate_measurement(TSI_PROBE, 32.5, meas[1]);
    compensate_measurement(TSI_PROBE, 77.5, meas[2]);
    compensate_measurement(TSI_PROBE, 82.5, meas[3]);
    SERIAL_ECHOLNPGM("DEBUG_PTC 27.5:", meas[0], " 32.5:", meas[1], " 77.5:", meas[2], " 82.5:", meas[3]);
  #endif
}

void ProbeTempComp::prepare_new_calibration(const_float_t init_meas_z) {
  calib_idx = 0;
  init_measurement = init_meas_z;
}

void ProbeTempComp::push_back_new_measurement(const TempSensorID tsi, const_float_t meas_z) {
  if (calib_idx >= cali_info[tsi].measurements) return;
  sensor_z_offsets[tsi][calib_idx++] = static_cast<int16_t>((meas_z - init_measurement) * 1000.0f);
}

bool ProbeTempComp::finish_calibration(const TempSensorID tsi) {
  if (!calib_idx) {
    SERIAL_ECHOLNPGM("!No measurements.");
    clear_offsets(tsi);
    return false;
  }

  const uint8_t measurements = cali_info[tsi].measurements;
  const celsius_t start_temp = cali_info[tsi].start_temp,
                    res_temp = cali_info[tsi].temp_resolution;
  int16_t * const data = sensor_z_offsets[tsi];

  // Extrapolate
  float k, d;
  if (calib_idx < measurements) {
    SERIAL_ECHOLNPGM("Got ", calib_idx, " measurements. ");
    if (linear_regression(tsi, k, d)) {
      SERIAL_ECHOPGM("Applying linear extrapolation");
      for (; calib_idx < measurements; ++calib_idx) {
        const celsius_float_t temp = start_temp + float(calib_idx + 1) * res_temp;
        data[calib_idx] = static_cast<int16_t>(k * temp + d);
      }
    }
    else {
      // Simply use the last measured value for higher temperatures
      SERIAL_ECHOPGM("Failed to extrapolate");
      const int16_t last_val = data[calib_idx-1];
      for (; calib_idx < measurements; ++calib_idx)
        data[calib_idx] = last_val;
    }
    SERIAL_ECHOLNPGM(" for higher temperatures.");
  }

  // Sanity check
  for (calib_idx = 0; calib_idx < measurements; ++calib_idx) {
    // Restrict the max. offset
    if (ABS(data[calib_idx]) > 2000) {
      SERIAL_ECHOLNPGM("!Invalid Z-offset detected (0-2).");
      clear_offsets(tsi);
      return false;
    }
    // Restrict the max. offset difference between two probings
    if (calib_idx > 0 && ABS(data[calib_idx - 1] - data[calib_idx]) > 800) {
      SERIAL_ECHOLNPGM("!Invalid Z-offset between two probings detected (0-0.8).");
      clear_offsets(tsi);
      return false;
    }
  }

  return true;
}

void ProbeTempComp::apply_compensation(float &meas_z) {
  if (!enabled) return;
  TERN_(PTC_BED,    compensate_measurement(TSI_BED,   thermalManager.degBed(),     meas_z));
  TERN_(PTC_PROBE,  compensate_measurement(TSI_PROBE, thermalManager.degProbe(),   meas_z));
  TERN_(PTC_HOTEND, compensate_measurement(TSI_EXT,   thermalManager.degHotend(0), meas_z));
}

void ProbeTempComp::compensate_measurement(const TempSensorID tsi, const celsius_t temp, float &meas_z) {
  const uint8_t measurements = cali_info[tsi].measurements;
  const celsius_t start_temp = cali_info[tsi].start_temp,
                  res_temp = cali_info[tsi].temp_resolution,
                  end_temp = start_temp + measurements * res_temp;
  const int16_t * const data = sensor_z_offsets[tsi];

  // Given a data index, return { celsius, zoffset } in the form { x, y }
  auto tpoint = [&](uint8_t i) -> xy_float_t {
    return xy_float_t({ static_cast<float>(start_temp) + i * res_temp, i ? static_cast<float>(data[i - 1]) : 0.0f });
  };

  // Interpolate Z based on a temperature being within a given range
  auto linear_interp = [](const_float_t x, xy_float_t p1, xy_float_t p2) {
    //   zoffs1 +      zoffset_per_toffset      *  toffset
    return p1.y + (p2.y - p1.y) / (p2.x - p1.x) * (x - p1.x);
  };

  // offset in µm
  float offset = 0.0f;

  #if PTC_LINEAR_EXTRAPOLATION
    if (temp < start_temp)
      offset = linear_interp(temp, tpoint(0), tpoint(PTC_LINEAR_EXTRAPOLATION));
    else if (temp >= end_temp)
      offset = linear_interp(temp, tpoint(measurements - PTC_LINEAR_EXTRAPOLATION), tpoint(measurements));
  #else
    if (temp < start_temp)
      offset = 0.0f;
    else if (temp >= end_temp)
      offset = static_cast<float>(data[measurements - 1]);
  #endif
    else {
      // Linear interpolation
      const int8_t idx = static_cast<int8_t>((temp - start_temp) / res_temp);
      offset = linear_interp(temp, tpoint(idx), tpoint(idx + 1));
    }

  // convert offset to mm and apply it
  meas_z -= offset / 1000.0f;
}

bool ProbeTempComp::linear_regression(const TempSensorID tsi, float &k, float &d) {
  if (!WITHIN(calib_idx, 1, cali_info[tsi].measurements)) return false;

  const celsius_t start_temp = cali_info[tsi].start_temp,
                    res_temp = cali_info[tsi].temp_resolution;
  const int16_t * const data = sensor_z_offsets[tsi];

  float sum_x = start_temp,
        sum_x2 = sq(start_temp),
        sum_xy = 0, sum_y = 0;

  float xi = static_cast<float>(start_temp);
  for (uint_fast8_t i = 0; i < calib_idx; ++i) {
    const float yi = static_cast<float>(data[i]);
    xi += res_temp;
    sum_x += xi;
    sum_x2 += sq(xi);
    sum_xy += xi * yi;
    sum_y += yi;
  }

  const float denom = static_cast<float>(calib_idx + 1) * sum_x2 - sq(sum_x);
  if (fabs(denom) <= 10e-5) {
    // Singularity - unable to solve
    k = d = 0.0;
    return false;
  }

  k = (static_cast<float>(calib_idx + 1) * sum_xy - sum_x * sum_y) / denom;
  d = (sum_y - k * sum_x) / static_cast<float>(calib_idx + 1);

  return true;
}

#endif // HAS_PTC
