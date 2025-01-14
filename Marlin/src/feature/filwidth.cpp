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

#if ENABLED(FILAMENT_WIDTH_SENSOR)

#include "filwidth.h"

FilamentWidthSensor filwidth;

bool FilamentWidthSensor::enabled; // = false                           // (M405-M406) Filament Width Sensor ON/OFF.
uint32_t FilamentWidthSensor::accum; // = 0                             // ADC accumulator
uint16_t FilamentWidthSensor::raw; // = 0                               // Measured filament diameter - one extruder only
float FilamentWidthSensor::nominal_mm = DEFAULT_NOMINAL_FILAMENT_DIA,   // (M104) Nominal filament width
      FilamentWidthSensor::measured_mm = DEFAULT_MEASURED_FILAMENT_DIA, // Measured filament diameter
      FilamentWidthSensor::e_count = 0,
      FilamentWidthSensor::delay_dist = 0;
uint8_t FilamentWidthSensor::meas_delay_cm = MEASUREMENT_DELAY_CM;      // Distance delay setting
int8_t FilamentWidthSensor::ratios[MAX_MEASUREMENT_DELAY + 1],          // Ring buffer to delay measurement. (Extruder factor minus 100)
       FilamentWidthSensor::index_r,                                    // Indexes into ring buffer
       FilamentWidthSensor::index_w;

void FilamentWidthSensor::init() {
  const int8_t ratio = sample_to_size_ratio();
  for (uint8_t i = 0; i < COUNT(ratios); ++i) ratios[i] = ratio;
  index_r = index_w = 0;
}

#endif // FILAMENT_WIDTH_SENSOR
