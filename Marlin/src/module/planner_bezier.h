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
 * module/planner_bezier.h
 *
 * Compute and buffer movement commands for Bézier curves
 */

#include "../core/types.h"

void cubic_b_spline(
  const xyze_pos_t &position,       // current position
  const xyze_pos_t &target,         // target position
  const xy_pos_t (&offsets)[2],     // a pair of offsets
  const_feedRate_t scaled_fr_mm_s,  // mm/s scaled by feedrate %
  const uint8_t extruder
);
