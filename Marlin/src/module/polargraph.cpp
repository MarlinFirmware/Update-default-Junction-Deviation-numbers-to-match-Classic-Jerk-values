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
 * polargraph.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(POLARGRAPH)

#include "polargraph.h"
#include "motion.h"

// For homing:
#include "planner.h"
#include "endstops.h"
#include "../lcd/marlinui.h"
#include "../MarlinCore.h"

// Initialized by settings.load
float segments_per_second, polargraph_max_belt_len;
xy_pos_t draw_area_min, draw_area_max;

void inverse_kinematics(const xyz_pos_t &raw) {
  const float x1 = raw.x - draw_area_min.x, x2 = draw_area_max.x - raw.x, y = raw.y - draw_area_max.y;
  delta.set(HYPOT(x1, y), HYPOT(x2, y) OPTARG(HAS_Z_AXIS, raw.z));
}

#endif // POLARGRAPH
