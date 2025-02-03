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
 * module/delta.h - Delta-specific functions
 */

#include "../core/types.h"
#include "../core/macros.h"

extern float delta_height;
extern abc_float_t delta_endstop_adj;
extern float delta_radius,
             delta_diagonal_rod,
             segments_per_second;
extern abc_float_t delta_tower_angle_trim;
extern xy_float_t delta_tower[ABC];
extern abc_float_t delta_diagonal_rod_2_tower;
extern float delta_clip_start_height;
extern abc_float_t delta_diagonal_rod_trim;

/**
 * Recalculate factors used for delta kinematics whenever
 * settings have been changed (e.g., by M665).
 */
void recalc_delta_settings();

/**
 * Get a safe radius for calibration
 */
#if HAS_DELTA_SENSORLESS_PROBING
  static constexpr float sensorless_radius_factor = 0.7f;
#endif

/**
 * Delta Inverse Kinematics
 *
 * Calculate the tower positions for a given machine
 * position, storing the result in the delta[] array.
 *
 * This is an expensive calculation, requiring 3 square
 * roots per segmented linear move, and strains the limits
 * of a Mega2560 with a Graphical Display.
 *
 * Suggested optimizations include:
 *
 * - Disable the home_offset (M206) and/or workspace_offset (G92)
 *   features to remove up to 12 float additions.
 *
 * - Use a fast-inverse-sqrt function and add the reciprocal.
 *   (see above)
 */

// Macro to obtain the Z position of an individual tower
#define DELTA_Z(V,T) V.z + SQRT(          \
  delta_diagonal_rod_2_tower[T] - HYPOT2( \
      delta_tower[T].x - V.x,             \
      delta_tower[T].y - V.y              \
    )                                     \
  )

#define DELTA_IK(V) delta.set(DELTA_Z(V, A_AXIS), DELTA_Z(V, B_AXIS), DELTA_Z(V, C_AXIS))

void inverse_kinematics(const xyz_pos_t &raw);

/**
 * Calculate the highest Z position where the
 * effector has the full range of XY motion.
 */
float delta_safe_distance_from_top();

void refresh_delta_clip_start_height();

/**
 * Delta Forward Kinematics
 *
 * See the Wikipedia article "Trilateration"
 * https://en.wikipedia.org/wiki/Trilateration
 *
 * Establish a new coordinate system in the plane of the
 * three carriage points. This system has its origin at
 * tower1, with tower2 on the X axis. Tower3 is in the X-Y
 * plane with a Z component of zero.
 * We will define unit vectors in this coordinate system
 * in our original coordinate system. Then when we calculate
 * the Xnew, Ynew and Znew values, we can translate back into
 * the original system by moving along those unit vectors
 * by the corresponding values.
 *
 * Variable names matched to Marlin, c-version, and avoid the
 * use of any vector library.
 *
 * by Andreas Hardtung 2016-06-07
 * based on a Java function from "Delta Robot Kinematics V3"
 * by Steve Graves
 *
 * The result is stored in the cartes[] array.
 */
void forward_kinematics(const_float_t z1, const_float_t z2, const_float_t z3);

FORCE_INLINE void forward_kinematics(const abc_float_t &point) {
  forward_kinematics(point.a, point.b, point.c);
}

void home_delta();
