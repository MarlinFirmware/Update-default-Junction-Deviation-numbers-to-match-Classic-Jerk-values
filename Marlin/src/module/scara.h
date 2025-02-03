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
 * module/scara.h - SCARA-specific functions
 */

#include "../core/macros.h"

extern float segments_per_second;

#if ENABLED(AXEL_TPARA)

  float constexpr L1 = TPARA_LINKAGE_1, L2 = TPARA_LINKAGE_2, // Float constants for Robot arm calculations
                  L1_2 = FLOAT_SQ(L1), L1_2_2 = 2.0 * L1_2,
                  L2_2 = FLOAT_SQ(L2);

  void forward_kinematics(const_float_t a, const_float_t b, const_float_t c);
  void home_TPARA();

#else

  float constexpr L1 = SCARA_LINKAGE_1, L2 = SCARA_LINKAGE_2, // Float constants for SCARA calculations
                  L1_2 = FLOAT_SQ(L1), L1_2_2 = 2.0 * L1_2,
                  L2_2 = FLOAT_SQ(L2);

  void forward_kinematics(const_float_t a, const_float_t b);

#endif

void inverse_kinematics(const xyz_pos_t &raw);
void scara_set_axis_is_at_home(const AxisEnum axis);
void scara_report_positions();
