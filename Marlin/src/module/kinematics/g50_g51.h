/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "transform_gcode.h"

/**
 * G50 takes a center and a scale, and scales subsequent G-code coordinates by that scale centered on that center
 *
 * G51 cancels scaling
 */
class G50G51 : TransformGCode {

  // This is currently just a skeleton to give an idea as to how the kinematics refactoring might be implemented

  matrix_3x3 *get_forward_kinematics();
  matrix_3x3 *get_inverse_kinematics();
  vector_3 *get_offset();

  void execute_g50();
  void execute_g51(xyz_pos_t &center, xyz_float_t &scale);

};
