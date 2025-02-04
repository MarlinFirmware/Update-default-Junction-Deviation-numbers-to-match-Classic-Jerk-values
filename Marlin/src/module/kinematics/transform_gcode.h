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

#include "../../libs/vector_3.h"

/**
 * A transform gcode is a gcode which changes how subsequent gcode coordinates are mapped onto machine coordinates
 */
class TransformGCode {

  /**
   * Forward kinematics go in the direction of mechanical -> machine -> gcode
   */
  virtual matrix_3x3 *get_forward_kinematics() = 0;

  /**
   * Inverse kinematics go in the direction gcode -> machine -> mechanical
   */
  virtual matrix_3x3 *get_inverse_kinematics() = 0;

  /**
   * The offset is added before applying forward kinematics, or subtracted after applying inverse kinematics
   */
  virtual vector_3 *get_offset() = 0;

};
