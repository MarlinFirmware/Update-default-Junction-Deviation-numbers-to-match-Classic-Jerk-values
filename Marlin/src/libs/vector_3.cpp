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
 * vector_3.cpp - Vector library for bed leveling
 * Copyright (c) 2012 Lars Brubaker.  All right reserved.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "../inc/MarlinConfig.h"

#if ABL_PLANAR || ENABLED(AUTO_BED_LEVELING_UBL)

#include "vector_3.h"

#include <math.h>

/**
 *  vector_3
 */

vector_3 vector_3::cross(const vector_3 &left, const vector_3 &right) {
  return vector_3(left.y * right.z - left.z * right.y,  // YZ cross
                  left.z * right.x - left.x * right.z,  // ZX cross
                  left.x * right.y - left.y * right.x); // XY cross
}

vector_3 vector_3::get_normal() const {
  vector_3 normalized = *this;
  normalized.normalize();
  return normalized;
}

float vector_3::magnitude() const { return SQRT(sq(x) + sq(y) + sq(z)); }

void vector_3::normalize() { *this *= RSQRT(sq(x) + sq(y) + sq(z)); }

// Apply a rotation to the matrix
void vector_3::apply_rotation(const matrix_3x3 &matrix) {
  const float _x = x, _y = y, _z = z;
  *this = { matrix.vectors[0].x * _x + matrix.vectors[1].x * _y + matrix.vectors[2].x * _z,
            matrix.vectors[0].y * _x + matrix.vectors[1].y * _y + matrix.vectors[2].y * _z,
            matrix.vectors[0].z * _x + matrix.vectors[1].z * _y + matrix.vectors[2].z * _z };
}

void vector_3::debug(FSTR_P const title) {
  SERIAL_ECHOLN(title,
    FPSTR(SP_X_STR), p_float_t(x, 6),
    FPSTR(SP_Y_STR), p_float_t(y, 6),
    FPSTR(SP_Z_STR), p_float_t(z, 6)
  );
}

/**
 *  matrix_3x3
 */

void matrix_3x3::apply_rotation_xyz(float &_x, float &_y, float &_z) {
  vector_3 vec = vector_3(_x, _y, _z); vec.apply_rotation(*this);
  _x = vec.x; _y = vec.y; _z = vec.z;
}

// Reset to identity. No rotate or translate.
void matrix_3x3::set_to_identity() {
  for (uint_fast8_t i = 0; i < 3; ++i)
    for (uint_fast8_t j = 0; j < 3; ++j)
      vectors[i][j] = float(i == j);
}

// Create a matrix from 3 vector_3 inputs
matrix_3x3 matrix_3x3::create_from_rows(const vector_3 &row_0, const vector_3 &row_1, const vector_3 &row_2) {
  //row_0.debug(F("row_0"));
  //row_1.debug(F("row_1"));
  //row_2.debug(F("row_2"));
  matrix_3x3 new_matrix;
  new_matrix.vectors[0] = row_0;
  new_matrix.vectors[1] = row_1;
  new_matrix.vectors[2] = row_2;
  //new_matrix.debug(F("new_matrix"));
  return new_matrix;
}

// Create a matrix rotated to point towards a target
matrix_3x3 matrix_3x3::create_look_at(const vector_3 &target) {
  const vector_3 z_row = target.get_normal(),
                 x_row = vector_3(1, 0, -target.x / target.z).get_normal(),
                 y_row = vector_3::cross(z_row, x_row).get_normal();

  // x_row.debug(F("x_row"));
  // y_row.debug(F("y_row"));
  // z_row.debug(F("z_row"));

  // create the matrix already correctly transposed
  matrix_3x3 rot = matrix_3x3::create_from_rows(x_row, y_row, z_row);

  // rot.debug(F("rot"));
  return rot;
}

// Get a transposed copy of the matrix
matrix_3x3 matrix_3x3::transpose(const matrix_3x3 &original) {
  matrix_3x3 new_matrix;
  for (uint_fast8_t i = 0; i < 3; ++i)
    for (uint_fast8_t j = 0; j < 3; ++j)
      new_matrix.vectors[i][j] = original.vectors[j][i];
  return new_matrix;
}

void matrix_3x3::debug(FSTR_P const title) {
  if (title) SERIAL_ECHOLN(title);
  for (uint_fast8_t i = 0; i < 3; ++i) {
    for (uint_fast8_t j = 0; j < 3; ++j) {
      serial_offset(vectors[i][j], 2);
      SERIAL_CHAR(' ');
    }
    SERIAL_EOL();
  }
}

#endif // HAS_ABL_OR_UBL
