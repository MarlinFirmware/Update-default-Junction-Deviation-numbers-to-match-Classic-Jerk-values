/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Copyright (c) 2022 Carlon LaMont
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
 * variables.h
 * Copyright (c) 2022 Carlon LaMont
 */

#include "../../inc/MarlinConfigPre.h"

class CNCVariables {
public:
  //char[] letter = { '#' }
  //string L = new_string(letter)

  static void var_from_command(const int8_t dval=0);
  static void append();
  static uint8_t StoredVar();
  static void input_var();
  static void export_var();
  static void has_val();
  static bool is_var();
  static bool used_var_arg();
  static void var_arg();

  static void L100();
  static void L101();
  static void L102();
  static void L103();
  static void L104();
  static void L105();
  static void L106();
  static void L107();
  static void L108();
  static void L109();
  static void L110();
  static void L111();
  static void L112();
  static void L113();
  static void L114();
  static void L115();
};
