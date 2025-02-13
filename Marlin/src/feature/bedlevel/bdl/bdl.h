/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include <stdint.h>

#ifndef BD_SENSOR_HOME_Z_POSITION
  #define BD_SENSOR_HOME_Z_POSITION 0.5
#endif

enum BDS_State : int8_t {
  BDS_IDLE,
  BDS_VERSION         = -1,
  BDS_READ_MM         = -2,
  BDS_HOMING_Z        = -3,
  BDS_READ_RAW        = -5,
  BDS_CALIBRATE_START = -6,
  BDS_CALIBRATING     = -7,
  BDS_HOME_END        = -8
};

class BDS_Leveling {
public:
  static int config_state;
  static int sw_mode;
  static bool bd_triggered;
  static float pos_zero_offset;
  static void init(uint8_t _sda, uint8_t _scl, uint16_t delay_s);
  void process();
  static float bd_read();
  static void prepare_homing();
  static void adjust_probe_up(float up_steps);
  static float adjust_probe_down(float down_steps);
  static void end_homing();
  static bool read_endstop();
  static void read_version(int len_s);
  static float interpret(const uint16_t data);
  static float good_data(const uint16_t data) { return (data & 0x3FF) < 1016; }
  static bool check(const uint16_t data, const bool raw_data=false, const bool hicheck=false);
};

extern BDS_Leveling bdl;
