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

#include "../../core/types.h"

class Kinematics {

  public:
    static xyz_pos_t mechanical_to_machine(xyz_pos_t &mechanical);
    static xyz_pos_t machine_to_gcode(xyz_pos_t &machine);
    static xyz_pos_t gcode_to_machine(xyz_pos_t &gcode);
    static xyz_pos_t machine_to_mechanical(xyz_pos_t &machine);
    static xyz_pos_t gcode_to_mechanical(xyz_pos_t &gcode);
    static xyz_pos_t mechanical_to_gcode(xyz_pos_t &mechanical);
    static void set_cache_invalid(); // Called when a gcode has been executed which invalidates the previously-cached aggregrate transforms

};
