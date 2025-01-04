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
 * @file G43_G49.cpp
 * @author DerAndere
 * @brief G-codes related to Tool Length Compensation and Rotational Tool Center Point Control
 * 
 * Copyright 2024 DerAndere
 */

#include "../../inc/MarlinConfig.h"

#if HAS_TOOL_LENGTH_COMPENSATION

#include "../gcode.h"
#include "../../module/motion.h"

/**
 * G43: Enable Tool Length Compensation.
 * 
 * G43: Enable Simple Tool Length Compensation.
 * G43 Tool Length Compensation can be canceled with G49.
 *
 * G43.4: Enable Rotational Tool Center Point Control Mode.
 * G43.4 Rotational Tool Center Point Control Mode can be canceled with G49.
 * 
 * Only one can be active at any time.
 */
void GcodeSuite::G43() {

  #if USE_GCODE_SUBCODES
    const uint8_t subcode_G43 = parser.subcode;
  #else
    constexpr uint8_t subcode_G43 = 0;                           
  #endif

  switch (subcode_G43) {
    default: return;                                              // Ignore unknown G43.x

    case 0:                                                       // G43 - Simple Tool Length Compensation Mode.
      #if HAS_TOOL_CENTERPOINT_CONTROL
        tool_centerpoint_control = false;
      #endif
      simple_tool_length_compensation = true;
      break;

    #if HAS_TOOL_CENTERPOINT_CONTROL
      case 4:                                                     // G43.4 - Rotational Tool Center Point Control Mode.
        simple_tool_length_compensation = false;
        tool_centerpoint_control = true;
        break;
    #endif
  }

  current_position += hotend_offset[active_extruder];
  sync_plan_position();
}


/**
 * G49: Cancel Tool Length Compensation
 * 
 * Cancels Simple Tool Length Compensation Mode and Rotational Tool Center Point Control Mode.
 * 
 * Simple Tool Length Compensation Mode can be enabled with G43
 * Rotational Tool Center Point Control Mode can be enabled with G43.4
 */
void GcodeSuite::G49() {
  simple_tool_length_compensation = false;
  #if HAS_TOOL_CENTERPOINT_CONTROL
    tool_centerpoint_control = false;
  #endif

  current_position -= hotend_offset[active_extruder];
  sync_plan_position();
}

#endif
