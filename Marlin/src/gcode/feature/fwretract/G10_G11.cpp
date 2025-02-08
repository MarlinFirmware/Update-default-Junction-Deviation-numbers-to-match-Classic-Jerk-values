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

#include "../../../inc/MarlinConfig.h"

#if ANY(FWRETRACT, CNC_COORDINATE_SYSTEMS, HAS_TOOL_LENGTH_COMPENSATION)
  #include "../../gcode.h"
  #include "../../../module/motion.h"

  #if ENABLED(FWRETRACT)
    #include "../../../feature/fwretract.h"
  #endif


/**
 * G10
 *     S#                    - Retract filament according to settings of M207
 *     L1 P# X# Y# Z#...     - Sets the tool length of the tool specified by the P parameter (0 to (TOOLS - 1))
 *     L10 P# Z#             - Set the tool length of the tool specified by the P parameter (0 to (TOOLS - 1)) so that if the tool offset is reloaded, with the machine in its 
 *                             current position and with the current G5x and G52/G92 offsets active, the current coordinates for the given axes 
 *                             will become the given Z value.
 *     L2/L20 P# X# Y# Z#... - Sets the origin of the coordinate system, specified by the P parameter (1 to 9)
 *     L2/L20 P0 X# Y# Z#... - Sets the origin of the current coordinate system
 *
 *
 * Beta L2/L20 implementation based on https://github.com/MarlinFirmware/Marlin/issues/14734
 * and http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G10-L1_
 *
 * !Beta - Use with caution.
 */

void GcodeSuite::G10() {
  #if ENABLED(FWRETRACT)
    fwretract.retract(true E_OPTARG(parser.boolval('S')));
  #endif

  #if ENABLED(CNC_COORDINATE_SYSTEMS)
    uint8_t index,
            offset_type;

    if (parser.seenval('P'))
      index = parser.value_byte();
    else
      return;
    if (parser.seenval('L'))
      offset_type = parser.value_byte();
    else
      return;

    #if HAS_HOTEND_OFFSET && HAS_TOOL_LENGTH_COMPENSATION
      if ((offset_type == 1 || offset_type == 10) && WITHIN(index, 1, TOOLS)) {
        switch (offset_type) {

          default: break;                                              // Ignore unknown G10 Lx

          case 1: // Sets the tool offset. Intended to work as in LinuxCNC.
            LOOP_NUM_AXES(i) {
              if (parser.seen(axis_codes[i])) {
                const float axis_value = parser.value_axis_units((AxisEnum)i);
                if (i == Z_AXIS) {
                  hotend_offset[index][i] = -axis_value;
                }
                else {
                  hotend_offset[index][i] = axis_value;
                }
              }
            }
            break;

          // G10 L10 changes the tool table entry for tool P so that if the tool offset is reloaded, with the machine in its 
          // current position and with the current G5x and G52/G92 offsets active, the current coordinates for the given axes 
          // will become the given values. Intended to work as in LinuxCNC.
          // e.g. G10 L10 P1 Z1.5 followed by G43 sets the current position for Z to be 1.5.
          case 10:
            LOOP_NUM_AXES(i) {
              if (parser.seen(axis_codes[i])) {
                const float axis_value = parser.value_axis_units((AxisEnum)i);
                hotend_offset[index][i] = current_position[i] - axis_value;
              }
            }
            break;
        }
      }
    #endif
    if (offset_type == 2 || offset_type == 20) {
      const int8_t target_system = (index == 0) ? gcode.active_coordinate_system : (index - 1);  // P0 selects current coordinate system. P1 is G54, which is Marlin coordinate_system 0 
      const int8_t current_system = gcode.active_coordinate_system; // Store current coord system
      if (WITHIN(target_system, 0, MAX_COORDINATE_SYSTEMS - 1)) {
        if (current_system != target_system) {
          gcode.select_coordinate_system(target_system); // Select new coordinate system if needed
          #if ENABLED(DEBUG_G10)
            SERIAL_ECHOLNPGM("Switching to workspace ", target_system);
            report_current_position();
          #endif
        }
      }
      switch (offset_type) {
        default: break;                                              // Ignore unknown G10 Lx

        // Sets the work offsets of the specified (P[1-9]) coordinate system, by subtracting the specified X, Y, Z... values
        // from the current machine coordinate X, Y, Z... values (Works the same as LinuxCNC)
        // eg: If G53's X=50, Y=100 then G10 P1 L2 X10 Y2 will set G54's X=40, Y=98
        case 2:
          LOOP_LOGICAL_AXES(i) {
            if (parser.seenval(axis_codes[i])) {
              if (TERN1(HAS_EXTRUDERS, i != E_AXIS)) {
                const float axis_shift = parser.value_axis_units((AxisEnum)i);
                workspace_offset[i] = -axis_shift;
              }
            }
          }
          break;

        // Sets the work coordinates of the specified (P[1-9]) coordinate system, by matching the specified X, Y, Z... values
        // Works similar to G92, except you can specifiy a coordinate system directly (Works the same as LinuxCNC)
        // eg: G10 P2 L20 X10 Y50 will set G55's X=10, Y=50 no matter which coordinate system is currently selected
        case 20:
          LOOP_LOGICAL_AXES(i) {
            if (parser.seenval(axis_codes[i])) {
              if (TERN1(HAS_EXTRUDERS, i != E_AXIS)) {
                const float axis_value = parser.value_axis_units((AxisEnum)i);
                workspace_offset[i] = axis_value - current_position[i];
              }
            }
          }
          break;
      }
      if (WITHIN(active_coordinate_system, 0, MAX_COORDINATE_SYSTEMS - 1)) {
        coordinate_system[active_coordinate_system] = workspace_offset;
      }
      #if ENABLED(DEBUG_G10)
        SERIAL_ECHOLNPGM("New position for workspace ", target_system);
        report_current_position();
      #endif
      if (current_system != target_system) {
        gcode.select_coordinate_system(current_system);
      }
    }
  #endif // ENABLED(CNC_COORDINATE_SYSTEMS)
}

#if ENABLED(FWRETRACT)

/**
 * G11 - Recover filament according to settings of M208
 */
void GcodeSuite::G11() { fwretract.retract(false); }

#endif // FWRETRACT

#endif   // ANY(FWRETRACT, CNC_COORDINATE_SYSTEMS, HAS_TOOL_LENGTH_COMPENSATION)