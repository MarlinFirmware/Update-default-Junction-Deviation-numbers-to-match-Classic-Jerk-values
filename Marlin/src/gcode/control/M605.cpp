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

#include "../../inc/MarlinConfig.h"

#if HAS_DUPLICATION_MODE

//#define DEBUG_DXC_MODE

#include "../gcode.h"
#include "../../module/motion.h"
#include "../../module/tool_change.h"
#include "../../module/planner.h"

#define DEBUG_OUT ENABLED(DEBUG_DXC_MODE)
#include "../../core/debug_out.h"

#if ENABLED(DUAL_X_CARRIAGE)

  /**
   * M605: Set dual x-carriage movement mode
   *
   *   M605 S0 : (FULL_CONTROL) The slicer has full control over both X-carriages and can achieve optimal travel
   *             results as long as it supports dual X-carriages.
   *
   *   M605 S1 : (AUTO_PARK) The firmware automatically parks and unparks the X-carriages on tool-change so that
   *             additional slicer support is not required.
   *
   *   M605 S2 X R : (DUPLICATION) The firmware moves the second X-carriage and extruder in synchronization with
   *             the first X-carriage and extruder, to print 2 copies of the same object at the same time.
   *             Set the constant X-offset and temperature differential with M605 S2 X[offs] R[deg] and
   *             follow with "M605 S2" to initiate duplicated movement. For example, use "M605 S2 X100 R2" to
   *             make a copy 100mm to the right with E1 2° hotter than E0.
   *
   *   M605 S3 : (MIRRORED) Formbot/Vivedino-inspired mirrored mode in which the second extruder duplicates
   *             the movement of the first except the second extruder is reversed in the X axis.
   *             The temperature differential and initial X offset must be set with "M605 S2 X[offs] R[deg]",
   *             then followed by "M605 S3" to initiate mirrored movement.
   *
   *    M605 W  : IDEX What? command.
   *
   *    Note: the X axis should be homed after changing Dual X-carriage mode.
   */
  void GcodeSuite::M605() {
    planner.synchronize();

    if (parser.seenval('S')) {
      const DualXMode previous_mode = dual_x_carriage_mode;

      dual_x_carriage_mode = (DualXMode)parser.value_byte();
      idex_set_mirrored_mode(false);

      switch (dual_x_carriage_mode) {

        case DXC_FULL_CONTROL_MODE:
        case DXC_AUTO_PARK_MODE:
          break;

        case DXC_DUPLICATION_MODE:
          // Set the X offset, but no less than the safety gap
          if (parser.seenval('X')) duplicate_extruder_x_offset = _MAX(parser.value_linear_units(), (X2_MIN_POS) - (X1_MIN_POS));
          if (parser.seenval('R')) duplicate_extruder_temp_offset = parser.value_celsius_diff();
          // Always switch back to tool 0
          if (active_extruder != 0) tool_change(0);
          break;

        case DXC_MIRRORED_MODE: {
          if (previous_mode != DXC_DUPLICATION_MODE) {
            SERIAL_ECHOLNPGM("Printer must be in DXC_DUPLICATION_MODE prior to ");
            SERIAL_ECHOLNPGM("specifying DXC_MIRRORED_MODE.");
            dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
            return;
          }
          idex_set_mirrored_mode(true);

          // Do a small 'jog' motion in the X axis
          xyze_pos_t dest = current_position; dest.x -= 0.1f;
          for (uint_fast8_t i = 2; --i;) {
            planner.buffer_line(dest, feedrate_mm_s, 0);
            dest.x += 0.1f;
          }
        } return;

        default:
          dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;
          break;
      }

      idex_set_parked(false);
      set_duplication_enabled(false);

      #ifdef EVENT_GCODE_IDEX_AFTER_MODECHANGE
        process_subcommands_now(F(EVENT_GCODE_IDEX_AFTER_MODECHANGE));
      #endif
    }
    else if (!parser.seen('W'))  // if no S or W parameter, the DXC mode gets reset to the user's default
      dual_x_carriage_mode = DEFAULT_DUAL_X_CARRIAGE_MODE;

    #ifdef DEBUG_DXC_MODE

      if (parser.seen('W')) {
        DEBUG_ECHO_START();
        DEBUG_ECHOPGM("Dual X Carriage Mode ");
        switch (dual_x_carriage_mode) {
          case DXC_FULL_CONTROL_MODE: DEBUG_ECHOPGM("FULL_CONTROL"); break;
          case DXC_AUTO_PARK_MODE:    DEBUG_ECHOPGM("AUTO_PARK");    break;
          case DXC_DUPLICATION_MODE:  DEBUG_ECHOPGM("DUPLICATION");  break;
          case DXC_MIRRORED_MODE:     DEBUG_ECHOPGM("MIRRORED");     break;
        }
        DEBUG_ECHOPGM("\nActive Ext: ", active_extruder);
        if (!active_extruder_parked) DEBUG_ECHOPGM(" NOT ", F(" parked."));
        DEBUG_ECHOLNPGM(
          "\nactive_extruder_x_pos: ", current_position.x,
          "\ninactive_extruder_x: ", inactive_extruder_x,
          "\nextruder_duplication_enabled: ", extruder_duplication_enabled,
          "\nduplicate_extruder_x_offset: ", duplicate_extruder_x_offset,
          "\nduplicate_extruder_temp_offset: ", duplicate_extruder_temp_offset,
          "\ndelayed_move_time: ", delayed_move_time,
          "\nX1 Home: ", x_home_pos(0), " X1_MIN_POS=", X1_MIN_POS, " X1_MAX_POS=", X1_MAX_POS,
          "\nX2 Home: ", x_home_pos(1), " X2_MIN_POS=", X2_MIN_POS, " X2_MAX_POS=", X2_MAX_POS,
          "\nDEFAULT_DUAL_X_CARRIAGE_MODE=", STRINGIFY(DEFAULT_DUAL_X_CARRIAGE_MODE),
          "\toolchange_settings.z_raise=", toolchange_settings.z_raise,
          "\nDEFAULT_DUPLICATION_X_OFFSET=", DEFAULT_DUPLICATION_X_OFFSET
        );

        HOTEND_LOOP() {
          DEBUG_ECHOPGM_P(SP_T_STR, e);
          LOOP_NUM_AXES(a) DEBUG_ECHOPGM("  hotend_offset[", e, "].", C(AXIS_CHAR(a) | 0x20), "=", hotend_offset[e][a]);
          DEBUG_EOL();
        }
        DEBUG_EOL();
      }
    #endif // DEBUG_DXC_MODE
  }

#elif ENABLED(MULTI_NOZZLE_DUPLICATION)

  /**
   * M605: Set multi-nozzle duplication mode
   *
   *  S2       - Enable duplication mode
   *  P[mask]  - Bit-mask of nozzles to include in the duplication set.
   *             A value of 0 disables duplication.
   *  E[index] - Last nozzle index to include in the duplication set.
   *             A value of 0 disables duplication.
   */
  void GcodeSuite::M605() {
    bool ena = false;
    if (parser.seen("EPS")) {
      planner.synchronize();
      if (parser.seenval('P')) duplication_e_mask = parser.value_int();   // Set the mask directly
      else if (parser.seenval('E')) duplication_e_mask = _BV(parser.value_int() + 1) - 1; // Set the mask by E index
      ena = (2 == parser.intval('S', extruder_duplication_enabled ? 2 : 0));
      set_duplication_enabled(ena && (duplication_e_mask >= 3));
    }
    SERIAL_ECHO_START();
    SERIAL_ECHOPGM(STR_DUPLICATION_MODE);
    serialprint_onoff(extruder_duplication_enabled);
    if (ena) {
      SERIAL_ECHOPGM(" ( ");
      HOTEND_LOOP() if (TEST(duplication_e_mask, e)) { SERIAL_ECHO(e); SERIAL_CHAR(' '); }
      SERIAL_CHAR(')');
    }
    SERIAL_EOL();
  }

#endif // MULTI_NOZZLE_DUPLICATION

#endif // HAS_DUPICATION_MODE
