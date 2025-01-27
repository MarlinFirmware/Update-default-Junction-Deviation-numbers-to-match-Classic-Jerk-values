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
 * G29.cpp - Mesh Bed Leveling
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(MESH_BED_LEVELING)

#include "../../../feature/bedlevel/bedlevel.h"

#include "../../gcode.h"
#include "../../queue.h"

#include "../../../libs/buzzer.h"
#include "../../../lcd/marlinui.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"

#if ENABLED(EXTENSIBLE_UI)
  #include "../../../lcd/extui/ui_api.h"
#endif

#define DEBUG_OUT ENABLED(DEBUG_LEVELING_FEATURE)
#include "../../../core/debug_out.h"

#if FT_MOTION_DISABLE_FOR_PROBING
  #include "../../module/ft_motion.h"
#endif

// Save 130 bytes with non-duplication of PSTR
inline void echo_not_entered(const char c) { SERIAL_CHAR(c); SERIAL_ECHOLNPGM(" not entered."); }

/**
 * G29: Mesh-based Z probe, probes a grid and produces a
 *      mesh to compensate for variable bed height
 *
 * Parameters With MESH_BED_LEVELING:
 *
 *  S0              Report the current mesh values
 *  S1              Start probing mesh points
 *  S2              Probe the next mesh point
 *  S3 In Jn Zn.nn  Manually modify a single point
 *  S4 Zn.nn        Set z offset. Positive away from bed, negative closer to bed.
 *  S5              Reset and disable mesh
 */
void GcodeSuite::G29() {

  #if FT_MOTION_DISABLE_FOR_PROBING
    FTMotionDisableInScope FT_Disabler; // Disable Fixed-Time Motion for probing
  #endif

  DEBUG_SECTION(log_G29, "G29", true);

  // G29 Q is also available if debugging
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    const bool seenQ = parser.seen_test('Q');
    if (seenQ || DEBUGGING(LEVELING)) {
      log_machine_info();
      if (seenQ) return;
    }
  #endif

  static int mbl_probe_index = -1;

  MeshLevelingState state = (MeshLevelingState)parser.byteval('S', (int8_t)MeshReport);
  if (!WITHIN(state, 0, 5)) {
    SERIAL_ECHOLNPGM("S out of range (0-5).");
    return;
  }

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_PROBE));

  int8_t ix, iy;
  ix = iy = 0;

  switch (state) {
    case MeshReport:
      SERIAL_ECHOPGM("Mesh Bed Leveling ");
      if (leveling_is_valid()) {
        serialprintln_onoff(planner.leveling_active);
        bedlevel.report_mesh();
      }
      else
        SERIAL_ECHOLNPGM("has no data.");
      break;

    case MeshStart:
      bedlevel.reset();
      mbl_probe_index = 0;
      if (!ui.wait_for_move) {
        if (parser.seen_test('N'))
          queue.inject(F("G28" TERN_(CAN_SET_LEVELING_AFTER_G28, "L0")));

        // Position bed horizontally and Z probe vertically.
        #if HAS_SAFE_BED_LEVELING
          xyze_pos_t safe_position = current_position;
          #ifdef SAFE_BED_LEVELING_START_X
            safe_position.x = SAFE_BED_LEVELING_START_X;
          #endif
          #ifdef SAFE_BED_LEVELING_START_Y
            safe_position.y = SAFE_BED_LEVELING_START_Y;
          #endif
          #ifdef SAFE_BED_LEVELING_START_Z
            safe_position.z = SAFE_BED_LEVELING_START_Z;
          #endif
          #ifdef SAFE_BED_LEVELING_START_I
            safe_position.i = SAFE_BED_LEVELING_START_I;
          #endif
          #ifdef SAFE_BED_LEVELING_START_J
            safe_position.j = SAFE_BED_LEVELING_START_J;
          #endif
          #ifdef SAFE_BED_LEVELING_START_K
            safe_position.k = SAFE_BED_LEVELING_START_K;
          #endif
          #ifdef SAFE_BED_LEVELING_START_U
            safe_position.u = SAFE_BED_LEVELING_START_U;
          #endif
          #ifdef SAFE_BED_LEVELING_START_V
            safe_position.v = SAFE_BED_LEVELING_START_V;
          #endif
          #ifdef SAFE_BED_LEVELING_START_W
            safe_position.w = SAFE_BED_LEVELING_START_W;
          #endif

          do_blocking_move_to(safe_position);
        #endif // HAS_SAFE_BED_LEVELING

        queue.inject(F("G29S2"));

        TERN_(EXTENSIBLE_UI, ExtUI::onLevelingStart());

        return;
      }
      state = MeshNext;

    case MeshNext:
      if (mbl_probe_index < 0) {
        SERIAL_ECHOLNPGM("Start mesh probing with \"G29 S1\" first.");
        return;
      }
      // For each G29 S2...
      if (mbl_probe_index == 0) {
        // Move close to the bed before the first point
        do_blocking_move_to_z(
          #ifdef MANUAL_PROBE_START_Z
            MANUAL_PROBE_START_Z
          #else
            0.4f
          #endif
        );
      }
      else {
        // Save Z for the previous mesh position
        bedlevel.set_zigzag_z(mbl_probe_index - 1, current_position.z);
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(ix, iy, current_position.z));
        SET_SOFT_ENDSTOP_LOOSE(false);
      }
      // If there's another point to sample, move there with optional lift.
      if (mbl_probe_index < GRID_MAX_POINTS) {
        // Disable software endstops to allow manual adjustment
        // If G29 is left hanging without completion they won't be re-enabled!
        SET_SOFT_ENDSTOP_LOOSE(true);
        bedlevel.zigzag(mbl_probe_index++, ix, iy);
        _manual_goto_xy({ bedlevel.index_to_xpos[ix], bedlevel.index_to_ypos[iy] });
      }
      else {
        // Move to the after probing position
        current_position.z = (
          #ifdef Z_AFTER_PROBING
            Z_AFTER_PROBING
          #else
            Z_CLEARANCE_BETWEEN_MANUAL_PROBES
          #endif
        );
        line_to_current_position();
        planner.synchronize();

        // After recording the last point, activate home and activate
        mbl_probe_index = -1;
        SERIAL_ECHOLNPGM("Mesh probing done.");
        TERN_(HAS_STATUS_MESSAGE, LCD_MESSAGE(MSG_MESH_DONE));
        OKAY_BUZZ();

        home_all_axes();
        set_bed_leveling_enabled(true);

        #if ENABLED(MESH_G28_REST_ORIGIN)
          current_position.z = 0;
          line_to_current_position(homing_feedrate(Z_AXIS));
          planner.synchronize();
        #endif

        TERN_(LCD_BED_LEVELING, ui.wait_for_move = false);
        TERN_(EXTENSIBLE_UI, ExtUI::onLevelingDone());
      }
      break;

    case MeshSet:
      if (parser.seenval('I')) {
        ix = parser.value_int();
        if (!WITHIN(ix, 0, (GRID_MAX_POINTS_X) - 1)) {
          SERIAL_ECHOLNPGM("I out of range (0-", (GRID_MAX_POINTS_X) - 1, ")");
          return;
        }
      }
      else
        return echo_not_entered('J');

      if (parser.seenval('J')) {
        iy = parser.value_int();
        if (!WITHIN(iy, 0, (GRID_MAX_POINTS_Y) - 1)) {
          SERIAL_ECHOLNPGM("J out of range (0-", (GRID_MAX_POINTS_Y) - 1, ")");
          return;
        }
      }
      else
        return echo_not_entered('J');

      if (parser.seenval('Z')) {
        bedlevel.z_values[ix][iy] = parser.value_linear_units();
        TERN_(EXTENSIBLE_UI, ExtUI::onMeshUpdate(ix, iy, bedlevel.z_values[ix][iy]));
      }
      else
        return echo_not_entered('Z');
      break;

    case MeshSetZOffset:
      if (parser.seenval('Z'))
        bedlevel.z_offset = parser.value_linear_units();
      else
        return echo_not_entered('Z');
      break;

    case MeshReset:
      reset_bed_level();
      break;

  } // switch(state)

  if (state == MeshNext) {
    SERIAL_ECHOLNPGM("MBL G29 point ", _MIN(mbl_probe_index, GRID_MAX_POINTS), " of ", GRID_MAX_POINTS);
    if (mbl_probe_index > 0) TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT_F(MSG_PROBING_POINT), _MIN(mbl_probe_index, GRID_MAX_POINTS), int(GRID_MAX_POINTS)));
  }

  report_current_position();

  TERN_(FULL_REPORT_TO_HOST_FEATURE, set_and_report_grblstate(M_IDLE));
}

#endif // MESH_BED_LEVELING
