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

#if ENABLED(ADVANCED_PAUSE_FEATURE)

#include "../../gcode.h"
#include "../../../feature/pause.h"
#include "../../../module/motion.h"
#include "../../../module/printcounter.h"

#include "../../../lcd/marlinui.h"
#if ENABLED(SOVOL_SV06_RTS)
  #include "../../../lcd/sovol_rts/sovol_rts.h"
#endif

#if HAS_MULTI_EXTRUDER
  #include "../../../module/tool_change.h"
#endif

#if HAS_PRUSA_MMU3
  #include "../../../feature/mmu3/mmu3.h"
  #if ENABLED(MMU_MENUS)
    #include "../../../lcd/menu/menu_mmu2.h"
  #endif
#elif HAS_PRUSA_MMU2
  #include "../../../feature/mmu/mmu2.h"
  #if ENABLED(MMU_MENUS)
    #include "../../../lcd/menu/menu_mmu2.h"
  #endif
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../../../feature/mixing.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

/**
 * M600: Pause for filament change
 *
 *  E[distance] - Retract the filament this far
 *  Z[distance] - Move the Z axis by this distance
 *  X[position] - Move to this X position (instead of NOZZLE_PARK_POINT.x)
 *  Y[position] - Move to this Y position (instead of NOZZLE_PARK_POINT.y)
 *  I[position] - Move to this I position (instead of NOZZLE_PARK_POINT.i)
 *  J[position] - Move to this J position (instead of NOZZLE_PARK_POINT.j)
 *  K[position] - Move to this K position (instead of NOZZLE_PARK_POINT.k)
 *  C[position] - Move to this U position (instead of NOZZLE_PARK_POINT.u)
 *  H[position] - Move to this V position (instead of NOZZLE_PARK_POINT.v)
 *  O[position] - Move to this W position (instead of NOZZLE_PARK_POINT.w)
 *  U[distance] - Retract distance for removal (manual reload)
 *  L[distance] - Extrude distance for insertion (manual reload)
 *  B[count]    - Number of times to beep, -1 for indefinite (if equipped with a buzzer)
 *  T[toolhead] - Select extruder for filament change
 *  R[temp]     - Resume temperature (in current units)
 *
 * With MMU_MENUS:
 *  A           - Automatic
 *
 *  Default values are used for omitted arguments.
 */
void GcodeSuite::M600() {

  #if ENABLED(MIXING_EXTRUDER)
    const int8_t eindex = get_target_e_stepper_from_command();
    if (eindex < 0) return;

    const uint8_t old_mixing_tool = mixer.get_current_vtool();
    mixer.T(MIXER_DIRECT_SET_TOOL);

    MIXER_STEPPER_LOOP(i) mixer.set_collector(i, i == uint8_t(eindex) ? 1.0 : 0.0);
    mixer.normalize();

    const int8_t target_extruder = active_extruder;
  #else
    const int8_t target_extruder = get_target_extruder_from_command();
    if (target_extruder < 0) return;
  #endif

  #if ENABLED(DUAL_X_CARRIAGE)
    int8_t DXC_ext = target_extruder;
    if (!parser.seen_test('T')) {  // If no tool index is specified, M600 was (probably) sent in response to filament runout.
                                   // In this case, for duplicating modes set DXC_ext to the extruder that ran out.
      #if MULTI_FILAMENT_SENSOR
        if (idex_is_duplicating())
          DXC_ext = (READ(FIL_RUNOUT2_PIN) == FIL_RUNOUT2_STATE) ? 1 : 0;
      #else
        DXC_ext = active_extruder;
      #endif
    }
  #endif

  const bool standardM600 = TERN1(MMU_MENUS, TERN1(HAS_PRUSA_MMU2, !mmu2.enabled()) && TERN1(HAS_PRUSA_MMU3, !mmu3.mmu_hw_enabled));

  // Show initial "wait for start" message
  if (standardM600)
    ui.pause_show_message(PAUSE_MESSAGE_CHANGING, PAUSE_MODE_PAUSE_PRINT, target_extruder);

  TERN_(SOVOL_SV06_RTS, rts.gotoPage(ID_ChangeWait_L, ID_ChangeWait_D)); //given the context it seems this likely should have been pages 6 & 61

  // If needed, home before parking for filament change
  TERN_(HOME_BEFORE_FILAMENT_CHANGE, home_if_needed(true));

  #if HAS_MULTI_EXTRUDER
    // Change toolhead if specified
    const uint8_t active_extruder_before_filament_change = active_extruder;
    if (active_extruder != target_extruder && TERN1(DUAL_X_CARRIAGE, !idex_is_duplicating()))
      tool_change(target_extruder);
  #endif

  // Initial retract before move to filament change position
  const float retract = -ABS(parser.axisunitsval('E', E_AXIS, PAUSE_PARK_RETRACT_LENGTH));

  xyz_pos_t park_point NOZZLE_PARK_POINT;

  // Move XY axes to filament change position or given position
  NUM_AXIS_CODE(
    if (parser.seenval('X')) park_point.x = parser.linearval('X'),
    if (parser.seenval('Y')) park_point.y = parser.linearval('Y'),
    if (parser.seenval('Z')) park_point.z = parser.linearval('Z'),    // Lift Z axis
    if (parser.seenval('I')) park_point.i = parser.linearval('I'),
    if (parser.seenval('J')) park_point.j = parser.linearval('J'),
    if (parser.seenval('K')) park_point.k = parser.linearval('K'),
    if (parser.seenval('C')) park_point.u = parser.linearval('C'),    // U axis
    if (parser.seenval('H')) park_point.v = parser.linearval('H'),    // V axis
    if (parser.seenval('O')) park_point.w = parser.linearval('O')     // W axis
  );

  #if HAS_HOTEND_OFFSET && NONE(DUAL_X_CARRIAGE, DELTA)
    park_point += hotend_offset[active_extruder];
  #endif

  // Unload filament
  // For MMU2, when enabled, reset retract value so it doesn't mess with MMU filament handling
  const float unload_length = standardM600 ? -ABS(parser.axisunitsval('U', E_AXIS, fc_settings[active_extruder].unload_length)) : 0.5f;

  const int beep_count = parser.intval('B', -1
    #ifdef FILAMENT_CHANGE_ALERT_BEEPS
      + 1 + FILAMENT_CHANGE_ALERT_BEEPS
    #endif
  );

  if (pause_print(retract, park_point, true, unload_length DXC_PASS)) {
    if (standardM600) {
      wait_for_confirmation(true, beep_count DXC_PASS);
      resume_print(
        FILAMENT_CHANGE_SLOW_LOAD_LENGTH,
        ABS(parser.axisunitsval('L', E_AXIS, fc_settings[active_extruder].load_length)),
        ADVANCED_PAUSE_PURGE_LENGTH,
        beep_count,
        parser.celsiusval('R'),
        true,
        false
        DXC_PASS
      );
    }
    else {
      #if ENABLED(MMU_MENUS)
        const bool automatic = parser.seen_test('A');
        mmu2_M600(automatic);
        resume_print(0, 0, 0, beep_count, 0, !automatic, false DXC_PASS);
      #endif
    }
  }

  #if HAS_MULTI_EXTRUDER
    // Restore toolhead if it was changed
    if (active_extruder_before_filament_change != active_extruder)
      tool_change(active_extruder_before_filament_change);
  #endif

  TERN_(MIXING_EXTRUDER, mixer.T(old_mixing_tool)); // Restore original mixing tool
}

#endif // ADVANCED_PAUSE_FEATURE
