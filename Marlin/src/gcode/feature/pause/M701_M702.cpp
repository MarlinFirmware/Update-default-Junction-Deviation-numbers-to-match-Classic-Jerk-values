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

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)

#include "../../gcode.h"
#include "../../../MarlinCore.h"
#include "../../../module/motion.h"
#include "../../../module/temperature.h"
#include "../../../feature/pause.h"
#include "../../../lcd/marlinui.h"

#if HAS_MULTI_EXTRUDER
  #include "../../../module/tool_change.h"
#endif

#if HAS_PRUSA_MMU3
  #include "../../../feature/mmu3/mmu3.h"
#elif HAS_PRUSA_MMU2
  #include "../../../feature/mmu/mmu2.h"
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "../../../feature/mixing.h"
#endif

/**
 * M701: Load filament
 *
 *  T<extruder> - Extruder number. Required for mixing extruder.
 *                For non-mixing, current extruder if omitted.
 *  Z<distance> - Move the Z axis by this distance
 *  L<distance> - Extrude distance for insertion (positive value) (manual reload)
 *
 *  Default values are used for omitted arguments.
 */
void GcodeSuite::M701() {
  xyz_pos_t park_point = NOZZLE_PARK_POINT;

  // Don't raise Z if the machine isn't homed
  if (TERN0(NO_MOTION_BEFORE_HOMING, axes_should_home())) park_point.z = 0;

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

  // Z axis lift
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  // Show initial "wait for load" message
  ui.pause_show_message(PAUSE_MESSAGE_LOAD, PAUSE_MODE_LOAD_FILAMENT, target_extruder);

  #if HAS_MULTI_EXTRUDER && (HAS_PRUSA_MMU1 || !HAS_MMU)
    // Change toolhead if specified
    uint8_t active_extruder_before_filament_change = active_extruder;
    if (active_extruder != target_extruder)
      tool_change(target_extruder);
  #endif

  auto move_z_by = [](const_float_t zdist) {
    if (zdist) {
      destination = current_position;
      destination.z += zdist;
      prepare_internal_move_to_destination(NOZZLE_PARK_Z_FEEDRATE);
    }
  };

  // Raise the Z axis (with max limit)
  const float park_raise = _MIN(park_point.z, (Z_MAX_POS) - current_position.z);
  move_z_by(park_raise);

  // Load filament
  #if HAS_PRUSA_MMU3
    mmu3.load_to_nozzle(target_extruder);
  #elif HAS_PRUSA_MMU2
    mmu2.load_to_nozzle(target_extruder);
  #else
    constexpr float     purge_length = ADVANCED_PAUSE_PURGE_LENGTH,
                    slow_load_length = FILAMENT_CHANGE_SLOW_LOAD_LENGTH;
        const float fast_load_length = ABS(parser.seenval('L') ? parser.value_axis_units(E_AXIS)
                                                            : fc_settings[active_extruder].load_length);
    load_filament(
      slow_load_length, fast_load_length, purge_length,
      FILAMENT_CHANGE_ALERT_BEEPS,
      true,                                           // show_lcd
      thermalManager.still_heating(target_extruder),  // pause_for_user
      PAUSE_MODE_LOAD_FILAMENT                        // pause_mode
      OPTARG(DUAL_X_CARRIAGE, target_extruder)        // Dual X target
    );
  #endif

  // Restore Z axis
  move_z_by(-park_raise);

  #if HAS_MULTI_EXTRUDER && (HAS_PRUSA_MMU1 || !HAS_MMU)
    // Restore toolhead if it was changed
    if (active_extruder_before_filament_change != active_extruder)
      tool_change(active_extruder_before_filament_change);
  #endif

  TERN_(MIXING_EXTRUDER, mixer.T(old_mixing_tool)); // Restore original mixing tool

  // Show status screen
  ui.pause_show_message(PAUSE_MESSAGE_STATUS);
}

/**
 * M702: Unload filament
 *
 *  T<extruder> - Extruder number. Required for mixing extruder.
 *                For non-mixing, if omitted, current extruder
 *                (or ALL extruders with FILAMENT_UNLOAD_ALL_EXTRUDERS).
 *  Z<distance> - Move the Z axis by this distance
 *  U<distance> - Retract distance for removal (manual reload)
 *
 *  Default values are used for omitted arguments.
 */
void GcodeSuite::M702() {
  xyz_pos_t park_point = NOZZLE_PARK_POINT;

  // Don't raise Z if the machine isn't homed
  if (TERN0(NO_MOTION_BEFORE_HOMING, axes_should_home())) park_point.z = 0;

  #if ENABLED(MIXING_EXTRUDER)
    const uint8_t old_mixing_tool = mixer.get_current_vtool();

    #if ENABLED(FILAMENT_UNLOAD_ALL_EXTRUDERS)
      float mix_multiplier = 1.0;
      const bool seenT = parser.seenval('T');
      if (!seenT) {
        mixer.T(MIXER_AUTORETRACT_TOOL);
        mix_multiplier = MIXING_STEPPERS;
      }
    #else
      constexpr bool seenT = true;
    #endif

    if (seenT) {
      const int8_t eindex = get_target_e_stepper_from_command();
      if (eindex < 0) return;
      mixer.T(MIXER_DIRECT_SET_TOOL);
      MIXER_STEPPER_LOOP(i) mixer.set_collector(i, i == uint8_t(eindex) ? 1.0 : 0.0);
      mixer.normalize();
    }

    const int8_t target_extruder = active_extruder;
  #else
    const int8_t target_extruder = get_target_extruder_from_command();
    if (target_extruder < 0) return;
  #endif

  // Z axis lift
  if (parser.seenval('Z')) park_point.z = parser.linearval('Z');

  // Show initial "wait for unload" message
  ui.pause_show_message(PAUSE_MESSAGE_UNLOAD, PAUSE_MODE_UNLOAD_FILAMENT, target_extruder);

  #if HAS_MULTI_EXTRUDER && (HAS_PRUSA_MMU1 || !HAS_MMU)
    // Change toolhead if specified
    uint8_t active_extruder_before_filament_change = active_extruder;
    if (active_extruder != target_extruder)
      tool_change(target_extruder);
  #endif

  // Lift Z axis
  if (park_point.z > 0)
    do_blocking_move_to_z(_MIN(current_position.z + park_point.z, Z_MAX_POS), feedRate_t(NOZZLE_PARK_Z_FEEDRATE));

  // Unload filament
  #if HAS_PRUSA_MMU3
    mmu3.unload();
  #elif HAS_PRUSA_MMU2
    mmu2.unload();
  #else
    #if ALL(HAS_MULTI_EXTRUDER, FILAMENT_UNLOAD_ALL_EXTRUDERS)
      if (!parser.seenval('T')) {
        HOTEND_LOOP() {
          if (e != active_extruder) tool_change(e);
          unload_filament(-fc_settings[e].unload_length, true, PAUSE_MODE_UNLOAD_FILAMENT);
        }
      }
      else
    #endif
    {
      // Unload length
      const float unload_length = -ABS(parser.seen('U') ? parser.value_axis_units(E_AXIS)
                                                        : fc_settings[target_extruder].unload_length);

      unload_filament(unload_length, true, PAUSE_MODE_UNLOAD_FILAMENT
        #if ALL(FILAMENT_UNLOAD_ALL_EXTRUDERS, MIXING_EXTRUDER)
          , mix_multiplier
        #endif
      );
    }
  #endif

  // Restore Z axis
  if (park_point.z > 0)
    do_blocking_move_to_z(_MAX(current_position.z - park_point.z, 0), feedRate_t(NOZZLE_PARK_Z_FEEDRATE));

  #if HAS_MULTI_EXTRUDER && (HAS_PRUSA_MMU1 || !HAS_MMU)
    // Restore toolhead if it was changed
    if (active_extruder_before_filament_change != active_extruder)
      tool_change(active_extruder_before_filament_change);
  #endif

  TERN_(MIXING_EXTRUDER, mixer.T(old_mixing_tool)); // Restore original mixing tool

  // Show status screen
  ui.pause_show_message(PAUSE_MESSAGE_STATUS);
}

#endif // ADVANCED_PAUSE_FEATURE
