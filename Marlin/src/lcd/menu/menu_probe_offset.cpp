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

//
// Calibrate Probe offset menu.
//

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(PROBE_OFFSET_WIZARD)

#include "menu_item.h"
#include "menu_addon.h"
#include "../../gcode/queue.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/probe.h"

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

void _goto_manual_move_z(const_float_t);

// Global storage
float z_offset_backup, calculated_z_offset, z_offset_ref;

void set_offset_and_go_back(const_float_t z) {
  probe.offset.z = z;
  SET_SOFT_ENDSTOP_LOOSE(false);
  TERN_(HAS_LEVELING, set_bed_leveling_enabled(menu_leveling_was_active));
  ui.goto_previous_screen_no_defer();
}

void probe_offset_wizard_menu() {
  START_MENU();
  calculated_z_offset = probe.offset.z + current_position.z - z_offset_ref;

  if (LCD_HEIGHT >= 4)
    STATIC_ITEM(MSG_MOVE_NOZZLE_TO_BED, SS_CENTER|SS_INVERT);

  STATIC_ITEM_F(F("Z"), SS_CENTER, ftostr42_52(current_position.z));
  STATIC_ITEM_N(Z_AXIS, MSG_ZPROBE_OFFSET_N, SS_FULL, ftostr42_52(calculated_z_offset));

  SUBMENU_S(F("1.0"), MSG_MOVE_N_MM, []{ _goto_manual_move_z( 1.0f); });
  SUBMENU_S(F("0.1"), MSG_MOVE_N_MM, []{ _goto_manual_move_z( 0.1f); });

  if ((FINE_MANUAL_MOVE) > 0.0f && (FINE_MANUAL_MOVE) < 0.1f)
    SUBMENU_f(F(STRINGIFY(FINE_MANUAL_MOVE)), MSG_MOVE_N_MM, []{ _goto_manual_move_z(float(FINE_MANUAL_MOVE)); });

  ACTION_ITEM(MSG_BUTTON_DONE, []{
    set_offset_and_go_back(calculated_z_offset);
    current_position.z = z_offset_ref;  // Set Z to z_offset_ref, as we can expect it is at probe height
    sync_plan_position();
    do_z_post_clearance();
  });

  ACTION_ITEM(MSG_BUTTON_CANCEL, []{
    set_offset_and_go_back(z_offset_backup);
    // On cancel the Z position needs correction
    #if HOMING_Z_WITH_PROBE && defined(PROBE_OFFSET_WIZARD_START_Z)
      set_axis_never_homed(Z_AXIS);
      queue.inject(F("G28Z"));
    #else
      do_z_post_clearance();
    #endif
  });

  END_MENU();
}

void prepare_for_probe_offset_wizard() {
  #if defined(PROBE_OFFSET_WIZARD_XY_POS) || !HOMING_Z_WITH_PROBE
    if (ui.should_draw()) MenuItem_static::draw(1, GET_TEXT_F(MSG_PROBE_WIZARD_PROBING));

    if (ui.wait_for_move) return;

    #ifndef PROBE_OFFSET_WIZARD_XY_POS
      #define PROBE_OFFSET_WIZARD_XY_POS XY_CENTER
    #endif
    // Get X and Y from configuration, or use center
    constexpr xy_pos_t wizard_pos = PROBE_OFFSET_WIZARD_XY_POS;

    // Probe for Z reference
    ui.wait_for_move = true;
    z_offset_ref = probe.probe_at_point(wizard_pos, PROBE_PT_RAISE);
    ui.wait_for_move = false;

    // Stow the probe, as the last call to probe.probe_at_point(...) left
    // the probe deployed if it was successful.
    probe.stow();
  #else
    if (ui.wait_for_move) return;
  #endif

  // Move Nozzle to Probing/Homing Position
  ui.wait_for_move = true;
  current_position += probe.offset_xy;
  line_to_current_position(XY_PROBE_FEEDRATE_MM_S);
  ui.synchronize(GET_TEXT_F(MSG_PROBE_WIZARD_MOVING));
  ui.wait_for_move = false;

  SET_SOFT_ENDSTOP_LOOSE(true); // Disable soft endstops for free Z movement

  // Go to Calibration Menu
  ui.goto_screen(probe_offset_wizard_menu);
  ui.defer_status_screen();
}

void goto_probe_offset_wizard() {
  ui.defer_status_screen();
  set_all_unhomed();

  // Store probe.offset.z for Case: Cancel
  z_offset_backup = probe.offset.z;

  #ifdef PROBE_OFFSET_WIZARD_START_Z
    probe.offset.z = PROBE_OFFSET_WIZARD_START_Z;
  #endif

  // Store Bed-Leveling-State and disable
  #if HAS_LEVELING
    menu_leveling_was_active = planner.leveling_active;
    set_bed_leveling_enabled(false);
  #endif

  // Home all axes
  queue.inject_P(G28_STR);

  ui.goto_screen([]{
    _lcd_draw_homing();
    if (all_axes_homed()) {
      z_offset_ref = 0;             // Set Z Value for Wizard Position to 0
      ui.goto_screen(prepare_for_probe_offset_wizard);
      ui.defer_status_screen();
    }
  });

}

#endif // PROBE_OFFSET_WIZARD
