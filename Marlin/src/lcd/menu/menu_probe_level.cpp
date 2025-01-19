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
// Probe and Level (Calibrate?) Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU && ANY(HAS_LEVELING, HAS_BED_PROBE, ASSISTED_TRAMMING_WIZARD, LCD_BED_TRAMMING)

#include "menu_item.h"

#include "../../feature/bedlevel/bedlevel.h"

#if HAS_LEVELING
  #include "../../module/planner.h" // for leveling_active, z_fade_height
#endif

#if HAS_BED_PROBE
  #include "../../module/probe.h"
#endif

#if ENABLED(BABYSTEP_ZPROBE_OFFSET)
  #include "../../feature/babystep.h"
#endif

#if ALL(TOUCH_SCREEN, HAS_GRAPHICAL_TFT)
  #include "../tft/tft.h"
  #include "../tft/touch.h"
#endif

#if ENABLED(LCD_BED_LEVELING) && ANY(PROBE_MANUALLY, MESH_BED_LEVELING)

  #include "../../module/motion.h"
  #include "../../gcode/queue.h"

  //
  // Motion > Level Bed handlers
  //

  // LCD probed points are from defaults
  constexpr grid_count_t total_probe_points = TERN(AUTO_BED_LEVELING_3POINT, 3, GRID_MAX_POINTS);

  //
  // Bed leveling is done. Wait for G29 to complete.
  // A flag is used so that this can release control
  // and allow the command queue to be processed.
  //
  // When G29 finishes the last move:
  // - Raise Z to the "Z after probing" height
  // - Don't return until done.
  //
  // ** This blocks the command queue! **
  //
  void _lcd_level_bed_done() {
    if (!ui.wait_for_move) {
      #if DISABLED(MESH_BED_LEVELING) && defined(Z_AFTER_PROBING)
        if (Z_AFTER_PROBING) {
          // Display "Done" screen and wait for moves to complete
          line_to_z(Z_AFTER_PROBING);
          ui.synchronize(GET_TEXT_F(MSG_LEVEL_BED_DONE));
        }
      #endif
      ui.goto_previous_screen_no_defer();
      ui.completion_feedback();
    }
    if (ui.should_draw()) MenuItem_static::draw(LCD_HEIGHT >= 4, GET_TEXT_F(MSG_LEVEL_BED_DONE));
    ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
  }

  void _lcd_level_goto_next_point();

  //
  // Step 7: Get the Z coordinate, click goes to the next point or exits
  //
  void _lcd_level_bed_get_z() {

    if (ui.use_click()) {

      //
      // Save the current Z position and move
      //

      // If done...
      if (++manual_probe_index >= total_probe_points) {
        //
        // The last G29 records the point and enables bed leveling
        //
        ui.wait_for_move = true;
        ui.goto_screen(_lcd_level_bed_done);
        #if ENABLED(MESH_BED_LEVELING)
          queue.inject(F("G29S2"));
        #elif ENABLED(PROBE_MANUALLY)
          queue.inject(F("G29V1"));
        #endif
      }
      else
        _lcd_level_goto_next_point();

      return;
    }

    //
    // Encoder knob or keypad buttons adjust the Z position
    //
    if (ui.encoderPosition) {
      const float z = current_position.z + float(int32_t(ui.encoderPosition)) * (MESH_EDIT_Z_STEP);
      line_to_z(constrain(z, -(LCD_PROBE_Z_RANGE) * 0.5f, (LCD_PROBE_Z_RANGE) * 0.5f));
      ui.refresh(LCDVIEW_CALL_REDRAW_NEXT);
      ui.encoderPosition = 0;
    }

    //
    // Draw on first display, then only on Z change
    //
    if (ui.should_draw()) {
      const float v = current_position.z;
      MenuEditItemBase::draw_edit_screen(GET_TEXT_F(MSG_MOVE_Z), ftostr43sign(v + (v < 0 ? -0.0001f : 0.0001f), '+'));
    }
  }

  //
  // Step 6: Display "Next point: 1 / 9" while waiting for move to finish
  //
  void _lcd_level_bed_moving() {
    if (ui.should_draw()) {
      MString<10> msg;
      msg.setf(F(" %i / %u"), int(manual_probe_index + 1), total_probe_points);
      MenuItem_static::draw(LCD_HEIGHT / 2, GET_TEXT_F(MSG_LEVEL_BED_NEXT_POINT), SS_CENTER, msg);
    }
    ui.refresh(LCDVIEW_CALL_NO_REDRAW);
    if (!ui.wait_for_move) ui.goto_screen(_lcd_level_bed_get_z);
  }

  //
  // Step 5: Initiate a move to the next point
  //
  void _lcd_level_goto_next_point() {
    ui.goto_screen(_lcd_level_bed_moving);

    // G29 Records Z, moves, and signals when it pauses
    ui.wait_for_move = true;
    #if ENABLED(MESH_BED_LEVELING)
      queue.inject(manual_probe_index ? F("G29S2") : F("G29S1"));
    #elif ENABLED(PROBE_MANUALLY)
      queue.inject(F("G29V1"));
    #endif
  }

  //
  // Step 4: Display "Click to Begin", wait for click
  //         Move to the first probe position
  //
  void _lcd_level_bed_homing_done() {
    if (ui.should_draw()) {
      MenuItem_static::draw(1, GET_TEXT_F(MSG_LEVEL_BED_WAITING));
      // Color UI needs a control to detect a touch
      #if ALL(TOUCH_SCREEN, HAS_GRAPHICAL_TFT)
        touch.add_control(CLICK, 0, 0, TFT_WIDTH, TFT_HEIGHT);
      #endif
    }
    if (ui.use_click()) {
      manual_probe_index = 0;
      _lcd_level_goto_next_point();
    }
  }

  //
  // Step 3: Display "Homing XYZ" - Wait for homing to finish
  //
  void _lcd_level_bed_homing() {
    _lcd_draw_homing();
    if (all_axes_homed()) ui.goto_screen(_lcd_level_bed_homing_done);
  }

  #if ENABLED(PROBE_MANUALLY)
    extern bool g29_in_progress;
  #endif

  //
  // Step 2: Continue Bed Leveling...
  //
  void _lcd_level_bed_continue() {
    ui.defer_status_screen();
    set_all_unhomed();
    ui.goto_screen(_lcd_level_bed_homing);
    queue.inject_P(G28_STR);
  }

#endif // LCD_BED_LEVELING && (PROBE_MANUALLY || MESH_BED_LEVELING)

#if ENABLED(MESH_EDIT_MENU)

  inline void refresh_planner() {
    set_current_from_steppers_for_axis(ALL_AXES_ENUM);
    sync_plan_position();
  }

  void menu_edit_mesh() {
    static uint8_t xind, yind; // =0
    START_MENU();
    BACK_ITEM(MSG_BED_LEVELING);
    EDIT_ITEM(uint8, MSG_MESH_X, &xind, 0, (GRID_MAX_POINTS_X) - 1);
    EDIT_ITEM(uint8, MSG_MESH_Y, &yind, 0, (GRID_MAX_POINTS_Y) - 1);
    EDIT_ITEM_FAST(float43, MSG_MESH_EDIT_Z, &bedlevel.z_values[xind][yind], -(LCD_PROBE_Z_RANGE) * 0.5, (LCD_PROBE_Z_RANGE) * 0.5, refresh_planner);
    END_MENU();
  }

#endif // MESH_EDIT_MENU

#if ENABLED(AUTO_BED_LEVELING_UBL)
  void _lcd_ubl_level_bed();
#endif

#if ENABLED(ASSISTED_TRAMMING_WIZARD)
  void goto_tramming_wizard();
#endif

// Include a sub-menu when there's manual probing

void menu_probe_level() {
  const bool can_babystep_z = TERN0(BABYSTEP_ZPROBE_OFFSET, babystep.can_babystep(Z_AXIS));

  #if HAS_LEVELING
    const bool is_homed = all_axes_homed(),
               is_valid = leveling_is_valid();
  #endif

  #if NONE(PROBE_MANUALLY, MESH_BED_LEVELING)
    const bool is_trusted = all_axes_trusted();
  #endif

  START_MENU();

  //
  // ^ Main
  //
  BACK_ITEM(MSG_MAIN_MENU);

  if (!g29_in_progress) {

    // Auto Home if not using manual probing
    #if NONE(PROBE_MANUALLY, MESH_BED_LEVELING)
      if (!is_trusted) GCODES_ITEM(MSG_AUTO_HOME, FPSTR(G28_STR));
    #endif

    #if HAS_LEVELING

      // Homed and leveling is valid? Then leveling can be toggled.
      if (is_homed && is_valid) {
        bool show_state = planner.leveling_active;
        EDIT_ITEM(bool, MSG_BED_LEVELING, &show_state, _lcd_toggle_bed_leveling);
      }

      //
      // Level Bed
      //
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        // UBL uses a guided procedure
        SUBMENU(MSG_UBL_LEVELING, _lcd_ubl_level_bed);
      #elif ANY(PROBE_MANUALLY, MESH_BED_LEVELING)
        #if ENABLED(LCD_BED_LEVELING)
          // Manual leveling uses a guided procedure
          SUBMENU(MSG_LEVEL_BED, _lcd_level_bed_continue);
        #endif
      #else
        // Automatic leveling can just run the G-code
        GCODES_ITEM(MSG_LEVEL_BED, is_homed ? F("G29") : F("G29N"));
      #endif

      //
      // Edit Mesh (non-UBL)
      //
      #if ENABLED(MESH_EDIT_MENU)
        if (is_valid) SUBMENU(MSG_EDIT_MESH, menu_edit_mesh);
      #endif

      //
      // Mesh Bed Leveling Z-Offset
      //
      #if ENABLED(MESH_BED_LEVELING)
        #if WITHIN(PROBE_OFFSET_ZMIN, -9, 9)
          #define LCD_Z_OFFSET_TYPE float43    // Values from -9.000 to +9.000
        #else
          #define LCD_Z_OFFSET_TYPE float42_52 // Values from -99.99 to 99.99
        #endif
        EDIT_ITEM(LCD_Z_OFFSET_TYPE, MSG_MESH_Z_OFFSET, &bedlevel.z_offset, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX);
      #endif

    #endif

  } // no G29 in progress

  // Z Fade Height
  #if ENABLED(ENABLE_LEVELING_FADE_HEIGHT) && DISABLED(SLIM_LCD_MENUS)
    // Shadow for editing the fade height
    editable.decimal = planner.z_fade_height;
    EDIT_ITEM_FAST(float3, MSG_Z_FADE_HEIGHT, &editable.decimal, 0, 100, []{ set_z_fade_height(editable.decimal); });
  #endif

  if (!g29_in_progress) {
    //
    // Probe Deploy/Stow
    //
    #if ENABLED(PROBE_DEPLOY_STOW_MENU)
      GCODES_ITEM(MSG_MANUAL_DEPLOY, F("M401"));
      GCODES_ITEM(MSG_MANUAL_STOW, F("M402"));
    #endif

    // Tare the probe on-demand
    #if ENABLED(PROBE_TARE_MENU)
      ACTION_ITEM(MSG_TARE_PROBE, probe.tare);
    #endif

    //
    // Probe XY Offsets
    //
    #if HAS_PROBE_XY_OFFSET
      EDIT_ITEM_N(float31sign, X_AXIS, MSG_ZPROBE_OFFSET_N, &probe.offset.x, PROBE_OFFSET_XMIN, PROBE_OFFSET_XMAX);
      EDIT_ITEM_N(float31sign, Y_AXIS, MSG_ZPROBE_OFFSET_N, &probe.offset.y, PROBE_OFFSET_YMIN, PROBE_OFFSET_YMAX);
    #endif

    //
    // Probe Z Offset - Babystep or Edit
    //
    if (can_babystep_z) {
      #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
        SUBMENU(MSG_BABYSTEP_PROBE_Z, lcd_babystep_zoffset);
      #endif
    }
    else {
      #if HAS_BED_PROBE
        EDIT_ITEM_N(LCD_Z_OFFSET_TYPE, Z_AXIS, MSG_ZPROBE_OFFSET_N, &probe.offset.z, PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX);
      #endif
    }

    //
    // Probe Z Offset Wizard
    //
    #if ENABLED(PROBE_OFFSET_WIZARD)
      SUBMENU(MSG_PROBE_WIZARD, goto_probe_offset_wizard);
    #endif

    //
    // Probe Repeatability Test
    //
    #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
      GCODES_ITEM(MSG_M48_TEST, F("G28O\nM48 P10"));
    #endif

    //
    // Assisted Bed Tramming
    //
    #if ENABLED(ASSISTED_TRAMMING_WIZARD)
      SUBMENU(MSG_TRAMMING_WIZARD, goto_tramming_wizard);
    #endif

    //
    // Bed Tramming Submenu
    //
    #if ENABLED(LCD_BED_TRAMMING)
      SUBMENU(MSG_BED_TRAMMING, _lcd_bed_tramming);
    #endif

    //
    // Auto Z-Align
    //
    #if ANY(Z_STEPPER_AUTO_ALIGN, MECHANICAL_GANTRY_CALIBRATION)
      GCODES_ITEM(MSG_AUTO_Z_ALIGN, F("G34"));
    #endif

    //
    // Twist Compensation
    //
    #if ENABLED(X_AXIS_TWIST_COMPENSATION)
      SUBMENU(MSG_XATC, xatc_wizard_continue);
    #endif

    //
    // Store to EEPROM
    //
    #if ENABLED(EEPROM_SETTINGS)
      ACTION_ITEM(MSG_STORE_EEPROM, ui.store_settings);
    #endif

  }

  END_MENU();
}

#endif // HAS_MARLINUI_MENU && (HAS_LEVELING || HAS_BED_PROBE || ASSISTED_TRAMMING_WIZARD || LCD_BED_TRAMMING)
