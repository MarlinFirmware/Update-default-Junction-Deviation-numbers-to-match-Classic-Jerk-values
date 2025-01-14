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
// Tune Menu
//

#include "../../inc/MarlinConfigPre.h"

#if HAS_MARLINUI_MENU

#include "menu_item.h"
#include "../../module/motion.h"
#include "../../module/planner.h"
#include "../../module/temperature.h"
#include "../../MarlinCore.h"

#if HAS_LEVELING
  #include "../../feature/bedlevel/bedlevel.h"
#endif

#if ENABLED(BABYSTEPPING)

  #include "../../feature/babystep.h"
  #include "../lcdprint.h"
  #if HAS_MARLINUI_U8GLIB
    #include "../dogm/marlinui_DOGM.h"
  #endif

  // TODO: Replace fmsg with MSG_BABYSTEP_N and index substitution

  void _lcd_babystep(const AxisEnum axis, FSTR_P const fmsg) {
    if (ui.use_click()) return ui.goto_previous_screen_no_defer();
    if (ui.encoderPosition) {
      const int16_t steps = int16_t(ui.encoderPosition) * (
        #if ENABLED(BABYSTEP_XY)
          axis == X_AXIS ? BABYSTEP_SIZE_X :
          axis == Y_AXIS ? BABYSTEP_SIZE_Y :
        #endif
        BABYSTEP_SIZE_Z
      );
      ui.encoderPosition = 0;
      ui.refresh(LCDVIEW_REDRAW_NOW);
      babystep.add_steps(axis, steps);
    }
    if (ui.should_draw()) {
      const float mps = planner.mm_per_step[axis];
      MenuEditItemBase::draw_edit_screen(fmsg, BABYSTEP_TO_STR(mps * babystep.accum));
      #if ENABLED(BABYSTEP_DISPLAY_TOTAL)
        const bool in_view = TERN1(HAS_MARLINUI_U8GLIB, PAGE_CONTAINS(LCD_PIXEL_HEIGHT - MENU_FONT_HEIGHT, LCD_PIXEL_HEIGHT - 1));
        if (in_view) {
          TERN_(HAS_MARLINUI_U8GLIB, ui.set_font(FONT_MENU));
          #if ENABLED(TFT_COLOR_UI)
            lcd_moveto(4, 3);
            lcd_put_u8str(GET_TEXT_F(MSG_BABYSTEP_TOTAL));
            lcd_put_u8str(F(":"));
            lcd_moveto(10, 3);
          #else
            lcd_moveto(0, TERN(HAS_MARLINUI_U8GLIB, LCD_PIXEL_HEIGHT - MENU_FONT_DESCENT, LCD_HEIGHT - 1));
            lcd_put_u8str(GET_TEXT_F(MSG_BABYSTEP_TOTAL));
            lcd_put_u8str(F(":"));
          #endif
          lcd_put_u8str(BABYSTEP_TO_STR(mps * babystep.axis_total[BS_TOTAL_IND(axis)]));
        }
      #endif
    }
  }

  inline void _lcd_babystep_go(const screenFunc_t screen) {
    ui.goto_screen(screen);
    ui.defer_status_screen();
    babystep.accum = 0;
  }

  #if ENABLED(BABYSTEP_XY)
    void _lcd_babystep_x() { _lcd_babystep(X_AXIS, GET_TEXT_F(MSG_BABYSTEP_X)); }
    void _lcd_babystep_y() { _lcd_babystep(Y_AXIS, GET_TEXT_F(MSG_BABYSTEP_Y)); }
  #endif

  #if DISABLED(BABYSTEP_ZPROBE_OFFSET)
    void _lcd_babystep_z() { _lcd_babystep(Z_AXIS, GET_TEXT_F(MSG_BABYSTEP_Z)); }
    void lcd_babystep_z()  { _lcd_babystep_go(_lcd_babystep_z); }
  #endif

#endif // BABYSTEPPING

void menu_tune() {
  START_MENU();
  BACK_ITEM(MSG_MAIN_MENU);

  //
  // Speed:
  //
  EDIT_ITEM(int3, MSG_SPEED, &feedrate_percentage, SPEED_EDIT_MIN, SPEED_EDIT_MAX);

  //
  // Manual bed leveling, Bed Z:
  //
  #if ALL(MESH_BED_LEVELING, LCD_BED_LEVELING)
    EDIT_ITEM(float43, MSG_MESH_Z_OFFSET, &bedlevel.z_offset, -1, 1);
  #endif

  //
  // Nozzle:
  // Nozzle [1-4]:
  //
  #if HOTENDS == 1
    EDIT_ITEM_FAST(int3, MSG_NOZZLE, &thermalManager.temp_hotend[0].target, 0, thermalManager.hotend_max_target(0), []{ thermalManager.start_watching_hotend(0); });
  #elif HAS_MULTI_HOTEND
    HOTEND_LOOP()
      EDIT_ITEM_FAST_N(int3, e, MSG_NOZZLE_N, &thermalManager.temp_hotend[e].target, 0, thermalManager.hotend_max_target(e), []{ thermalManager.start_watching_hotend(MenuItemBase::itemIndex); });
  #endif

  #if ENABLED(SINGLENOZZLE_STANDBY_TEMP)
    for (uint8_t e = 1; e < EXTRUDERS; ++e)
      EDIT_ITEM_FAST_N(int3, e, MSG_NOZZLE_STANDBY, &thermalManager.singlenozzle_temp[e], 0, thermalManager.hotend_max_target(0));
  #endif

  //
  // Bed:
  //
  #if HAS_HEATED_BED
    EDIT_ITEM_FAST(int3, MSG_BED, &thermalManager.temp_bed.target, 0, BED_MAX_TARGET, thermalManager.start_watching_bed);
  #endif

  //
  // Fan Speed:
  //
  #if HAS_FAN

    DEFINE_SINGLENOZZLE_ITEM();

    #if FAN_IS_M106ABLE(0)
      _FAN_EDIT_ITEMS(0, FIRST_FAN_SPEED);
    #endif
    #if FAN_IS_M106ABLE(1)
      FAN_EDIT_ITEMS(1);
    #elif SNFAN(1)
      singlenozzle_item(1);
    #endif
    #if FAN_IS_M106ABLE(2)
      FAN_EDIT_ITEMS(2);
    #elif SNFAN(2)
      singlenozzle_item(2);
    #endif
    #if FAN_IS_M106ABLE(3)
      FAN_EDIT_ITEMS(3);
    #elif SNFAN(3)
      singlenozzle_item(3);
    #endif
    #if FAN_IS_M106ABLE(4)
      FAN_EDIT_ITEMS(4);
    #elif SNFAN(4)
      singlenozzle_item(4);
    #endif
    #if FAN_IS_M106ABLE(5)
      FAN_EDIT_ITEMS(5);
    #elif SNFAN(5)
      singlenozzle_item(5);
    #endif
    #if FAN_IS_M106ABLE(6)
      FAN_EDIT_ITEMS(6);
    #elif SNFAN(6)
      singlenozzle_item(6);
    #endif
    #if FAN_IS_M106ABLE(7)
      FAN_EDIT_ITEMS(7);
    #elif SNFAN(7)
      singlenozzle_item(7);
    #endif

  #endif // HAS_FAN

  //
  // FT_MOTION
  //
  #if ENABLED(FT_MOTION_MENU)
    void menu_tune_ft_motion();
    SUBMENU(MSG_FIXED_TIME_MOTION, menu_tune_ft_motion);
  #endif

  //
  // Flow:
  //
  #if HAS_EXTRUDERS
    EDIT_ITEM(int3, MSG_FLOW, &planner.flow_percentage[active_extruder], FLOW_EDIT_MIN, FLOW_EDIT_MAX, []{ planner.refresh_e_factor(active_extruder); });
    // Flow En:
    #if HAS_MULTI_EXTRUDER
      EXTRUDER_LOOP()
        EDIT_ITEM_N(int3, e, MSG_FLOW_N, &planner.flow_percentage[e], FLOW_EDIT_MIN, FLOW_EDIT_MAX, []{ planner.refresh_e_factor(MenuItemBase::itemIndex); });
    #endif
  #endif

  //
  // Advance K:
  //
  #if ENABLED(LIN_ADVANCE) && DISABLED(SLIM_LCD_MENUS)
    #if DISTINCT_E < 2
      EDIT_ITEM(float42_52, MSG_ADVANCE_K, &planner.extruder_advance_K[0], 0, 10);
    #else
      EXTRUDER_LOOP()
        EDIT_ITEM_N(float42_52, e, MSG_ADVANCE_K_E, &planner.extruder_advance_K[e], 0, 10);
    #endif
  #endif

  //
  // Babystep X:
  // Babystep Y:
  // Babystep Z:
  //
  #if ENABLED(BABYSTEPPING)
    #if ENABLED(BABYSTEP_XY)
      SUBMENU_N(X_AXIS, MSG_BABYSTEP_N, []{ _lcd_babystep_go(_lcd_babystep_x); });
      SUBMENU_N(Y_AXIS, MSG_BABYSTEP_N, []{ _lcd_babystep_go(_lcd_babystep_y); });
    #endif
    #if ENABLED(BABYSTEP_ZPROBE_OFFSET)
      SUBMENU_N(Z_AXIS, MSG_ZPROBE_OFFSET_N, lcd_babystep_zoffset);
    #else
      SUBMENU_N(Z_AXIS, MSG_BABYSTEP_N, lcd_babystep_z);
    #endif
  #endif

  END_MENU();
}

#endif // HAS_MARLINUI_MENU
