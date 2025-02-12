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

#if HAS_TFT_LVGL_UI

#include "draw_ui.h"
#include <lv_conf.h>

#include "../../../module/temperature.h"
#include "../../../gcode/gcode.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../inc/MarlinConfig.h"

extern lv_group_t *g;
static lv_obj_t *scr;
static lv_obj_t *buttonType;
static lv_obj_t *labelType;
static lv_obj_t *tempText1;

enum {
  ID_FILAMNT_IN = 1,
  ID_FILAMNT_OUT,
  ID_FILAMNT_TYPE,
  ID_FILAMNT_RETURN
};

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;
  switch (obj->mks_obj_id) {
    case ID_FILAMNT_IN:
      uiCfg.filament_load_heat_flg = true;
      if (ABS(thermalManager.degTargetHotend(uiCfg.extruderIndex) - thermalManager.wholeDegHotend(uiCfg.extruderIndex)) <= 1
        || gCfgItems.filament_limit_temp <= thermalManager.wholeDegHotend(uiCfg.extruderIndex)
      ) {
        lv_clear_filament_change();
        lv_draw_dialog(DIALOG_TYPE_FILAMENT_HEAT_LOAD_COMPLETED);
      }
      else {
        lv_clear_filament_change();
        lv_draw_dialog(DIALOG_TYPE_FILAMENT_LOAD_HEAT);
        if (thermalManager.degTargetHotend(uiCfg.extruderIndex) < gCfgItems.filament_limit_temp) {
          thermalManager.setTargetHotend(gCfgItems.filament_limit_temp, uiCfg.extruderIndex);
          thermalManager.start_watching_hotend(uiCfg.extruderIndex);
        }
      }
      break;
    case ID_FILAMNT_OUT:
      uiCfg.filament_unload_heat_flg = true;
      if (thermalManager.degTargetHotend(uiCfg.extruderIndex)
          && (ABS(thermalManager.degTargetHotend(uiCfg.extruderIndex) - thermalManager.wholeDegHotend(uiCfg.extruderIndex)) <= 1
              || thermalManager.wholeDegHotend(uiCfg.extruderIndex) >= gCfgItems.filament_limit_temp)
      ) {
        lv_clear_filament_change();
        lv_draw_dialog(DIALOG_TYPE_FILAMENT_HEAT_UNLOAD_COMPLETED);
      }
      else {
        lv_clear_filament_change();
        lv_draw_dialog(DIALOG_TYPE_FILAMENT_UNLOAD_HEAT);
        if (thermalManager.degTargetHotend(uiCfg.extruderIndex) < gCfgItems.filament_limit_temp) {
          thermalManager.setTargetHotend(gCfgItems.filament_limit_temp, uiCfg.extruderIndex);
          thermalManager.start_watching_hotend(uiCfg.extruderIndex);
        }
        filament_sprayer_temp();
      }
      break;
    case ID_FILAMNT_TYPE:
      #if HAS_MULTI_EXTRUDER
        FLIP(uiCfg.extruderIndex);
      #endif
      disp_filament_type();
      break;
    case ID_FILAMNT_RETURN:
      #if HAS_MULTI_EXTRUDER
        if (uiCfg.print_state != IDLE && uiCfg.print_state != REPRINTED)
          gcode.process_subcommands_now(uiCfg.extruderIndexBak == 1 ? F("T1") : F("T0"));
      #endif
      feedrate_mm_s = (float)uiCfg.moveSpeed_bak;
      if (uiCfg.print_state == PAUSED)
        planner.set_e_position_mm((destination.e = current_position.e = uiCfg.current_e_position_bak));
      thermalManager.setTargetHotend(uiCfg.hotendTargetTempBak, uiCfg.extruderIndex);

      goto_previous_ui();
      break;
  }
}

void lv_draw_filament_change() {
  scr = lv_screen_create(FILAMENTCHANGE_UI);
  // Create an Image button
  lv_obj_t *buttonIn = lv_big_button_create(scr, "F:/bmp_in.bin", filament_menu.in, INTERVAL_V, titleHeight, event_handler, ID_FILAMNT_IN);
  lv_obj_clear_protect(buttonIn, LV_PROTECT_FOLLOW);
  lv_big_button_create(scr, "F:/bmp_out.bin", filament_menu.out, BTN_X_PIXEL * 3 + INTERVAL_V * 4, titleHeight, event_handler, ID_FILAMNT_OUT);

  buttonType = lv_imgbtn_create(scr, nullptr, INTERVAL_V, BTN_Y_PIXEL + INTERVAL_H + titleHeight, event_handler, ID_FILAMNT_TYPE);
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable)
      lv_group_add_obj(g, buttonType);
  #endif

  lv_big_button_create(scr, "F:/bmp_return.bin", common_menu.text_back, BTN_X_PIXEL * 3 + INTERVAL_V * 4, BTN_Y_PIXEL + INTERVAL_H + titleHeight, event_handler, ID_FILAMNT_RETURN);

  // Create labels on the image buttons
  labelType = lv_label_create_empty(buttonType);

  disp_filament_type();

  tempText1 = lv_label_create_empty(scr);
  lv_obj_set_style(tempText1, &tft_style_label_rel);
  disp_filament_temp();
}

void disp_filament_type() {
  if (uiCfg.extruderIndex == 1) {
    lv_imgbtn_set_src_both(buttonType, "F:/bmp_extru2.bin");
    if (gCfgItems.multiple_language) {
      lv_label_set_text(labelType, preheat_menu.ext2);
      lv_obj_align(labelType, buttonType, LV_ALIGN_IN_BOTTOM_MID, 0, BUTTON_TEXT_Y_OFFSET);
    }
  }
  else {
    lv_imgbtn_set_src_both(buttonType, "F:/bmp_extru1.bin");
    if (gCfgItems.multiple_language) {
      lv_label_set_text(labelType, preheat_menu.ext1);
      lv_obj_align(labelType, buttonType, LV_ALIGN_IN_BOTTOM_MID, 0, BUTTON_TEXT_Y_OFFSET);
    }
  }
}

void disp_filament_temp() {
  char buf[20] = {0};

  public_buf_l[0] = '\0';

  strcat(public_buf_l, uiCfg.extruderIndex < 1 ? preheat_menu.ext1 : preheat_menu.ext2);
  sprintf(buf, preheat_menu.value_state, thermalManager.wholeDegHotend(uiCfg.extruderIndex), thermalManager.degTargetHotend(uiCfg.extruderIndex));

  strcat_P(public_buf_l, PSTR(": "));
  strcat(public_buf_l, buf);
  lv_label_set_text(tempText1, public_buf_l);
  lv_obj_align(tempText1, nullptr, LV_ALIGN_CENTER, 0, -50);
}

void lv_clear_filament_change() {
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // HAS_TFT_LVGL_UI
