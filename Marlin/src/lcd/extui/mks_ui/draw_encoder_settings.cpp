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

#include "../../../inc/MarlinConfig.h"

#if BUTTONS_EXIST(EN1, EN2)

extern lv_group_t *g;
static lv_obj_t *scr;
static lv_obj_t *buttonEncoderState = nullptr;

enum {
  ID_ENCODER_RETURN = 1,
  ID_ENCODER_STATE
};

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;
  switch (obj->mks_obj_id) {
    case ID_ENCODER_RETURN:
      lv_clear_encoder_settings();
      draw_return_ui();
      break;
    case ID_ENCODER_STATE:
      FLIP(gCfgItems.encoder_enable);
      lv_screen_menu_item_onoff_update(buttonEncoderState, gCfgItems.encoder_enable);
      update_spi_flash();
      break;
  }
}

void lv_draw_encoder_settings() {
  scr = lv_screen_create(ENCODER_SETTINGS_UI, machine_menu.EncoderConfTitle);
  buttonEncoderState = lv_screen_menu_item_onoff(scr, machine_menu.EncoderConfText, PARA_UI_POS_X, PARA_UI_POS_Y, event_handler, ID_ENCODER_STATE, 0, gCfgItems.encoder_enable);
  lv_big_button_create(scr, "F:/bmp_back70x40.bin", common_menu.text_back, PARA_UI_BACK_POS_X, PARA_UI_BACK_POS_Y, event_handler, ID_ENCODER_RETURN, true);
}

void lv_clear_encoder_settings() {
  #if HAS_ROTARY_ENCODER
    lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // BUTTONS_EXIST(EN1, EN2)

#endif // HAS_TFT_LVGL_UI
