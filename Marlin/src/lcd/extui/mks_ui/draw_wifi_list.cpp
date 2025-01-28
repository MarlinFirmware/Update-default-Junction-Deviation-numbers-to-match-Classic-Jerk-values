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

#include <lv_conf.h>
#include "tft_lvgl_configuration.h"

#if ENABLED(MKS_WIFI_MODULE)

#include "draw_ui.h"

#define NAME_BTN_X 330
#define NAME_BTN_Y 48

#define MARK_BTN_X 0
#define MARK_BTN_Y 68

WIFI_LIST wifi_list;
list_menu_def list_menu;

extern lv_group_t *g;
static lv_obj_t *scr;
static lv_obj_t *buttonWifiN[NUMBER_OF_PAGE];
static lv_obj_t *labelWifiText[NUMBER_OF_PAGE];
static lv_obj_t *labelPageText;

#define ID_WL_RETURN 11
#define ID_WL_DOWN   12

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;

  if (obj->mks_obj_id == ID_WL_RETURN) {
    clear_cur_ui();
    lv_draw_set();
  }
  else if (obj->mks_obj_id == ID_WL_DOWN) {
    if (wifi_list.getNameNum > 0) {
      if ((wifi_list.nameIndex + NUMBER_OF_PAGE) >= wifi_list.getNameNum) {
        wifi_list.nameIndex = 0;
        wifi_list.currentWifipage = 1;
      }
      else {
        wifi_list.nameIndex += NUMBER_OF_PAGE;
        wifi_list.currentWifipage++;
      }
      disp_wifi_list();
    }
  }
  else {
    for (uint_fast8_t i = 0; i < NUMBER_OF_PAGE; i++) {
      if (obj->mks_obj_id == i + 1) {
        if (wifi_list.getNameNum != 0) {
          const bool do_wifi = wifi_link_state == WIFI_CONNECTED && strcmp((const char *)wifi_list.wifiConnectedName, (const char *)wifi_list.wifiName[wifi_list.nameIndex + i]) == 0;
          wifi_list.nameIndex += i;
          last_disp_state = WIFI_LIST_UI;
          lv_clear_wifi_list();
          if (do_wifi)
            lv_draw_wifi();
          else {
            keyboard_value = wifiConfig;
            lv_draw_keyboard();
          }
        }
      }
    }
  }
}

void lv_draw_wifi_list() {
  scr = lv_screen_create(WIFI_LIST_UI);

  lv_obj_t *buttonDown = lv_imgbtn_create(scr, "F:/bmp_pageDown.bin", OTHER_BTN_XPIEL * 3 + INTERVAL_V * 4, titleHeight + OTHER_BTN_YPIEL + INTERVAL_H, event_handler, ID_WL_DOWN);
  lv_obj_t *buttonBack = lv_imgbtn_create(scr, "F:/bmp_back.bin", OTHER_BTN_XPIEL * 3 + INTERVAL_V * 4, titleHeight + (OTHER_BTN_YPIEL + INTERVAL_H) * 2, event_handler, ID_WL_RETURN);

  for (uint_fast8_t i = 0; i < NUMBER_OF_PAGE; i++) {
    buttonWifiN[i] = lv_label_btn_create(scr, 0, NAME_BTN_Y * i + 10 + titleHeight, NAME_BTN_X, NAME_BTN_Y, event_handler, i + 1);
    labelWifiText[i] = lv_label_create_empty(buttonWifiN[i]);
    #if HAS_ROTARY_ENCODER
      uint8_t j = 0;
      if (gCfgItems.encoder_enable) {
        j = wifi_list.nameIndex + i;
        if (j < wifi_list.getNameNum) lv_group_add_obj(g, buttonWifiN[i]);
      }
    #endif
  }

  labelPageText = lv_label_create_empty(scr);
  lv_obj_set_style(labelPageText, &tft_style_label_rel);

  wifi_list.nameIndex = 0;
  wifi_list.currentWifipage = 1;

  if (wifi_link_state == WIFI_CONNECTED && wifiPara.mode == STA_MODEL) {
    ZERO(wifi_list.wifiConnectedName);
    memcpy(wifi_list.wifiConnectedName, wifiPara.ap_name, sizeof(wifi_list.wifiConnectedName));
  }

  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) {
      lv_group_add_obj(g, buttonDown);
      lv_group_add_obj(g, buttonBack);
    }
  #else
    UNUSED(buttonDown);
    UNUSED(buttonBack);
  #endif

  disp_wifi_list();
}

void disp_wifi_list() {
  int8_t tmpStr[WIFI_NAME_BUFFER_SIZE] = { 0 };
  uint8_t i, j;

  sprintf((char *)tmpStr, list_menu.file_pages, wifi_list.currentWifipage, wifi_list.getPage);
  lv_label_set_text(labelPageText, (const char *)tmpStr);
  lv_obj_align(labelPageText, nullptr, LV_ALIGN_CENTER, 50, -100);

  for (i = 0; i < NUMBER_OF_PAGE; i++) {
    ZERO(tmpStr);

    j = wifi_list.nameIndex + i;
    if (j >= wifi_list.getNameNum) {
      lv_label_set_text(labelWifiText[i], (const char *)tmpStr);
      lv_obj_align(labelWifiText[i], buttonWifiN[i], LV_ALIGN_IN_LEFT_MID, 20, 0);
    }
    else {
      lv_label_set_text(labelWifiText[i], (char const *)wifi_list.wifiName[j]);
      lv_obj_align(labelWifiText[i], buttonWifiN[i], LV_ALIGN_IN_LEFT_MID, 20, 0);

      const bool btext = (wifi_link_state == WIFI_CONNECTED && strcmp((const char *)wifi_list.wifiConnectedName, (const char *)wifi_list.wifiName[j]) == 0);
      lv_btn_set_style(buttonWifiN[i], LV_BTN_STYLE_REL, btext ? &style_sel_text : &tft_style_label_rel);
    }
  }
}

void wifi_scan_handle() {
  if (!DIALOG_IS(WIFI_ENABLE_TIPS) || !uiCfg.command_send) return;
  last_disp_state = DIALOG_UI;
  lv_clear_dialog();
  if (wifi_link_state == WIFI_CONNECTED && wifiPara.mode != AP_MODEL)
    lv_draw_wifi();
  else
    lv_draw_wifi_list();
}

void lv_clear_wifi_list() {
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // MKS_WIFI_MODULE
#endif // HAS_TFT_LVGL_UI
