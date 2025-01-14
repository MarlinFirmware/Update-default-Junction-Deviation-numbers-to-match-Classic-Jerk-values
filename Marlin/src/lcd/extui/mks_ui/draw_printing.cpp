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

#include "../../../MarlinCore.h" // for marlin_state
#include "../../../module/temperature.h"
#include "../../../module/motion.h"
#include "../../../sd/cardreader.h"
#include "../../../gcode/queue.h"
#include "../../../gcode/gcode.h"
#include "../../../inc/MarlinConfig.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if ENABLED(SET_REMAINING_TIME)
  #include "../../marlinui.h"
#endif

extern lv_group_t *g;
static lv_obj_t *scr;
static lv_obj_t *labelExt1, *labelFan, *labelZpos, *labelTime;
static lv_obj_t *labelPause, *labelStop, *labelOperat;
static lv_obj_t *bar1, *bar1ValueText;
static lv_obj_t *buttonPause, *buttonOperat, *buttonStop, *buttonExt1, *buttonFanstate, *buttonZpos;

#if HAS_MULTI_EXTRUDER
  static lv_obj_t *labelExt2;
  static lv_obj_t *buttonExt2;
#endif

#if HAS_HEATED_BED
  static lv_obj_t* labelBed;
  static lv_obj_t* buttonBedstate;
#endif

enum {
  ID_PAUSE = 1,
  ID_STOP,
  ID_OPTION,
  ID_TEMP_EXT,
  ID_TEMP_BED,
  ID_BABYSTEP,
  ID_FAN
};

bool once_flag; // = false
extern bool flash_preview_begin, default_preview_flg, gcode_preview_over;
extern uint32_t To_pre_view;

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;
  if (gcode_preview_over) return;
  switch (obj->mks_obj_id) {
    case ID_PAUSE:
      if (uiCfg.print_state == WORKING) {
        #if HAS_MEDIA
          card.pauseSDPrint();
          stop_print_time();
          uiCfg.print_state = PAUSING;
        #endif
        lv_imgbtn_set_src_both(buttonPause, "F:/bmp_resume.bin");
        lv_label_set_text(labelPause, printing_menu.resume);
        lv_obj_align(labelPause, buttonPause, LV_ALIGN_CENTER, 30, 0);
      }
      else if (uiCfg.print_state == PAUSED) {
        uiCfg.print_state = RESUMING;
        lv_imgbtn_set_src_both(obj, "F:/bmp_pause.bin");
        lv_label_set_text(labelPause, printing_menu.pause);
        lv_obj_align(labelPause, buttonPause, LV_ALIGN_CENTER, 30, 0);
      }
      #if ENABLED(POWER_LOSS_RECOVERY)
        else if (uiCfg.print_state == REPRINTING) {
          uiCfg.print_state = REPRINTED;
          lv_imgbtn_set_src_both(obj, "F:/bmp_pause.bin");
          lv_label_set_text(labelPause, printing_menu.pause);
          lv_obj_align(labelPause, buttonPause, LV_ALIGN_CENTER, 30, 0);
          print_time.minutes = recovery.info.print_job_elapsed / 60;
          print_time.seconds = recovery.info.print_job_elapsed % 60;
          print_time.hours   = print_time.minutes / 60;
        }
      #endif
      break;
    case ID_STOP:
      lv_clear_printing();
      lv_draw_dialog(DIALOG_TYPE_STOP);
      break;
    case ID_OPTION:
      lv_clear_printing();
      lv_draw_operation();
      break;
    case ID_TEMP_EXT:
      uiCfg.curTempType = 0;
      lv_clear_printing();
      lv_draw_preHeat();
      break;
    case ID_TEMP_BED:
      uiCfg.curTempType = 1;
      lv_clear_printing();
      lv_draw_preHeat();
      break;
    case ID_BABYSTEP:
      lv_clear_printing();
      lv_draw_baby_stepping();
      break;
    case ID_FAN:
      lv_clear_printing();
      lv_draw_fan();
      break;
  }
}

void lv_draw_printing() {
  disp_state_stack._disp_index = 0;
  ZERO(disp_state_stack._disp_state);
  scr = lv_screen_create(PRINTING_UI);

  // Create image buttons
  buttonExt1 = lv_imgbtn_create(scr, "F:/bmp_ext1_state.bin", 206, 136, event_handler, ID_TEMP_EXT);

  #if HAS_MULTI_EXTRUDER
    buttonExt2 = lv_imgbtn_create(scr, "F:/bmp_ext2_state.bin", 350, 136, event_handler, ID_TEMP_EXT);
  #endif

  #if HAS_HEATED_BED
    buttonBedstate = lv_imgbtn_create(scr, "F:/bmp_bed_state.bin", 206, 186, event_handler, ID_TEMP_BED);
  #endif

  buttonFanstate = lv_imgbtn_create(scr, "F:/bmp_fan_state.bin", 350, 186, event_handler, ID_FAN);

  lv_obj_t *buttonTime = lv_img_create(scr, nullptr);
  lv_img_set_src(buttonTime, "F:/bmp_time_state.bin");
  lv_obj_set_pos(buttonTime, 206, 86);

  buttonZpos = lv_imgbtn_create(scr, "F:/bmp_zpos_state.bin", 350, 86, event_handler, ID_BABYSTEP);

  buttonPause  = lv_imgbtn_create(scr, uiCfg.print_state == WORKING ? "F:/bmp_pause.bin" : "F:/bmp_resume.bin", 5, 240, event_handler, ID_PAUSE);
  buttonStop   = lv_imgbtn_create(scr, "F:/bmp_stop.bin", 165, 240, event_handler, ID_STOP);
  buttonOperat = lv_imgbtn_create(scr, "F:/bmp_operate.bin", 325, 240, event_handler, ID_OPTION);

  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) {
      lv_group_add_obj(g, buttonPause);
      lv_group_add_obj(g, buttonStop);
      lv_group_add_obj(g, buttonOperat);
      lv_group_add_obj(g, buttonPause);
      lv_group_add_obj(g, buttonPause);
      lv_group_add_obj(g, buttonPause);
    }
  #endif

  labelExt1 = lv_label_create(scr, 250, 146, nullptr);

  #if HAS_MULTI_EXTRUDER
    labelExt2 = lv_label_create(scr, 395, 146, nullptr);
  #endif

  #if HAS_HEATED_BED
    labelBed = lv_label_create(scr, 250, 196, nullptr);
  #endif

  labelFan  = lv_label_create(scr, 395, 196, nullptr);
  labelTime = lv_label_create(scr, 250, 96, nullptr);
  labelZpos = lv_label_create(scr, 395, 96, nullptr);

  labelPause  = lv_label_create_empty(buttonPause);
  labelStop   = lv_label_create_empty(buttonStop);
  labelOperat = lv_label_create_empty(buttonOperat);

  if (gCfgItems.multiple_language) {
    lv_label_set_text(labelPause, uiCfg.print_state == WORKING ? printing_menu.pause : printing_menu.resume);
    lv_obj_align(labelPause, buttonPause, LV_ALIGN_CENTER, 20, 0);

    lv_label_set_text(labelStop, printing_menu.stop);
    lv_obj_align(labelStop, buttonStop, LV_ALIGN_CENTER, 20, 0);

    lv_label_set_text(labelOperat, printing_menu.option);
    lv_obj_align(labelOperat, buttonOperat, LV_ALIGN_CENTER, 20, 0);
  }

  bar1 = lv_bar_create(scr, nullptr);
  lv_obj_set_pos(bar1, 205, 36);
  lv_obj_set_size(bar1, 270, 40);
  lv_bar_set_style(bar1, LV_BAR_STYLE_INDIC, &lv_bar_style_indic);
  lv_bar_set_anim_time(bar1, 1000);
  lv_bar_set_value(bar1, 0, LV_ANIM_ON);
  bar1ValueText = lv_label_create_empty(bar1);
  lv_label_set_text(bar1ValueText, "0%");
  lv_obj_align(bar1ValueText, bar1, LV_ALIGN_CENTER, 0, 0);

  disp_ext_temp();
  disp_bed_temp();
  disp_fan_speed();
  disp_print_time();
  disp_fan_Zpos();
}

void disp_ext_temp() {
  sprintf(public_buf_l, printing_menu.temp1, thermalManager.wholeDegHotend(0), thermalManager.degTargetHotend(0));
  lv_label_set_text(labelExt1, public_buf_l);

  #if HAS_MULTI_EXTRUDER
    sprintf(public_buf_l, printing_menu.temp1, thermalManager.wholeDegHotend(1), thermalManager.degTargetHotend(1));
    lv_label_set_text(labelExt2, public_buf_l);
  #endif
}

void disp_bed_temp() {
  #if HAS_HEATED_BED
    sprintf(public_buf_l, printing_menu.bed_temp, thermalManager.wholeDegBed(), thermalManager.degTargetBed());
    lv_label_set_text(labelBed, public_buf_l);
  #endif
}

void disp_fan_speed() {
  sprintf_P(public_buf_l, PSTR("%d%%"), (int)thermalManager.fanSpeedPercent(0));
  lv_label_set_text(labelFan, public_buf_l);
}

void disp_print_time() {
  #if ENABLED(SET_REMAINING_TIME)
    const uint32_t r = ui.get_remaining_time();
    sprintf_P(public_buf_l, PSTR("%02d:%02d R"), r / 3600, (r % 3600) / 60);
  #else
    sprintf_P(public_buf_l, PSTR("%d%d:%d%d:%d%d"), print_time.hours / 10, print_time.hours % 10, print_time.minutes / 10, print_time.minutes % 10, print_time.seconds / 10, print_time.seconds % 10);
  #endif
  lv_label_set_text(labelTime, public_buf_l);
}

void disp_fan_Zpos() {
  dtostrf(current_position.z, 1, 3, public_buf_l);
  lv_label_set_text(labelZpos, public_buf_l);
}

void reset_print_time() {
  print_time.hours   = 0;
  print_time.minutes = 0;
  print_time.seconds = 0;
  print_time.ms_10   = 0;
}

void start_print_time() { print_time.start = 1; }

void stop_print_time() { print_time.start = 0; }

void setProBarRate() {
  int rate;
  volatile long long rate_tmp_r;

  if (!gCfgItems.from_flash_pic) {
    #if HAS_MEDIA
      rate_tmp_r = (long long)card.getIndex() * 100;
    #endif
    rate = rate_tmp_r / gCfgItems.curFilesize;
  }
  else {
    #if HAS_MEDIA
      rate_tmp_r = (long long)card.getIndex();
    #endif
    rate = (rate_tmp_r - (PREVIEW_SIZE + To_pre_view)) * 100 / (gCfgItems.curFilesize - (PREVIEW_SIZE + To_pre_view));
  }

  if (rate <= 0) return;

  if (disp_state == PRINTING_UI) {
    lv_bar_set_value(bar1, rate, LV_ANIM_ON);
    sprintf_P(public_buf_l, "%d%%", rate);
    lv_label_set_text(bar1ValueText, public_buf_l);
    lv_obj_align(bar1ValueText, bar1, LV_ALIGN_CENTER, 0, 0);

    if (marlin_state == MarlinState::MF_SD_COMPLETE) {
      if (once_flag == 0) {
        stop_print_time();

        flash_preview_begin = false;
        default_preview_flg = false;
        lv_clear_printing();
        lv_draw_dialog(DIALOG_TYPE_FINISH_PRINT);

        once_flag = true;

        #if HAS_SUICIDE
          if (gCfgItems.finish_power_off) {
            gcode.process_subcommands_now(F("M1001"));
            queue.inject(F("M81"));
            marlin_state = MarlinState::MF_RUNNING;
          }
        #endif
      }
    }
  }
}

void lv_clear_printing() {
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // HAS_TFT_LVGL_UI
