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

#include "../../../gcode/gcode.h"
#include "../../../module/planner.h"
#include "../../../module/motion.h"
#include "../../../sd/cardreader.h"
#include "../../../inc/MarlinConfig.h"
#include "../../../MarlinCore.h"
#include "../../../gcode/queue.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#define FILAMENT_IS_OUT(N...) (READ(FIL_RUNOUT##N##_PIN) == FIL_RUNOUT##N##_STATE)
#ifndef FILAMENT_RUNOUT_THRESHOLD
  #define FILAMENT_RUNOUT_THRESHOLD 20
#endif

extern uint32_t To_pre_view;
extern bool flash_preview_begin, default_preview_flg, gcode_preview_over;

void printer_state_polling() {
  char str_1[16];
  if (uiCfg.print_state == PAUSING) {
    #if HAS_MEDIA
      if (!planner.has_blocks_queued() && card.getIndex() > MIN_FILE_PRINTED)
        uiCfg.waitEndMoves++;

      if (uiCfg.waitEndMoves > 20) {
        uiCfg.waitEndMoves = 0;
        planner.synchronize();

        gcode.process_subcommands_now(F("M25"));

        // save the position
        uiCfg.current_x_position_bak = current_position.x;
        uiCfg.current_y_position_bak = current_position.y;
        uiCfg.current_z_position_bak = current_position.z;

        if (gCfgItems.pausePosZ != (float)-1) {
          sprintf_P(public_buf_l, PSTR("G91\nG1 Z%s\nG90"), dtostrf(gCfgItems.pausePosZ, 1, 1, str_1));
          gcode.process_subcommands_now(public_buf_l);
        }
        if (gCfgItems.pausePosX != (float)-1 && gCfgItems.pausePosY != (float)-1) {
          sprintf_P(public_buf_l, PSTR("G1 X%s Y%s"), dtostrf(gCfgItems.pausePosX, 1, 1, str_1), dtostrf(gCfgItems.pausePosY, 1, 1, str_1));
          gcode.process_subcommands_now(public_buf_l);
        }
        uiCfg.print_state = PAUSED;
        uiCfg.current_e_position_bak = current_position.e;

        gCfgItems.pause_reprint = true;
        update_spi_flash();
      }
    #endif
  }
  else
    uiCfg.waitEndMoves = 0;

  if (uiCfg.print_state == PAUSED) {
  }

  if (uiCfg.print_state == RESUMING) {
    if (IS_SD_PAUSED()) {
      if (gCfgItems.pausePosX != (float)-1 && gCfgItems.pausePosY != (float)-1) {
        sprintf_P(public_buf_m, PSTR("G1 X%s Y%s"), dtostrf(uiCfg.current_x_position_bak, 1, 1, str_1), dtostrf(uiCfg.current_y_position_bak, 1, 1, str_1));
        gcode.process_subcommands_now(public_buf_m);
      }
      if (gCfgItems.pausePosZ != (float)-1) {
        ZERO(public_buf_m);
        sprintf_P(public_buf_m, PSTR("G1 Z%s"), dtostrf(uiCfg.current_z_position_bak, 1, 1, str_1));
        gcode.process_subcommands_now(public_buf_m);
      }
      gcode.process_subcommands_now(FPSTR(M24_STR));
      uiCfg.print_state = WORKING;
      start_print_time();

      gCfgItems.pause_reprint = false;
      update_spi_flash();
    }
  }

  #if ENABLED(POWER_LOSS_RECOVERY)
    if (uiCfg.print_state == REPRINTED) {
      #if HAS_HOTEND
        HOTEND_LOOP() {
          const int16_t et = recovery.info.target_temperature[e];
          if (et) {
            #if HAS_MULTI_HOTEND
              sprintf_P(public_buf_m, PSTR("T%i"), e);
              gcode.process_subcommands_now(public_buf_m);
            #endif
            sprintf_P(public_buf_m, PSTR("M109 S%i"), et);
            gcode.process_subcommands_now(public_buf_m);
          }
        }
      #endif

      recovery.resume();

      #if 0
        // Move back to the saved XY
        char str_1[16], str_2[16];
        sprintf_P(public_buf_m, PSTR("G1 X%s Y%s F2000"),
          dtostrf(recovery.info.current_position.x, 1, 3, str_1),
          dtostrf(recovery.info.current_position.y, 1, 3, str_2)
        );
        gcode.process_subcommands_now(public_buf_m);

        if (gCfgItems.pause_reprint && gCfgItems.pausePosZ != -1.0f) {
          sprintf_P(public_buf_l, PSTR("G91\nG1 Z-%s\nG90"), dtostrf(gCfgItems.pausePosZ, 1, 1, str_2));
          gcode.process_subcommands_now(public_buf_l);
        }
      #endif
      uiCfg.print_state = WORKING;
      start_print_time();

      gCfgItems.pause_reprint = false;
      update_spi_flash();
    }
  #endif

  if (uiCfg.print_state == WORKING) filament_check();

  TERN_(MKS_WIFI_MODULE, wifi_looping());
}

void filament_pin_setup() {
  #if PIN_EXISTS(FIL_RUNOUT1)
    SET_INPUT_PULLUP(FIL_RUNOUT1_PIN);
  #endif
  #if PIN_EXISTS(FIL_RUNOUT2)
    SET_INPUT_PULLUP(FIL_RUNOUT2_PIN);
  #endif
  #if PIN_EXISTS(FIL_RUNOUT3)
    SET_INPUT_PULLUP(FIL_RUNOUT3_PIN);
  #endif
}

void filament_check() {
  #if PIN_EXISTS(FIL_RUNOUT1)
    static int fil_det_count_1 = 0;
    if (FILAMENT_IS_OUT(1))
      fil_det_count_1++;
    else if (fil_det_count_1 > 0)
      fil_det_count_1--;
  #endif

  #if PIN_EXISTS(FIL_RUNOUT2)
    static int fil_det_count_2 = 0;
    if (FILAMENT_IS_OUT(2))
      fil_det_count_2++;
    else if (fil_det_count_2 > 0)
      fil_det_count_2--;
  #endif

  #if PIN_EXISTS(FIL_RUNOUT3)
    static int fil_det_count_3 = 0;
    if (FILAMENT_IS_OUT(3))
      fil_det_count_3++;
    else if (fil_det_count_3 > 0)
      fil_det_count_3--;
  #endif

  if (false
    #if PIN_EXISTS(FIL_RUNOUT1)
      || fil_det_count_1 >= FILAMENT_RUNOUT_THRESHOLD
    #endif
    #if PIN_EXISTS(FIL_RUNOUT2)
      || fil_det_count_2 >= FILAMENT_RUNOUT_THRESHOLD
    #endif
    #if PIN_EXISTS(FIL_RUNOUT3)
      || fil_det_count_3 >= FILAMENT_RUNOUT_THRESHOLD
    #endif
  ) {
    clear_cur_ui();
    card.pauseSDPrint();
    stop_print_time();
    uiCfg.print_state = PAUSING;

    if (gCfgItems.from_flash_pic)
      flash_preview_begin = true;
    else
      default_preview_flg = true;

    lv_draw_printing();
  }
}

#endif // HAS_TFT_LVGL_UI
