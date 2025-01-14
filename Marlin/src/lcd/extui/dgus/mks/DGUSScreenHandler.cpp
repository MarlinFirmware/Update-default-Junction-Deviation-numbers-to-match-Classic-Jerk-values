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

#include "../../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_MKS

#include "../DGUSScreenHandler.h"

#include "../../../../inc/MarlinConfig.h"

#include "../../../../MarlinCore.h"
#include "../../../../module/settings.h"
#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"
#include "../../../../module/printcounter.h"

#include "../../../../gcode/gcode.h"

#if HAS_STEALTHCHOP
  #include "../../../../module/stepper/trinamic.h"
  #include "../../../../module/stepper/indirection.h"
#endif
#include "../../../../module/probe.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../../feature/powerloss.h"
#endif

#if ENABLED(DGUS_MKS_RUNOUT_SENSOR)
  #define FILAMENT_IS_OUT(N...) (READ(FIL_RUNOUT##N##_PIN) == FIL_RUNOUT##N##_STATE)
#endif

#if HAS_MEDIA
  extern ExtUI::FileList filelist;
#endif

bool DGUSAutoTurnOff = false;
MKS_Language mks_language_index; // Initialized by settings.load

#if 0
void DGUSScreenHandlerMKS::sendinfoscreen_ch(const uint16_t *line1, const uint16_t *line2, const uint16_t *line3, const uint16_t *line4) {
  dgus.writeVariable(VP_MSGSTR1, line1, 32, true);
  dgus.writeVariable(VP_MSGSTR2, line2, 32, true);
  dgus.writeVariable(VP_MSGSTR3, line3, 32, true);
  dgus.writeVariable(VP_MSGSTR4, line4, 32, true);
}

void DGUSScreenHandlerMKS::sendinfoscreen_en(PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4) {
  dgus.writeVariable(VP_MSGSTR1, line1, 32, true);
  dgus.writeVariable(VP_MSGSTR2, line2, 32, true);
  dgus.writeVariable(VP_MSGSTR3, line3, 32, true);
  dgus.writeVariable(VP_MSGSTR4, line4, 32, true);
}

void DGUSScreenHandlerMKS::sendInfoScreen(const void *line1, const void *line2, const void *line3, const void *line4, uint16_t language) {
  if (language == MKS_English)
    DGUSScreenHandlerMKS::sendinfoscreen_en((char *)line1, (char *)line2, (char *)line3, (char *)line4);
  else if (language == MKS_SimpleChinese)
    DGUSScreenHandlerMKS::sendinfoscreen_ch((uint16_t *)line1, (uint16_t *)line2, (uint16_t *)line3, (uint16_t *)line4);
}

#endif

void DGUSScreenHandlerMKS::sendFanToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    uint16_t tmp = *(uint8_t *) var.memadr; // +1 -> avoid rounding issues for the display.
    // tmp = map(tmp, 0, 255, 0, 100);
    dgus.writeVariable(var.VP, tmp);
  }
}

void DGUSScreenHandlerMKS::sendBabyStepToDisplay(DGUS_VP_Variable &var) {
  float value = current_position.z;
  value *= cpow(10, 2);
  dgus.writeVariable(VP_SD_Print_Baby, (uint16_t)value);
}

void DGUSScreenHandlerMKS::sendPrintTimeToDisplay(DGUS_VP_Variable &var) {
  duration_t elapsed = print_job_timer.duration();
  uint32_t time = elapsed.value;
  dgus.writeVariable(VP_PrintTime_H, uint16_t(time / 3600));
  dgus.writeVariable(VP_PrintTime_M, uint16_t(time % 3600 / 60));
  dgus.writeVariable(VP_PrintTime_S, uint16_t((time % 3600) % 60));
}

void DGUSScreenHandlerMKS::setUint8(DGUS_VP_Variable &var, void *val_ptr) {
  if (var.memadr) {
    const uint16_t value = BE16_P(val_ptr);
    *(uint8_t*)var.memadr = map(constrain(value, 0, 255), 0, 255, 0, 255);
  }
}

void DGUSScreenHandlerMKS::sendGbkToDisplay(DGUS_VP_Variable &var) {
  uint16_t *tmp = (uint16_t*) var.memadr;
  dgus.writeVariable(var.VP, tmp, var.size, true);
}

void DGUSScreenHandlerMKS::sendStringToDisplay_Language(DGUS_VP_Variable &var) {
  if (mks_language_index == MKS_English) {
    char *tmp = (char*) var.memadr;
    dgus.writeVariable(var.VP, tmp, var.size, true);
  }
  else if (mks_language_index == MKS_SimpleChinese) {
    uint16_t *tmp = (uint16_t *)var.memadr;
    dgus.writeVariable(var.VP, tmp, var.size, true);
  }
}

void DGUSScreenHandlerMKS::sendTMCStepValue(DGUS_VP_Variable &var) {
  #if ENABLED(SENSORLESS_HOMING)
    #if X_HAS_STEALTHCHOP
      tmc_step.x = stepperX.homing_threshold();
      dgus.writeVariable(var.VP, *(int16_t*)var.memadr);
    #endif
    #if Y_HAS_STEALTHCHOP
      tmc_step.y = stepperY.homing_threshold();
      dgus.writeVariable(var.VP, *(int16_t*)var.memadr);
    #endif
    #if Z_HAS_STEALTHCHOP
      tmc_step.z = stepperZ.homing_threshold();
      dgus.writeVariable(var.VP, *(int16_t*)var.memadr);
    #endif
  #endif
}

#if HAS_MEDIA

  void DGUSScreenHandler::sdFileSelected(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t touched_nr = (int16_t)BE16_P(val_ptr) + top_file;
    if (touched_nr != 0x0F && touched_nr > filelist.count()) return;
    if (!filelist.seek(touched_nr) && touched_nr != 0x0F) return;

    if (touched_nr == 0x0F) {
      if (filelist.isAtRootDir())
        gotoScreen(DGUS_SCREEN_MAIN);
      else
        filelist.upDir();
      return;
    }

    if (filelist.isDir()) {
      filelist.changeDir(filelist.filename());
      top_file = 0;
      forceCompleteUpdate();
      return;
    }

    #if ENABLED(DGUS_PRINT_FILENAME)
      // Send print filename
      dgus.writeVariable(VP_SD_Print_Filename, filelist.filename(), VP_SD_FileName_LEN, true);
    #endif

    // Setup Confirmation screen
    file_to_print = touched_nr;
    gotoScreen(MKSLCD_SCREEN_PRINT_CONFIRM);
  }

  void DGUSScreenHandler::sdStartPrint(DGUS_VP_Variable &var, void *val_ptr) {
    if (!filelist.seek(file_to_print)) return;
    ExtUI::printFile(filelist.shortFilename());
    gotoScreen(MKSLCD_SCREEN_PRINT);
    z_offset_add = 0;
  }

  void DGUSScreenHandler::sdResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr) {

    if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    switch (BE16_P(val_ptr)) {
      case 0: { // Resume
        auto cs = getCurrentScreen();
        if (runout_mks.runout_status != RUNOUT_WAITING_STATUS && runout_mks.runout_status != UNRUNOUT_STATUS) {
          if (cs == MKSLCD_SCREEN_PRINT || cs == MKSLCD_SCREEN_PAUSE)
            gotoScreen(MKSLCD_SCREEN_PAUSE);
          return;
        }
        else
          runout_mks.runout_status = UNRUNOUT_STATUS;

        gotoScreen(MKSLCD_SCREEN_PRINT);

        if (ExtUI::isPrintingFromMediaPaused()) {
          nozzle_park_mks.print_pause_start_flag = 0;
          nozzle_park_mks.blstatus = true;
          ExtUI::resumePrint();
        }
      } break;

      case 1: // Pause
        gotoScreen(MKSLCD_SCREEN_PAUSE);
        if (!ExtUI::isPrintingFromMediaPaused()) {
          nozzle_park_mks.print_pause_start_flag = 1;
          nozzle_park_mks.blstatus = true;
          ExtUI::pausePrint();
          //ExtUI::mks_pausePrint();
        }
        break;

      case 2: // Abort
        handleUserConfirmationPopUp(VP_SD_AbortPrintConfirmed, nullptr, PSTR("Abort printing"), filelist.filename(), PSTR("?"), true, true, false, true);
        break;
    }
  }

  void DGUSScreenHandler::sdSendFilename(DGUS_VP_Variable& var) {
    uint16_t target_line = (var.VP - VP_SD_FileName0) / VP_SD_FileName_LEN;
    if (target_line > DGUS_SD_FILESPERSCREEN) return;
    char tmpfilename[VP_SD_FileName_LEN + 1] = "";
    var.memadr = (void*)tmpfilename;

    uint16_t dir_icon_val = 25;
    if (filelist.seek(top_file + target_line)) {
      snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s%c"), filelist.filename(), filelist.isDir() ? '/' : 0); // snprintf_P(tmpfilename, VP_SD_FileName_LEN, PSTR("%s"), filelist.filename());
      dir_icon_val = filelist.isDir() ? 0 : 1;
    }
    sendStringToDisplay(var);

    dgus.writeVariable(VP_File_Pictutr0 + target_line * 2, dir_icon_val);
  }

  void DGUSScreenHandler::sdCardInserted() {
    top_file = 0;
    filelist.refresh();
    auto cs = getCurrentScreen();
    if (cs == DGUS_SCREEN_MAIN || cs == DGUS_SCREEN_STATUS)
      gotoScreen(MKSLCD_SCREEN_CHOOSE_FILE);
  }

  void DGUSScreenHandler::sdCardRemoved() {
    if (current_screenID == DGUS_SCREEN_SDFILELIST
        || (current_screenID == DGUS_SCREEN_CONFIRM && (confirmVP == VP_SD_AbortPrintConfirmed || confirmVP == VP_SD_FileSelectConfirm))
        || current_screenID == DGUS_SCREEN_SDPRINTMANIPULATION
    ) filelist.refresh();
  }

  void DGUSScreenHandlerMKS::sdPrintingFinished() {
    if (DGUSAutoTurnOff) {
      queue.exhaust();
      gcode.process_subcommands_now(F("M81"));
    }
    gotoScreen(MKSLCD_SCREEN_PrintDone);
  }

#else
  void DGUSScreenHandlerMKS::printReturn(DGUS_VP_Variable& var, void *val_ptr) {
    const uint16_t value = BE16_P(val_ptr);
    if (value == 0x0F) gotoScreen(DGUS_SCREEN_MAIN);
  }
#endif // HAS_MEDIA

void DGUSScreenHandler::screenChangeHook(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t *tmp = (uint8_t*)val_ptr;

  // The keycode in target is coded as <from-frame><to-frame>, so 0x0100A means
  // from screen 1 (main) to 10 (temperature). DGUS_SCREEN_POPUP is special,
  // meaning "return to previous screen"
  DGUS_ScreenID target = (DGUS_ScreenID)tmp[1];

  // when the dgus had reboot, it will enter the DGUS_SCREEN_MAIN page,
  // so user can change any page to use this function, an it will check
  // if robin nano is printing. when it is, dgus will enter the printing
  // page to continue print;
  //
  //if (printJobOngoing() || printingIsPaused()) {
  //  if (target == MKSLCD_PAUSE_SETTING_MOVE || target == MKSLCD_PAUSE_SETTING_EX
  //    || target == MKSLCD_SCREEN_PRINT || target == MKSLCD_SCREEN_PAUSE
  //  ) {
  //  }
  //  else
  //    gotoScreen(MKSLCD_SCREEN_PRINT);
  // return;
  //}

  if (target == DGUS_SCREEN_POPUP) {
    setupConfirmAction(ExtUI::setUserConfirmed);

    // Special handling for popup is to return to previous menu
    if (current_screenID == DGUS_SCREEN_POPUP && confirm_action_cb) confirm_action_cb();
    popToOldScreen();
    return;
  }

  updateNewScreen(target);

  #ifdef DEBUG_DGUSLCD
    if (!findScreenVPMapList(target)) DEBUG_ECHOLNPGM("WARNING: No screen Mapping found for ", target);
  #endif
}

void DGUSScreenHandlerMKS::screenBackChange(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t target = BE16_P(val_ptr);
  switch (target) {
  }
}

void DGUSScreenHandlerMKS::zOffsetConfirm(DGUS_VP_Variable &var, void *val_ptr) {
  settings.save();
  if (printJobOngoing())
    gotoScreen(MKSLCD_SCREEN_PRINT);
  else if (print_job_timer.isPaused)
    gotoScreen(MKSLCD_SCREEN_PAUSE);
}

void DGUSScreenHandlerMKS::getTurnOffCtrl(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t value = BE16_P(val_ptr);
  switch (value) {
    case 0 ... 1: DGUSAutoTurnOff = (bool)value; break;
    default: break;
  }
}

void DGUSScreenHandlerMKS::getMinExtrudeTemp(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t value = BE16_P(val_ptr);
  TERN_(PREVENT_COLD_EXTRUSION, thermalManager.extrude_min_temp = value);
  mks_min_extrusion_temp = value;
  settings.save();
}

void DGUSScreenHandlerMKS::getZoffsetDistance(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t value = BE16_P(val_ptr);
  float val_distance = 0;
  switch (value) {
    case 0: val_distance = 0.01; break;
    case 1: val_distance = 0.1; break;
    case 2: val_distance = 0.5; break;
    case 3: val_distance = 1; break;
    default: val_distance = 0.01; break;
  }
  ZOffset_distance = val_distance;
}

void DGUSScreenHandlerMKS::getManualMovestep(DGUS_VP_Variable &var, void *val_ptr) {
  *(uint16_t *)var.memadr = BE16_P(val_ptr);
}

void DGUSScreenHandlerMKS::eepromControl(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t eep_flag = BE16_P(val_ptr);
  switch (eep_flag) {
    case 0:
      settings.save();
      settings.load(); // load eeprom data to check the data is right
      gotoScreen(MKSLCD_SCREEN_EEP_Config);
      break;

    case 1:
      settings.reset();
      gotoScreen(MKSLCD_SCREEN_EEP_Config);
      break;

    default: break;
  }
}

void DGUSScreenHandlerMKS::zOffsetSelect(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t z = BE16_P(val_ptr);
  switch (z) {
    case 0: Z_distance = 0.01; break;
    case 1: Z_distance = 0.1; break;
    case 2: Z_distance = 0.5; break;
    default: Z_distance = 1; break;
  }
}

void DGUSScreenHandlerMKS::getOffsetValue(DGUS_VP_Variable &var, void *val_ptr) {

  #if HAS_BED_PROBE
    const int32_t value = BE32_P(val_ptr);
    const float Offset = value / 100.0f;

    switch (var.VP) {
      default: break;
        case VP_OFFSET_X: probe.offset.x = Offset; break;
        case VP_OFFSET_Y: probe.offset.y = Offset; break;
        case VP_OFFSET_Z: probe.offset.z = Offset; break;
    }
    settings.save();
  #endif
}

void DGUSScreenHandlerMKS::languageChange(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t lag_flag = BE16_P(val_ptr);
  switch (lag_flag) {
    case MKS_SimpleChinese:
      languageDisplay(MKS_SimpleChinese);
      mks_language_index = MKS_SimpleChinese;
      dgus.writeVariable(VP_LANGUAGE_CHANGE1, (uint8_t)MKS_Language_Choose);
      dgus.writeVariable(VP_LANGUAGE_CHANGE2, (uint8_t)MKS_Language_NoChoose);
      settings.save();
      break;
    case MKS_English:
      languageDisplay(MKS_English);
      mks_language_index = MKS_English;
      dgus.writeVariable(VP_LANGUAGE_CHANGE1, (uint8_t)MKS_Language_NoChoose);
      dgus.writeVariable(VP_LANGUAGE_CHANGE2, (uint8_t)MKS_Language_Choose);
      settings.save();
      break;
    default: break;
  }
}

#if ENABLED(MESH_BED_LEVELING)
  grid_count_t mesh_point_count = GRID_MAX_POINTS;
#endif

void DGUSScreenHandlerMKS::levelControl(DGUS_VP_Variable &var, void *val_ptr) {
  #if ENABLED(MESH_BED_LEVELING)
    auto cs = getCurrentScreen();
  #endif
  const uint16_t lev_but = BE16_P(val_ptr);
  switch (lev_but) {
    case 0:
      #if ENABLED(AUTO_BED_LEVELING_BILINEAR)

        static uint8_t a_first_level = 1;
        if (a_first_level == 1) {
          a_first_level = 0;
          queue.enqueue_now_P(G28_STR);
        }
        queue.enqueue_now(F("G29"));

      #elif ENABLED(MESH_BED_LEVELING)

        mesh_point_count = GRID_MAX_POINTS;

        if (mks_language_index == MKS_English) {
          const char level_buf_en[] = "Start Level";
          dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_en, 32, true);
        }
        else if (mks_language_index == MKS_SimpleChinese) {
          const uint16_t level_buf_ch[] = {0xAABF, 0xBCCA, 0xF7B5, 0xBDC6, 0x2000};
          dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_ch, 32, true);
        }

        cs = getCurrentScreen();
        if (cs != MKSLCD_AUTO_LEVEL) gotoScreen(MKSLCD_AUTO_LEVEL);
      #else

        gotoScreen(MKSLCD_SCREEN_LEVEL);

      #endif
      break;

    case 1:
      soft_endstop._enabled = true;
      gotoScreen(MKSLCD_SCREEM_TOOL);
      break;

    default: break;
  }
}

void DGUSScreenHandlerMKS::meshLevelDistanceConfig(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t mesh_dist = BE16_P(val_ptr);
  switch (mesh_dist) {
    case 0: mesh_adj_distance = 0.01; break;
    case 1: mesh_adj_distance = 0.1; break;
    case 2: mesh_adj_distance = 1; break;
    default: mesh_adj_distance = 0.1; break;
  }
}

void DGUSScreenHandlerMKS::meshLevel(DGUS_VP_Variable &var, void *val_ptr) {
  #if ENABLED(MESH_BED_LEVELING)
    const uint16_t mesh_val = BE16_P(val_ptr);
    // static uint8_t a_first_level = 1;
    char cmd_buf[30];
    float offset = mesh_adj_distance;
    int16_t integer, Deci, Deci2;

    if (!queue.ring_buffer.empty()) return;

    switch (mesh_val) {
      case 0:
        offset = mesh_adj_distance;
        integer = offset; // get int
        Deci = (offset * 10);
        Deci = Deci % 10;
        Deci2 = offset * 100;
        Deci2 = Deci2 % 10;
        soft_endstop._enabled = false;
        queue.enqueue_now(F("G91"));
        snprintf_P(cmd_buf, 30, PSTR("G1 Z%d.%d%d"), integer, Deci, Deci2);
        queue.enqueue_one_now(cmd_buf);
        queue.enqueue_now(F("G90"));
        //soft_endstop._enabled = true;
        break;

      case 1:
        offset = mesh_adj_distance;
        integer = offset;       // get int
        Deci = (offset * 10);
        Deci = Deci % 10;
        Deci2 = offset * 100;
        Deci2 = Deci2 % 10;
        soft_endstop._enabled = false;
        queue.enqueue_now(F("G91"));
        snprintf_P(cmd_buf, 30, PSTR("G1 Z-%d.%d%d"), integer, Deci, Deci2);
        queue.enqueue_one_now(cmd_buf);
        queue.enqueue_now(F("G90"));
        break;

      case 2:
        if (mesh_point_count == GRID_MAX_POINTS) { // The first point

          queue.enqueue_now(F("G28"));
          queue.enqueue_now(F("G29S1"));
          mesh_point_count--;

          if (mks_language_index == MKS_English) {
            const char level_buf_en1[] = "Next Point";
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_en1, 32, true);
          }
          else if (mks_language_index == MKS_SimpleChinese) {
            const uint16_t level_buf_ch1[] = {0xC2CF, 0xBBD2, 0xE3B5, 0x2000};
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_ch1, 32, true);
          }
        }
        else if (mesh_point_count > 1) {                              // 倒数第二个点
          queue.enqueue_now(F("G29S2"));
          mesh_point_count--;
          if (mks_language_index == MKS_English) {
            const char level_buf_en2[] = "Next Point";
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_en2, 32, true);
          }
          else if (mks_language_index == MKS_SimpleChinese) {
            const uint16_t level_buf_ch2[] = {0xC2CF, 0xBBD2, 0xE3B5, 0x2000};
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_ch2, 32, true);
          }
        }
        else if (mesh_point_count == 1) {
          queue.enqueue_now(F("G29S2"));
          mesh_point_count--;
          if (mks_language_index == MKS_English) {
            const char level_buf_en2[] = "Leveling Done";
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_en2, 32, true);
          }
          else if (mks_language_index == MKS_SimpleChinese) {
            const uint16_t level_buf_ch2[] = {0xF7B5, 0xBDC6, 0xEACD, 0xC9B3, 0x2000};
            dgus.writeVariable(VP_AutoLevel_1_Dis, level_buf_ch2, 32, true);
          }
          settings.save();
        }
        else if (mesh_point_count == 0) {
          mesh_point_count = GRID_MAX_POINTS;
          soft_endstop._enabled = true;
          settings.save();
          gotoScreen(MKSLCD_SCREEM_TOOL);
        }
        break;

      default:
        break;
    }
  #endif // MESH_BED_LEVELING
}

void DGUSScreenHandlerMKS::sdFileBack(DGUS_VP_Variable&, void*) {
  gotoScreen(MKSLCD_SCREEN_HOME);
}

void DGUSScreenHandlerMKS::lcdBLKAdjust(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t lcd_val = BE16_P(val_ptr);
  lcd_default_light = constrain(lcd_val, 10, 100);

  const uint16_t lcd_data[2] = { lcd_default_light, lcd_default_light };
  dgus.writeVariable(0x0082, &lcd_data, 5, true);
}

void DGUSScreenHandlerMKS::manualAssistLeveling(DGUS_VP_Variable &var, void *val_ptr) {
  const int16_t point_val = BE16_P(val_ptr);

  // Insist on leveling first time at this screen
  static bool first_level_flag = false;
  if (!first_level_flag || point_val == 0x0001) {
    queue.enqueue_now_P(G28_STR);
    first_level_flag = true;
  }

  constexpr uint16_t level_speed = 1500;

  auto enqueue_corner_move = [](int16_t lx, int16_t ly, uint16_t fr) {
    char buf_level[32];
    sprintf_P(buf_level, "G0X%dY%dF%d", lx, ly, fr);
    queue.enqueue_one_now(buf_level);
  };

  if (WITHIN(point_val, 0x0001, 0x0005))
    queue.enqueue_now(F("G1Z10"));

  switch (point_val) {
    case 0x0001:
      enqueue_corner_move(X_MIN_POS + ABS(mks_corner_offsets[0].x),
                          Y_MIN_POS + ABS(mks_corner_offsets[0].y), level_speed);
      queue.enqueue_now(F("G28Z"));
      break;
    case 0x0002:
      enqueue_corner_move(X_MAX_POS - ABS(mks_corner_offsets[1].x),
                          Y_MIN_POS + ABS(mks_corner_offsets[1].y), level_speed);
      break;
    case 0x0003:
      enqueue_corner_move(X_MAX_POS - ABS(mks_corner_offsets[2].x),
                          Y_MAX_POS - ABS(mks_corner_offsets[2].y), level_speed);
      break;
    case 0x0004:
      enqueue_corner_move(X_MIN_POS + ABS(mks_corner_offsets[3].x),
                          Y_MAX_POS - ABS(mks_corner_offsets[3].y), level_speed);
      break;
    case 0x0005:
      enqueue_corner_move(ABS(mks_corner_offsets[4].x),
                          ABS(mks_corner_offsets[4].y), level_speed);
      break;
  }

  if (WITHIN(point_val, 0x0002, 0x0005)) {
    //queue.enqueue_now(F("G28Z"));
    queue.enqueue_now(F("G1Z-10"));
  }
}

#define mks_min(a, b) ((a) < (b)) ? (a) : (b)
#define mks_max(a, b) ((a) > (b)) ? (a) : (b)
void DGUSScreenHandlerMKS::tmcChangeConfig(DGUS_VP_Variable &var, void *val_ptr) {
  #if ANY(HAS_TRINAMIC_CONFIG, HAS_STEALTHCHOP)
    const uint16_t tmc_val = BE16_P(val_ptr);
  #endif

  switch (var.VP) {
    case VP_TMC_X_STEP:
      #if USE_SENSORLESS
        #if X_HAS_STEALTHCHOP
          stepperX.homing_threshold(mks_min(tmc_val, 255));
          settings.save();
          //tmc_step.x = stepperX.homing_threshold();
        #endif
      #endif
      break;
    case VP_TMC_Y_STEP:
      #if USE_SENSORLESS
        #if Y_HAS_STEALTHCHOP
          stepperY.homing_threshold(mks_min(tmc_val, 255));
          settings.save();
          //tmc_step.y = stepperY.homing_threshold();
        #endif
      #endif
      break;
    case VP_TMC_Z_STEP:
      #if USE_SENSORLESS
        #if Z_HAS_STEALTHCHOP
          stepperZ.homing_threshold(mks_min(tmc_val, 255));
          settings.save();
          //tmc_step.z = stepperZ.homing_threshold();
        #endif
      #endif
      break;
    case VP_TMC_X_Current:
      #if AXIS_IS_TMC(X)
        stepperX.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_X1_Current:
      #if AXIS_IS_TMC(X2)
        stepperX2.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_Y_Current:
      #if AXIS_IS_TMC(Y)
        stepperY.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_Y1_Current:
      #if AXIS_IS_TMC(X2)
        stepperY2.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_Z_Current:
      #if AXIS_IS_TMC(Z)
        stepperZ.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_Z1_Current:
      #if AXIS_IS_TMC(Z2)
        stepperZ2.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_E0_Current:
      #if AXIS_IS_TMC(E0)
        stepperE0.rms_current(tmc_val);
        settings.save();
      #endif
      break;
    case VP_TMC_E1_Current:
      #if AXIS_IS_TMC(E1)
        stepperE1.rms_current(tmc_val);
        settings.save();
      #endif
      break;

    default:
      break;
  }
  #if USE_SENSORLESS
    TERN_(X_HAS_STEALTHCHOP, tmc_step.x = stepperX.homing_threshold());
    TERN_(Y_HAS_STEALTHCHOP, tmc_step.y = stepperY.homing_threshold());
    TERN_(Z_HAS_STEALTHCHOP, tmc_step.z = stepperZ.homing_threshold());
  #endif
}

void DGUSScreenHandler::handleManualMove(DGUS_VP_Variable &var, void *val_ptr) {
  int16_t movevalue = BE16_P(val_ptr);

  // Choose Move distance
       if (manualMoveStep == 0x01) manualMoveStep =   10;
  else if (manualMoveStep == 0x02) manualMoveStep =  100;
  else if (manualMoveStep == 0x03) manualMoveStep = 1000;

  if (!print_job_timer.isPaused() && !queue.ring_buffer.empty())
    return;

  char axiscode;
  uint16_t speed = manual_feedrate_mm_m.x; // Default feedrate for manual moves

  switch (var.VP) { // switch X Y Z or Home
    default: return;
    #if HAS_X_AXIS
      case VP_MOVE_X:
        axiscode = 'X';
        if (!ExtUI::canMove(ExtUI::axis_t::X)) goto cannotmove;
        break;
    #endif

    #if HAS_Y_AXIS
      case VP_MOVE_Y:
        axiscode = 'Y';
        speed = manual_feedrate_mm_m.y;
        if (!ExtUI::canMove(ExtUI::axis_t::Y)) goto cannotmove;
        break;
    #endif

    #if HAS_Z_AXIS
      case VP_MOVE_Z:
        axiscode = 'Z';
        speed = manual_feedrate_mm_m.z;
        if (!ExtUI::canMove(ExtUI::axis_t::Z)) goto cannotmove;
        break;
    #endif

    case VP_MOTOR_LOCK_UNLOK:
      movevalue = 5;
      break;

    case VP_HOME_ALL: // only used for homing
      axiscode = '\0';
      movevalue = 0; // ignore value sent from display, this VP is _ONLY_ for homing.
      break;

    #if HAS_X_AXIS
      case VP_X_HOME:
        axiscode = 'X';
        movevalue = 0;
        break;
    #endif

    #if HAS_Y_AXIS
      case VP_Y_HOME:
        axiscode = 'Y';
        movevalue = 0;
        break;
    #endif

    #if HAS_Z_AXIS
      case VP_Z_HOME:
        axiscode = 'Z';
        movevalue = 0;
        break;
    #endif
  }

  if (movevalue != 0 && movevalue != 5) { // get move distance
    switch (movevalue) {
      case 0x0001: movevalue =  manualMoveStep; break;
      case 0x0002: movevalue = -manualMoveStep; break;
      default:     movevalue = 0; break;
    }
  }

  if (!movevalue) {
    // char buf[6] = "G28 X";
    // buf[4] = axiscode;

    char buf[6];
    sprintf(buf, "G28 %c", axiscode);
    queue.enqueue_one_now(buf);
    forceCompleteUpdate();
    return;
  }
  else if (movevalue == 5) {
    char buf[6];
    snprintf_P(buf,6,PSTR("M84 %c"), axiscode);
    queue.enqueue_one_now(buf);
    forceCompleteUpdate();
    return;
  }
  else {
    // movement
    const bool old_relative_mode = relative_mode;
    if (!relative_mode) queue.enqueue_now(F("G91"));
    char buf[32]; // G1 X9999.99 F12345
    //const uint16_t backup_speed = MMS_TO_MMM(feedrate_mm_s);
    char sign[] = "\0";
    int16_t value = movevalue / 100;
    if (movevalue < 0) { value = -value; sign[0] = '-'; }
    const int16_t fraction = ABS(movevalue) % 100;
    snprintf_P(buf, 32, PSTR("G0 %c%s%d.%02d F%d"), axiscode, sign, value, fraction, speed);
    queue.enqueue_one_now(buf);

    //if (backup_speed != speed) {
    //  snprintf_P(buf, 32, PSTR("G0 F%d"), backup_speed);
    //  queue.enqueue_one_now(buf);
    //}

    //while (!enqueue_and_echo_command(buf)) idle();

    if (!old_relative_mode) queue.enqueue_now(F("G90"));
  }

  forceCompleteUpdate();

  cannotmove:
    return;
}

void DGUSScreenHandlerMKS::getParkPos(DGUS_VP_Variable &var, void *val_ptr) {
  const int16_t pos = BE16_P(val_ptr);

  switch (var.VP) {
    case VP_X_PARK_POS: mks_park_pos.x = pos; break;
    case VP_Y_PARK_POS: mks_park_pos.y = pos; break;
    case VP_Z_PARK_POS: mks_park_pos.z = pos; break;
    default: break;
  }
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleChangeLevelPoint(DGUS_VP_Variable &var, void *val_ptr) {
  const int16_t raw = BE16_P(val_ptr);

  *(int16_t*)var.memadr = raw;

  settings.save();
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

  void DGUSScreenHandlerMKS::handleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t raw = BE16_P(val_ptr);
    const float value = (float)raw;

    ExtUI::axis_t axis;
    switch (var.VP) {
      default: return;
      case VP_X_STEP_PER_MM: axis = ExtUI::axis_t::X; break;
      case VP_Y_STEP_PER_MM: axis = ExtUI::axis_t::Y; break;
      case VP_Z_STEP_PER_MM: axis = ExtUI::axis_t::Z; break;
    }
    ExtUI::setAxisSteps_per_mm(value, axis);
    settings.save();
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }

  void DGUSScreenHandlerMKS::handleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t raw = BE16_P(val_ptr);
    const float value = (float)raw;

    ExtUI::extruder_t extruder;
    switch (var.VP) {
      default: return;
      #if HAS_HOTEND
        case VP_E0_STEP_PER_MM: extruder = ExtUI::extruder_t::E0; break;
      #endif
      #if HAS_MULTI_HOTEND
        case VP_E1_STEP_PER_MM: extruder = ExtUI::extruder_t::E1; break;
      #endif
    }
    ExtUI::setAxisSteps_per_mm(value, extruder);
    settings.save();
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }

#endif // EDITABLE_STEPS_PER_UNIT

void DGUSScreenHandlerMKS::handleMaxSpeedChange(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t raw = BE16_P(val_ptr);
  const float value = (float)raw;

  ExtUI::axis_t axis;
  switch (var.VP) {
    case VP_X_MAX_SPEED: axis = ExtUI::axis_t::X; break;
    case VP_Y_MAX_SPEED: axis = ExtUI::axis_t::Y; break;
    case VP_Z_MAX_SPEED: axis = ExtUI::axis_t::Z; break;
    default: return;
  }
  ExtUI::setAxisMaxFeedrate_mm_s(value, axis);
  settings.save();
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleExtruderMaxSpeedChange(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t raw = BE16_P(val_ptr);
  const float value = (float)raw;

  ExtUI::extruder_t extruder;
  switch (var.VP) {
    default: return;
      #if HAS_HOTEND
        case VP_E0_MAX_SPEED: extruder = ExtUI::extruder_t::E0; break;
      #endif
      #if HAS_MULTI_HOTEND
      #endif
    case VP_E1_MAX_SPEED: extruder = ExtUI::extruder_t::E1; break;
  }
  ExtUI::setAxisMaxFeedrate_mm_s(value, extruder);
  settings.save();
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleMaxAccChange(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t raw = BE16_P(val_ptr);
  const float value = (float)raw;

  ExtUI::axis_t axis;
  switch (var.VP) {
    default: return;
    case VP_X_ACC_MAX_SPEED: axis = ExtUI::axis_t::X;  break;
    case VP_Y_ACC_MAX_SPEED: axis = ExtUI::axis_t::Y;  break;
    case VP_Z_ACC_MAX_SPEED: axis = ExtUI::axis_t::Z;  break;
  }
  ExtUI::setAxisMaxAcceleration_mm_s2(value, axis);
  settings.save();
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleExtruderAccChange(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t raw = BE16_P(val_ptr);
  float value = (float)raw;
  ExtUI::extruder_t extruder;
  switch (var.VP) {
    default: return;
    #if HAS_HOTEND
      case VP_E0_ACC_MAX_SPEED: extruder = ExtUI::extruder_t::E0; settings.load(); break;
    #endif
    #if HAS_MULTI_HOTEND
      case VP_E1_ACC_MAX_SPEED: extruder = ExtUI::extruder_t::E1; settings.load(); break;
    #endif
  }
  ExtUI::setAxisMaxAcceleration_mm_s2(value, extruder);
  settings.save();
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleTravelAccChange(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t travel = BE16_P(val_ptr);
  planner.settings.travel_acceleration = (float)travel;
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleFeedRateMinChange(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t t = BE16_P(val_ptr);
  planner.settings.min_feedrate_mm_s = (float)t;
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleMin_T_F(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t t_f = BE16_P(val_ptr);
  planner.settings.min_travel_feedrate_mm_s = (float)t_f;
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::handleAccChange(DGUS_VP_Variable &var, void *val_ptr) {
  uint16_t acc = BE16_P(val_ptr);
  planner.settings.acceleration = (float)acc;
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

#if ENABLED(PREVENT_COLD_EXTRUSION)
  void DGUSScreenHandlerMKS::handleGetExMinTemp(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t ex_min_temp = BE16_P(val_ptr);
    thermalManager.extrude_min_temp = ex_min_temp;
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }
#endif

#if HAS_PID_HEATING
  void DGUSScreenHandler::handleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t rawvalue = BE16_P(val_ptr);
    const float value = float(rawvalue);
    float newvalue = 0;

    switch (var.VP) {
      default: return;
        #if HAS_HOTEND
          case VP_E0_PID_P: newvalue = value; break;
          case VP_E0_PID_I: newvalue = scalePID_i(value); break;
          case VP_E0_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HAS_MULTI_HOTEND
          case VP_E1_PID_P: newvalue = value; break;
          case VP_E1_PID_I: newvalue = scalePID_i(value); break;
          case VP_E1_PID_D: newvalue = scalePID_d(value); break;
        #endif
        #if HAS_HEATED_BED
          case VP_BED_PID_P: newvalue = value; break;
          case VP_BED_PID_I: newvalue = scalePID_i(value); break;
          case VP_BED_PID_D: newvalue = scalePID_d(value); break;
        #endif
    }

    *(float *)var.memadr = newvalue;

    settings.save();
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }
#endif // HAS_PID_HEATING

#if ENABLED(BABYSTEPPING)
  void DGUSScreenHandler::handleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr) {
    const float step = ZOffset_distance;

    const uint16_t flag = BE16_P(val_ptr);
    switch (flag) {
      case 0:
        if (step == 0.01)
          queue.inject(F("M290 Z-0.01"));
        else if (step == 0.1)
          queue.inject(F("M290 Z-0.1"));
        else if (step == 0.5)
          queue.inject(F("M290 Z-0.5"));
        else if (step == 1)
          queue.inject(F("M290 Z-1"));
        else
          queue.inject(F("M290 Z-0.01"));

        z_offset_add -= ZOffset_distance;
        break;

      case 1:
        if (step == 0.01)
          queue.inject(F("M290 Z0.01"));
        else if (step == 0.1)
          queue.inject(F("M290 Z0.1"));
        else if (step == 0.5)
          queue.inject(F("M290 Z0.5"));
        else if (step == 1)
          queue.inject(F("M290 Z1"));
        else
          queue.inject(F("M290 Z-0.01"));

        z_offset_add += ZOffset_distance;
        break;

      default: break;
    }
    forceCompleteUpdate();
  }
#endif // BABYSTEPPING

void DGUSScreenHandlerMKS::getManualFilament(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t len = BE16_P(val_ptr);
  const float value = (float)len;

  distanceFilament = value;

  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::getManualFilamentSpeed(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t len = BE16_P(val_ptr);
  filamentSpeed_mm_s = len;

  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandlerMKS::filamentLoadUnload(DGUS_VP_Variable &var, void *val_ptr, const int filamentDir) {
  #if ANY(HAS_MULTI_HOTEND, SINGLENOZZLE)
    uint8_t swap_tool = 0;
  #else
    constexpr uint8_t swap_tool = 1; // T0 (or none at all)
  #endif

  #if HAS_HOTEND
    uint8_t hotend_too_cold = 0;
  #endif

  if (!print_job_timer.isPaused() && !queue.ring_buffer.empty())
    return;

  const uint16_t val_t = BE16_P(val_ptr);
  switch (val_t) {
    default: break;
    case 0:
      #if HAS_HOTEND
        if (thermalManager.tooColdToExtrude(0))
          hotend_too_cold = 1;
        else {
          #if ANY(HAS_MULTI_HOTEND, SINGLENOZZLE)
            swap_tool = 1;
          #endif
        }
      #endif
      break;
    case 1:
      #if HAS_MULTI_HOTEND
        if (thermalManager.tooColdToExtrude(1)) hotend_too_cold = 2; else swap_tool = 2;
      #elif ENABLED(SINGLENOZZLE)
        if (thermalManager.tooColdToExtrude(0)) hotend_too_cold = 1; else swap_tool = 2;
      #endif
      break;
  }

  #if ALL(HAS_HOTEND, PREVENT_COLD_EXTRUSION)
    if (hotend_too_cold) {
      if (thermalManager.targetTooColdToExtrude(hotend_too_cold - 1)) thermalManager.setTargetHotend(thermalManager.extrude_min_temp, hotend_too_cold - 1);
      sendInfoScreen(F("NOTICE"), nullptr, F("Please wait."), F("Nozzle heating!"), true, true, true, true);
      setupConfirmAction(nullptr);
      gotoScreen(DGUS_SCREEN_POPUP);
    }
  #endif

  if (swap_tool) {
    char buf[30];
    snprintf_P(buf, 30
      #if ANY(HAS_MULTI_HOTEND, SINGLENOZZLE)
        , PSTR("M1002T%cE%dF%d"), char('0' + swap_tool - 1)
      #else
        , PSTR("M1002E%dF%d")
      #endif
      , (int)distanceFilament * filamentDir, filamentSpeed_mm_s * 60
    );
    queue.inject(buf);
  }
}

/**
 * M1002: Do a tool-change and relative move for filamentLoadUnload
 *        within the G-code execution window for best concurrency.
 */
void GcodeSuite::M1002() {
  #if ANY(HAS_MULTI_HOTEND, SINGLENOZZLE)
  {
    char buf[3];
    sprintf_P(buf, PSTR("T%c"), char('0' + parser.intval('T')));
    process_subcommands_now(buf);
  }
  #endif

  const uint8_t old_axis_relative = axis_relative;
  set_e_relative(); // M83
  {
    char buf[20];
    snprintf_P(buf, 20, PSTR("G1E%dF%d"), parser.intval('E'), parser.intval('F'));
    process_subcommands_now(buf);
  }
  axis_relative = old_axis_relative;
}

void DGUSScreenHandlerMKS::filamentLoad(DGUS_VP_Variable &var, void *val_ptr) {
  filamentLoadUnload(var, val_ptr, 1);
}

void DGUSScreenHandlerMKS::filamentUnload(DGUS_VP_Variable &var, void *val_ptr) {
  filamentLoadUnload(var, val_ptr, -1);
}

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)

  void DGUSScreenHandler::handleFilamentOption(DGUS_VP_Variable &var, void *val_ptr) {
    uint8_t e_temp = 0;
    filament_data.heated = false;
    uint16_t preheat_option = BE16_P(val_ptr);
    if (preheat_option >= 10) {     // Unload filament type
      preheat_option -= 10;
      filament_data.action = 2;
      filament_data.purge_length = DGUS_FILAMENT_PURGE_LENGTH;
    }
    else if (preheat_option <= 8)   // Load filament type
      filament_data.action = 1;
    else                            // Cancel filament operation
      filament_data.action = 0;

    switch (preheat_option) {
      case 0: // Load PLA
        #ifdef PREHEAT_1_TEMP_HOTEND
          e_temp = PREHEAT_1_TEMP_HOTEND;
        #endif
        break;
      case 1: // Load ABS
        TERN_(PREHEAT_2_TEMP_HOTEND, e_temp = PREHEAT_2_TEMP_HOTEND);
        break;
      case 2: // Load PET
        #ifdef PREHEAT_3_TEMP_HOTEND
          e_temp = PREHEAT_3_TEMP_HOTEND;
        #endif
        break;
      case 3: // Load FLEX
        #ifdef PREHEAT_4_TEMP_HOTEND
          e_temp = PREHEAT_4_TEMP_HOTEND;
        #endif
        break;
      case 9: // Cool down
      default:
        e_temp = 0;
        break;
    }

    if (filament_data.action == 0) { // Go back to utility screen
      #if HAS_HOTEND
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E0);
      #endif
      #if HAS_MULTI_HOTEND
        thermalManager.setTargetHotend(e_temp, ExtUI::extruder_t::E1);
      #endif
      gotoScreen(DGUS_SCREEN_UTILITY);
    }
    else { // Go to the preheat screen to show the heating progress
      switch (var.VP) {
        default: return;
          #if HAS_HOTEND
            case VP_E0_FILAMENT_LOAD_UNLOAD:
              filament_data.extruder = ExtUI::extruder_t::E0;
              thermalManager.setTargetHotend(e_temp, filament_data.extruder);
              break;
          #endif
          #if HAS_MULTI_HOTEND
            case VP_E1_FILAMENT_LOAD_UNLOAD:
              filament_data.extruder = ExtUI::extruder_t::E1;
              thermalManager.setTargetHotend(e_temp, filament_data.extruder);
              break;
          #endif
      }
    }
  }

  void DGUSScreenHandler::handleFilamentLoadUnload(DGUS_VP_Variable &var) {
    if (filament_data.action <= 0) return;

    // If we close to the target temperature, we can start load or unload the filament
    if (thermalManager.hotEnoughToExtrude(filament_data.extruder) && \
        thermalManager.targetHotEnoughToExtrude(filament_data.extruder)) {
      float movevalue = DGUS_FILAMENT_LOAD_LENGTH_PER_TIME;

      if (filament_data.action == 1) { // load filament
        if (!filament_data.heated) {
          filament_data.heated = true;
        }
        movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
      }
      else { // unload filament
        if (!filament_data.heated) {
          gotoScreen(DGUS_SCREEN_FILAMENT_UNLOADING);
          filament_data.heated = true;
        }
        // Before unloading extrude to prevent jamming
        if (filament_data.purge_length >= 0) {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) + movevalue;
          filament_data.purge_length -= movevalue;
        }
        else {
          movevalue = ExtUI::getAxisPosition_mm(filament_data.extruder) - movevalue;
        }
      }
      ExtUI::setAxisPosition_mm(movevalue, filament_data.extruder);
    }
  }

#endif // DGUS_FILAMENT_LOADUNLOAD

bool DGUSScreenHandlerMKS::loop() {
  dgus.loop();

  const millis_t ms = millis();
  static millis_t next_event_ms = 0;

  static uint8_t language_times = 2;

  if (!isScreenComplete() || ELAPSED(ms, next_event_ms)) {
    next_event_ms = ms + DGUS_UPDATE_INTERVAL_MS;
    updateScreenVPData();
  }

  if (language_times != 0) {
    languagePInit();
    languageDisplay(mks_language_index);
    language_times--;
  }

  #if ENABLED(SHOW_BOOTSCREEN)
    static bool booted = false;
    if (!booted && ELAPSED(ms, TERN(USE_MKS_GREEN_UI, 1000, BOOTSCREEN_TIMEOUT))) {
      booted = true;
      #if USE_SENSORLESS
        TERN_(X_HAS_STEALTHCHOP, tmc_step.x = stepperX.homing_threshold());
        TERN_(Y_HAS_STEALTHCHOP, tmc_step.y = stepperY.homing_threshold());
        TERN_(Z_HAS_STEALTHCHOP, tmc_step.z = stepperZ.homing_threshold());
      #endif

      #if ENABLED(PREVENT_COLD_EXTRUSION)
        if (mks_min_extrusion_temp != 0)
          thermalManager.extrude_min_temp = mks_min_extrusion_temp;
      #endif

      extrudeLoadInit();

      TERN_(DGUS_MKS_RUNOUT_SENSOR, runoutInit());

      if (TERN0(POWER_LOSS_RECOVERY, recovery.valid()))
        gotoScreen(DGUS_SCREEN_POWER_LOSS);
      else
        gotoScreen(DGUS_SCREEN_MAIN);
    }

    #if ENABLED(DGUS_MKS_RUNOUT_SENSOR)
      if (booted && printingIsActive()) runoutIdle();
    #endif
  #endif // SHOW_BOOTSCREEN

  return isScreenComplete();
}

void DGUSScreenHandlerMKS::languagePInit() {
  switch (mks_language_index) {
    case MKS_SimpleChinese:
      dgus.writeVariable(VP_LANGUAGE_CHANGE1, (uint8_t)MKS_Language_Choose);
      dgus.writeVariable(VP_LANGUAGE_CHANGE2, (uint8_t)MKS_Language_NoChoose);
      break;
    case MKS_English:
      dgus.writeVariable(VP_LANGUAGE_CHANGE1, (uint8_t)MKS_Language_NoChoose);
      dgus.writeVariable(VP_LANGUAGE_CHANGE2, (uint8_t)MKS_Language_Choose);
      break;
    default:
      break;
  }
}

void DGUSScreenHandlerMKS::extrudeLoadInit() {
  ex_filament.ex_length           = distanceFilament;
  ex_filament.ex_load_unload_flag = 0;
  ex_filament.ex_need_time        = filamentSpeed_mm_s;
  ex_filament.ex_speed            = 0;
  ex_filament.ex_status           = EX_NONE;
  ex_filament.ex_tick_end         = 0;
  ex_filament.ex_tick_start       = 0;
}

void DGUSScreenHandlerMKS::runoutInit() {
  #if ENABLED(DGUS_MKS_RUNOUT_SENSOR) && PIN_EXISTS(FIL_RUNOUT)
    SET_INPUT_PULLUP(FIL_RUNOUT_PIN);
  #endif
  runout_mks.de_count      = 0;
  runout_mks.de_times      = 10;
  runout_mks.pin_status    = 1;
  runout_mks.runout_status = UNRUNOUT_STATUS;
}

void DGUSScreenHandlerMKS::runoutIdle() {
  #if ENABLED(DGUS_MKS_RUNOUT_SENSOR)
    // scanf runout pin
    switch (runout_mks.runout_status) {

      case RUNOUT_STATUS:
        runout_mks.runout_status = RUNOUT_BEGIN_STATUS;
        queue.inject(F("M25"));
        gotoScreen(MKSLCD_SCREEN_PAUSE);

        sendInfoScreen(F("NOTICE"), nullptr, F("Please change filament!"), nullptr, true, true, true, true);
        //setupConfirmAction(nullptr);
        gotoScreen(DGUS_SCREEN_POPUP);
        break;

      case UNRUNOUT_STATUS:
        if (FILAMENT_IS_OUT())
          runout_mks.runout_status = RUNOUT_STATUS;
        break;

      case RUNOUT_BEGIN_STATUS:
        if (!FILAMENT_IS_OUT())
          runout_mks.runout_status = RUNOUT_WAITING_STATUS;
        break;

      case RUNOUT_WAITING_STATUS:
        if (FILAMENT_IS_OUT())
          runout_mks.runout_status = RUNOUT_BEGIN_STATUS;
        break;

      default: break;
    }
  #endif
}

void DGUSScreenHandlerMKS::languageDisplay(uint8_t var) {
  if (var == MKS_English) {
    const char home_buf_en[] = "Home";
    dgus.writeVariable(VP_HOME_Dis, home_buf_en, 32, true);

    const char setting_buf_en[] = "Setting";
    dgus.writeVariable(VP_Setting_Dis, setting_buf_en, 32, true);

    const char Tool_buf_en[] = "Tool";
    dgus.writeVariable(VP_Tool_Dis, Tool_buf_en, 32, true);

    const char Print_buf_en[] = "Print";
    dgus.writeVariable(VP_Print_Dis, Print_buf_en, 32, true);

    const char Language_buf_en[] = "Language";
    dgus.writeVariable(VP_Language_Dis, Language_buf_en, 32, true);

    const char About_buf_en[] = "About";
    dgus.writeVariable(VP_About_Dis, About_buf_en, 32, true);

    const char Config_buf_en[] = "Config";
    dgus.writeVariable(VP_Config_Dis, Config_buf_en, 32, true);

    const char MotorConfig_buf_en[] = "MotorConfig";
    dgus.writeVariable(VP_MotorConfig_Dis, MotorConfig_buf_en, 32, true);

    const char LevelConfig_buf_en[] = "LevelConfig";
    dgus.writeVariable(VP_LevelConfig_Dis, LevelConfig_buf_en, 32, true);

    const char TemperatureConfig_buf_en[] = "Temperature";
    dgus.writeVariable(VP_TemperatureConfig_Dis, TemperatureConfig_buf_en, 32, true);

    const char Advance_buf_en[] = "Advance";
    dgus.writeVariable(VP_Advance_Dis, Advance_buf_en, 32, true);

    const char Filament_buf_en[] = "Extrude";
    dgus.writeVariable(VP_Filament_Dis, Filament_buf_en, 32, true);

    const char Move_buf_en[] = "Move";
    dgus.writeVariable(VP_Move_Dis, Move_buf_en, 32, true);

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      const char Level_buf_en[] = "AutoLevel";
      dgus.writeVariable(VP_Level_Dis, Level_buf_en, 32, true);
    #elif ENABLED(MESH_BED_LEVELING)
      const char Level_buf_en[] = "MeshLevel";
      dgus.writeVariable(VP_Level_Dis, Level_buf_en, 32, true);
    #else
      const char Level_buf_en[] = "Level";
      dgus.writeVariable(VP_Level_Dis, Level_buf_en, 32, true);
    #endif

    const char MotorPluse_buf_en[] = "MotorPluse";
    dgus.writeVariable(VP_MotorPluse_Dis, MotorPluse_buf_en, 32, true);

    const char MotorMaxSpeed_buf_en[] = "MotorMaxSpeed";
    dgus.writeVariable(VP_MotorMaxSpeed_Dis, MotorMaxSpeed_buf_en, 32, true);

    const char MotorMaxAcc_buf_en[] = "MotorAcc";
    dgus.writeVariable(VP_MotorMaxAcc_Dis, MotorMaxAcc_buf_en, 32, true);

    const char TravelAcc_buf_en[] = "TravelAcc";
    dgus.writeVariable(VP_TravelAcc_Dis, TravelAcc_buf_en, 32, true);

    const char FeedRateMin_buf_en[] = "FeedRateMin";
    dgus.writeVariable(VP_FeedRateMin_Dis, FeedRateMin_buf_en, 32, true);

    const char TravelFeeRateMin_buf_en[] = "TravelFeedRateMin";
    dgus.writeVariable(VP_TravelFeeRateMin_Dis, TravelFeeRateMin_buf_en, 32, true);

    const char Acc_buf_en[] = "Acc";
    dgus.writeVariable(VP_ACC_Dis, Acc_buf_en, 32, true);

    const char Point_One_buf_en[] = "Point_First";
    dgus.writeVariable(VP_Point_One_Dis, Point_One_buf_en, 32, true);

    const char Point_Two_buf_en[] = "Point_Second";
    dgus.writeVariable(VP_Point_Two_Dis, Point_Two_buf_en, 32, true);

    const char Point_Three_buf_en[] = "Point_Third";
    dgus.writeVariable(VP_Point_Three_Dis, Point_Three_buf_en, 32, true);

    const char Point_Four_buf_en[] = "Point_Fourth";
    dgus.writeVariable(VP_Point_Four_Dis, Point_Four_buf_en, 32, true);

    const char Point_Five_buf_en[] = "Point_Fifth";
    dgus.writeVariable(VP_Point_Five_Dis, Point_Five_buf_en, 32, true);

    const char Extrusion_buf_en[] = "Extrusion";
    dgus.writeVariable(VP_Extrusion_Dis, Extrusion_buf_en, 32, true);

    const char HeatBed_buf_en[] = "HeatBed";
    dgus.writeVariable(VP_HeatBed_Dis, HeatBed_buf_en, 32, true);

    const char FactoryDefaults_buf_en[] = "FactoryDefaults";
    dgus.writeVariable(VP_FactoryDefaults_Dis, FactoryDefaults_buf_en, 32, true);

    const char StoreSetting_buf_en[] = "StoreSetting";
    dgus.writeVariable(VP_StoreSetting_Dis, StoreSetting_buf_en, 32, true);

    const char PrintPauseConfig_buf_en[] = "PrintPauseConfig";
    dgus.writeVariable(VP_PrintPauseConfig_Dis, PrintPauseConfig_buf_en, 32, true);

    const char X_Pluse_buf_en[] = "X_Pluse";
    dgus.writeVariable(VP_X_Pluse_Dis, X_Pluse_buf_en, 32, true);

    const char Y_Pluse_buf_en[] = "Y_Pluse";
    dgus.writeVariable(VP_Y_Pluse_Dis, Y_Pluse_buf_en, 32, true);

    const char Z_Pluse_buf_en[] = "Z_Pluse";
    dgus.writeVariable(VP_Z_Pluse_Dis, Z_Pluse_buf_en, 32, true);

    const char E0_Pluse_buf_en[] = "E0_Pluse";
    dgus.writeVariable(VP_E0_Pluse_Dis, E0_Pluse_buf_en, 32, true);

    const char E1_Pluse_buf_en[] = "E1_Pluse";
    dgus.writeVariable(VP_E1_Pluse_Dis, E1_Pluse_buf_en, 32, true);

    const char X_Max_Speed_buf_en[] = "X_Max_Speed";
    dgus.writeVariable(VP_X_Max_Speed_Dis, X_Max_Speed_buf_en, 32, true);

    const char Y_Max_Speed_buf_en[] = "Y_Max_Speed";
    dgus.writeVariable(VP_Y_Max_Speed_Dis, Y_Max_Speed_buf_en, 32, true);

    const char Z_Max_Speed_buf_en[] = "Z_Max_Speed";
    dgus.writeVariable(VP_Z_Max_Speed_Dis, Z_Max_Speed_buf_en, 32, true);

    const char E0_Max_Speed_buf_en[] = "E0_Max_Speed";
    dgus.writeVariable(VP_E0_Max_Speed_Dis, E0_Max_Speed_buf_en, 32, true);

    const char E1_Max_Speed_buf_en[] = "E1_Max_Speed";
    dgus.writeVariable(VP_E1_Max_Speed_Dis, E1_Max_Speed_buf_en, 32, true);

    const char X_Max_Acc_Speed_buf_en[] = "X_Max_Acc_Speed";
    dgus.writeVariable(VP_X_Max_Acc_Speed_Dis, X_Max_Acc_Speed_buf_en, 32, true);

    const char Y_Max_Acc_Speed_buf_en[] = "Y_Max_Acc_Speed";
    dgus.writeVariable(VP_Y_Max_Acc_Speed_Dis, Y_Max_Acc_Speed_buf_en, 32, true);

    const char Z_Max_Acc_Speed_buf_en[] = "Z_Max_Acc_Speed";
    dgus.writeVariable(VP_Z_Max_Acc_Speed_Dis, Z_Max_Acc_Speed_buf_en, 32, true);

    const char E0_Max_Acc_Speed_buf_en[] = "E0_Max_Acc_Speed";
    dgus.writeVariable(VP_E0_Max_Acc_Speed_Dis, E0_Max_Acc_Speed_buf_en, 32, true);

    const char E1_Max_Acc_Speed_buf_en[] = "E1_Max_Acc_Speed";
    dgus.writeVariable(VP_E1_Max_Acc_Speed_Dis, E1_Max_Acc_Speed_buf_en, 32, true);

    const char X_PARK_POS_buf_en[] = "X_PARK_POS";
    dgus.writeVariable(VP_X_PARK_POS_Dis, X_PARK_POS_buf_en, 32, true);

    const char Y_PARK_POS_buf_en[] = "Y_PARK_POS";
    dgus.writeVariable(VP_Y_PARK_POS_Dis, Y_PARK_POS_buf_en, 32, true);

    const char Z_PARK_POS_buf_en[] = "Z_PARK_POS";
    dgus.writeVariable(VP_Z_PARK_POS_Dis, Z_PARK_POS_buf_en, 32, true);

    const char Length_buf_en[] = "Length";
    dgus.writeVariable(VP_Length_Dis, Length_buf_en, 32, true);

    const char Speed_buf_en[] = "Speed";
    dgus.writeVariable(VP_Speed_Dis, Speed_buf_en, 32, true);

    const char InOut_buf_en[] = "InOut";
    dgus.writeVariable(VP_InOut_Dis, InOut_buf_en, 32, true);

    const char PrintTimet_buf_en[] = "PrintTime";
    dgus.writeVariable(VP_PrintTime_Dis, PrintTimet_buf_en, 32, true);

    const char E0_Temp_buf_en[] = "E0_Temp";
    dgus.writeVariable(VP_E0_Temp_Dis, E0_Temp_buf_en, 32, true);

    const char E1_Temp_buf_en[] = "E1_Temp";
    dgus.writeVariable(VP_E1_Temp_Dis, E1_Temp_buf_en, 32, true);

    const char HB_Temp_buf_en[] = "HB_Temp";
    dgus.writeVariable(VP_HB_Temp_Dis, HB_Temp_buf_en, 32, true);

    const char Feedrate_buf_en[] = "Feedrate";
    dgus.writeVariable(VP_Feedrate_Dis, Feedrate_buf_en, 32, true);

    const char PrintAcc_buf_en[] = "PrintSpeed";
    dgus.writeVariable(VP_PrintAcc_Dis, PrintAcc_buf_en, 32, true);

    const char FAN_Speed_buf_en[] = "FAN_Speed";
    dgus.writeVariable(VP_Fan_Speed_Dis, FAN_Speed_buf_en, 32, true);

    const char Printing_buf_en[] = "Printing";
    dgus.writeVariable(VP_Printing_Dis, Printing_buf_en, 32, true);

    const char Info_EEPROM_1_buf_en[] = "Store setting?";
    dgus.writeVariable(VP_Info_EEPROM_1_Dis, Info_EEPROM_1_buf_en, 32, true);

    const char Info_EEPROM_2_buf_en[] = "Revert setting?";
    dgus.writeVariable(VP_Info_EEPROM_2_Dis, Info_EEPROM_2_buf_en, 32, true);

    const char Info_PrintFinish_1_buf_en[] = "Print Done";
    dgus.writeVariable(VP_Info_PrintFinish_1_Dis, Info_PrintFinish_1_buf_en, 32, true);

    const char TMC_X_Step_buf_en[] = "X_SenSitivity";
    dgus.writeVariable(VP_TMC_X_Step_Dis, TMC_X_Step_buf_en, 32, true);

    const char TMC_Y_Step_buf_en[] = "Y_SenSitivity";
    dgus.writeVariable(VP_TMC_Y_Step_Dis, TMC_Y_Step_buf_en, 32, true);

    const char TMC_Z_Step_buf_en[] = "Z_SenSitivity";
    dgus.writeVariable(VP_TMC_Z_Step_Dis, TMC_Z_Step_buf_en, 32, true);

    const char TMC_X_Current_buf_en[] = "X_Current";
    dgus.writeVariable(VP_TMC_X_Current_Dis, TMC_X_Current_buf_en, 32, true);

    const char TMC_Y_Current_buf_en[] = "Y_Current";
    dgus.writeVariable(VP_TMC_Y_Current_Dis, TMC_Y_Current_buf_en, 32, true);

    const char TMC_Z_Current_buf_en[] = "Z_Current";
    dgus.writeVariable(VP_TMC_Z_Current_Dis, TMC_Z_Current_buf_en, 32, true);

    const char TMC_E0_Current_buf_en[] = "E0_Current";
    dgus.writeVariable(VP_TMC_E0_Current_Dis, TMC_E0_Current_buf_en, 32, true);

    const char TMC_X1_Current_buf_en[] = "X1_Current";
    dgus.writeVariable(VP_TMC_X1_Current_Dis, TMC_X1_Current_buf_en, 32, true);

    const char TMC_Y1_Current_buf_en[] = "Y1_Current";
    dgus.writeVariable(VP_TMC_Y1_Current_Dis, TMC_Y1_Current_buf_en, 32, true);

    const char TMC_Z1_Current_buf_en[] = "Z1_Current";
    dgus.writeVariable(VP_TMC_Z1_Current_Dis, TMC_Z1_Current_buf_en, 32, true);

    const char TMC_E1_Current_buf_en[] = "E1_Current";
    dgus.writeVariable(VP_TMC_E1_Current_Dis, TMC_E1_Current_buf_en, 32, true);

    const char Min_Ex_Temp_buf_en[] = "Min_Ex_Temp";
    dgus.writeVariable(VP_Min_Ex_Temp_Dis, Min_Ex_Temp_buf_en, 32, true);

    const char AutoLEVEL_INFO1_buf_en[] = "Please Press Button!";
    dgus.writeVariable(VP_AutoLEVEL_INFO1, AutoLEVEL_INFO1_buf_en, 32, true);

    const char EX_TEMP_INFO2_buf_en[] = "Please wait a monent";
    dgus.writeVariable(VP_EX_TEMP_INFO2_Dis, EX_TEMP_INFO2_buf_en, 32, true);

    const char EX_TEMP_INFO3_buf_en[] = "Cancle";
    dgus.writeVariable(VP_EX_TEMP_INFO3_Dis, EX_TEMP_INFO3_buf_en, 32, true);

    const char PrintConfrim_Info_buf_en[] = "Start Print?";
    dgus.writeVariable(VP_PrintConfrim_Info_Dis, PrintConfrim_Info_buf_en, 32, true);

    const char StopPrintConfrim_Info_buf_en[] = "Stop Print?";
    dgus.writeVariable(VP_StopPrintConfrim_Info_Dis, StopPrintConfrim_Info_buf_en, 32, true);

    const char Printting_buf_en[] = "Printing";
    dgus.writeVariable(VP_Printting_Dis, Printting_buf_en, 32, true);

    const char LCD_BLK_buf_en[] = "Backlight";
    dgus.writeVariable(VP_LCD_BLK_Dis, LCD_BLK_buf_en, 32, true);
  }
  else if (var == MKS_SimpleChinese) {
    uint16_t home_buf_ch[] = { 0xF7D6, 0xB3D2 };
    dgus.writeVariable(VP_HOME_Dis, home_buf_ch, 4, true);

    const uint16_t Setting_Dis[] = { 0xE8C9, 0xC3D6, 0x2000, 0x2000, 0x2000 };
    dgus.writeVariable(VP_Setting_Dis, Setting_Dis, 7, true);

    const uint16_t Tool_Dis[] = { 0xA4B9, 0xDFBE };
    dgus.writeVariable(VP_Tool_Dis, Tool_Dis, 4, true);

    const uint16_t Print_buf_ch[] = { 0xF2B4, 0xA1D3, 0x2000 };
    dgus.writeVariable(VP_Print_Dis, Print_buf_ch, 6, true);

    const uint16_t Language_buf_ch[] = { 0xEFD3, 0xD4D1, 0x2000, 0x2000 };
    dgus.writeVariable(VP_Language_Dis, Language_buf_ch, 8, true);

    const uint16_t About_buf_ch[] = { 0xD8B9, 0xDAD3, 0x2000 };
    dgus.writeVariable(VP_About_Dis, About_buf_ch, 6, true);

    const uint16_t Config_buf_ch[] = { 0xE4C5, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_Config_Dis, Config_buf_ch, 6, true);

    const uint16_t MotorConfig_buf_ch[] = { 0xE7B5, 0xFABB, 0xE4C5, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_MotorConfig_Dis, MotorConfig_buf_ch, 12, true);

    const uint16_t LevelConfig_buf_ch[] = { 0xD6CA, 0xAFB6, 0xF7B5, 0xBDC6, 0xE8C9, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_LevelConfig_Dis, LevelConfig_buf_ch, 32, true);

    const uint16_t TemperatureConfig_buf_ch[] = { 0xC2CE, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TemperatureConfig_Dis, TemperatureConfig_buf_ch, 11, true);

    const uint16_t Advance_buf_ch[] = { 0xDFB8, 0xB6BC, 0xE8C9, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_Advance_Dis, Advance_buf_ch, 16, true);

    const uint16_t Filament_buf_ch[] = { 0xB7BC, 0xF6B3, 0x2000 };
    dgus.writeVariable(VP_Filament_Dis, Filament_buf_ch, 8, true);

    const uint16_t Move_buf_ch[] = { 0xC6D2, 0xAFB6, 0x2000  };
    dgus.writeVariable(VP_Move_Dis, Move_buf_ch, 4, true);

    #if ENABLED(AUTO_BED_LEVELING_BILINEAR)
      const uint16_t Level_buf_ch[] = { 0xD4D7, 0xAFB6, 0xF7B5, 0xBDC6, 0x2000 };
      dgus.writeVariable(VP_Level_Dis, Level_buf_ch, 32, true);
    #elif ENABLED(MESH_BED_LEVELING)
      const uint16_t Level_buf_ch[] = { 0xF8CD, 0xF1B8, 0xF7B5, 0xBDC6, 0x2000 };
      dgus.writeVariable(VP_Level_Dis, Level_buf_ch, 32, true);
    #else
      const uint16_t Level_buf_ch[] = { 0xD6CA, 0xAFB6, 0xF7B5, 0xBDC6, 0x2000 };
      dgus.writeVariable(VP_Level_Dis, Level_buf_ch, 32, true);
    #endif

    const uint16_t MotorPluse_buf_ch[] = { 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_MotorPluse_Dis, MotorPluse_buf_ch, 16, true);

    const uint16_t MotorMaxSpeed_buf_ch[] = { 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_MotorMaxSpeed_Dis, MotorMaxSpeed_buf_ch, 16, true);

    const uint16_t MotorMaxAcc_buf_ch[] = { 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_MotorMaxAcc_Dis, MotorMaxAcc_buf_ch, 16, true);

    const uint16_t TravelAcc_buf_ch[] = { 0xD5BF, 0xD0D0, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TravelAcc_Dis, TravelAcc_buf_ch, 16, true);

    const uint16_t FeedRateMin_buf_ch[] = { 0xEED7, 0xA1D0, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_FeedRateMin_Dis, FeedRateMin_buf_ch, 12, true);

    const uint16_t TravelFeeRateMin_buf_ch[] = { 0xD5BF, 0xD0D0, 0xEED7, 0xA1D0, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TravelFeeRateMin_Dis, TravelFeeRateMin_buf_ch, 24, true);

    const uint16_t Acc_buf_ch[] = { 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_ACC_Dis, Acc_buf_ch, 16, true);

    const uint16_t Point_One_buf_ch[] = { 0xDAB5, 0xBBD2, 0xE3B5, 0x2000 };
    dgus.writeVariable(VP_Point_One_Dis, Point_One_buf_ch, 12, true);

    const uint16_t Point_Two_buf_ch[] = { 0xDAB5, 0xFEB6, 0xE3B5, 0x2000 };
    dgus.writeVariable(VP_Point_Two_Dis, Point_Two_buf_ch, 12, true);

    const uint16_t Point_Three_buf_ch[] = { 0xDAB5, 0xFDC8, 0xE3B5, 0x2000 };
    dgus.writeVariable(VP_Point_Three_Dis, Point_Three_buf_ch, 12, true);

    const uint16_t Point_Four_buf_ch[] = { 0xDAB5, 0xC4CB, 0xE3B5, 0x2000 };
    dgus.writeVariable(VP_Point_Four_Dis, Point_Four_buf_ch, 12, true);

    const uint16_t Point_Five_buf_ch[] = { 0xDAB5, 0xE5CE, 0xE3B5, 0x2000 };
    dgus.writeVariable(VP_Point_Five_Dis, Point_Five_buf_ch, 12, true);

    const uint16_t Extrusion_buf_ch[] = { 0xB7BC, 0xF6B3, 0xB7CD, 0x2000 };
    dgus.writeVariable(VP_Extrusion_Dis, Extrusion_buf_ch, 12, true);

    const uint16_t HeatBed_buf_ch[] = { 0xC8C8, 0xB2B4, 0x2000 };
    dgus.writeVariable(VP_HeatBed_Dis, HeatBed_buf_ch, 12, true);

    const uint16_t FactoryDefaults_buf_ch[] = { 0xD6BB, 0xB4B8, 0xF6B3, 0xA7B3, 0xE8C9, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_FactoryDefaults_Dis, FactoryDefaults_buf_ch, 16, true);

    const uint16_t StoreSetting_buf_ch[] = { 0xA3B1, 0xE6B4, 0xE8C9, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_StoreSetting_Dis, StoreSetting_buf_ch, 16, true);

    const uint16_t PrintPauseConfig_buf_ch[] = { 0xDDD4, 0xA3CD, 0xBBCE, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_PrintPauseConfig_Dis, PrintPauseConfig_buf_ch, 32, true);

    const uint16_t X_Pluse_buf_ch[] = { 0x2058, 0xE1D6, 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_X_Pluse_Dis, X_Pluse_buf_ch, 16, true);

    const uint16_t Y_Pluse_buf_ch[] = { 0x2059, 0xE1D6, 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_Y_Pluse_Dis, Y_Pluse_buf_ch, 16, true);

    const uint16_t Z_Pluse_buf_ch[] = { 0x205A, 0xE1D6, 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_Z_Pluse_Dis, Z_Pluse_buf_ch, 16, true);

    const uint16_t E0_Pluse_buf_ch[] = { 0x3045, 0xE1D6, 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_E0_Pluse_Dis, E0_Pluse_buf_ch, 16, true);

    const uint16_t E1_Pluse_buf_ch[] = { 0x3145, 0xE1D6, 0xF6C2, 0xE5B3, 0x2000 };
    dgus.writeVariable(VP_E1_Pluse_Dis, E1_Pluse_buf_ch, 16, true);

    const uint16_t X_Max_Speed_buf_ch[] = { 0x2058, 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_X_Max_Speed_Dis, X_Max_Speed_buf_ch, 16, true);

    const uint16_t Y_Max_Speed_buf_ch[] = { 0x2059, 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Y_Max_Speed_Dis, Y_Max_Speed_buf_ch, 16, true);

    const uint16_t Z_Max_Speed_buf_ch[] = { 0x205A, 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Z_Max_Speed_Dis, Z_Max_Speed_buf_ch, 16, true);

    const uint16_t E0_Max_Speed_buf_ch[] = { 0x3045, 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E0_Max_Speed_Dis, E0_Max_Speed_buf_ch, 16, true);

    const uint16_t E1_Max_Speed_buf_ch[] = { 0x3145, 0xEED7, 0xF3B4, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E1_Max_Speed_Dis, E1_Max_Speed_buf_ch, 16, true);

    const uint16_t X_Max_Acc_Speed_buf_ch[] = { 0x2058, 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_X_Max_Acc_Speed_Dis, X_Max_Acc_Speed_buf_ch, 16, true);

    const uint16_t Y_Max_Acc_Speed_buf_ch[] = { 0x2059, 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Y_Max_Acc_Speed_Dis, Y_Max_Acc_Speed_buf_ch, 16, true);

    const uint16_t Z_Max_Acc_Speed_buf_ch[] = { 0x205A, 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Z_Max_Acc_Speed_Dis, Z_Max_Acc_Speed_buf_ch, 16, true);

    const uint16_t E0_Max_Acc_Speed_buf_ch[] = { 0x3045, 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E0_Max_Acc_Speed_Dis, E0_Max_Acc_Speed_buf_ch, 16, true);

    const uint16_t E1_Max_Acc_Speed_buf_ch[] = { 0x3145, 0xEED7, 0xF3B4, 0xD3BC, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E1_Max_Acc_Speed_Dis, E1_Max_Acc_Speed_buf_ch, 16, true);

    const uint16_t X_PARK_POS_buf_ch[] = { 0x2058, 0xDDD4, 0xA3CD, 0xBBCE, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_X_PARK_POS_Dis, X_PARK_POS_buf_ch, 16, true);

    const uint16_t Y_PARK_POS_buf_ch[] = { 0x2059, 0xDDD4, 0xA3CD, 0xBBCE, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_Y_PARK_POS_Dis, Y_PARK_POS_buf_ch, 16, true);

    const uint16_t Z_PARK_POS_buf_ch[] = { 0x205A, 0xDDD4, 0xA3CD, 0xBBCE, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_Z_PARK_POS_Dis, Z_PARK_POS_buf_ch, 16, true);

    const uint16_t Length_buf_ch[] = { 0xBDB2, 0xA4B3, 0x2000 };
    dgus.writeVariable(VP_Length_Dis, Length_buf_ch, 8, true);

    const uint16_t Speed_buf_ch[] = { 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Speed_Dis, Speed_buf_ch, 8, true);

    const uint16_t InOut_buf_ch[] = { 0xF8BD, 0xF6B3, 0x2000 };
    dgus.writeVariable(VP_InOut_Dis, InOut_buf_ch, 8, true);

    const uint16_t PrintTimet_buf_en[] = { 0xF2B4, 0xA1D3, 0xB1CA, 0xE4BC, 0x2000 };
    dgus.writeVariable(VP_PrintTime_Dis, PrintTimet_buf_en, 16, true);

    const uint16_t E0_Temp_buf_ch[] = { 0x3045, 0xC2CE, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E0_Temp_Dis, E0_Temp_buf_ch, 16, true);

    const uint16_t E1_Temp_buf_ch[] = { 0x3145, 0xC2CE, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_E1_Temp_Dis, E1_Temp_buf_ch, 16, true);

    const uint16_t HB_Temp_buf_ch[] = { 0xC8C8, 0xB2B4, 0xC2CE, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_HB_Temp_Dis, HB_Temp_buf_ch, 16, true);

    const uint16_t Feedrate_buf_ch[] = { 0xB7BC, 0xF6B3, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Feedrate_Dis, Feedrate_buf_ch, 16, true);

    const uint16_t PrintAcc_buf_ch[] = { 0xF2B4, 0xA1D3, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_PrintAcc_Dis, PrintAcc_buf_ch, 16, true);

    const uint16_t FAN_Speed_buf_ch[] = { 0xE7B7, 0xC8C9, 0xD9CB, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Fan_Speed_Dis, FAN_Speed_buf_ch, 16, true);

    const uint16_t Printing_buf_ch[] = { 0xF2B4, 0xA1D3, 0xD0D6, 0x2000 };
    dgus.writeVariable(VP_Printing_Dis, Printing_buf_ch, 16, true);

    const uint16_t Info_EEPROM_1_buf_ch[] = { 0xC7CA, 0xF1B7, 0xA3B1, 0xE6B4, 0xE8C9, 0xC3D6, 0xBFA3, 0x2000 };
    dgus.writeVariable(VP_Info_EEPROM_1_Dis, Info_EEPROM_1_buf_ch, 32, true);

    const uint16_t Info_EEPROM_2_buf_ch[] = { 0xC7CA, 0xF1B7, 0xD6BB, 0xB4B8, 0xF6B3, 0xA7B3, 0xE8C9, 0xC3D6, 0xBFA3, 0x2000 };
    dgus.writeVariable(VP_Info_EEPROM_2_Dis, Info_EEPROM_2_buf_ch, 32, true);

    const uint16_t TMC_X_Step_buf_ch[] = { 0x2058, 0xE9C1, 0xF4C3, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TMC_X_Step_Dis, TMC_X_Step_buf_ch, 16, true);

    const uint16_t TMC_Y_Step_buf_ch[] = { 0x2059, 0xE9C1, 0xF4C3, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TMC_Y_Step_Dis, TMC_Y_Step_buf_ch, 16, true);

    const uint16_t TMC_Z_Step_buf_ch[] = { 0x205A, 0xE9C1, 0xF4C3, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_TMC_Z_Step_Dis, TMC_Z_Step_buf_ch, 16, true);

    const uint16_t Info_PrintFinish_1_buf_ch[] = { 0xF2B4, 0xA1D3, 0xEACD, 0xC9B3, 0x2000 };
    dgus.writeVariable(VP_Info_PrintFinish_1_Dis, Info_PrintFinish_1_buf_ch, 32, true);

    const uint16_t TMC_X_Current_buf_ch[] = { 0x2058, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_X_Current_Dis, TMC_X_Current_buf_ch, 16, true);

    const uint16_t TMC_Y_Current_buf_ch[] = { 0x2059, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_Y_Current_Dis, TMC_Y_Current_buf_ch, 16, true);

    const uint16_t TMC_Z_Current_buf_ch[] = { 0x205A, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_Z_Current_Dis, TMC_Z_Current_buf_ch, 16, true);

    const uint16_t TMC_E0_Current_buf_ch[] = { 0x3045, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_E0_Current_Dis, TMC_E0_Current_buf_ch, 16, true);

    const uint16_t TMC_X1_Current_buf_ch[] = { 0x3158, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_X1_Current_Dis, TMC_X1_Current_buf_ch, 16, true);

    const uint16_t TMC_Y1_Current_buf_ch[] = { 0x3159, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_Y1_Current_Dis, TMC_Y1_Current_buf_ch, 16, true);

    const uint16_t TMC_Z1_Current_buf_ch[] = { 0x315A, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_Z1_Current_Dis, TMC_Z1_Current_buf_ch, 16, true);

    const uint16_t TMC_E1_Current_buf_ch[] = { 0x3145, 0xE1D6, 0xE7B5, 0xF7C1, 0x2000 };
    dgus.writeVariable(VP_TMC_E1_Current_Dis, TMC_E1_Current_buf_ch, 16, true);

    const uint16_t Min_Ex_Temp_buf_ch[] = { 0xEED7, 0xA1D0, 0xB7BC, 0xF6B3, 0xC2CE, 0xC8B6, 0x2000 };
    dgus.writeVariable(VP_Min_Ex_Temp_Dis, Min_Ex_Temp_buf_ch, 32, true);

    const uint16_t AutoLEVEL_INFO1_buf_ch[] = { 0xEBC7, 0xB4B0, 0xC2CF, 0xB4B0, 0xA5C5, 0x2000 };
    dgus.writeVariable(VP_AutoLEVEL_INFO1, AutoLEVEL_INFO1_buf_ch, 32, true);

    const uint16_t EX_TEMP_INFO2_buf_ch[] = { 0xEBC7, 0xD4C9, 0xC8B5, 0x2000 };
    dgus.writeVariable(VP_EX_TEMP_INFO2_Dis, EX_TEMP_INFO2_buf_ch, 32, true);

    const uint16_t EX_TEMP_INFO3_buf_ch[] = { 0xA1C8, 0xFBCF, 0xD3BC, 0xC8C8, 0x2000 };
    dgus.writeVariable(VP_EX_TEMP_INFO3_Dis, EX_TEMP_INFO3_buf_ch, 32, true);

    const uint16_t PrintConfrim_Info_buf_ch[] = { 0xC7CA, 0xF1B7, 0xAABF, 0xBCCA, 0xF2B4, 0xA1D3, 0x2000 };
    dgus.writeVariable(VP_PrintConfrim_Info_Dis, PrintConfrim_Info_buf_ch, 32, true);

    const uint16_t StopPrintConfrim_Info_buf_ch[] = { 0xC7CA, 0xF1B7, 0xA3CD, 0xB9D6, 0xF2B4, 0xA1D3, 0x2000 };
    dgus.writeVariable(VP_StopPrintConfrim_Info_Dis, StopPrintConfrim_Info_buf_ch, 32, true);

    const uint16_t Printting_buf_ch[] = { 0xF2B4, 0xA1D3, 0xD0D6, 0x2000 };
    dgus.writeVariable(VP_Printting_Dis, Printting_buf_ch, 32, true);

    const uint16_t LCD_BLK_buf_ch[] = { 0xB3B1, 0xE2B9, 0xE8C9, 0xC3D6, 0x2000 };
    dgus.writeVariable(VP_LCD_BLK_Dis, LCD_BLK_buf_ch, 32, true);
  }
}

#endif // DGUS_LCD_UI_MKS
