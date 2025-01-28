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

#if HAS_DGUS_LCD_CLASSIC

#include "DGUSScreenHandler.h"

#include "../../../MarlinCore.h"
#include "../../../gcode/queue.h"
#include "../../../libs/duration_t.h"
#include "../../../module/settings.h"
#include "../../../module/temperature.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/printcounter.h"
#include "../../../sd/cardreader.h"

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

DGUSScreenHandlerClass screen;

uint16_t DGUSScreenHandler::confirmVP;

DGUS_ScreenID DGUSScreenHandler::current_screenID;
DGUS_ScreenID DGUSScreenHandler::past_screenIDs[NUM_PAST_SCREENS];
uint8_t DGUSScreenHandler::update_ptr;
uint16_t DGUSScreenHandler::skipVP;
bool DGUSScreenHandler::screenComplete;

void (*DGUSScreenHandler::confirm_action_cb)() = nullptr;

#if HAS_MEDIA
  int16_t DGUSScreenHandler::top_file = 0,
          DGUSScreenHandler::file_to_print = 0;
  ExtUI::FileList filelist;
#endif

#if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
  filament_data_t filament_data;
#endif

void DGUSScreenHandler::sendInfoScreen_P(PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4, bool l1inflash, bool l2inflash, bool l3inflash, bool l4inflash) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_MSGSTR1, &ramcopy)) {
    ramcopy.memadr = (void*) line1;
    l1inflash ? DGUSScreenHandler::sendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::sendStringToDisplay(ramcopy);
  }
  if (populate_VPVar(VP_MSGSTR2, &ramcopy)) {
    ramcopy.memadr = (void*) line2;
    l2inflash ? DGUSScreenHandler::sendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::sendStringToDisplay(ramcopy);
  }
  if (populate_VPVar(VP_MSGSTR3, &ramcopy)) {
    ramcopy.memadr = (void*) line3;
    l3inflash ? DGUSScreenHandler::sendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::sendStringToDisplay(ramcopy);
  }
  #ifdef VP_MSGSTR4
    if (populate_VPVar(VP_MSGSTR4, &ramcopy)) {
      ramcopy.memadr = (void*) line4;
      l4inflash ? DGUSScreenHandler::sendStringToDisplayPGM(ramcopy) : DGUSScreenHandler::sendStringToDisplay(ramcopy);
    }
  #endif
}

void DGUSScreenHandler::handleUserConfirmationPopUp(uint16_t VP, PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4, bool l1, bool l2, bool l3, bool l4) {
  if (current_screenID == DGUS_SCREEN_CONFIRM) // Already showing a pop up, so we need to cancel that first.
    popToOldScreen();

  confirmVP = VP;
  sendInfoScreen_P(line1, line2, line3, line4, l1, l2, l3, l4);
  gotoScreen(DGUS_SCREEN_CONFIRM);
}

void DGUSScreenHandler::setStatusMessage(const char *msg) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_M117, &ramcopy)) {
    ramcopy.memadr = (void*) msg;
    sendStringToDisplay(ramcopy);
  }
}

void DGUSScreenHandler::setstatusmessagePGM(PGM_P const msg) {
  DGUS_VP_Variable ramcopy;
  if (populate_VPVar(VP_M117, &ramcopy)) {
    ramcopy.memadr = (void*) msg;
    sendStringToDisplayPGM(ramcopy);
  }
}

// Send an 8 bit or 16 bit value to the display.
void DGUSScreenHandler::sendWordValueToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    if (var.size > 1)
      dgus.writeVariable(var.VP, *(int16_t*)var.memadr);
    else
      dgus.writeVariable(var.VP, *(int8_t*)var.memadr);
  }
}

// Send an uint8_t between 0 and 255 to the display, but scale to a percentage (0..100)
void DGUSScreenHandler::sendPercentageToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    uint16_t tmp = *(uint8_t *) var.memadr + 1; // +1 -> avoid rounding issues for the display.
    tmp = map(tmp, 0, 255, 0, 100);
    dgus.writeVariable(var.VP, tmp);
  }
}

// Send the current print progress to the display.
void DGUSScreenHandler::sendPrintProgressToDisplay(DGUS_VP_Variable &var) {
  uint16_t tmp = ExtUI::getProgress_percent();
  dgus.writeVariable(var.VP, tmp);
}

// Send the current print time to the display.
// It is using a hex display for that: It expects BSD coded data in the format xxyyzz
void DGUSScreenHandler::sendPrintTimeToDisplay(DGUS_VP_Variable &var) {
  duration_t elapsed = print_job_timer.duration();
  char buf[32];
  elapsed.toString(buf);
  dgus.writeVariable(VP_PrintTime, buf, var.size, true);
}

// Send an uint8_t between 0 and 100 to a variable scale to 0..255
void DGUSScreenHandler::percentageToUint8(DGUS_VP_Variable &var, void *val_ptr) {
  if (var.memadr) {
    const uint16_t value = BE16_P(val_ptr);
    *(uint8_t*)var.memadr = map(constrain(value, 0, 100), 0, 100, 0, 255);
  }
}

// Sends a (RAM located) string to the DGUS Display
// (Note: The DGUS Display does not clear after the \0, you have to
// overwrite the remainings with spaces.// var.size has the display buffer size!
void DGUSScreenHandler::sendStringToDisplay(DGUS_VP_Variable &var) {
  char *tmp = (char*) var.memadr;
  dgus.writeVariable(var.VP, tmp, var.size, true);
}

// Sends a (flash located) string to the DGUS Display
// (Note: The DGUS Display does not clear after the \0, you have to
// overwrite the remainings with spaces.// var.size has the display buffer size!
void DGUSScreenHandler::sendStringToDisplayPGM(DGUS_VP_Variable &var) {
  char *tmp = (char*) var.memadr;
  dgus.writeVariablePGM(var.VP, tmp, var.size, true);
}

#if HAS_PID_HEATING
  void DGUSScreenHandler::sendTemperaturePID(DGUS_VP_Variable &var) {
    float value = *(float *)var.memadr;
    value /= 10;
    float valuesend = 0;
    switch (var.VP) {
      default: return;
      #if HAS_HOTEND
        case VP_E0_PID_P: valuesend = value; break;
        case VP_E0_PID_I: valuesend = unscalePID_i(value); break;
        case VP_E0_PID_D: valuesend = unscalePID_d(value); break;
      #endif
      #if HAS_MULTI_HOTEND
        case VP_E1_PID_P: valuesend = value; break;
        case VP_E1_PID_I: valuesend = unscalePID_i(value); break;
        case VP_E1_PID_D: valuesend = unscalePID_d(value); break;
      #endif
      #if HAS_HEATED_BED
        case VP_BED_PID_P: valuesend = value; break;
        case VP_BED_PID_I: valuesend = unscalePID_i(value); break;
        case VP_BED_PID_D: valuesend = unscalePID_d(value); break;
      #endif
    }

    valuesend *= cpow(10, 1);
    union { int16_t i; char lb[2]; } endian;

    char tmp[2];
    endian.i = valuesend;
    tmp[0] = endian.lb[1];
    tmp[1] = endian.lb[0];
    dgus.writeVariable(var.VP, tmp, 2);
  }
#endif

#if ENABLED(PRINTCOUNTER)

  // Send the accumulate print time to the display.
  // It is using a hex display for that: It expects BSD coded data in the format xxyyzz
  void DGUSScreenHandler::sendPrintAccTimeToDisplay(DGUS_VP_Variable &var) {
    printStatistics state = print_job_timer.getStats();
    char buf[22];
    duration_t elapsed = state.printTime;
    elapsed.toString(buf);
    dgus.writeVariable(VP_PrintAccTime, buf, var.size, true);
  }

  void DGUSScreenHandler::sendPrintsTotalToDisplay(DGUS_VP_Variable &var) {
    printStatistics state = print_job_timer.getStats();
    char buf[10];
    sprintf_P(buf, PSTR("%u"), state.totalPrints);
    dgus.writeVariable(VP_PrintsTotal, buf, var.size, true);
  }

#endif

// Send fan status value to the display.
#if HAS_FAN

  void DGUSScreenHandler::sendFanStatusToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      uint16_t data_to_send = 0;
      if (*(uint8_t *) var.memadr) data_to_send = 1;
      dgus.writeVariable(var.VP, data_to_send);
    }
  }

#endif

// Send heater status value to the display.
void DGUSScreenHandler::sendHeaterStatusToDisplay(DGUS_VP_Variable &var) {
  if (var.memadr) {
    uint16_t data_to_send = 0;
    if (*(int16_t *) var.memadr) data_to_send = 1;
    dgus.writeVariable(var.VP, data_to_send);
  }
}

#if ENABLED(DGUS_UI_WAITING)

  void DGUSScreenHandler::sendWaitingStatusToDisplay(DGUS_VP_Variable &var) {
    // In FYSETC UI design there are 10 statuses to loop
    static uint16_t period = 0;
    static uint16_t index = 0;
    if (period++ > DGUS_UI_WAITING_STATUS_PERIOD) {
      dgus.writeVariable(var.VP, index);
      if (++index >= DGUS_UI_WAITING_STATUS) index = 0;
      period = 0;
    }
  }

#endif

#if HAS_MEDIA

  void DGUSScreenHandler::screenChangeHookIfSD(DGUS_VP_Variable &var, void *val_ptr) {
    // default action executed when there is a SD card, but not printing
    if (ExtUI::isMediaMounted() && !ExtUI::isPrintingFromMedia()) {
      screenChangeHook(var, val_ptr);
      dgus.requestScreen(current_screenID);
      return;
    }

    // if we are printing, we jump to two screens after the requested one.
    // This should host e.g a print pause / print abort / print resume dialog.
    // This concept allows to recycle this hook for other file
    if (ExtUI::isPrintingFromMedia() && !card.flag.abort_sd_printing) {
      gotoScreen(DGUS_SCREEN_SDPRINTMANIPULATION);
      return;
    }

    // Don't let the user in the dark why there is no reaction.
    if (!ExtUI::isMediaMounted()) {
      setStatusMessage(GET_TEXT_F(MSG_NO_MEDIA));
      return;
    }
    if (card.flag.abort_sd_printing) {
      setStatusMessage(GET_TEXT_F(MSG_MEDIA_ABORTING));
      return;
    }
  }

  void DGUSScreenHandler::sdScrollFilelist(DGUS_VP_Variable& var, void *val_ptr) {
    auto old_top = top_file;
    const int16_t scroll = (int16_t)BE16_P(val_ptr);
    if (scroll) {
      top_file += scroll;
      if (top_file < 0) {
        top_file = 0;
      }
      else {
        int16_t max_top = filelist.count() -  DGUS_SD_FILESPERSCREEN;
        NOLESS(max_top, 0);
        NOMORE(top_file, max_top);
      }
    }
    else if (!filelist.isAtRootDir()) {
      IF_DISABLED(DGUS_LCD_UI_MKS, filelist.upDir());
      top_file = 0;
      forceCompleteUpdate();
    }

    if (old_top != top_file) forceCompleteUpdate();
  }

  void DGUSScreenHandler::sdReallyAbort(DGUS_VP_Variable &var, void *val_ptr) {
    ExtUI::stopPrint();
    gotoScreen(DGUS_SCREEN_MAIN);
  }

  void DGUSScreenHandler::sdPrintTune(DGUS_VP_Variable &var, void *val_ptr) {
    if (!ExtUI::isPrintingFromMedia()) return; // avoid race condition when user stays in this menu and printer finishes.
    gotoScreen(DGUS_SCREEN_SDPRINTTUNE);
  }

  void DGUSScreenHandler::sdCardError() {
    DGUSScreenHandler::sdCardRemoved();
    sendInfoScreen(F("NOTICE"), nullptr, F("SD card error"), nullptr, true, true, true, true);
    setupConfirmAction(nullptr);
    gotoScreen(DGUS_SCREEN_POPUP);
  }

#endif // HAS_MEDIA

void DGUSScreenHandler::screenConfirmedOK(DGUS_VP_Variable &var, void *val_ptr) {
  DGUS_VP_Variable ramcopy;
  if (!populate_VPVar(confirmVP, &ramcopy)) return;
  if (ramcopy.set_by_display_handler) ramcopy.set_by_display_handler(ramcopy, val_ptr);
}

const uint16_t* findScreenVPMapList(uint8_t screen) {
  const uint16_t *ret;
  const struct VPMapping *map = VPMap;
  while ((ret = (uint16_t*) pgm_read_ptr(&(map->VPList)))) {
    if (pgm_read_byte(&(map->screen)) == screen) return ret;
    map++;
  }
  return nullptr;
}

const DGUS_VP_Variable* findVPVar(const uint16_t vp) {
  const DGUS_VP_Variable *ret = ListOfVP;
  do {
    const uint16_t vpcheck = pgm_read_word(&(ret->VP));
    if (vpcheck == 0) break;
    if (vpcheck == vp) return ret;
    ++ret;
  } while (1);

  return nullptr;
}

void DGUSScreenHandler::screenChangeHookIfIdle(DGUS_VP_Variable &var, void *val_ptr) {
  if (!ExtUI::isPrinting()) {
    screenChangeHook(var, val_ptr);
    dgus.requestScreen(current_screenID);
  }
}

void DGUSScreenHandler::handleAllHeatersOff(DGUS_VP_Variable &var, void *val_ptr) {
  thermalManager.disable_all_heaters();
  forceCompleteUpdate(); // hint to send all data.
}

void DGUSScreenHandler::handleTemperatureChanged(DGUS_VP_Variable &var, void *val_ptr) {
  celsius_t newvalue = BE16_P(val_ptr);
  celsius_t acceptedvalue;

  switch (var.VP) {
    default: return;
    #if HAS_HOTEND
      case VP_T_E0_Set:
        NOMORE(newvalue, thermalManager.hotend_max_target(0));
        thermalManager.setTargetHotend(newvalue, 0);
        acceptedvalue = thermalManager.degTargetHotend(0);
        break;
    #endif
    #if HAS_MULTI_HOTEND
      case VP_T_E1_Set:
        NOMORE(newvalue, thermalManager.hotend_max_target(1));
        thermalManager.setTargetHotend(newvalue, 1);
        acceptedvalue = thermalManager.degTargetHotend(1);
        break;
    #endif
    #if HAS_HEATED_BED
      case VP_T_Bed_Set:
        NOMORE(newvalue, BED_MAX_TARGET);
        thermalManager.setTargetBed(newvalue);
        acceptedvalue = thermalManager.degTargetBed();
        break;
    #endif
  }

  // reply to display the new value to update the view if the new value was rejected by the Thermal Manager.
  if (newvalue != acceptedvalue && var.send_to_display_handler) var.send_to_display_handler(var);
  skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
}

void DGUSScreenHandler::handleFlowRateChanged(DGUS_VP_Variable &var, void *val_ptr) {
  #if HAS_EXTRUDERS
    const uint16_t newvalue = BE16_P(val_ptr);
    uint_fast8_t target_extruder;
    switch (var.VP) {
      default: return;
      case VP_Flowrate_E0: target_extruder = 0; break;
      #if HAS_MULTI_EXTRUDER
        case VP_Flowrate_E1: target_extruder = 1; break;
      #endif
    }

    planner.set_flow(target_extruder, newvalue);
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  #else
    UNUSED(var); UNUSED(val_ptr);
  #endif
}

void DGUSScreenHandler::handleManualExtrude(DGUS_VP_Variable &var, void *val_ptr) {
  const int16_t movevalue = BE16_P(val_ptr);
  float target = movevalue * 0.01f;
  ExtUI::extruder_t target_extruder;

  switch (var.VP) {
    #if HAS_HOTEND
      case VP_MOVE_E0: target_extruder = ExtUI::extruder_t::E0; break;
      #if HAS_MULTI_EXTRUDER
        case VP_MOVE_E1: target_extruder = ExtUI::extruder_t::E1; break;
      #endif
    #endif
    default: return;
  }

  target += ExtUI::getAxisPosition_mm(target_extruder);
  ExtUI::setAxisPosition_mm(target, target_extruder);
  skipVP = var.VP;
}

#if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
  void DGUSScreenHandler::handleManualMoveOption(DGUS_VP_Variable &var, void *val_ptr) {
    *(uint16_t*)var.memadr = BE16_P(val_ptr);
  }
#endif

void DGUSScreenHandler::handleMotorLockUnlock(DGUS_VP_Variable &var, void *val_ptr) {
  const int16_t lock = BE16_P(val_ptr);
  queue.enqueue_one_now(lock ? F("M18") : F("M17"));
}

void DGUSScreenHandler::handleSettings(DGUS_VP_Variable &var, void *val_ptr) {
  const uint16_t value = BE16_P(val_ptr);
  switch (value) {
    default: break;
    case 1:
      TERN_(PRINTCOUNTER, print_job_timer.initStats());
      settings.reset();
      settings.save();
      break;
    case 2: settings.load(); break;
    case 3: settings.save(); break;
  }
}

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

  void DGUSScreenHandler::handleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t value_raw = BE16_P(val_ptr);
    const float value = (float)value_raw / 10;
    ExtUI::axis_t axis;
    switch (var.VP) {
      case VP_X_STEP_PER_MM: axis = ExtUI::axis_t::X; break;
      case VP_Y_STEP_PER_MM: axis = ExtUI::axis_t::Y; break;
      case VP_Z_STEP_PER_MM: axis = ExtUI::axis_t::Z; break;
      default: return;
    }
    ExtUI::setAxisSteps_per_mm(value, axis);
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }

  void DGUSScreenHandler::handleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t value_raw = BE16_P(val_ptr);
    const float value = (float)value_raw / 10;
    ExtUI::extruder_t extruder;
    switch (var.VP) {
      default: return;
        #if HAS_EXTRUDERS
          case VP_E0_STEP_PER_MM: extruder = ExtUI::extruder_t::E0; break;
          #if HAS_MULTI_EXTRUDER
            case VP_E1_STEP_PER_MM: extruder = ExtUI::extruder_t::E1; break;
          #endif
        #endif
    }
    ExtUI::setAxisSteps_per_mm(value, extruder);
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
  }

#endif // EDITABLE_STEPS_PER_UNIT

#if HAS_PID_HEATING
  void DGUSScreenHandler::handlePIDAutotune(DGUS_VP_Variable &var, void *val_ptr) {
    char buf[32] = {0};

    switch (var.VP) {
      default: break;
        #if ENABLED(PIDTEMP)
          #if HAS_HOTEND
            case VP_PID_AUTOTUNE_E0: // Autotune Extruder 0
              sprintf_P(buf, PSTR("M303 E%d C5 S210 U1"), ExtUI::extruder_t::E0);
              queue.enqueue_one_now(buf);
              break;
          #endif
          #if HAS_MULTI_HOTEND
            case VP_PID_AUTOTUNE_E1:
              sprintf_P(buf, PSTR("M303 E%d C5 S210 U1"), ExtUI::extruder_t::E1);
              queue.enqueue_one_now(buf);
              break;
          #endif
        #endif
        #if ENABLED(PIDTEMPBED)
          case VP_PID_AUTOTUNE_BED:
            queue.enqueue_one_now(F("M303 E-1 C5 S70 U1"));
            break;
        #endif
    }

    #if ENABLED(DGUS_UI_WAITING)
      sendInfoScreen(F("PID is autotuning"), F("please wait"), NUL_STR, NUL_STR, true, true, true, true);
      gotoScreen(DGUS_SCREEN_WAITING);
    #endif
  }
#endif // HAS_PID_HEATING

#if HAS_BED_PROBE
  void DGUSScreenHandler::handleProbeOffsetZChanged(DGUS_VP_Variable &var, void *val_ptr) {
    const float offset = float(int16_t(BE16_P(val_ptr))) / 100.0f;
    ExtUI::setZOffset_mm(offset);
    skipVP = var.VP; // don't overwrite value the next update time as the display might autoincrement in parallel
    return;
  }
#endif

#if HAS_FAN
  void DGUSScreenHandler::handleFanControl(DGUS_VP_Variable &var, void *val_ptr) {
    *(uint8_t*)var.memadr = *(uint8_t*)var.memadr > 0 ? 0 : 255;
  }
#endif

void DGUSScreenHandler::handleHeaterControl(DGUS_VP_Variable &var, void *val_ptr) {
  uint8_t preheat_temp = 0;
  switch (var.VP) {
    #if HAS_HOTEND
      case VP_E0_CONTROL:
      #if HAS_MULTI_HOTEND
        case VP_E1_CONTROL:
        #if HOTENDS >= 3
          case VP_E2_CONTROL:
        #endif
      #endif
      preheat_temp = PREHEAT_1_TEMP_HOTEND;
      break;
    #endif

    #if HAS_HEATED_BED
      case VP_BED_CONTROL:
        preheat_temp = PREHEAT_1_TEMP_BED;
        break;
    #endif
  }

  *(int16_t*)var.memadr = *(int16_t*)var.memadr > 0 ? 0 : preheat_temp;
}

#if ENABLED(DGUS_PREHEAT_UI)

  void DGUSScreenHandler::handlePreheat(DGUS_VP_Variable &var, void *val_ptr) {
    const uint16_t preheat_option = BE16_P(val_ptr);
    switch (preheat_option) {
      default:
      switch (var.VP) {
        default: return;
        case VP_E0_BED_PREHEAT: TERN_(HAS_HOTEND, ui.preheat_all(0)); break;
        #if DISABLED(DGUS_LCD_UI_HIPRECY) && HAS_MULTI_HOTEND
          case VP_E1_BED_PREHEAT: ui.preheat_all(1); break;
        #endif
      }
      case 7: break; // Custom preheat
      case 9: thermalManager.cooldown(); break; // Cool down
    }

    // Go to the preheat screen to show the heating progress
    gotoScreen(DGUS_SCREEN_PREHEAT);
  }

#endif // DGUS_PREHEAT_UI

#if ENABLED(POWER_LOSS_RECOVERY)

  void DGUSScreenHandler::handlePowerLossRecovery(DGUS_VP_Variable &var, void *val_ptr) {
    uint16_t value = BE16_P(val_ptr);
    if (value) {
      queue.inject(F("M1000"));
      dgus.writeVariable(VP_SD_Print_Filename, filelist.filename(), 32, true);
      gotoScreen(PLR_SCREEN_RECOVER);
    }
    else {
      recovery.cancel();
      gotoScreen(PLR_SCREEN_CANCEL);
    }
  }

#endif

void DGUSScreenHandler::updateNewScreen(const DGUS_ScreenID screenID, const bool popup) {
  if (!popup) {
    memmove(&past_screenIDs[1], &past_screenIDs[0], sizeof(past_screenIDs) - 1);
    past_screenIDs[0] = current_screenID;
  }
  current_screenID = screenID;
  skipVP = 0;
  forceCompleteUpdate();
}

void DGUSScreenHandler::popToOldScreen() {
  gotoScreen(past_screenIDs[0], true);
  memmove(&past_screenIDs[0], &past_screenIDs[1], sizeof(past_screenIDs) - 1);
  past_screenIDs[sizeof(past_screenIDs) - 1] = DGUS_SCREEN_MAIN;
}

void DGUSScreenHandler::updateScreenVPData() {
  const uint16_t *VPList = findScreenVPMapList(current_screenID);
  if (!VPList) {
    screenComplete = true;
    return; // nothing to do, likely a bug or boring screen.
  }

  // Round-robin updating of all VPs.
  VPList += update_ptr;

  bool sent_one = false;
  do {
    uint16_t VP = pgm_read_word(VPList);
    if (!VP) {
      update_ptr = 0;
      screenComplete = true;
      return; // Screen completed.
    }

    if (VP == skipVP) { skipVP = 0; continue; }

    DGUS_VP_Variable rcpy;
    if (populate_VPVar(VP, &rcpy)) {
      uint8_t expected_tx = 6 + rcpy.size; // expected overhead is 6 bytes + payload.
      // Send the VP to the display, but try to avoid overrunning the Tx Buffer.
      // But send at least one VP, to avoid getting stalled.
      if (rcpy.send_to_display_handler && (!sent_one || expected_tx <= dgus.getFreeTxBuffer())) {
        sent_one = true;
        rcpy.send_to_display_handler(rcpy);
      }
      else {
        screenComplete = false;
        return; // please call again!
      }
    }

  } while (++update_ptr, ++VPList, true);
}

void DGUSScreenHandler::gotoScreen(const DGUS_ScreenID screenID, const bool popup/*=false*/) {
  dgus.requestScreen(screenID);
  updateNewScreen(screenID, popup);
}

void DGUSDisplay::requestScreen(const DGUS_ScreenID screenID) {
  const unsigned char gotoscreen[] = { 0x5A, 0x01, (unsigned char) (screenID >> 8U), (unsigned char) (screenID & 0xFFU) };
  writeVariable(0x84, gotoscreen, sizeof(gotoscreen));
}

#endif // HAS_DGUS_LCD_CLASSIC
