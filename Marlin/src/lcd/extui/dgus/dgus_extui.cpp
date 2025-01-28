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

/**
 * lcd/extui/dgus/dgus_extui.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if HAS_DGUS_LCD_CLASSIC

#include "../ui_api.h"
#include "DGUSDisplay.h"
#include "DGUSDisplayDef.h"
#include "DGUSScreenHandler.h"

extern const char NUL_STR[];

namespace ExtUI {

  void onStartup() {
    dgus.initDisplay();
    screen.updateScreenVPData();
  }

  void onIdle() { screen.loop(); }

  void onPrinterKilled(FSTR_P const error, FSTR_P const) {
    DGUS_ScreenID screenID = DGUS_SCREEN_KILL;
    #if DGUS_LCD_UI_CREALITY_TOUCH
      if (strcmp_P(FTOP(error), GET_TEXT(MSG_ERR_MAXTEMP)) == 0 || strcmp_P(FTOP(error), GET_TEXT(MSG_ERR_THERMAL_RUNAWAY)) == 0)
        screenID = DGUS_SCREEN_THERMAL_RUNAWAY;
      else if (strcmp_P(FTOP(error), GET_TEXT(MSG_ERR_HEATING_FAILED)) == 0)
        screenID = DGUS_SCREEN_HEATING_FAILED;
      else if (strcmp_P(FTOP(error), GET_TEXT(MSG_ERR_MINTEMP)) == 0)
        screenID = DGUS_SCREEN_THERMISTOR_ERROR;
      screen.sendInfoScreen(GET_TEXT_F(MSG_HALTED), error, GET_TEXT_F(MSG_PLEASE_RESET), FPSTR(NUL_STR), true, true, true, true);
    #else
      screen.sendInfoScreen(GET_TEXT_F(MSG_HALTED), error, FPSTR(NUL_STR), GET_TEXT_F(MSG_PLEASE_RESET), true, true, true, true);
    #endif

    screen.gotoScreen(screenID);
    while (!screen.loop());  // Wait while anything is left to be sent
  }

  void onMediaMounted() { TERN_(HAS_MEDIA, screen.sdCardInserted()); }
  void onMediaError()   { TERN_(HAS_MEDIA, screen.sdCardError()); }
  void onMediaRemoved() { TERN_(HAS_MEDIA, screen.sdCardRemoved()); }

  void onHeatingError(const heater_id_t header_id) {}
  void onMinTempError(const heater_id_t header_id) {}
  void onMaxTempError(const heater_id_t header_id) {}

  void onPlayTone(const uint16_t frequency, const uint16_t duration/*=0*/) {
    TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.buzzer(frequency, duration));
  }
  void onPrintTimerStarted() {}

  void onPrintTimerPaused() {
    #if DGUS_LCD_UI_CREALITY_TOUCH
      // Handle M28 Pause SD print - But only if we're not waiting on a user
      if (isPrintingFromMediaPaused() && screen.getCurrentScreen() == DGUS_SCREEN_PRINT_RUNNING)
        screen.gotoScreen(DGUS_SCREEN_PRINT_PAUSED);
    #endif
  }

  void onPrintTimerStopped() {}

  void onFilamentRunout(const extruder_t extruder) { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.filamentRunout()); }

  inline void onUserConfirmed() {
    screen.setupConfirmAction(nullptr);
    #if DGUS_LCD_UI_CREALITY_TOUCH
      //SERIAL_ECHOLNPGM("User confirmation invoked");
      setUserConfirmed();
    #else
      screen.popToOldScreen();
    #endif
  }

  void onUserConfirmRequired(const char * const msg) {
    if (msg) {
      #if DGUS_LCD_UI_CREALITY_TOUCH
        //SERIAL_ECHOLNPGM("User confirmation requested: ", msg);
        screen.setstatusmessagePGM(msg);
        screen.setupConfirmAction(onUserConfirmed);
        screen.sendInfoScreen(F("Confirmation required"), FPSTR(msg), FPSTR(NUL_STR), FPSTR(NUL_STR), true, true, false, true);
      #else
        screen.sendInfoScreen(F("Please confirm."), nullptr, FPSTR(msg), nullptr, true, true, false, true);
        screen.setupConfirmAction(setUserConfirmed);
      #endif
      screen.gotoScreen(DGUS_SCREEN_POPUP);
    }
    else if (screen.getCurrentScreen() == DGUS_SCREEN_POPUP ) {
      #if DGUS_LCD_UI_CREALITY_TOUCH
        //SERIAL_ECHOLNPGM("User confirmation canceled");
        //screen.setupConfirmAction(nullptr);
        screen.setstatusmessagePGM(nullptr);
      #else
        screen.setupConfirmAction(nullptr);
      #endif
      screen.popToOldScreen();
    }
  }

  // For fancy LCDs include an icon ID, message, and translated button title
  void onUserConfirmRequired(const int icon, const char * const cstr, FSTR_P const fBtn) {
    onUserConfirmRequired(cstr);
    UNUSED(icon); UNUSED(fBtn);
  }
  void onUserConfirmRequired(const int icon, FSTR_P const fstr, FSTR_P const fBtn) {
    onUserConfirmRequired(fstr);
    UNUSED(icon); UNUSED(fBtn);
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void onPauseMode(
      const PauseMessage message,
      const PauseMode mode/*=PAUSE_MODE_SAME*/,
      const uint8_t extruder/*=active_extruder*/
    ) {
      stdOnPauseMode(message, mode, extruder);
    }
  #endif

  void onStatusChanged(const char * const msg) { screen.setStatusMessage(msg); }

  void onHomingStart()  { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onHomingStart()); }
  void onHomingDone()   { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onHomingDone()); }

  void onPrintDone()    { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onPrintDone()); }

  void onFactoryReset() { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onFactoryReset()); }

  void onStoreSettings(char *buff) {
    // Called when saving to EEPROM (i.e. M500). If the ExtUI needs
    // permanent data to be stored, it can write up to eeprom_data_size bytes
    // into buff.

    // Example:
    //  static_assert(sizeof(myDataStruct) <= eeprom_data_size);
    //  memcpy(buff, &myDataStruct, sizeof(myDataStruct));
  }

  void onLoadSettings(const char *buff) {
    // Called while loading settings from EEPROM. If the ExtUI
    // needs to retrieve data, it should copy up to eeprom_data_size bytes
    // from buff

    // Example:
    //  static_assert(sizeof(myDataStruct) <= eeprom_data_size);
    //  memcpy(&myDataStruct, buff, sizeof(myDataStruct));
  }

  void onPostprocessSettings() {
    // Called after loading or resetting stored settings
  }

  void onSettingsStored(const bool success) {
    // Called after the entire EEPROM has been written,
    // whether successful or not.
  }

  void onSettingsLoaded(const bool success) {
    // Called after the entire EEPROM has been read,
    // whether successful or not.
  }

  #if HAS_LEVELING
    void onLevelingStart() { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onLevelingStart()); }
    void onLevelingDone() {}
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp() { return LEVELING_BED_TEMP; }
    #endif
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
      // Called when any mesh points are updated
      TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.onLevelingUpdate(xpos, ypos));
    }

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const probe_state_t state) {
      // Called to indicate a special condition
      #if DGUS_LCD_UI_CREALITY_TOUCH
        // Only called for UBL
        if (state == G29_START) screen.onLevelingStart();
        screen.onLevelingUpdate(xpos, ypos);
      #endif
    }
  #endif

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    void onSetMinExtrusionTemp(const celsius_t) {}
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void onSetPowerLoss(const bool onoff) {
      // Called when power-loss is enabled/disabled
    }
    void onPowerLoss() {
      // Called when power-loss state is detected
    }
    void onPowerLossResume() {
      // Called on resume from power-loss
      IF_DISABLED(DGUS_LCD_UI_MKS, screen.gotoScreen(DGUS_SCREEN_POWER_LOSS));
    }
  #endif

  #if HAS_PID_HEATING
    void onPIDTuning(const pidresult_t rst) {
      #if DISABLED(DGUS_LCD_UI_CREALITY_TOUCH)
        // Called for temperature PID tuning result
        switch (rst) {
          case PID_STARTED:
          case PID_BED_STARTED:
          case PID_CHAMBER_STARTED:
            screen.setStatusMessage(GET_TEXT_F(MSG_PID_AUTOTUNE));
            break;
          case PID_BAD_HEATER_ID:
            screen.setStatusMessage(GET_TEXT_F(MSG_PID_BAD_HEATER_ID));
            break;
          case PID_TEMP_TOO_HIGH:
            screen.setStatusMessage(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH));
            break;
          case PID_TUNING_TIMEOUT:
            screen.setStatusMessage(GET_TEXT_F(MSG_PID_TIMEOUT));
            break;
          case PID_DONE:
            screen.setStatusMessage(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE));
            break;
        }
        screen.gotoScreen(DGUS_SCREEN_MAIN);
      #else
        UNUSED(rst);
      #endif
    }
    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
      // Called by M303 to update the UI
    }
  #endif

  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst) {
      // Called for temperature MPC tuning result
      switch (rst) {
        case MPC_STARTED:
          screen.setStatusMessage(GET_TEXT_F(MSG_MPC_AUTOTUNE));
          break;
        case MPC_TEMP_ERROR:
          //screen.setStatusMessage(GET_TEXT_F(MSG_MPC_TEMP_ERROR));
          break;
        case MPC_INTERRUPTED:
          //screen.setStatusMessage(GET_TEXT_F(MSG_MPC_INTERRUPTED));
          break;
        case MPC_DONE:
          //screen.setStatusMessage(GET_TEXT_F(MSG_MPC_AUTOTUNE_DONE));
          break;
      }
      screen.gotoScreen(DGUS_SCREEN_MAIN);
    }
  #endif

  #if ENABLED(PLATFORM_M997_SUPPORT)
    void onFirmwareFlash() {}
  #endif

  void onSteppersDisabled() { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.handleStepperState(false)); }
  void onSteppersEnabled()  { TERN_(DGUS_LCD_UI_CREALITY_TOUCH, screen.handleStepperState(true)); }
  void onAxisDisabled(const axis_t) {}
  void onAxisEnabled(const axis_t) {}
}

#endif // HAS_DGUS_LCD_CLASSIC
