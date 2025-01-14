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
 * lcd/extui/ftdi_eve_touch_ui/ftdi_eve_extui.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(TOUCH_UI_FTDI_EVE)

#include "screens.h"

namespace ExtUI {
  using namespace Theme;
  using namespace FTDI;

  void onStartup() { EventLoop::setup(); }

  void onIdle() { EventLoop::loop(); }

  void onPrinterKilled(FSTR_P const error, FSTR_P const component) {
    char str[strlen_P(FTOP(error)) + strlen_P(FTOP(component)) + 3];
    sprintf_P(str, PSTR(S_FMT ": " S_FMT), FTOP(error), FTOP(component));
    KillScreen::show(str);
  }

  void onMediaMounted() {
    #if HAS_MEDIA
      sound.play(media_inserted, PLAY_ASYNCHRONOUS);
      StatusScreen::onMediaMounted();
    #endif
  }

  void onMediaError() {
    sound.play(sad_trombone, PLAY_ASYNCHRONOUS);
    AlertDialogBox::showError(F("Unable to read media."));
  }

  void onMediaRemoved() {
    #if HAS_MEDIA
      if (isPrintingFromMedia()) {
        stopPrint();
        InterfaceSoundsScreen::playEventSound(InterfaceSoundsScreen::PRINTING_FAILED);
      }
      else
        sound.play(media_removed, PLAY_ASYNCHRONOUS);

      StatusScreen::onMediaRemoved();
      FilesScreen::onMediaRemoved();
    #endif
  }

  void onHeatingError(const heater_id_t header_id) {}
  void onMinTempError(const heater_id_t header_id) {}
  void onMaxTempError(const heater_id_t header_id) {}

  void onStatusChanged(const char *lcd_msg) { StatusScreen::setStatusMessage(lcd_msg); }

  void onPrintTimerStarted() {
    InterfaceSoundsScreen::playEventSound(InterfaceSoundsScreen::PRINTING_STARTED);
    current_screen.forget();
    PUSH_SCREEN(StatusScreen);
  }
  void onPrintTimerStopped() {
    InterfaceSoundsScreen::playEventSound(InterfaceSoundsScreen::PRINTING_FINISHED);
  }
  void onPrintTimerPaused() {}

  void onPrintDone() {}

  void onFilamentRunout(const extruder_t extruder) {
    char lcd_msg[30];
    sprintf_P(lcd_msg, PSTR("Extruder %d Filament Error"), extruder + 1);
    StatusScreen::setStatusMessage(lcd_msg);
    InterfaceSoundsScreen::playEventSound(InterfaceSoundsScreen::PRINTING_FAILED, FTDI::PLAY_SYNCHRONOUS);
  }

  void onHomingStart() {}
  void onHomingDone() {}

  void onFactoryReset() { InterfaceSettingsScreen::defaultSettings(); }
  void onStoreSettings(char *buff) { InterfaceSettingsScreen::saveSettings(buff); }
  void onLoadSettings(const char *buff) { InterfaceSettingsScreen::loadSettings(buff); }
  void onPostprocessSettings() {} // Called after loading or resetting stored settings

  void onSettingsStored(const bool success) {
    #ifdef ARCHIM2_SPI_FLASH_EEPROM_BACKUP_SIZE
      if (success && InterfaceSettingsScreen::backupEEPROM()) {
        SERIAL_ECHOLNPGM("EEPROM backed up to SPI Flash");
      }
    #else
      UNUSED(success);
    #endif
  }
  void onSettingsLoaded(const bool) {}

  void onPlayTone(const uint16_t frequency, const uint16_t duration/*=0*/) { sound.play_tone(frequency, duration); }

  void onUserConfirmRequired(const char * const msg) {
    if (msg)
      ConfirmUserRequestAlertBox::show(msg);
    else
      ConfirmUserRequestAlertBox::show("Press Resume to Continue");
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void filament_load_prompt(const char * const msg) {
      if (msg)
        FilamentPromptBox::show();
      else
        FilamentPromptBox::hide();
    }
  #endif

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

  #if HAS_LEVELING
    void onLevelingStart() {}
    void onLevelingDone() {}
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp() { return LEVELING_BED_TEMP; }
    #endif
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t x, const int8_t y, const_float_t val) {
      BedMeshViewScreen::onMeshUpdate(x, y, val);
    }
    void onMeshUpdate(const int8_t x, const int8_t y, const ExtUI::probe_state_t state) {
      BedMeshViewScreen::onMeshUpdate(x, y, state);
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
    }
  #endif

  #if HAS_PID_HEATING
    void onPIDTuning(const pidresult_t rst) {
      // Called for temperature PID tuning result
      //SERIAL_ECHOLNPGM("OnPIDTuning:", rst);
      switch (rst) {
        case PID_STARTED:
        case PID_BED_STARTED:
        case PID_CHAMBER_STARTED:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_PID_AUTOTUNE));
          break;
        case PID_BAD_HEATER_ID:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_PID_BAD_HEATER_ID));
          break;
        case PID_TEMP_TOO_HIGH:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH));
          break;
        case PID_TUNING_TIMEOUT:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_PID_TIMEOUT));
          break;
        case PID_DONE:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE));
          break;
      }
      GOTO_SCREEN(StatusScreen);
    }
    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
      // Called by M303 to update the UI
    }
  #endif // HAS_PID_HEATING

  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst) {
      // Called for temperature PID tuning result
      switch (rst) {
        case MPC_STARTED:
          StatusScreen::setStatusMessage(GET_TEXT_F(MSG_MPC_AUTOTUNE));
          break;
      }
      GOTO_SCREEN(StatusScreen);
    }
  #endif

  #if ENABLED(PLATFORM_M997_SUPPORT)
    void onFirmwareFlash() {}
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled() {}
  void onAxisDisabled(const axis_t) {}
  void onAxisEnabled(const axis_t) {}
}

#endif // TOUCH_UI_FTDI_EVE
