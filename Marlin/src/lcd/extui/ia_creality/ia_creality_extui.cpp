/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/* ****************************************
 * lcd/extui/ia_creality/ia_creality_extui.cpp
 * ****************************************
 * Extensible_UI implementation for Creality DWIN
 * 10SPro, Max, CRX, and others
 * Based original Creality release, ported to Marlin ExtUI
 * Written by Insanity Automation, sponsored by Tiny Machines 3D
 *
 * ***************************************/

#include "../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_IA_CREALITY

#include "ia_creality_rts.h"
#include "FileNavigator.h"
#include "../ui_api.h"

namespace ExtUI {

uint8_t dwin_settings_version = 1;

void onStartup() { rts.onStartup(); }

void onIdle() { rts.onIdle(); }

void onPrinterKilled(FSTR_P const error, FSTR_P const component) {
  rts.sendData(ExchangePageBase + 88, ExchangepageAddr);
  int16_t j = 0;
  char outmsg[40];
  char killMsg[strlen_P(FTOP(error)) + strlen_P(FTOP(component)) + 3];
  sprintf_P(killMsg, PSTR(S_FMT ": " S_FMT), FTOP(error), FTOP(component));
  while (j < 4) outmsg[j++] = '*';
  while (const char c = killMsg[j - 4]) outmsg[j++] = c;
  while (j < 40) outmsg[j++] = '*';
  rts.sendData(outmsg, MacVersion);
  delay_ms(10);
}

void onMediaMounted() {
  filenavigator.reset();
  filenavigator.getFiles(0);
  fileIndex = 0;
  recordcount = 0;
}

void onMediaError() {
  filenavigator.reset();
  for (int16_t i = 0; i < MaxFileNumber; i++)
    for (int16_t j = 0; j < 10; j++) rts.sendData(0, SDFILE_ADDR + i * 10 + j);

  for (int16_t j = 0; j < 10; j++) {
    rts.sendData(0, Printfilename + j); // clean screen.
    rts.sendData(0, Choosefilename + j); // clean filename
  }
  for (int16_t j = 0; j < 8; j++) rts.sendData(0, FilenameCount + j);
  for (int16_t j = 1; j <= MaxFileNumber; j++) {
    rts.sendData(10, FilenameIcon + j);
    rts.sendData(10, FilenameIcon1 + j);
  }
}

void onMediaRemoved() {
  filenavigator.reset();
  for (int16_t i = 0; i < MaxFileNumber; i++)
    for (int16_t j = 0; j < 10; j++) rts.sendData(0, SDFILE_ADDR + i * 10 + j);

  for (int16_t j = 0; j < 10; j++) {
    rts.sendData(0, Printfilename + j); // clean screen.
    rts.sendData(0, Choosefilename + j); // clean filename
  }
  for (int16_t j = 0; j < 8; j++) rts.sendData(0, FilenameCount + j);
  for (int16_t j = 1; j <= MaxFileNumber; j++) {
    rts.sendData(10, FilenameIcon + j);
    rts.sendData(10, FilenameIcon1 + j);
  }
}

void onHeatingError(const heater_id_t header_id) {}
void onMinTempError(const heater_id_t header_id) {}
void onMaxTempError(const heater_id_t header_id) {}

void onPlayTone(const uint16_t, const uint16_t/*=0*/) {
  rts.sendData(StartSoundSet, SoundAddr);
}

void onPrintTimerStarted() {
  if (waitway == 7) return;
  printerStatusKey[1] = 3;
  show_status = true;
  delay_ms(1);
  rts.sendData(ExchangePageBase + 53, ExchangepageAddr);
}

void onPrintTimerPaused() {
  rts.sendData(ExchangePageBase + 78, ExchangepageAddr); // Display Pause Screen
  onStatusChanged(F("Pausing..."));
}

void onPrintTimerStopped() {
  if (waitway == 3) return;

  #if FAN_COUNT > 0
    for (uint8_t i = 0; i < FAN_COUNT; i++) setTargetFan_percent(FanOff, (fan_t)i);
  #endif

  printerStatusKey[0] = 0;
  show_status = true;
  tpShowStatus = false;
  rts.sendData(ExchangePageBase + 51, ExchangepageAddr);
}

void onFilamentRunout() {
  printerStatusKey[1] = 4;
  tpShowStatus = false;
  rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
}

void onFilamentRunout(extruder_t extruder) {
  printerStatusKey[1] = 4;
  tpShowStatus = false;
  rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
}

void onUserConfirmRequired(const char *const msg) {
  printerStatusKey[1] = 4;
  tpShowStatus = false;
  if (lastPauseMsgState == ExtUI::pauseModeStatus && msg == (const char*)GET_TEXT_F(MSG_FILAMENT_CHANGE_LOAD))
    return;

  switch (ExtUI::pauseModeStatus) {
    case PAUSE_MESSAGE_WAITING: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Press Yes to Continue"));
      break;
    }
    case PAUSE_MESSAGE_INSERT: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Load Filament to           Continue"));
      break;
    }
    case PAUSE_MESSAGE_HEAT: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Add Filament and Press    Yes to Reheat"));
      break;
    }

    #if DISABLED(ADVANCED_PAUSE_CONTINUOUS_PURGE)
      case PAUSE_MESSAGE_PURGE: {
        rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
        char newMsg[40] = "Yes to ";
        strcat_P(newMsg, TERN1(HAS_FILAMENT_SENSOR, !ExtUI::getFilamentRunoutState() && getFilamentRunoutEnabled()) ? PSTR("Continue") : PSTR("Disable "));
        strcat_P(newMsg, PSTR("           No to Purge"));
        onStatusChanged(newMsg);
        break;
      }
    #endif

    case PAUSE_MESSAGE_OPTION: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      char newMsg[40] = "Yes to ";
      strcat_P(newMsg, TERN1(HAS_FILAMENT_SENSOR, !ExtUI::getFilamentRunoutState() && getFilamentRunoutEnabled()) ? PSTR("Continue") : PSTR("Disable "));
      strcat_P(newMsg, PSTR("           No to Purge"));
      onStatusChanged(newMsg);
      break;
    }

    case PAUSE_MESSAGE_PARKING: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Parking..."));
      break;
    }
    case PAUSE_MESSAGE_CHANGING: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Beginning Filament Change"));
      break;
    }
    case PAUSE_MESSAGE_UNLOAD: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Unloading..."));
      break;
    }
    case PAUSE_MESSAGE_LOAD: {
      rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
      onStatusChanged(F("Reloading..."));
      break;
    }
    case PAUSE_MESSAGE_RESUME:
      #if ENABLED(ADVANCED_PAUSE_CONTINUOUS_PURGE)
        case PAUSE_MESSAGE_PURGE: {
          rts.sendData(ExchangePageBase + 78, ExchangepageAddr);
          onStatusChanged(F("Press Yes to Stop Purge"));
          break;
        }
      #endif

    case PAUSE_MESSAGE_HEATING: {
      rts.sendData(ExchangePageBase + 68, ExchangepageAddr);
      onStatusChanged(F("Reheating"));
      break;
    }

    case PAUSE_MESSAGE_STATUS:
    default: {
      setPauseMenuResponse(PAUSE_RESPONSE_RESUME_PRINT);
      setUserConfirmed();
      break;
    }
  }
  lastPauseMsgState = ExtUI::pauseModeStatus;
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

void onStatusChanged(const char *const statMsg) {
  for (int16_t j = 0; j < 20; j++) // Clear old message
    rts.sendData(' ', StatusMessageString + j);
  rts.sendData(statMsg, StatusMessageString);
}

void onFactoryReset() {
  dwin_settings.settings_size         = sizeof(creality_dwin_settings_t);
  dwin_settings.settings_version      = dwin_settings_version;
  dwin_settings.display_standby       = true;
  dwin_settings.display_sound         = true;
  dwin_settings.display_volume        = 32;
  dwin_settings.standby_brightness    = 15;
  dwin_settings.screen_brightness     = 100;
  dwin_settings.standby_time_seconds  = 60;
  dwin_settings.screen_rotation       = 0;
  onStartup();
  startprogress = 0;
  show_status = true;
}

static_assert(eeprom_data_size >= sizeof(creality_dwin_settings_t), "Insufficient space in EEPROM for UI parameters");

void onStoreSettings(char *buff) {
  // Write to buffer
  memcpy(buff, &dwin_settings, sizeof(creality_dwin_settings_t));
}

void onLoadSettings(const char *buff) {
  creality_dwin_settings_t eepromSettings;
  memcpy(&eepromSettings, buff, sizeof(creality_dwin_settings_t));

  // If size is not the same, discard settings
  if (eepromSettings.settings_size != sizeof(creality_dwin_settings_t)) {
    onFactoryReset();
    return;
  }

  if (eepromSettings.settings_version != dwin_settings_version) {
    onFactoryReset();
    return;
  }

  // Copy into final location
  memcpy(&dwin_settings, &eepromSettings, sizeof(creality_dwin_settings_t));

  rts.setTouchScreenConfiguration();
}

void onSettingsStored(const bool success) {
  // Called after the entire EEPROM has been written,
  // whether successful or not.
}

void onSettingsLoaded(const bool success) {
  #if HAS_MESH
    if (ExtUI::getLevelingIsValid()) {
      uint8_t abl_probe_index = 0;
      for (uint8_t outer = 0; outer < GRID_MAX_POINTS_Y; outer++)
        for (uint8_t inner = 0; inner < GRID_MAX_POINTS_X; inner++) {
          const bool zig = outer & 1;
          const xy_uint8_t point = { uint8_t(zig ? (GRID_MAX_POINTS_X - 1) - inner : inner), outer };
          rts.sendData(ExtUI::getMeshPoint(point) * 1000, AutolevelVal + (abl_probe_index * 2));
          ++abl_probe_index;
        }

      rts.sendData(3, AutoLevelIcon); // 2=On, 3=Off
      setLevelingActive(true);
    }
    else {
      rts.sendData(2, AutoLevelIcon); /*Off*/
      setLevelingActive(false);
    }
  #endif

  rts.sendData(getZOffset_mm() * 100, ProbeOffset_Z);
  rts.setTouchScreenConfiguration();
}

void onPostprocessSettings() {}

#if HAS_LEVELING
  void onLevelingStart() {}

  void onLevelingDone() {
    #if HAS_MESH
      if (ExtUI::getLevelingIsValid()) {
        uint8_t abl_probe_index = 0;
        for (uint8_t outer = 0; outer < GRID_MAX_POINTS_Y; outer++)
          for (uint8_t inner = 0; inner < GRID_MAX_POINTS_X; inner++) {
            const bool zig = outer & 1;
            const xy_uint8_t point = { uint8_t(zig ? (GRID_MAX_POINTS_X - 1) - inner : inner), outer };
            rts.sendData(ExtUI::getMeshPoint(point) * 1000, AutolevelVal + abl_probe_index * 2);
            ++abl_probe_index;
          }

        rts.sendData(3, AutoLevelIcon); // 2=On, 3=Off
        setLevelingActive(true);
      }
      else {
        rts.sendData(2, AutoLevelIcon); /*Off*/
        setLevelingActive(false);
      }
    #endif
  }
#endif

#if HAS_MESH
  void onMeshUpdate(const int8_t xpos, const int8_t ypos, probe_state_t state) {}

  void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
    if (waitway == 3)
      if (isPositionKnown() && (getActualTemp_celsius(BED) >= (getTargetTemp_celsius(BED) - 1)))
        rts.sendData(ExchangePageBase + 64, ExchangepageAddr);
    #if HAS_MESH
      uint8_t abl_probe_index = 0;
      for (uint8_t outer = 0; outer < GRID_MAX_POINTS_Y; outer++)
        for (uint8_t inner = 0; inner < GRID_MAX_POINTS_X; inner++) {
          const bool zig = outer & 1; // != ((PR_OUTER_END) & 1);
          const xy_uint8_t point = { uint8_t(zig ? (GRID_MAX_POINTS_X - 1) - inner : inner), outer };
          if (point.x == xpos && outer == ypos)
            rts.sendData(ExtUI::getMeshPoint(point) * 1000, AutolevelVal + (abl_probe_index * 2));
          ++abl_probe_index;
        }
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
    startprogress = 254;
    show_status   = true;
    tpShowStatus  = false;
    no_reentry    = false;
    rts.sendData(ExchangePageBase + 76, ExchangepageAddr);
  }
#endif

#if HAS_PID_HEATING
  void onPIDTuning(const pidresult_t rst) {
    // Called for temperature PID tuning result
    rts.sendData(pid_hotendAutoTemp, HotendPID_AutoTmp);
    rts.sendData(pid_bedAutoTemp, BedPID_AutoTmp);
    rts.sendData(uint16_t(getPID_Kp(E0)) * 10, HotendPID_P);
    rts.sendData(uint16_t(getPID_Ki(E0)) * 10, HotendPID_I);
    rts.sendData(uint16_t(getPID_Kd(E0)) * 10, HotendPID_D);
    #if ENABLED(PIDTEMPBED)
      rts.sendData(uint16_t(getBedPID_Kp()) * 10, BedPID_P);
      rts.sendData(uint16_t(getBedPID_Ki()) * 10, BedPID_I);
      rts.sendData(uint16_t(getBedPID_Kd()) * 10, BedPID_D);
    #endif
    onStatusChanged(F("PID Tune Finished"));
  }
  void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
    // Called by M303 to update the UI
  }
#endif

#if ENABLED(MPC_AUTOTUNE)
  void onMPCTuning(const mpcresult_t rst) {
    // Called for temperature PID tuning result
  }
#endif

#if ENABLED(PLATFORM_M997_SUPPORT)
  void onFirmwareFlash() {}
#endif

void onHomingStart() {}
void onHomingDone() {}

void onPrintDone() {}

void onSteppersDisabled() {}
void onSteppersEnabled() {}
void onAxisDisabled(const axis_t) {}
void onAxisEnabled(const axis_t) {}

} // ExtUI

#endif // DGUS_LCD_UI_IA_CREALITY
