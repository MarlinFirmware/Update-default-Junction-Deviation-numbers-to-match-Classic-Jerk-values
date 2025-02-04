/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

/*********************
 * proui_extui.cpp   *
 *********************/

/****************************************************************************
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../../../inc/MarlinConfigPre.h"

#if ALL(DWIN_LCD_PROUI, EXTENSIBLE_UI)

#include "dwin_popup.h"

#include "../../extui/ui_api.h"
#include "../../../module/stepper.h"

#if ENABLED(PREVENT_COLD_EXTRUSION)
  #include "../../../module/temperature.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if HAS_MESH
  #include "../../../feature/bedlevel/bedlevel.h"
  #include "meshviewer.h"
#endif

namespace ExtUI {
  void onStartup() { dwinInitScreen(); }
  void onIdle() {}
  void onPrinterKilled(FSTR_P const error, FSTR_P const component) {}

  void onMediaMounted() {}
  void onMediaError()   {}
  void onMediaRemoved() {}

  void onHeatingError(const heater_id_t heater_id) {
    dwinPopupTemperature(heater_id, 2); // "Heating failed"
  }
  void onMinTempError(const heater_id_t heater_id) {
    dwinPopupTemperature(heater_id, 0); // "Too low"
  }
  void onMaxTempError(const heater_id_t heater_id) {
    dwinPopupTemperature(heater_id, 1); // "Too high"
  }

  void onPlayTone(const uint16_t frequency, const uint16_t duration/*=0*/) {}

  void onPrintTimerStarted() {}
  void onPrintTimerPaused() {}
  void onPrintTimerStopped() {}

  #if HAS_FILAMENT_SENSOR
    void onFilamentRunout(const extruder_t extruder) {
      UNUSED(extruder);
      dwinFilamentRunout();
    }
  #endif

  void onUserConfirmRequired(const char * const cstr) {
    // TODO: A version of this method that takes an icon and button title,
    // or implement some kind of ExtUI enum.
    dwinPopupConfirm(ICON_Continue_1, cstr, GET_TEXT_F(MSG_USERWAIT));
  }
  // For fancy LCDs include an icon ID, message, and translated button title
  void onUserConfirmRequired(const int icon, const char * const cstr, FSTR_P const fBtn) {
    dwinPopupConfirm(icon, cstr, fBtn);
  }
  void onUserConfirmRequired(const int icon, FSTR_P const fstr, FSTR_P const fBtn) {
    dwinPopupConfirm(icon, fstr, fBtn);
  }

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void onPauseMode(const PauseMessage message, const PauseMode mode/*=PAUSE_MODE_SAME*/, const uint8_t extruder/*=active_extruder*/) {
      if (mode != PAUSE_MODE_SAME) pause_mode = mode;
      switch (message) {
        case PAUSE_MESSAGE_PARKING:  dwinPopupPause(GET_TEXT_F(MSG_PAUSE_PRINT_PARKING)); break; // M125
        case PAUSE_MESSAGE_CHANGING: dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_INIT)); break; // pause_print (M125, M600)
        case PAUSE_MESSAGE_WAITING:  dwinPopupPause(GET_TEXT_F(MSG_ADVANCED_PAUSE_WAITING), BTN_Continue); break;
        case PAUSE_MESSAGE_INSERT:   dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_INSERT), BTN_Continue); break;
        case PAUSE_MESSAGE_LOAD:     dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_LOAD)); break;
        case PAUSE_MESSAGE_UNLOAD:   dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_UNLOAD)); break; // Unload of pause and Unload of M702
        case PAUSE_MESSAGE_PURGE:    dwinPopupPause(GET_TEXT_F(TERN(ADVANCED_PAUSE_CONTINUOUS_PURGE, MSG_FILAMENT_CHANGE_CONT_PURGE, MSG_FILAMENT_CHANGE_PURGE))); break;
        case PAUSE_MESSAGE_OPTION:   gotoFilamentPurge(); break;
        case PAUSE_MESSAGE_RESUME:   dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_RESUME)); break;
        case PAUSE_MESSAGE_HEAT:     dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_HEAT), BTN_Continue); break;
        case PAUSE_MESSAGE_HEATING:  dwinPopupPause(GET_TEXT_F(MSG_FILAMENT_CHANGE_HEATING)); break;
        case PAUSE_MESSAGE_STATUS:   hmiReturnScreen(); break; // Exit from Pause, Load and Unload
        default: break;
      }
    }
  #endif

  void onStatusChanged(const char * const msg) {
    UNUSED(msg);
    dwinCheckStatusMessage();
    dwinDrawStatusMessage();
  }

  void onHomingStart() { dwinHomingStart(); }
  void onHomingDone() { dwinHomingDone(); }

  void onPrintDone() {}

  void onFactoryReset() { dwinSetDataDefaults(); }

  // Copy settings to EEPROM buffer for write
  void onStoreSettings(char *buff) { dwinCopySettingsTo(buff); }

  // Get settings from loaded EEPROM data
  void onLoadSettings(const char *buff) { dwinCopySettingsFrom(buff); }

  void onPostprocessSettings() {
    // Called after loading or resetting stored settings
  }

  void onSettingsStored(const bool) {
    // Called after the entire EEPROM has been written,
    // whether successful or not.
  }

  void onSettingsLoaded(const bool) {
    // Called after the entire EEPROM has been read,
    // whether successful or not.
  }

  #if HAS_LEVELING
    void onLevelingStart() { dwinLevelingStart(); }
    void onLevelingDone() { dwinLevelingDone(); }
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp() { return hmiData.bedLevT; }
    #endif
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
      dwinMeshUpdate(xpos, ypos, zval);
    }
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const probe_state_t state) {
      // Called to indicate a special condition
    }
    void onMeshUpdate(const xy_int8_t xypos, const_float_t zval) {
      // Called to indicate a special condition
      meshViewer.drawMeshPoint(xypos.x, xypos.y, zval);
    }
  #endif

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    void onSetMinExtrusionTemp(const celsius_t t) { hmiData.extMinT = t; }
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    void onSetPowerLoss(const bool) {
      // Called when power-loss is enabled/disabled
    }
    void onPowerLoss() {
      // Called when power-loss state is detected
    }
    void onPowerLossResume() {
      // Called on resume from power-loss
      recovery.ui_flag_resume = true;
    }
  #endif

  #if HAS_PID_HEATING
    void onPIDTuning(const pidresult_t rst) {
      // Called for temperature PID tuning result
      dwinPIDTuning(tempcontrol_t(rst));
    }
    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
      dwinStartM303(count, hid, temp);
    }
  #endif

  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst) {
      // Called for temperature MPC tuning result
      dwinMPCTuning(tempcontrol_t(rst));
    }
  #endif

  #if ENABLED(PLATFORM_M997_SUPPORT)
    void onFirmwareFlash() { dwinRebootScreen(); }
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled() {}
  void onAxisDisabled(const axis_t a) {
    AxisEnum axis;
    switch (a) {
      TERN_(HAS_X_AXIS, case X:)
      default: axis = X_AXIS;
      OPTCODE(HAS_Y_AXIS, case Y: axis = Y_AXIS)
      OPTCODE(HAS_Z_AXIS, case Z: axis = Z_AXIS)
      OPTCODE(HAS_I_AXIS, case I: axis = I_AXIS)
      OPTCODE(HAS_J_AXIS, case J: axis = J_AXIS)
      OPTCODE(HAS_K_AXIS, case K: axis = K_AXIS)
      OPTCODE(HAS_U_AXIS, case U: axis = U_AXIS)
      OPTCODE(HAS_V_AXIS, case V: axis = V_AXIS)
      OPTCODE(HAS_W_AXIS, case W: axis = W_AXIS)
    }
    set_axis_never_homed(AxisEnum(axis)); // MRISCOC workaround: https://github.com/MarlinFirmware/Marlin/issues/23095
  }
  void onAxisEnabled(const axis_t) {}
} // ExtUI

#endif // DWIN_LCD_PROUI && EXTENSIBLE_UI
