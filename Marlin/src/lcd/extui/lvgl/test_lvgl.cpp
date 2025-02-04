/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#if ENABLED(EXTUI_LVGL_TEST)

#include "../ui_api.h"

#include <lvgl.h>
#include "lvgl_init.h"

static void btn_event_cb(lv_obj_t * btn, lv_event_t event) {
  if (event == LV_EVENT_CLICKED) {
    static uint8_t cnt = 0;
    cnt++;

    // Get the first child of the button which is the label and change its text
    lv_obj_t * label = lv_obj_get_child(btn, NULL);
    lv_label_set_text_fmt(label, "Button: %d", cnt);
  }
}

namespace ExtUI {
  void onStartup() {
    lvgl_init();
    lv_obj_t * btn = lv_btn_create(lv_scr_act(), NULL);     // Add a button the current screen
    lv_obj_set_pos(btn, 10, 10);                            // Set its position
    lv_obj_set_size(btn, 120, 50);                          // Set its size
    lv_obj_set_event_cb(btn, btn_event_cb);                 // Assign a callback to the button

    lv_obj_t * label = lv_label_create(btn, NULL);          // Add a label to the button
    lv_label_set_text(label, "Button");                     // Set the labels text
  }
  void onIdle() { lv_task_handler(); }
  void onPrinterKilled(FSTR_P const error, FSTR_P const component) {}
  void onMediaInserted() {}
  void onMediaError() {}
  void onMediaRemoved() {}
  void onPlayTone(const uint16_t frequency, const uint16_t duration) {}
  void onPrintTimerStarted() {}
  void onPrintTimerPaused() {}
  void onPrintTimerStopped() {}
  void onFilamentRunout(const extruder_t extruder) {}
  void onUserConfirmRequired(const char * const msg) {}
  void onStatusChanged(const char * const msg) {}

  void onHomingStart() {}
  void onHomingDone() {}
  void onPrintDone() {}

  void onFactoryReset() {}

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
    void onLevelingStart() {}
    void onLevelingDone() {}
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const_float_t zval) {
      // Called when any mesh points are updated
      //SERIAL_ECHOLNPGM("onMeshUpdate() x:", xpos, " y:", ypos, " z:", zval);
    }

    void onMeshUpdate(const int8_t xpos, const int8_t ypos, const probe_state_t state) {
      // Called to indicate a special condition
      //SERIAL_ECHOLNPGM("onMeshUpdate() x:", xpos, " y:", ypos, " state:", state);
    }
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
    void onPidTuning(const result_t rst) {
      // Called for temperature PID tuning result
      switch (rst) {
        case PID_STARTED:        break;
        case PID_BAD_HEATER_ID:  break;
        case PID_TEMP_TOO_HIGH:  break;
        case PID_TUNING_TIMEOUT: break;
        case PID_DONE:           break;
      }
    }
  #endif

  void onSteppersDisabled() {}
  void onSteppersEnabled()  {}
}

#endif // EXTUI_LVGL_TEST
