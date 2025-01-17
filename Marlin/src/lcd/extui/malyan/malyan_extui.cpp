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
 * lcd/extui/malyan/malyan_extui.cpp
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(MALYAN_LCD)

#include "../ui_api.h"
#include "malyan.h"

//#include "../../marlinui.h"
//#include "../../../sd/cardreader.h"
//#include "../../../module/temperature.h"
//#include "../../../module/stepper.h"
//#include "../../../module/motion.h"
//#include "../../../libs/duration_t.h"
//#include "../../../module/printcounter.h"
//#include "../../../gcode/queue.h"

namespace ExtUI {
  void onStartup() {
    /**
     * The Malyan LCD actually runs as a separate MCU on Serial 1.
     * This code's job is to siphon the weird curly-brace commands from
     * it and translate into ExtUI operations where possible.
     */
    inbound_count = 0;

    #ifndef LCD_BAUDRATE
      #define LCD_BAUDRATE 500000
    #endif
    LCD_SERIAL.begin(LCD_BAUDRATE);

    // Signal init
    write_to_lcd(F("{SYS:STARTED}\r\n"));

    // send a version that says "unsupported"
    write_to_lcd(F("{VER:99}\r\n"));

    // No idea why it does this twice.
    write_to_lcd(F("{SYS:STARTED}\r\n"));
    update_usb_status(true);
  }

  void onIdle() {
    /**
     * - from printer on startup:
     * {SYS:STARTED}{VER:29}{SYS:STARTED}{R:UD}
     */

    // First report USB status.
    update_usb_status(false);

    // now drain commands...
    while (LCD_SERIAL.available())
      parse_lcd_byte((byte)LCD_SERIAL.read());

    #if HAS_MEDIA
      // The way last printing status works is simple:
      // The UI needs to see at least one TQ which is not 100%
      // and then when the print is complete, one which is.
      static uint8_t last_percent_done = 100;

      // If there was a print in progress, we need to emit the final
      // print status as {TQ:100}. Reset last percent done so a new print will
      // issue a percent of 0.
      const uint8_t percent_done = (ExtUI::isPrinting() || ExtUI::isPrintingFromMediaPaused()) ? ExtUI::getProgress_percent() : last_printing_status ? 100 : 0;
      if (percent_done != last_percent_done) {
        char message_buffer[16];
        sprintf_P(message_buffer, PSTR("{TQ:%03i}"), percent_done);
        write_to_lcd(message_buffer);
        last_percent_done = percent_done;
        last_printing_status = ExtUI::isPrinting();
      }
    #endif
  }

  void onPrinterKilled(FSTR_P const error, FSTR_P const component) {
    set_lcd_error(error, component);
  }

  void onPrintTimerStarted() { write_to_lcd(F("{SYS:BUILD}")); }
  void onPrintTimerPaused() {}
  void onPrintTimerStopped() { write_to_lcd(F("{TQ:100}")); }

  // Not needed for Malyan LCD
  void onStatusChanged(const char * const) {}

  void onMediaMounted() {}
  void onMediaError() {}
  void onMediaRemoved() {}

  void onHeatingError(const heater_id_t header_id) {}
  void onMinTempError(const heater_id_t header_id) {}
  void onMaxTempError(const heater_id_t header_id) {}

  void onPlayTone(const uint16_t, const uint16_t/*=0*/) {}

  void onFilamentRunout(const extruder_t extruder) {}
  void onUserConfirmRequired(const char * const) {}

  // For fancy LCDs include an icon ID, message, and translated button title
  void onUserConfirmRequired(const int icon, const char * const cstr, FSTR_P const fBtn) {}
  void onUserConfirmRequired(const int icon, FSTR_P const fstr, FSTR_P const fBtn) {}

  #if ENABLED(ADVANCED_PAUSE_FEATURE)
    void onPauseMode(
      const PauseMessage message,
      const PauseMode mode/*=PAUSE_MODE_SAME*/,
      const uint8_t extruder/*=active_extruder*/
    ) {}
  #endif

  void onHomingStart() {}
  void onHomingDone() {}

  void onPrintDone() {}
  void onFactoryReset() {}

  void onStoreSettings(char*) {}
  void onLoadSettings(const char*) {}
  void onPostprocessSettings() {}
  void onSettingsStored(const bool) {}
  void onSettingsLoaded(const bool) {}

  #if HAS_LEVELING
    void onLevelingStart() {}
    void onLevelingDone() {}
    #if ENABLED(PREHEAT_BEFORE_LEVELING)
      celsius_t getLevelingBedTemp() { return LEVELING_BED_TEMP; }
    #endif
  #endif

  #if HAS_MESH
    void onMeshUpdate(const int8_t, const int8_t, const_float_t) {}
    void onMeshUpdate(const int8_t, const int8_t, const ExtUI::probe_state_t) {}
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
          set_lcd_error(GET_TEXT_F(MSG_PID_AUTOTUNE));
          break;
        case PID_BAD_HEATER_ID:
          set_lcd_error(GET_TEXT_F(MSG_PID_BAD_HEATER_ID));
          break;
        case PID_TEMP_TOO_HIGH:
          set_lcd_error(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH));
          break;
        case PID_TUNING_TIMEOUT:
          set_lcd_error(GET_TEXT_F(MSG_PID_TIMEOUT));
          break;
        case PID_DONE:
          set_lcd_error(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE));
          break;
      }
    }

    void onStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
      // Called by M303 to update the UI
    }

  #endif // HAS_PID_HEATING

  #if ENABLED(MPC_AUTOTUNE)
    void onMPCTuning(const mpcresult_t rst) {
      // Called for temperature MPC tuning result
      switch (rst) {
        case MPC_STARTED:     break;
        case MPC_TEMP_ERROR:  break;
        case MPC_INTERRUPTED: break;
        case MPC_DONE:        break;
      }
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

#endif // MALYAN_LCD
