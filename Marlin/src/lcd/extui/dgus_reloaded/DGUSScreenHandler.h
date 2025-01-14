/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#pragma once

#include "config/DGUS_Addr.h"
#include "config/DGUS_Data.h"
#include "config/DGUS_ScreenID.h"
#include "config/DGUS_Constants.h"

#include "../ui_api.h"
#include "../../../inc/MarlinConfigPre.h"

class DGUSScreenHandler {
public:
  DGUSScreenHandler() = default;

  static void init();
  static void ready();
  static void loop();

  static void printerKilled(FSTR_P const error, FSTR_P const component);
  static void userConfirmRequired(const char * const msg);
  static void settingsReset();
  static void storeSettings(char *buff);
  static void loadSettings(const char *buff);
  static void configurationStoreWritten(bool success);
  static void configurationStoreRead(bool success);

  static void playTone(const uint16_t frequency, const uint16_t duration=0);
  static void meshUpdate(const int8_t xpos, const int8_t ypos);
  static void printTimerStarted();
  static void printTimerPaused();
  static void printTimerStopped();
  static void filamentRunout(const ExtUI::extruder_t extruder);

  #if HAS_MEDIA
    // Marlin informed us that a new SD has been inserted.
    static void sdCardInserted();
    // Marlin informed us that the SD Card has been removed().
    static void sdCardRemoved();
    // Marlin informed us about a bad SD Card.
    static void sdCardError();
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    static void powerLossResume();
  #endif

  #if HAS_PID_HEATING
    static void pidTuning(const ExtUI::pidresult_t rst);
  #endif

  static void setMessageLine(const char * const msg, const uint8_t line);
  static void setMessageLine_P(PGM_P const msg, const uint8_t line);
  static void setMessageLine(FSTR_P const msg, const uint8_t line) { setMessageLine_P(FTOP(msg), line); }

  static void setStatusMessage(const char* msg, const millis_t duration=DGUS_STATUS_EXPIRATION_MS);
  static void setStatusMessage(FSTR_P const msg, const millis_t duration=DGUS_STATUS_EXPIRATION_MS);

  static void showWaitScreen(const DGUS_ScreenID return_screenID, const bool has_continue=false);
  static void showWaitScreen(FSTR_P const msg, const DGUS_ScreenID return_screenID, const bool has_continue=false);

  static DGUS_ScreenID getCurrentScreen();
  static void triggerScreenChange(const DGUS_ScreenID screenID);
  static void triggerFullUpdate();

  static void triggerEEPROMSave();

  static bool isPrinterIdle();

  static uint8_t debug_count;

  #if HAS_MEDIA
    static ExtUI::FileList filelist;
    static uint16_t filelist_offset;
    static int16_t filelist_selected;
  #endif

  static DGUS_Data::StepSize offset_steps;
  static DGUS_Data::StepSize move_steps;

  static uint16_t probing_icons[2];

  static DGUS_Data::Extruder filament_extruder;
  static uint16_t filament_length;

  static char gcode[DGUS_GCODE_LEN + 1];

  static DGUS_Data::Heater pid_heater;
  static celsius_t pid_temp;
  static uint8_t pid_cycles;

  static bool wait_continue;

  static bool leveling_active;

private:
  static const DGUS_Addr* findScreenAddrList(const DGUS_ScreenID screenID);
  static bool callScreenSetup(const DGUS_ScreenID screenID);

  static void moveToScreen(const DGUS_ScreenID screenID, bool abort_wait=false);
  static bool sendScreenVPData(const DGUS_ScreenID screenID, bool complete_update);

  static bool settings_ready;
  static bool booted;

  static DGUS_ScreenID current_screenID;
  static DGUS_ScreenID new_screenID;
  static bool full_update;

  static DGUS_ScreenID wait_return_screenID;

  static millis_t status_expire;
  static millis_t eeprom_save;

  typedef struct {
    bool initialized;
    uint8_t volume;
    uint8_t brightness;
    bool abl_okay;
  } eeprom_data_t;
};

extern DGUSScreenHandler screen;
