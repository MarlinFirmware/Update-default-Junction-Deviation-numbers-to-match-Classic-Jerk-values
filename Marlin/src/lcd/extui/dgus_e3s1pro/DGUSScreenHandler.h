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
#pragma once

#include "../../../inc/MarlinConfigPre.h"

#include "config/DGUS_Addr.h"
#include "config/DGUS_Data.h"
#include "config/DGUS_Screen.h"
#include "config/DGUS_Constants.h"

#include "DGUSSDCardHandler.h"

#include "../ui_api.h"

class DGUSScreenHandler {
public:
  typedef struct {
    bool initialized;
    bool levelingEnabled;
    uint8_t volume;
    uint8_t brightness;
    DGUS_Data::Language language;
    DGUS_Data::AxisControlCommand jogLength;
    uint16_t plaExtruderTemp;
    uint16_t plaBedTemp;
    uint16_t absExtruderTemp;
    uint16_t absBedTemp;
  } eeprom_data_t;

public:
  static eeprom_data_t config;
  static uint16_t currentMeshPointIndex;
  static bool isLeveling;

  static char homeStatusMessage[128];

private:
  static bool settings_ready;
  static bool booted;

  static DGUS_ScreenID current_screen;
  static DGUS_ScreenID new_screen;
  static DGUS_ScreenID wait_return_screen;
  static DGUS_ScreenID confirm_return_screen;
  static bool full_update;
  static uint8_t angry_beeps;

  #if HAS_MEDIA
    static const char* sdPrintFilename;
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    static bool powerLossRecoveryAvailable;
  #endif

  #if ENABLED(DGUS_SOFTWARE_AUTOSCROLL)
    static ssize_t currentScrollIndex;
    static size_t pageMaxStringLen;
    static size_t pageMaxControlLen;
  #endif

  static millis_t status_expire;
  static millis_t eeprom_save;

public:
  DGUSScreenHandler() = default;

  static void init();
  static void ready();
  static void loop();

  static void printerKilled(FSTR_P const error, FSTR_P const component);
  static bool isOnUserConfirmationScreen() { return confirm_return_screen != DGUS_ScreenID::BOOT; }
  static void userConfirmRequired(const char * const msg);
  static void userConfirmation();
  static void settingsReset();
  static void storeSettings(char *buff);
  static void loadSettings(const char *buff);
  static void configurationStoreWritten(bool success);
  static void configurationStoreRead(bool success);

  static void playTone(const uint16_t frequency, const uint16_t duration=0);
  static void angryBeeps(const uint8_t beepCount);

  static void levelingStart();
  static void levelingEnd();
  static void meshUpdate(const int8_t xpos, const int8_t ypos);

  static void printTimerStarted();
  static void printTimerPaused();
  static void printTimerStopped();
  static void filamentRunout(const ExtUI::extruder_t extruder);

  #if ENABLED(DGUS_SOFTWARE_AUTOSCROLL)
    static ssize_t getScrollIndex();
    static void addCurrentPageStringLength(size_t stringLength, size_t textControlLength);
  #endif

  #if HAS_MEDIA
    // Marlin informed us that a new SD has been inserted.
    static void sdCardInserted();
    // Marlin informed us that the SD Card has been removed().
    static void sdCardRemoved();
    // Marlin informed us about a bad SD Card.
    static void sdCardError();

    static const char* getSDCardPrintFilename() { return sdPrintFilename; }
  #endif

  #if ENABLED(POWER_LOSS_RECOVERY)
    static void powerLossResume();
  #endif

  #if HAS_PID_HEATING
    static void pidTuning(const ExtUI::pidresult_t rst);
  #endif

  static void steppersStatusChanged(bool steppersEnabled);
  static void homingDone();

  static void startPrintFromSD(const char* const filename);
  static void setStatusMessage(FSTR_P msg, const millis_t duration=DGUS_STATUS_EXPIRATION_MS);
  static void setStatusMessage(const char* msg, const millis_t duration=DGUS_STATUS_EXPIRATION_MS);

  static DGUS_ScreenID getCurrentScreen();
  static void homeThenChangeScreen(DGUS_ScreenID screen);
  static void triggerScreenChange(DGUS_ScreenID screen);
  static void triggerTempScreenChange(DGUS_ScreenID screen, DGUS_ScreenID returnScreen);
  static void triggerReturnScreen();
  static bool isOnTempScreen(DGUS_ScreenID screen = DGUS_ScreenID::BOOT);
  static void triggerFullUpdate();

  static void triggerEEPROMSave();

  static bool isPrinterIdle();

private:
  static const DGUS_Addr* findScreenAddrList(DGUS_ScreenID screen);
  static bool callScreenSetup(DGUS_ScreenID screen);

  static void moveToScreen(DGUS_ScreenID screen, bool abort_wait=false);
  static bool sendScreenVPData(DGUS_ScreenID screen, bool complete_update);
  static bool refreshVP(DGUS_Addr vpAddr);
};

extern DGUSScreenHandler screen;
