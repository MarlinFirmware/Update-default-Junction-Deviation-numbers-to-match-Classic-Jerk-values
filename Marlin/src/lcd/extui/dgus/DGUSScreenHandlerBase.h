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
#pragma once

#include "DGUSDisplay.h"
#include "DGUSVPVariable.h"
#include "DGUSDisplayDef.h"

#include "../../../inc/MarlinConfig.h"

enum DGUS_ScreenID : uint8_t;

class DGUSScreenHandler {
public:
  DGUSScreenHandler() = default;

  static bool loop();

  // Send all 4 strings that are displayed on the infoscreen, confirmation screen and kill screen
  // The bools specifying whether the strings are in RAM or FLASH.
  static void sendInfoScreen_P(PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash);
  static void sendInfoScreen(FSTR_P const line1, FSTR_P const line2, PGM_P const line3, PGM_P const line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash) {
    sendInfoScreen_P(FTOP(line1), FTOP(line2), line3, line4, l1inflash, l2inflash, l3inflash, liinflash);
  }
  static void sendInfoScreen(FSTR_P const line1, FSTR_P const line2, FSTR_P const line3, FSTR_P const line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash) {
    sendInfoScreen_P(FTOP(line1), FTOP(line2), FTOP(line3), FTOP(line4), l1inflash, l2inflash, l3inflash, liinflash);
  }

  static void handleUserConfirmationPopUp(uint16_t confirmVP, PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4, bool l1inflash, bool l2inflash, bool l3inflash, bool liinflash);

  // "M117" Message -- msg is a RAM ptr.
  static void setStatusMessage(const char *msg);
  // The same for messages from Flash
  static void setstatusmessagePGM(PGM_P const msg);
  static void setStatusMessage(FSTR_P const fmsg) { setstatusmessagePGM(FTOP(fmsg)); }
  // Callback for VP "Display wants to change screen on idle printer"
  static void screenChangeHookIfIdle(DGUS_VP_Variable &var, void *val_ptr);
  // Callback for VP "Screen has been changed"
  static void screenChangeHook(DGUS_VP_Variable &var, void *val_ptr);

  // Callback for VP "All Heaters Off"
  static void handleAllHeatersOff(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for "Change this temperature"
  static void handleTemperatureChanged(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for "Change Flowrate"
  static void handleFlowRateChanged(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
    // Hook for manual move option
    static void handleManualMoveOption(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  // Hook for manual move.
  static void handleManualMove(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for manual extrude.
  static void handleManualExtrude(DGUS_VP_Variable &var, void *val_ptr);
  // Hook for motor lock and unlook
  static void handleMotorLockUnlock(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(POWER_LOSS_RECOVERY)
    // Hook for power loss recovery.
    static void handlePowerLossRecovery(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  // Hook for settings
  static void handleSettings(DGUS_VP_Variable &var, void *val_ptr);

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    static void handleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr);
    static void handleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  #if HAS_PID_HEATING
    // Hook for "Change this temperature PID para"
    static void handleTemperaturePIDChanged(DGUS_VP_Variable &var, void *val_ptr);
    // Hook for PID autotune
    static void handlePIDAutotune(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if HAS_BED_PROBE
    // Hook for "Change probe offset z"
    static void handleProbeOffsetZChanged(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if ENABLED(BABYSTEPPING)
    // Hook for live z adjust action
    static void handleLiveAdjustZ(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if HAS_FAN
    // Hook for fan control
    static void handleFanControl(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  // Hook for heater control
  static void handleHeaterControl(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(DGUS_PREHEAT_UI)
    // Hook for preheat
    static void handlePreheat(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
    // Hook for filament load and unload filament option
    static void handleFilamentOption(DGUS_VP_Variable &var, void *val_ptr);
    // Hook for filament load and unload
    static void handleFilamentLoadUnload(DGUS_VP_Variable &var);
  #endif

  #if HAS_MEDIA
    // Callback for VP "Display wants to change screen when there is a SD card"
    static void screenChangeHookIfSD(DGUS_VP_Variable &var, void *val_ptr);
    // Scroll buttons on the file listing screen.
    static void sdScrollFilelist(DGUS_VP_Variable &var, void *val_ptr);
    // File touched.
    static void sdFileSelected(DGUS_VP_Variable &var, void *val_ptr);
    // start print after confirmation received.
    static void sdStartPrint(DGUS_VP_Variable &var, void *val_ptr);
    // User hit the pause, resume or abort button.
    static void sdResumePauseAbort(DGUS_VP_Variable &var, void *val_ptr);
    // User confirmed the abort action
    static void sdReallyAbort(DGUS_VP_Variable &var, void *val_ptr);
    // User hit the tune button
    static void sdPrintTune(DGUS_VP_Variable &var, void *val_ptr);
    // Send a single filename to the display.
    static void sdSendFilename(DGUS_VP_Variable &var);
    // Marlin informed us that a new SD has been inserted.
    static void sdCardInserted();
    // Marlin informed us that the SD Card has been removed().
    static void sdCardRemoved();
    // Marlin informed us about a bad SD Card.
    static void sdCardError();
  #endif

  // OK Button on the Confirm screen.
  static void screenConfirmedOK(DGUS_VP_Variable &var, void *val_ptr);

  // Update data after going to a new screen (by display or by gotoScreen)
  // remember to store the last-displayed screen so it can be restored.
  // (e.g., for popup messages)
  static void updateNewScreen(const DGUS_ScreenID screenID, const bool popup=false);

  // Recall the remembered screen.
  static void popToOldScreen();

  // Make the display show the screen and update all VPs in it.
  static void gotoScreen(const DGUS_ScreenID screenID, const bool popup=false);

  static void updateScreenVPData();

  // Helpers to convert and transfer data to the display.
  static void sendWordValueToDisplay(DGUS_VP_Variable &var);
  static void sendStringToDisplay(DGUS_VP_Variable &var);
  static void sendStringToDisplayPGM(DGUS_VP_Variable &var);
  static void sendTemperaturePID(DGUS_VP_Variable &var);
  static void sendPercentageToDisplay(DGUS_VP_Variable &var);
  static void sendPrintProgressToDisplay(DGUS_VP_Variable &var);
  static void sendPrintTimeToDisplay(DGUS_VP_Variable &var);

  #if ENABLED(PRINTCOUNTER)
    static void sendPrintAccTimeToDisplay(DGUS_VP_Variable &var);
    static void sendPrintsTotalToDisplay(DGUS_VP_Variable &var);
  #endif
  #if HAS_FAN
    static void sendFanStatusToDisplay(DGUS_VP_Variable &var);
  #endif
  static void sendHeaterStatusToDisplay(DGUS_VP_Variable &var);
  #if ENABLED(DGUS_UI_WAITING)
    static void sendWaitingStatusToDisplay(DGUS_VP_Variable &var);
  #endif

  // Send a value from 0..100 to a variable with a range from 0..255
  static void percentageToUint8(DGUS_VP_Variable &var, void *val_ptr);

  template<typename T>
  static void setValueDirectly(DGUS_VP_Variable &var, void *val_ptr) {
    if (!var.memadr) return;
    union { unsigned char tmp[sizeof(T)]; T t; } x;
    unsigned char *ptr = (unsigned char*)val_ptr;
    for (uint_fast8_t i = 0; i < sizeof(T); ++i) x.tmp[i] = ptr[sizeof(T) - i - 1];
    *(T*)var.memadr = x.t;
  }

  // Send a float value to the display.
  // Display will get a 4-byte integer scaled to the number of digits:
  // Tell the display the number of digits and it cheats by displaying a dot between...
  template<uint16_t decimals>
  static void sendFloatAsLongValueToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      float f = *(float *)var.memadr;
      f *= cpow(10, decimals);
      dgus.writeVariable(var.VP, (long)f);
    }
  }

  // Send a float value to the display.
  // Display will get a 2-byte integer scaled to the number of digits:
  // Tell the display the number of digits and it cheats by displaying a dot between...
  template<uint16_t decimals>
  static void sendFloatAsIntValueToDisplay(DGUS_VP_Variable &var) {
    if (var.memadr) {
      float f = *(float *)var.memadr;
      DEBUG_ECHOLNPGM(" >> ", p_float_t(f, 6));
      f *= cpow(10, decimals);
      dgus.writeVariable(var.VP, (int16_t)f);
    }
  }

  // Force an update of all VP on the current screen.
  static void forceCompleteUpdate() { update_ptr = 0; screenComplete = false; }
  // Has all VPs sent to the screen
  static bool isScreenComplete() { return screenComplete; }

  static DGUS_ScreenID getCurrentScreen() { return current_screenID; }

  static void setupConfirmAction( void (*f)()) { confirm_action_cb = f; }

protected:
  static DGUS_ScreenID current_screenID;  //< currently on screen
  static constexpr uint8_t NUM_PAST_SCREENS = 4;
  static DGUS_ScreenID past_screenIDs[NUM_PAST_SCREENS]; //< LIFO with past screens for the "back" button.

  static uint8_t update_ptr;      //< Last sent entry in the VPList for the actual screen.
  static uint16_t skipVP;         //< When updating the screen data, skip this one, because the user is interacting with it.
  static bool screenComplete;     //< All VPs sent to screen?

  static uint16_t confirmVP;      //< context for confirm screen (VP that will be emulated-sent on "OK").

  #if HAS_MEDIA
    static int16_t top_file;      //< file on top of file chooser
    static int16_t file_to_print; //< touched file to be confirmed
  #endif

  static void (*confirm_action_cb)();
};
