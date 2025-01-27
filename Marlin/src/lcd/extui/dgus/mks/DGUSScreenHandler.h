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
#pragma once

#include "../DGUSScreenHandlerBase.h"

enum DGUS_ScreenID : uint8_t;

class DGUSScreenHandlerMKS : public DGUSScreenHandler {
public:
  DGUSScreenHandlerMKS() = default;

  #if 0
  static void sendinfoscreen_ch(const uint16_t *line1, const uint16_t *line2, const uint16_t *line3, const uint16_t *line4);
  static void sendinfoscreen_en(PGM_P const line1, PGM_P const line2, PGM_P const line3, PGM_P const line4);
  static void sendInfoScreen(const void *line1, const void *line2, const void *line3, const void *line4, uint16_t language);
  #endif

  static void screenBackChange(DGUS_VP_Variable &var, void *val_ptr);

  static void eepromControl(DGUS_VP_Variable &var, void *val_ptr);
  static void languageChange(DGUS_VP_Variable &var, void *val_ptr);
  static void getOffsetValue(DGUS_VP_Variable &var, void *val_ptr);
  static void levelControl(DGUS_VP_Variable &var, void *val_ptr);
  static void meshLevel(DGUS_VP_Variable &var, void *val_ptr);
  static void meshLevelDistanceConfig(DGUS_VP_Variable &var, void *val_ptr);
  static void manualAssistLeveling(DGUS_VP_Variable &var, void *val_ptr);
  static void zOffsetConfirm(DGUS_VP_Variable &var, void *val_ptr);
  static void zOffsetSelect(DGUS_VP_Variable &var, void *val_ptr);
  static void getManualMovestep(DGUS_VP_Variable &var, void *val_ptr);
  static void getZoffsetDistance(DGUS_VP_Variable &var, void *val_ptr);
  static void getMinExtrudeTemp(DGUS_VP_Variable &var, void *val_ptr);
  static void getParkPos(DGUS_VP_Variable &var, void *val_ptr);
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    static void handleGetExMinTemp(DGUS_VP_Variable &var, void *val_ptr);
  #endif
  static void languageDisplay(uint8_t var);
  static void tmcChangeConfig(DGUS_VP_Variable &var, void *val_ptr);
  static void getTurnOffCtrl(DGUS_VP_Variable &var, void *val_ptr);
  static void languagePInit();
  static void runoutIdle();
  static void runoutInit();
  static void extrudeLoadInit();
  static void lcdBLKAdjust(DGUS_VP_Variable &var, void *val_ptr);
  static void sdFileBack(DGUS_VP_Variable &var, void *val_ptr);

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    static void handleStepPerMMChanged(DGUS_VP_Variable &var, void *val_ptr);
    static void handleStepPerMMExtruderChanged(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  static void handleMaxSpeedChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleExtruderMaxSpeedChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleAccChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleMaxAccChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleExtruderAccChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleChangeLevelPoint(DGUS_VP_Variable &var, void *val_ptr);
  static void handleTravelAccChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleFeedRateMinChange(DGUS_VP_Variable &var, void *val_ptr);
  static void handleMin_T_F(DGUS_VP_Variable &var, void *val_ptr);

  #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
    static void filamentLoadUnload(DGUS_VP_Variable &var, void *val_ptr, const int filamentDir);
    static void filamentLoad(DGUS_VP_Variable &var, void *val_ptr);
    static void filamentUnload(DGUS_VP_Variable &var, void *val_ptr);
    static void getManualFilament(DGUS_VP_Variable &var, void *val_ptr);
    static void getManualFilamentSpeed(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  #if HAS_MEDIA
    // Marlin informed us about SD print completion.
    static void sdPrintingFinished();
  #else
    static void printReturn(DGUS_VP_Variable &var, void *val_ptr);
  #endif

  static void sendPrintTimeToDisplay(DGUS_VP_Variable &var);
  static void sendBabyStepToDisplay(DGUS_VP_Variable &var);
  static void sendFanToDisplay(DGUS_VP_Variable &var);
  static void sendGbkToDisplay(DGUS_VP_Variable &var);
  static void sendStringToDisplay_Language(DGUS_VP_Variable &var);
  static void sendTMCStepValue(DGUS_VP_Variable &var);

  static void setUint8(DGUS_VP_Variable &var, void *val_ptr);

  static bool loop();
};

enum MKS_Choose : uint8_t { MKS_Language_Choose, MKS_Language_NoChoose };
enum MKS_Language : uint8_t { MKS_SimpleChinese, MKS_English };

extern MKS_Language mks_language_index;
extern bool DGUSAutoTurnOff;

#if ENABLED(POWER_LOSS_RECOVERY)
  #define PLR_SCREEN_RECOVER MKSLCD_SCREEN_PRINT
  #define PLR_SCREEN_CANCEL MKSLCD_SCREEN_HOME
#endif

typedef DGUSScreenHandlerMKS DGUSScreenHandlerClass;
