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

#include "../../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_CREALITY_TOUCH

#include "DGUSDisplayDef.h"
#include "../DGUSDisplay.h"
#include "../DGUSScreenHandler.h"
#include "PageHandlers.h"

#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"
#include "../../../../feature/caselight.h"

#include "../../ui_api.h"
#include "../../../marlinui.h" // For material presets

#if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
  uint16_t distanceToMove = 10;
#endif
using namespace ExtUI;

PGMSTR(MarlinVersion, SHORT_BUILD_VERSION);

// ----- Which variables to auto-update on which screens
const uint16_t VPList_None[] PROGMEM = { 0x0000 };
const uint16_t VPList_DialogStop[] PROGMEM = { 0x0000 };

const uint16_t VPList_Main[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,
  #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
    VP_PrintProgress_Percentage,
  #endif
  0x0000
};

const uint16_t VPList_SDFileList[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,
  #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
    VP_PrintProgress_Percentage,
  #endif

  VP_SD_FileName0,
  VP_SD_FileName1,
  VP_SD_FileName2,
  VP_SD_FileName3,
  VP_SD_FileName4,
  VP_SD_FileName5,

  0x0000
};

const uint16_t VPList_Control[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,
  #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
    VP_PrintProgress_Percentage,
  #endif

  VP_LED_TOGGLE,

  0x0000
};

const uint16_t VPList_Feed[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,


  0x0000
};

const uint16_t VPList_Temp[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,
  #if ENABLED(LCD_SET_PROGRESS_MANUALLY)
    VP_PrintProgress_Percentage,
  #endif

  VP_FAN_TOGGLE,

  0x0000
};


const uint16_t VPList_PreheatPLASettings[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_PREHEAT_PLA_HOTEND_TEMP,
  VP_PREHEAT_PLA_BED_TEMP,

  0x0000
};

const uint16_t VPList_PreheatABSSettings[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_PREHEAT_ABS_HOTEND_TEMP,
  VP_PREHEAT_ABS_BED_TEMP,

  0x0000
};


const uint16_t VPList_PrintPausingError[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_PrintProgress_Percentage,
  VP_PrintTimeProgressBar,
  VP_PrintTime,

  0x0000
};

const uint16_t VPList_PrintScreen[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  VP_M117,

  VP_PrintTime,

  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_PrintProgress_Percentage,
  VP_PrintTimeProgressBar,
  VP_PrintTime,

  0x0000
};

const uint16_t VPList_Leveling[] PROGMEM = {
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_MESH_LEVEL_TEMP,
  VP_MESH_LEVEL_STATUS,

  0x0000
};

const uint16_t VPList_ZOffsetLevel[] PROGMEM = {
  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  0x0000
};

const uint16_t VPList_TuneScreen[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  VP_M117,

  VP_PrintTime,

  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_LED_TOGGLE,
  VP_FAN_TOGGLE,


  0x0000
};

const uint16_t VPList_Prepare[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  VP_M117,

  VP_PrintTime,

  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_STEPPERS,

  0x0000
};

const uint16_t VPList_Info[] PROGMEM = {
  /* VP_M117, for completeness, but it cannot be auto-uploaded. */
  VP_M117,

  VP_PrintTime,

  #if HOTENDS >= 1
    VP_T_E0_Is, VP_T_E0_Set,// VP_E0_STATUS,
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set,// VP_BED_STATUS,
  #endif
  /*VP_XPos, VP_YPos,*/ VP_ZPos,
  //VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  VP_PRINTER_BEDSIZE,
  VP_MARLIN_VERSION,

  0x0000
};

// Toggle button handler
void DGUSCrealityDisplay_HandleToggleButton(DGUS_VP_Variable &var, void *val_ptr) {
  switch (*(uint16_t*)var.memadr) {
    case ICON_TOGGLE_ON:
      *((bool*)var.memadr) = true;
      break;

    case ICON_TOGGLE_OFF:
      *((bool*)var.memadr) = true;
      break;
  }
}

void DGUSCrealityDisplay_SendToggleButton(DGUS_VP_Variable &var) {
  dgus.writeVariable(var.VP, *(bool*)var.memadr ? ICON_TOGGLE_ON : ICON_TOGGLE_OFF);
}

// -- Mapping from screen to variable list
const struct VPMapping VPMap[] PROGMEM = {
  { DGUS_SCREEN_BOOT, VPList_None },
  { DGUS_SCREEN_MAIN, VPList_Main },

  { DGUS_SCREEN_SDFILELIST, VPList_SDFileList },

  { DGUS_SCREEN_FILAMENTRUNOUT1, VPList_PrintPausingError },
  { DGUS_SCREEN_FILAMENTRUNOUT2, VPList_PrintPausingError },

  { DGUS_SCREEN_PRINT_FINISH, VPList_PrintScreen },
  { DGUS_SCREEN_PRINT_RUNNING, VPList_PrintScreen },
  { DGUS_SCREEN_PRINT_PAUSED, VPList_PrintScreen },

  { DGUS_SCREEN_TUNING, VPList_TuneScreen },
  { DGUS_SCREEN_PREPARE, VPList_Prepare },

  { DGUS_SCREEN_INFO, VPList_Info },

  { DGUS_SCREEN_MOVE1MM, VPList_PrintScreen },
  { DGUS_SCREEN_MOVE10MM, VPList_PrintScreen },
  { DGUS_SCREEN_MOVE01MM, VPList_PrintScreen },

  { DGUS_SCREEN_FEED, VPList_Feed },
  { DGUS_SCREEN_CONTROL, VPList_Control },

  { DGUS_SCREEN_TEMP, VPList_Temp },
  { DGUS_SCREEN_TEMP_PLA, VPList_PreheatPLASettings },
  { DGUS_SCREEN_TEMP_ABS, VPList_PreheatABSSettings },

  { DGUS_SCREEN_INFO, VPList_PrintScreen },
  { DGUS_SCREEN_ZOFFSET_LEVEL, VPList_ZOffsetLevel },
  { DGUS_SCREEN_LEVELING, VPList_Leveling },

  { DGUS_SCREEN_POWER_LOSS, VPList_None },
  { DGUS_SCREEN_THERMAL_RUNAWAY, VPList_None },
  { DGUS_SCREEN_HEATING_FAILED, VPList_None },
  { DGUS_SCREEN_THERMISTOR_ERROR, VPList_None },

  { DGUS_SCREEN_AUTOHOME, VPList_None },

  { DGUS_SCREEN_DIALOG_PAUSE, VPList_None },
  { DGUS_SCREEN_DIALOG_STOP, VPList_DialogStop },

  { DGUS_SCREEN_CONFIRM, VPList_None },
  { DGUS_SCREEN_POPUP, VPList_None },


  { 0 , nullptr } // List is terminated with an nullptr as table entry.
};

const struct DGUS_VP_Variable ListOfVP[] PROGMEM = {
  // TODO:

  #if HOTENDS >= 1
    VPHELPER(VP_T_E0_Is, &thermalManager.temp_hotend[0].celsius, nullptr, screen.sendFloatAsIntValueToDisplay<0>),
    VPHELPER(VP_T_E0_Set, &thermalManager.temp_hotend[0].target, screen.handleTemperatureChanged, &screen.sendWordValueToDisplay),
  #endif

  #if HAS_HEATED_BED
    VPHELPER(VP_T_Bed_Is, &thermalManager.temp_bed.celsius, nullptr, screen.sendFloatAsIntValueToDisplay<0>),
    VPHELPER(VP_T_Bed_Set, &thermalManager.temp_bed.target, screen.handleTemperatureChanged, &screen.sendWordValueToDisplay),
  #endif

  VPHELPER(VP_MESH_LEVEL_TEMP, &thermalManager.temp_hotend[0].target, nullptr, &screen.sendWordValueToDisplay),
  VPHELPER(VP_MESH_LEVEL_STATUS, nullptr, nullptr, nullptr),

  // Feedrate
  VPHELPER(VP_Feedrate_Percentage, &feedrate_percentage, screen.setValueDirectly<int16_t>, &screen.sendWordValueToDisplay ),

  VPHELPER(VP_PrintProgress_Percentage, nullptr, nullptr, screen.sendPrintProgressToDisplay),
  VPHELPER(VP_PrintTimeProgressBar, nullptr, nullptr, screen.sendPrintProgressToDisplay),

  // Preheat settings
  #if PREHEAT_COUNT
    #ifdef PREHEAT_1_LABEL
      VPHELPER(VP_PREHEAT_PLA_HOTEND_TEMP, &ui.material_preset[0].hotend_temp, screen.setValueDirectly<int16_t>, &screen.sendWordValueToDisplay ),
      VPHELPER(VP_PREHEAT_PLA_BED_TEMP, &ui.material_preset[0].bed_temp, screen.setValueDirectly<int16_t>, &screen.sendWordValueToDisplay ),
    #endif
    #ifdef PREHEAT_2_LABEL
      VPHELPER(VP_PREHEAT_ABS_HOTEND_TEMP, &ui.material_preset[1].hotend_temp, screen.setValueDirectly<int16_t>, &screen.sendWordValueToDisplay ),
      VPHELPER(VP_PREHEAT_ABS_BED_TEMP, &ui.material_preset[1].bed_temp, screen.setValueDirectly<int16_t>, &screen.sendWordValueToDisplay ),
    #endif
  #endif

  // About info
  VPHELPER(VP_MARLIN_VERSION, nullptr, nullptr, screen.sendAboutFirmwareVersion),
  VPHELPER(VP_PRINTER_BEDSIZE, nullptr, nullptr, screen.sendAboutPrintSize),

  // Position Data
  //VPHELPER(VP_XPos, &current_position.x, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
  //VPHELPER(VP_YPos, &current_position.y, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
  //VPHELPER(VP_YPos, &current_position.y, nullptr, screen.sendFloatAsLongValueToDisplay<2>),

  VPHELPER(VP_ZPos, &probe.offset.z, screen.handleProbeOffsetZChanged, screen.sendFloatAsIntValueToDisplay<2>),

  VPHELPER(VP_FAN_TOGGLE, &thermalManager.fan_speed[0], screen.handleFanControl, screen.sendFanStatusToDisplay),

  #if ENABLED(POWER_LOSS_RECOVERY)
    VPHELPER(VP_POWER_LOSS_RECOVERY, nullptr, &screen.handlePowerLossRecovery, nullptr),
  #endif

  VPHELPER_STR(VP_PrintTime, nullptr, VP_PrintTime_LEN, nullptr, screen.sendPrintTimeToDisplay ),
  VPHELPER(VP_SCREENCHANGE, nullptr, screen.screenChangeHook, nullptr),
  VPHELPER(VP_CONFIRMED, nullptr, screen.screenConfirmedOK, nullptr),

  // Feed
  VPHELPER(VP_FEED_AMOUNT, &screen.feed_amount, screen.handleFeedAmountChanged, screen.sendFloatAsIntValueToDisplay<2>),

  // Creality has the same button ID mapped all over the place, so let the generic handler figure it out
  VPHELPER(VP_BUTTON_MAINENTERKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_ADJUSTENTERKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_PREPAREENTERKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_RESUMEPRINTKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_SELECTFILEKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_STARTPRINTKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_STOPPRINTKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_PAUSEPRINTKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_COOLDOWN, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_TEMPCONTROL, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_BEDLEVELKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),
  VPHELPER(VP_BUTTON_HEATLOADSTARTKEY, nullptr, DGUSCrealityDisplay_HandleReturnKeyEvent, nullptr),

  // File listing
  VPHELPER(VP_SD_ScrollEvent, nullptr, screen.sdScrollFilelist, nullptr),
  VPHELPER(VP_SD_FileSelected, nullptr, screen.sdFileSelected, nullptr),
  VPHELPER(VP_SD_FileSelectConfirm, nullptr, screen.sdStartPrint, nullptr),
  VPHELPER_STR(VP_SD_FileName0,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),
  VPHELPER_STR(VP_SD_FileName1,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),
  VPHELPER_STR(VP_SD_FileName2,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),
  VPHELPER_STR(VP_SD_FileName3,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),
  VPHELPER_STR(VP_SD_FileName4,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),
  VPHELPER_STR(VP_SD_FileName5,  nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename ),

  // Icons
  VPHELPER(VP_STEPPERS, &screen.are_steppers_enabled, nullptr, (screen.sendIconValue<ICON_TOGGLE_OFF, ICON_TOGGLE_ON>)),
  VPHELPER(VP_LED_TOGGLE, &caselight.on, nullptr, (screen.sendIconValue<ICON_TOGGLE_ON,ICON_TOGGLE_OFF>)),

  // M117 LCD String (We don't need the string in memory but "just" push it to the display on demand, hence the nullptr
  { .VP = VP_M117, .memadr = nullptr, .size = VP_M117_LEN, .set_by_display_handler = nullptr, .send_to_display_handler =&screen.sendStringToDisplay },

 // Messages for the User, shared by the popup and the kill screen. They cant be autouploaded as we do not buffer content.
  { .VP = VP_MSGSTR1, .memadr = nullptr, .size = VP_MSGSTR1_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = &screen.sendStringToDisplayPGM },
  { .VP = VP_MSGSTR2, .memadr = nullptr, .size = VP_MSGSTR2_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = &screen.sendStringToDisplayPGM },
  { .VP = VP_MSGSTR3, .memadr = nullptr, .size = VP_MSGSTR3_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = &screen.sendStringToDisplayPGM },
  //{ .VP = VP_MSGSTR4, .memadr = nullptr, .size = VP_MSGSTR4_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = &screen.sendStringToDisplayPGM },

  VPHELPER(0, 0, 0, 0)  // must be last entry.
};

#endif // DGUS_LCD_UI_ORIGIN
