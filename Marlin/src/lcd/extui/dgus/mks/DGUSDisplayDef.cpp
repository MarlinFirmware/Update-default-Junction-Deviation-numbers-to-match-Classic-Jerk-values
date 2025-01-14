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

#include "../../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_MKS

#include "DGUSDisplayDef.h"
#include "../DGUSDisplay.h"
#include "../DGUSScreenHandler.h"

#include "../../../../module/temperature.h"
#include "../../../../module/motion.h"
#include "../../../../module/planner.h"

#include "../../ui_api.h"
#include "../../../marlinui.h" // For material presets

#if HAS_STEALTHCHOP
  #include "../../../../module/stepper/trinamic.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../../feature/powerloss.h"
#endif

#if ENABLED(DGUS_UI_MOVE_DIS_OPTION)
  uint16_t distanceToMove = 10;
#endif

uint16_t manualMoveStep = 1;
uint16_t distanceFilament = 10;
uint16_t filamentSpeed_mm_s = 25;
float ZOffset_distance = 0.1;
float mesh_adj_distance = 0.01;
float Z_distance = 0.1;

//struct { uint16_t h, m, s; } dgus_time;

//
// Persistent settings
//
xy_int_t mks_corner_offsets[5];   // Initialized by settings.load
xyz_int_t mks_park_pos;           // Initialized by settings.load
celsius_t mks_min_extrusion_temp; // Initialized by settings.load

void MKS_reset_settings() {
  constexpr xy_int_t init_dgus_level_offsets[5] = {
    { 20, 20 }, { 20, 20 },
    { 20, 20 }, { 20, 20 },
    { X_CENTER, Y_CENTER }
  };
  mks_language_index = MKS_English;
  COPY(mks_corner_offsets, init_dgus_level_offsets);
  mks_park_pos.set(20, 20, 10);
  mks_min_extrusion_temp = 0;
}

xyz_pos_t position_before_pause;
constexpr feedRate_t park_speed_xy = TERN(NOZZLE_PARK_FEATURE, NOZZLE_PARK_XY_FEEDRATE, 100),
                     park_speed_z  = TERN(NOZZLE_PARK_FEATURE, NOZZLE_PARK_Z_FEEDRATE,    5);

void MKS_pause_print_move() {
  queue.exhaust();
  position_before_pause = current_position;

  // Save the current position, the raise amount, and 'already raised'
  TERN_(POWER_LOSS_RECOVERY, if (recovery.enabled) recovery.save(true, mks_park_pos.z, true));

  destination.z = _MIN(current_position.z + mks_park_pos.z, Z_MAX_POS);
  prepare_internal_move_to_destination(park_speed_z);

  destination.set(X_MIN_POS + mks_park_pos.x, Y_MIN_POS + mks_park_pos.y);
  prepare_internal_move_to_destination(park_speed_xy);
}

void MKS_resume_print_move() {
  destination.set(position_before_pause.x, position_before_pause.y);
  prepare_internal_move_to_destination(park_speed_xy);
  destination.z = position_before_pause.z;
  prepare_internal_move_to_destination(park_speed_z);
  TERN_(POWER_LOSS_RECOVERY, if (recovery.enabled) recovery.save(true));
}

float z_offset_add = 0;

xyz_int_t tmc_step; // = { 0, 0, 0 }

uint16_t lcd_default_light = 50;

EX_FILAMENT_DEF ex_filament;
RUNOUT_MKS_DEF runout_mks;
NOZZLE_PARK_DEF nozzle_park_mks;

const uint16_t VPList_Boot[] PROGMEM = {
  VP_MARLIN_VERSION,
  0x0000
};

#define MKSLIST_E_ITEM(N) VP_T_E##N##_Is, VP_T_E##N##_Set,

const uint16_t VPList_Main[] PROGMEM = {
  // VP_M117, for completeness, but it cannot be auto-uploaded.
  #if HAS_HOTEND
    MKSLIST_E_ITEM(0) VP_E0_STATUS,
    #if HAS_MULTI_HOTEND
      MKSLIST_E_ITEM(1) VP_E1_STATUS,
    #endif
  #endif
  #if HAS_HEATED_BED
    VP_T_Bed_Is, VP_T_Bed_Set, VP_BED_STATUS,
  #endif
  #if HAS_FAN
    VP_Fan0_Percentage, VP_FAN0_STATUS,
  #endif
  VP_XPos, VP_YPos, VP_ZPos,
  VP_Fan0_Percentage,
  VP_Feedrate_Percentage,
  #if ENABLED(SET_PROGRESS_PERCENT)
    VP_PrintProgress_Percentage,
  #endif
  0x0000
};

const uint16_t MKSList_Home[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,
  // Language
  // VP_HOME_Dis,

  0x0000
};

const uint16_t MKSList_Setting[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,
  // Language
  VP_Setting_Dis,
  0x0000
};

const uint16_t MKSList_Tool[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,
  // Language
  VP_Tool_Dis,
  // LCD BLK
  VP_LCD_BLK,
  0x0000
};

const uint16_t MKSList_EXTRUE[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  VP_Filament_distance,
  VP_Filament_speed,

  0x0000
};

const uint16_t MKSList_LEVEL[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  0x0000
};

const uint16_t MKSList_MOVE[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  0x0000
};

const uint16_t MKSList_Print[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,
  // Print Percent
  VP_PrintProgress_Percentage,

  VP_PrintTime,

  VP_Flowrate_E0,
  VP_Flowrate_E1,
  VP_Feedrate_Percentage,

  VP_PrintTime_H,
  VP_PrintTime_M,
  VP_PrintTime_S,

  VP_XPos,
  VP_YPos,
  VP_ZPos,

  0x0000
};

const uint16_t MKSList_SD_File[] PROGMEM = {
  VP_SD_FileName0, VP_SD_FileName1,
  VP_SD_FileName2, VP_SD_FileName3,
  VP_SD_FileName4, VP_SD_FileName5,
  VP_SD_FileName6, VP_SD_FileName7,
  VP_SD_FileName8, VP_SD_FileName9,

  0x0000
};

const uint16_t MKSList_TempOnly[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,
  // LCD BLK
  VP_LCD_BLK,
  0x0000
};

const uint16_t MKSList_Pluse[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  // Pluse
  VP_X_STEP_PER_MM,
  VP_Y_STEP_PER_MM,
  VP_Z_STEP_PER_MM,
  VP_E0_STEP_PER_MM,
  VP_E1_STEP_PER_MM,

  0x0000
};

const uint16_t MKSList_MaxSpeed[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  // Pluse
  VP_X_MAX_SPEED,
  VP_Y_MAX_SPEED,
  VP_Z_MAX_SPEED,
  VP_E0_MAX_SPEED,
  VP_E1_MAX_SPEED,

  0x0000
};

const uint16_t MKSList_MaxAcc[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  // ACC
  VP_ACC_SPEED,
  VP_X_ACC_MAX_SPEED,
  VP_Y_ACC_MAX_SPEED,
  VP_Z_ACC_MAX_SPEED,
  VP_E0_ACC_MAX_SPEED,
  VP_E1_ACC_MAX_SPEED,

  0x0000
};

const uint16_t MKSList_PID[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  // PID
  VP_E0_PID_P,
  VP_E0_PID_I,
  VP_E0_PID_D,

  0x0000
};

const uint16_t MKSList_Level_Point[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  // FAN
  VP_Fan0_Percentage,

  // Level Point
  VP_Level_Point_One_X,
  VP_Level_Point_One_Y,
  VP_Level_Point_Two_X,
  VP_Level_Point_Two_Y,
  VP_Level_Point_Three_X,
  VP_Level_Point_Three_Y,
  VP_Level_Point_Four_X,
  VP_Level_Point_Four_Y,
  VP_Level_Point_Five_X,
  VP_Level_Point_Five_Y,

  0x0000
};

const uint16_t MKSList_Level_PrintConfig[] PROGMEM = {
  VP_T_E0_Set,
  VP_T_E1_Set,
  VP_T_Bed_Set,
  VP_Flowrate_E0,
  VP_Flowrate_E1,
  VP_Fan0_Percentage,
  VP_Feedrate_Percentage,

  0x0000
};

const uint16_t MKSList_PrintPauseConfig[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,

  VP_X_PARK_POS,
  VP_Y_PARK_POS,
  VP_Z_PARK_POS,

  0x0000
};

const uint16_t MKSList_MotoConfig[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,

  VP_TRAVEL_SPEED,
  VP_FEEDRATE_MIN_SPEED,
  VP_T_F_SPEED,

  0x0000
};

const uint16_t MKSList_EX_Config[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  VP_MIN_EX_T,VP_Min_EX_T_E,
  0x0000
};

const uint16_t MKSTMC_Config[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  // HB Temp
  VP_T_Bed_Is, VP_T_Bed_Set,
  VP_MIN_EX_T,

  VP_TMC_X_STEP,
  VP_TMC_Y_STEP,
  VP_TMC_Z_STEP,
  VP_TMC_X1_Current,
  VP_TMC_Y1_Current,
  VP_TMC_X_Current,
  VP_TMC_Y_Current,
  VP_TMC_Z_Current,
  VP_TMC_E0_Current,
  VP_TMC_E1_Current,
  VP_TMC_Z1_Current,

  0x0000
};

const uint16_t MKSAuto_Level[] PROGMEM = {
  VP_MESH_LEVEL_POINT_DIS,
  VP_ZPos,
  0x0000
};

const uint16_t MKSOffset_Config[] PROGMEM = {
  // E Temp
  REPEAT(EXTRUDERS, MKSLIST_E_ITEM)
  VP_OFFSET_X,
  VP_OFFSET_Y,
  VP_OFFSET_Z,
  0x0000
};

const uint16_t MKSBabyStep[] PROGMEM = {
  VP_ZOffset_DE_DIS,
  0x0000
};

const uint16_t MKSList_About[] PROGMEM = {
  // Marlin version
  VP_MARLIN_VERSION,
  // H43 Version
  VP_MKS_H43_VERSION,
  VP_MKS_H43_UpdataVERSION,
  0x0000
};

// Page data updata
const struct VPMapping VPMap[] PROGMEM = {
  { MKSLCD_SCREEN_BOOT, VPList_Boot },                        // Boot Page to show logo  0
  { MKSLCD_SCREEN_HOME, MKSList_Home },                       // Home, Page 1
  { MKSLCD_SCREEN_SETTING, MKSList_Setting },                 // Setting, Page 2
  { MKSLCD_SCREEM_TOOL, MKSList_Tool },                       // Page 3
  { MKSLCD_SCREEN_EXTRUDE_P1, MKSList_EXTRUE },               // Page 4
  { MKSLCD_SCREEN_EXTRUDE_P2, MKSList_EXTRUE },               // Page 11
  { MKSLCD_PAUSE_SETTING_EX, MKSList_EXTRUE },                // Page 57
  { MKSLCD_PAUSE_SETTING_EX2, MKSList_EXTRUE },               // Page 61
  { MKSLCD_SCREEN_LEVEL, MKSList_LEVEL },                     // Page 5
  { MKSLCD_SCREEN_MOVE, MKSList_MOVE },                       // Page 6
  { MKSLCD_SCREEN_PRINT, MKSList_Print },                     // Page 7
  { MKSLCD_SCREEN_PAUSE, MKSList_Print },                     // Page 26
  { MKSLCD_SCREEN_CHOOSE_FILE, MKSList_SD_File },             // Page 15
  { MKSLCD_SCREEN_MOTOR_PLUSE, MKSList_Pluse },               // Page 51
  { MKSLCD_SCREEN_MOTOR_SPEED, MKSList_MaxSpeed },            // Page 55
  { MKSLCD_SCREEN_MOTOR_ACC_MAX, MKSList_MaxAcc },            // Page 53
  { MKSLCD_SCREEN_LEVEL_DATA, MKSList_Level_Point },          // Page 48
  { MKSLCD_PrintPause_SET, MKSList_PrintPauseConfig },        // Page 49
  { MKSLCD_FILAMENT_DATA, MKSList_SD_File },                  // Page 50
  { MKSLCD_SCREEN_Config, MKSList_TempOnly },                 // Page 46
  { MKSLCD_SCREEN_Config_MOTOR, MKSList_MotoConfig },         // Page 47
  { MKSLCD_PID, MKSList_PID },                                // Page 56
  { MKSLCD_ABOUT, MKSList_About },                            // Page 36
  { MKSLCD_SCREEN_PRINT_CONFIG, MKSList_Level_PrintConfig },  // Page 60
  { MKSLCD_SCREEN_EX_CONFIG, MKSList_EX_Config },             // Page 65
  { MKSLCD_SCREEN_TMC_Config, MKSTMC_Config },                // Page 70
  { MKSLCD_AUTO_LEVEL, MKSAuto_Level },                       // Page 73
  { MKSLCD_Screen_Offset_Config, MKSOffset_Config },          // Page 30
  { MKSLCD_Screen_PMove, MKSList_MOVE },                      // Page 64
  { MKSLCD_Screen_Baby, MKSBabyStep },                        // Page 71
  //{ MKSLCD_SCREEN_LEVEL_DATA, MKSList_SD_File},
  //{ MKSLCD_SCREEN_HOME, VPList_Boot },
  { 0, nullptr } // List is terminated with an nullptr as table entry.
};

const char MarlinVersion[] PROGMEM = SHORT_BUILD_VERSION;
const char H43Version[] PROGMEM = "MKS H43_V1.30";
const char Updata_Time[] PROGMEM = STRING_DISTRIBUTION_DATE;

const struct DGUS_VP_Variable ListOfVP[] PROGMEM = {
  // Helper to detect touch events
  VPHELPER(VP_SCREENCHANGE, nullptr, screen.screenChangeHook, nullptr),
  VPHELPER(VP_SCREENCHANGE_ASK, nullptr, screen.screenChangeHookIfIdle, nullptr),
  #if HAS_MEDIA
    VPHELPER(VP_SCREENCHANGE_WHENSD, nullptr, screen.screenChangeHookIfSD, nullptr),
  #endif
  VPHELPER(VP_CONFIRMED, nullptr, screen.screenConfirmedOK, nullptr),

  // Back Button
  VPHELPER(VP_BACK_PAGE, nullptr, screen.screenBackChange, nullptr),
  VPHELPER(VP_TEMP_ALL_OFF, nullptr, screen.handleAllHeatersOff, nullptr),

  VPHELPER(VP_MOVE_X, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_MOVE_Y, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_MOVE_Z, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_HOME_ALL, nullptr, screen.handleManualMove, nullptr),

  VPHELPER(VP_X_HOME, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_Y_HOME, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_Z_HOME, nullptr, screen.handleManualMove, nullptr),

  VPHELPER(VP_MOVE_DISTANCE, &manualMoveStep, screen.getManualMovestep, nullptr),

  VPHELPER(VP_MOTOR_LOCK_UNLOK, nullptr, screen.handleManualMove, nullptr),
  VPHELPER(VP_LEVEL_POINT, nullptr, screen.manualAssistLeveling, nullptr),

  #if ENABLED(POWER_LOSS_RECOVERY)
    VPHELPER(VP_POWER_LOSS_RECOVERY, nullptr, screen.handlePowerLossRecovery, nullptr),
  #endif
  VPHELPER(VP_SETTINGS, nullptr, screen.handleSettings, nullptr),
  #if ENABLED(SINGLE_Z_CALIBRATION)
    VPHELPER(VP_Z_CALIBRATE, nullptr, screen.handleZCalibration, nullptr),
  #endif
  #if ENABLED(FIRST_LAYER_CAL)
    VPHELPER(VP_Z_FIRST_LAYER_CAL, nullptr, screen.handleFirstLayerCal, nullptr),
  #endif
  {.VP = VP_MARLIN_VERSION, .memadr = (void *)MarlinVersion, .size = VP_MARLIN_VERSION_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},
  // M117 LCD String (We don't need the string in memory but "just" push it to the display on demand, hence the nullptr
  {.VP = VP_M117, .memadr = nullptr, .size = VP_M117_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplay},
  {.VP = VP_MKS_H43_VERSION, .memadr = (void *)H43Version, .size = VP_MKS_H43_VERSION_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},
  {.VP = VP_MKS_H43_UpdataVERSION, .memadr = (void *)Updata_Time, .size = VP_MKS_H43_VERSION_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},

  // Temperature Data
  #if HAS_HOTEND
    VPHELPER(VP_T_E0_Is, &thermalManager.temp_hotend[0].celsius, nullptr, screen.sendFloatAsLongValueToDisplay<0>),
    VPHELPER(VP_T_E0_Set, &thermalManager.temp_hotend[0].target, screen.handleTemperatureChanged, screen.sendWordValueToDisplay),
    VPHELPER(VP_Flowrate_E0, &planner.flow_percentage[ExtUI::extruder_t::E0], screen.handleFlowRateChanged, screen.sendWordValueToDisplay),
    VPHELPER(VP_EPos, &destination.e, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
    VPHELPER(VP_MOVE_E0, nullptr, screen.handleManualExtrude, nullptr),
    VPHELPER(VP_E0_CONTROL, &thermalManager.temp_hotend[0].target, screen.handleHeaterControl, nullptr),
    VPHELPER(VP_E0_STATUS, &thermalManager.temp_hotend[0].target, nullptr, screen.sendHeaterStatusToDisplay),
    #if ENABLED(DGUS_PREHEAT_UI)
      VPHELPER(VP_E0_BED_PREHEAT, nullptr, screen.handlePreheat, nullptr),
    #endif
    #if ENABLED(PIDTEMP)
      VPHELPER(VP_E0_PID_P, &thermalManager.temp_hotend[0].pid.Kp, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_E0_PID_I, &thermalManager.temp_hotend[0].pid.Ki, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_E0_PID_D, &thermalManager.temp_hotend[0].pid.Kd, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_PID_AUTOTUNE_E0, nullptr, screen.handlePIDAutotune, nullptr),
    #endif
    #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
      VPHELPER(VP_LOAD_Filament, nullptr, screen.filamentLoad, nullptr),
      VPHELPER(VP_UNLOAD_Filament, nullptr, screen.filamentUnload, nullptr),
      VPHELPER(VP_Filament_distance, &distanceFilament, screen.getManualFilament, screen.sendWordValueToDisplay),
      VPHELPER(VP_Filament_speed, &filamentSpeed_mm_s, screen.getManualFilamentSpeed, screen.sendWordValueToDisplay),
    #endif
  #endif

  #if HAS_MULTI_HOTEND
    VPHELPER(VP_T_E1_Is, &thermalManager.temp_hotend[1].celsius, nullptr, screen.sendFloatAsLongValueToDisplay<0>),
    VPHELPER(VP_T_E1_Set, &thermalManager.temp_hotend[1].target, screen.handleTemperatureChanged, screen.sendWordValueToDisplay),
    VPHELPER(VP_Flowrate_E1, &planner.flow_percentage[ExtUI::extruder_t::E1], screen.handleFlowRateChanged, screen.sendWordValueToDisplay),
    VPHELPER(VP_MOVE_E1, nullptr, screen.handleManualExtrude, nullptr),
    VPHELPER(VP_E1_CONTROL, &thermalManager.temp_hotend[1].target, screen.handleHeaterControl, nullptr),
    VPHELPER(VP_E1_STATUS, &thermalManager.temp_hotend[1].target, nullptr, screen.sendHeaterStatusToDisplay),

    #if ENABLED(DGUS_FILAMENT_LOADUNLOAD)
      VPHELPER(VP_Filament_distance, &distanceFilament, screen.getManualFilament, screen.sendFloatAsIntValueToDisplay<0>),
      VPHELPER(VP_Filament_speed, &filamentSpeed_mm_s, screen.getManualFilamentSpeed, screen.sendWordValueToDisplay),
    #endif

    #if ENABLED(PIDTEMP)
      VPHELPER(VP_PID_AUTOTUNE_E1, nullptr, screen.handlePIDAutotune, nullptr),
    #endif

    VPHELPER(VP_E1_FILAMENT_LOAD_UNLOAD, nullptr, screen.handleFilamentOption, screen.handleFilamentLoadUnload),
  #endif

  #if HAS_HEATED_BED
    VPHELPER(VP_T_Bed_Is, &thermalManager.temp_bed.celsius, nullptr, screen.sendFloatAsLongValueToDisplay<0>),
    VPHELPER(VP_T_Bed_Set, &thermalManager.temp_bed.target, screen.handleTemperatureChanged, screen.sendWordValueToDisplay),
    VPHELPER(VP_BED_CONTROL, &thermalManager.temp_bed.target, screen.handleHeaterControl, nullptr),
    VPHELPER(VP_BED_STATUS, &thermalManager.temp_bed.target, nullptr, screen.sendHeaterStatusToDisplay),
    #if ENABLED(PIDTEMPBED)
      VPHELPER(VP_BED_PID_P, &thermalManager.temp_bed.pid.Kp, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_BED_PID_I, &thermalManager.temp_bed.pid.Ki, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_BED_PID_D, &thermalManager.temp_bed.pid.Kd, screen.handleTemperaturePIDChanged, screen.sendTemperaturePID),
      VPHELPER(VP_PID_AUTOTUNE_BED, nullptr, screen.handlePIDAutotune, nullptr),
    #endif
  #endif

  // Fan Data
  #if HAS_FAN
    #if HOTENDS <= 4
      #define FAN_CONTROL HOTENDS
    #elif FAN_COUNT <= 4
      #define FAN_CONTROL FAN_COUNT
    #else
      #define FAN_CONTROL 4
    #endif
    #define FAN_VPHELPER(N) \
      VPHELPER(VP_Fan##N##_Percentage, &thermalManager.fan_speed[N], screen.percentageToUint8, screen.sendFanToDisplay), \
      VPHELPER(VP_FAN##N##_CONTROL, &thermalManager.fan_speed[N], screen.handleFanControl, nullptr), \
      VPHELPER(VP_FAN##N##_STATUS, &thermalManager.fan_speed[N], nullptr, screen.sendFanStatusToDisplay),
    REPEAT(FAN_CONTROL, FAN_VPHELPER)
  #endif

  // Feedrate
  VPHELPER(VP_Feedrate_Percentage, &feedrate_percentage, screen.setValueDirectly<int16_t>, screen.sendWordValueToDisplay),

  // Position Data
  VPHELPER(VP_XPos, &current_position.x, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
  VPHELPER(VP_YPos, &current_position.y, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
  VPHELPER(VP_ZPos, &current_position.z, nullptr, screen.sendFloatAsLongValueToDisplay<2>),

  // Level Point Set
  VPHELPER(VP_Level_Point_One_X, &mks_corner_offsets[0].x, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_One_Y, &mks_corner_offsets[0].y, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Two_X, &mks_corner_offsets[1].x, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Two_Y, &mks_corner_offsets[1].y, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Three_X, &mks_corner_offsets[2].x, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Three_Y, &mks_corner_offsets[2].y, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Four_X, &mks_corner_offsets[3].x, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Four_Y, &mks_corner_offsets[3].y, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Five_X, &mks_corner_offsets[4].x, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),
  VPHELPER(VP_Level_Point_Five_Y, &mks_corner_offsets[4].y, screen.handleChangeLevelPoint, screen.sendWordValueToDisplay),

  // Print Progress
  VPHELPER(VP_PrintProgress_Percentage, nullptr, nullptr, screen.sendPrintProgressToDisplay),

  // LCD Control
  VPHELPER(VP_LCD_BLK, &lcd_default_light, screen.lcdBLKAdjust, screen.sendWordValueToDisplay),

  // SD File - Back
  VPHELPER(VP_SD_FileSelect_Back, nullptr, screen.sdFileBack, nullptr),

  // Print Time
  VPHELPER_STR(VP_PrintTime, nullptr, VP_PrintTime_LEN, nullptr, screen.sendPrintTimeToDisplay),

  #if ENABLED(PRINTCOUNTER)
    VPHELPER_STR(VP_PrintAccTime, nullptr, VP_PrintAccTime_LEN, nullptr, screen.sendPrintAccTimeToDisplay),
    VPHELPER_STR(VP_PrintsTotal, nullptr, VP_PrintsTotal_LEN, nullptr, screen.sendPrintsTotalToDisplay),
  #endif

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    VPHELPER(VP_X_STEP_PER_MM, &planner.settings.axis_steps_per_mm[X_AXIS], screen.handleStepPerMMChanged, screen.sendFloatAsIntValueToDisplay<0>),
    VPHELPER(VP_Y_STEP_PER_MM, &planner.settings.axis_steps_per_mm[Y_AXIS], screen.handleStepPerMMChanged, screen.sendFloatAsIntValueToDisplay<0>),
    VPHELPER(VP_Z_STEP_PER_MM, &planner.settings.axis_steps_per_mm[Z_AXIS], screen.handleStepPerMMChanged, screen.sendFloatAsIntValueToDisplay<0>),
  #endif

  VPHELPER(VP_X_MAX_SPEED, &planner.settings.max_feedrate_mm_s[X_AXIS], screen.handleMaxSpeedChange, screen.sendFloatAsIntValueToDisplay<0>),
  VPHELPER(VP_Y_MAX_SPEED, &planner.settings.max_feedrate_mm_s[Y_AXIS], screen.handleMaxSpeedChange, screen.sendFloatAsIntValueToDisplay<0>),
  VPHELPER(VP_Z_MAX_SPEED, &planner.settings.max_feedrate_mm_s[Z_AXIS], screen.handleMaxSpeedChange, screen.sendFloatAsIntValueToDisplay<0>),

  #if HAS_HOTEND
    VPHELPER(VP_E0_MAX_SPEED, &planner.settings.max_feedrate_mm_s[E_AXIS_N(0)], screen.handleExtruderMaxSpeedChange, screen.sendFloatAsIntValueToDisplay<0>),
    #if HAS_MULTI_HOTEND
      VPHELPER(VP_E1_MAX_SPEED, &planner.settings.max_feedrate_mm_s[E_AXIS_N(1)], screen.handleExtruderMaxSpeedChange, screen.sendFloatAsIntValueToDisplay<0>),
    #endif
  #endif

  VPHELPER(VP_X_ACC_MAX_SPEED, (uint16_t *)&planner.settings.max_acceleration_mm_per_s2[X_AXIS], screen.handleMaxAccChange, screen.sendWordValueToDisplay),
  VPHELPER(VP_Y_ACC_MAX_SPEED, (uint16_t *)&planner.settings.max_acceleration_mm_per_s2[Y_AXIS], screen.handleMaxAccChange, screen.sendWordValueToDisplay),
  VPHELPER(VP_Z_ACC_MAX_SPEED, (uint16_t *)&planner.settings.max_acceleration_mm_per_s2[Z_AXIS], screen.handleMaxAccChange, screen.sendWordValueToDisplay),

  #if HAS_HOTEND
    VPHELPER(VP_E0_ACC_MAX_SPEED, (uint16_t *)&planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(0)], screen.handleExtruderAccChange, screen.sendWordValueToDisplay),
    #if HAS_MULTI_HOTEND
      VPHELPER(VP_E1_ACC_MAX_SPEED, (uint16_t *)&planner.settings.max_acceleration_mm_per_s2[E_AXIS_N(1)], screen.handleExtruderAccChange, screen.sendWordValueToDisplay),
    #endif
  #endif

  VPHELPER(VP_TRAVEL_SPEED, (uint16_t *)&planner.settings.travel_acceleration, screen.handleTravelAccChange, screen.sendFloatAsIntValueToDisplay<0>),
  VPHELPER(VP_FEEDRATE_MIN_SPEED, (uint16_t *)&planner.settings.min_feedrate_mm_s, screen.handleFeedRateMinChange, screen.sendFloatAsIntValueToDisplay<0>),
  VPHELPER(VP_T_F_SPEED, (uint16_t *)&planner.settings.min_travel_feedrate_mm_s, screen.handleMin_T_F, screen.sendFloatAsIntValueToDisplay<0>),
  VPHELPER(VP_ACC_SPEED, (uint16_t *)&planner.settings.acceleration, screen.handleAccChange, screen.sendWordValueToDisplay),

  VPHELPER(VP_X_PARK_POS, &mks_park_pos.x, screen.getParkPos, screen.sendWordValueToDisplay),
  VPHELPER(VP_Y_PARK_POS, &mks_park_pos.y, screen.getParkPos, screen.sendWordValueToDisplay),
  VPHELPER(VP_Z_PARK_POS, &mks_park_pos.z, screen.getParkPos, screen.sendWordValueToDisplay),

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    VPHELPER(VP_MIN_EX_T, &thermalManager.extrude_min_temp, screen.handleGetExMinTemp, screen.sendWordValueToDisplay),
  #endif

  #if ENABLED(SENSORLESS_HOMING)  // TMC SENSORLESS Setting
    #if X_HAS_STEALTHCHOP
      VPHELPER(VP_TMC_X_STEP, &tmc_step.x, screen.tmcChangeConfig, screen.sendTMCStepValue),
    #endif
    #if Y_HAS_STEALTHCHOP
      VPHELPER(VP_TMC_Y_STEP, &tmc_step.y, screen.tmcChangeConfig, screen.sendTMCStepValue),
    #endif
    #if Z_HAS_STEALTHCHOP
      VPHELPER(VP_TMC_Z_STEP, &tmc_step.z, screen.tmcChangeConfig, screen.sendTMCStepValue),
    #endif
  #endif

  #if HAS_TRINAMIC_CONFIG   // TMC Current Setting
    #if AXIS_IS_TMC(X)
      VPHELPER(VP_TMC_X_Current, &stepperX.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(Y)
      VPHELPER(VP_TMC_Y_Current, &stepperY.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(Z)
      VPHELPER(VP_TMC_Z_Current, &stepperZ.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(E0)
      VPHELPER(VP_TMC_E0_Current, &stepperE0.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(E1)
      VPHELPER(VP_TMC_E1_Current, &stepperE1.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(X2)
      VPHELPER(VP_TMC_X1_Current, &stepperX2.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(Y2)
      VPHELPER(VP_TMC_Y1_Current, &stepperY2.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
    #if AXIS_IS_TMC(Z2)
      VPHELPER(VP_TMC_Z1_Current, &stepperZ2.val_mA, screen.tmcChangeConfig, screen.sendWordValueToDisplay),
    #endif
  #endif

  VPHELPER(VP_EEPROM_CTRL, nullptr, screen.eepromControl, nullptr),
  VPHELPER(VP_LEVEL_BUTTON, nullptr, screen.levelControl, nullptr),
  VPHELPER(VP_LANGUAGE_CHANGE, nullptr, screen.languageChange, nullptr),

  //VPHELPER(VP_SD_Print_LiveAdjustZ, nullptr, screen.handleLiveAdjustZ, nullptr),

  VPHELPER(VP_SD_Print_LiveAdjustZ_Confirm, nullptr, screen.zOffsetConfirm, nullptr),

  VPHELPER(VP_ZOffset_Distance,nullptr ,screen.getZoffsetDistance, nullptr),
  VPHELPER(VP_MESH_LEVEL_ADJUST, nullptr, screen.meshLevelDistanceConfig, nullptr),
  VPHELPER(VP_MESH_LEVEL_POINT,nullptr, screen.meshLevel, nullptr),

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    VPHELPER(VP_Min_EX_T_E, &thermalManager.extrude_min_temp, screen.getMinExtrudeTemp, screen.sendWordValueToDisplay),
  #endif

  VPHELPER(VP_AutoTurnOffSw, nullptr, screen.getTurnOffCtrl, nullptr),

  #if ENABLED(EDITABLE_STEPS_PER_UNIT)
    #if HAS_HOTEND
      VPHELPER(VP_E0_STEP_PER_MM, &planner.settings.axis_steps_per_mm[E_AXIS_N(0)], screen.handleStepPerMMExtruderChanged, screen.sendFloatAsIntValueToDisplay<0>),
      #if HAS_MULTI_HOTEND
        VPHELPER(VP_E1_STEP_PER_MM, &planner.settings.axis_steps_per_mm[E_AXIS_N(1)], screen.handleStepPerMMExtruderChanged, screen.sendFloatAsIntValueToDisplay<0>),
      #endif
    #endif
  #endif

  // SDCard File listing

  #if HAS_MEDIA
    VPHELPER(VP_SD_FileSelected, nullptr, screen.sdFileSelected, nullptr),
    VPHELPER(VP_SD_ScrollEvent, nullptr, screen.sdScrollFilelist, nullptr),
    VPHELPER(VP_SD_FileSelectConfirm, nullptr, screen.sdStartPrint, nullptr),
    VPHELPER_STR(VP_SD_FileName0, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName1, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName2, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName3, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName4, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName5, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName6, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName7, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName8, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER_STR(VP_SD_FileName9, nullptr, VP_SD_FileName_LEN, nullptr, screen.sdSendFilename),
    VPHELPER(VP_SD_ResumePauseAbort, nullptr, screen.sdResumePauseAbort, nullptr),
    VPHELPER(VP_SD_AbortPrintConfirmed, nullptr, screen.sdReallyAbort, nullptr),
    VPHELPER(VP_SD_Print_Setting, nullptr, screen.sdPrintTune, nullptr),
    #if ENABLED(BABYSTEPPING)
      VPHELPER(VP_SD_Print_LiveAdjustZ, nullptr, screen.handleLiveAdjustZ, screen.sendFloatAsIntValueToDisplay<2>),
      VPHELPER(VP_ZOffset_DE_DIS, &z_offset_add, nullptr, screen.sendFloatAsLongValueToDisplay<2>),
    #endif
    #if HAS_BED_PROBE
      VPHELPER(VP_OFFSET_X, &probe.offset.x, screen.getOffsetValue,screen.sendFloatAsLongValueToDisplay<2>),
      VPHELPER(VP_OFFSET_Y, &probe.offset.y, screen.getOffsetValue,screen.sendFloatAsLongValueToDisplay<2>),
      VPHELPER(VP_OFFSET_Z, &probe.offset.z, screen.getOffsetValue,screen.sendFloatAsLongValueToDisplay<2>),
    #endif
  #else
    VPHELPER(VP_SD_FileSelected, nullptr, screen.printReturn, nullptr),
  #endif

  #if ENABLED(DGUS_UI_WAITING)
    VPHELPER(VP_WAITING_STATUS, nullptr, nullptr, screen.sendWaitingStatusToDisplay),
  #endif

  // Messages for the User, shared by the popup and the kill screen. They can't be autouploaded as we do not buffer content.
  //{.VP = VP_MSGSTR1, .memadr = nullptr, .size = VP_MSGSTR1_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},
  //{.VP = VP_MSGSTR2, .memadr = nullptr, .size = VP_MSGSTR2_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},
  //{.VP = VP_MSGSTR3, .memadr = nullptr, .size = VP_MSGSTR3_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},
  //{.VP = VP_MSGSTR4, .memadr = nullptr, .size = VP_MSGSTR4_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplayPGM},

  {.VP = VP_MSGSTR1, .memadr = nullptr, .size = VP_MSGSTR1_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplay_Language},
  {.VP = VP_MSGSTR2, .memadr = nullptr, .size = VP_MSGSTR2_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplay_Language},
  {.VP = VP_MSGSTR3, .memadr = nullptr, .size = VP_MSGSTR3_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplay_Language},
  {.VP = VP_MSGSTR4, .memadr = nullptr, .size = VP_MSGSTR4_LEN, .set_by_display_handler = nullptr, .send_to_display_handler = screen.sendStringToDisplay_Language},

  VPHELPER(0, 0, 0, 0) // must be last entry.
};

#endif // DGUS_LCD_UI_MKS
