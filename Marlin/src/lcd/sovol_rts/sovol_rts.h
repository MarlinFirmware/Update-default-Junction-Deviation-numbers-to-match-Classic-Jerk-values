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

#include <stdint.h>

/*********************************/
#define FHONE   (0x5A)
#define FHTWO   (0xA5)
#define FHLENG  (0x06)

#define MAX_NUM_FILES       20
#define FILENAME_LEN        20
#define RTS_UPDATE_INTERVAL 2000
#define RTS_UPDATE_VALUE    (2 * RTS_UPDATE_INTERVAL)

#define DATA_BUF_SIZE       26

/*************Register and Variable addr*****************/
#define RegAddr_W 0x80
#define RegAddr_R 0x81
#define VarAddr_W 0x82
#define VarAddr_R 0x83
#define ExchangePageBase  0x5A010000UL  // the first page ID. other page = first page ID + relevant num;
#define StartSoundSet     0x060480A0UL  // 06,start-music; 04, 4 musics; 80, the volume value; 04, return value about music number.
#define Beep              0x02AF0100UL
#define Beep1             0xFFFF0101UL
#define FONT_EEPROM 90

/*variable addr*/
#define ExchangepageAddr  0x0084
#define SoundAddr         0x00A0

#define START1_PROCESS_ICON_VP          0x1000
#define PRINT_SPEED_RATE_VP             0x1006
#define PRINT_PROCESS_ICON_VP           0x100E
#define PRINT_TIME_HOUR_VP              0x1010
#define PRINT_TIME_MIN_VP               0x1012
#define PRINT_PROCESS_VP                0x1016
#define HEAD0_FAN_ICON_VP               0x101E
#define CHANGE_FILAMENT0_TEMP_VP        0x1020
#define AUTO_BED_LEVEL_ZOFFSET_VP       0x1026

#define HEAD0_SET_TEMP_VP               0x1034
#define HEAD0_CURRENT_TEMP_VP           0x1036
#define BED_SET_TEMP_VP                 0x103A
#define BED_CURRENT_TEMP_VP             0x103C
#define AUTO_HOME_DISPLAY_ICON_VP       0x1042
#define AXIS_X_COORD_VP                 0x1048
#define AXIS_Y_COORD_VP                 0x104A
#define AXIS_Z_COORD_VP                 0x104C
#define HEAD0_FILAMENT_LOAD_DATA_VP     0x1054
#define PRINTER_MACHINE_TEXT_VP         0x1060
#define PRINTER_VERSION_TEXT_VP         0x106A
#define PRINTER_PRINTSIZE_TEXT_VP       0x1074
#define PRINTER_WEBSITE_TEXT_VP         0x107E
#define MARLIN_VERSION_TEXT_VP          0x1088
#define AUTO_BED_LEVEL_ICON_VP          0x108D
#define CHANGE_FILAMENT_ICON_VP         0x108E
#define TWO_EXTRUDER_HOTEND_XOFFSET_VP  0x1092
#define TWO_EXTRUDER_HOTEND_YOFFSET_VP  0x1094
#define TWO_EXTRUDER_HOTEND_ZOFFSET_VP  0x1096
#define AUTO_BED_LEVEL_1POINT_VP        0x1100

#define PRINT_SURPLUS_TIME_HOUR_VP      0x1162
#define PRINT_SURPLUS_TIME_MIN_VP       0x1164
#define SELECT_MODE_ICON_VP             0x1166
#define CHANGE_SDCARD_ICON_VP           0x1168

#define MOTOR_FREE_ICON_VP              0x1200
#define FILE1_SELECT_ICON_VP            0x1221
#define FILE2_SELECT_ICON_VP            0x1222
#define FILE3_SELECT_ICON_VP            0x1223
#define FILE4_SELECT_ICON_VP            0x1224
#define FILE5_SELECT_ICON_VP            0x1225
#define FILE6_SELECT_ICON_VP            0x1226
#define FILE7_SELECT_ICON_VP            0x1227
#define FILE8_SELECT_ICON_VP            0x1228
#define FILE9_SELECT_ICON_VP            0x1229
#define FILE10_SELECT_ICON_VP           0x122A
#define FILE11_SELECT_ICON_VP           0x122B
#define FILE12_SELECT_ICON_VP           0x122C
#define FILE13_SELECT_ICON_VP           0x122D
#define FILE14_SELECT_ICON_VP           0x122E
#define FILE15_SELECT_ICON_VP           0x122F
#define FILE16_SELECT_ICON_VP           0x1230
#define FILE17_SELECT_ICON_VP           0x1231
#define FILE18_SELECT_ICON_VP           0x1232
#define FILE19_SELECT_ICON_VP           0x1233
#define FILE20_SELECT_ICON_VP           0x1234

#define FILE1_TEXT_VP                   0x200A
#define FILE2_TEXT_VP                   0x201E
#define FILE3_TEXT_VP                   0x2032
#define FILE4_TEXT_VP                   0x2046
#define FILE5_TEXT_VP                   0x205A
#define FILE6_TEXT_VP                   0x206E
#define FILE7_TEXT_VP                   0x2082
#define FILE8_TEXT_VP                   0x2096
#define FILE9_TEXT_VP                   0x20AA
#define FILE10_TEXT_VP                  0x20BE
#define FILE11_TEXT_VP                  0x20D2
#define FILE12_TEXT_VP                  0x20E6
#define FILE13_TEXT_VP                  0x20FA
#define FILE14_TEXT_VP                  0x210E
#define FILE15_TEXT_VP                  0x2122
#define FILE16_TEXT_VP                  0x2136
#define FILE17_TEXT_VP                  0x214A
#define FILE18_TEXT_VP                  0x215E
#define FILE19_TEXT_VP                  0x2172
#define FILE20_TEXT_VP                  0x2186

#define SELECT_FILE_TEXT_VP             0x219A
#define TWO_COLOR_MODE_ICON_VP          0x21B8
#define COPY_MODE_ICON_VP               0x21B9
#define MIRROR_MODE_ICON_VP             0x21BA
#define SINGLE_MODE_ICON_VP             0x21BB
#define EXCHANGE_NOZZLE_ICON_VP         0x21BC
#define PRINT_MODE_ICON_VP              0x21BD
#define PRINT_FILE_TEXT_VP              0x21C0

#define Nozzle_P_VP                     0x2200
#define Nozzle_I_VP                     0x2204
#define Nozzle_D_VP                     0x2208
#define Hot_Bed_P_VP                    0x220C
#define Hot_Bed_I_VP                    0x2240
#define Hot_Bed_D_VP                    0x2244

#define Vmax_X_VP                       0x2210
#define Vmax_Y_VP                       0x2212
#define Vmax_Z_VP                       0x2214
#define Vmax_E_VP                       0x2216

#define Accel_VP                        0x2220
#define A_Retract_VP                    0x2222
#define A_Travel_VP                     0x2224
#define Amax_X_VP                       0x2226
#define Amax_Y_VP                       0x2228
#define Amax_Z_VP                       0x222A
#define Amax_E_VP                       0x222C

#define Jerk_X_VP                       0x2230
#define Jerk_Y_VP                       0x2232
#define Jerk_Z_VP                       0x2234
#define Jerk_E_VP                       0x2236

#define Steps_X_VP                      0x1130
#define Steps_Y_VP                      0x1132
#define Steps_Z_VP                      0x1134
#define Steps_E_VP                      0x1136

#define Advance_K_VP                    0x1138
#define Time_VP                         0x2450
#define Time1_VP                        0x2455
#define FAN_SPEED_VP                    0x2460
#define Wait_VP                         0x2480
#define Zoffset_UNIT_VP                 0x2500
#define Current_X_VP                    0x2468
#define Current_Y_VP                    0x246A
#define Current_Z_VP                    0x246C
#define Current_E_VP                    0x246F
#define Threshold_X_VP                  0x2471
#define Threshold_Y_VP                  0x2473
#define Threshold_Z_VP                  0x2475
#define Threshold_E_VP                  0x2477
#define Sensorless_X_VP                 0x2479
#define Sensorless_Y_VP                 0x247B

#define FilenameNature                  0x6003

enum SovolPage : uint8_t {
  ID_Startup        =   0,                                                       // Startup screen
  ID_Home_D         =   1,      ID_Home_L         =  55 + ID_Home_D,             // Home screen
  ID_Page1_D        =   2,      ID_Page1_L        =  55 + ID_Page1_D,            // File picker page 1
  ID_Page2_D        =   3,      ID_Page2_L        =  55 + ID_Page2_D,            // File picker page 2
  ID_Page3_D        =   4,      ID_Page3_L        =  55 + ID_Page3_D,            // File picker page 3
  ID_Page4_D        =   5,      ID_Page4_L        =  55 + ID_Page4_D,            // File picker page 4
  ID_ChangeWait_D   =   6,      ID_ChangeWait_L   =  55 + ID_ChangeWait_D,       // Wait for filament change to start
  ID_Cold_D         =   7,      ID_Cold_L         =  55 + ID_Cold_D,             // Nozzle heating, please wait
  ID_Change_D       =   8,      ID_Change_L       =  55 + ID_Change_D,           // Manually change filament dialog
  ID_Finish_D       =   9,      ID_Finish_L       =  55 + ID_Finish_D,           // Confirm print finish
  ID_PrintHeating_D =  10,      ID_PrintHeating_L =  55 + ID_PrintHeating_D,     // Wait for heating before print starts
  ID_PrintStatus_D  =  11,      ID_PrintStatus_L  =  55 + ID_PrintStatus_D,      // Printing status progress
  ID_PrintResume_D  =  12,      ID_PrintResume_L  =  55 + ID_PrintResume_D,      // Resume printing
  ID_PrintAdjust1_D =  14,      ID_PrintAdjust1_L =  55 + ID_PrintAdjust1_D,     // Modify print settings while printing, 0.01mm z offset
  ID_TempChange_D   =  15,      ID_TempChange_L   =  55 + ID_TempChange_D,       // Temp / fan change and preset menu
  ID_Unload_D       =  16,      ID_Unload_L       =  55 + ID_Unload_D,           // Wait for filament unload
  ID_Insert_D       =  17,      ID_Insert_L       =  55 + ID_Insert_D,           // Insert filament
  ID_Advanced_D     =  18,      ID_Advanced_L     =  55 + ID_Advanced_D,         // Advanced settings
  ID_NoFilament_D   =  20,      ID_NoFilament_L   =  55 + ID_NoFilament_D,       // No filament
  ID_Settings_D     =  21,      ID_Settings_L     =  55 + ID_Settings_D,         // Settings screen
  ID_Level1_D       =  22,      ID_Level1_L       =  55 + ID_Level1_D,           // Leveling screen 0.01mm
  ID_Load_D         =  23,      ID_Load_L         =  55 + ID_Load_D,             // Filament loading screen, feed / retreat filament
  ID_LoadCold_D     =  24,      ID_LoadCold_L     =  55 + ID_LoadCold_D,         // Cold nozzle warning on filament load screen
  ID_Velocity_D     =  25,      ID_Velocity_L     =  55 + ID_Velocity_D,         // Speed limit settings menu
  ID_LoadHeating_D  =  26,      ID_LoadHeating_L  =  55 + ID_LoadHeating_D,      // Nozzle heating in filament load screen
  ID_LoadCancel_D   =  27,      ID_LoadCancel_L   =  55 + ID_LoadCancel_D,       // Filament change menu, cancel feed / retreat
  ID_PrintAdjust5_D =  28,      ID_PrintAdjust5_L =  55 + ID_PrintAdjust5_D,     // Modify print settings while printing, 0.05mm z offset
  ID_Move10_D       =  29,      ID_Move10_L       =  55 + ID_Move10_D,           // Move axis, 10mm
  ID_Move1_D        =  30,      ID_Move1_L        =  55 + ID_Move1_D,            // Move axis, 1mm
  ID_Move01_D       =  31,      ID_Move01_L       =  55 + ID_Move01_D,           // Move axis, 0.1mm
  ID_AutoHome_D     =  32,      ID_AutoHome_L     =  55 + ID_AutoHome_D,         // Auto homing in progress
  ID_Info_D         =  33,      ID_Info_L         =  55 + ID_Info_D,             // Printer info screen
  ID_Accel_D        =  34,      ID_Accel_L        =  55 + ID_Accel_D,            // Accelaration settings menu
  ID_Jerk_D         =  35,      ID_Jerk_L         =  55 + ID_Jerk_D,             // Jerk settings menu
  ID_Resume_D       =  36,      ID_Resume_L       =  55 + ID_Resume_D,           // Resume print after power loss
  ID_Steps_D        =  37,      ID_Steps_L        =  55 + ID_Steps_D,            // Steps settings menu
  ID_ABL_Wait_D     =  38,      ID_ABL_Wait_L     =  55 + ID_ABL_Wait_D,         // Auto leveling, please wait
  ID_FilamentOut_D  =  39,      ID_FilamentOut_L  =  55 + ID_FilamentOut_D,      // Click yes to heat and change filament
  ID_Processing_D   =  40,      ID_Processing_L   =  55 + ID_Processing_D,       // Processing please wait
  ID_TempPID_D      =  41,      ID_TempPID_L      =  55 + ID_TempPID_D,          // PID settings menu
  ID_Purge_D        =  43,      ID_Purge_L        =  55 + ID_Purge_D,            // Wait for filament purge
  ID_PurgeMore_D    =  44,      ID_PurgeMore_L    =  56 + ID_PurgeMore_D,        // Purge more or resume screen
  ID_HeatNozzle_D   =  45,      ID_HeatNozzle_L   =  54 + ID_HeatNozzle_D,       // Heat nozzle after pause
  ID_MediaFail_D    =  46,      ID_MediaFail_L    =  55 + ID_MediaFail_D,        // Sd card removed
  ID_BrowseNoSd_D   =  47,      ID_BrowseNoSd_L   =  55 + ID_BrowseNoSd_D,       // No sd card on browse screen
  ID_AdvWarn_D      =  49,      ID_AdvWarn_L      =  55 + ID_AdvWarn_D,          // Warning when entering advanced settings
  ID_KillRunaway_D  =  52,      ID_KillRunaway_L  =  55 + ID_KillRunaway_D,      // Thermal runaway
  ID_KillHeat_D     =  53,      ID_KillHeat_L     =  55 + ID_KillHeat_D,         // Thermistor error
  ID_KillBadTemp_D  =  54,      ID_KillBadTemp_L  =  55 + ID_KillBadTemp_D,      // Heating failed
  ID_KillHome_D     =  55,      ID_KillHome_L     =  55 + ID_KillHome_D,         // Auto-home failed
  ID_Level5_D       =  111,     ID_Level5_L       =   6 + ID_Level5_D,           // Leveling screen 0.05mm
  ID_DriverError_D  =  112,     ID_DriverError_L  =   6 + ID_DriverError_D,      // Driver error
  ID_SettingsTMC_D  =  113,     ID_SettingsTMC_L  =   6 + ID_SettingsTMC_D,      // TMC Driver settings
  ID_DriverA_D      =  114,     ID_DriverA_L      =   6 + ID_DriverA_D,          // TMC Driver current settings
  ID_DriverTrsh_D   =  115,     ID_DriverTrsh_L   =   6 + ID_DriverTrsh_D,       // TMC Driver hybrid treshold settings
  ID_DriverSens_D   =  116,     ID_DriverSens_L   =   6 + ID_DriverSens_D,       // TMC Driver sensorless homing settings
  ID_ABL_HeatWait_D =  123,     ID_ABL_HeatWait_L =   1 + ID_ABL_HeatWait_D,     // Wait for nozzle & bed to heat up
};

/************struct**************/

typedef struct DataBuf {
  uint8_t len;
  uint8_t head[2];
  uint8_t command;
  uint32_t addr;
  uint32_t bytelen;
  uint16_t data[32];
  uint8_t reserv[4];
} DB;

typedef struct CardRecord {
  int16_t recordcount;
  int16_t Filesum;
  uint32_t addr[MAX_NUM_FILES];
  char display_filename[MAX_NUM_FILES][FILENAME_LEN];
  char filename[MAX_NUM_FILES][FILENAME_LEN];
} CRec;

class RTS {
  public:
    RTS();
    static int16_t receiveData();
    static void sdCardInit();
    static bool sdDetected();
    static void sdCardUpdate();
    static void sendData();
    static void sendData(const String&,  const uint32_t, const uint8_t=VarAddr_W);
    static void sendData(const char[],   const uint32_t, const uint8_t=VarAddr_W);
    static void sendData(const char,     const uint32_t, const uint8_t=VarAddr_W);
    static void sendData(const int16_t,  const uint32_t, const uint8_t=VarAddr_W);
    static void sendData(const uint32_t, const uint32_t, const uint8_t=VarAddr_W);

    static void sendData(const uint8_t str[], const uint32_t addr, const uint8_t cmd=VarAddr_W) { sendData((char *)str, addr, cmd); }
    static void sendData(const uint16_t n,    const uint32_t addr, const uint8_t cmd=VarAddr_W) { sendData(int16_t(n), addr, cmd); }
    static void sendData(const_float_t n,     const uint32_t addr, const uint8_t cmd=VarAddr_W) { sendData(int16_t(n), addr, cmd); } //was originally int16 ?
    static void sendData(const int32_t n,     const uint32_t addr, const uint8_t cmd=VarAddr_W) { sendData(uint32_t(n), addr, cmd); }
    static void sendData(const int n,         const uint32_t addr, const uint8_t cmd=VarAddr_W) { sendData(int16_t(n), addr, cmd); }

    static void sdCardStop();
    static void handleData();
    static void init();

    static uint8_t print_state;
    static bool start_print_flag;

    static bool dark_mode;
    static void gotoPage(SovolPage page) { sendData(ExchangePageBase + page, ExchangepageAddr); }
    static void gotoPage(SovolPage p1, SovolPage p2) { gotoPage(dark_mode ? p2 : p1); }
    static void gotoPageBeep(SovolPage p1, SovolPage p2) { gotoPage(p1, p2); sendData(Beep1, SoundAddr); }

    static void sendPrinterInfo();
    static void updateTempE0();
    static void updateTempBed();
    static void updateFan0();

    static void onIdle();

    static void refreshTime() { sendData(1, dark_mode ? Time_VP : Time1_VP); gotoPage(ID_PrintStatus_L, ID_PrintStatus_D); }

    static DB recdat;
    static DB snddat;
  private:
    static uint8_t databuf[DATA_BUF_SIZE];
};

extern RTS rts;

enum PROC_COM {
  MainPageKey = 0,
  AdjustmentKey,
  PrintSpeedKey,
  StopPrintKey,
  PausePrintKey,
  ResumePrintKey,
  ZOffsetKey,
  TempScreenKey,
  CoolScreenKey,
  Heater0TempEnterKey, Heater1TempEnterKey,
  HotBedTempEnterKey,
  SettingScreenKey,
  SettingBackKey,
  BedLevelFunKey,
  AxisPageSelectKey,
  XaxismoveKey, YaxismoveKey, ZaxismoveKey,
  SelectExtruderKey,
  Heater0LoadEnterKey,
  FilamentLoadKey,
  Heater1LoadEnterKey,
  SelectLanguageKey,
  FilamentCheckKey,
  PowerContinuePrintKey,
  PrintSelectModeKey,
  XhotendOffsetKey, YhotendOffsetKey, ZhotendOffsetKey,
  StoreMemoryKey,
  PrintFileKey,
  SelectFileKey,
  AdvancedKey,
  Nozzle_P, Nozzle_I, Nozzle_D,
  Hot_Bed_P, Hot_Bed_I, Hot_Bed_D,
  Vmax_X, Vmax_Y, Vmax_Z, Vmax_E,
  Accel, A_Retract, A_Travel,
  Amax_X, Amax_Y, Amax_Z, Amax_E,
  Jerk_X, Jerk_Y, Jerk_Z, Jerk_E,
  Steps_X, Steps_Y, Steps_Z, Steps_E,
  Advance_K,
  AdvancedBackKey,
  FilamentChange,
  FanSpeedKey,
  ZoffsetUnitKey,
  TMCDriver,
  Current_X, Current_Y, Current_Z, Current_E,
  Threshold_X, Threshold_Y, Threshold_Z, Threshold_E,
  Sensorless_X, Sensorless_Y,
  ChangePageKey
};

const uint32_t Addrbuf[] = {
  0x1002, 0x1004, 0x1006, 0x1008, 0x100A, 0x100C, 0x1026, 0x1030, 0x1032, 0x1034,
  0x1038, 0x103A, 0x103E, 0x1040, 0x1044, 0x1046, 0x1048, 0x104A, 0x104C, 0x104E,
  0x1054, 0x1056, 0x1058, 0x105C, 0x105E, 0x105F, 0x1090, 0x1092, 0x1094, 0x1096,
  0x1098, 0x2198, 0x2199, 0x21E0, 0x2200, 0x2204, 0x2208, 0x220C, 0x2240, 0x2244,
  0x2210, 0x2212, 0x2214, 0x2216, 0x2220, 0x2222, 0x2224, 0x2226, 0x2228, 0x222A,
  0x222C, 0x2230, 0x2232, 0x2234, 0x2236, 0x1130, 0x1132, 0x1134, 0x1136, 0x1138,
  0x2250, 0x2300, 0x2460, 0x2464, 0x2466, 0x2468, 0x246A, 0x246C, 0x246F, 0x2471,
  0x2473, 0x2475, 0x2477, 0x2479, 0x247B, 0x110E, 0
};

extern void RTS_Update();
extern void RTS_Init();

extern int16_t update_time_value;
extern bool poweroff_continue;
extern bool sdcard_pause_check;
extern bool sd_printing_autopause;
extern bool pause_flag;

void RTS_AutoBedLevelPage();
void RTS_MoveAxisHoming();
void RTS_PauseMoveAxisPage();
