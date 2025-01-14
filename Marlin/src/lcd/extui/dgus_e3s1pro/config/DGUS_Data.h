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

#include <inttypes.h>

#include "../../../../inc/MarlinConfigPre.h"

#ifndef LCD_LANGUAGE
  #define LCD_LANGUAGE en
  #warning "LCD_LANGUAGE not defined, defaulting to English."
#endif

namespace DGUS_Data {

  // RX constants

  enum class Language : uint16_t {
    Chinese_Simplified = 1,
    English,
    German,
    Spanish,
    French,
    Italian,
    Portuguese,
    Russian,
    Turkish,

    // Compatibility with LCD_LANGUAGE
    zh_CN = Chinese_Simplified,
    en = English,
    de = German,
    es = Spanish,
    fr = French,
    fr_na = French,
    it = Italian,
    pt = Portuguese,
    ru = Russian,
    tr = Turkish,

    Default = LCD_LANGUAGE
  };

  // 1002
  enum class MenuSelectCommand : uint16_t {
    Print = 1,
    Ready = 2,
    Settings = 3,
    PrintFinished = 5,
    StartAutoLevel = 6,
    Main = 8,
    PausePrint = 9,
    PauseStopPrint = 10
  };

  // 1004
  enum class AdjustCommand : uint16_t {
    Show_Adjust = 1,
    Show_PrintStatus = 2,
    Show_Adjust_ZOffset = 5,
    Validate_ZOffset = 7,
  };

  // 1008
  enum class CheckKOCommand : uint16_t {
    KO = 2,
    SDCard_No = 4, // screen fw 1.0.3
  };

  // 100A
  enum class StopPauseCommand : uint16_t {
    Pause_Laser = 1,
    Pause = 2,
    Stop_Confirm = 3
  };

  // 100C
  enum class CheckOKCommand : uint16_t {
    ContinueStop_Continue = 1,
    FilamentRunout_Yes = 2,
    FilamentLoad_Yes = 3,
    SDCardCheck_Yes = 4,
  };

  // 1030
  enum class PresetControlCommand : uint16_t {
    Show_Settings_PLA_Settings = 3,
    Show_Settings_ABS_Settings = 4,
    Apply_PLA_Settings = 5,
    Apply_ABS_Settings = 6,
    Show_Ready_Manual = 8,
  };

  // 1032
  enum class TemperatureControlCommand : uint16_t {
    Cooldown = 1,
    Exit_Temperature_Preset_And_Discard_Temperature = 2
  };

  // 103E
  enum class SettingsMenuCommand : uint16_t {
    Show_Settings_Device_Language = 1,
    Show_Settings_Advanced = 2,
    Show_Ready_Jog = 3,
    Show_Settings_About = 5,
    DisableStepperMotors = 6,
    Reset_All_Settings = 7,
    Show_Settings_Device_and_Save_Temperatures = 8,
    FactoryReset_Validate = 11,
    FactoryReset_Cancel = 14,
    Exit_Settings_Tramming = 15,
    Exit_Settings_Leveling = 16,
    Show_Settings_Device_and_Discard_Temperatures = 17
  };

  // 1044
  enum class LevelingCommand : uint16_t {
    Show_Settings_Leveling = 1,
    Show_AuxLeveling = 4,
    Goto_Center = 5,
    Goto_LF = 6,
    Goto_RF = 7,
    Goto_RB = 8,
    Goto_LB = 9,
    Show_AutoLeveling = 10,
  };

  // 1046
  enum class AxisControlCommand : uint16_t {
    Jog_10mm = 1,
    Jog_1mm = 2,
    Jog_0_1mm = 3,
    Home_XY = 4,
    Home_Z = 5,
  };

  // 1056
  enum class FilamentIoCommand : uint16_t {
    FilamentCheck_Yes = 1,
    FilamentCheck_No = 2,
    Show_Ready_IO = 3
  };

  // 105F
  enum class PowerLossCommand : uint16_t {
    PowerLoss_Continue = 1,
    PowerLoss_No = 2
  };

  // 1098
  enum class AdvancedSettingsCommand : uint16_t {
    Show_AdvSettings_Movement = 4,
    Show_AdvSettings_PID = 5,
    Show_AdvSettings_Movement_StepsPerMm = 7,
    Show_AdvSettings_Movement_Acceleration = 8,
    Show_AdvSettings_Movement_Jerk = 9,
    Show_AdvSettings_Movement_MaxFeedrate = 10,
    Exit_AdvSettings_Movement_Submenu = 12,
    Show_AdvSettings = 13,
  };

  // 2198
  enum class FilelistControlCommand : uint16_t {
    Start_Print = 1,
    F1_Down  = 2,
    F1_Up = 3,
    F2_Down  = 4,
    F2_Up = F1_Up,
    F3_Down  = 6,
    F3_Up = 5,
    F4_Down  = 8,
    F4_Up = 7,
    Begin = 9,
    End = 10
  };

  // 2198
  enum class LaserControlCommand : uint16_t {
    Mode_FDM = 1,
    Mode_Cutter = 2,
    Mode_FDM_Confirm = 3,
    Mode_FDM_Cancel = 4,
    Mode_Cutter_Confirm = 5,
    Mode_Cutter_Cancel = 6,
    Mode_Change = 0xB
  };
};
