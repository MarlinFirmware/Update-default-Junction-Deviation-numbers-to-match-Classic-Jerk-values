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

// Picture ID
#define Language_English    1
#define Language_Chinese    2

#define ICON 7 // 7.ICO

#ifndef CORP_WEBSITE
  #define CORP_WEBSITE WEBSITE_URL
#endif

#define ICON_LOGO                0
#define ICON_Print_0             1
#define ICON_Print_1             2
#define ICON_Prepare_0           3
#define ICON_Prepare_1           4
#define ICON_Control_0           5
#define ICON_Control_1           6
#define ICON_Leveling_0          7
#define ICON_Leveling_1          8
#define ICON_HotendTemp          9
#define ICON_BedTemp            10
#define ICON_Speed              11
#define ICON_Zoffset            12
#define ICON_Back               13
#define ICON_File               14
#define ICON_PrintTime          15
#define ICON_RemainTime         16
#define ICON_Setup_0            17
#define ICON_Setup_1            18
#define ICON_Pause_0            19
#define ICON_Pause_1            20
#define ICON_Continue_0         21
#define ICON_Continue_1         22
#define ICON_Stop_0             23
#define ICON_Stop_1             24
#define ICON_Bar                25
#define ICON_More               26

#define ICON_Axis               27
#define ICON_CloseMotor         28
#define ICON_Homing             29
#define ICON_SetHome            30
#define ICON_PLAPreheat         31
#define ICON_ABSPreheat         32
#define ICON_Cool               33
#define ICON_Language           34

#define ICON_MoveX              35
#define ICON_MoveY              36
#define ICON_MoveZ              37
#define ICON_Extruder           38

#define ICON_Temperature        40
#define ICON_Motion             41
#define ICON_WriteEEPROM        42
#define ICON_ReadEEPROM         43
#define ICON_ResetEEPROM        44
#define ICON_Info               45

#define ICON_SetEndTemp         46
#define ICON_SetBedTemp         47
#define ICON_FanSpeed           48
#define ICON_SetPLAPreheat      49
#define ICON_SetABSPreheat      50

#define ICON_MaxSpeed           51
#define ICON_MaxAccelerated     52
#define ICON_MaxJerk            53
#define ICON_Step               54
#define ICON_PrintSize          55
#define ICON_Version            56
#define ICON_Contact            57
#define ICON_StockConfiguration 58
#define ICON_MaxSpeedX          59
#define ICON_MaxSpeedY          60
#define ICON_MaxSpeedZ          61
#define ICON_MaxSpeedE          62
#define ICON_MaxAccX            63
#define ICON_MaxAccY            64
#define ICON_MaxAccZ            65
#define ICON_MaxAccE            66
#define ICON_MaxSpeedJerkX      67
#define ICON_MaxSpeedJerkY      68
#define ICON_MaxSpeedJerkZ      69
#define ICON_MaxSpeedJerkE      70
#define ICON_StepX              71
#define ICON_StepY              72
#define ICON_StepZ              73
#define ICON_StepE              74
#define ICON_Setspeed           75
#define ICON_SetZOffset         76
#define ICON_Rectangle          77
#define ICON_BLTouch            78
#define ICON_TempTooLow         79
#define ICON_AutoLeveling       80
#define ICON_TempTooHigh        81
#define ICON_NoTips_C           82
#define ICON_NoTips_E           83
#define ICON_Continue_C         84
#define ICON_Continue_E         85
#define ICON_Cancel_C           86
#define ICON_Cancel_E           87
#define ICON_Confirm_C          88
#define ICON_Confirm_E          89
#define ICON_Info_0             90
#define ICON_Info_1             91

// Extra Icons
#define ICON_Printer_0         93
#define ICON_Box              200
#define ICON_Checkbox         201
#define ICON_Fade             202
#define ICON_Mesh             203
#define ICON_Tilt             204
#define ICON_Brightness       205
#define ICON_Probe            206
#define ICON_AxisD            249
#define ICON_AxisBR           250
#define ICON_AxisTR           251
#define ICON_AxisBL           252
#define ICON_AxisTL           253
#define ICON_AxisC            254

#define ICON_Folder       ICON_More
#define ICON_AdvSet       ICON_Language
#define ICON_HomeOffset   ICON_PrintSize
#define ICON_HomeOffsetX  ICON_StepX
#define ICON_HomeOffsetY  ICON_StepY
#define ICON_HomeOffsetZ  ICON_StepZ
#define ICON_ProbeOffset  ICON_AdvSet
#define ICON_ProbeOffsetX ICON_StepX
#define ICON_ProbeOffsetY ICON_StepY
#define ICON_ProbeOffsetZ ICON_StepZ
#define ICON_PIDNozzle    ICON_HotendTemp
#define ICON_PIDBed       ICON_BedTemp
#define ICON_FWRetract    ICON_StepE
#define ICON_FWRetLength  ICON_StepE
#define ICON_FWRetSpeed   ICON_Setspeed
#define ICON_FWRetZRaise  ICON_MoveZ
#define ICON_FWRecSpeed   ICON_Setspeed
#define ICON_FWRecExtra   ICON_StepE
