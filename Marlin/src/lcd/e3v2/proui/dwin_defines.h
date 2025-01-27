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

/**
 * DWIN general defines and data structs for PRO UI
 * Based on the original work of: Miguel Risco-Castillo (MRISCOC)
 * https://github.com/mriscoc/Ender3V2S1
 * Version: 3.12.2
 * Date: 2022/08/08
 */

#include "../../../inc/MarlinConfigPre.h"

//#define TJC_DISPLAY           // Enable for TJC display
//#define DACAI_DISPLAY         // Enable for DACAI display
//#define TITLE_CENTERED        // Center Menu Title Text

#if HAS_MESH
  #define PROUI_MESH_EDIT       // Add a menu to edit mesh points
  #if ENABLED(PROUI_MESH_EDIT)
    #define Z_OFFSET_MIN  -3.0  // (mm)
    #define Z_OFFSET_MAX   3.0  // (mm)
  #endif
#endif

#if defined(__STM32F1__) || defined(STM32F1)
  #define DASH_REDRAW 1
#endif

#if DISABLED(PROBE_MANUALLY) && ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT, AUTO_BED_LEVELING_UBL)
  #define HAS_ONESTEP_LEVELING 1
#endif

#if !HAS_BED_PROBE && ENABLED(BABYSTEPPING)
  #define JUST_BABYSTEP 1
#endif

#if ANY(BABYSTEPPING, HAS_BED_PROBE, HAS_WORKSPACE_OFFSET)
  #define HAS_ZOFFSET_ITEM 1
#endif

#define defColorBackground  RGB(1, 12, 8)
#define defColorCursor      RGB(20, 49, 31)
#define defColorTitleBg     RGB(0, 23, 16)
#define defColorTitleTxt    COLOR_WHITE
#define defColorText        COLOR_WHITE
#define defColorSelected    COLOR_SELECT
#define defColorSplitLine   RGB(0, 23, 16)
#define defColorHighlight   COLOR_WHITE
#define defColorStatusBg    RGB(0, 23, 16)
#define defColorStatusTxt   COLOR_YELLOW
#define defColorPopupBg     COLOR_BG_WINDOW
#define defColorPopupTxt    COLOR_POPUP_TEXT
#define defColorAlertBg     COLOR_BG_RED
#define defColorAlertTxt    COLOR_YELLOW
#define defColorPercentTxt  COLOR_PERCENT
#define defColorBarfill     COLOR_BARFILL
#define defColorIndicator   COLOR_WHITE
#define defColorCoordinate  COLOR_WHITE
#define defColorButton      RGB(0, 23, 16)
#if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
  #define defColorLeds      LEDColorWhite()
#endif
#if CASELIGHT_USES_BRIGHTNESS
  #define defCaseLightBrightness 255
#endif

#ifdef Z_AFTER_HOMING
  #define DEF_Z_AFTER_HOMING Z_AFTER_HOMING
#elif ALL(INDIVIDUAL_AXIS_HOMING_SUBMENU, MESH_BED_LEVELING)
  #define DEF_Z_AFTER_HOMING 10
#endif

#ifndef PREHEAT_1_TEMP_HOTEND
  #define PREHEAT_1_TEMP_HOTEND 195
#endif
#ifndef PREHEAT_1_TEMP_BED
  #define PREHEAT_1_TEMP_BED 60
#endif
#ifndef PREHEAT_1_TEMP_CHAMBER
  #define PREHEAT_1_TEMP_CHAMBER 35
#endif
#define DEF_HOTENDPIDT PREHEAT_1_TEMP_HOTEND
#define DEF_BEDPIDT PREHEAT_1_TEMP_BED
#define DEF_CHAMBERPIDT PREHEAT_1_TEMP_CHAMBER
#define DEF_PIDCYCLES 5

/**
 * ProUI internal feature flags
 */
#if ALL(SDCARD_SORT_ALPHA, SDSORT_GCODE)
  #define PROUI_MEDIASORT     // Enable option to sort G-code files
#endif
#if ENABLED(POWER_LOSS_RECOVERY)
  #define PROUI_ITEM_PLR      // Tune > Power-loss Recovery
#endif
#if ENABLED(HAS_JUNCTION_DEVIATION)
  #define PROUI_ITEM_JD       // Tune > Junction Deviation
#endif
#if ENABLED(LIN_ADVANCE)
  #define PROUI_ITEM_ADVK     // Tune > Linear Advance
#endif
#if ANY(HAS_PID_HEATING, MPC_AUTOTUNE) && DISABLED(DISABLE_TUNING_GRAPH)
  #define PROUI_TUNING_GRAPH 1
#endif
#if PROUI_TUNING_GRAPH
  #define PROUI_ITEM_PLOT     // Plot temp graph viewer
#endif
#define HAS_GCODE_PREVIEW 1   // Preview G-code model thumbnail
#define HAS_CUSTOM_COLORS 1   // Change display colors
#define HAS_ESDIAG 1          // View End-stop/Runout switch continuity
#define HAS_LOCKSCREEN 1      // Simple lockscreen
#define HAS_SD_EXTENDER 1     // Enable to support SD card extender cables
