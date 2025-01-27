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
#include <stddef.h>
#include "../../../core/types.h"

//#define TJC_DISPLAY         // Enable for TJC display
//#define DACAI_DISPLAY       // Enable for DACAI display
//#define TITLE_CENTERED      // Center Menu Title Text

#if HAS_MESH
  #define PROUI_MESH_EDIT     // Add a menu to edit mesh inset
  #if ENABLED(PROUI_MESH_EDIT)
    #define Z_OFFSET_MIN -3.0 // (mm)
    #define Z_OFFSET_MAX  3.0 // (mm)
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
#else
  #define DEF_Z_AFTER_HOMING 0
#endif

#ifdef PREHEAT_1_TEMP_HOTEND
  #define DEF_HOTENDPIDT PREHEAT_1_TEMP_HOTEND
#else
  #define DEF_HOTENDPIDT 195
#endif
#ifdef PREHEAT_1_TEMP_BED
  #define DEF_BEDPIDT PREHEAT_1_TEMP_BED
#else
  #define DEF_BEDPIDT 60
#endif
#ifdef PREHEAT_1_TEMP_CHAMBER
  #define DEF_CHAMBERPIDT PREHEAT_1_TEMP_CHAMBER
#else
  #define DEF_CHAMBERPIDT 0
#endif
#define DEF_PIDCYCLES 5

/**
 * ProUI internal feature flags
 */
#if HAS_MESH
  #define PROUI_MESH_EDIT     // Add a menu to edit mesh inset + points
  #if ENABLED(PROUI_MESH_EDIT)
    #define Z_OFFSET_MIN  -3.0  // (mm)
    #define Z_OFFSET_MAX   3.0  // (mm)
  #endif
#endif
#if HAS_BED_PROBE
  #define PROUI_ITEM_ZFRS     // Add a menu item to change Z_PROBE_FEEDRATE_SLOW - probe speed
#endif
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
#if DISABLED(DISABLE_TUNING_GRAPH)
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

#if ENABLED(PROUI_MESH_EDIT)
  #ifndef   MESH_INSET
    #define MESH_INSET 10
  #endif
  #ifndef   MESH_MIN_X
    #define MESH_MIN_X MESH_INSET
  #endif
  #ifndef   MESH_MIN_Y
    #define MESH_MIN_Y MESH_INSET
  #endif
  #ifndef   MESH_MAX_X
    #define MESH_MAX_X  X_BED_SIZE - (MESH_INSET)
  #endif
  #ifndef   MESH_MAX_Y
    #define MESH_MAX_Y  Y_BED_SIZE - (MESH_INSET)
  #endif
  constexpr uint16_t DEF_MESH_MIN_X = MESH_MIN_X;
  constexpr uint16_t DEF_MESH_MAX_X = MESH_MAX_X;
  constexpr uint16_t DEF_MESH_MIN_Y = MESH_MIN_Y;
  constexpr uint16_t DEF_MESH_MAX_Y = MESH_MAX_Y;
  #define MIN_MESH_INSET 0
  #define MAX_MESH_INSET X_BED_SIZE
#endif

#ifndef MULTIPLE_PROBING
  #define MULTIPLE_PROBING 0
#endif

#if HAS_BED_PROBE
  constexpr uint16_t DEF_Z_PROBE_FEEDRATE_SLOW = Z_PROBE_FEEDRATE_SLOW;
#endif

#if HAS_EXTRUDERS
  constexpr bool DEF_INVERT_E0_DIR = INVERT_E0_DIR;
#endif

typedef struct {
  // Color settings
  uint16_t colorBackground;
  uint16_t colorCursor;
  uint16_t colorTitleBg;
  uint16_t colorTitleTxt;
  uint16_t colorText;
  uint16_t colorSelected;
  uint16_t colorSplitLine;
  uint16_t colorHighlight;
  uint16_t colorStatusBg;
  uint16_t colorStatusTxt;
  uint16_t colorPopupBg;
  uint16_t colorPopupTxt;
  uint16_t colorAlertBg;
  uint16_t colorAlertTxt;
  uint16_t colorPercentTxt;
  uint16_t colorBarfill;
  uint16_t colorIndicator;
  uint16_t colorCoordinate;

  // Temperatures
  int16_t pidCycles = DEF_PIDCYCLES;
  #if ENABLED(PIDTEMP)
    celsius_t hotendPIDT = DEF_HOTENDPIDT;
  #endif
  #if ENABLED(PIDTEMPBED)
    celsius_t bedPIDT = DEF_BEDPIDT;
  #endif
  #if ENABLED(PIDTEMPCHAMBER)
    celsius_t chamberPIDT = DEF_CHAMBERPIDT;
  #endif
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    int16_t extMinT = EXTRUDE_MINTEMP;
  #endif
  #if ENABLED(PREHEAT_BEFORE_LEVELING)
    int16_t bedLevT = LEVELING_BED_TEMP;
  #endif

  // Various Options
  #if ENABLED(BAUD_RATE_GCODE)
    bool baud115K = false;
  #endif
  #if ALL(LCD_BED_TRAMMING, HAS_BED_PROBE)
    bool fullManualTramming = false;
  #endif
  #if ENABLED(PROUI_MEDIASORT)
    bool mediaSort = true;
  #endif
  bool mediaAutoMount = ENABLED(HAS_SD_EXTENDER);
  #if ALL(INDIVIDUAL_AXIS_HOMING_SUBMENU, MESH_BED_LEVELING)
    uint8_t zAfterHoming = DEF_Z_AFTER_HOMING;
    #define Z_POST_CLEARANCE hmiData.zAfterHoming
  #endif
  #if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
    LEDColor ledColor = defColorLeds;
  #endif
  #if HAS_GCODE_PREVIEW
    bool enablePreview = true;
  #endif
  #if ENABLED(PROUI_MESH_EDIT)
    float mesh_min_x = DEF_MESH_MIN_X;
    float mesh_max_x = DEF_MESH_MAX_X;
    float mesh_min_y = DEF_MESH_MIN_Y;
    float mesh_max_y = DEF_MESH_MAX_Y;
  #endif
  #if HAS_BED_PROBE
    IF_DISABLED(BD_SENSOR, uint8_t multiple_probing = MULTIPLE_PROBING);
    uint16_t zprobeFeed = DEF_Z_PROBE_FEEDRATE_SLOW;
  #endif
  #if HAS_EXTRUDERS
    bool Invert_E0 = DEF_INVERT_E0_DIR;
  #endif
} hmi_data_t;

extern hmi_data_t hmiData;

#define EXTUI_EEPROM_DATA_SIZE sizeof(hmi_data_t)

// ProUI extra feature redefines
#if ENABLED(PROUI_MESH_EDIT)
  #undef  MESH_MIN_X
  #undef  MESH_MAX_X
  #undef  MESH_MIN_Y
  #undef  MESH_MAX_Y
  #define MESH_MIN_X hmiData.mesh_min_x
  #define MESH_MAX_X hmiData.mesh_max_x
  #define MESH_MIN_Y hmiData.mesh_min_y
  #define MESH_MAX_Y hmiData.mesh_max_y
#endif

#if HAS_BED_PROBE
  #undef Z_PROBE_FEEDRATE_SLOW
  #define Z_PROBE_FEEDRATE_SLOW hmiData.zprobeFeed
#endif

#if HAS_EXTRUDERS
  #undef INVERT_E0_DIR
  #define INVERT_E0_DIR hmiData.Invert_E0
#endif
