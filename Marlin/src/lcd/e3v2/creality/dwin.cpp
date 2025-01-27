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

/**
 * DWIN by Creality3D
 */

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(DWIN_CREALITY_LCD)

#include "dwin.h"

//#define USE_STRING_HEADINGS
//#define USE_STRING_TITLES

#if DISABLED(PROBE_MANUALLY) && ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT)
  #define HAS_ONESTEP_LEVELING 1
#endif

#if ANY(BABYSTEPPING, HAS_BED_PROBE, HAS_WORKSPACE_OFFSET)
  #define HAS_ZOFFSET_ITEM 1
#endif

#if !HAS_BED_PROBE && ENABLED(BABYSTEPPING)
  #define JUST_BABYSTEP 1
#endif

#include "../../utf8.h"
#include "../../marlinui.h"

#include "../../../sd/cardreader.h"

#include "../../../MarlinCore.h"
#include "../../../gcode/queue.h"

#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../module/settings.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if HAS_ONESTEP_LEVELING
  #include "../../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if ANY(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
  #include "../../../feature/babystep.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#include <WString.h>
#include <stdio.h>
#include <string.h>

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE STRINGIFY(X_BED_SIZE) "x" STRINGIFY(Y_BED_SIZE) "x" STRINGIFY(Z_MAX_POS)
#endif

#define PAUSE_HEAT

#define MENU_CHAR_LIMIT  24
#define STATUS_Y        354

#define FEEDRATE_E      (60)

// Minimum unit (0.1) : multiple (10)
#define UNITFDIGITS 1
#define MINUNITMULT pow(10, UNITFDIGITS)

#define DWIN_VAR_UPDATE_INTERVAL         1024
#define DWIN_SCROLL_UPDATE_INTERVAL      SEC_TO_MS(2)
#define DWIN_REMAIN_TIME_UPDATE_INTERVAL SEC_TO_MS(20)

#define TROWS 6                         // Total rows
constexpr uint16_t MROWS = TROWS - 1,   // Last Row Index
                   TITLE_HEIGHT = 30,   // Title bar height
                   MLINE = 53,          // Menu line height
                   LBLX = 60,           // Menu item label X
                   MENU_CHR_W = 8, STAT_CHR_W = 10;

#define MBASE(L) (49 + MLINE * (L))
#define EBASE(L) (MBASE(L) - 2 * DISABLED(USE_STRING_TITLES))

#define BABY_Z_VAR TERN(HAS_BED_PROBE, probe.offset.z, dwin_zoffset)

#define DWIN_BOTTOM (DWIN_HEIGHT-1)
#define DWIN_RIGHT (DWIN_WIDTH-1)

// Value Init
hmi_value_t hmiValues;
hmi_flag_t hmiFlag{0};

millis_t dwin_heat_time = 0;

uint8_t checkkey = 0;

enum SelectItem : uint8_t {
  PAGE_PRINT = 0,
  PAGE_PREPARE,
  PAGE_CONTROL,
  PAGE_INFO_LEVELING,

  PRINT_SETUP = 0,
  PRINT_PAUSE_RESUME,
  PRINT_STOP
};

typedef struct {
  uint8_t now, last;
  void set(uint8_t v) { now = last = v; }
  void reset() { set(0); }
  bool changed() { bool c = (now != last); if (c) last = now; return c; }
  bool dec() { if (now) now--; return changed(); }
  bool inc(uint8_t v) { if (now < (v - 1)) now++; else now = (v - 1); return changed(); }
} select_t;

select_t select_page{0}, select_file{0}, select_print{0}, select_prepare{0}
       , select_control{0}, select_axis{0}, select_temp{0}, select_motion{0}, select_tune{0}
       , select_advset{0}, select_speed{0}, select_acc{0}, select_jerk{0}, select_step{0}, select_item{0};

#if HAS_PREHEAT
  select_t select_PLA{0};
  #if PREHEAT_COUNT > 1
    select_t select_ABS{0};
  #endif
#endif

uint8_t index_file     = MROWS,
        index_prepare  = MROWS,
        index_control  = MROWS,
        index_leveling = MROWS,
        index_tune     = MROWS,
        index_advset   = MROWS;

bool dwin_abort_flag = false; // Flag to reset feedrate, return to Home

static uint8_t _card_percent = 0;
static uint16_t _remain_time = 0;

#if ENABLED(PAUSE_HEAT)
  #if HAS_HOTEND
    uint16_t resume_hotend_temp = 0;
  #endif
  #if HAS_HEATED_BED
    uint16_t resume_bed_temp = 0;
  #endif
#endif

#if HAS_ZOFFSET_ITEM
  float dwin_zoffset = 0, last_zoffset = 0;
#endif

#define DWIN_LANGUAGE_EEPROM_ADDRESS 0x01   // Between 0x01 and 0x63 (EEPROM_OFFSET-1)
                                            // BL24CXX::check() uses 0x00

inline bool hmiIsChinese() { return hmiFlag.language == DWIN_CHINESE; }

void hmiSetLanguageCache() {
  dwinJPGCacheTo1(hmiIsChinese() ? Language_Chinese : Language_English);
}

void hmiSetLanguage() {
  #if ALL(EEPROM_SETTINGS, IIC_BL24CXX_EEPROM)
    BL24CXX::read(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&hmiFlag.language, sizeof(hmiFlag.language));
  #endif
  hmiSetLanguageCache();
}

void hmiToggleLanguage() {
  hmiFlag.language = hmiIsChinese() ? DWIN_ENGLISH : DWIN_CHINESE;
  hmiSetLanguageCache();
  #if ALL(EEPROM_SETTINGS, IIC_BL24CXX_EEPROM)
    BL24CXX::write(DWIN_LANGUAGE_EEPROM_ADDRESS, (uint8_t*)&hmiFlag.language, sizeof(hmiFlag.language));
  #endif
}

typedef struct { uint16_t x, y, w, h; } icon_info_t;
typedef struct { uint16_t x, y[2], w, h; } text_info_t;

void iconButton(const bool here, const int iconid, const icon_info_t &ico, const text_info_t (&txt)[2]) {
  const bool cn = hmiIsChinese();
  dwinIconShow(ICON, iconid + here, ico.x, ico.y);
  if (here) dwinDrawRectangle(0, COLOR_WHITE, ico.x, ico.y, ico.x + ico.w - 1, ico.y + ico.h - 1);
  dwinFrameAreaCopy(1, txt[cn].x, txt[cn].y[here], txt[cn].x + txt[cn].w - 1, txt[cn].y[here] + txt[cn].h - 1, ico.x + (ico.w - txt[cn].w) / 2, (ico.y + ico.h - 28) - txt[cn].h/2);
}

//
// Main Menu: "Print"
//
void iconPrint() {
  constexpr icon_info_t ico = { 17, 110, 110, 100 };
  constexpr text_info_t txt[2] = {
    { 1, { 417, 449 }, 30, 14 },
    { 1, { 405, 447 }, 27, 15 }
  };
  iconButton(select_page.now == PAGE_PRINT, ICON_Print_0, ico, txt);
}

//
// Main Menu: "Prepare"
//
void iconPrepare() {
  constexpr icon_info_t ico = { 145, 110, 110, 100 };
  constexpr text_info_t txt[2] = {
    { 33, { 417, 449 }, 51, 14 },
    { 31, { 405, 447 }, 27, 15 }
  };
  iconButton(select_page.now == PAGE_PREPARE, ICON_Prepare_0, ico, txt);
}

//
// Main Menu: "Control"
//
void iconControl() {
  constexpr icon_info_t ico = { 17, 226, 110, 100 };
  constexpr text_info_t txt[2] = {
    { 85, { 417, 449 }, 46, 14 },
    { 61, { 405, 447 }, 27, 15 }
  };
  iconButton(select_page.now == PAGE_CONTROL, ICON_Control_0, ico, txt);
}

//
// Main Menu: "Info"
//
void iconStartInfo() {
  constexpr icon_info_t ico = { 145, 226, 110, 100 };
  constexpr text_info_t txt[2] = {
    { 133, { 417, 449 }, 23, 14 },
    {  91, { 405, 447 }, 27, 15 }
  };
  iconButton(select_page.now == PAGE_INFO_LEVELING, ICON_Info_0, ico, txt);
}

//
// Main Menu: "Level"
//
void iconLeveling() {
  constexpr icon_info_t ico = { 145, 226, 110, 100 };
  constexpr text_info_t txt[2] = {
    {  88, { 433, 464 }, 36, 14 },
    { 211, { 405, 447 }, 27, 15 }
  };
  iconButton(select_page.now == PAGE_INFO_LEVELING, ICON_Leveling_0, ico, txt);
}

//
// Printing: "Tune"
//
void iconTune() {
  constexpr icon_info_t ico = { 8, 232, 80, 100 };
  constexpr text_info_t txt[2] = {
    {   0, { 433, 464 }, 32, 14 },
    { 121, { 405, 447 }, 27, 15 }
  };
  iconButton(select_print.now == PRINT_SETUP, ICON_Setup_0, ico, txt);
}

//
// Printing: "Pause"
//
void iconPause() {
  constexpr icon_info_t ico = { 96, 232, 80, 100 };
  constexpr text_info_t txt[2] = {
    { 157, { 417, 449 }, 39, 14 },
    { 181, { 405, 447 }, 27, 15 }
  };
  iconButton(select_print.now == PRINT_PAUSE_RESUME, ICON_Pause_0, ico, txt);
}

//
// Printing: "Resume"
//
void iconResume() {
  constexpr icon_info_t ico = { 96, 232, 80, 100 };
  constexpr text_info_t txt[2] = {
    { 33, { 433, 464 }, 53, 14 },
    {  1, { 405, 447 }, 27, 15 }
  };
  iconButton(select_print.now == PRINT_PAUSE_RESUME, ICON_Continue_0, ico, txt);
}

void iconResumeOrPause() {
  if (printingIsPaused() || hmiFlag.pause_flag || hmiFlag.pause_action)
    iconResume();
  else
    iconPause();
}

//
// Printing: "Stop"
//
void iconStop() {
  constexpr icon_info_t ico = { 184, 232, 80, 100 };
  constexpr text_info_t txt[2] = {
    { 196, { 417, 449 }, 29, 14 },
    { 151, { 405, 447 }, 27, 12 }
  };
  iconButton(select_print.now == PRINT_STOP, ICON_Stop_0, ico, txt);
}

inline void clearTitleBar() {
  dwinDrawBox(1, COLOR_BG_BLUE, 0, 0, DWIN_WIDTH, TITLE_HEIGHT);
}

void drawTitle(FSTR_P ftitle) {
  dwinDrawString(false, DWIN_FONT_HEAD, COLOR_WHITE, COLOR_BG_BLUE, 14, 4, ftitle);
}

inline void clearMenuArea() {
  dwinDrawBox(1, COLOR_BG_BLACK, 0, TITLE_HEIGHT, DWIN_WIDTH, STATUS_Y - TITLE_HEIGHT);
}

void clearMainWindow() {
  clearTitleBar();
  clearMenuArea();
}

void clearPopupArea() {
  clearTitleBar();
  dwinDrawRectangle(1, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, DWIN_HEIGHT);
}

void drawPopupBkgd60() {
  dwinDrawRectangle(1, COLOR_BG_WINDOW, 14, 60, 258, 330);
}

void drawPopupBkgd105() {
  dwinDrawRectangle(1, COLOR_BG_WINDOW, 14, 105, 258, 374);
}

void drawMoreIcon(const uint8_t line) {
  dwinIconShow(ICON, ICON_More, 226, MBASE(line) - 3);
}

void drawMenuCursor(const uint8_t line) {
  //dwinIconShow(ICON, ICON_Rectangle, 0, MBASE(line) - 18);
  dwinDrawRectangle(1, COLOR_RECTANGLE, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

void eraseMenuCursor(const uint8_t line) {
  dwinDrawRectangle(1, COLOR_BG_BLACK, 0, MBASE(line) - 18, 14, MBASE(line + 1) - 20);
}

void moveHighlight(const int16_t from, const uint16_t newline) {
  eraseMenuCursor(newline - from);
  drawMenuCursor(newline);
}

void addMenuLine() {
  moveHighlight(1, MROWS);
  dwinDrawLine(COLOR_LINE, 16, MBASE(MROWS + 1) - 20, 256, MBASE(MROWS + 1) - 19);
}

void scrollMenu(const uint8_t dir) {
  dwinFrameAreaMove(1, dir, MLINE, COLOR_BG_BLACK, 0, 31, DWIN_WIDTH, 349);
  switch (dir) {
    case DWIN_SCROLL_DOWN: moveHighlight(-1, 0); break;
    case DWIN_SCROLL_UP:   addMenuLine(); break;
  }
}

inline uint16_t nr_sd_menu_items() {
  return card.get_num_items() + !card.flag.workDirIsRoot;
}

void eraseMenuText(const uint8_t line) {
  dwinDrawRectangle(1, COLOR_BG_BLACK, LBLX, MBASE(line) - 14, 271, MBASE(line) + 28);
}

void drawMenuIcon(const uint8_t line, const uint8_t icon) {
  dwinIconShow(ICON, icon, 26, MBASE(line) - 3);
}

void _decorateMenuItem(const uint8_t line, const uint8_t icon, bool more) {
  if (icon) drawMenuIcon(line, icon);
  if (more) drawMoreIcon(line);
}
void drawMenuItem(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr, bool more=false) {
  if (label) dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, LBLX, MBASE(line) - 1, (char*)label);
  _decorateMenuItem(line, icon, more);
}
void drawMenuItem(const uint8_t line, const uint8_t icon=0, FSTR_P const flabel=nullptr, bool more=false) {
  if (flabel) dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, LBLX, MBASE(line) - 1, flabel);
  _decorateMenuItem(line, icon, more);
}

void drawMenuLine(const uint8_t line, const uint8_t icon=0, const char * const label=nullptr, bool more=false) {
  drawMenuItem(line, icon, label, more);
  dwinDrawLine(COLOR_LINE, 16, MBASE(line) + 33, 256, MBASE(line) + 34);
}

void drawMenuLine(const uint8_t line, const uint8_t icon, FSTR_P const flabel, bool more=false) {
  drawMenuItem(line, icon, flabel, more);
  dwinDrawLine(COLOR_LINE, 16, MBASE(line) + 33, 256, MBASE(line) + 34);
}

void drawCheckboxLine(const uint8_t line, const bool ison) {
  const uint16_t x = 225, y = EBASE(line) - 2;
  dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, x + 5, y, ison ? F("X") : F(" "));
  dwinDrawRectangle(0, COLOR_WHITE, x + 2, y + 2, x + 16, y + 16);
}

// AreaCopy for a Menu Item
void itemAreaCopy(const uint16_t x1, const uint16_t y1, const uint16_t x2, const uint16_t y2, const uint8_t row=0, const uint16_t inset=0, const uint16_t yadd=0) {
  dwinFrameAreaCopy(1, x1, y1, x2, y2, LBLX + inset, MBASE(row) + yadd);
}

// AreaCopy for a Screen Title
void dwinFrameTitleCopy(const uint16_t x1, const uint16_t y1, const uint16_t w, const uint16_t h) {
  dwinFrameAreaCopy(1, x1, y1, x1 + w - 1, y1 + h - 1, (DWIN_WIDTH - w) / 2, (TITLE_HEIGHT - h) / 2);
}

// Draw "Back" line at the top
void drawBackFirst(const bool is_sel=true) {
  drawMenuLine(0, ICON_Back);
  if (hmiIsChinese())
    itemAreaCopy(129,  72, 156,  84);
  else
    itemAreaCopy(223, 179, 254, 189);
  if (is_sel) drawMenuCursor(0);
}

//
// Draw Menus
//
#define CASE_BACK          0

#define MOTION_CASE_RATE   1
#define MOTION_CASE_ACCEL  2
#define MOTION_CASE_JERK   (MOTION_CASE_ACCEL + ENABLED(CLASSIC_JERK))
#define MOTION_CASE_STEPS  (MOTION_CASE_JERK + 1)
#define MOTION_CASE_TOTAL  MOTION_CASE_STEPS

#define PREPARE_CASE_MOVE  1
#define PREPARE_CASE_DISA  2
#define PREPARE_CASE_HOME  3
#define PREPARE_CASE_ZOFF (PREPARE_CASE_HOME + ENABLED(HAS_ZOFFSET_ITEM))
#define PREPARE_CASE_PLA  (PREPARE_CASE_ZOFF + ENABLED(HAS_PREHEAT))
#define PREPARE_CASE_ABS  (PREPARE_CASE_PLA + (TERN0(HAS_PREHEAT, PREHEAT_COUNT > 1)))
#define PREPARE_CASE_COOL (PREPARE_CASE_ABS + ANY(HAS_HOTEND, HAS_HEATED_BED))
#define PREPARE_CASE_LANG (PREPARE_CASE_COOL + 1)
#define PREPARE_CASE_TOTAL PREPARE_CASE_LANG

#define CONTROL_CASE_TEMP 1
#define CONTROL_CASE_MOVE  (CONTROL_CASE_TEMP + 1)
#define CONTROL_CASE_SAVE  (CONTROL_CASE_MOVE + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_LOAD  (CONTROL_CASE_SAVE + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_RESET (CONTROL_CASE_LOAD + ENABLED(EEPROM_SETTINGS))
#define CONTROL_CASE_ADVSET (CONTROL_CASE_RESET + 1)
#define CONTROL_CASE_INFO  (CONTROL_CASE_ADVSET + 1)
#define CONTROL_CASE_TOTAL CONTROL_CASE_INFO

#define TUNE_CASE_SPEED 1
#define TUNE_CASE_TEMP (TUNE_CASE_SPEED + ENABLED(HAS_HOTEND))
#define TUNE_CASE_BED  (TUNE_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TUNE_CASE_FAN  (TUNE_CASE_BED + ENABLED(HAS_FAN))
#define TUNE_CASE_ZOFF (TUNE_CASE_FAN + ENABLED(HAS_ZOFFSET_ITEM))
#define TUNE_CASE_TOTAL TUNE_CASE_ZOFF

#define TEMP_CASE_TEMP (0 + ENABLED(HAS_HOTEND))
#define TEMP_CASE_BED  (TEMP_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define TEMP_CASE_FAN  (TEMP_CASE_BED + ENABLED(HAS_FAN))
#define TEMP_CASE_PLA  (TEMP_CASE_FAN + ENABLED(HAS_PREHEAT))
#define TEMP_CASE_ABS  (TEMP_CASE_PLA + (TERN0(HAS_PREHEAT, PREHEAT_COUNT > 1)))
#define TEMP_CASE_TOTAL TEMP_CASE_ABS

#define PREHEAT_CASE_TEMP (0 + ENABLED(HAS_HOTEND))
#define PREHEAT_CASE_BED  (PREHEAT_CASE_TEMP + ENABLED(HAS_HEATED_BED))
#define PREHEAT_CASE_FAN  (PREHEAT_CASE_BED + ENABLED(HAS_FAN))
#define PREHEAT_CASE_SAVE (PREHEAT_CASE_FAN + ENABLED(EEPROM_SETTINGS))
#define PREHEAT_CASE_TOTAL PREHEAT_CASE_SAVE

#define ADVSET_CASE_HOMEOFF   1
#define ADVSET_CASE_PROBEOFF  (ADVSET_CASE_HOMEOFF + ENABLED(HAS_ONESTEP_LEVELING))
#define ADVSET_CASE_HEPID     (ADVSET_CASE_PROBEOFF + ENABLED(HAS_HOTEND))
#define ADVSET_CASE_BEDPID    (ADVSET_CASE_HEPID + ENABLED(HAS_HEATED_BED))
#define ADVSET_CASE_PWRLOSSR  (ADVSET_CASE_BEDPID + ENABLED(POWER_LOSS_RECOVERY))
#define ADVSET_CASE_TOTAL     ADVSET_CASE_PWRLOSSR

//
// Draw Menus
//

void say_move_en(const uint8_t row) {
  itemAreaCopy( 69,  61, 102,  71, row);         // "Move"
}
void say_max_en(const uint8_t row) {
  itemAreaCopy( 75, 119, 100, 129, row);         // "Max"
}
void say_jerk_en(const uint8_t row) {
  itemAreaCopy(104, 119, 128, 129, row, 30);     // "Jerk"
}
void say_speed_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(133, 119, 172, 132, row, inset);  // "Speed"
}
void say_max_accel_en(const uint8_t row) {
   say_max_en(row);                               // "Max"
   itemAreaCopy(  0, 135,  79, 145, row, 30);    // "Acceleration"
}
void say_max_jerk_speed_en(const uint8_t row) {
  itemAreaCopy( 75, 119, 172, 132, row);         // "Max Jerk Speed"
}
void say_x_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(175, 119, 184, 129, row, inset);  // "X"
}
void say_y_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(184, 119, 192, 129, row, inset);  // "Y"
}
void say_z_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(193, 119, 201, 129, row, inset);  // "Z"
}
void say_e_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(201, 119, 209, 129, row, inset);  // "E"
}
void say_pla_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(131, 164, 153, 174, row, inset);  // "PLA"
}
void say_abs_en(const uint16_t inset, const uint8_t row) {
  itemAreaCopy(157,  76, 181,  86, row, inset);  // "ABS"
}
void say_home_offs_en(const uint8_t row) {
  itemAreaCopy(153, 193, 225, 203, row);         // "Home Offset"
}
void say_probe_offs_en(const uint8_t row) {
  itemAreaCopy(153, 205, 225, 215, row);         // "Probe Offset"
}
void say_steps_per_mm_en(const uint8_t row) {
  itemAreaCopy(  1, 151,  91, 161, row);         // "Steps-per-mm"
}

void dwinDrawLabel(const uint8_t row, char *string) {
  dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, LBLX, MBASE(row), string);
}
void dwinDrawLabel(const uint8_t row, FSTR_P title) {
  dwinDrawLabel(row, (char*)title);
}

void dwinDrawSignedFloat(uint8_t size, uint16_t bColor, uint8_t iNum, uint8_t fNum, uint16_t x, uint16_t y, int32_t value) {
  dwinDrawString(true, size, COLOR_WHITE, bColor, x - 8, y, value < 0 ? F("-") : F(" "));
  dwinDrawFloatValue(true, true, 0, size, COLOR_WHITE, bColor, iNum, fNum, x, y, value < 0 ? -value : value);
}

void drawEditInteger3(const uint8_t row, const uint16_t value, const bool active=false) {
  dwinDrawIntValue(true, true, 0, font8x16, COLOR_WHITE, active ? COLOR_SELECT : COLOR_BG_BLACK, 3, 220, EBASE(row), value);
}

void drawEditInteger4(const uint8_t row, const uint16_t value, const bool active=false) {
  dwinDrawIntValue(true, true, 0, font8x16, COLOR_WHITE, active ? COLOR_SELECT : COLOR_BG_BLACK, 4, 220 - 1 * 8, EBASE(row), value);
}

void drawEditFloat3(const uint8_t row, const uint16_t value, const bool active=false) {
  dwinDrawFloatValue(true, true, 0, font8x16, COLOR_WHITE, active ? COLOR_SELECT : COLOR_BG_BLACK, 3, UNITFDIGITS, 220 - UNITFDIGITS * 8, EBASE(row), (int32_t)value);
}

void drawEditSignedFloat2(const uint8_t row, const float value, const bool active=false) {
  dwinDrawSignedFloat(font8x16, active ? COLOR_SELECT : COLOR_BG_BLACK, 2, UNITFDIGITS + 1, 220 + 8 - UNITFDIGITS * 8, EBASE(row), value);
}

void drawEditSignedFloat3(const uint8_t row, const float value, const bool active=false) {
  dwinDrawSignedFloat(font8x16, active ? COLOR_SELECT : COLOR_BG_BLACK, 3, UNITFDIGITS, 220 - UNITFDIGITS * 8, EBASE(row), value);
}

void drawStatInt(const uint16_t xpos, const uint16_t ypos, const uint16_t value) {
  dwinDrawIntValue(true, true, 0, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 3, xpos, ypos, value);
}

void drawStatFloat(const uint16_t xpos, const uint16_t ypos, const float value) {
  dwinDrawFloatValue(true, true, 0, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 2, 2, xpos, ypos, value);
}

//
// Prepare Menu
//

void itemPrepareMove(const uint8_t row) {
  if (hmiIsChinese())
    itemAreaCopy(159, 70, 200, 84, row);
  else
    say_move_en(row);             // "Move"
  drawMenuLine(row, ICON_Axis);
  drawMoreIcon(row);
}

void itemPrepareDisable(const uint8_t row) {
  if (hmiIsChinese())
    itemAreaCopy(204, 70, 259, 82, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_DISABLE_STEPPERS));
    #else
      itemAreaCopy(104, 61, 191, 74, row); // "Disable Stepper"
    #endif
  }
  drawMenuLine(row, ICON_CloseMotor);
}

void itemPrepareHome(const uint8_t row) {
  if (hmiIsChinese())
    itemAreaCopy(0, 89, 41, 101, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_AUTO_HOME));
    #else
      itemAreaCopy(202, 61, 271, 71, row); // "Auto Home"
    #endif
  }
  drawMenuLine(row, ICON_Homing);
}

#if HAS_ZOFFSET_ITEM

  void itemPrepareOffset(const uint8_t row) {
    if (hmiIsChinese()) {
      #if HAS_BED_PROBE
        itemAreaCopy(174, 164, 223, 177, row);
      #else
        itemAreaCopy(43, 89, 98, 101, row);
      #endif
    }
    else {
      #if HAS_BED_PROBE
        #ifdef USE_STRING_TITLES
          dwinDrawLabel(row, GET_TEXT_F(MSG_ZPROBE_ZOFFSET));
        #else
          itemAreaCopy( 94, 179, 143, 190, row); // "Z-Offset"
        #endif
      #else
        #ifdef USE_STRING_TITLES
          dwinDrawLabel(row, GET_TEXT_F(MSG_SET_HOME_OFFSETS));
        #else
          itemAreaCopy(  1,  76, 103,  87, row); // "Set home offsets"
        #endif
      #endif
    }
    drawEditSignedFloat2(row, BABY_Z_VAR * 100);
    drawMenuLine(row, ICON_SetHome);
  }

#endif

#if HAS_PREHEAT
  void itemPrepare_PLA(const uint8_t row) {
    if (hmiIsChinese())
      itemAreaCopy(100, 89, 151, 101, row);
    else {
      #ifdef USE_STRING_TITLES
        dwinDrawLabel(row, GET_TEXT_F(MSG_PREHEAT_1));
      #else
        itemAreaCopy(108,  76, 155,  87, row); // "Preheat"
        say_pla_en(52, row);                    // "PLA"
      #endif
    }
    drawMenuLine(row, ICON_PLAPreheat);
  }

  #if PREHEAT_COUNT > 1
    void itemPrepare_ABS(const uint8_t row) {
      if (hmiIsChinese())
        itemAreaCopy(180,  89, 233, 100, row);
      else {
        #ifdef USE_STRING_TITLES
          dwinDrawLabel(row, F("Preheat " PREHEAT_2_LABEL));
        #else
          itemAreaCopy(108,  76, 155,  87, row); // "Preheat"
          say_abs_en(52, row);                    // "ABS"
        #endif
      }
      drawMenuLine(row, ICON_ABSPreheat);
    }
  #endif

  void itemPrepareCool(const uint8_t row) {
    if (hmiIsChinese())
      itemAreaCopy(1, 104,  56, 117, row);
    else {
      #ifdef USE_STRING_TITLES
        dwinDrawLabel(row, GET_TEXT_F(MSG_COOLDOWN));
      #else
        itemAreaCopy(200, 76, 264, 86, row); // "Cooldown"
      #endif
    }
    drawMenuLine(row, ICON_Cool);
  }
#endif

void itemPrepareLang(const uint8_t row) {
  if (hmiIsChinese())
    itemAreaCopy(239, 134, 266, 146, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, F("UI Language"));
    #else
      itemAreaCopy(1, 194, 96, 206, row);    // "LCD Language"
    #endif
  }
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 226, EBASE(row), hmiIsChinese() ? F("CN") : F("EN"));
  drawMenuIcon(row, ICON_Language);
}

#define VISI(T,L,S) (WITHIN(L, T - MROWS, MROWS) || WITHIN(S, 0, MROWS))

void drawPrepareMenu() {
  clearMainWindow();

  const int16_t scroll = MROWS - index_prepare; // Scrolled-up lines
  #define PSCROL(L) (scroll + (L))
  #define PVISI(L) VISI(PREPARE_CASE_TOTAL, L, PSCROL(L))

  if (hmiIsChinese())
    dwinFrameTitleCopy(133, 1, 28, 13);   // "Prepare"
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_PREPARE));
    #else
      dwinFrameTitleCopy(179, 0, 48, 14); // "Prepare"
    #endif
  }

  if (PVISI(0)) drawBackFirst(select_prepare.now == CASE_BACK);                 // < Back
  if (PVISI(PREPARE_CASE_MOVE)) itemPrepareMove(PSCROL(PREPARE_CASE_MOVE));     // Move >
  if (PVISI(PREPARE_CASE_DISA)) itemPrepareDisable(PSCROL(PREPARE_CASE_DISA));  // Disable Stepper
  if (PVISI(PREPARE_CASE_HOME)) itemPrepareHome(PSCROL(PREPARE_CASE_HOME));     // Auto Home
  #if HAS_ZOFFSET_ITEM
    if (PVISI(PREPARE_CASE_ZOFF)) itemPrepareOffset(PSCROL(PREPARE_CASE_ZOFF)); // Edit Z-Offset / Babystep / Set Home Offset
  #endif
  #if HAS_PREHEAT
    if (PVISI(PREPARE_CASE_PLA)) itemPrepare_PLA(PSCROL(PREPARE_CASE_PLA));      // Preheat PLA
    #if PREHEAT_COUNT > 1
      if (PVISI(PREPARE_CASE_ABS)) itemPrepare_ABS(PSCROL(PREPARE_CASE_ABS));    // Preheat ABS
    #endif
  #endif
  #if HAS_HOTEND || HAS_HEATED_BED
    if (PVISI(PREPARE_CASE_COOL)) itemPrepareCool(PSCROL(PREPARE_CASE_COOL));   // Cooldown
  #endif
  if (PVISI(PREPARE_CASE_LANG)) itemPrepareLang(PSCROL(PREPARE_CASE_LANG));     // Language CN/EN

  if (select_prepare.now != CASE_BACK) drawMenuCursor(PSCROL(select_prepare.now));
}

//
// Control Menu
//

void itemControlTemp(const uint16_t row) {
  if (hmiIsChinese())
    itemAreaCopy(57, 104,  84, 116, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_TEMPERATURE));
    #else
      itemAreaCopy(1, 89,  83, 101, row);
    #endif
  }
  drawMenuLine(row, ICON_Temperature);
  drawMoreIcon(row);
}

void itemControlMotion(const uint16_t row) {
  if (hmiIsChinese())
    itemAreaCopy(87, 104, 114, 116, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_MOTION));
    #else
      itemAreaCopy(84, 89, 128,  99, row);
    #endif
  }
  drawMenuLine(row, ICON_Motion);
  drawMoreIcon(row);
}

void itemControlAdvanced(const uint16_t row) {
  if (hmiIsChinese())
    itemAreaCopy(62, 180, 120, 192, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_ADVANCED_SETTINGS));
    #else
      itemAreaCopy(82, 135, 200, 149, row);
    #endif
  }
  drawMenuLine(row, ICON_AdvSet);
  drawMoreIcon(row);
}

void itemControlInfo(const uint16_t row) {
  if (hmiIsChinese())
    itemAreaCopy(231, 104, 258, 116, row);
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_INFO_SCREEN));
    #else
      itemAreaCopy(0, 104, 24, 114, row);
    #endif
  }
  drawMenuLine(row, ICON_Info);
  drawMoreIcon(row);
}

void drawControlMenu() {
  clearMainWindow();

  #if CONTROL_CASE_TOTAL >= TROWS
    const int16_t scroll = MROWS - index_control; // Scrolled-up lines
  #else
    constexpr int16_t scroll = 0;
  #endif
  #define CSCROL(L) (scroll + (L))
  #define CLINE(L) MBASE(CSCROL(L))
  #define CVISI(L) VISI(CONTROL_CASE_TOTAL, L, CSCROL(L))

  if (hmiIsChinese())
    dwinFrameTitleCopy(103, 1, 28, 14);     // "Control"
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_CONTROL));
    #else
      dwinFrameTitleCopy(128, 2, 49, 11);   // "Control"
    #endif
  }

  if (CVISI(0)) drawBackFirst(select_control.now == CASE_BACK);               // < Back
  if (CVISI(CONTROL_CASE_TEMP)) itemControlTemp(CSCROL(CONTROL_CASE_TEMP));   // Temperature >
  if (CVISI(CONTROL_CASE_MOVE)) itemControlMotion(CSCROL(CONTROL_CASE_MOVE)); // Motion >

  if (hmiIsChinese()) {
    #if ENABLED(EEPROM_SETTINGS)
      itemAreaCopy(117, 104, 172, 116, CSCROL(CONTROL_CASE_SAVE));   // "Store Configuration"
      itemAreaCopy(174, 103, 229, 116, CSCROL(CONTROL_CASE_LOAD));   // "Read Configuration"
      itemAreaCopy(  1, 118,  56, 131, CSCROL(CONTROL_CASE_RESET));  // "Reset Configuration"
    #endif
  }
  else {
    #ifdef USE_STRING_TITLES
      #if ENABLED(EEPROM_SETTINGS)
        if (CVISI(CONTROL_CASE_SAVE)) dwinDrawLabel(CSCROL(CONTROL_CASE_SAVE), GET_TEXT_F(MSG_STORE_EEPROM));        // "Store Configuration"
        if (CVISI(CONTROL_CASE_LOAD)) dwinDrawLabel(CSCROL(CONTROL_CASE_LOAD), GET_TEXT_F(MSG_LOAD_EEPROM));         // "Read Configuration"
        if (CVISI(CONTROL_CASE_RESET)) dwinDrawLabel(CSCROL(CONTROL_CASE_RESET), GET_TEXT_F(MSG_RESTORE_DEFAULTS));  // "Reset Configuration"
      #endif
    #else
      #if ENABLED(EEPROM_SETTINGS)
        if (CVISI(CONTROL_CASE_SAVE))
          itemAreaCopy(150,  89, 263, 102, CSCROL(CONTROL_CASE_SAVE));       // "Store Configuration"
        if (CVISI(CONTROL_CASE_LOAD)) {
          itemAreaCopy( 26, 104,  57, 114, CSCROL(CONTROL_CASE_LOAD));       // "Read"
          itemAreaCopy(182,  89, 263, 102, CSCROL(CONTROL_CASE_LOAD), 34);   // "Configuration"
        }
        if (CVISI(CONTROL_CASE_RESET)) {
          itemAreaCopy( 59, 104,  93, 114, CSCROL(CONTROL_CASE_RESET));      // "Reset"
          itemAreaCopy(182,  89, 263, 102, CSCROL(CONTROL_CASE_RESET), 37);  // "Configuration"
        }
      #endif
    #endif
  }

  if (CVISI(CONTROL_CASE_ADVSET)) itemControlAdvanced(CSCROL(CONTROL_CASE_ADVSET));
  if (CVISI(CONTROL_CASE_INFO)) itemControlInfo(CSCROL(CONTROL_CASE_INFO));

  if (select_control.now != CASE_BACK && CVISI(select_control.now))
    drawMenuCursor(CSCROL(select_control.now));

  // Draw icons and lines
  #define _TEMP_ICON(N, I, M) do { \
    if (CVISI(N)) { \
      drawMenuLine(CSCROL(N), I); \
      if (M) { \
        drawMoreIcon(CSCROL(N)); \
      } \
    } \
  } while(0)

  #if ENABLED(EEPROM_SETTINGS)
    _TEMP_ICON(CONTROL_CASE_SAVE, ICON_WriteEEPROM, false);
    _TEMP_ICON(CONTROL_CASE_LOAD, ICON_ReadEEPROM, false);
    _TEMP_ICON(CONTROL_CASE_RESET, ICON_ResetEEPROM, false);
  #endif
}

//
// Tune Menu
//

void drawTuneMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(73, 2, 28, 12);       // "Tune"
    itemAreaCopy(116, 164, 171, 176, TUNE_CASE_SPEED);
    #if HAS_HOTEND
      itemAreaCopy(1, 134, 56, 146, TUNE_CASE_TEMP);
    #endif
    #if HAS_HEATED_BED
      itemAreaCopy(58, 134, 113, 146, TUNE_CASE_BED);
    #endif
    #if HAS_FAN
      itemAreaCopy(115, 134, 170, 146, TUNE_CASE_FAN);
    #endif
    #if HAS_ZOFFSET_ITEM
      itemAreaCopy(174, 164, 223, 177, TUNE_CASE_ZOFF);
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_TUNE));
    #else
      dwinFrameTitleCopy(94, 2, 33, 11);    // "Tune"
    #endif
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(TUNE_CASE_SPEED, GET_TEXT_F(MSG_SPEED));
      #if HAS_HOTEND
        dwinDrawLabel(TUNE_CASE_TEMP, GET_TEXT_F(MSG_UBL_SET_TEMP_HOTEND));
      #endif
      #if HAS_HEATED_BED
        dwinDrawLabel(TUNE_CASE_BED, GET_TEXT_F(MSG_UBL_SET_TEMP_BED));
      #endif
      #if HAS_FAN
        dwinDrawLabel(TUNE_CASE_FAN, GET_TEXT_F(MSG_FAN_SPEED));
      #endif
      dwinDrawLabel(TUNE_CASE_ZOFF, GET_TEXT_F(MSG_ZPROBE_ZOFFSET));
    #else
      itemAreaCopy(1, 179, 92, 190, TUNE_CASE_SPEED);          // "Print speed"
      #if HAS_HOTEND
        itemAreaCopy(197, 104, 238, 114, TUNE_CASE_TEMP);      // "Hotend"
        itemAreaCopy(  1,  89,  83, 101, TUNE_CASE_TEMP, 44);  // "Temperature"
      #endif
      #if HAS_HEATED_BED
        itemAreaCopy(240, 104, 264, 114, TUNE_CASE_BED);       // "Bed"
        itemAreaCopy(  1,  89,  83, 101, TUNE_CASE_BED, 27);   // "Temperature"
      #endif
      #if HAS_FAN
        itemAreaCopy(0, 119, 64, 132, TUNE_CASE_FAN);          // "Fan speed"
      #endif
      #if HAS_ZOFFSET_ITEM
        itemAreaCopy(93, 179, 141, 189, TUNE_CASE_ZOFF);       // "Z-offset"
      #endif
    #endif
  }

  drawBackFirst(select_tune.now == CASE_BACK);
  if (select_tune.now != CASE_BACK) drawMenuCursor(select_tune.now);

  drawMenuLine(TUNE_CASE_SPEED, ICON_Speed);
  drawEditInteger3(TUNE_CASE_SPEED, feedrate_percentage);

  #if HAS_HOTEND
    drawMenuLine(TUNE_CASE_TEMP, ICON_HotendTemp);
    drawEditInteger3(TUNE_CASE_TEMP, thermalManager.degTargetHotend(0));
  #endif
  #if HAS_HEATED_BED
    drawMenuLine(TUNE_CASE_BED, ICON_BedTemp);
    drawEditInteger3(TUNE_CASE_BED, thermalManager.degTargetBed());
  #endif
  #if HAS_FAN
    drawMenuLine(TUNE_CASE_FAN, ICON_FanSpeed);
    drawEditInteger3(TUNE_CASE_FAN, thermalManager.fan_speed[0]);
  #endif
  #if HAS_ZOFFSET_ITEM
    drawMenuLine(TUNE_CASE_ZOFF, ICON_Zoffset);
    drawEditSignedFloat2(TUNE_CASE_ZOFF, BABY_Z_VAR * 100);
  #endif
}

//
// Motion Menu
//
void drawMotionMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(1, 16, 28, 13);                        // "Motion"
    itemAreaCopy(173, 133, 228, 147, MOTION_CASE_RATE);        // Max speed
    itemAreaCopy(173, 133, 200, 147, MOTION_CASE_ACCEL);       // Max...
    itemAreaCopy(28, 149, 69, 161, MOTION_CASE_ACCEL, 30, 1);  // ...Acceleration
    #if ENABLED(CLASSIC_JERK)
      itemAreaCopy(173, 133, 200, 147, MOTION_CASE_JERK);      // Max...
      itemAreaCopy(1, 180, 28, 192, MOTION_CASE_JERK, 30, 1);  // ...
      itemAreaCopy(202, 133, 228, 147, MOTION_CASE_JERK, 57);  // ...Jerk
    #endif
    itemAreaCopy(153, 148, 194, 161, MOTION_CASE_STEPS);       // Flow ratio
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_MOTION));
    #else
      dwinFrameTitleCopy(144, 16, 46, 11);                            // "Motion"
    #endif
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(MOTION_CASE_RATE, F("Feedrate"));                 // "Feedrate"
      dwinDrawLabel(MOTION_CASE_ACCEL, GET_TEXT_F(MSG_ACCELERATION)); // "Acceleration"
      #if ENABLED(CLASSIC_JERK)
        dwinDrawLabel(MOTION_CASE_JERK, GET_TEXT_F(MSG_JERK));        // "Jerk"
      #endif
      dwinDrawLabel(MOTION_CASE_STEPS, GET_TEXT_F(MSG_STEPS_PER_MM)); // "Steps/mm"
    #else
      say_max_en(MOTION_CASE_RATE); say_speed_en(30, MOTION_CASE_RATE); // "Max Speed"
      say_max_accel_en(MOTION_CASE_ACCEL);                              // "Max Acceleration"
      #if ENABLED(CLASSIC_JERK)
        say_max_en(MOTION_CASE_JERK); say_jerk_en(MOTION_CASE_JERK);    // "Max Jerk"
      #endif
      say_steps_per_mm_en(MOTION_CASE_STEPS);                           // "Steps-per-mm"
    #endif
  }

  drawBackFirst(select_motion.now == CASE_BACK);
  if (select_motion.now != CASE_BACK) drawMenuCursor(select_motion.now);

  uint8_t i = 0;
  #define _MOTION_ICON(N) drawMenuLine(++i, ICON_MaxSpeed + (N) - 1)
  _MOTION_ICON(MOTION_CASE_RATE); drawMoreIcon(i);
  _MOTION_ICON(MOTION_CASE_ACCEL); drawMoreIcon(i);
  #if ENABLED(CLASSIC_JERK)
    _MOTION_ICON(MOTION_CASE_JERK); drawMoreIcon(i);
  #endif
  _MOTION_ICON(MOTION_CASE_STEPS); drawMoreIcon(i);
}

//
// Draw Popup Windows
//

#if HAS_HOTEND || HAS_HEATED_BED

  void dwinPopupTemperature(const bool toohigh) {
    clearPopupArea();
    drawPopupBkgd105();
    if (toohigh) {
      dwinIconShow(ICON, ICON_TempTooHigh, 102, 165);
      if (hmiIsChinese()) {
        dwinFrameAreaCopy(1, 103, 371, 237, 386,  52, 285); // Temp Too High
        dwinFrameAreaCopy(1, 151, 389, 185, 402, 187, 285);
        dwinFrameAreaCopy(1, 189, 389, 271, 402,  95, 310);
      }
      else {
        dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, 36, 300, F("Nozzle or Bed temperature"));
        dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, 92, 300, F("is too high"));
      }
    }
    else {
      dwinIconShow(ICON, ICON_TempTooLow, 102, 165);
      if (hmiIsChinese()) {
        dwinFrameAreaCopy(1, 103, 371, 270, 386, 52, 285); // Tenp Too Low
        dwinFrameAreaCopy(1, 189, 389, 271, 402, 95, 310);
      }
      else {
        dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, 36, 300, F("Nozzle or Bed temperature"));
        dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, 92, 300, F("is too low"));
      }
    }
  }

#endif

#if HAS_HOTEND

  void popupWindowETempTooLow() {
    clearMainWindow();
    drawPopupBkgd60();
    dwinIconShow(ICON, ICON_TempTooLow, 102, 105);
    if (hmiIsChinese()) {
      dwinFrameAreaCopy(1, 103, 371, 136, 386, 69, 240);      // Nozzle Too Cold
      dwinFrameAreaCopy(1, 170, 371, 270, 386, 69 + 33, 240);
      dwinIconShow(ICON, ICON_Confirm_C, 86, 280);
    }
    else {
      dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, 20, 235, F("Nozzle is too cold"));
      dwinIconShow(ICON, ICON_Confirm_E, 86, 280);
    }
  }

#endif

void popupWindowResume() {
  clearPopupArea();
  drawPopupBkgd105();
  if (hmiIsChinese()) {
    dwinFrameAreaCopy(1, 160, 338, 235, 354, 98, 135);    // Resume Interrupted Print
    dwinFrameAreaCopy(1, 103, 321, 271, 335, 52, 192);
    dwinIconShow(ICON, ICON_Cancel_C,    26, 307);
    dwinIconShow(ICON, ICON_Continue_C, 146, 307);
  }
  else {
    dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 14) / 2, 115, F("Continue Print"));
    dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 22) / 2, 192, F("It looks like the last"));
    dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 21) / 2, 212, F("file was interrupted."));
    dwinIconShow(ICON, ICON_Cancel_E,    26, 307);
    dwinIconShow(ICON, ICON_Continue_E, 146, 307);
  }
}

void popupWindowHome(const bool parking/*=false*/) {
  clearMainWindow();
  drawPopupBkgd60();
  dwinIconShow(ICON, ICON_Printer_0, 101, 105);
  if (hmiIsChinese()) {
    dwinFrameAreaCopy(1, 0, 371, 33, 386, 85, 240);       // Wait for Move to Complete
    dwinFrameAreaCopy(1, 203, 286, 271, 302, 118, 240);
    dwinFrameAreaCopy(1, 0, 389, 150, 402, 61, 280);
  }
  else {
    dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * (parking ? 7 : 10)) / 2, 230, parking ? F("Parking") : F("Homing XYZ"));
    dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
  }
}

#if HAS_ONESTEP_LEVELING

  void popupWindowLeveling() {
    clearMainWindow();
    drawPopupBkgd60();
    dwinIconShow(ICON, ICON_AutoLeveling, 101, 105);
    if (hmiIsChinese()) {
      dwinFrameAreaCopy(1, 0, 371, 100, 386, 84, 240);    // Wait for Leveling
      dwinFrameAreaCopy(1, 0, 389, 150, 402, 61, 280);
    }
    else {
      dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 12) / 2, 230, GET_TEXT_F(MSG_BED_LEVELING));
      dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 23) / 2, 260, F("Please wait until done."));
    }
  }

#endif

void drawSelectHighlight(const bool sel) {
  hmiFlag.select_flag = sel;
  const uint16_t c1 = sel ? COLOR_SELECT : COLOR_BG_WINDOW,
                 c2 = sel ? COLOR_BG_WINDOW : COLOR_SELECT;
  dwinDrawRectangle(0, c1, 25, 279, 126, 318);
  dwinDrawRectangle(0, c1, 24, 278, 127, 319);
  dwinDrawRectangle(0, c2, 145, 279, 246, 318);
  dwinDrawRectangle(0, c2, 144, 278, 247, 319);
}

void popupwindowPauseOrStop() {
  clearMainWindow();
  drawPopupBkgd60();
  if (hmiIsChinese()) {
         if (select_print.now == PRINT_PAUSE_RESUME) dwinFrameAreaCopy(1, 237, 338, 269, 356, 98, 150); // Pause
    else if (select_print.now == PRINT_STOP) dwinFrameAreaCopy(1, 221, 320, 253, 336, 98, 150);         // Stop
    dwinFrameAreaCopy(1, 220, 304, 264, 319, 130, 150); // Print
    dwinIconShow(ICON, ICON_Confirm_C, 26, 280);
    dwinIconShow(ICON, ICON_Cancel_C, 146, 280);
  }
  else {
         if (select_print.now == PRINT_PAUSE_RESUME) dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 11) / 2, 150, GET_TEXT_F(MSG_PAUSE_PRINT));
    else if (select_print.now == PRINT_STOP) dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 10) / 2, 150, GET_TEXT_F(MSG_STOP_PRINT));
    dwinIconShow(ICON, ICON_Confirm_E, 26, 280);
    dwinIconShow(ICON, ICON_Cancel_E, 146, 280);
  }
  drawSelectHighlight(true);
}

void drawPrintingScreen() {
  const uint16_t y = 168;
  if (hmiIsChinese()) {
    dwinFrameTitleCopy(30, 1, 42, 14);              // "Printing"
    dwinFrameAreaCopy(1,  0, 72,  63, 86,  43, y);  // "Printing Time"
    dwinFrameAreaCopy(1, 65, 72, 128, 86, 178, y);  // "Remain"
  }
  else {
    dwinFrameTitleCopy(42, 0, 47, 14);              // "Printing"
    dwinFrameAreaCopy(1,   1, 43,  97, 59,  43, y); // "Printing Time"
    dwinFrameAreaCopy(1, 100, 43, 152, 56, 178, y); // "Remain"
  }
}

void drawPrintProgressBar() {
  constexpr uint16_t y = 93, h = 21;
  dwinIconShow(ICON, ICON_Bar, 15, 93);
  dwinDrawRectangle(1, COLOR_BARFILL, 16 + _card_percent * 240 / 100, y, 256, y + h - 1);
  dwinDrawIntValue(true, true, 0, font8x16, COLOR_PERCENT, COLOR_BG_BLACK, 2, 117, y + 40, _card_percent);
  dwinDrawString(false, font8x16, COLOR_PERCENT, COLOR_BG_BLACK, 133, y + 40, F("%"));
}

void drawPrintProgressElapsed() {
  constexpr uint16_t x = 45, y = 192;
  duration_t elapsed = print_job_timer.duration(); // Print timer
  dwinDrawIntValue(true, true, 1, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 2, x, y, elapsed.value / 3600);
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, x + 8 * 2, y, F(":"));
  dwinDrawIntValue(true, true, 1, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 2, x + 8 * 3, y, (elapsed.value % 3600) / 60);
}

void drawPrintProgressRemain() {
  constexpr uint16_t x = 179, y = 192;
  dwinDrawIntValue(true, true, 1, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 2, x, y, _remain_time / 3600);
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, x + 8 * 2, y, F(":"));
  dwinDrawIntValue(true, true, 1, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 2, x + 8 * 3, y, (_remain_time % 3600) / 60);
}

void gotoPrintProcess() {
  checkkey = ID_PrintProcess;

  clearMainWindow();
  drawPrintingScreen();

  iconTune();
  iconResumeOrPause();
  iconStop();

  // Copy into filebuf string before entry
  char * const name = card.longest_filename();
  const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * MENU_CHR_W) / 2;
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, npos, 60, name);

  dwinIconShow(ICON, ICON_PrintTime,   17, 163);
  dwinIconShow(ICON, ICON_RemainTime, 150, 161);

  drawPrintProgressBar();
  drawPrintProgressElapsed();
  drawPrintProgressRemain();
}

void gotoMainMenu() {
  checkkey = ID_MainMenu;

  clearMainWindow();

  if (hmiIsChinese())
    dwinFrameTitleCopy(2, 2, 26, 13);   // "Home" etc
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_MAIN_MENU));
    #else
      dwinFrameTitleCopy(0, 2, 40, 11); // "Home"
    #endif
  }

  dwinIconShow(ICON, ICON_LOGO, 71, 52);

  iconPrint();
  iconPrepare();
  iconControl();
  TERN(HAS_ONESTEP_LEVELING, iconLeveling, iconStartInfo)();
}

void hmiPlanMove(const feedRate_t fr_mm_s) {
  if (!planner.is_full()) {
    planner.synchronize();
    planner.buffer_line(current_position, fr_mm_s);
    dwinUpdateLCD();
  }
}

void hmiMoveDone(const AxisEnum axis) {
  encoderRate.enabled = false;
  planner.synchronize();
  checkkey = ID_AxisMove;
  dwinUpdateLCD();
}

#if HAS_X_AXIS

  void hmiMoveX() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.moveScaled.x)) {
      drawEditFloat3(1, hmiValues.moveScaled.x);
      return hmiMoveDone(X_AXIS);
    }
    LIMIT(hmiValues.moveScaled.x, (X_MIN_POS) * MINUNITMULT, (X_MAX_POS) * MINUNITMULT);
    current_position.x = hmiValues.moveScaled.x / MINUNITMULT;
    drawEditFloat3(1, hmiValues.moveScaled.x, true);
    dwinUpdateLCD();
    hmiPlanMove(homing_feedrate(X_AXIS));
  }

#endif

#if HAS_Y_AXIS

  void hmiMoveY() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.moveScaled.y)) {
      drawEditFloat3(2, hmiValues.moveScaled.y);
      return hmiMoveDone(Y_AXIS);
    }
    LIMIT(hmiValues.moveScaled.y, (Y_MIN_POS) * MINUNITMULT, (Y_MAX_POS) * MINUNITMULT);
    current_position.y = hmiValues.moveScaled.y / MINUNITMULT;
    drawEditFloat3(2, hmiValues.moveScaled.y, true);
    dwinUpdateLCD();
    hmiPlanMove(homing_feedrate(Y_AXIS));
  }

#endif

#if HAS_Z_AXIS

  void hmiMoveZ() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.moveScaled.z)) {
      drawEditFloat3(3, hmiValues.moveScaled.z);
      return hmiMoveDone(Z_AXIS);
    }
    LIMIT(hmiValues.moveScaled.z, (Z_MIN_POS) * MINUNITMULT, (Z_MAX_POS) * MINUNITMULT);
    current_position.z = hmiValues.moveScaled.z / MINUNITMULT;
    drawEditFloat3(3, hmiValues.moveScaled.z, true);
    dwinUpdateLCD();
    hmiPlanMove(homing_feedrate(Z_AXIS));
  }

#endif

#if HAS_HOTEND

  void hmiMoveE() {
    static float last_E_scaled = 0;
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.moveScaled.e)) {
      last_E_scaled = hmiValues.moveScaled.e;
      drawEditSignedFloat3(4, last_E_scaled);
      return hmiMoveDone(E_AXIS);
    }
    LIMIT(hmiValues.moveScaled.e, last_E_scaled - (EXTRUDE_MAXLENGTH) * MINUNITMULT, last_E_scaled + (EXTRUDE_MAXLENGTH) * MINUNITMULT);
    current_position.e = hmiValues.moveScaled.e / MINUNITMULT;
    drawEditSignedFloat3(4, hmiValues.moveScaled.e, true);
    dwinUpdateLCD();
    hmiPlanMove(MMM_TO_MMS(FEEDRATE_E));
  }

#endif

#if HAS_ZOFFSET_ITEM

  void hmiZoffset() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    uint8_t zoff_line;
    switch (hmiValues.show_mode) {
      case -4: zoff_line = PREPARE_CASE_ZOFF + MROWS - index_prepare; break;
      default: zoff_line = TUNE_CASE_ZOFF + MROWS - index_tune;
    }
    if (applyEncoder(encoder_diffState, hmiValues.offset_value)) {
      encoderRate.enabled = false;
      #if HAS_BED_PROBE
        probe.offset.z = dwin_zoffset;
        TERN_(EEPROM_SETTINGS, settings.save());
      #endif
      checkkey = hmiValues.show_mode == -4 ? ID_Prepare : ID_Tune;
      drawEditSignedFloat2(zoff_line, TERN(HAS_BED_PROBE, BABY_Z_VAR * 100, hmiValues.offset_value));
      dwinUpdateLCD();
      return;
    }

    #if ENABLED(BABYSTEP_ZPROBE_OFFSET) && defined(PROBE_OFFSET_ZMIN)
      #define _OFFSET_ZMIN (PROBE_OFFSET_ZMIN)
    #else
      #define _OFFSET_ZMIN -20
    #endif
    #if ENABLED(BABYSTEP_ZPROBE_OFFSET) && defined(PROBE_OFFSET_ZMAX)
      #define _OFFSET_ZMAX (PROBE_OFFSET_ZMAX)
    #else
      #define _OFFSET_ZMAX 20
    #endif
    LIMIT(hmiValues.offset_value, _OFFSET_ZMIN * 100, _OFFSET_ZMAX * 100);

    last_zoffset = dwin_zoffset;
    dwin_zoffset = hmiValues.offset_value / 100.0f;
    #if ANY(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
      if (BABYSTEP_ALLOWED()) babystep.add_mm(Z_AXIS, dwin_zoffset - last_zoffset);
    #endif
    drawEditSignedFloat2(zoff_line, hmiValues.offset_value, true);
    dwinUpdateLCD();
  }

#endif // HAS_ZOFFSET_ITEM

#if HAS_HOTEND

  void hmiETemp() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    uint8_t temp_line;
    switch (hmiValues.show_mode) {
      case -1: temp_line = TEMP_CASE_TEMP; break;
      #if HAS_PREHEAT
        case -2: temp_line = PREHEAT_CASE_TEMP; break;
        #if PREHEAT_COUNT > 1
          case -3: temp_line = PREHEAT_CASE_TEMP; break;
        #endif
      #endif
      default: temp_line = TUNE_CASE_TEMP + MROWS - index_tune;
    }
    if (applyEncoder(encoder_diffState, hmiValues.tempE)) {
      encoderRate.enabled = false;
      #if HAS_PREHEAT
        if (hmiValues.show_mode == -2) {
          checkkey = ID_PLAPreheat;
          ui.material_preset[0].hotend_temp = hmiValues.tempE;
          drawEditInteger3(temp_line, ui.material_preset[0].hotend_temp);
          return;
        }
        #if PREHEAT_COUNT > 1
          if (hmiValues.show_mode == -3) {
            checkkey = ID_ABSPreheat;
            ui.material_preset[1].hotend_temp = hmiValues.tempE;
            drawEditInteger3(temp_line, ui.material_preset[1].hotend_temp);
            return;
          }
        #endif
      #endif

      if (hmiValues.show_mode == -1) // Temperature
        checkkey = ID_TemperatureID;
      else
        checkkey = ID_Tune;
      drawEditInteger3(temp_line, hmiValues.tempE);
      thermalManager.setTargetHotend(hmiValues.tempE, 0);
      return;
    }
    // tempE limit
    LIMIT(hmiValues.tempE, HEATER_0_MINTEMP, thermalManager.hotend_max_target(0));
    // tempE value
    drawEditInteger3(temp_line, hmiValues.tempE, true);
  }

#endif // HAS_HOTEND

#if HAS_HEATED_BED

  void hmiBedTemp() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    uint8_t bed_line;
    switch (hmiValues.show_mode) {
      case -1: bed_line = TEMP_CASE_BED; break;
      #if HAS_PREHEAT
        case -2: bed_line = PREHEAT_CASE_BED; break;
        #if PREHEAT_COUNT > 1
          case -3: bed_line = PREHEAT_CASE_BED; break;
        #endif
      #endif
      default: bed_line = TUNE_CASE_BED + MROWS - index_tune;
    }
    if (applyEncoder(encoder_diffState, hmiValues.tempBed)) {
      encoderRate.enabled = false;
      #if HAS_PREHEAT
        if (hmiValues.show_mode == -2) {
          checkkey = ID_PLAPreheat;
          ui.material_preset[0].bed_temp = hmiValues.tempBed;
          drawEditInteger3(bed_line, ui.material_preset[0].bed_temp);
          return;
        }
        #if PREHEAT_COUNT > 1
          if (hmiValues.show_mode == -3) {
            checkkey = ID_ABSPreheat;
            ui.material_preset[1].bed_temp = hmiValues.tempBed;
            drawEditInteger3(bed_line, ui.material_preset[1].bed_temp);
            return;
          }
        #endif
      #endif
      checkkey = hmiValues.show_mode == -1 ? ID_TemperatureID : ID_Tune;
      drawEditInteger3(bed_line, hmiValues.tempBed);
      thermalManager.setTargetBed(hmiValues.tempBed);
      return;
    }
    // tempBed limit
    LIMIT(hmiValues.tempBed, BED_MINTEMP, BED_MAX_TARGET);
    // tempBed value
    drawEditInteger3(bed_line, hmiValues.tempBed, true);
  }

#endif // HAS_HEATED_BED

#if HAS_PREHEAT && HAS_FAN

  void hmiFanSpeed() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    uint8_t fan_line;
    switch (hmiValues.show_mode) {
      case -1: fan_line = TEMP_CASE_FAN; break;
      case -2: fan_line = PREHEAT_CASE_FAN; break;
      case -3: fan_line = PREHEAT_CASE_FAN; break;
      default: fan_line = TUNE_CASE_FAN + MROWS - index_tune;
    }

    if (applyEncoder(encoder_diffState, hmiValues.fanSpeed)) {
      encoderRate.enabled = false;
      if (hmiValues.show_mode == -2) {
        checkkey = ID_PLAPreheat;
        ui.material_preset[0].fan_speed = hmiValues.fanSpeed;
        drawEditInteger3(fan_line, ui.material_preset[0].fan_speed);
        return;
      }
      #if PREHEAT_COUNT > 1
        if (hmiValues.show_mode == -3) {
          checkkey = ID_ABSPreheat;
          ui.material_preset[1].fan_speed = hmiValues.fanSpeed;
          drawEditInteger3(fan_line, ui.material_preset[1].fan_speed);
          return;
        }
      #endif
      checkkey = hmiValues.show_mode == -1 ? ID_TemperatureID : ID_Tune;
      drawEditInteger3(fan_line, hmiValues.fanSpeed);
      thermalManager.set_fan_speed(0, hmiValues.fanSpeed);
      return;
    }
    // fanSpeed limit
    LIMIT(hmiValues.fanSpeed, 0, 255);
    // fanSpeed value
    drawEditInteger3(fan_line, hmiValues.fanSpeed, true);
  }

#endif // HAS_PREHEAT && HAS_FAN

void hmiPrintSpeed() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (applyEncoder(encoder_diffState, hmiValues.printSpeed)) {
    checkkey = ID_Tune;
    encoderRate.enabled = false;
    feedrate_percentage = hmiValues.printSpeed;
    drawEditInteger3(select_tune.now + MROWS - index_tune, hmiValues.printSpeed);
    return;
  }
  // printSpeed limit
  LIMIT(hmiValues.printSpeed, SPEED_EDIT_MIN, SPEED_EDIT_MAX);
  // printSpeed value
  drawEditInteger3(select_tune.now + MROWS - index_tune, hmiValues.printSpeed, true);
}

#define LAST_AXIS TERN(HAS_HOTEND, E_AXIS, Z_AXIS)

void hmiMaxFeedspeedXYZE() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (applyEncoder(encoder_diffState, hmiValues.maxFeedSpeed)) {
    checkkey = ID_MaxSpeed;
    encoderRate.enabled = false;
    if (WITHIN(hmiFlag.feedspeed_axis, X_AXIS, LAST_AXIS))
      planner.set_max_feedrate(hmiFlag.feedspeed_axis, hmiValues.maxFeedSpeed);
    drawEditInteger4(select_speed.now, hmiValues.maxFeedSpeed);
    return;
  }
  // MaxFeedspeed limit
  if (WITHIN(hmiFlag.feedspeed_axis, X_AXIS, LAST_AXIS))
    LIMIT(hmiValues.maxFeedSpeed, min_feedrate_edit_values[hmiFlag.feedspeed_axis], max_feedrate_edit_values[hmiFlag.feedspeed_axis]);
  // MaxFeedspeed value
  drawEditInteger4(select_speed.now, hmiValues.maxFeedSpeed, true);
}

void hmiMaxAccelerationXYZE() {
  EncoderState encoder_diffState = encoderReceiveAnalyze();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (applyEncoder(encoder_diffState, hmiValues.maxAcceleration)) {
    checkkey = ID_MaxAcceleration;
    encoderRate.enabled = false;
    if (WITHIN(hmiFlag.acc_axis, X_AXIS, LAST_AXIS))
      planner.set_max_acceleration(hmiFlag.acc_axis, hmiValues.maxAcceleration);
    drawEditInteger4(select_acc.now, hmiValues.maxAcceleration);
    return;
  }
  // MaxAcceleration limit
  if (WITHIN(hmiFlag.acc_axis, X_AXIS, LAST_AXIS))
    LIMIT(hmiValues.maxAcceleration, min_acceleration_edit_values[hmiFlag.acc_axis], max_acceleration_edit_values[hmiFlag.acc_axis]);
  // MaxAcceleration value
  drawEditInteger4(select_acc.now, hmiValues.maxAcceleration, true);
}

#if ENABLED(CLASSIC_JERK)

  void hmiMaxJerkXYZE() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.maxJerkScaled)) {
      checkkey = ID_MaxJerk;
      encoderRate.enabled = false;
      if (WITHIN(hmiFlag.jerk_axis, X_AXIS, LAST_AXIS))
        planner.set_max_jerk(hmiFlag.jerk_axis, hmiValues.maxJerkScaled / MINUNITMULT);
      drawEditFloat3(select_jerk.now, hmiValues.maxJerkScaled);
      return;
    }
    // MaxJerk limit
    if (WITHIN(hmiFlag.jerk_axis, X_AXIS, LAST_AXIS))
      LIMIT(hmiValues.maxJerkScaled, min_jerk_edit_values[hmiFlag.jerk_axis] * MINUNITMULT, max_jerk_edit_values[hmiFlag.jerk_axis] * MINUNITMULT);
    // MaxJerk value
    drawEditFloat3(select_jerk.now, hmiValues.maxJerkScaled, true);
  }

#endif // CLASSIC_JERK

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

  void hmiStepXYZE() {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    if (applyEncoder(encoder_diffState, hmiValues.maxStepScaled)) {
      checkkey = ID_Step;
      encoderRate.enabled = false;
      if (WITHIN(hmiFlag.step_axis, X_AXIS, LAST_AXIS))
        planner.settings.axis_steps_per_mm[hmiFlag.step_axis] = hmiValues.maxStepScaled / MINUNITMULT;
      drawEditFloat3(select_step.now, hmiValues.maxStepScaled);
      return;
    }
    // Step limit
    if (WITHIN(hmiFlag.step_axis, X_AXIS, LAST_AXIS))
      LIMIT(hmiValues.maxStepScaled, min_steps_edit_values[hmiFlag.step_axis] * MINUNITMULT, max_steps_edit_values[hmiFlag.step_axis] * MINUNITMULT);
    // Step value
    drawEditFloat3(select_step.now, hmiValues.maxStepScaled, true);
  }

#endif // EDITABLE_STEPS_PER_UNIT

// Draw X, Y, Z and blink if in an un-homed or un-trusted state
void _update_axis_value(const AxisEnum axis, const uint16_t x, const uint16_t y, const bool blink, const bool force) {
  const bool draw_qmark = axis_should_home(axis),
             draw_empty = NONE(HOME_AFTER_DEACTIVATE, DISABLE_REDUCED_ACCURACY_WARNING) && !draw_qmark && !axis_is_trusted(axis);

  // Check for a position change
  static xyz_pos_t oldpos = { -1, -1, -1 };
  const float p = current_position[axis];
  const bool changed = oldpos[axis] != p;
  if (changed) oldpos[axis] = p;

  if (force || changed || draw_qmark || draw_empty) {
    if (blink && draw_qmark)
      dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, x, y, F("???.?"));
    else if (blink && draw_empty)
      dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, x, y, F("     "));
    else
      dwinDrawFloatValue(true, true, 0, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 3, 1, x, y, p);
  }
}

void _draw_xyz_position(const bool force) {
  //SERIAL_ECHOPGM("Draw XYZ:");
  static bool _blink = false;
  const bool blink = !!(millis() & 0x400UL);
  if (force || blink != _blink) {
    _blink = blink;
    //SERIAL_ECHOPGM(" (blink)");
    _update_axis_value(X_AXIS,  35, 459, blink, true);
    _update_axis_value(Y_AXIS, 120, 459, blink, true);
    _update_axis_value(Z_AXIS, 205, 459, blink, true);
  }
  //SERIAL_EOL();
}

void updateVariable() {
  #if HAS_HOTEND
    static celsius_t _hotendtemp = 0, _hotendtarget = 0;
    const celsius_t hc = thermalManager.wholeDegHotend(0),
                    ht = thermalManager.degTargetHotend(0);
    const bool _new_hotend_temp = _hotendtemp != hc,
               _new_hotend_target = _hotendtarget != ht;
    if (_new_hotend_temp) _hotendtemp = hc;
    if (_new_hotend_target) _hotendtarget = ht;
  #endif
  #if HAS_HEATED_BED
    static celsius_t _bedtemp = 0, _bedtarget = 0;
    const celsius_t bc = thermalManager.wholeDegBed(),
                    bt = thermalManager.degTargetBed();
    const bool _new_bed_temp = _bedtemp != bc,
               _new_bed_target = _bedtarget != bt;
    if (_new_bed_temp) _bedtemp = bc;
    if (_new_bed_target) _bedtarget = bt;
  #endif
  #if HAS_FAN
    static uint8_t _fanspeed = 0;
    const bool _new_fanspeed = _fanspeed != thermalManager.fan_speed[0];
    if (_new_fanspeed) _fanspeed = thermalManager.fan_speed[0];
  #endif

  if (checkkey == ID_Tune) {
    // Tune page temperature update
    #if HAS_HOTEND
      if (_new_hotend_target)
        drawEditInteger3(TUNE_CASE_TEMP + MROWS - index_tune, _hotendtarget);
    #endif
    #if HAS_HEATED_BED
      if (_new_bed_target)
        drawEditInteger3(TUNE_CASE_BED + MROWS - index_tune, _bedtarget);
    #endif
    #if HAS_FAN
      if (_new_fanspeed)
        drawEditInteger3(TUNE_CASE_FAN + MROWS - index_tune, _fanspeed);
    #endif
  }
  else if (checkkey == ID_TemperatureID) {
    // Temperature page temperature update
    #if HAS_HOTEND
      if (_new_hotend_target) drawEditInteger3(TEMP_CASE_TEMP, _hotendtarget);
    #endif
    #if HAS_HEATED_BED
      if (_new_bed_target) drawEditInteger3(TEMP_CASE_BED, _bedtarget);
    #endif
    #if HAS_FAN
      if (_new_fanspeed) drawEditInteger3(TEMP_CASE_FAN, _fanspeed);
    #endif
  }

  // Bottom temperature update

  #if HAS_HOTEND
    if (_new_hotend_temp)
      drawStatInt(28, 384, _hotendtemp);
    if (_new_hotend_target)
      drawStatInt(25 + 4 * STAT_CHR_W + 6, 384, _hotendtarget);

    static int16_t _flow = 0;
    if (_flow != planner.flow_percentage[0]) {
      _flow = planner.flow_percentage[0];
      drawStatInt(116 + 2 * STAT_CHR_W, 417, _flow);
    }
  #endif

  #if HAS_HEATED_BED
    if (_new_bed_temp)
      drawStatInt(28, 417, _bedtemp);
    if (_new_bed_target)
      drawStatInt(25 + 4 * STAT_CHR_W + 6, 417, _bedtarget);
  #endif

  static int16_t _feedrate = 0;
  if (_feedrate != feedrate_percentage) {
    _feedrate = feedrate_percentage;
    drawStatInt(116 + 2 * STAT_CHR_W, 384, _feedrate);
  }

  #if HAS_FAN
    if (_new_fanspeed) {
      _fanspeed = thermalManager.fan_speed[0];
      drawStatInt(195 + 2 * STAT_CHR_W, 384, _fanspeed);
    }
  #endif

  static float _offset = 0;
  if (BABY_Z_VAR != _offset) {
    _offset = BABY_Z_VAR;
    if (BABY_Z_VAR < 0) {
      drawStatFloat(207, 417, -_offset);
      dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 205, 419, F("-"));
    }
    else {
      drawStatFloat(207, 417, _offset);
      dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 205, 419, F(" "));
    }
  }

  _draw_xyz_position(false);
}

/**
 * Read and cache the working directory.
 *
 * TODO: New code can follow the pattern of menu_media.cpp
 * and rely on Marlin caching for performance. No need to
 * cache files here.
 */

void make_name_without_ext(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
  char * const name = card.longest_filename();
  size_t pos        = strlen(name); // index of ending nul

  // For files, remove the extension
  // which may be .gcode, .gco, or .g
  if (!card.flag.filenameIsDir)
    while (pos && src[pos] != '.') pos--; // find last '.' (stop at 0)

  size_t len = pos;   // nul or '.'
  if (len > maxlen) { // Keep the name short
    pos        = len = maxlen; // move nul down
    dst[--pos] = '.'; // insert dots
    dst[--pos] = '.';
    dst[--pos] = '.';
  }

  dst[len] = '\0';    // end it

  // Copy down to 0
  while (pos--) dst[pos] = src[pos];
}

void hmiSDCardInit() { card.cdroot(); }

// Initialize or re-initialize the LCD
void MarlinUI::init_lcd() { dwinStartup(); }

void MarlinUI::clear_lcd() {}

void MarlinUI::update() {
  eachMomentUpdate(); // Status update
  hmiSDCardUpdate();  // SD card update
  dwinHandleScreen(); // Rotary encoder update
}

#if HAS_LCD_BRIGHTNESS
  void MarlinUI::_set_brightness() { dwinLCDBrightness(backlight ? brightness : 0); }
#endif

void MarlinUI::kill_screen(FSTR_P const lcd_error, FSTR_P const) {
  clearMainWindow();
  drawPopupBkgd60();
  dwinIconShow(ICON, ICON_Printer_0, 101, 105);
  dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 15) / 2, 230, GET_TEXT_F(MSG_PRINTER_KILLED));
  dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, (272 - 8 * 20) / 2, 260, GET_TEXT_F(MSG_TURN_OFF));
}

#if ENABLED(SCROLL_LONG_FILENAMES)

  char shift_name[LONG_FILENAME_LENGTH + 1];
  int8_t shift_amt; // = 0
  millis_t shift_ms; // = 0

  // Init the shift name based on the highlighted item
  void initShiftName() {
    const bool is_subdir = !card.flag.workDirIsRoot;
    const int8_t filenum = select_file.now - 1 - is_subdir; // Skip "Back" and ".."
    const int16_t fileCnt = card.get_num_items();
    if (WITHIN(filenum, 0, fileCnt - 1)) {
      card.selectFileByIndexSorted(filenum);
      char * const name = card.longest_filename();
      make_name_without_ext(shift_name, name, 100);
    }
  }

  void initSDItemShift() {
    shift_amt = 0;
    shift_ms  = select_file.now != CASE_BACK && strlen(shift_name) > MENU_CHAR_LIMIT
           ? millis() + 750UL : 0;
  }

#endif

/**
 * Display an SD item, adding a CDUP for subfolders.
 */
void drawSDItem(const uint16_t item, int16_t row=-1) {
  if (row < 0) row = item + 1 + MROWS - index_file;
  const bool is_subdir = !card.flag.workDirIsRoot;
  if (is_subdir && item == 0) {
    drawMenuLine(row, ICON_Folder, F(".."));
    return;
  }

  card.selectFileByIndexSorted(item - is_subdir);
  char * const name = card.longest_filename();

  #if ENABLED(SCROLL_LONG_FILENAMES)
    // Init the current selected name
    // This is used during scroll drawing
    if (item == select_file.now - 1) {
      make_name_without_ext(shift_name, name, 100);
      initSDItemShift();
    }
  #endif

  // Draw the file/folder with name aligned left
  char str[strlen(name) + 1];
  make_name_without_ext(str, name);
  drawMenuLine(row, card.flag.filenameIsDir ? ICON_Folder : ICON_File, str);
}

#if ENABLED(SCROLL_LONG_FILENAMES)

  void drawSDItemShifted(uint8_t &shift) {
    // Limit to the number of chars past the cutoff
    const size_t len = strlen(shift_name);
    NOMORE(shift, _MAX(len - MENU_CHAR_LIMIT, 0U));

    // Shorten to the available space
    const size_t lastchar = _MIN((signed)len, shift + MENU_CHAR_LIMIT);

    const char c = shift_name[lastchar];
    shift_name[lastchar] = '\0';

    const uint8_t row = select_file.now + MROWS - index_file; // skip "Back" and scroll
    eraseMenuText(row);
    drawMenuLine(row, 0, &shift_name[shift]);

    shift_name[lastchar] = c;
  }

#endif

// Redraw the first set of SD Files
void redrawSDList() {
  select_file.reset();
  index_file = MROWS;

  clearMenuArea(); // Leave title bar unchanged

  drawBackFirst();

  if (card.isMounted()) {
    // As many files as will fit
    for (uint8_t i = 0; i < _MIN(nr_sd_menu_items(), MROWS); ++i)
      drawSDItem(i, i + 1);

    TERN_(SCROLL_LONG_FILENAMES, initSDItemShift());
  }
  else {
    dwinDrawRectangle(1, COLOR_BG_RED, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
    dwinDrawString(false, font16x32, COLOR_YELLOW, COLOR_BG_RED, ((DWIN_WIDTH) - 8 * 16) / 2, MBASE(3), F("No Media"));
  }
}

bool DWIN_lcd_sd_status = false;

void sdCardUp() {
  card.cdup();
  redrawSDList();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

void sdCardFolder(char * const dirname) {
  card.cd(dirname);
  redrawSDList();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

//
// Watch for media mount / unmount
//
void hmiSDCardUpdate() {
  if (hmiFlag.home_flag) return;
  if (DWIN_lcd_sd_status != card.isMounted()) {
    DWIN_lcd_sd_status = card.isMounted();
    //SERIAL_ECHOLNPGM("HMI_SDCardUpdate: ", DWIN_lcd_sd_status);
    if (DWIN_lcd_sd_status) {
      if (checkkey == ID_SelectFile)
        redrawSDList();
    }
    else {
      // clean file icon
      if (checkkey == ID_SelectFile) {
        redrawSDList();
      }
      else if (checkkey == ID_PrintProcess || checkkey == ID_Tune || printingIsActive()) {
        // TODO: Move card removed abort handling
        //       to CardReader::manage_media.
        card.abortFilePrintSoon();
        wait_for_heatup = wait_for_user = false;
        dwin_abort_flag = true; // Reset feedrate, return to Home
      }
    }
    dwinUpdateLCD();
  }
}

//
// The status area is always on-screen, except during
// full-screen modal dialogs. (TODO: Keep alive during dialogs)
//
void drawStatusArea(const bool with_update) {

  dwinDrawRectangle(1, COLOR_BG_BLACK, 0, STATUS_Y, DWIN_WIDTH, DWIN_HEIGHT - 1);

  #if HAS_HOTEND
    dwinIconShow(ICON, ICON_HotendTemp, 10, 383);
    drawStatInt(28, 384, thermalManager.wholeDegHotend(0));
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 25 + 3 * STAT_CHR_W + 5, 384, F("/"));
    drawStatInt(25 + 4 * STAT_CHR_W + 6, 384, thermalManager.degTargetHotend(0));

    dwinIconShow(ICON, ICON_StepE, 112, 417);
    drawStatInt(116 + 2 * STAT_CHR_W, 417, planner.flow_percentage[0]);
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 116 + 5 * STAT_CHR_W + 2, 417, F("%"));
  #endif

  #if HAS_HEATED_BED
    dwinIconShow(ICON, ICON_BedTemp, 10, 416);
    drawStatInt(28, 417, thermalManager.wholeDegBed());
    dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 25 + 3 * STAT_CHR_W + 5, 417, F("/"));
    drawStatInt(25 + 4 * STAT_CHR_W + 6, 417, thermalManager.degTargetBed());
  #endif

  dwinIconShow(ICON, ICON_Speed, 113, 383);
  drawStatInt(116 + 2 * STAT_CHR_W, 384, feedrate_percentage);
  dwinDrawString(false, DWIN_FONT_STAT, COLOR_WHITE, COLOR_BG_BLACK, 116 + 5 * STAT_CHR_W + 2, 384, F("%"));

  #if HAS_FAN
    dwinIconShow(ICON, ICON_FanSpeed, 187, 383);
    drawStatInt(195 + 2 * STAT_CHR_W, 384, thermalManager.fan_speed[0]);
  #endif

  #if HAS_ZOFFSET_ITEM
    dwinIconShow(ICON, ICON_Zoffset, 187, 416);
  #endif

  if (BABY_Z_VAR < 0) {
    drawStatFloat(207, 417, -BABY_Z_VAR * 100);
    dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 205, 419, F("-"));
  }
  else {
    drawStatFloat(207, 417, BABY_Z_VAR * 100);
    dwinDrawString(true, font8x16, COLOR_WHITE, COLOR_BG_BLACK, 205, 419, F(" "));
  }

  dwinDrawRectangle(1, COLOR_LINE, 0, 449, DWIN_WIDTH, 451);

  dwinIconShow(ICON, ICON_MaxSpeedX,  10, 456);
  dwinIconShow(ICON, ICON_MaxSpeedY,  95, 456);
  dwinIconShow(ICON, ICON_MaxSpeedZ, 180, 456);
  _draw_xyz_position(true);

  if (with_update) {
    dwinUpdateLCD();
    delay(5);
  }
}

void hmiStartFrame(const bool with_update) {
  gotoMainMenu();
  drawStatusArea(with_update);
}

void drawInfoMenu() {
  clearMainWindow();

  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, (DWIN_WIDTH - strlen(MACHINE_SIZE) * MENU_CHR_W) / 2, 122, F(MACHINE_SIZE));
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, (DWIN_WIDTH - strlen(SHORT_BUILD_VERSION) * MENU_CHR_W) / 2, 195, F(SHORT_BUILD_VERSION));

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(30, 17, 28, 13);                   // "Info"

    dwinFrameAreaCopy(1, 197, 149, 252, 161, 108, 102);   // "Size"
    dwinFrameAreaCopy(1,   1, 164,  56, 176, 108, 175);   // "Firmware Version"
    dwinFrameAreaCopy(1,  58, 164, 113, 176, 105, 248);   // "Contact Details"
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_INFO_SCREEN));
    #else
      dwinFrameTitleCopy(192, 15, 23, 12);                // "Info"
    #endif

    dwinFrameAreaCopy(1, 120, 150, 146, 161, 124, 102);   // "Size"
    dwinFrameAreaCopy(1, 146, 151, 254, 161,  82, 175);   // "Firmware Version"
    dwinFrameAreaCopy(1,   1, 164,  96, 175,  89, 248);   // "Contact details"
  }
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLACK, (DWIN_WIDTH - strlen(CORP_WEBSITE) * MENU_CHR_W) / 2, 268, F(CORP_WEBSITE));

  drawBackFirst();
  for (uint8_t i = 0; i < 3; ++i) {
    dwinIconShow(ICON, ICON_PrintSize + i, 26, 99 + i * 73);
    dwinDrawLine(COLOR_LINE, 16, MBASE(2) + i * 73, 256, 156 + i * 73);
  }
}

void drawPrintFileMenu() {
  clearTitleBar();

  if (hmiIsChinese())
    dwinFrameTitleCopy(0, 31, 56, 14);  // "Print file"
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_MEDIA_MENU));
    #else
      dwinFrameTitleCopy(52, 31, 86, 11); // "Print file"
    #endif
  }

  redrawSDList();
}

// Main Process
void hmiMainMenu() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_page.inc(4)) {
      switch (select_page.now) {
        case PAGE_PRINT: iconPrint(); break;
        case PAGE_PREPARE: iconPrint(); iconPrepare(); break;
        case PAGE_CONTROL: iconPrepare(); iconControl(); break;
        case PAGE_INFO_LEVELING: iconControl(); TERN(HAS_ONESTEP_LEVELING, iconLeveling, iconStartInfo)(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_page.dec()) {
      switch (select_page.now) {
        case PAGE_PRINT: iconPrint(); iconPrepare(); break;
        case PAGE_PREPARE: iconPrepare(); iconControl(); break;
        case PAGE_CONTROL: iconControl(); TERN(HAS_ONESTEP_LEVELING, iconLeveling, iconStartInfo)(); break;
        case PAGE_INFO_LEVELING: TERN(HAS_ONESTEP_LEVELING, iconLeveling, iconStartInfo)(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_page.now) {
      case PAGE_PRINT:
        checkkey = ID_SelectFile;
        drawPrintFileMenu();
        break;

      case PAGE_PREPARE:
        checkkey = ID_Prepare;
        select_prepare.reset();
        index_prepare = MROWS;
        drawPrepareMenu();
        break;

      case PAGE_CONTROL:
        checkkey = ID_Control;
        select_control.reset();
        index_control = MROWS;
        drawControlMenu();
        break;

      case PAGE_INFO_LEVELING:
        #if HAS_ONESTEP_LEVELING
          checkkey = ID_Leveling;
          hmiLeveling();
        #else
          checkkey = ID_Info;
          drawInfoMenu();
        #endif
        break;
    }
  }
  dwinUpdateLCD();
}

// Select (and Print) File
void hmiSelectFile() {
  EncoderState encoder_diffState = get_encoder_state();

  const uint16_t hasUpDir = !card.flag.workDirIsRoot;

  if (encoder_diffState == ENCODER_DIFF_NO) {
    #if ENABLED(SCROLL_LONG_FILENAMES)
      if (shift_ms && select_file.now >= 1 + hasUpDir) {
        // Scroll selected filename every second
        const millis_t ms = millis();
        if (ELAPSED(ms, shift_ms)) {
          const bool was_reset = shift_amt < 0;
          shift_ms = ms + 375UL + was_reset * 250UL;  // ms per character
          uint8_t shift_new = shift_amt + 1;          // Try to shift by...
          drawSDItemShifted(shift_new);               // Draw the item
          if (!was_reset && shift_new == 0)           // Was it limited to 0?
            shift_ms = 0;                             // No scrolling needed
          else if (shift_new == shift_amt)            // Scroll reached the end
            shift_new = -1;                           // Reset
          shift_amt = shift_new;                      // Set new scroll
        }
      }
    #endif
    return;
  }

  // First pause is long. Easy.
  // On reset, long pause must be after 0.

  const uint16_t fullCnt = nr_sd_menu_items();

  if (encoder_diffState == ENCODER_DIFF_CW && fullCnt) {
    if (select_file.inc(1 + fullCnt)) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        eraseMenuText(itemnum + MROWS - index_file);            // Erase and
        drawSDItem(itemnum - 1);                                // redraw
      }
      if (select_file.now > MROWS && select_file.now > index_file) { // Cursor past the bottom
        index_file = select_file.now;                           // New bottom line
        scrollMenu(DWIN_SCROLL_UP);
        drawSDItem(itemnum, MROWS);                             // Draw and init the shift name
      }
      else {
        moveHighlight(1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, initShiftName());          // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, initSDItemShift());
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW && fullCnt) {
    if (select_file.dec()) {
      const uint8_t itemnum = select_file.now - 1;              // -1 for "Back"
      if (TERN0(SCROLL_LONG_FILENAMES, shift_ms)) {             // If line was shifted
        eraseMenuText(select_file.now + 1 + MROWS - index_file); // Erase and
        drawSDItem(itemnum + 1);                                // redraw
      }
      if (select_file.now < index_file - MROWS) {               // Cursor past the top
        index_file--;                                           // New bottom line
        scrollMenu(DWIN_SCROLL_DOWN);
        if (index_file == MROWS) {
          drawBackFirst();
          TERN_(SCROLL_LONG_FILENAMES, shift_ms = 0);
        }
        else
          drawSDItem(itemnum, 0);                               // Draw the item (and init shift name)
      }
      else {
        moveHighlight(-1, select_file.now + MROWS - index_file); // Just move highlight
        TERN_(SCROLL_LONG_FILENAMES, initShiftName());          // ...and init the shift name
      }
      TERN_(SCROLL_LONG_FILENAMES, initSDItemShift());        // Reset left. Init timer.
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_file.now == CASE_BACK) { // Back
      select_page.set(0);
      gotoMainMenu();
    }
    else if (hasUpDir && select_file.now == 1) { // CD-Up
      sdCardUp();
      goto HMI_SelectFileExit;
    }
    else {
      const uint16_t filenum = select_file.now - 1 - hasUpDir;
      card.selectFileByIndexSorted(filenum);

      // Enter that folder!
      if (card.flag.filenameIsDir) {
        sdCardFolder(card.filename);
        goto HMI_SelectFileExit;
      }

      // Reset highlight for next entry
      select_print.reset();
      select_file.reset();

      // Start choice and print SD file
      hmiFlag.heat_flag = true;
      hmiFlag.print_finish = false;
      hmiValues.show_mode = 0;

      card.openAndPrintFile(card.filename);

      #if HAS_FAN
        // All fans on for Ender-3 v2 ?
        // The slicer should manage this for us.
        //for (uint8_t i = 0; i < FAN_COUNT; i++)
        //  thermalManager.fan_speed[i] = 255;
      #endif

      _card_percent = 0;
      _remain_time = 0;
      gotoPrintProcess();
    }
  }

  HMI_SelectFileExit:
    dwinUpdateLCD();
}

// Printing
void hmiPrinting() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (hmiFlag.done_confirm_flag) {
    if (encoder_diffState == ENCODER_DIFF_ENTER) {
      hmiFlag.done_confirm_flag = false;
      dwin_abort_flag = true; // Reset feedrate, return to Home
    }
    return;
  }

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_print.inc(3)) {
      switch (select_print.now) {
        case PRINT_SETUP: iconTune(); break;
        case PRINT_PAUSE_RESUME: iconTune(); iconResumeOrPause(); break;
        case PRINT_STOP: iconResumeOrPause(); iconStop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_print.dec()) {
      switch (select_print.now) {
        case PRINT_SETUP: iconTune(); iconResumeOrPause(); break;
        case PRINT_PAUSE_RESUME: iconResumeOrPause(); iconStop(); break;
        case PRINT_STOP: iconStop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_print.now) {
      case PRINT_SETUP:
        checkkey = ID_Tune;
        hmiValues.show_mode = 0;
        select_tune.reset();
        index_tune = MROWS;
        drawTuneMenu();
        break;

      case PRINT_PAUSE_RESUME:
        if (hmiFlag.pause_flag) {
          iconPause();

          char cmd[40];
          cmd[0] = '\0';

          #if ALL(HAS_HEATED_BED, PAUSE_HEAT)
            if (resume_bed_temp) sprintf_P(cmd, PSTR("M190 S%i\n"), resume_bed_temp);
          #endif
          #if ALL(HAS_HOTEND, PAUSE_HEAT)
            if (resume_hotend_temp) sprintf_P(&cmd[strlen(cmd)], PSTR("M109 S%i\n"), resume_hotend_temp);
          #endif

          strcat_P(cmd, M24_STR);
          queue.inject(cmd);
        }
        else {
          hmiFlag.select_flag = true;
          checkkey = ID_PrintWindow;
          popupwindowPauseOrStop();
        }
        break;

      case PRINT_STOP:
        hmiFlag.select_flag = true;
        checkkey = ID_PrintWindow;
        popupwindowPauseOrStop();
        break;

      default: break;
    }
  }
  dwinUpdateLCD();
}

// Pause and Stop window
void hmiPauseOrStop() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW)
    drawSelectHighlight(false);
  else if (encoder_diffState == ENCODER_DIFF_CCW)
    drawSelectHighlight(true);
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (select_print.now == PRINT_PAUSE_RESUME) {
      if (hmiFlag.select_flag) {
        hmiFlag.pause_action = true;
        queue.inject(F("M25"));
      }
      gotoPrintProcess();
    }
    else if (select_print.now == PRINT_STOP) {
      if (hmiFlag.select_flag) {
        checkkey = ID_BackMain;
        wait_for_heatup = wait_for_user = false;      // Stop waiting for heating/user
        card.abortFilePrintSoon();                    // Let the main loop handle SD abort
        dwin_abort_flag = true;                       // Reset feedrate, return to Home
        #ifdef ACTION_ON_CANCEL
          hostui.cancel();
        #endif
        popupWindowHome(true);
        if (hmiFlag.home_flag) planner.synchronize(); // Wait for planner moves to finish!
      }
      else
        gotoPrintProcess(); // cancel stop
    }
  }
  dwinUpdateLCD();
}

void drawMoveMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(192, 1, 42, 14); // "Move"
    itemAreaCopy(58, 118, 106, 132, 1);
    itemAreaCopy(109, 118, 157, 132, 2);
    itemAreaCopy(160, 118, 209, 132, 3);
    TERN_(HAS_HOTEND, itemAreaCopy(212, 118, 253, 131, 4));
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_MOVE_AXIS));
    #else
      dwinFrameTitleCopy(231, 2, 35, 11);   // "Move"
    #endif

    #ifdef USE_STRING_TITLES
      dwinDrawLabel(1, GET_TEXT_F(MSG_MOVE_X));
      dwinDrawLabel(2, GET_TEXT_F(MSG_MOVE_Y));
      dwinDrawLabel(3, GET_TEXT_F(MSG_MOVE_Z));
      TERN_(HAS_HOTEND, dwinDrawLabel(4, GET_TEXT_F(MSG_MOVE_E)));
    #else
      say_move_en(1); say_x_en(38, 1); // "Move X"
      say_move_en(2); say_y_en(38, 2); // "Move Y"
      say_move_en(3); say_z_en(38, 3); // "Move Z"
      TERN_(HAS_HOTEND, (say_move_en(4), itemAreaCopy(99, 194, 151, 204, 4, 38))); // "Move Extruder"
    #endif
  }

  drawBackFirst(select_axis.now == CASE_BACK);
  if (select_axis.now != CASE_BACK) drawMenuCursor(select_axis.now);

  // Draw separators and icons
  for (uint8_t i = 0; i < 3 + ENABLED(HAS_HOTEND); ++i) drawMenuLine(i + 1, ICON_MoveX + i);
}

void itemAdvHomeOffsets(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Set Home Offsets"
  }
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, GET_TEXT_F(MSG_SET_HOME_OFFSETS));
    #else
      itemAreaCopy(1,  76, 102,  87, row); // "Set Home Offsets"
    #endif
  }
  drawMenuLine(row, ICON_HomeOffset);
  drawMoreIcon(row);
}

#if HAS_ONESTEP_LEVELING

  void itemAdvProbeOffsets(const uint8_t row) {
    if (false && hmiIsChinese()) {
      // TODO: Chinese "Probe Offsets"
    }
    else {
      #ifdef USE_STRING_TITLES
        dwinDrawLabel(row, GET_TEXT_F(MSG_ZPROBE_OFFSETS));
      #else
        say_probe_offs_en(row);
      #endif
    }
    drawMenuLine(row, ICON_ProbeOffset);
    drawMoreIcon(row);
  }

#endif

void itemAdvHotendPID(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Hotend PID"
  }
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, F("Hotend PID"));
    #else
      itemAreaCopy(96, 104, 167, 114, row); // "Hotend PID"
    #endif
  }
  drawMenuLine(row, ICON_PIDNozzle);
}

void itemAdvBedPID(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Bed PID"
  }
  else {
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(row, F("Bed PID"));
    #else
      itemAreaCopy(241, 104, 263, 115, row);     // "Bed"
      itemAreaCopy(145, 104, 167, 114, row, 27); // "PID"
    #endif
  }
  drawMenuLine(row, ICON_PIDBed);
}

#if ENABLED(POWER_LOSS_RECOVERY)

  void itemAdv_PLR(const uint8_t row) {
    if (false && hmiIsChinese()) {
      // TODO: Chinese "Power-loss Recovery"
    }
    else {
      #ifdef USE_STRING_TITLES
        dwinDrawLabel(row, GET_TEXT_F(MSG_ZPROBE_OFFSETS));
      #else
        itemAreaCopy(1, 208, 137, 221, row);  // "Power-loss Recovery"
      #endif
    }
    drawMenuLine(row, ICON_Motion);
    drawCheckboxLine(row, recovery.enabled);
  }

#endif

void drawAdvancedSettingsMenu() {
  clearMainWindow();

  #if ADVSET_CASE_TOTAL >= TROWS
    const int16_t scroll = MROWS - index_advset; // Scrolled-up lines
  #else
    constexpr int16_t scroll = 0;
  #endif
  #define ASCROL(L) (scroll + (L))
  #define AVISI(L) VISI(ADVSET_CASE_TOTAL, L, ASCROL(L))

  if (false && hmiIsChinese()) {
    // TODO: Chinese "Advanced Settings"
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_ADVANCED_SETTINGS));
    #else
      dwinFrameTitleCopy(93, 401, 126, 15); // "Advanced Settings"
    #endif
  }

  if (AVISI(0)) drawBackFirst(select_advset.now == CASE_BACK);
  if (AVISI(ADVSET_CASE_HOMEOFF)) itemAdvHomeOffsets(ASCROL(ADVSET_CASE_HOMEOFF));      // Set Home Offsets >
  #if HAS_ONESTEP_LEVELING
    if (AVISI(ADVSET_CASE_PROBEOFF)) itemAdvProbeOffsets(ASCROL(ADVSET_CASE_PROBEOFF)); // Probe Offsets >
  #endif
  if (AVISI(ADVSET_CASE_HEPID)) itemAdvHotendPID(ASCROL(ADVSET_CASE_HEPID));            // Nozzle PID
  if (AVISI(ADVSET_CASE_BEDPID)) itemAdvBedPID(ASCROL(ADVSET_CASE_BEDPID));             // Bed PID
  #if ENABLED(POWER_LOSS_RECOVERY)
    if (AVISI(ADVSET_CASE_PWRLOSSR)) itemAdv_PLR(ASCROL(ADVSET_CASE_PWRLOSSR));          // Power-loss recovery
  #endif
  if (select_advset.now != CASE_BACK) drawMenuCursor(ASCROL(select_advset.now));
}

void itemHomeOffs_X(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Home Offset X"
  }
  else {
    #ifdef USE_STRING_TITLES
      drawMenuLine(row, ICON_HomeOffsetX, GET_TEXT_F(MSG_HOME_OFFSET_X));
    #else
      say_home_offs_en(row); say_x_en(75, row);   // "Home Offset X"
    #endif
  }
  drawMenuLine(row, ICON_HomeOffset);
  drawEditSignedFloat3(row, hmiValues.homeOffsScaled.x);
}

void itemHomeOffs_Y(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Home Offset Y"
  }
  else {
    #ifdef USE_STRING_TITLES
      drawMenuLine(row, ICON_HomeOffsetY, GET_TEXT_F(MSG_HOME_OFFSET_Y));
    #else
      say_home_offs_en(row); say_y_en(75, row);   // "Home Offset X"
    #endif
  }
  drawMenuLine(row, ICON_HomeOffset);
  drawEditSignedFloat3(row, hmiValues.homeOffsScaled.y);
}

void itemHomeOffs_Z(const uint8_t row) {
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Home Offset Z"
  }
  else {
    #ifdef USE_STRING_TITLES
      drawMenuLine(row, ICON_HomeOffsetZ, GET_TEXT_F(MSG_HOME_OFFSET_Z));
    #else
      say_home_offs_en(row); say_z_en(75, row);   // "Home Offset Z"
    #endif
  }
  drawMenuLine(row, ICON_HomeOffset);
  drawEditSignedFloat3(row, hmiValues.homeOffsScaled.z);
}

void drawHomeOffMenu() {
  clearMainWindow();
  if (false && hmiIsChinese()) {
    // TODO: Chinese "Home Offsets"
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_SET_HOME_OFFSETS));
    #else
      dwinFrameTitleCopy(1, 401, 91, 12);         // "Home Offsets"
    #endif
  }
  drawBackFirst(select_item.now == CASE_BACK);
  itemHomeOffs_X(1);                               // "Home Offset X"
  itemHomeOffs_Y(2);                               // "Home Offset Y"
  itemHomeOffs_Z(3);                               // "Home Offset Z"
  if (select_item.now != CASE_BACK) drawMenuCursor(select_item.now);
}

#if HAS_ONESTEP_LEVELING

  void drawProbeOffMenu() {
    clearMainWindow();
    drawBackFirst(select_item.now == CASE_BACK);
    if (false && hmiIsChinese()) {
      // TODO: Chinese "Probe Offsets"
    }
    else {
      #ifdef USE_STRING_HEADINGS
        drawTitle(GET_TEXT_F(MSG_ZPROBE_OFFSETS));
      #else
        dwinFrameTitleCopy(124, 431, 91, 12);                             // "Probe Offsets"
      #endif
      #ifdef USE_STRING_TITLES
        drawMenuLine(1, ICON_ProbeOffsetX, GET_TEXT_F(MSG_ZPROBE_XOFFSET));  // Probe X Offset
        drawMenuLine(2, ICON_ProbeOffsetY, GET_TEXT_F(MSG_ZPROBE_YOFFSET));  // Probe Y Offset
      #else
        say_probe_offs_en(1); say_x_en(75, 1);  // "Probe Offset X"
        say_probe_offs_en(2); say_y_en(75, 2);  // "Probe Offset Y"
      #endif
    }

    drawEditSignedFloat3(1, hmiValues.probeOffsScaled.x);
    drawEditSignedFloat3(2, hmiValues.probeOffsScaled.y);

    if (select_item.now != CASE_BACK) drawMenuCursor(select_item.now);
  }

#endif

#include "../../../libs/buzzer.h"

void hmiAudioFeedback(const bool success=true) {
  if (success) {
    BUZZ(100, 659);
    BUZZ(10, 0);
    BUZZ(100, 698);
  }
  else
    BUZZ(40, 440);
}

// Prepare
void hmiPrepare() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_prepare.inc(1 + PREPARE_CASE_TOTAL)) {
      if (select_prepare.now > MROWS && select_prepare.now > index_prepare) {
        index_prepare = select_prepare.now;

        // Scroll up and draw a blank bottom line
        scrollMenu(DWIN_SCROLL_UP);
        drawMenuIcon(MROWS, ICON_Axis + select_prepare.now - 1);

        // Draw "More" icon for sub-menus
        if (index_prepare < 7) drawMoreIcon(MROWS - index_prepare + 1);

        #if PREHEAT_COUNT > 1
          if (index_prepare == PREPARE_CASE_ABS) itemPrepare_ABS(MROWS);
        #endif
        #if HAS_HOTEND || HAS_HEATED_BED
          if (index_prepare == PREPARE_CASE_COOL) itemPrepareCool(MROWS);
        #endif
        if (index_prepare == PREPARE_CASE_LANG) itemPrepareLang(MROWS);
      }
      else {
        moveHighlight(1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_prepare.dec()) {
      if (select_prepare.now < index_prepare - MROWS) {
        index_prepare--;
        scrollMenu(DWIN_SCROLL_DOWN);

        if (index_prepare == MROWS)
          drawBackFirst();
        else
          drawMenuLine(0, ICON_Axis + select_prepare.now - 1);

        if (index_prepare < 7) drawMoreIcon(MROWS - index_prepare + 1);

             if (index_prepare == 6) itemPrepareMove(0);
        else if (index_prepare == 7) itemPrepareDisable(0);
        else if (index_prepare == 8) itemPrepareHome(0);
      }
      else {
        moveHighlight(-1, select_prepare.now + MROWS - index_prepare);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_prepare.now) {
      case CASE_BACK:
        select_page.set(1);
        gotoMainMenu();
        break;
      case PREPARE_CASE_MOVE:
        checkkey = ID_AxisMove;
        select_axis.reset();
        drawMoveMenu();

        drawEditFloat3(1, current_position.x * MINUNITMULT);
        drawEditFloat3(2, current_position.y * MINUNITMULT);
        drawEditFloat3(3, current_position.z * MINUNITMULT);
        #if HAS_HOTEND
          hmiValues.moveScaled.e = current_position.e * MINUNITMULT;
          drawEditSignedFloat3(4, hmiValues.moveScaled.e);
        #endif
        break;

      case PREPARE_CASE_DISA: queue.inject(F("M84")); break;

      case PREPARE_CASE_HOME: // Homing
        checkkey = ID_LastPrepare;
        index_prepare = MROWS;
        queue.inject_P(G28_STR); // G28 will set home_flag
        popupWindowHome();
        break;

      #if HAS_ZOFFSET_ITEM
        case PREPARE_CASE_ZOFF:
          #if ANY(HAS_BED_PROBE, BABYSTEPPING)
            checkkey = ID_HomeOffset;
            hmiValues.show_mode = -4;
            hmiValues.offset_value = BABY_Z_VAR * 100;
            drawEditSignedFloat2(PREPARE_CASE_ZOFF + MROWS - index_prepare, hmiValues.offset_value, true);
            encoderRate.enabled = true;
          #else
            // Apply workspace offset, making the current position 0,0,0
            queue.inject(F("G92X0Y0Z0"));
            hmiAudioFeedback();
          #endif
          break;
      #endif

      #if HAS_PREHEAT
        case PREPARE_CASE_PLA: ui.preheat_all(0); break;
        #if PREHEAT_COUNT > 1
          case PREPARE_CASE_ABS: ui.preheat_all(1); break;
        #endif
      #endif

      #if HAS_HOTEND || HAS_HEATED_BED
        case PREPARE_CASE_COOL:
          thermalManager.cooldown();
          ui.reset_status();
          break;
      #endif

      case PREPARE_CASE_LANG:
        hmiToggleLanguage();
        drawPrepareMenu();
        break;

      default: break;
    }
  }
  dwinUpdateLCD();
}

void drawTemperatureMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(236, 2, 28, 12); // "Temperature"
    #if HAS_HOTEND
      itemAreaCopy(1, 134, 56, 146, TEMP_CASE_TEMP);
    #endif
    #if HAS_HEATED_BED
      itemAreaCopy(58, 134, 113, 146, TEMP_CASE_BED);
    #endif
    #if HAS_FAN
      itemAreaCopy(115, 134, 170, 146, TEMP_CASE_FAN);
    #endif
    #if HAS_PREHEAT
      itemAreaCopy(100, 89, 178, 101, TEMP_CASE_PLA);
      #if PREHEAT_COUNT > 1
        itemAreaCopy(180, 89, 260, 100, TEMP_CASE_ABS);
      #endif
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_TEMPERATURE));
    #else
      dwinFrameTitleCopy(56, 15, 85, 14);   // "Temperature"
    #endif
    #ifdef USE_STRING_TITLES
      #if HAS_HOTEND
        dwinDrawLabel(TEMP_CASE_TEMP, GET_TEXT_F(MSG_UBL_SET_TEMP_HOTEND));
      #endif
      #if HAS_HEATED_BED
        dwinDrawLabel(TEMP_CASE_BED, GET_TEXT_F(MSG_UBL_SET_TEMP_BED));
      #endif
      #if HAS_FAN
        dwinDrawLabel(TEMP_CASE_FAN, GET_TEXT_F(MSG_FAN_SPEED));
      #endif
      #if HAS_PREHEAT
        dwinDrawLabel(TEMP_CASE_PLA, F(PREHEAT_1_LABEL " Preheat Settings"));
        #if PREHEAT_COUNT > 1
          dwinDrawLabel(TEMP_CASE_ABS, F(PREHEAT_2_LABEL " Preheat Settings"));
        #endif
      #endif
    #else
      #if HAS_HOTEND
        itemAreaCopy(197, 104, 238, 114, TEMP_CASE_TEMP);      // "Nozzle"
        itemAreaCopy(1,  89,  83, 101, TEMP_CASE_TEMP, 44);    // "Temperature"
      #endif
      #if HAS_HEATED_BED
        itemAreaCopy(240, 104, 264, 114, TEMP_CASE_BED);       // "Bed"
        itemAreaCopy(1,  89,  83, 101, TEMP_CASE_BED, 27);     // "Temperature"
      #endif
      #if HAS_FAN
        itemAreaCopy(  1, 119,  61, 132, TEMP_CASE_FAN);       // "Fan speed"
      #endif
      #if HAS_PREHEAT
        itemAreaCopy(107,  76, 156,  86, TEMP_CASE_PLA);       // "Preheat"
        say_pla_en(52, TEMP_CASE_PLA);                          // "PLA"
        itemAreaCopy(150, 135, 202, 148, TEMP_CASE_PLA, 79);   // "Settings"
        #if PREHEAT_COUNT > 1
          itemAreaCopy(107,  76, 156,  86, TEMP_CASE_ABS);     // "Preheat"
          say_abs_en(52, TEMP_CASE_ABS);                        // "ABS"
          itemAreaCopy(150, 135, 202, 148, TEMP_CASE_ABS, 81); // "Settings"
        #endif
      #endif
    #endif
  }

  drawBackFirst(select_temp.now == CASE_BACK);
  if (select_temp.now != CASE_BACK) drawMenuCursor(select_temp.now);

  // Draw icons and lines
  uint8_t i = 0;
  #define _TMENU_ICON(N) drawMenuLine(++i, ICON_SetEndTemp + (N) - 1)
  #if HAS_HOTEND
    _TMENU_ICON(TEMP_CASE_TEMP);
    drawEditInteger3(i, thermalManager.degTargetHotend(0));
  #endif
  #if HAS_HEATED_BED
    _TMENU_ICON(TEMP_CASE_BED);
    drawEditInteger3(i, thermalManager.degTargetBed());
  #endif
  #if HAS_FAN
    _TMENU_ICON(TEMP_CASE_FAN);
    drawEditInteger3(i, thermalManager.fan_speed[0]);
  #endif
  #if HAS_PREHEAT
    // PLA/ABS items have submenus
    _TMENU_ICON(TEMP_CASE_PLA); drawMoreIcon(i);
    #if PREHEAT_COUNT > 1
      _TMENU_ICON(TEMP_CASE_ABS); drawMoreIcon(i);
    #endif
  #endif
}

// Control
void hmiControl() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_control.inc(1 + CONTROL_CASE_TOTAL)) {
      if (select_control.now > MROWS && select_control.now > index_control) {
        index_control = select_control.now;

        // Scroll up and draw a blank bottom line
        scrollMenu(DWIN_SCROLL_UP);

        switch (index_control) {  // Last menu items
          case CONTROL_CASE_ADVSET: itemControlAdvanced(MROWS); break;
          case CONTROL_CASE_INFO:   itemControlInfo(MROWS);     break;
          default: break;
        }

      }
      else
        moveHighlight(1, select_control.now + MROWS - index_control);
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_control.dec()) {
      if (select_control.now < index_control - MROWS) {
        index_control--;
        scrollMenu(DWIN_SCROLL_DOWN);
        switch (index_control) {  // First menu items
          case MROWS:     drawBackFirst();      break;
          case MROWS + 1: itemControlTemp(0);   break;
          case MROWS + 2: itemControlMotion(0); break;
          default: break;
        }
      }
      else
        moveHighlight(-1, select_control.now + MROWS - index_control);
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_control.now) {
      case CASE_BACK:
        select_page.set(2);
        gotoMainMenu();
        break;
      case CONTROL_CASE_TEMP:
        checkkey = ID_TemperatureID;
        hmiValues.show_mode = -1;
        select_temp.reset();
        drawTemperatureMenu();
        break;
      case CONTROL_CASE_MOVE:
        checkkey = ID_Motion;
        select_motion.reset();
        drawMotionMenu();
        break;
      #if ENABLED(EEPROM_SETTINGS)
        case CONTROL_CASE_SAVE: {
          const bool success = settings.save();
          hmiAudioFeedback(success);
        } break;
        case CONTROL_CASE_LOAD: {
          const bool success = settings.load();
          hmiAudioFeedback(success);
        } break;
        case CONTROL_CASE_RESET:
          settings.reset();
          hmiAudioFeedback();
          break;
      #endif
      case CONTROL_CASE_ADVSET:
        checkkey = ID_AdvSet;
        select_advset.reset();
        drawAdvancedSettingsMenu();
        break;
      case CONTROL_CASE_INFO:
        checkkey = ID_Info;
        drawInfoMenu();
        break;
      default: break;
    }
  }
  dwinUpdateLCD();
}

#if HAS_ONESTEP_LEVELING
  // Leveling
  void hmiLeveling() {
    popupWindowLeveling();
    dwinUpdateLCD();
    queue.inject(F("G28O\nG29"));
  }
#endif

// Axis Move
void hmiAxisMove() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  #if ENABLED(PREVENT_COLD_EXTRUSION)
    // popup window resume
    if (hmiFlag.cold_flag) {
      if (encoder_diffState == ENCODER_DIFF_ENTER) {
        hmiFlag.cold_flag = false;
        hmiValues.moveScaled.e = current_position.e * MINUNITMULT;
        drawMoveMenu();
        TERN_(HAS_X_AXIS, drawEditFloat3(1, hmiValues.moveScaled.x));
        TERN_(HAS_Y_AXIS, drawEditFloat3(2, hmiValues.moveScaled.y));
        TERN_(HAS_Z_AXIS, drawEditFloat3(3, hmiValues.moveScaled.z));
        drawEditSignedFloat3(4, 0);
        dwinUpdateLCD();
      }
      return;
    }
  #endif

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_axis.inc(1 + 3 + ENABLED(HAS_HOTEND))) moveHighlight(1, select_axis.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_axis.dec()) moveHighlight(-1, select_axis.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_axis.now) {
      case CASE_BACK:
        checkkey = ID_Prepare;
        select_prepare.set(1);
        index_prepare = MROWS;
        drawPrepareMenu();
        break;

      #if HAS_X_AXIS
        case 1: // X axis move
          checkkey = ID_MoveX;
          hmiValues.moveScaled.x = current_position.x * MINUNITMULT;
          drawEditFloat3(1, hmiValues.moveScaled.x, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_Y_AXIS
        case 2: // Y axis move
          checkkey = ID_MoveY;
          hmiValues.moveScaled.y = current_position.y * MINUNITMULT;
          drawEditFloat3(2, hmiValues.moveScaled.y, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_Z_AXIS
        case 3: // Z axis move
          checkkey = ID_MoveZ;
          hmiValues.moveScaled.z = current_position.z * MINUNITMULT;
          drawEditFloat3(3, hmiValues.moveScaled.z, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_HOTEND
        case 4: // Extruder
          #if ENABLED(PREVENT_COLD_EXTRUSION)
            if (thermalManager.tooColdToExtrude(0)) {
              hmiFlag.cold_flag = true;
              popupWindowETempTooLow();
              dwinUpdateLCD();
              return;
            }
          #endif
          checkkey = ID_Extruder;
          hmiValues.moveScaled.e = current_position.e * MINUNITMULT;
          drawEditSignedFloat3(4, hmiValues.moveScaled.e, true);
          encoderRate.enabled = true;
          break;
      #endif
    }
  }
  dwinUpdateLCD();
}

// TemperatureID
void hmiTemperature() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_temp.inc(1 + TEMP_CASE_TOTAL)) moveHighlight(1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_temp.dec()) moveHighlight(-1, select_temp.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_temp.now) {
      case CASE_BACK:
        checkkey = ID_Control;
        select_control.set(1);
        index_control = MROWS;
        drawControlMenu();
        break;
      #if HAS_HOTEND
        case TEMP_CASE_TEMP:
          checkkey = ID_ETemp;
          hmiValues.tempE = thermalManager.degTargetHotend(0);
          drawEditInteger3(1, hmiValues.tempE, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_HEATED_BED
        case TEMP_CASE_BED:
          checkkey = ID_BedTemp;
          hmiValues.tempBed = thermalManager.degTargetBed();
          drawEditInteger3(2, hmiValues.tempBed, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_FAN
        case TEMP_CASE_FAN:
          checkkey = ID_FanSpeed;
          hmiValues.fanSpeed = thermalManager.fan_speed[0];
          drawEditInteger3(3, hmiValues.fanSpeed, true);
          encoderRate.enabled = true;
          break;
      #endif

      #if HAS_PREHEAT
        case TEMP_CASE_PLA: {
          checkkey = ID_PLAPreheat;
          select_PLA.reset();
          hmiValues.show_mode = -2;

          clearMainWindow();

          if (hmiIsChinese()) {
            dwinFrameTitleCopy(59, 16, 81, 14);                       // "PLA Settings"
            itemAreaCopy(100, 89, 124, 101, PREHEAT_CASE_TEMP);
            itemAreaCopy(1, 134, 56, 146, PREHEAT_CASE_TEMP, 24);      // PLA nozzle temp
            #if HAS_HEATED_BED
              itemAreaCopy(100, 89, 124, 101, PREHEAT_CASE_BED);
              itemAreaCopy(58, 134, 113, 146, PREHEAT_CASE_BED, 24);   // PLA bed temp
            #endif
            #if HAS_FAN
              itemAreaCopy(100, 89, 124, 101, PREHEAT_CASE_FAN);
              itemAreaCopy(115, 134, 170, 146, PREHEAT_CASE_FAN, 24);  // PLA fan speed
            #endif
            #if ENABLED(EEPROM_SETTINGS)
              itemAreaCopy(72, 148, 151, 162, PREHEAT_CASE_SAVE);      // Save PLA configuration
            #endif
          }
          else {
            #ifdef USE_STRING_HEADINGS
              drawTitle(F(PREHEAT_1_LABEL " Settings")); // TODO: GET_TEXT_F
            #else
              dwinFrameTitleCopy(56, 15, 85, 14);                       // "Temperature"  TODO: "PLA Settings"
            #endif
            #ifdef USE_STRING_TITLES
              dwinDrawLabel(PREHEAT_CASE_TEMP, F("Nozzle Temp"));
              #if HAS_HEATED_BED
                dwinDrawLabel(PREHEAT_CASE_BED, F("Bed Temp"));
              #endif
              #if HAS_FAN
                dwinDrawLabel(PREHEAT_CASE_FAN, GET_TEXT_F(MSG_FAN_SPEED));
              #endif
              #if ENABLED(EEPROM_SETTINGS)
                dwinDrawLabel(PREHEAT_CASE_SAVE, GET_TEXT_F(MSG_STORE_EEPROM));
              #endif
            #else
              say_pla_en(0, PREHEAT_CASE_TEMP);                           // "PLA"
              itemAreaCopy(198, 104, 237, 114, PREHEAT_CASE_TEMP, 27);   // "Nozzle"
              itemAreaCopy(1,  89,  81, 102, PREHEAT_CASE_TEMP, 71);     // "Temperature"
              #if HAS_HEATED_BED
                say_pla_en(0, PREHEAT_CASE_BED);                          // "PLA"
                itemAreaCopy(240, 104, 264, 114, PREHEAT_CASE_BED, 27);  // "Bed"
                itemAreaCopy(1, 89, 83, 101, PREHEAT_CASE_BED, 54);      // "Temperature"
              #endif
              #if HAS_FAN
                say_pla_en(0, PREHEAT_CASE_FAN);                          // "PLA"
                itemAreaCopy(0, 119, 64, 132, PREHEAT_CASE_FAN, 27);     // "Fan speed"
              #endif
              #if ENABLED(EEPROM_SETTINGS)
                itemAreaCopy(98, 164, 233, 177, PREHEAT_CASE_SAVE);      // "Save PLA parameters"
              #endif
            #endif
          }

          drawBackFirst();

          uint8_t i = 0;
          drawMenuLine(++i, ICON_SetEndTemp);
          drawEditInteger3(i, ui.material_preset[0].hotend_temp);
          #if HAS_HEATED_BED
            drawMenuLine(++i, ICON_SetBedTemp);
            drawEditInteger3(i, ui.material_preset[0].bed_temp);
          #endif
          #if HAS_FAN
            drawMenuLine(++i, ICON_FanSpeed);
            drawEditInteger3(i, ui.material_preset[0].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            drawMenuLine(++i, ICON_WriteEEPROM);
          #endif
        } break;
      #endif // HAS_PREHEAT

      #if PREHEAT_COUNT > 1
        case TEMP_CASE_ABS: { // ABS preheat setting
          checkkey = ID_ABSPreheat;
          select_ABS.reset();
          hmiValues.show_mode = -3;

          clearMainWindow();

          if (hmiIsChinese()) {
            dwinFrameTitleCopy(142, 16, 82, 14);                        // "ABS Settings"

            itemAreaCopy(180, 89, 204, 100, PREHEAT_CASE_TEMP);
            itemAreaCopy(1, 134, 56, 146, PREHEAT_CASE_TEMP, 24);        // ABS nozzle temp
            #if HAS_HEATED_BED
              itemAreaCopy(180, 89, 204, 100, PREHEAT_CASE_BED);
              itemAreaCopy(58, 134, 113, 146, PREHEAT_CASE_BED, 24);     // ABS bed temp
            #endif
            #if HAS_FAN
              itemAreaCopy(180, 89, 204, 100, PREHEAT_CASE_FAN);
              itemAreaCopy(115, 134, 170, 146, PREHEAT_CASE_FAN, 24);    // ABS fan speed
            #endif
            #if ENABLED(EEPROM_SETTINGS)
              itemAreaCopy(72, 148, 151, 162, PREHEAT_CASE_SAVE);
              itemAreaCopy(180, 89, 204, 100, PREHEAT_CASE_SAVE, 28, 2); // Save ABS configuration
            #endif
          }
          else {
            #ifdef USE_STRING_HEADINGS
              drawTitle(F("ABS Settings")); // TODO: GET_TEXT_F
            #else
              dwinFrameTitleCopy(56, 15, 85, 14);                       // "Temperature"  TODO: "ABS Settings"
            #endif
            #ifdef USE_STRING_TITLES
              dwinDrawLabel(PREHEAT_CASE_TEMP, F("Nozzle Temp"));
              #if HAS_HEATED_BED
                dwinDrawLabel(PREHEAT_CASE_BED, F("Bed Temp"));
              #endif
              #if HAS_FAN
                dwinDrawLabel(PREHEAT_CASE_FAN, GET_TEXT_F(MSG_FAN_SPEED));
              #endif
              #if ENABLED(EEPROM_SETTINGS)
                dwinDrawLabel(PREHEAT_CASE_SAVE, GET_TEXT_F(MSG_STORE_EEPROM));
              #endif
            #else
              say_abs_en(0, PREHEAT_CASE_TEMP);                           // "ABS"
              itemAreaCopy(197, 104, 238, 114, PREHEAT_CASE_TEMP, 29);   // "Nozzle"
              itemAreaCopy(1,  89,  34, 102, PREHEAT_CASE_TEMP, 73);     // "Temp"
              #if HAS_HEATED_BED
                say_abs_en(0, PREHEAT_CASE_BED);                          // "ABS"
                itemAreaCopy(240, 104, 264, 114, PREHEAT_CASE_BED, 29);  // "Bed"
                itemAreaCopy(1,  89,  83, 102, PREHEAT_CASE_BED, 56);    // "Temperature"
              #endif
              #if HAS_FAN
                say_abs_en(0, PREHEAT_CASE_FAN);                          // "ABS"
                itemAreaCopy(0, 119,  64, 132, PREHEAT_CASE_FAN, 29);    // "Fan speed"
              #endif
              #if ENABLED(EEPROM_SETTINGS)
                itemAreaCopy(98, 165, 233, 177, PREHEAT_CASE_SAVE);      // "Save PLA parameters"
                say_abs_en(33, PREHEAT_CASE_SAVE);                        // "ABS"
              #endif
            #endif
          }

          drawBackFirst();

          uint8_t i = 0;
          drawMenuLine(++i, ICON_SetEndTemp);
          drawEditInteger3(i, ui.material_preset[1].hotend_temp);
          #if HAS_HEATED_BED
            drawMenuLine(++i, ICON_SetBedTemp);
            drawEditInteger3(i, ui.material_preset[1].bed_temp);
          #endif
          #if HAS_FAN
            drawMenuLine(++i, ICON_FanSpeed);
            drawEditInteger3(i, ui.material_preset[1].fan_speed);
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            drawMenuLine(++i, ICON_WriteEEPROM);
          #endif

        } break;

      #endif // PREHEAT_COUNT > 1
    }
  }
  dwinUpdateLCD();
}

void drawMaxSpeedMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(1, 16, 28, 13);          // "Max Speed (mm/s)"

    auto say_max_speed_cn = [](const uint8_t line) {
      itemAreaCopy(173, 133, 228, 147, line);    // "Max speed"
    };

    say_max_speed_cn(1);                          // "Max speed"
    itemAreaCopy(229, 133, 236, 147, 1, 58);     // "X"
    say_max_speed_cn(2);                          // "Max speed"
    itemAreaCopy(1, 150, 7, 160, 2, 58, 3);      // "Y"
    say_max_speed_cn(3);                          // "Max speed"
    itemAreaCopy(9, 150, 16, 160, 3, 58, 3);     // "Z"
    #if HAS_HOTEND
      say_max_speed_cn(4);                        // "Max speed"
      itemAreaCopy(18, 150, 25, 160, 4, 58, 3);  // "E"
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(F("Max Speed (mm/s)")); // TODO: GET_TEXT_F
    #else
      dwinFrameTitleCopy(144, 16, 46, 11);                  // "Max Speed (mm/s)"
    #endif
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(1, F("Max Feedrate X"));
      dwinDrawLabel(2, F("Max Feedrate Y"));
      dwinDrawLabel(3, F("Max Feedrate Z"));
      #if HAS_HOTEND
        dwinDrawLabel(4, F("Max Feedrate E"));
      #endif
    #else
      say_max_en(1); say_speed_en(30, 1); say_x_en(73, 1);    // "Max Speed X"
      say_max_en(2); say_speed_en(30, 2); say_y_en(73, 2);    // "Max Speed Y"
      say_max_en(3); say_speed_en(30, 3); say_z_en(73, 3);    // "Max Speed Z"
      #if HAS_HOTEND
        say_max_en(4); say_speed_en(30, 4); say_e_en(73, 4);  // "Max Speed E"
      #endif
    #endif
  }

  drawBackFirst();
  for (uint8_t i = 0; i < 3 + ENABLED(HAS_HOTEND); ++i) drawMenuLine(i + 1, ICON_MaxSpeedX + i);
  drawEditInteger4(1, planner.settings.max_feedrate_mm_s[X_AXIS]);
  drawEditInteger4(2, planner.settings.max_feedrate_mm_s[Y_AXIS]);
  drawEditInteger4(3, planner.settings.max_feedrate_mm_s[Z_AXIS]);
  #if HAS_HOTEND
    drawEditInteger4(4, planner.settings.max_feedrate_mm_s[E_AXIS]);
  #endif
}

void drawMaxAccelMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(1, 16, 28, 13);            // "Acceleration"

    itemAreaCopy(173, 133, 200, 147, 1);
    itemAreaCopy( 28, 149,  69, 161, 1, 30, 1);
    itemAreaCopy(229, 133, 236, 147, 1, 74);       // Max acceleration X
    itemAreaCopy(173, 133, 200, 147, 2);
    itemAreaCopy( 28, 149,  69, 161, 2, 30, 1);
    itemAreaCopy(  1, 150,   7, 160, 2, 74, 2);    // Max acceleration Y
    itemAreaCopy(173, 133, 200, 147, 3);
    itemAreaCopy( 28, 149,  69, 161, 3, 30, 1);
    itemAreaCopy(  9, 150,  16, 160, 3, 74, 2);    // Max acceleration Z
    #if HAS_HOTEND
      itemAreaCopy(173, 133, 200, 147, 4);
      itemAreaCopy( 28, 149,  69, 161, 4, 30, 1);
      itemAreaCopy( 18, 150,  25, 160, 4, 74, 2);  // Max acceleration E
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_ACCELERATION));
    #else
      dwinFrameTitleCopy(144, 16, 46, 11);    // "Acceleration"
    #endif
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(1, F("Max Accel X"));
      dwinDrawLabel(2, F("Max Accel Y"));
      dwinDrawLabel(3, F("Max Accel Z"));
      #if HAS_HOTEND
        dwinDrawLabel(4, F("Max Accel E"));
      #endif
    #else
      say_max_accel_en(1); say_x_en(112, 1);    // "Max Acceleration X"
      say_max_accel_en(2); say_y_en(112, 2);    // "Max Acceleration Y"
      say_max_accel_en(3); say_z_en(112, 3);    // "Max Acceleration Z"
      #if HAS_HOTEND
        say_max_accel_en(4); say_e_en(112, 4);  // "Max Acceleration E"
      #endif
    #endif
  }

  drawBackFirst();
  for (uint8_t i = 0; i < 3 + ENABLED(HAS_HOTEND); ++i) drawMenuLine(i + 1, ICON_MaxAccX + i);
  drawEditInteger4(1, planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
  drawEditInteger4(2, planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
  drawEditInteger4(3, planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
  #if HAS_HOTEND
    drawEditInteger4(4, planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
  #endif
}

#if ENABLED(CLASSIC_JERK)
  void drawMaxJerkMenu() {
    clearMainWindow();

    if (hmiIsChinese()) {
      dwinFrameTitleCopy(1, 16, 28, 13);            // "Jerk"

      itemAreaCopy(173, 133, 200, 147, 1);
      itemAreaCopy(  1, 180,  28, 192, 1, 30, 1);
      itemAreaCopy(202, 133, 228, 147, 1, 56);
      itemAreaCopy(229, 133, 236, 147, 1, 86);       // Max Jerk speed X
      itemAreaCopy(173, 133, 200, 147, 2);
      itemAreaCopy(  1, 180,  28, 192, 2, 30, 1);
      itemAreaCopy(202, 133, 228, 147, 2, 56);
      itemAreaCopy(  1, 150,   7, 160, 2, 86, 3);    // Max Jerk speed Y
      itemAreaCopy(173, 133, 200, 147, 3);
      itemAreaCopy(  1, 180,  28, 192, 3, 30, 1);
      itemAreaCopy(202, 133, 228, 147, 3, 56);
      itemAreaCopy(  9, 150,  16, 160, 3, 86, 3);    // Max Jerk speed Z
      #if HAS_HOTEND
        itemAreaCopy(173, 133, 200, 147, 4);
        itemAreaCopy(  1, 180,  28, 192, 4, 30, 1);
        itemAreaCopy(202, 133, 228, 147, 4, 56);
        itemAreaCopy( 18, 150,  25, 160, 4, 86, 3);  // Max Jerk speed E
      #endif
    }
    else {
      #ifdef USE_STRING_HEADINGS
        drawTitle(GET_TEXT_F(MSG_JERK));
      #else
        dwinFrameTitleCopy(144, 16, 46, 11);        // "Jerk"
      #endif
      #ifdef USE_STRING_TITLES
        dwinDrawLabel(1, GET_TEXT_F(MSG_VA_JERK));
        dwinDrawLabel(2, GET_TEXT_F(MSG_VB_JERK));
        dwinDrawLabel(3, GET_TEXT_F(MSG_VC_JERK));
        #if HAS_HOTEND
          dwinDrawLabel(4, GET_TEXT_F(MSG_VE_JERK));
        #endif
      #else
        say_max_jerk_speed_en(1); say_x_en(102, 1);   // Max Jerk speed X
        say_max_jerk_speed_en(2); say_y_en(102, 2);   // Max Jerk speed Y
        say_max_jerk_speed_en(3); say_z_en(102, 3);   // Max Jerk speed Z
        #if HAS_HOTEND
          say_max_jerk_speed_en(4); say_e_en(102, 4); // Max Jerk speed E
        #endif
      #endif
    }

    drawBackFirst();
    for (uint8_t i = 0; i < 3 + ENABLED(HAS_HOTEND); ++i) drawMenuLine(i + 1, ICON_MaxSpeedJerkX + i);
    drawEditFloat3(1, planner.max_jerk.x * MINUNITMULT);
    drawEditFloat3(2, planner.max_jerk.y * MINUNITMULT);
    drawEditFloat3(3, planner.max_jerk.z * MINUNITMULT);
    #if HAS_HOTEND
      drawEditFloat3(4, planner.max_jerk.e * MINUNITMULT);
    #endif
  }
#endif

void drawStepsMenu() {
  clearMainWindow();

  if (hmiIsChinese()) {
    dwinFrameTitleCopy(1, 16, 28, 13);            // "Steps per mm"

    itemAreaCopy(153, 148, 194, 161, 1);
    itemAreaCopy(229, 133, 236, 147, 1, 44);       // Transmission Ratio X
    itemAreaCopy(153, 148, 194, 161, 2);
    itemAreaCopy(  1, 150,   7, 160, 2, 44, 3);    // Transmission Ratio Y
    itemAreaCopy(153, 148, 194, 161, 3);
    itemAreaCopy(  9, 150,  16, 160, 3, 44, 3);    // Transmission Ratio Z
    #if HAS_HOTEND
      itemAreaCopy(153, 148, 194, 161, 4);
      itemAreaCopy( 18, 150,  25, 160, 4, 44, 3);  // Transmission Ratio E
    #endif
  }
  else {
    #ifdef USE_STRING_HEADINGS
      drawTitle(GET_TEXT_F(MSG_STEPS_PER_MM));
    #else
      dwinFrameTitleCopy(144, 16, 46, 11);        // "Steps per mm"
    #endif
    #ifdef USE_STRING_TITLES
      dwinDrawLabel(1, GET_TEXT_F(MSG_A_STEPS));
      dwinDrawLabel(2, GET_TEXT_F(MSG_B_STEPS));
      dwinDrawLabel(3, GET_TEXT_F(MSG_C_STEPS));
      #if HAS_HOTEND
        dwinDrawLabel(4, GET_TEXT_F(MSG_E_STEPS));
      #endif
    #else
      say_steps_per_mm_en(1); say_x_en(101, 1);     // "Steps-per-mm X"
      say_steps_per_mm_en(2); say_y_en(101, 2);     // "Y"
      say_steps_per_mm_en(3); say_z_en(101, 3);     // "Z"
      #if HAS_HOTEND
        say_steps_per_mm_en(4); say_e_en(101, 4);   // "E"
      #endif
    #endif
  }

  drawBackFirst();
  for (uint8_t i = 0; i < 3 + ENABLED(HAS_HOTEND); ++i) drawMenuLine(i + 1, ICON_StepX + i);
  drawEditFloat3(1, planner.settings.axis_steps_per_mm[X_AXIS] * MINUNITMULT);
  drawEditFloat3(2, planner.settings.axis_steps_per_mm[Y_AXIS] * MINUNITMULT);
  drawEditFloat3(3, planner.settings.axis_steps_per_mm[Z_AXIS] * MINUNITMULT);
  #if HAS_HOTEND
    drawEditFloat3(4, planner.settings.axis_steps_per_mm[E_AXIS] * MINUNITMULT);
  #endif
}

// Motion
void hmiMotion() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_motion.inc(1 + MOTION_CASE_TOTAL)) moveHighlight(1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_motion.dec()) moveHighlight(-1, select_motion.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_motion.now) {
      case CASE_BACK:
        checkkey = ID_Control;
        select_control.set(CONTROL_CASE_MOVE);
        index_control = MROWS;
        drawControlMenu();
        break;
      case MOTION_CASE_RATE:
        checkkey = ID_MaxSpeed;
        select_speed.reset();
        drawMaxSpeedMenu();
        break;
      case MOTION_CASE_ACCEL:
        checkkey = ID_MaxAcceleration;
        select_acc.reset();
        drawMaxAccelMenu();
        break;
      #if ENABLED(CLASSIC_JERK)
        case MOTION_CASE_JERK:
          checkkey = ID_MaxJerk;
          select_jerk.reset();
          drawMaxJerkMenu();
         break;
      #endif
      case MOTION_CASE_STEPS:
        checkkey = ID_Step;
        select_step.reset();
        drawStepsMenu();
        break;
      default: break;
    }
  }
  dwinUpdateLCD();
}

// Advanced Settings
void hmiAdvSet() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_advset.inc(1 + ADVSET_CASE_TOTAL)) {
      if (select_advset.now > MROWS && select_advset.now > index_advset) {
        index_advset = select_advset.now;

        // Scroll up and draw a blank bottom line
        scrollMenu(DWIN_SCROLL_UP);

        //switch (index_advset) {  // Redraw last menu items
        //  default: break;
        //}

      }
      else {
        moveHighlight(1, select_advset.now + MROWS - index_advset);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_advset.dec()) {
      if (select_advset.now < index_advset - MROWS) {
        index_advset--;
        scrollMenu(DWIN_SCROLL_DOWN);

        //switch (index_advset) {  // Redraw first menu items
        //  default: break;
        //}
      }
      else {
        moveHighlight(-1, select_advset.now + MROWS - index_advset);
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_advset.now) {
      case CASE_BACK:
        checkkey = ID_Control;
        select_control.set(CONTROL_CASE_ADVSET);
        index_control = CONTROL_CASE_ADVSET;
        drawControlMenu();
        break;

      #if HAS_HOME_OFFSET
        case ADVSET_CASE_HOMEOFF:
          checkkey = ID_HomeOff;
          select_item.reset();
          hmiValues.homeOffsScaled.x = home_offset.x * 10;
          hmiValues.homeOffsScaled.y = home_offset.y * 10;
          hmiValues.homeOffsScaled.z = home_offset.z * 10;
          drawHomeOffMenu();
          break;
      #endif

      #if HAS_ONESTEP_LEVELING
        case ADVSET_CASE_PROBEOFF:
          checkkey = ID_ProbeOff;
          select_item.reset();
          hmiValues.probeOffsScaled.x = probe.offset.x * 10;
          hmiValues.probeOffsScaled.y = probe.offset.y * 10;
          drawProbeOffMenu();
          break;
      #endif

      #if HAS_HOTEND
        case ADVSET_CASE_HEPID:
          thermalManager.PID_autotune(ui.material_preset[0].hotend_temp, H_E0, 10, true);
          break;
      #endif

      #if HAS_HEATED_BED
        case ADVSET_CASE_BEDPID:
          thermalManager.PID_autotune(ui.material_preset[0].bed_temp, H_BED, 10, true);
          break;
      #endif

      #if ENABLED(POWER_LOSS_RECOVERY)
        case ADVSET_CASE_PWRLOSSR:
          recovery.enable(!recovery.enabled);
          drawCheckboxLine(ADVSET_CASE_PWRLOSSR + MROWS - index_advset, recovery.enabled);
          break;
      #endif
      default: break;
    }
  }
  dwinUpdateLCD();
}

#if HAS_HOME_OFFSET

  // Home Offset
  void hmiHomeOff() {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_item.inc(1 + 3)) moveHighlight(1, select_item.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_item.dec()) moveHighlight(-1, select_item.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_item.now) {
        case CASE_BACK:
          checkkey = ID_AdvSet;
          select_advset.set(ADVSET_CASE_HOMEOFF);
          drawAdvancedSettingsMenu();
          break;
        case 1: // Home Offset X
          checkkey = ID_HomeOffX;
          drawEditSignedFloat3(1, hmiValues.homeOffsScaled.x, true);
          encoderRate.enabled = true;
          break;
        case 2: // Home Offset Y
          checkkey = ID_HomeOffY;
          drawEditSignedFloat3(2, hmiValues.homeOffsScaled.y, true);
          encoderRate.enabled = true;
          break;
        case 3: // Home Offset Z
          checkkey = ID_HomeOffZ;
          drawEditSignedFloat3(3, hmiValues.homeOffsScaled.z, true);
          encoderRate.enabled = true;
          break;
        default: break;
      }
    }
    dwinUpdateLCD();
  }

  void hmiHomeOffN(const AxisEnum axis, float &posScaled, const_float_t lo, const_float_t hi) {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    if (applyEncoder(encoder_diffState, posScaled)) {
      checkkey = ID_HomeOff;
      encoderRate.enabled = false;
      set_home_offset(axis, posScaled / 10);
      drawEditSignedFloat3(select_item.now, posScaled);
      return;
    }
    LIMIT(posScaled, lo, hi);
    drawEditSignedFloat3(select_item.now, posScaled, true);
  }

  void hmiHomeOffX() { hmiHomeOffN(X_AXIS, hmiValues.homeOffsScaled.x, -500, 500); }
  void hmiHomeOffY() { hmiHomeOffN(Y_AXIS, hmiValues.homeOffsScaled.y, -500, 500); }
  void hmiHomeOffZ() { hmiHomeOffN(Z_AXIS, hmiValues.homeOffsScaled.z,  -20,  20); }

#endif // HAS_HOME_OFFSET

#if HAS_ONESTEP_LEVELING

  // Probe Offset
  void hmiProbeOff() {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_item.inc(1 + 2)) moveHighlight(1, select_item.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_item.dec()) moveHighlight(-1, select_item.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_item.now) {
        case CASE_BACK:
          checkkey = ID_AdvSet;
          select_advset.set(ADVSET_CASE_PROBEOFF);
          drawAdvancedSettingsMenu();
          break;
        case 1: // Probe Offset X
          checkkey = ID_ProbeOffX;
          drawEditSignedFloat3(1, hmiValues.probeOffsScaled.x, true);
          encoderRate.enabled = true;
          break;
        case 2: // Probe Offset Y
          checkkey = ID_ProbeOffY;
          drawEditSignedFloat3(2, hmiValues.probeOffsScaled.y, true);
          encoderRate.enabled = true;
          break;
      }
    }
    dwinUpdateLCD();
  }

  void hmiProbeOffN(float &posScaled, float &offset_ref) {
    EncoderState encoder_diffState = encoderReceiveAnalyze();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    if (applyEncoder(encoder_diffState, posScaled)) {
      checkkey = ID_ProbeOff;
      encoderRate.enabled = false;
      offset_ref = posScaled / 10;
      drawEditSignedFloat3(select_item.now, posScaled);
      return;
    }
    LIMIT(posScaled, -500, 500);
    drawEditSignedFloat3(select_item.now, posScaled, true);
  }

  void hmiProbeOffX() { hmiProbeOffN(hmiValues.probeOffsScaled.x, probe.offset.x); }
  void hmiProbeOffY() { hmiProbeOffN(hmiValues.probeOffsScaled.y, probe.offset.y); }

#endif // HAS_ONESTEP_LEVELING

// Info
void hmiInfo() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  if (encoder_diffState == ENCODER_DIFF_ENTER) {
    #if HAS_ONESTEP_LEVELING
      checkkey = ID_Control;
      select_control.set(CONTROL_CASE_INFO);
      drawControlMenu();
    #else
      select_page.set(3);
      gotoMainMenu();
    #endif
  }
  dwinUpdateLCD();
}

// Tune
void hmiTune() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_tune.inc(1 + TUNE_CASE_TOTAL)) {
      if (select_tune.now > MROWS && select_tune.now > index_tune) {
        index_tune = select_tune.now;
        scrollMenu(DWIN_SCROLL_UP);
      }
      else
        moveHighlight(1, select_tune.now + MROWS - index_tune);
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_tune.dec()) {
      if (select_tune.now < index_tune - MROWS) {
        index_tune--;
        scrollMenu(DWIN_SCROLL_DOWN);
        if (index_tune == MROWS) drawBackFirst();
      }
      else
        moveHighlight(-1, select_tune.now + MROWS - index_tune);
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_tune.now) {
      case 0: { // Back
        select_print.set(0);
        gotoPrintProcess();
      }
      break;
      case TUNE_CASE_SPEED: // Print speed
        checkkey = ID_PrintSpeed;
        hmiValues.printSpeed = feedrate_percentage;
        drawEditInteger3(TUNE_CASE_SPEED + MROWS - index_tune, hmiValues.printSpeed, true);
        encoderRate.enabled = true;
        break;
      #if HAS_HOTEND
        case TUNE_CASE_TEMP: // Nozzle temp
          checkkey = ID_ETemp;
          hmiValues.tempE = thermalManager.degTargetHotend(0);
          drawEditInteger3(TUNE_CASE_TEMP + MROWS - index_tune, hmiValues.tempE, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_HEATED_BED
        case TUNE_CASE_BED: // Bed temp
          checkkey = ID_BedTemp;
          hmiValues.tempBed = thermalManager.degTargetBed();
          drawEditInteger3(TUNE_CASE_BED + MROWS - index_tune, hmiValues.tempBed, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_FAN
        case TUNE_CASE_FAN: // Fan speed
          checkkey = ID_FanSpeed;
          hmiValues.fanSpeed = thermalManager.fan_speed[0];
          drawEditInteger3(TUNE_CASE_FAN + MROWS - index_tune, hmiValues.fanSpeed, true);
          encoderRate.enabled = true;
          break;
      #endif
      #if HAS_ZOFFSET_ITEM
        case TUNE_CASE_ZOFF: // Z-offset
          #if ANY(HAS_BED_PROBE, BABYSTEPPING)
            checkkey = ID_HomeOffset;
            hmiValues.offset_value = BABY_Z_VAR * 100;
            drawEditSignedFloat2(TUNE_CASE_ZOFF + MROWS - index_tune, hmiValues.offset_value, true);
            encoderRate.enabled = true;
          #else
            // Apply workspace offset, making the current position 0,0,0
            queue.inject(F("G92X0Y0Z0"));
            hmiAudioFeedback();
          #endif
        break;
      #endif
      default: break;
    }
  }
  dwinUpdateLCD();
}

#if HAS_PREHEAT

  // PLA Preheat
  void hmiPLAPreheatSetting() {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_PLA.inc(1 + PREHEAT_CASE_TOTAL)) moveHighlight(1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_PLA.dec()) moveHighlight(-1, select_PLA.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      switch (select_PLA.now) {
        case CASE_BACK:
          checkkey = ID_TemperatureID;
          select_temp.now = TEMP_CASE_PLA;
          hmiValues.show_mode = -1;
          drawTemperatureMenu();
          break;
        #if HAS_HOTEND
          case PREHEAT_CASE_TEMP:
            checkkey = ID_ETemp;
            hmiValues.tempE = ui.material_preset[0].hotend_temp;
            drawEditInteger3(PREHEAT_CASE_TEMP, ui.material_preset[0].hotend_temp, true);
            encoderRate.enabled = true;
            break;
        #endif
        #if HAS_HEATED_BED
          case PREHEAT_CASE_BED:
            checkkey = ID_BedTemp;
            hmiValues.tempBed = ui.material_preset[0].bed_temp;
            drawEditInteger3(PREHEAT_CASE_BED, ui.material_preset[0].bed_temp, true);
            encoderRate.enabled = true;
            break;
        #endif
        #if HAS_FAN
          case PREHEAT_CASE_FAN:
            checkkey = ID_FanSpeed;
            hmiValues.fanSpeed = ui.material_preset[0].fan_speed;
            drawEditInteger3(PREHEAT_CASE_FAN, ui.material_preset[0].fan_speed, true);
            encoderRate.enabled = true;
            break;
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          case PREHEAT_CASE_SAVE: {
            const bool success = settings.save();
            hmiAudioFeedback(success);
          } break;
        #endif
        default: break;
      }
    }
    dwinUpdateLCD();
  }

  #if PREHEAT_COUNT > 1
    // ABS Preheat
    void hmiABSPreheatSetting() {
      EncoderState encoder_diffState = get_encoder_state();
      if (encoder_diffState == ENCODER_DIFF_NO) return;

      // Avoid flicker by updating only the previous menu
      if (encoder_diffState == ENCODER_DIFF_CW) {
        if (select_ABS.inc(1 + PREHEAT_CASE_TOTAL)) moveHighlight(1, select_ABS.now);
      }
      else if (encoder_diffState == ENCODER_DIFF_CCW) {
        if (select_ABS.dec()) moveHighlight(-1, select_ABS.now);
      }
      else if (encoder_diffState == ENCODER_DIFF_ENTER) {
        switch (select_ABS.now) {
          case CASE_BACK:
            checkkey = ID_TemperatureID;
            select_temp.now = TEMP_CASE_ABS;
            hmiValues.show_mode = -1;
            drawTemperatureMenu();
            break;
          #if HAS_HOTEND
            case PREHEAT_CASE_TEMP:
              checkkey = ID_ETemp;
              hmiValues.tempE = ui.material_preset[1].hotend_temp;
              drawEditInteger3(PREHEAT_CASE_TEMP, ui.material_preset[1].hotend_temp, true);
              encoderRate.enabled = true;
              break;
          #endif
          #if HAS_HEATED_BED
            case PREHEAT_CASE_BED:
              checkkey = ID_BedTemp;
              hmiValues.tempBed = ui.material_preset[1].bed_temp;
              drawEditInteger3(PREHEAT_CASE_BED, ui.material_preset[1].bed_temp, true);
              encoderRate.enabled = true;
              break;
          #endif
          #if HAS_FAN
            case PREHEAT_CASE_FAN:
              checkkey = ID_FanSpeed;
              hmiValues.fanSpeed = ui.material_preset[1].fan_speed;
              drawEditInteger3(PREHEAT_CASE_FAN, ui.material_preset[1].fan_speed, true);
              encoderRate.enabled = true;
              break;
          #endif
          #if ENABLED(EEPROM_SETTINGS)
            case PREHEAT_CASE_SAVE: {
              const bool success = settings.save();
              hmiAudioFeedback(success);
            } break;
          #endif
          default: break;
        }
      }
      dwinUpdateLCD();
    }
  #endif // PREHEAT_COUNT > 1

#endif // HAS_PREHEAT

// Max Speed
void hmiMaxSpeed() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_speed.inc(1 + 3 + ENABLED(HAS_HOTEND))) moveHighlight(1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_speed.dec()) moveHighlight(-1, select_speed.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_speed.now, 1, 4)) {
      checkkey = ID_MaxSpeedValue;
      hmiFlag.feedspeed_axis = AxisEnum(select_speed.now - 1);
      hmiValues.maxFeedSpeed = planner.settings.max_feedrate_mm_s[hmiFlag.feedspeed_axis];
      drawEditInteger4(select_speed.now, hmiValues.maxFeedSpeed, true);
      encoderRate.enabled = true;
    }
    else { // Back
      checkkey = ID_Motion;
      select_motion.now = MOTION_CASE_RATE;
      drawMotionMenu();
    }
  }
  dwinUpdateLCD();
}

// Max Acceleration
void hmiMaxAcceleration() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_acc.inc(1 + 3 + ENABLED(HAS_HOTEND))) moveHighlight(1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_acc.dec()) moveHighlight(-1, select_acc.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_acc.now, 1, 4)) {
      checkkey = ID_MaxAccelerationValue;
      hmiFlag.acc_axis = AxisEnum(select_acc.now - 1);
      hmiValues.maxAcceleration = planner.settings.max_acceleration_mm_per_s2[hmiFlag.acc_axis];
      drawEditInteger4(select_acc.now, hmiValues.maxAcceleration, true);
      encoderRate.enabled = true;
    }
    else { // Back
      checkkey = ID_Motion;
      select_motion.now = MOTION_CASE_ACCEL;
      drawMotionMenu();
    }
  }
  dwinUpdateLCD();
}

#if ENABLED(CLASSIC_JERK)
  // Max Jerk
  void hmiMaxJerk() {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;

    // Avoid flicker by updating only the previous menu
    if (encoder_diffState == ENCODER_DIFF_CW) {
      if (select_jerk.inc(1 + 3 + ENABLED(HAS_HOTEND))) moveHighlight(1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_CCW) {
      if (select_jerk.dec()) moveHighlight(-1, select_jerk.now);
    }
    else if (encoder_diffState == ENCODER_DIFF_ENTER) {
      if (WITHIN(select_jerk.now, 1, 4)) {
        checkkey = ID_MaxJerkValue;
        hmiFlag.jerk_axis = AxisEnum(select_jerk.now - 1);
        hmiValues.maxJerkScaled = planner.max_jerk[hmiFlag.jerk_axis] * MINUNITMULT;
        drawEditFloat3(select_jerk.now, hmiValues.maxJerkScaled, true);
        encoderRate.enabled = true;
      }
      else { // Back
        checkkey = ID_Motion;
        select_motion.now = MOTION_CASE_JERK;
        drawMotionMenu();
      }
    }
    dwinUpdateLCD();
  }
#endif // CLASSIC_JERK

// Step
void hmiStep() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_step.inc(1 + 3 + ENABLED(HAS_HOTEND))) moveHighlight(1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_step.dec()) moveHighlight(-1, select_step.now);
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    if (WITHIN(select_step.now, 1, 4)) {
      checkkey = ID_StepValue;
      hmiFlag.step_axis = AxisEnum(select_step.now - 1);
      hmiValues.maxStepScaled = planner.settings.axis_steps_per_mm[hmiFlag.step_axis] * MINUNITMULT;
      drawEditFloat3(select_step.now, hmiValues.maxStepScaled, true);
      encoderRate.enabled = true;
    }
    else { // Back
      checkkey = ID_Motion;
      select_motion.now = MOTION_CASE_STEPS;
      drawMotionMenu();
    }
  }
  dwinUpdateLCD();
}

void hmiInit() {
  hmiSDCardInit();

  for (uint16_t t = 0; t <= 100; t += 2) {
    dwinIconShow(ICON, ICON_Bar, 15, 260);
    dwinDrawRectangle(1, COLOR_BG_BLACK, 15 + t * 242 / 100, 260, 257, 280);
    dwinUpdateLCD();
    delay(20);
  }

  hmiSetLanguage();
}

void dwinInitScreen() {
  hmiInit();
  hmiSetLanguageCache();
  hmiStartFrame(true);
}

void eachMomentUpdate() {
  static millis_t next_var_update_ms = 0, next_rts_update_ms = 0;

  const millis_t ms = millis();
  if (ELAPSED(ms, next_var_update_ms)) {
    next_var_update_ms = ms + DWIN_VAR_UPDATE_INTERVAL;
    updateVariable();
  }

  if (PENDING(ms, next_rts_update_ms)) return;
  next_rts_update_ms = ms + DWIN_SCROLL_UPDATE_INTERVAL;

  if (checkkey == ID_PrintProcess) {
    // if print done
    if (hmiFlag.print_finish && !hmiFlag.done_confirm_flag) {
      hmiFlag.print_finish = false;
      hmiFlag.done_confirm_flag = true;

      TERN_(POWER_LOSS_RECOVERY, recovery.cancel());

      planner.finish_and_disable();

      // show percent bar and value
      _card_percent = 0;
      drawPrintProgressBar();

      // show print done confirm
      dwinDrawRectangle(1, COLOR_BG_BLACK, 0, 250, DWIN_WIDTH - 1, STATUS_Y);
      dwinIconShow(ICON, hmiIsChinese() ? ICON_Confirm_C : ICON_Confirm_E, 86, 283);
    }
    else if (hmiFlag.pause_flag != printingIsPaused()) {
      // print status update
      hmiFlag.pause_flag = printingIsPaused();
      iconResumeOrPause();
    }
  }

  // pause after homing
  if (hmiFlag.pause_action && printingIsPaused() && !planner.has_blocks_queued()) {
    hmiFlag.pause_action = false;
    #if ENABLED(PAUSE_HEAT)
      TERN_(HAS_HOTEND, resume_hotend_temp = thermalManager.degTargetHotend(0));
      TERN_(HAS_HEATED_BED, resume_bed_temp = thermalManager.degTargetBed());
      thermalManager.disable_all_heaters();
    #endif
    queue.inject(F("G1 F1200 X0 Y0"));
  }

  if (card.isPrinting() && checkkey == ID_PrintProcess) { // Print process
    const uint8_t card_pct = card.percentDone();
    static uint8_t last_cardpercentValue = 101;
    if (last_cardpercentValue != card_pct) { // print percent
      last_cardpercentValue = card_pct;
      if (card_pct) {
        _card_percent = card_pct;
        drawPrintProgressBar();
      }
    }

    duration_t elapsed = print_job_timer.duration(); // Print timer

    // Print time so far
    static uint16_t last_Printtime = 0;
    const uint16_t min = (elapsed.value % 3600) / 60;
    if (last_Printtime != min) { // 1 minute update
      last_Printtime = min;
      drawPrintProgressElapsed();
    }

    // Estimate remaining time every 20 seconds
    static millis_t next_remain_time_update = 0;
    if (_card_percent > 1 && ELAPSED(ms, next_remain_time_update) && !hmiFlag.heat_flag) {
      _remain_time = (elapsed.value - dwin_heat_time) / (_card_percent * 0.01f) - (elapsed.value - dwin_heat_time);
      next_remain_time_update += DWIN_REMAIN_TIME_UPDATE_INTERVAL;
      drawPrintProgressRemain();
    }
  }
  else if (dwin_abort_flag && !hmiFlag.home_flag) { // Print Stop
    dwin_abort_flag = false;
    hmiValues.printSpeed = feedrate_percentage = 100;
    dwin_zoffset = BABY_Z_VAR;
    select_page.set(0);
    gotoMainMenu();
  }
  #if ENABLED(POWER_LOSS_RECOVERY)
    else if (DWIN_lcd_sd_status && recovery.ui_flag_resume) { // Resume interrupted print
      recovery.ui_flag_resume = false;

      auto update_selection = [&](const bool sel) {
        hmiFlag.select_flag = sel;
        const uint16_t c1 = sel ? COLOR_BG_WINDOW : COLOR_SELECT;
        dwinDrawRectangle(0, c1, 25, 306, 126, 345);
        dwinDrawRectangle(0, c1, 24, 305, 127, 346);
        const uint16_t c2 = sel ? COLOR_SELECT : COLOR_BG_WINDOW;
        dwinDrawRectangle(0, c2, 145, 306, 246, 345);
        dwinDrawRectangle(0, c2, 144, 305, 247, 346);
      };

      popupWindowResume();
      update_selection(true);

      char * const name = card.longest_filename();
      const int8_t npos = _MAX(0U, DWIN_WIDTH - strlen(name) * (MENU_CHR_W)) / 2;
      dwinDrawString(true, font8x16, COLOR_POPUP_TEXT, COLOR_BG_WINDOW, npos, 252, name);
      dwinUpdateLCD();

      bool recovery_flag = true;
      while (recovery_flag) {
        EncoderState encoder_diffState = encoderReceiveAnalyze();
        if (encoder_diffState != ENCODER_DIFF_NO) {
          if (encoder_diffState == ENCODER_DIFF_ENTER) {
            recovery_flag = false;
            if (hmiFlag.select_flag) break;
            queue.inject(F("M1000C"));
            hmiStartFrame(true);
            return;
          }
          else
            update_selection(encoder_diffState == ENCODER_DIFF_CW);

          dwinUpdateLCD();
        }
      }

      select_print.set(0);
      hmiValues.show_mode = 0;
      queue.inject(F("M1000"));
      gotoPrintProcess();
      drawStatusArea(true);
    }
  #endif // POWER_LOSS_RECOVERY

  dwinUpdateLCD();
}

void dwinHandleScreen() {
  switch (checkkey) {
    case ID_MainMenu:       hmiMainMenu(); break;
    case ID_SelectFile:     hmiSelectFile(); break;
    case ID_Prepare:        hmiPrepare(); break;
    case ID_Control:        hmiControl(); break;
    case ID_Leveling:       break;
    case ID_PrintProcess:   hmiPrinting(); break;
    case ID_PrintWindow:    hmiPauseOrStop(); break;
    case ID_AxisMove:       hmiAxisMove(); break;
    case ID_TemperatureID:  hmiTemperature(); break;
    case ID_Motion:         hmiMotion(); break;
    case ID_AdvSet:         hmiAdvSet(); break;
    #if HAS_HOME_OFFSET
      case ID_HomeOff:      hmiHomeOff(); break;
      case ID_HomeOffX:     hmiHomeOffX(); break;
      case ID_HomeOffY:     hmiHomeOffY(); break;
      case ID_HomeOffZ:     hmiHomeOffZ(); break;
    #endif
    #if HAS_ONESTEP_LEVELING
      case ID_ProbeOff:     hmiProbeOff(); break;
      case ID_ProbeOffX:    hmiProbeOffX(); break;
      case ID_ProbeOffY:    hmiProbeOffY(); break;
    #endif
    case ID_Info:           hmiInfo(); break;
    case ID_Tune:           hmiTune(); break;
    #if HAS_PREHEAT
      case ID_PLAPreheat:   hmiPLAPreheatSetting(); break;
      #if PREHEAT_COUNT > 1
        case ID_ABSPreheat: hmiABSPreheatSetting(); break;
      #endif
    #endif
    case ID_MaxSpeed:       hmiMaxSpeed(); break;
    case ID_MaxAcceleration: hmiMaxAcceleration(); break;
    #if ENABLED(CLASSIC_JERK)
      case ID_MaxJerk:      hmiMaxJerk(); break;
    #endif
    case ID_Step:           hmiStep(); break;
    case ID_MoveX:          hmiMoveX(); break;
    case ID_MoveY:          hmiMoveY(); break;
    case ID_MoveZ:          hmiMoveZ(); break;
    #if HAS_HOTEND
      case ID_Extruder:     hmiMoveE(); break;
      case ID_ETemp:        hmiETemp(); break;
    #endif
    #if ANY(HAS_BED_PROBE, BABYSTEPPING)
      case ID_HomeOffset:   hmiZoffset(); break;
    #endif
    #if HAS_HEATED_BED
      case ID_BedTemp:      hmiBedTemp(); break;
    #endif
    #if HAS_PREHEAT && HAS_FAN
      case ID_FanSpeed:     hmiFanSpeed(); break;
    #endif
    case ID_PrintSpeed:     hmiPrintSpeed(); break;
    case ID_MaxSpeedValue:  hmiMaxFeedspeedXYZE(); break;
    case ID_MaxAccelerationValue: hmiMaxAccelerationXYZE(); break;
    #if ENABLED(CLASSIC_JERK)
      case ID_MaxJerkValue: hmiMaxJerkXYZE(); break;
    #endif
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      case ID_StepValue:    hmiStepXYZE(); break;
    #endif
    default: break;
  }
}

void dwinHomingDone() {
  hmiFlag.home_flag = false;
  dwin_zoffset = TERN0(HAS_BED_PROBE, probe.offset.z);
  if (checkkey == ID_LastPrepare) {
    checkkey = ID_Prepare;
    select_prepare.now = PREPARE_CASE_HOME;
    index_prepare = MROWS;
    drawPrepareMenu();
  }
  else if (checkkey == ID_BackMain) {
    hmiValues.printSpeed = feedrate_percentage = 100;
    planner.finish_and_disable();
    gotoMainMenu();
  }
}

void dwinLevelingDone() {
  if (checkkey == ID_Leveling) gotoMainMenu();
}

void dwinStatusChanged(const char * const cstr/*=nullptr*/) {
  dwinDrawRectangle(1, COLOR_BG_BLUE, 0, STATUS_Y, DWIN_WIDTH, STATUS_Y + 24);
  const int8_t x = _MAX(0U, DWIN_WIDTH - strlen(cstr) * MENU_CHR_W) / 2;
  dwinDrawString(false, font8x16, COLOR_WHITE, COLOR_BG_BLUE, x, STATUS_Y + 3, cstr);
  dwinUpdateLCD();
}

void dwinStatusChanged(FSTR_P const fstr) {
  #ifdef __AVR__
    char str[strlen_P(FTOP(fstr)) + 1];
    strcpy_P(str, FTOP(fstr));
    dwinStatusChanged(str);
  #else
    dwinStatusChanged(FTOP(fstr));
  #endif
}

#endif // DWIN_CREALITY_LCD
