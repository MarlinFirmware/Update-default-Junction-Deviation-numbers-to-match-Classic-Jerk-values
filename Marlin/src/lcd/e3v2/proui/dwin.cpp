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

/**
 * DWIN Enhanced implementation for PRO UI
 * Based on the original work of: Miguel Risco-Castillo (MRISCOC)
 * https://github.com/mriscoc/Ender3V2S1
 * Version: 3.25.3
 * Date: 2023/05/18
 */

#include "../../../inc/MarlinConfig.h"

#if ENABLED(DWIN_LCD_PROUI)

#include "dwin.h"
#include "menus.h"
#include "dwin_popup.h"

#include "../../utf8.h"
#include "../../marlinui.h"
#include "../../extui/ui_api.h"
#include "../../../MarlinCore.h"
#include "../../../module/temperature.h"
#include "../../../module/printcounter.h"
#include "../../../module/motion.h"
#include "../../../module/planner.h"
#include "../../../module/stepper.h"
#include "../../../gcode/gcode.h"
#include "../../../gcode/queue.h"

#if HAS_MEDIA
  #include "../../../sd/cardreader.h"
#endif

#if NEED_HEX_PRINT
  #include "../../../libs/hex_print.h"
#endif

#if HAS_FILAMENT_SENSOR
  #include "../../../feature/runout.h"
#endif

#if ENABLED(EEPROM_SETTINGS)
  #include "../../../module/settings.h"
#endif

#if ENABLED(HOST_ACTION_COMMANDS)
  #include "../../../feature/host_actions.h"
#endif

#if DISABLED(PROBE_MANUALLY) && ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT)
  #define HAS_ONESTEP_LEVELING 1
#endif

#if HAS_MESH || (HAS_LEVELING && HAS_ZOFFSET_ITEM)
  #include "../../../feature/bedlevel/bedlevel.h"
  #include "bedlevel_tools.h"
#endif

#if HAS_BED_PROBE
  #include "../../../module/probe.h"
#endif

#if ENABLED(BLTOUCH)
  #include "../../../feature/bltouch.h"
#endif

#if ENABLED(BABYSTEPPING)
  #include "../../../feature/babystep.h"
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  #include "../../../feature/powerloss.h"
#endif

#if ENABLED(PRINTCOUNTER)
  #include "printstats.h"
#endif

#if ENABLED(CASE_LIGHT_MENU)
  #include "../../../feature/caselight.h"
#endif

#if ENABLED(LED_CONTROL_MENU)
  #include "../../../feature/leds/leds.h"
#endif

#if HAS_TRINAMIC_CONFIG
  #include "../../../feature/tmc_util.h"
#endif

#if HAS_GCODE_PREVIEW
  #include "gcode_preview.h"
#endif

#if HAS_ESDIAG
  #include "endstop_diag.h"
#endif

#if PROUI_TUNING_GRAPH
  #include "plot.h"
#endif

#if HAS_MESH
  #include "meshviewer.h"
#endif

#if HAS_LOCKSCREEN
  #include "lockscreen.h"
#endif

#ifndef MACHINE_SIZE
  #define MACHINE_SIZE STRINGIFY(X_BED_SIZE) "x" STRINGIFY(Y_BED_SIZE) "x" STRINGIFY(Z_MAX_POS)
#endif

#define PAUSE_HEAT

// Load and Unload limits
#ifndef EXTRUDE_MAXLENGTH
  #ifdef FILAMENT_CHANGE_UNLOAD_LENGTH
    #define EXTRUDE_MAXLENGTH (FILAMENT_CHANGE_UNLOAD_LENGTH + 10)
  #else
    #define EXTRUDE_MAXLENGTH 500
  #endif
#endif

// Juntion deviation limits
#define MIN_JD_MM             0.001
#define MAX_JD_MM             TERN(LIN_ADVANCE, 0.3f, 0.5f)

#if HAS_TRINAMIC_CONFIG
  #define MIN_TMC_CURRENT 100
  #define MAX_TMC_CURRENT 3000
#endif

// Editable temperature limits
#define MIN_ETEMP   0
#define MAX_ETEMP   thermalManager.hotend_max_target(0)
#define MIN_BEDTEMP 0
#define MAX_BEDTEMP BED_MAX_TARGET
#define MIN_CHAMBERTEMP 0
#define MAX_CHAMBERTEMP CHAMBER_MAX_TARGET

#define DWIN_VAR_UPDATE_INTERVAL          500
#define DWIN_UPDATE_INTERVAL             1000

#if HAS_MESH && HAS_BED_PROBE
  #define BABY_Z_VAR probe.offset.z
#else
  float z_offset = 0;
  #define BABY_Z_VAR z_offset
#endif

// Structs
hmi_value_t hmiValue;
hmi_flag_t hmiFlag{0};
hmi_data_t hmiData;

enum SelectItem : uint8_t {
  PAGE_PRINT = 0,
  PAGE_PREPARE,
  PAGE_CONTROL,
  PAGE_ADVANCE,
  PAGE_COUNT,

  PRINT_SETUP = 0,
  PRINT_PAUSE_RESUME,
  PRINT_STOP,
  PRINT_COUNT
};

typedef struct {
  uint8_t now, last;
  void set(uint8_t v) { now = last = v; }
  void reset() { set(0); }
  bool changed() { bool c = (now != last); if (c) last = now; return c; }
  bool dec() { if (now) now--; return changed(); }
  bool inc(uint8_t v) { if (now < (v - 1)) now++; else now = (v - 1); return changed(); }
} select_t;
select_t select_page{0}, select_print{0};

#if ENABLED(LCD_BED_TRAMMING)
  constexpr float bed_tramming_inset_lfbr[] = BED_TRAMMING_INSET_LFRB;
#endif

bool hash_changed = true; // Flag to know if message status was changed
bool blink = false;
uint8_t checkkey = 255, last_checkkey = ID_MainMenu;

// New menu system pointers
Menu *fileMenu = nullptr;
Menu *prepareMenu = nullptr;
#if ENABLED(LCD_BED_TRAMMING)
  Menu *trammingMenu = nullptr;
#endif
Menu *moveMenu = nullptr;
Menu *controlMenu = nullptr;
Menu *advancedSettingsMenu = nullptr;
#if HAS_HOME_OFFSET
  Menu *homeOffsetMenu = nullptr;
#endif
#if HAS_BED_PROBE
  Menu *probeSettingsMenu = nullptr;
#endif
Menu *filSetMenu = nullptr;
Menu *selectColorMenu = nullptr;
Menu *getColorMenu = nullptr;
Menu *tuneMenu = nullptr;
Menu *motionMenu = nullptr;
Menu *filamentMenu = nullptr;
#if ENABLED(MESH_BED_LEVELING)
  Menu *manualMeshMenu = nullptr;
#endif
#if HAS_PREHEAT
  Menu *preheatMenu = nullptr;
  Menu *preheatHotendMenu = nullptr;
#endif
Menu *temperatureMenu = nullptr;
Menu *maxSpeedMenu = nullptr;
Menu *maxAccelMenu = nullptr;
#if ENABLED(CLASSIC_JERK)
  Menu *maxJerkMenu = nullptr;
#endif
Menu *stepsMenu = nullptr;
#if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
  Menu *hotendMPCMenu = nullptr;
#endif
#if ANY(PID_EDIT_MENU, PID_AUTOTUNE_MENU)
  #if ENABLED(PIDTEMP)
    Menu *hotendPIDMenu = nullptr;
  #endif
  #if ENABLED(PIDTEMPBED)
    Menu *bedPIDMenu = nullptr;
  #endif
  #if ENABLED(PIDTEMPCHAMBER)
    Menu *chamberPIDMenu = nullptr;
  #endif
#endif
#if CASELIGHT_USES_BRIGHTNESS
  Menu *caseLightMenu = nullptr;
#endif
#if ENABLED(LED_CONTROL_MENU)
  Menu *ledControlMenu = nullptr;
#endif
#if HAS_BED_PROBE
  Menu *zOffsetWizMenu = nullptr;
#endif
#if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
  Menu *homingMenu = nullptr;
#endif
#if ENABLED(EDITABLE_HOMING_FEEDRATE)
  Menu *homingFRMenu = nullptr;
#endif
#if ENABLED(FWRETRACT)
  Menu *fwRetractMenu = nullptr;
#endif
#if HAS_MESH
  Menu *meshMenu = nullptr;
  #if ENABLED(PROUI_MESH_EDIT)
    Menu *editMeshMenu = nullptr;
  #endif
#endif
#if ENABLED(SHAPING_MENU)
  Menu *inputShapingMenu = nullptr;
#endif
#if HAS_TRINAMIC_CONFIG
  Menu *trinamicConfigMenu = nullptr;
#endif

// Updatable menuitems pointers
MenuItem *hotendTargetItem = nullptr;
MenuItem *bedTargetItem = nullptr;
MenuItem *fanSpeedItem = nullptr;
MenuItem *mMeshMoveZItem = nullptr;
MenuItem *editZValueItem = nullptr;

bool isPrinting() { return printingIsActive() || printingIsPaused(); }
bool sdPrinting() { return isPrinting() && IS_SD_FILE_OPEN(); }
bool hostPrinting() { return isPrinting() && !IS_SD_FILE_OPEN(); }

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

//-----------------------------------------------------------------------------
// Main Buttons
//-----------------------------------------------------------------------------

typedef struct { uint16_t x, y[2], w, h; } text_info_t;

void ICON_Button(const bool selected, const int iconid, const frame_rect_t &ico, const text_info_t (&txt), FSTR_P caption) {
  DWINUI::drawIconWB(iconid + selected, ico.x, ico.y);
  if (selected) DWINUI::drawBox(0, hmiData.colorHighlight, ico);
  if (hmiIsChinese()) {
    dwinFrameAreaCopy(1, txt.x, txt.y[selected], txt.x + txt.w - 1, txt.y[selected] + txt.h - 1, ico.x + (ico.w - txt.w) / 2, (ico.y + ico.h - 25) - txt.h/2);
  }
  else {
    const uint16_t x = ico.x + (ico.w - strlen_P(FTOP(caption)) * DWINUI::fontWidth()) / 2,
                   y = (ico.y + ico.h - 20) - DWINUI::fontHeight() / 2;
    DWINUI::drawString(x, y, caption);
  }
}

//
// Main Menu: "Print"
//
void ICON_Print() {
  constexpr frame_rect_t ico = { 17, 110, 110, 100 };
  constexpr text_info_t txt = { 1, { 405, 447 }, 27, 15 };
  ICON_Button(select_page.now == PAGE_PRINT, ICON_Print_0, ico, txt, GET_TEXT_F(MSG_BUTTON_PRINT));
}

//
// Main Menu: "Prepare"
//
void ICON_Prepare() {
  constexpr frame_rect_t ico = { 145, 110, 110, 100 };
  constexpr text_info_t txt = { 31, { 405, 447 }, 27, 15 };
  ICON_Button(select_page.now == PAGE_PREPARE, ICON_Prepare_0, ico, txt, GET_TEXT_F(MSG_PREPARE));
}

//
// Main Menu: "Control"
//
void ICON_Control() {
  constexpr frame_rect_t ico = { 17, 226, 110, 100 };
  constexpr text_info_t txt = { 61, { 405, 447 }, 27, 15 };
  ICON_Button(select_page.now == PAGE_CONTROL, ICON_Control_0, ico, txt, GET_TEXT_F(MSG_CONTROL));
}

//
// Main Menu: "Advanced Settings"
//
void ICON_AdvSettings() {
  constexpr frame_rect_t ico = { 145, 226, 110, 100 };
  constexpr text_info_t txt = { 91, { 405, 447 }, 27, 15 };
  ICON_Button(select_page.now == PAGE_ADVANCE, ICON_Info_0, ico, txt, GET_TEXT_F(MSG_BUTTON_ADVANCED));
}

//
// Printing: "Tune"
//
void ICON_Tune() {
  constexpr frame_rect_t ico = { 8, 232, 80, 100 };
  constexpr text_info_t txt = { 121, { 405, 447 }, 27, 15 };
  ICON_Button(select_print.now == PRINT_SETUP, ICON_Setup_0, ico, txt, GET_TEXT_F(MSG_TUNE));
}

//
// Printing: "Pause"
//
void ICON_Pause() {
  constexpr frame_rect_t ico = { 96, 232, 80, 100 };
  constexpr text_info_t txt = { 181, { 405, 447 }, 27, 15 };
  ICON_Button(select_print.now == PRINT_PAUSE_RESUME, ICON_Pause_0, ico, txt, GET_TEXT_F(MSG_BUTTON_PAUSE));
}

//
// Printing: "Resume"
//
void ICON_Resume() {
  constexpr frame_rect_t ico = { 96, 232, 80, 100 };
  constexpr text_info_t txt = { 1, { 405, 447 }, 27, 15 };
  ICON_Button(select_print.now == PRINT_PAUSE_RESUME, ICON_Continue_0, ico, txt, GET_TEXT_F(MSG_BUTTON_RESUME));
}

//
// Printing: "Stop"
//
void ICON_Stop() {
  constexpr frame_rect_t ico = { 184, 232, 80, 100 };
  constexpr text_info_t txt = { 151, { 405, 447 }, 27, 12 };
  ICON_Button(select_print.now == PRINT_STOP, ICON_Stop_0, ico, txt, GET_TEXT_F(MSG_BUTTON_STOP));
}

//
// PopUps
//
void popupPauseOrStop() {
  if (hmiIsChinese()) {
    DWINUI::clearMainArea();
    drawPopupBkgd();
         if (select_print.now == PRINT_PAUSE_RESUME) dwinFrameAreaCopy(1, 237, 338, 269, 356, 98, 150);
    else if (select_print.now == PRINT_STOP) dwinFrameAreaCopy(1, 221, 320, 253, 336, 98, 150);
    dwinFrameAreaCopy(1, 220, 304, 264, 319, 130, 150);
    DWINUI::drawIconWB(ICON_Confirm_C, 26, 280);
    DWINUI::drawIconWB(ICON_Cancel_C, 146, 280);
    drawSelectHighlight(true);
    dwinUpdateLCD();
  }
  else {
    switch (select_print.now) {
      case PRINT_PAUSE_RESUME: dwinPopupConfirmCancel(ICON_Pause_1, GET_TEXT_F(MSG_PAUSE_PRINT)); break;
      case PRINT_STOP: dwinPopupConfirmCancel(ICON_Stop_1, GET_TEXT_F(MSG_STOP_PRINT)); break;
      default: break;
    }
  }
}

#if HAS_HOTEND || HAS_HEATED_BED || HAS_HEATED_CHAMBER
  void dwinPopupTemperature(const int_fast8_t heater_id, const uint8_t state) {
    hmiSaveProcessID(ID_WaitResponse);
    if (hmiIsChinese()) {
      DWINUI::clearMainArea();
      drawPopupBkgd();
      if (state == 1) {
        DWINUI::drawIcon(ICON_TempTooHigh, 102, 165);
        dwinFrameAreaCopy(1, 103, 371, 237, 386, 52, 285);
        dwinFrameAreaCopy(1, 151, 389, 185, 402, 187, 285);
        dwinFrameAreaCopy(1, 189, 389, 271, 402, 95, 310);
      }
      else if (state == 0) {
        DWINUI::drawIcon(ICON_TempTooLow, 102, 165);
        dwinFrameAreaCopy(1, 103, 371, 270, 386, 52, 285);
        dwinFrameAreaCopy(1, 189, 389, 271, 402, 95, 310);
      }
      else {
        // Chinese "Temp Error"
      }
    }
    else {
      FSTR_P heaterstr = nullptr;
           if (TERN0(HAS_HEATED_BED,     heater_id == H_BED))     heaterstr = F("Bed");
      else if (TERN0(HAS_HEATED_CHAMBER, heater_id == H_CHAMBER)) heaterstr = F("Chamber");
      else if (TERN0(HAS_HOTEND,         heater_id >= 0))         heaterstr = F("Nozzle");
      FSTR_P errorstr;
      uint8_t icon;
      switch (state) {
        case 0:  errorstr = GET_TEXT_F(MSG_TEMP_TOO_LOW);       icon = ICON_TempTooLow;  break;
        case 1:  errorstr = GET_TEXT_F(MSG_TEMP_TOO_HIGH);      icon = ICON_TempTooHigh; break;
        default: errorstr = GET_TEXT_F(MSG_ERR_HEATING_FAILED); icon = ICON_Temperature; break; // May be thermal runaway, temp malfunction, etc.
      }
      dwinShowPopup(icon, heaterstr, errorstr, BTN_Continue);
    }
  }
#endif

//
// Draw status line
//
void dwinDrawStatusLine(const char *text) {
  dwinDrawRectangle(1, hmiData.colorStatusBg, 0, STATUS_Y, DWIN_WIDTH, STATUS_Y + 20);
  if (text) DWINUI::drawCenteredString(hmiData.colorStatusTxt, STATUS_Y + 2, text);
}
void dwinDrawStatusLine(FSTR_P fstr) { dwinDrawStatusLine(FTOP(fstr)); }

// Clear & reset status line
void dwinResetStatusLine() {
  ui.status_message.clear();
  dwinCheckStatusMessage();
}

// Djb2 hash algorithm
uint32_t getHash(char * str) {
  uint32_t hash = 5381;
  for (char c; (c = *str++);) hash = ((hash << 5) + hash) + c; /* hash * 33 + c */
  return hash;
}

// Check for a change in the status message
void dwinCheckStatusMessage() {
  static MString<>::hash_t old_hash = 0x0000;
  const MString<>::hash_t hash = ui.status_message.hash();
  hash_changed = hash != old_hash;
  old_hash = hash;
}

void dwinDrawStatusMessage() {
  #if ENABLED(STATUS_MESSAGE_SCROLLING)

    // Get the UTF8 character count of the string
    uint8_t slen = ui.status_message.glyphs();

    // If the string fits the status line do not scroll it
    if (slen <= LCD_WIDTH) {
      if (hash_changed) {
        dwinDrawStatusLine(ui.status_message);
        hash_changed = false;
      }
    }
    else {
      // String is larger than the available line space

      // Get a pointer to the next valid UTF8 character
      // and the string remaining length
      uint8_t rlen;
      const char *stat = ui.status_and_len(rlen);
      dwinDrawRectangle(1, hmiData.colorStatusBg, 0, STATUS_Y, DWIN_WIDTH, STATUS_Y + 20);
      DWINUI::moveTo(0, STATUS_Y + 2);
      DWINUI::drawString(hmiData.colorStatusTxt, stat, LCD_WIDTH);

      // If the string doesn't completely fill the line...
      if (rlen < LCD_WIDTH) {
        DWINUI::drawChar(hmiData.colorStatusTxt, '.');     // Always at 1+ spaces left, draw a dot
        uint8_t chars = LCD_WIDTH - rlen;                  // Amount of space left in characters
        if (--chars) {                                     // Draw a second dot if there's space
          DWINUI::drawChar(hmiData.colorStatusTxt, '.');
          if (--chars)
            DWINUI::drawString(hmiData.colorStatusTxt, ui.status_message, chars); // Print a second copy of the message
        }
      }
      ui.advance_status_scroll();
    }

  #else

    if (hash_changed) {
      ui.status_message.trunc(LCD_WIDTH);
      dwinDrawStatusLine(ui.status_message);
      hash_changed = false;
    }

  #endif
}

void drawPrintLabels() {
  if (hmiIsChinese()) {
    dwinFrameAreaCopy(1,  0, 72,  63, 86,  41, 173);  // Printing Time
    dwinFrameAreaCopy(1, 65, 72, 128, 86, 176, 173);  // Remain
  }
  else {
    DWINUI::drawString( 46, 173, GET_TEXT_F(MSG_INFO_PRINT_TIME));
    DWINUI::drawString(181, 173, GET_TEXT_F(MSG_REMAINING_TIME));
  }
}

void drawPrintProgressBar() {
  const uint8_t _percent_done = ui.get_progress_percent();
  DWINUI::drawIconWB(ICON_Bar, 15, 93);
  dwinDrawRectangle(1, hmiData.colorBarfill, 16 + _percent_done * 240 / 100, 93, 256, 113);
  DWINUI::drawInt(hmiData.colorPercentTxt, hmiData.colorBackground, 3, 117, 133, _percent_done);
  DWINUI::drawString(hmiData.colorPercentTxt, 142, 133, F("%"));
}

void drawPrintProgressElapsed() {
  MString<12> buf;
  duration_t elapsed = print_job_timer.duration(); // Print timer
  buf.setf(F("%02i:%02i "), uint16_t(elapsed.value / 3600), (uint16_t(elapsed.value) % 3600) / 60);
  DWINUI::drawString(hmiData.colorText, hmiData.colorBackground, 47, 192, buf);
}

#if ENABLED(SHOW_REMAINING_TIME)
  uint32_t _remain_time = 0;
  void drawPrintProgressRemain() {
    MString<12> buf;
    buf.setf(F("%02i:%02i "), _remain_time / 3600, (_remain_time % 3600) / 60);
    DWINUI::drawString(hmiData.colorText, hmiData.colorBackground, 181, 192, buf);
  }
#endif

void ICON_ResumeOrPause() {
  if (checkkey == ID_PrintProcess) (print_job_timer.isPaused() || hmiFlag.pause_flag) ? ICON_Resume() : ICON_Pause();
}

// Print a string (up to 30 characters) in the header,
// e.g., The filename or string sent with M75.
void dwinPrintHeader(const char * const cstr/*=nullptr*/) {
  static char headertxt[31] = "";  // Print header text
  if (cstr) {
    const int8_t size = _MIN(30U, strlen(cstr));
    for (uint8_t i = 0; i < size; ++i) headertxt[i] = cstr[i];
    headertxt[size] = '\0';
  }
  if (checkkey == ID_PrintProcess || checkkey == ID_PrintDone) {
    dwinDrawRectangle(1, hmiData.colorBackground, 0, 60, DWIN_WIDTH, 60 + 16);
    DWINUI::drawCenteredString(60, headertxt);
  }
}

void drawPrintProcess() {
  if (hmiIsChinese())
    title.frameCopy(30, 1, 42, 14);                     // "Printing"
  else
    title.showCaption(GET_TEXT_F(MSG_PRINTING));
  DWINUI::clearMainArea();
  dwinPrintHeader();
  drawPrintLabels();
  DWINUI::drawIcon(ICON_PrintTime, 15, 173);
  DWINUI::drawIcon(ICON_RemainTime, 150, 171);
  drawPrintProgressBar();
  drawPrintProgressElapsed();
  TERN_(SHOW_REMAINING_TIME, drawPrintProgressRemain());
  ICON_Tune();
  ICON_ResumeOrPause();
  ICON_Stop();
}

void gotoPrintProcess() {
  if (checkkey == ID_PrintProcess)
    ICON_ResumeOrPause();
  else {
    checkkey = ID_PrintProcess;
    drawPrintProcess();
    TERN_(DASH_REDRAW, dwinRedrawDash());
  }
  dwinUpdateLCD();
}

void drawPrintDone() {
  TERN_(SET_PROGRESS_PERCENT, ui.set_progress_done());
  TERN_(SET_REMAINING_TIME, ui.reset_remaining_time());
  title.showCaption(GET_TEXT_F(MSG_PRINT_DONE));
  DWINUI::clearMainArea();
  dwinPrintHeader();
  #if HAS_GCODE_PREVIEW
    const bool haspreview = preview.valid();
    if (haspreview) {
      preview.show();
      DWINUI::drawButton(BTN_Continue, 86, 295);
    }
  #else
    constexpr bool haspreview = false;
  #endif

  if (!haspreview) {
    drawPrintProgressBar();
    drawPrintLabels();
    DWINUI::drawIcon(ICON_PrintTime, 15, 173);
    DWINUI::drawIcon(ICON_RemainTime, 150, 171);
    drawPrintProgressElapsed();
    TERN_(SHOW_REMAINING_TIME, drawPrintProgressRemain());
    DWINUI::drawButton(BTN_Continue, 86, 273);
  }
}

void gotoPrintDone() {
  wait_for_user = true;
  if (checkkey != ID_PrintDone) {
    checkkey = ID_PrintDone;
    drawPrintDone();
    dwinUpdateLCD();
  }
}

void drawMainMenu() {
  DWINUI::clearMainArea();
  if (hmiIsChinese())
    title.frameCopy(2, 2, 26, 13);   // "Home" etc
  else
    title.showCaption(MACHINE_NAME);
  DWINUI::drawIcon(ICON_LOGO, 71, 52);  // CREALITY logo
  ICON_Print();
  ICON_Prepare();
  ICON_Control();
  ICON_AdvSettings();
}

void gotoMainMenu() {
  if (checkkey == ID_MainMenu) return;
  checkkey = ID_MainMenu;
  drawMainMenu();
  dwinUpdateLCD();
}

// Draw X, Y, Z and blink if in an un-homed or un-trusted state
void _update_axis_value(const AxisEnum axis, const uint16_t x, const uint16_t y, const bool force) {
  const bool draw_qmark = axis_should_home(axis),
             draw_empty = NONE(HOME_AFTER_DEACTIVATE, DISABLE_REDUCED_ACCURACY_WARNING) && !draw_qmark && !axis_is_trusted(axis);

  // Check for a position change
  static xyz_pos_t oldpos = { -1, -1, -1 };

  const float p = (
    #if ALL(IS_FULL_CARTESIAN, SHOW_REAL_POS)
      planner.get_axis_position_mm(axis)
    #else
      current_position[axis]
    #endif
  );

  const bool changed = oldpos[axis] != p;
  if (changed) oldpos[axis] = p;

  if (force || changed || draw_qmark || draw_empty) {
    if (blink && draw_qmark)
      DWINUI::drawString(hmiData.colorCoordinate, hmiData.colorBackground, x, y, F("  - ? -"));
    else if (blink && draw_empty)
      DWINUI::drawString(hmiData.colorCoordinate, hmiData.colorBackground, x, y, F("       "));
    else
      DWINUI::drawSignedFloat(hmiData.colorCoordinate, hmiData.colorBackground, 3, 2, x, y, p);
  }
}

void _drawIconBlink(bool &flag, const bool sensor, const uint8_t icon1, const uint8_t icon2, const uint16_t x, const uint16_t y) {
  #if DISABLED(NO_BLINK_IND)
    if (flag != sensor) {
      flag = sensor;
      if (!flag) {
        dwinDrawBox(1, hmiData.colorBackground, x, y, 20, 20);
        DWINUI::drawIcon(icon1, x, y);
      }
    }
    if (flag) {
      dwinDrawBox(1, blink ? hmiData.colorSplitLine : hmiData.colorBackground, x, y, 20, 20);
      DWINUI::drawIcon(icon2, x, y);
    }
  #else
    if (flag != sensor) {
      flag = sensor;
      dwinDrawBox(1, hmiData.colorBackground, x, y, 20, 20);
      DWINUI::drawIcon(flag ? icon2 : icon1, x, y);
    }
  #endif
}

void _drawZOffsetIcon() {
  #if HAS_LEVELING
    static bool _leveling_active = false;
    _drawIconBlink(_leveling_active, planner.leveling_active, ICON_Zoffset, ICON_SetZOffset, 187, 416);
  #else
    DWINUI::drawIcon(ICON_Zoffset, 187, 416);
  #endif
}

void _drawFeedrate() {
  #if ENABLED(SHOW_SPEED_IND)
    int16_t _value;
    if (blink) {
      _value = feedrate_percentage;
      DWINUI::drawString(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 116 + 4 * STAT_CHR_W + 2, 384, F(" %"));
    }
    else {
      _value = CEIL(MMS_SCALED(feedrate_mm_s));
      dwinDrawBox(1, hmiData.colorBackground, 116 + 5 * STAT_CHR_W + 2, 384, 20, 20);
    }
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 116 + 2 * STAT_CHR_W, 384, _value);
  #else
    static int16_t _feedrate = 100;
    if (_feedrate != feedrate_percentage) {
      _feedrate = feedrate_percentage;
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 116 + 2 * STAT_CHR_W, 384, _feedrate);
    }
  #endif
}

void _drawXYZPosition(const bool force) {
  _update_axis_value(X_AXIS,  27, 457, force);
  _update_axis_value(Y_AXIS, 112, 457, force);
  _update_axis_value(Z_AXIS, 197, 457, force);
}

void updateVariable() {
  _drawXYZPosition(false);
  #if HAS_HOTEND
    static celsius_t _hotendtemp = 0, _hotendtarget = 0;
    const celsius_t hc = thermalManager.wholeDegHotend(0),
                    ht = thermalManager.degTargetHotend(0);
    const bool _new_hotend_temp = _hotendtemp != hc,
               _new_hotend_target = _hotendtarget != ht;
    if (_new_hotend_temp) _hotendtemp = hc;
    if (_new_hotend_target) _hotendtarget = ht;

    // if hotend is near target or heating, ICON indicates hot
    if (thermalManager.degHotendNear(0, ht) || thermalManager.isHeatingHotend(0)) {
      dwinDrawBox(1, hmiData.colorBackground, 10, 383, 20, 20);
      DWINUI::drawIcon(ICON_SetEndTemp, 10, 383);
    }
    else {
      dwinDrawBox(1, hmiData.colorBackground, 10, 383, 20, 20);
      DWINUI::drawIcon(ICON_HotendTemp, 10, 383);
    }
  #endif // HAS_HOTEND

  #if HAS_HEATED_BED
    static celsius_t _bedtemp = 0, _bedtarget = 0;
    const celsius_t bc = thermalManager.wholeDegBed(),
                    bt = thermalManager.degTargetBed();
    const bool _new_bed_temp = _bedtemp != bc,
               _new_bed_target = _bedtarget != bt;
    if (_new_bed_temp) _bedtemp = bc;
    if (_new_bed_target) _bedtarget = bt;

    // if bed is near target, heating, or if degrees > 44, ICON indicates hot
    if (thermalManager.degBedNear(bt) || thermalManager.isHeatingBed() || (bc > 44)) {
      dwinDrawBox(1, hmiData.colorBackground, 10, 416, 20, 20);
      DWINUI::drawIcon(ICON_BedTemp, 10, 416);
    }
    else {
      dwinDrawBox(1, hmiData.colorBackground, 10, 416, 20, 20);
      DWINUI::drawIcon(ICON_SetBedTemp, 10, 416);
    }
  #endif // HAS_HEATED_BED

  #if HAS_FAN
    static uint8_t _fanspeed = 0;
    const bool _new_fanspeed = _fanspeed != thermalManager.fan_speed[0];
    if (_new_fanspeed) _fanspeed = thermalManager.fan_speed[0];
  #endif

  if (isMenu(tuneMenu) || isMenu(temperatureMenu)) {
    // Tune page temperature update
    TERN_(HAS_HOTEND, if (_new_hotend_target) hotendTargetItem->redraw());
    TERN_(HAS_HEATED_BED, if (_new_bed_target) bedTargetItem->redraw());
    TERN_(HAS_FAN, if (_new_fanspeed) fanSpeedItem->redraw());
  }

  // Bottom temperature update

  #if HAS_HOTEND
    if (_new_hotend_temp)
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 28, 384, _hotendtemp);
    if (_new_hotend_target)
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 25 + 4 * STAT_CHR_W + 6, 384, _hotendtarget);

    static int16_t _flow = planner.flow_percentage[0];
    if (_flow != planner.flow_percentage[0]) {
      _flow = planner.flow_percentage[0];
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 116 + 2 * STAT_CHR_W, 417, _flow);
    }
  #endif

  #if HAS_HEATED_BED
    if (_new_bed_temp)
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 28, 417, _bedtemp);
    if (_new_bed_target)
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 25 + 4 * STAT_CHR_W + 6, 417, _bedtarget);
  #endif

  _drawFeedrate();

  #if HAS_FAN
    if (_new_fanspeed)
      DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 195 + 2 * STAT_CHR_W, 384, _fanspeed);
  #endif

  static float _offset = 0;
  if (BABY_Z_VAR != _offset) {
    _offset = BABY_Z_VAR;
    DWINUI::drawSignedFloat(DWIN_FONT_STAT, hmiData.colorIndicator,  hmiData.colorBackground, 2, 2, 204, 417, _offset);
  }

  _drawZOffsetIcon();
}

/**
 * Memory card and file management
 */

bool DWIN_lcd_sd_status = false;

#if ENABLED(PROUI_MEDIASORT)
  void setMediaSort() {
    toggleCheckboxLine(hmiData.mediaSort);
    card.setSortOn(hmiData.mediaSort ? TERN(SDSORT_REVERSE, AS_REV, AS_FWD) : AS_OFF);
  }
#endif

void setMediaAutoMount() { toggleCheckboxLine(hmiData.mediaAutoMount); }

inline uint16_t nr_sd_menu_items() {
  return _MIN(card.get_num_items() + !card.flag.workDirIsRoot, MENU_MAX_ITEMS);
}

void makeNameWithoutExt(char *dst, char *src, size_t maxlen=MENU_CHAR_LIMIT) {
  size_t pos = strlen(src); // Index of ending nul

  // For files, remove the extension
  // which may be .gcode, .gco, or .g
  if (!card.flag.filenameIsDir)
    while (pos && src[pos] != '.') pos--; // Find last '.' (stop at 0)

  if (!pos) pos = strlen(src); // pos = 0 ('.' not found) restore pos

  size_t len = pos;     // nul or '.'
  if (len > maxlen) {   // Keep the name short
    pos = len = maxlen; // Move nul down
    dst[--pos] = '.';   // Insert dots
    dst[--pos] = '.';
    dst[--pos] = '.';
  }

  dst[len] = '\0';      // End it

  // Copy down to 0
  while (pos--) dst[pos] = src[pos];
}

void sdCardUp() {
  card.cdup();
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

void sdCardFolder(char * const dirname) {
  card.cd(dirname);
  DWIN_lcd_sd_status = false; // On next DWIN_Update
}

void onClickSDItem() {
  const uint16_t hasUpDir = !card.flag.workDirIsRoot;
  if (hasUpDir && currentMenu->selected == 1) return sdCardUp();
  else {
    const uint16_t filenum = currentMenu->selected - 1 - hasUpDir;
    card.selectFileByIndexSorted(filenum);

    // Enter that folder!
    if (card.flag.filenameIsDir) return sdCardFolder(card.filename);

    if (card.fileIsBinary())
      return dwinPopupConfirm(ICON_Error, F("Please check filenames"), F("Only G-code can be printed"));
    else {
      dwinPrintHeader(card.longest_filename()); // Save filename
      return gotoConfirmToPrint();
    }
  }
}

#if ENABLED(SCROLL_LONG_FILENAMES)

  char shift_name[LONG_FILENAME_LENGTH + 1] = "";

  void drawSDItemShifted(uint8_t &shift) {
    // Shorten to the available space
    const size_t lastchar = shift + MENU_CHAR_LIMIT;
    const char c = shift_name[lastchar];
    shift_name[lastchar] = '\0';

    const uint8_t row = fileMenu->line();
    eraseMenuText(row);
    drawMenuLine(row, 0, &shift_name[shift]);

    shift_name[lastchar] = c;
  }

  void fileMenuIdle(bool reset=false) {
    static bool hasUpDir = false;
    static uint8_t last_itemselected = 0;
    static int8_t shift_amt = 0, shift_len = 0;
    if (reset) {
      last_itemselected = 0;
      hasUpDir = !card.flag.workDirIsRoot; // Is a SubDir
      return;
    }
    const uint8_t selected = fileMenu->selected;
    if (last_itemselected != selected) {
      if (last_itemselected >= 1 + hasUpDir) fileMenu->items()[last_itemselected]->redraw(true);
      last_itemselected = selected;
      if (selected >= 1 + hasUpDir) {
        const int8_t filenum = selected - 1 - hasUpDir; // Skip "Back" and ".."
        card.selectFileByIndexSorted(filenum);
        makeNameWithoutExt(shift_name, card.longest_filename(), LONG_FILENAME_LENGTH);
        shift_len = strlen(shift_name);
        shift_amt = 0;
      }
    }
    else if ((selected >= 1 + hasUpDir) && (shift_len > MENU_CHAR_LIMIT)) {
      uint8_t shift_new = _MIN(shift_amt + 1, shift_len - MENU_CHAR_LIMIT); // Try to shift by...
      drawSDItemShifted(shift_new);               // Draw the item
      if (shift_new == shift_amt)                 // Scroll reached the end
        shift_new = -1;                           // Reset
      shift_amt = shift_new;                      // Set new scroll
    }
  }

#else // !SCROLL_LONG_FILENAMES

  char shift_name[FILENAME_LENGTH + 1] = "";

#endif

void onDrawFileName(MenuItem* menuitem, int8_t line) {
  const bool is_subdir = !card.flag.workDirIsRoot;
  if (is_subdir && menuitem->pos == 1) {
    drawMenuLine(line, ICON_Folder, "..");
  }
  else {
    uint8_t icon;
    card.selectFileByIndexSorted(menuitem->pos - is_subdir - 1);
    makeNameWithoutExt(shift_name, card.longest_filename());
    icon = card.flag.filenameIsDir ? ICON_Folder : card.fileIsBinary() ? ICON_Binary : ICON_File;
    drawMenuLine(line, icon, shift_name);
  }
}

void drawPrintFileMenu() {
  checkkey = ID_Menu;
  if (card.isMounted()) {
    if (SET_MENU(fileMenu, MSG_MEDIA_MENU, nr_sd_menu_items() + 1)) {
      BACK_ITEM(gotoMainMenu);
      for (uint8_t i = 0; i < nr_sd_menu_items(); ++i)
        menuItemAdd(onDrawFileName, onClickSDItem);
    }
    updateMenu(fileMenu);
    TERN_(DASH_REDRAW, dwinRedrawDash());
  }
  else {
    if (SET_MENU(fileMenu, MSG_MEDIA_MENU, 1)) BACK_ITEM(gotoMainMenu);
    updateMenu(fileMenu);
    dwinDrawRectangle(1, hmiData.colorAlertBg, 10, MBASE(3) - 10, DWIN_WIDTH - 10, MBASE(4));
    DWINUI::drawCenteredString(font12x24, hmiData.colorAlertTxt, MBASE(3), GET_TEXT_F(MSG_MEDIA_NOT_INSERTED));
  }
  TERN_(SCROLL_LONG_FILENAMES, fileMenuIdle(true));
}

//
// Watch for media mount / unmount
//
void hmiSDCardUpdate() {
  if (hmiFlag.home_flag) return;
  if (DWIN_lcd_sd_status != card.isMounted()) {
    DWIN_lcd_sd_status = card.isMounted();
    resetMenu(fileMenu);
    if (isMenu(fileMenu)) {
      currentMenu = nullptr;
      drawPrintFileMenu();
    }
    if (!DWIN_lcd_sd_status && sdPrinting()) ExtUI::stopPrint();  // Media removed while printing
  }
}

/**
 * Dash board and indicators
 */

void dwinDrawDashboard() {
  dwinDrawRectangle(1, hmiData.colorBackground, 0, STATUS_Y + 21, DWIN_WIDTH, DWIN_HEIGHT - 1);
  dwinDrawRectangle(1, hmiData.colorSplitLine, 0, 449, DWIN_WIDTH, 451);

  DWINUI::drawIcon(ICON_MaxSpeedX,  10, 454);
  DWINUI::drawIcon(ICON_MaxSpeedY,  95, 454);
  DWINUI::drawIcon(ICON_MaxSpeedZ, 180, 454);
  _drawXYZPosition(true);

  #if HAS_HOTEND
    DWINUI::drawIcon(ICON_HotendTemp, 10, 383);
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 28, 384, thermalManager.wholeDegHotend(0));
    DWINUI::drawString(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 25 + 3 * STAT_CHR_W + 5, 384, F("/"));
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 25 + 4 * STAT_CHR_W + 6, 384, thermalManager.degTargetHotend(0));

    DWINUI::drawIcon(ICON_StepE, 113, 416);
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 116 + 2 * STAT_CHR_W, 417, planner.flow_percentage[0]);
    DWINUI::drawString(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 116 + 5 * STAT_CHR_W + 2, 417, F("%"));
  #endif

  #if HAS_HEATED_BED
    DWINUI::drawIcon(ICON_SetBedTemp, 10, 416);
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 28, 417, thermalManager.wholeDegBed());
    DWINUI::drawString(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 25 + 3 * STAT_CHR_W + 5, 417, F("/"));
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 25 + 4 * STAT_CHR_W + 6, 417, thermalManager.degTargetBed());
  #endif

  DWINUI::drawIcon(ICON_Speed, 113, 383);
  DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 116 + 2 * STAT_CHR_W, 384, feedrate_percentage);
  IF_DISABLED(SHOW_SPEED_IND, DWINUI::drawString(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 116 + 5 * STAT_CHR_W + 2, 384, F("%")));

  #if HAS_FAN
    DWINUI::drawIcon(ICON_FanSpeed, 187, 383);
    DWINUI::drawInt(DWIN_FONT_STAT, hmiData.colorIndicator, hmiData.colorBackground, 3, 195 + 2 * STAT_CHR_W, 384, thermalManager.fan_speed[0]);
  #endif

  #if HAS_ZOFFSET_ITEM
    DWINUI::drawIcon(planner.leveling_active ? ICON_SetZOffset : ICON_Zoffset, 187, 416);
    DWINUI::drawSignedFloat(DWIN_FONT_STAT, hmiData.colorIndicator,  hmiData.colorBackground, 2, 2, 204, 417, BABY_Z_VAR);
  #endif
}

void drawInfoMenu() {
  DWINUI::clearMainArea();
  if (hmiIsChinese())
    title.frameCopy(30, 17, 28, 13);                      // "Info"
  else
    title.showCaption(GET_TEXT_F(MSG_INFO_SCREEN));
  drawMenuLine(0, ICON_Back, GET_TEXT_F(MSG_BACK), false, true);

  if (hmiIsChinese()) {
    dwinFrameAreaCopy(1, 197, 149, 252, 161, 108, 102);   // "Size"
    dwinFrameAreaCopy(1,   1, 164,  56, 176, 108, 175);   // "Firmware Version"
    dwinFrameAreaCopy(1,  58, 164, 113, 176, 105, 248);   // "Contact Details"
    DWINUI::drawCenteredString(268, F(CORP_WEBSITE));
  }
  else {
    DWINUI::drawCenteredString(102, F("Size"));
    DWINUI::drawCenteredString(175, F("Firmware version"));
    DWINUI::drawCenteredString(248, F("Build Datetime"));
    DWINUI::drawCenteredString(268, F(STRING_DISTRIBUTION_DATE));
  }
  DWINUI::drawCenteredString(122, F(MACHINE_SIZE));
  DWINUI::drawCenteredString(195, F(SHORT_BUILD_VERSION));

  for (uint8_t i = 0; i < 3; ++i) {
    DWINUI::drawIcon(ICON_PrintSize + i, ICOX, 99 + i * 73);
    dwinDrawHLine(hmiData.colorSplitLine, 16, MBASE(2) + i * 73, 240);
  }
}

// Main Process
void hmiMainMenu() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;

  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_page.inc(PAGE_COUNT)) {
      switch (select_page.now) {
        case PAGE_PRINT: ICON_Print(); break;
        case PAGE_PREPARE: ICON_Print(); ICON_Prepare(); break;
        case PAGE_CONTROL: ICON_Prepare(); ICON_Control(); break;
        case PAGE_ADVANCE: ICON_Control(); ICON_AdvSettings(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_page.dec()) {
      switch (select_page.now) {
        case PAGE_PRINT: ICON_Print(); ICON_Prepare(); break;
        case PAGE_PREPARE: ICON_Prepare(); ICON_Control(); break;
        case PAGE_CONTROL: ICON_Control(); ICON_AdvSettings(); break;
        case PAGE_ADVANCE: ICON_AdvSettings(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_page.now) {
      case PAGE_PRINT:
        if (hmiData.mediaAutoMount) {
          card.mount();
          safe_delay(800);
        }
        drawPrintFileMenu();
        break;
      case PAGE_PREPARE: drawPrepareMenu(); break;
      case PAGE_CONTROL: drawControlMenu(); break;
      case PAGE_ADVANCE: drawAdvancedSettingsMenu(); break;
    }
  }
  dwinUpdateLCD();
}

// Pause or Stop popup
void onClickPauseOrStop() {
  switch (select_print.now) {
    case PRINT_PAUSE_RESUME: if (hmiFlag.select_flag) ExtUI::pausePrint(); break; // Confirm pause
    case PRINT_STOP: if (hmiFlag.select_flag) ExtUI::stopPrint(); break; // Stop confirmed then abort print
    default: break;
  }
  return gotoPrintProcess();
}

// Printing
void hmiPrinting() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState == ENCODER_DIFF_NO) return;
  // Avoid flicker by updating only the previous menu
  if (encoder_diffState == ENCODER_DIFF_CW) {
    if (select_print.inc(PRINT_COUNT)) {
      switch (select_print.now) {
        case PRINT_SETUP: ICON_Tune(); break;
        case PRINT_PAUSE_RESUME: ICON_Tune(); ICON_ResumeOrPause(); break;
        case PRINT_STOP: ICON_ResumeOrPause(); ICON_Stop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_CCW) {
    if (select_print.dec()) {
      switch (select_print.now) {
        case PRINT_SETUP: ICON_Tune(); ICON_ResumeOrPause(); break;
        case PRINT_PAUSE_RESUME: ICON_ResumeOrPause(); ICON_Stop(); break;
        case PRINT_STOP: ICON_Stop(); break;
      }
    }
  }
  else if (encoder_diffState == ENCODER_DIFF_ENTER) {
    switch (select_print.now) {
      case PRINT_SETUP: drawTuneMenu(); break;
      case PRINT_PAUSE_RESUME:
        if (printingIsPaused()) {  // If printer is already in pause
          ExtUI::resumePrint();
          break;
        }
        else
          return gotoPopup(popupPauseOrStop, onClickPauseOrStop);
      case PRINT_STOP:
        return gotoPopup(popupPauseOrStop, onClickPauseOrStop);
      default: break;
    }
  }
  dwinUpdateLCD();
}

#include "../../../libs/buzzer.h"

void drawMainArea() {
  switch (checkkey) {
    case ID_MainMenu:         drawMainMenu(); break;
    case ID_PrintProcess:     drawPrintProcess(); break;
    case ID_PrintDone:        drawPrintDone(); break;
    #if HAS_ESDIAG
      case ID_ESDiagProcess:  drawEndStopDiag(); break;
    #endif
    #if ENABLED(PROUI_ITEM_PLOT)
      case ID_PlotProcess:
        switch (hmiValue.tempControl) {
          #if ENABLED(PIDTEMP)
            case PIDTEMP_START: drawHPlot(); break;
          #endif
          #if ENABLED(PIDTEMPBED)
            case PIDTEMPBED_START: drawBPlot(); break;
          #endif
          #if ENABLED(PIDTEMPCHAMBER)
            case PIDTEMPCHAMBER_START: drawCPlot(); break;
          #endif
        } break;
    #endif
    case ID_Popup:            popupDraw(); break;
    #if HAS_LOCKSCREEN
      case ID_Locked:         lockScreen.draw(); break;
    #endif
    case ID_Menu:
    case ID_SetInt:
    case ID_SetPInt:
    case ID_SetIntNoDraw:
    case ID_SetFloat:
    case ID_SetPFloat:        ReDrawMenu(true); break;
    default: break;
  }
}

void hmiWaitForUser() {
  EncoderState encoder_diffState = get_encoder_state();
  if (encoder_diffState != ENCODER_DIFF_NO && !ui.backlight) {
    if (checkkey == ID_WaitResponse) hmiReturnScreen();
    return ui.refresh_brightness();
  }
  if (!wait_for_user) {
    switch (checkkey) {
      case ID_PrintDone: select_page.reset(); gotoMainMenu(); break;
      default: hmiReturnScreen(); break;
    }
  }
}

void hmiInit() {
  #if ENABLED(SHOW_BOOTSCREEN)
    #ifndef BOOTSCREEN_TIMEOUT
      #define BOOTSCREEN_TIMEOUT 1100
    #endif
    DWINUI::drawBox(1, COLOR_BLACK, { 5, 220, DWIN_WIDTH - 5, DWINUI::fontHeight() });
    DWINUI::drawCenteredString(COLOR_WHITE, 220, F("ProUI starting up "));
    for (uint16_t t = 15; t < 257; t += 11) {
      DWINUI::drawIcon(ICON_Bar, 15, 260);
      dwinDrawRectangle(1, hmiData.colorBackground, t, 260, 257, 280);
      dwinUpdateLCD();
      safe_delay((BOOTSCREEN_TIMEOUT) / 22);
    }
  #endif
  hmiSetLanguage();
}

void eachMomentUpdate() {
  static millis_t next_var_update_ms = 0, next_rts_update_ms = 0, next_status_update_ms = 0;
  const millis_t ms = millis();

  #if HAS_BACKLIGHT_TIMEOUT
    if (ui.backlight_off_ms && ELAPSED(ms, ui.backlight_off_ms)) {
      turnOffBacklight(); // Backlight off
      ui.backlight_off_ms = 0;
    }
  #endif

  if (ELAPSED(ms, next_var_update_ms)) {
    next_var_update_ms = ms + DWIN_VAR_UPDATE_INTERVAL;
    FLIP(blink);
    updateVariable();
    #if HAS_ESDIAG
      if (checkkey == ID_ESDiagProcess) esDiag.update();
    #endif
    #if PROUI_TUNING_GRAPH
      if (checkkey == ID_PIDProcess) {
        TERN_(PIDTEMP, if (hmiValue.tempControl == PIDTEMP_START) plot.update(thermalManager.wholeDegHotend(0)));
        TERN_(PIDTEMPBED, if (hmiValue.tempControl == PIDTEMPBED_START) plot.update(thermalManager.wholeDegBed()));
        TERN_(PIDTEMPCHAMBER, if (hmiValue.tempControl == PIDTEMPCHAMBER_START) plot.update(thermalManager.wholeDegChamber()));
      }
      TERN_(MPCTEMP, if (checkkey == ID_MPCProcess) plot.update(thermalManager.wholeDegHotend(0)));
      #if ENABLED(PROUI_ITEM_PLOT)
        if (checkkey == ID_PlotProcess) {
          TERN_(PIDTEMP, if (hmiValue.tempControl == PIDTEMP_START) { plot.update(thermalManager.wholeDegHotend(0)); })
          TERN_(PIDTEMPBED, if (hmiValue.tempControl == PIDTEMPBED_START) { plot.update(thermalManager.wholeDegBed()); })
          TERN_(PIDTEMPCHAMBER, if (hmiValue.tempControl == PIDTEMPCHAMBER_START) { plot.update(thermalManager.wholeDegChamber()); })
          TERN_(MPCTEMP, if (hmiValue.tempControl == MPC_STARTED) { plot.update(thermalManager.wholeDegHotend(0)); })
          if (hmiFlag.abort_flag || hmiFlag.pause_flag || print_job_timer.isPaused()) {
            hmiReturnScreen();
          }
        }
      #endif
    #endif
  }

  #if HAS_STATUS_MESSAGE_TIMEOUT
    bool did_expire = ui.status_reset_callback && (*ui.status_reset_callback)();
    did_expire |= ui.status_message_expire_ms && ELAPSED(ms, ui.status_message_expire_ms);
    if (did_expire) ui.reset_status();
  #endif

  if (ELAPSED(ms, next_status_update_ms)) {
    next_status_update_ms = ms + DWIN_VAR_UPDATE_INTERVAL;
    dwinDrawStatusMessage();
    #if ENABLED(SCROLL_LONG_FILENAMES)
      if (isMenu(fileMenu)) fileMenuIdle();
    #endif
  }

  if (ELAPSED(ms, next_rts_update_ms)) {
    next_rts_update_ms = ms + DWIN_UPDATE_INTERVAL;

    if ((isPrinting() != hmiFlag.printing_flag) && !hmiFlag.home_flag) {
      hmiFlag.printing_flag = isPrinting();
      if (hmiFlag.printing_flag)
        dwinPrintStarted();
      else if (hmiFlag.abort_flag)
        dwinPrintAborted();
      else
        dwinPrintFinished();
    }

    if ((hmiFlag.pause_flag != printingIsPaused()) && !hmiFlag.home_flag) {
      hmiFlag.pause_flag = printingIsPaused();
      if (hmiFlag.pause_flag)
        dwinPrintPause();
      else if (hmiFlag.abort_flag)
        dwinPrintAborted();
      else
        dwinPrintResume();
    }

    if (checkkey == ID_PrintProcess) { // Print process

      // Progress percent
      static uint8_t _percent_done = 255;
      if (_percent_done != ui.get_progress_percent()) {
        _percent_done = ui.get_progress_percent();
        drawPrintProgressBar();
      }

      // Remaining time
      #if ENABLED(SHOW_REMAINING_TIME)
        if (_remain_time != ui.get_remaining_time()) {
          _remain_time = ui.get_remaining_time();
          drawPrintProgressRemain();
        }
      #endif

      // Elapsed print time
      static uint16_t _printtime = 0;
      const uint16_t min = (print_job_timer.duration() % 3600) / 60;
      if (_printtime != min) { // 1 minute update
        _printtime = min;
        drawPrintProgressElapsed();
      }
    }
    #if HAS_PLR_UI_FLAG
      else if (DWIN_lcd_sd_status && recovery.ui_flag_resume) { // Resume interrupted print
        return gotoPowerLossRecovery();
      }
    #endif

    dwinUpdateLCD();
  }
}

#if ENABLED(POWER_LOSS_RECOVERY)

  void popupPowerLossRecovery() {
    DWINUI::clearMainArea();
    drawPopupBkgd();
    if (hmiIsChinese()) {
      dwinFrameAreaCopy(1, 160, 338, 235, 354, 98, 115);
      dwinFrameAreaCopy(1, 103, 321, 271, 335, 52, 167);
      DWINUI::drawIconWB(ICON_Cancel_C,    26, 280);
      DWINUI::drawIconWB(ICON_Continue_C, 146, 280);
    }
    else {
      DWINUI::drawCenteredString(hmiData.colorPopupTxt, 70, GET_TEXT_F(MSG_OUTAGE_RECOVERY));
      DWINUI::drawCenteredString(hmiData.colorPopupTxt, 147, F("It looks like the last"));
      DWINUI::drawCenteredString(hmiData.colorPopupTxt, 167, F("file was interrupted."));
      DWINUI::drawButton(BTN_Cancel,    26, 280);
      DWINUI::drawButton(BTN_Continue, 146, 280);
    }
    MediaFile *dir = nullptr;
    const char * const filename = card.diveToFile(true, dir, recovery.info.sd_filename);
    card.selectFileByName(filename);
    DWINUI::drawCenteredString(hmiData.colorPopupTxt, 207, card.longest_filename());
    dwinPrintHeader(card.longest_filename()); // Save filename
    drawSelectHighlight(hmiFlag.select_flag);
    dwinUpdateLCD();
  }

  void onClickPowerLossRecovery() {
    if (hmiFlag.select_flag) {
      queue.inject(F("M1000C"));
      select_page.reset();
      return gotoMainMenu();
    }
    else {
      hmiSaveProcessID(ID_NothingToDo);
      select_print.set(PRINT_SETUP);
      queue.inject(F("M1000"));
    }
  }

  void gotoPowerLossRecovery() {
    recovery.ui_flag_resume = false;
    LCD_MESSAGE(MSG_CONTINUE_PRINT_JOB);
    gotoPopup(popupPowerLossRecovery, onClickPowerLossRecovery);
  }

#endif // POWER_LOSS_RECOVERY

void dwinHandleScreen() {
  switch (checkkey) {
    case ID_MainMenu:     hmiMainMenu(); break;
    case ID_Menu:         hmiMenu(); break;
    case ID_SetInt:       hmiSetDraw(); break;
    case ID_SetFloat:     hmiSetDraw(); break;
    case ID_SetPInt:      hmiSetPInt(); break;
    case ID_SetPFloat:    hmiSetPFloat(); break;
    case ID_SetIntNoDraw: hmiSetNoDraw(); break;
    case ID_PrintProcess: hmiPrinting(); break;
    case ID_Popup:        hmiPopup(); break;
    #if HAS_LOCKSCREEN
      case ID_Locked: hmiLockScreen(); break;
    #endif
    TERN_(HAS_ESDIAG, case ID_ESDiagProcess:)
    TERN_(PROUI_ITEM_PLOT, case ID_PlotProcess:)
    case ID_PrintDone:
    case ID_WaitResponse: hmiWaitForUser(); break;

    TERN_(HAS_BED_PROBE, case ID_Leveling:)
    case ID_Homing:
    TERN_(HAS_PID_HEATING, case ID_PIDProcess:)
    TERN_(MPCTEMP, case ID_MPCProcess:)
    case ID_NothingToDo:
    default: break;
  }
}

bool idIsPopUp() {    // If ID is popup...
  switch (checkkey) {
    TERN_(HAS_BED_PROBE, case ID_Leveling:)
    TERN_(HAS_ESDIAG, case ID_ESDiagProcess:)
    case ID_NothingToDo:
    case ID_WaitResponse:
    case ID_Popup:
    case ID_Homing:
    TERN_(HAS_PID_HEATING, case ID_PIDProcess:)
    TERN_(MPCTEMP, case ID_MPCProcess:)
    TERN_(PROUI_ITEM_PLOT, case ID_PlotProcess:)
      return true;
    default: break;
  }
  return false;
}

void hmiSaveProcessID(const uint8_t id) {
  if (checkkey == id) return;
  if (!idIsPopUp()) last_checkkey = checkkey; // If previous is not a popup
  checkkey = id;
  switch (id) {
    case ID_Popup:
    case ID_WaitResponse:
    case ID_PrintDone:
    TERN_(HAS_BED_PROBE, case ID_Leveling:)
    TERN_(HAS_ESDIAG, case ID_ESDiagProcess:)
    TERN_(PROUI_ITEM_PLOT, case ID_PlotProcess:)
      wait_for_user = true;
    default: break;
  }
}

void hmiReturnScreen() {
  checkkey = last_checkkey;
  wait_for_user = false;
  drawMainArea();
}

void dwinHomingStart() {
  hmiFlag.home_flag = true;
  hmiSaveProcessID(ID_Homing);
  title.showCaption(GET_TEXT_F(MSG_HOMING));
  #if ANY(TJC_DISPLAY, DACAI_DISPLAY)
    dwinShowPopup(ICON_BLTouch, GET_TEXT_F(MSG_HOMING), GET_TEXT_F(MSG_PLEASE_WAIT));
  #else
    dwinShowPopup(ICON_Printer_0, GET_TEXT_F(MSG_HOMING), GET_TEXT_F(MSG_PLEASE_WAIT));
  #endif
}

void dwinHomingDone() {
  hmiFlag.home_flag = false;
  if (last_checkkey == ID_PrintDone)
    gotoPrintDone();
  else
    hmiReturnScreen();
}

void dwinLevelingStart() {
  #if HAS_BED_PROBE
    hmiSaveProcessID(ID_Leveling);
    title.showCaption(GET_TEXT_F(MSG_BED_LEVELING));
    dwinShowPopup(ICON_AutoLeveling, GET_TEXT_F(MSG_BED_LEVELING), GET_TEXT_F(MSG_PLEASE_WAIT));
    #if ALL(AUTO_BED_LEVELING_UBL, PREHEAT_BEFORE_LEVELING)
      if (!DEBUGGING(DRYRUN)) probe.preheat_for_probing(LEVELING_NOZZLE_TEMP, hmiData.bedLevT);
    #endif
  #elif ENABLED(MESH_BED_LEVELING)
    drawManualMeshMenu();
  #endif
}

void dwinLevelingDone() {
  TERN_(HAS_MESH, gotoMeshViewer(true));
}

#if HAS_MESH
  void dwinMeshUpdate(const int8_t cpos, const int8_t tpos, const_float_t zval) {
    ui.set_status(
      &MString<32>(GET_TEXT_F(MSG_PROBING_POINT), ' ', cpos, '/', tpos, F(" Z="), p_float_t(zval, 2))
    );
  }
#endif

// PID/MPC process

#if PROUI_TUNING_GRAPH

  #include "plot.h"

  celsius_t _maxtemp, _target;
  void dwinDrawPIDMPCPopup() {
    constexpr frame_rect_t gfrm = { 30, 150, DWIN_WIDTH - 60, 160 };
    DWINUI::clearMainArea();
    drawPopupBkgd();

    switch (hmiValue.tempControl) {
      default: return;
      #if ENABLED(MPC_AUTOTUNE)
        case MPC_STARTED:
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 70, GET_TEXT_F(MSG_MPC_AUTOTUNE));
          DWINUI::drawString(hmiData.colorPopupTxt, gfrm.x, gfrm.y - DWINUI::fontHeight() - 4, F("MPC target:     Celsius"));
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 92, GET_TEXT_F(MSG_PID_FOR_NOZZLE));
          _maxtemp = thermalManager.hotend_maxtemp[0];
          _target = 200;
          break;
      #endif
      #if ENABLED(PIDTEMP)
        case PIDTEMP_START:
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 70, GET_TEXT_F(MSG_PID_AUTOTUNE));
          DWINUI::drawString(hmiData.colorPopupTxt, gfrm.x, gfrm.y - DWINUI::fontHeight() - 4, F("PID target:     Celsius"));
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 92, GET_TEXT_F(MSG_PID_FOR_NOZZLE));
          _maxtemp = thermalManager.hotend_maxtemp[0];
          _target = hmiData.hotendPIDT;
          break;
      #endif
      #if ENABLED(PIDTEMPBED)
        case PIDTEMPBED_START:
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 70, GET_TEXT_F(MSG_PID_AUTOTUNE));
          DWINUI::drawString(hmiData.colorPopupTxt, gfrm.x, gfrm.y - DWINUI::fontHeight() - 4, F("PID target:     Celsius"));
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 92, GET_TEXT_F(MSG_PID_FOR_BED));
          _maxtemp = BED_MAXTEMP;
          _target = hmiData.bedPIDT;
          break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case PIDTEMPCHAMBER_START:
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 70, GET_TEXT_F(MSG_PID_AUTOTUNE));
          DWINUI::drawString(hmiData.colorPopupTxt, gfrm.x, gfrm.y - DWINUI::fontHeight() - 4, F("PID target:     Celsius"));
          DWINUI::drawCenteredString(hmiData.colorPopupTxt, 92, GET_TEXT_F(MSG_PID_FOR_CHAMBER));
          _maxtemp = CHAMBER_MAXTEMP;
          _target = hmiData.chamberPIDT;
          break;
      #endif
    }

    plot.draw(gfrm, _maxtemp, _target);
    DWINUI::drawInt(false, 2, hmiData.colorStatusTxt, hmiData.colorPopupTxt, 3, gfrm.x + 92, gfrm.y - DWINUI::fontHeight() - 6, _target);
  }

  // Plot Temperature Graph (PID Tuning Graph)
  #if ENABLED(PROUI_ITEM_PLOT)

    void dwinDrawPlot(tempcontrol_t result) {
      hmiValue.tempControl = result;
      constexpr frame_rect_t gfrm = { 30, 135, DWIN_WIDTH - 60, 160 };
      DWINUI::clearMainArea();
      drawPopupBkgd();
      hmiSaveProcessID(ID_PlotProcess);

      switch (result) {
        #if ENABLED(MPCTEMP)
          case MPC_STARTED:
        #elif ENABLED(PIDTEMP)
          case PIDTEMP_START:
        #endif
            title.showCaption(GET_TEXT_F(MSG_HOTEND_TEMP_GRAPH));
            DWINUI::drawCenteredString(3, hmiData.colorPopupTxt, 75, GET_TEXT_F(MSG_TEMP_NOZZLE));
            _maxtemp = thermalManager.hotend_max_target(0);
            _target = thermalManager.degTargetHotend(0);
            break;
        #if ENABLED(PIDTEMPBED)
          case PIDTEMPBED_START:
            title.showCaption(GET_TEXT_F(MSG_BED_TEMP_GRAPH));
            DWINUI::drawCenteredString(3, hmiData.colorPopupTxt, 75, GET_TEXT_F(MSG_TEMP_BED));
            _maxtemp = BED_MAX_TARGET;
            _target = thermalManager.degTargetBed();
            break;
        #endif
        #if ENABLED(PIDTEMPCHAMBER)
          case PIDTEMPCHAMBER_START:
            title.showCaption(GET_TEXT_F(MSG_CHAMBER_TEMP_GRAPH));
            DWINUI::drawCenteredString(3, hmiData.colorPopupTxt, 75, GET_TEXT_F(MSG_TEMP_CHAMBER));
            _maxtemp = CHAMBER_MAX_TARGET;
            _target = thermalManager.degTargetChamber();
            break;
        #endif
        default: break;
      }

      dwinDrawString(false, 2, hmiData.colorPopupTxt, hmiData.colorPopupBg, gfrm.x, gfrm.y - DWINUI::fontHeight() - 4, F("Target:     Celsius"));
      plot.draw(gfrm, _maxtemp, _target);
      DWINUI::drawInt(false, 2, hmiData.colorStatusTxt, hmiData.colorPopupBg, 3, gfrm.x + 80, gfrm.y - DWINUI::fontHeight() - 4, _target);
      DWINUI::drawButton(BTN_Continue, 86, 305);
    }

    void drawHPlot() {
      TERN_(PIDTEMP, dwinDrawPlot(PIDTEMP_START));
      TERN_(MPCTEMP, dwinDrawPlot(MPC_STARTED));
    }
    void drawBPlot() {
      TERN_(PIDTEMPBED, dwinDrawPlot(PIDTEMPBED_START));
    }
    void drawCPlot() {
      TERN_(PIDTEMPCHAMBER, dwinDrawPlot(PIDTEMPCHAMBER_START));
    }

  #endif // PROUI_ITEM_PLOT

#endif // PROUI_TUNING_GRAPH

#if HAS_PID_HEATING

  void dwinStartM303(const int count, const heater_id_t hid, const celsius_t temp) {
    hmiData.pidCycles = count;
    switch (hid) {
      #if ENABLED(PIDTEMP)
        case 0 ... HOTENDS - 1: hmiData.hotendPIDT = temp; break;
      #endif
      #if ENABLED(PIDTEMPBED)
        case H_BED: hmiData.bedPIDT = temp; break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case H_CHAMBER: hmiData.chamberPIDT = temp; break;
      #endif
      default: break;
    }
  }

  void dwinPIDTuning(tempcontrol_t result) {
    hmiValue.tempControl = result;
    switch (result) {
      #if ENABLED(PIDTEMP)
        case PIDTEMP_START:
          hmiSaveProcessID(ID_PIDProcess);
          #if PROUI_TUNING_GRAPH
            dwinDrawPIDMPCPopup();
          #else
            dwinDrawPopup(ICON_TempTooHigh, GET_TEXT_F(MSG_PID_AUTOTUNE), GET_TEXT_F(MSG_PID_FOR_NOZZLE));
          #endif
          break;
      #endif
      #if ENABLED(PIDTEMPBED)
        case PIDTEMPBED_START:
          hmiSaveProcessID(ID_PIDProcess);
          dwinDrawPopup(ICON_TempTooHigh, GET_TEXT_F(MSG_PID_AUTOTUNE), GET_TEXT_F(MSG_PID_FOR_BED));
          break;
      #endif
      #if ENABLED(PIDTEMPCHAMBER)
        case PIDTEMPCHAMBER_START:
          hmiSaveProcessID(ID_PIDProcess);
          dwinDrawPopup(ICON_TempTooHigh, GET_TEXT_F(MSG_PID_AUTOTUNE), GET_TEXT_F(MSG_PID_FOR_CHAMBER));
          break;
      #endif
      case PID_BAD_HEATER_ID:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooLow, GET_TEXT_F(MSG_PID_AUTOTUNE_FAILED), GET_TEXT_F(MSG_PID_BAD_HEATER_ID));
        break;
      case PID_TUNING_TIMEOUT:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooHigh, GET_TEXT_F(MSG_ERROR), GET_TEXT_F(MSG_PID_TIMEOUT));
        break;
      case PID_TEMP_TOO_HIGH:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooHigh, GET_TEXT_F(MSG_PID_AUTOTUNE_FAILED), GET_TEXT_F(MSG_TEMP_TOO_HIGH));
        break;
      case AUTOTUNE_DONE:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooLow, GET_TEXT_F(MSG_PID_AUTOTUNE), GET_TEXT_F(MSG_BUTTON_DONE));
        break;
      default:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_Info_0, GET_TEXT_F(MSG_ERROR), GET_TEXT_F(MSG_STOPPING));
        break;
    }
  }

#endif // HAS_PID_HEATING

#if ENABLED(MPC_AUTOTUNE)

  void dwinMPCTuning(tempcontrol_t result) {
    hmiValue.tempControl = result;
    switch (result) {
      case MPC_STARTED:
        hmiSaveProcessID(ID_MPCProcess);
        #if PROUI_TUNING_GRAPH
          dwinDrawPIDMPCPopup();
        #else
          dwinDrawPopup(ICON_TempTooHigh, GET_TEXT_F(MSG_MPC_AUTOTUNE), F("for Nozzle is running."));
        #endif
        break;
      case MPC_TEMP_ERROR:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooHigh, GET_TEXT_F(MSG_PID_AUTOTUNE_FAILED), F(STR_MPC_TEMPERATURE_ERROR));
        ui.reset_alert_level();
        break;
      case MPC_INTERRUPTED:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooHigh, GET_TEXT_F(MSG_ERROR), F(STR_MPC_AUTOTUNE_INTERRUPTED));
        ui.reset_alert_level();
        break;
      case AUTOTUNE_DONE:
        checkkey = last_checkkey;
        dwinPopupConfirm(ICON_TempTooLow, GET_TEXT_F(MSG_MPC_AUTOTUNE), GET_TEXT_F(MSG_BUTTON_DONE));
        ui.reset_alert_level();
        break;
      default:
        checkkey = last_checkkey;
        ui.reset_alert_level();
        break;
    }
  }

#endif // MPC_AUTOTUNE

// Started a Print Job
void dwinPrintStarted() {
  TERN_(HAS_GCODE_PREVIEW, if (hostPrinting()) preview.invalidate());
  TERN_(SET_PROGRESS_PERCENT, ui.progress_reset());
  TERN_(SET_REMAINING_TIME, ui.reset_remaining_time());
  hmiFlag.pause_flag = false;
  hmiFlag.abort_flag = false;
  select_print.reset();
  gotoPrintProcess();
}

// Pause a print job
void dwinPrintPause() {
  ICON_ResumeOrPause();
}

// Resume print job
void dwinPrintResume() {
  ICON_ResumeOrPause();
  LCD_MESSAGE(MSG_RESUME_PRINT);
}

// Ended print job
void dwinPrintFinished() {
  TERN_(POWER_LOSS_RECOVERY, if (card.isPrinting()) recovery.cancel());
  hmiFlag.abort_flag = false;
  hmiFlag.pause_flag = false;
  wait_for_heatup = false;
  planner.finish_and_disable();
  thermalManager.cooldown();
  gotoPrintDone();
}

// Print was aborted
void dwinPrintAborted() {
  #ifndef EVENT_GCODE_SD_ABORT
    if (ExtUI::isMachineHomed()) {
      queue.inject(
        #if ENABLED(NOZZLE_PARK_FEATURE)
          F("G27")
        #else
          TS(F("G0Z"), float(_MIN(current_position.z + (Z_POST_CLEARANCE), Z_MAX_POS)), F("\nG0F2000Y"), Y_MAX_POS)
        #endif
      );
    }
  #endif
  TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PRINT_ABORTED)));
  dwinPrintFinished();
}

#if HAS_FILAMENT_SENSOR
  // Filament Runout process
  void dwinFilamentRunout(const uint8_t extruder) { LCD_MESSAGE(MSG_RUNOUT_SENSOR); }
#endif

void dwinSetColorDefaults() {
  hmiData.colorBackground = defColorBackground;
  hmiData.colorCursor     = defColorCursor;
  hmiData.colorTitleBg    = defColorTitleBg;
  hmiData.colorTitleTxt   = defColorTitleTxt;
  hmiData.colorText       = defColorText;
  hmiData.colorSelected   = defColorSelected;
  hmiData.colorSplitLine  = defColorSplitLine;
  hmiData.colorHighlight  = defColorHighlight;
  hmiData.colorStatusBg   = defColorStatusBg;
  hmiData.colorStatusTxt  = defColorStatusTxt;
  hmiData.colorPopupBg    = defColorPopupBg;
  hmiData.colorPopupTxt   = defColorPopupTxt;
  hmiData.colorAlertBg    = defColorAlertBg;
  hmiData.colorAlertTxt   = defColorAlertTxt;
  hmiData.colorPercentTxt = defColorPercentTxt;
  hmiData.colorBarfill    = defColorBarfill;
  hmiData.colorIndicator  = defColorIndicator;
  hmiData.colorCoordinate = defColorCoordinate;
}

static_assert(ExtUI::eeprom_data_size >= sizeof(hmi_data_t), "Insufficient space in EEPROM for UI parameters");

void dwinSetDataDefaults() {
  dwinSetColorDefaults();
  DWINUI::setColors(hmiData.colorText, hmiData.colorBackground, hmiData.colorStatusBg);
  TERN_(PIDTEMP, hmiData.hotendPIDT = DEF_HOTENDPIDT);
  TERN_(PIDTEMPBED, hmiData.bedPIDT = DEF_BEDPIDT);
  TERN_(HAS_PID_HEATING, hmiData.pidCycles = DEF_PIDCYCLES);
  #if ENABLED(PREVENT_COLD_EXTRUSION)
    hmiData.extMinT = EXTRUDE_MINTEMP;
    applyExtMinT();
  #endif
  TERN_(PREHEAT_BEFORE_LEVELING, hmiData.bedLevT = LEVELING_BED_TEMP);
  TERN_(BAUD_RATE_GCODE, setBaud250K());
  #if ALL(LCD_BED_TRAMMING, HAS_BED_PROBE)
    hmiData.fullManualTramming = DISABLED(BED_TRAMMING_USE_PROBE);
  #endif
  #if ENABLED(PROUI_MEDIASORT)
    hmiData.mediaSort = true;
    card.setSortOn(TERN(SDSORT_REVERSE, AS_REV, AS_FWD));
  #endif
  hmiData.mediaAutoMount = ENABLED(HAS_SD_EXTENDER);
  #if ALL(INDIVIDUAL_AXIS_HOMING_SUBMENU, MESH_BED_LEVELING)
    hmiData.zAfterHoming = DEF_Z_AFTER_HOMING;
  #endif
  #if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
    TERN_(LED_COLOR_PRESETS, leds.set_default());
    applyLEDColor();
  #endif
  TERN_(HAS_GCODE_PREVIEW, hmiData.enablePreview = true);
}

void dwinCopySettingsTo(char * const buff) {
  memcpy(buff, &hmiData, sizeof(hmi_data_t));
}

void dwinCopySettingsFrom(const char * const buff) {
  memcpy(&hmiData, buff, sizeof(hmi_data_t));
  if (hmiData.colorText == hmiData.colorBackground) dwinSetColorDefaults();
  DWINUI::setColors(hmiData.colorText, hmiData.colorBackground, hmiData.colorStatusBg);
  TERN_(PREVENT_COLD_EXTRUSION, applyExtMinT());
  feedrate_percentage = 100;
  TERN_(BAUD_RATE_GCODE, hmiSetBaudRate());
  #if ALL(LED_CONTROL_MENU, HAS_COLOR_LEDS)
    leds.set_color(
      hmiData.ledColor.r,
      hmiData.ledColor.g,
      hmiData.ledColor.b
      OPTARG(HAS_WHITE_LED, hmiData.ledColor.w)
    );
    leds.update();
  #endif
}

// Initialize or re-initialize the LCD
void MarlinUI::init_lcd() {
  delay(750);   // Wait to wakeup screen
  const bool hs = dwinHandshake(); UNUSED(hs);
  dwinFrameSetDir(1);
  dwinJPGCacheTo1(Language_English);
}

void MarlinUI::clear_lcd() {}

void dwinInitScreen() {
  dwinSetColorDefaults();
  hmiInit();   // Draws boot screen
  DWINUI::init();
  DWINUI::setColors(hmiData.colorText, hmiData.colorBackground, hmiData.colorStatusBg);
  DWINUI::onTitleDraw = drawTitle;
  initMenu();
  checkkey = 255;
  hash_changed = true;
  dwinDrawStatusLine();
  dwinDrawDashboard();
  gotoMainMenu();
}

void MarlinUI::update() {
  hmiSDCardUpdate();  // SD card update
  eachMomentUpdate(); // Status update
  dwinHandleScreen(); // Rotary encoder update
}

#if HAS_LCD_BRIGHTNESS
  void MarlinUI::_set_brightness() { dwinLCDBrightness(backlight ? brightness : 0); }
#endif

void MarlinUI::kill_screen(FSTR_P const lcd_error, FSTR_P const) {
  #if ANY(TJC_DISPLAY, DACAI_DISPLAY)
    dwinDrawPopup(ICON_BLTouch, GET_TEXT_F(MSG_PRINTER_KILLED), lcd_error);
  #else
    dwinDrawPopup(ICON_Printer_0, GET_TEXT_F(MSG_PRINTER_KILLED), lcd_error);
  #endif
  DWINUI::drawCenteredString(hmiData.colorPopupTxt, 270, GET_TEXT_F(MSG_TURN_OFF));
  dwinUpdateLCD();
}

void dwinRebootScreen() {
  dwinFrameClear(COLOR_BG_BLACK);
  dwinJPGShowAndCache(0);
  DWINUI::drawCenteredString(COLOR_WHITE, 220, GET_TEXT_F(MSG_PLEASE_WAIT_REBOOT));
  dwinUpdateLCD();
  safe_delay(500);
}

void dwinRedrawDash() {
  hash_changed = true;
  dwinDrawStatusMessage();
  dwinDrawDashboard();
}

void dwinRedrawScreen() {
  drawMainArea();
  dwinRedrawDash();
}

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  void dwinPopupPause(FSTR_P const fmsg, uint8_t button/*=0*/) {
    hmiSaveProcessID(button ? ID_WaitResponse : ID_NothingToDo);
    dwinShowPopup(ICON_Pause_1, GET_TEXT_F(MSG_ADVANCED_PAUSE), fmsg, button);
  }

  void drawPopupFilamentPurge() {
    dwinDrawPopup(ICON_AutoLeveling, GET_TEXT_F(MSG_ADVANCED_PAUSE), GET_TEXT_F(MSG_FILAMENT_CHANGE_PURGE_CONTINUE));
    DWINUI::drawButton(BTN_Purge, 26, 280);
    DWINUI::drawButton(BTN_Continue, 146, 280);
    drawSelectHighlight(true);
  }

  void onClickFilamentPurge() {
    if (hmiFlag.select_flag)
      pause_menu_response = PAUSE_RESPONSE_EXTRUDE_MORE;  // "Purge More" button
    else {
      hmiSaveProcessID(ID_NothingToDo);
      pause_menu_response = PAUSE_RESPONSE_RESUME_PRINT;  // "Continue" button
    }
  }

  void gotoFilamentPurge() {
    pause_menu_response = PAUSE_RESPONSE_WAIT_FOR;
    gotoPopup(drawPopupFilamentPurge, onClickFilamentPurge);
  }

#endif // ADVANCED_PAUSE_FEATURE

#if HAS_MESH
  void dwinMeshViewer() {
    if (!leveling_is_valid())
      dwinPopupContinue(ICON_Leveling_1, GET_TEXT_F(MSG_MESH_VIEWER), GET_TEXT_F(MSG_NO_VALID_MESH));
    else {
      hmiSaveProcessID(ID_WaitResponse);
      meshViewer.draw();
    }
  }
#endif

#if HAS_LOCKSCREEN

  void dwinLockScreen() {
    if (checkkey != ID_Locked) {
      lockScreen.rprocess = checkkey;
      checkkey = ID_Locked;
      lockScreen.init();
    }
  }

  void dwinUnLockScreen() {
    if (checkkey == ID_Locked) {
      checkkey = lockScreen.rprocess;
      drawMainArea();
    }
  }

  void hmiLockScreen() {
    EncoderState encoder_diffState = get_encoder_state();
    if (encoder_diffState == ENCODER_DIFF_NO) return;
    lockScreen.onEncoder(encoder_diffState);
    if (lockScreen.isUnlocked()) dwinUnLockScreen();
  }

#endif // HAS_LOCKSCREEN

#if HAS_GCODE_PREVIEW

  void setPreview() { toggleCheckboxLine(hmiData.enablePreview); }

  void onClickConfirmToPrint() {
    dwinResetStatusLine();
    if (hmiFlag.select_flag) {     // Confirm
      gotoMainMenu();
      return card.openAndPrintFile(card.filename);
    }
    else
      hmiReturnScreen();
  }

#endif // HAS_GCODE_PREVIEW

void gotoConfirmToPrint() {
  #if HAS_GCODE_PREVIEW
    if (hmiData.enablePreview) return gotoPopup(preview.drawFromSD, onClickConfirmToPrint);
  #endif
  card.openAndPrintFile(card.filename); // Direct print SD file
}

#if HAS_ESDIAG
  void drawEndStopDiag() {
    hmiSaveProcessID(ID_ESDiagProcess);
    esDiag.draw();
  }
#endif

//=============================================================================
// MENU SUBSYSTEM
//=============================================================================

// Tool functions

#if ENABLED(EEPROM_SETTINGS)

  void writeEEPROM() {
    dwinDrawStatusLine(GET_TEXT_F(MSG_STORE_EEPROM));
    dwinUpdateLCD();
    DONE_BUZZ(settings.save());
  }

  void readEEPROM() {
    const bool success = settings.load();
    dwinRedrawScreen();
    DONE_BUZZ(success);
  }

  void resetEEPROM() {
    settings.reset();
    dwinRedrawScreen();
    DONE_BUZZ(true);
  }

  #if HAS_MESH
    void saveMesh() { TERN(AUTO_BED_LEVELING_UBL, ublMeshSave(), writeEEPROM()); }
  #endif

#endif // EEPROM_SETTINGS

// Reset Printer
void rebootPrinter() {
  wait_for_heatup = wait_for_user = false;    // Stop waiting for heating/user
  thermalManager.disable_all_heaters();
  planner.finish_and_disable();
  dwinRebootScreen();
  hal.reboot();
}

void gotoInfoMenu() {
  drawInfoMenu();
  dwinUpdateLCD();
  hmiSaveProcessID(ID_WaitResponse);
}

void disableMotors() { queue.inject(F("M84")); }

void autoLevel() {   // Always reacquire the Z "home" position
  queue.inject(F(TERN(AUTO_BED_LEVELING_UBL, "G29P1", "G29")));
}

void autoHome() { queue.inject_P(G28_STR); }

#if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
  void homeX() { queue.inject(F("G28X")); }
  void homeY() { queue.inject(F("G28Y")); }
  void homeZ() { queue.inject(F("G28Z")); }
  #if ALL(INDIVIDUAL_AXIS_HOMING_SUBMENU, MESH_BED_LEVELING)
    void applyZAfterHoming() { hmiData.zAfterHoming = menuData.value; };
    void setZAfterHoming() { setIntOnClick(0, 20, hmiData.zAfterHoming, applyZAfterHoming); }
  #endif
#endif

#if HAS_ZOFFSET_ITEM

  void applyZOffset() { TERN_(EEPROM_SETTINGS, settings.save()); }
  void liveZOffset() {
    #if ANY(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
      const_float_t step_zoffset = round((menuData.value / 100.0f) * planner.settings.axis_steps_per_mm[Z_AXIS]) - babystep.accum;
      if (BABYSTEP_ALLOWED()) babystep.add_steps(Z_AXIS, step_zoffset);
    #endif
  }
  void setZOffset() {
    #if ANY(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
      babystep.accum = round(planner.settings.axis_steps_per_mm[Z_AXIS] * BABY_Z_VAR);
    #endif
    setPFloatOnClick(PROBE_OFFSET_ZMIN, PROBE_OFFSET_ZMAX, 2, applyZOffset, liveZOffset);
  }

  void setMoveZto0() {
    #if ENABLED(Z_SAFE_HOMING)
      gcode.process_subcommands_now(MString<54>(F("G28XYO\nG28Z\nG0F5000X"), Z_SAFE_HOMING_X_POINT, F("Y"), Z_SAFE_HOMING_Y_POINT, F("\nG0Z0F300\nM400")));
    #else
      TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));
      gcode.process_subcommands_now(F("G28Z\nG0Z0F300\nM400"));
    #endif
    ui.reset_status();
    DONE_BUZZ(true);
  }

  #if !HAS_BED_PROBE
    void homeZAndDisable() {
      setMoveZto0();
      disableMotors();
    }
  #endif

#endif // HAS_ZOFFSET_ITEM

#if HAS_PREHEAT
  #define _doPreheat(N) void DoPreheat##N() { ui.preheat_all(N-1); }\
                        void DoPreheatHotend##N() { ui.preheat_hotend(N-1); }
  REPEAT_1(PREHEAT_COUNT, _doPreheat)
#endif

void doCoolDown() { thermalManager.cooldown(); }

void setLanguage() {
  hmiToggleLanguage();
  currentMenu = nullptr;  // Invalidate menu to full redraw
  drawPrepareMenu();
}

bool enableLiveMove = false;
void setLiveMove() { toggleCheckboxLine(enableLiveMove); }
void axisMove(AxisEnum axis) {
  #if HAS_HOTEND
    if (axis == E_AXIS && thermalManager.tooColdToExtrude(0)) {
      gcode.process_subcommands_now(F("G92E0"));  // Reset extruder position
      return dwinPopupConfirm(ICON_TempTooLow, GET_TEXT_F(MSG_HOTEND_TOO_COLD), GET_TEXT_F(MSG_PLEASE_PREHEAT));
    }
  #endif
  planner.synchronize();
  if (!planner.is_full()) planner.buffer_line(current_position, manual_feedrate_mm_s[axis]);
}
void liveMove() {
  if (!enableLiveMove) return;
  *menuData.floatPtr = menuData.value / MINUNITMULT;
  axisMove(hmiValue.axis);
}
void applyMove() {
  if (enableLiveMove) return;
  axisMove(hmiValue.axis);
}

void setMoveX() { hmiValue.axis = X_AXIS; setPFloatOnClick(X_MIN_POS, X_MAX_POS, UNITFDIGITS, applyMove, liveMove); }
void setMoveY() { hmiValue.axis = Y_AXIS; setPFloatOnClick(Y_MIN_POS, Y_MAX_POS, UNITFDIGITS, applyMove, liveMove); }
void setMoveZ() { hmiValue.axis = Z_AXIS; setPFloatOnClick(Z_MIN_POS, Z_MAX_POS, UNITFDIGITS, applyMove, liveMove); }

#if ENABLED(Z_STEPPER_AUTO_ALIGN)
  void autoZAlign() {
    LCD_MESSAGE(MSG_AUTO_Z_ALIGN);
    queue.inject(F("G34"));
  }
#endif

#if HAS_HOTEND
  void setMoveE() {
    const float e_min = current_position.e - (EXTRUDE_MAXLENGTH),
                e_max = current_position.e + (EXTRUDE_MAXLENGTH);
    hmiValue.axis = E_AXIS; setPFloatOnClick(e_min, e_max, UNITFDIGITS, applyMove, liveMove);
  }
#endif

#if ENABLED(POWER_LOSS_RECOVERY)
  void setPwrLossr() {
    toggleCheckboxLine(recovery.enabled);
    recovery.changed();
  }
#endif

#if ENABLED(BAUD_RATE_GCODE)
  void hmiSetBaudRate() { hmiData.baud115K ? setBaud115K() : setBaud250K(); }
  void setBaudRate() {
    FLIP(hmiData.baud115K);
    hmiSetBaudRate();
    drawCheckboxLine(currentMenu->line(), hmiData.baud115K);
    dwinUpdateLCD();
  }
  void setBaud115K() { queue.inject(F("M575 P0 B115200")); hmiData.baud115K = true; }
  void setBaud250K() { queue.inject(F("M575 P0 B250000")); hmiData.baud115K = false; }
#endif

#if HAS_LCD_BRIGHTNESS
  void applyBrightness() { ui.set_brightness(menuData.value); }
  void liveBrightness() { dwinLCDBrightness(menuData.value); }
  void setBrightness() { setIntOnClick(LCD_BRIGHTNESS_MIN, LCD_BRIGHTNESS_MAX, ui.brightness, applyBrightness, liveBrightness); }
  void turnOffBacklight() { hmiSaveProcessID(ID_WaitResponse); ui.set_brightness(0); dwinRedrawScreen(); }
#endif

#if ENABLED(CASE_LIGHT_MENU)
  void setCaseLight() {
    toggleCheckboxLine(caselight.on);
    caselight.update_enabled();
  }
  #if CASELIGHT_USES_BRIGHTNESS
    bool enableLiveCaseLightBrightness = true;
    void liveCaseLightBrightness() { caselight.brightness = menuData.value; caselight.update_brightness(); }
    void setCaseLightBrightness() { setIntOnClick(0, 255, caselight.brightness, liveCaseLightBrightness, enableLiveCaseLightBrightness ? liveCaseLightBrightness : nullptr); }
  #endif
#endif

#if ENABLED(LED_CONTROL_MENU)
  #if !ALL(CASE_LIGHT_MENU, CASE_LIGHT_USE_NEOPIXEL)
    void setLedStatus() {
      leds.toggle();
      showCheckboxLine(leds.lights_on);
    }
  #endif
  #if HAS_COLOR_LEDS
    bool enableLiveLedColor = true;
    void applyLEDColor() {
      hmiData.ledColor = LEDColor( {leds.color.r, leds.color.g, leds.color.b OPTARG(HAS_WHITE_LED, hmiData.ledColor.w) } );
      if (!enableLiveLedColor) leds.update();
    }
    void liveLEDColor(uint8_t *color) { *color = menuData.value; if (enableLiveLedColor) leds.update(); }
    void liveLEDColorR() { liveLEDColor(&leds.color.r); }
    void liveLEDColorG() { liveLEDColor(&leds.color.g); }
    void liveLEDColorB() { liveLEDColor(&leds.color.b); }
    void setLEDColorR() { setIntOnClick(0, 255, leds.color.r, applyLEDColor, liveLEDColorR); }
    void setLEDColorG() { setIntOnClick(0, 255, leds.color.g, applyLEDColor, liveLEDColorG); }
    void setLEDColorB() { setIntOnClick(0, 255, leds.color.b, applyLEDColor, liveLEDColorB); }
    #if HAS_WHITE_LED
      void liveLEDColorW() { liveLEDColor(&leds.color.w); }
      void setLEDColorW() { setIntOnClick(0, 255, leds.color.w, applyLEDColor, liveLEDColorW); }
    #endif
  #endif
#endif

#if ENABLED(SOUND_MENU_ITEM)
  void setEnableSound() {
    toggleCheckboxLine(ui.sound_on);
  }
#endif

#if HAS_HOME_OFFSET
  void applyHomeOffset() { set_home_offset(hmiValue.axis, menuData.value / MINUNITMULT); }
  void setHomeOffsetX() { hmiValue.axis = X_AXIS; setPFloatOnClick(-50, 50, UNITFDIGITS, applyHomeOffset); }
  void setHomeOffsetY() { hmiValue.axis = Y_AXIS; setPFloatOnClick(-50, 50, UNITFDIGITS, applyHomeOffset); }
  void setHomeOffsetZ() { hmiValue.axis = Z_AXIS; setPFloatOnClick( -2,  2, UNITFDIGITS, applyHomeOffset); }
#endif

#if HAS_BED_PROBE
  void setProbeOffsetX() { setPFloatOnClick(-60, 60, UNITFDIGITS); }
  void setProbeOffsetY() { setPFloatOnClick(-60, 60, UNITFDIGITS); }
  void setProbeOffsetZ() { setPFloatOnClick(-10, 10, 2); }

  #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
    void probeTest() {
      LCD_MESSAGE(MSG_M48_TEST);
      queue.inject(F("G28O\nM48 P10"));
    }
  #endif

  void probeStow() { probe.stow(); }
  void probeDeploy() { probe.deploy(); }

  #if HAS_BLTOUCH_HS_MODE
    void setHSMode() { toggleCheckboxLine(bltouch.high_speed_mode); }
  #endif

#endif

#if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
  void applyTimer() { ui.backlight_timeout_minutes = menuData.value; }
  void setTimer() { setIntOnClick(ui.backlight_timeout_min, ui.backlight_timeout_max, ui.backlight_timeout_minutes, applyTimer); }
#endif

#if HAS_FILAMENT_SENSOR
  void setRunoutEnable() {
    runout.reset();
    toggleCheckboxLine(runout.enabled);
  }
  #if HAS_FILAMENT_RUNOUT_DISTANCE
    void applyRunoutDistance() { runout.set_runout_distance(menuData.value / MINUNITMULT); }
    void setRunoutDistance() { setFloatOnClick(0, 999, UNITFDIGITS, runout.runout_distance(), applyRunoutDistance); }
  #endif
#endif

#if ENABLED(CONFIGURE_FILAMENT_CHANGE)
  void setFilLoad()   { setPFloatOnClick(0, EXTRUDE_MAXLENGTH, UNITFDIGITS); }
  void setFilUnload() { setPFloatOnClick(0, EXTRUDE_MAXLENGTH, UNITFDIGITS); }
#endif

#if ENABLED(PREVENT_COLD_EXTRUSION)
  void applyExtMinT() { thermalManager.extrude_min_temp = hmiData.extMinT; thermalManager.allow_cold_extrude = (hmiData.extMinT == 0); }
  void setExtMinT() { setPIntOnClick(MIN_ETEMP, MAX_ETEMP, applyExtMinT); }
#endif

void setSpeed() { setPIntOnClick(SPEED_EDIT_MIN, SPEED_EDIT_MAX); }

#if HAS_HOTEND
  void applyHotendTemp() { thermalManager.setTargetHotend(menuData.value, 0); }
  void setHotendTemp() { setIntOnClick(MIN_ETEMP, MAX_ETEMP, thermalManager.degTargetHotend(0), applyHotendTemp); }
#endif

#if HAS_HEATED_BED
  void applyBedTemp() { thermalManager.setTargetBed(menuData.value); }
  void setBedTemp() { setIntOnClick(MIN_BEDTEMP, MAX_BEDTEMP, thermalManager.degTargetBed(), applyBedTemp); }
#endif

#if HAS_FAN
  void applyFanSpeed() { thermalManager.set_fan_speed(0, menuData.value); }
  void setFanSpeed() { setIntOnClick(0, 255, thermalManager.fan_speed[0], applyFanSpeed); }
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
  void parkHead() {
    LCD_MESSAGE(MSG_FILAMENT_PARK_ENABLED);
    queue.inject(F("G28O\nG27"));
  }
#endif

#if ENABLED(ADVANCED_PAUSE_FEATURE)

  void changeFilament() {
    hmiSaveProcessID(ID_NothingToDo);
    queue.inject(F("M600 B2"));
  }

  #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
    void unloadFilament() {
      LCD_MESSAGE(MSG_FILAMENTUNLOAD);
      queue.inject(F("M702 Z20"));
    }

    void loadFilament() {
      LCD_MESSAGE(MSG_FILAMENTLOAD);
      queue.inject(F("M701 Z20"));
    }
  #endif

#endif // ADVANCED_PAUSE_FEATURE

void setFlow() { setPIntOnClick(FLOW_EDIT_MIN, FLOW_EDIT_MAX, []{ planner.refresh_e_factor(0); }); }

// Bed Tramming

#if ENABLED(LCD_BED_TRAMMING)

  void tramXY(const uint8_t point, float &x, float &y) {
    switch (point) {
      case 0:
        LCD_MESSAGE(MSG_TRAM_FL);
        x = bed_tramming_inset_lfbr[0];
        y = bed_tramming_inset_lfbr[1];
        break;
      case 1:
        LCD_MESSAGE(MSG_TRAM_FR);
        x = X_BED_SIZE - bed_tramming_inset_lfbr[2];
        y = bed_tramming_inset_lfbr[1];
        break;
      case 2:
        LCD_MESSAGE(MSG_TRAM_BR);
        x = X_BED_SIZE - bed_tramming_inset_lfbr[2];
        y = Y_BED_SIZE - bed_tramming_inset_lfbr[3];
        break;
      case 3:
        LCD_MESSAGE(MSG_TRAM_BL);
        x = bed_tramming_inset_lfbr[0];
        y = Y_BED_SIZE - bed_tramming_inset_lfbr[3];
        break;
      #if ENABLED(BED_TRAMMING_INCLUDE_CENTER)
        case 4:
          LCD_MESSAGE(MSG_TRAM_C);
          x = X_CENTER;
          y = Y_CENTER;
          break;
      #endif
    }
  }

  #if HAS_BED_PROBE

    float tram(const uint8_t point) {
      static bool inLev = false;
      if (inLev) return NAN;

      float xpos = 0, ypos = 0, zval = 0;
      tramXY(point, xpos, ypos);

      if (hmiData.fullManualTramming) {
        queue.inject(MString<100>(
          F("M420S0\nG28O\nG90\nG0F300Z5\nG0F5000X"), p_float_t(xpos, 1), 'Y', p_float_t(ypos, 1), F("\nG0F300Z0")
        ));
      }
      else {
        // AUTO_BED_LEVELING_BILINEAR does not define MESH_INSET
        #ifndef MESH_MIN_X
          #define MESH_MIN_X (_MAX(X_MIN_BED + (PROBING_MARGIN_LEFT), X_MIN_POS))
        #endif
        #ifndef MESH_MIN_Y
          #define MESH_MIN_Y (_MAX(Y_MIN_BED + (PROBING_MARGIN_FRONT), Y_MIN_POS))
        #endif
        #ifndef MESH_MAX_X
          #define MESH_MAX_X (_MIN(X_MAX_BED - (PROBING_MARGIN_RIGHT), X_MAX_POS))
        #endif
        #ifndef MESH_MAX_Y
          #define MESH_MAX_Y (_MIN(Y_MAX_BED - (PROBING_MARGIN_BACK), Y_MAX_POS))
        #endif

        LIMIT(xpos, MESH_MIN_X, MESH_MAX_X);
        LIMIT(ypos, MESH_MIN_Y, MESH_MAX_Y);
        probe.stow();
        gcode.process_subcommands_now(F("M420S0\nG28O"));
        inLev = true;
        zval = probe.probe_at_point(xpos, ypos, PROBE_PT_STOW);
        if (isnan(zval))
          LCD_MESSAGE(MSG_ZPROBE_OUT);
        else
          ui.set_status(TS(F("X:"), p_float_t(xpos, 1), F(" Y:"), p_float_t(ypos, 1), F(" Z:"), p_float_t(zval, 2)));
        inLev = false;
      }
      return zval;
    }

  #else

    void tram(const uint8_t point) {
      float xpos = 0, ypos = 0;
      tramXY(point, xpos, ypos);
      queue.inject(MString<100>(
        F("M420S0\nG28O\nG90\nG0F300Z5\nG0F5000X"), p_float_t(xpos, 1), 'Y', p_float_t(ypos, 1), F("\nG0F300Z0")
      ));
    }

  #endif

  #if HAS_BED_PROBE && HAS_MESH

    void trammingwizard() {
      if (hmiData.fullManualTramming) {
        LCD_MESSAGE_F("Disable manual tramming");
        return;
      }
      bed_mesh_t zval = {0};
      zval[0][0] = tram(0);
      checkkey = ID_NothingToDo;
      meshViewer.drawMesh(zval, 2, 2);
      zval[1][0] = tram(1);
      meshViewer.drawMesh(zval, 2, 2);
      zval[1][1] = tram(2);
      meshViewer.drawMesh(zval, 2, 2);
      zval[0][1] = tram(3);
      meshViewer.drawMesh(zval, 2, 2);

      DWINUI::drawCenteredString(140, F("Calculating average"));
      DWINUI::drawCenteredString(160, F("and relative heights"));
      safe_delay(1000);
      float avg = 0.0f;
      for (uint8_t x = 0; x < 2; ++x) for (uint8_t y = 0; y < 2; ++y) avg += zval[x][y];
      avg /= 4.0f;
      for (uint8_t x = 0; x < 2; ++x) for (uint8_t y = 0; y < 2; ++y) zval[x][y] -= avg;
      meshViewer.drawMesh(zval, 2, 2);
      ui.reset_status();

      #ifndef BED_TRAMMING_PROBE_TOLERANCE
        #define BED_TRAMMING_PROBE_TOLERANCE 0.05f
      #endif

      if (ABS(meshViewer.max - meshViewer.min) < BED_TRAMMING_PROBE_TOLERANCE) {
        DWINUI::drawCenteredString(140, F("Corners leveled"));
        DWINUI::drawCenteredString(160, F("Tolerance achieved!"));
      }
      else {
        uint8_t p = 0;
        float max = 0.0f;
        FSTR_P plabel;
        bool s = true;
        for (uint8_t x = 0; x < 2; ++x) for (uint8_t y = 0; y < 2; ++y) {
          const float d = ABS(zval[x][y]);
          if (max < d) {
            s = (zval[x][y] >= 0);
            max = d;
            p = x + 2 * y;
          }
        }
        switch (p) {
          case 0b00 : plabel = GET_TEXT_F(MSG_TRAM_FL); break;
          case 0b01 : plabel = GET_TEXT_F(MSG_TRAM_FR); break;
          case 0b10 : plabel = GET_TEXT_F(MSG_TRAM_BL); break;
          case 0b11 : plabel = GET_TEXT_F(MSG_TRAM_BR); break;
          default   : plabel = F(""); break;
        }
        DWINUI::drawCenteredString(120, F("Corners not leveled"));
        DWINUI::drawCenteredString(140, F("Knob adjustment required"));
        DWINUI::drawCenteredString(COLOR_GREEN, 160, s ? F("Lower") : F("Raise"));
        DWINUI::drawCenteredString(COLOR_GREEN, 180, plabel);
      }
      DWINUI::drawButton(BTN_Continue, 86, 305);
      checkkey = ID_Menu;
      hmiSaveProcessID(ID_WaitResponse);
    }

    void setManualTramming() {
      toggleCheckboxLine(hmiData.fullManualTramming);
    }

  #endif // HAS_BED_PROBE && HAS_MESH

#endif // LCD_BED_TRAMMING

#if ENABLED(MESH_BED_LEVELING)

  void manualMeshStart() {
    LCD_MESSAGE(MSG_UBL_BUILD_MESH_MENU);
    gcode.process_subcommands_now(F("G28XYO\nG28Z\nM211S0\nG29S1"));
    #ifdef MANUAL_PROBE_START_Z
      const uint8_t line = currentMenu->line(mMeshMoveZItem->pos);
      DWINUI::drawSignedFloat(hmiData.colorText, hmiData.colorBackground, 3, 2, VALX - 2 * DWINUI::fontWidth(DWIN_FONT_MENU), MBASE(line), MANUAL_PROBE_START_Z);
    #endif
  }

  void liveMeshMoveZ() {
    *menuData.floatPtr = menuData.value / POW(10, 2);
    if (!planner.is_full()) {
      planner.synchronize();
      planner.buffer_line(current_position, manual_feedrate_mm_s[Z_AXIS]);
    }
  }
  void setMMeshMoveZ() { setPFloatOnClick(-1, 1, 2, planner.synchronize, liveMeshMoveZ); }

  void manualMeshContinue() {
    gcode.process_subcommands_now(F("G29S2"));
    mMeshMoveZItem->redraw();
  }

  void manualMeshSave() {
    LCD_MESSAGE(MSG_UBL_STORAGE_MESH_MENU);
    queue.inject(F("M211S1\nM500"));
  }

#endif // MESH_BED_LEVELING

#if HAS_PREHEAT
  #if HAS_HOTEND
    void setPreheatEndTemp() { setPIntOnClick(MIN_ETEMP, MAX_ETEMP); }
  #endif
  #if HAS_HEATED_BED
    void setPreheatBedTemp() { setPIntOnClick(MIN_BEDTEMP, MAX_BEDTEMP); }
  #endif
  #if HAS_FAN
    void setPreheatFanSpeed() { setPIntOnClick(0, 255); }
  #endif
#endif

void applyMaxSpeed() { planner.set_max_feedrate(hmiValue.axis, menuData.value / MINUNITMULT); }
#if HAS_X_AXIS
  void setMaxSpeedX() { hmiValue.axis = X_AXIS; setFloatOnClick(min_feedrate_edit_values.x, max_feedrate_edit_values.x, UNITFDIGITS, planner.settings.max_feedrate_mm_s[X_AXIS], applyMaxSpeed); }
#endif
#if HAS_Y_AXIS
  void setMaxSpeedY() { hmiValue.axis = Y_AXIS; setFloatOnClick(min_feedrate_edit_values.y, max_feedrate_edit_values.y, UNITFDIGITS, planner.settings.max_feedrate_mm_s[Y_AXIS], applyMaxSpeed); }
#endif
#if HAS_Z_AXIS
  void setMaxSpeedZ() { hmiValue.axis = Z_AXIS; setFloatOnClick(min_feedrate_edit_values.z, max_feedrate_edit_values.z, UNITFDIGITS, planner.settings.max_feedrate_mm_s[Z_AXIS], applyMaxSpeed); }
#endif
#if HAS_HOTEND
  void setMaxSpeedE() { hmiValue.axis = E_AXIS; setFloatOnClick(min_feedrate_edit_values.e, max_feedrate_edit_values.e, UNITFDIGITS, planner.settings.max_feedrate_mm_s[E_AXIS], applyMaxSpeed); }
#endif

void applyMaxAccel() { planner.set_max_acceleration(hmiValue.axis, menuData.value); }
#if HAS_X_AXIS
  void setMaxAccelX() { hmiValue.axis = X_AXIS; setIntOnClick(min_acceleration_edit_values.x, max_acceleration_edit_values.x, planner.settings.max_acceleration_mm_per_s2[X_AXIS], applyMaxAccel); }
#endif
#if HAS_Y_AXIS
  void setMaxAccelY() { hmiValue.axis = Y_AXIS; setIntOnClick(min_acceleration_edit_values.y, max_acceleration_edit_values.y, planner.settings.max_acceleration_mm_per_s2[Y_AXIS], applyMaxAccel); }
#endif
#if HAS_Z_AXIS
  void setMaxAccelZ() { hmiValue.axis = Z_AXIS; setIntOnClick(min_acceleration_edit_values.z, max_acceleration_edit_values.z, planner.settings.max_acceleration_mm_per_s2[Z_AXIS], applyMaxAccel); }
#endif
#if HAS_HOTEND
  void setMaxAccelE() { hmiValue.axis = E_AXIS; setIntOnClick(min_acceleration_edit_values.e, max_acceleration_edit_values.e, planner.settings.max_acceleration_mm_per_s2[E_AXIS], applyMaxAccel); }
#endif

#if ENABLED(CLASSIC_JERK)
  void applyMaxJerk() { planner.set_max_jerk(hmiValue.axis, menuData.value / MINUNITMULT); }
  #if HAS_X_AXIS
    void setMaxJerkX() { hmiValue.axis = X_AXIS; setFloatOnClick(min_jerk_edit_values.x, max_jerk_edit_values.x, UNITFDIGITS, planner.max_jerk.x, applyMaxJerk); }
  #endif
  #if HAS_Y_AXIS
    void setMaxJerkY() { hmiValue.axis = Y_AXIS; setFloatOnClick(min_jerk_edit_values.y, max_jerk_edit_values.y, UNITFDIGITS, planner.max_jerk.y, applyMaxJerk); }
  #endif
  #if HAS_Z_AXIS
    void setMaxJerkZ() { hmiValue.axis = Z_AXIS; setFloatOnClick(min_jerk_edit_values.z, max_jerk_edit_values.z, UNITFDIGITS, planner.max_jerk.z, applyMaxJerk); }
  #endif
  #if HAS_HOTEND
    void setMaxJerkE() { hmiValue.axis = E_AXIS; setFloatOnClick(min_jerk_edit_values.e, max_jerk_edit_values.e, UNITFDIGITS, planner.max_jerk.e, applyMaxJerk); }
  #endif
#elif HAS_JUNCTION_DEVIATION
  void applyJDmm() { TERN_(LIN_ADVANCE, planner.recalculate_max_e_jerk()); }
  void setJDmm() { setPFloatOnClick(MIN_JD_MM, MAX_JD_MM, 3, applyJDmm); }
#endif

#if ENABLED(LIN_ADVANCE)
  void setLA_K() { setPFloatOnClick(0, 10, 3); }
#endif

#if HAS_X_AXIS
  void setStepsX() { hmiValue.axis = X_AXIS; setPFloatOnClick( min_steps_edit_values.x, max_steps_edit_values.x, UNITFDIGITS); }
#endif
#if HAS_Y_AXIS
  void setStepsY() { hmiValue.axis = Y_AXIS; setPFloatOnClick( min_steps_edit_values.y, max_steps_edit_values.y, UNITFDIGITS); }
#endif
#if HAS_Z_AXIS
  void setStepsZ() { hmiValue.axis = Z_AXIS; setPFloatOnClick( min_steps_edit_values.z, max_steps_edit_values.z, UNITFDIGITS); }
#endif
#if HAS_HOTEND
  void setStepsE() { hmiValue.axis = E_AXIS; setPFloatOnClick( min_steps_edit_values.e, max_steps_edit_values.e, UNITFDIGITS); }
#endif

#if ENABLED(EDITABLE_HOMING_FEEDRATE)
  void updateHomingFR(AxisEnum axis, feedRate_t value) {
    switch (axis) {
      case X_AXIS: homing_feedrate_mm_m.x = value; break;
      case Y_AXIS: homing_feedrate_mm_m.y = value; break;
      case Z_AXIS: homing_feedrate_mm_m.z = value; break;
      default: break;
    }
  }
  void applyHomingFR() { updateHomingFR(hmiValue.axis, menuData.value); }
  #if HAS_X_AXIS
    void setHomingX() { hmiValue.axis = X_AXIS; setIntOnClick(min_homing_edit_values.x, max_homing_edit_values.x, homing_feedrate_mm_m.x, applyHomingFR); }
  #endif
  #if HAS_Y_AXIS
    void setHomingY() { hmiValue.axis = Y_AXIS; setIntOnClick(min_homing_edit_values.y, max_homing_edit_values.y, homing_feedrate_mm_m.x, applyHomingFR); }
  #endif
  #if HAS_Z_AXIS
    void setHomingZ() { hmiValue.axis = Z_AXIS; setIntOnClick(min_homing_edit_values.z, max_homing_edit_values.z, homing_feedrate_mm_m.x, applyHomingFR); }
  #endif
#endif

#if ENABLED(FWRETRACT)
  void returnFWRetractMenu() { (previousMenu == filSetMenu) ? drawFilSetMenu() : drawTuneMenu(); }
  void setRetractLength() { setPFloatOnClick( 0, 10, UNITFDIGITS); }
  void setRetractSpeed()  { setPFloatOnClick( 1, 90, UNITFDIGITS); }
  void setZRaise()        { setPFloatOnClick( 0, 2, 2); }
  void setRecoverSpeed()  { setPFloatOnClick( 1, 90, UNITFDIGITS); }
  void setAddRecover()    { setPFloatOnClick(-5, 5, UNITFDIGITS); }
#endif

// Special Menuitem Drawing functions =================================================

void onDrawBack(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 129, 72, 156, 84);
  onDrawMenuItem(menuitem, line);
}

void onDrawTempSubMenu(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1,  57, 104,  84, 116);
  onDrawSubMenu(menuitem, line);
}

void onDrawMotionSubMenu(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1,  87, 104, 114, 116);
  onDrawSubMenu(menuitem, line);
}

#if ENABLED(EEPROM_SETTINGS)
  void onDrawWriteEeprom(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 117, 104, 172, 116);
    onDrawMenuItem(menuitem, line);
  }

  void onDrawReadEeprom(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 174, 103, 229, 116);
    onDrawMenuItem(menuitem, line);
  }

  void onDrawResetEeprom(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1,   1, 118,  56, 131);
    onDrawMenuItem(menuitem, line);
  }
#endif

void onDrawInfoSubMenu(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 231, 104, 258, 116);
  onDrawSubMenu(menuitem, line);
}

void onDrawMoveX(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 58, 118, 106, 132);
  onDrawPFloatMenu(menuitem, line);
}

void onDrawMoveY(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 109, 118, 157, 132);
  onDrawPFloatMenu(menuitem, line);
}

void onDrawMoveZ(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 160, 118, 209, 132);
  onDrawPFloatMenu(menuitem, line);
}

#if HAS_HOTEND
  void onDrawMoveE(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 212, 118, 253, 131);
    onDrawPFloatMenu(menuitem, line);
  }
#endif

void onDrawMoveSubMenu(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 159, 70, 200, 84);
  onDrawSubMenu(menuitem, line);
}

void onDrawDisableMotors(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 204, 70, 259, 82);
  onDrawMenuItem(menuitem, line);
}

void onDrawAutoHome(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 0, 89, 41, 101);
  onDrawMenuItem(menuitem, line);
}

#if HAS_ZOFFSET_ITEM
  #if ANY(BABYSTEP_ZPROBE_OFFSET, JUST_BABYSTEP)
    void onDrawZOffset(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) menuitem->setFrame(1, 174, 164, 223, 177);
      onDrawPFloat2Menu(menuitem, line);
    }
  #endif
#endif

#if HAS_HOTEND
  void onDrawPreheat1(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 100, 89, 151, 101);
    onDrawMenuItem(menuitem, line);
  }
  #if PREHEAT_COUNT > 1
    void onDrawPreheat2(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) menuitem->setFrame(1, 180, 89, 233, 100);
      onDrawMenuItem(menuitem, line);
    }
  #endif
#endif

#if HAS_PREHEAT
  void onDrawCooldown(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 1, 104,  56, 117);
    onDrawMenuItem(menuitem, line);
  }
#endif

void onDrawLanguage(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 239, 134, 266, 146);
  onDrawMenuItem(menuitem, line);
  DWINUI::drawString(VALX, MBASE(line), hmiIsChinese() ? F("CN") : F("EN"));
}

void onDrawSelColorItem(MenuItem* menuitem, int8_t line) {
  const uint16_t color = *(uint16_t*)static_cast<MenuItemPtr*>(menuitem)->value;
  dwinDrawRectangle(0, hmiData.colorHighlight, ICOX + 1, MBASE(line) - 1 + 1, ICOX + 18, MBASE(line) - 1 + 18);
  dwinDrawRectangle(1, color, ICOX + 2, MBASE(line) - 1 + 2, ICOX + 17, MBASE(line) - 1 + 17);
  onDrawMenuItem(menuitem, line);
}

void onDrawGetColorItem(MenuItem* menuitem, int8_t line) {
  const uint8_t i = menuitem->icon;
  uint16_t color;
  switch (i) {
    case 0:  color = RGB(31, 0, 0); break; // Red
    case 1:  color = RGB(0, 63, 0); break; // Green
    case 2:  color = RGB(0, 0, 31); break; // Blue
    default: color = 0; break;
  }
  dwinDrawRectangle(0, hmiData.colorHighlight, ICOX + 1, MBASE(line) - 1 + 1, ICOX + 18, MBASE(line) - 1 + 18);
  dwinDrawRectangle(1, color, ICOX + 2, MBASE(line) - 1 + 2, ICOX + 17, MBASE(line) - 1 + 17);
  DWINUI::drawString(LBLX, MBASE(line) - 1, menuitem->caption);
  drawMenuIntValue(hmiData.colorBackground, line, 4, hmiValue.color[i]);
  dwinDrawHLine(hmiData.colorSplitLine, 16, MYPOS(line + 1), 240);
}

void onDrawSpeedItem(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 116, 164, 171, 176);
  onDrawPIntMenu(menuitem, line);
}

#if HAS_HOTEND
  void onDrawHotendTemp(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 1, 134, 56, 146);
    onDrawPIntMenu(menuitem, line);
  }
#endif

#if HAS_HEATED_BED
  void onDrawBedTemp(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 58, 134, 113, 146);
    onDrawPIntMenu(menuitem, line);
  }
#endif

#if HAS_FAN
  void onDrawFanSpeed(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 115, 134, 170, 146);
    onDrawPInt8Menu(menuitem, line);
  }
#endif

void onDrawSteps(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) menuitem->setFrame(1, 153, 148, 194, 161);
  onDrawSubMenu(menuitem, line);
}

#if ENABLED(MESH_BED_LEVELING)
  void onDrawMMeshMoveZ(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) menuitem->setFrame(1, 160, 118, 209, 132);
    onDrawPFloat2Menu(menuitem, line);
  }
#endif

#if HAS_PREHEAT
  #if HAS_HOTEND
    void onDrawSetPreheatHotend(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) menuitem->setFrame(1, 1, 134, 56, 146);
      onDrawPIntMenu(menuitem, line);
    }
  #endif
  #if HAS_HEATED_BED
    void onDrawSetPreheatBed(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) menuitem->setFrame(1, 58, 134, 113, 146);
      onDrawPIntMenu(menuitem, line);
    }
  #endif
  #if HAS_FAN
    void onDrawSetPreheatFan(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) menuitem->setFrame(1, 115, 134, 170, 146);
      onDrawPIntMenu(menuitem, line);
    }
  #endif
#endif // HAS_PREHEAT

void onDrawSpeed(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese())
    menuitem->setFrame(1, 173, 133, 228, 147);
  onDrawSubMenu(menuitem, line);
}

#if HAS_X_AXIS
  void onDrawMaxSpeedX(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 228, 147);
      dwinFrameAreaCopy(1, 229, 133, 236, 147, LBLX + 58, MBASE(line));   // X
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_Y_AXIS
  void onDrawMaxSpeedY(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 228, 147);
      dwinFrameAreaCopy(1, 1, 150, 7, 160, LBLX + 58, MBASE(line));       // Y
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_Z_AXIS
  void onDrawMaxSpeedZ(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 228, 147);
      dwinFrameAreaCopy(1, 9, 150, 16, 160, LBLX + 58, MBASE(line) + 3);  // Z
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_HOTEND
  void onDrawMaxSpeedE(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 228, 147);
      dwinFrameAreaCopy(1, 18, 150, 25, 160, LBLX + 58, MBASE(line));     // E
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

void onDrawAcc(MenuItem* menuitem, int8_t line) {
  if (hmiIsChinese()) {
    menuitem->setFrame(1, 173, 133, 200, 147);
    dwinFrameAreaCopy(1, 28, 149, 69, 161, LBLX + 27, MBASE(line) + 1);   // ...Acceleration
  }
  onDrawSubMenu(menuitem, line);
}

#if HAS_X_AXIS
  void onDrawMaxAccelX(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 200, 147);
      dwinFrameAreaCopy(1, 28,  149,  69, 161, LBLX + 27, MBASE(line));
      dwinFrameAreaCopy(1, 229, 133, 236, 147, LBLX + 71, MBASE(line));   // X
    }
    onDrawPInt32Menu(menuitem, line);
  }
#endif

#if HAS_Y_AXIS
  void onDrawMaxAccelY(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 200, 147);
      dwinFrameAreaCopy(1, 28, 149,  69, 161, LBLX + 27, MBASE(line));
      dwinFrameAreaCopy(1,  1, 150,   7, 160, LBLX + 71, MBASE(line));    // Y
    }
    onDrawPInt32Menu(menuitem, line);
  }
#endif

#if HAS_Z_AXIS
  void onDrawMaxAccelZ(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 200, 147);
      dwinFrameAreaCopy(1, 28, 149,  69, 161, LBLX + 27, MBASE(line));
      dwinFrameAreaCopy(1,  9, 150,  16, 160, LBLX + 71, MBASE(line));    // Z
    }
    onDrawPInt32Menu(menuitem, line);
  }
#endif

#if HAS_HOTEND
  void onDrawMaxAccelE(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 200, 147);
      dwinFrameAreaCopy(1, 28, 149,  69, 161, LBLX + 27, MBASE(line));
      dwinFrameAreaCopy(1, 18, 150,  25, 160, LBLX + 71, MBASE(line));    // E
    }
    onDrawPInt32Menu(menuitem, line);
  }
#endif

#if ENABLED(CLASSIC_JERK)

  void onDrawJerk(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 173, 133, 200, 147);
      dwinFrameAreaCopy(1, 1, 180, 28, 192, LBLX + 27, MBASE(line) + 1);  // ...
      dwinFrameAreaCopy(1, 202, 133, 228, 147, LBLX + 54, MBASE(line));   // ...Jerk
    }
    onDrawSubMenu(menuitem, line);
  }

  #if HAS_X_AXIS
    void onDrawMaxJerkX(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) {
        menuitem->setFrame(1, 173, 133, 200, 147);
        dwinFrameAreaCopy(1,   1, 180,  28, 192, LBLX + 27, MBASE(line));
        dwinFrameAreaCopy(1, 202, 133, 228, 147, LBLX + 53, MBASE(line));
        dwinFrameAreaCopy(1, 229, 133, 236, 147, LBLX + 83, MBASE(line));
      }
      onDrawPFloatMenu(menuitem, line);
    }
  #endif

  #if HAS_Y_AXIS
    void onDrawMaxJerkY(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) {
        menuitem->setFrame(1, 173, 133, 200, 147);
        dwinFrameAreaCopy(1,   1, 180,  28, 192, LBLX + 27, MBASE(line));
        dwinFrameAreaCopy(1, 202, 133, 228, 147, LBLX + 53, MBASE(line));
        dwinFrameAreaCopy(1,   1, 150,   7, 160, LBLX + 83, MBASE(line));
      }
      onDrawPFloatMenu(menuitem, line);
    }
  #endif

  #if HAS_Z_AXIS
    void onDrawMaxJerkZ(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) {
        menuitem->setFrame(1, 173, 133, 200, 147);
        dwinFrameAreaCopy(1,   1, 180,  28, 192, LBLX + 27, MBASE(line));
        dwinFrameAreaCopy(1, 202, 133, 228, 147, LBLX + 53, MBASE(line));
        dwinFrameAreaCopy(1,   9, 150,  16, 160, LBLX + 83, MBASE(line));
      }
      onDrawPFloatMenu(menuitem, line);
    }
  #endif

  #if HAS_HOTEND

    void onDrawMaxJerkE(MenuItem* menuitem, int8_t line) {
      if (hmiIsChinese()) {
        menuitem->setFrame(1, 173, 133, 200, 147);
        dwinFrameAreaCopy(1,   1, 180,  28, 192, LBLX + 27, MBASE(line));
        dwinFrameAreaCopy(1, 202, 133, 228, 147, LBLX + 53, MBASE(line));
        dwinFrameAreaCopy(1,  18, 150,  25, 160, LBLX + 83, MBASE(line));
      }
      onDrawPFloatMenu(menuitem, line);
    }

  #endif

#endif // CLASSIC_JERK

#if HAS_X_AXIS
  void onDrawStepsX(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 153, 148, 194, 161);
      dwinFrameAreaCopy(1, 229, 133, 236, 147, LBLX + 44, MBASE(line));      // X
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_Y_AXIS
  void onDrawStepsY(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 153, 148, 194, 161);
      dwinFrameAreaCopy(1,   1, 150,   7, 160, LBLX + 44, MBASE(line));      // Y
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_Z_AXIS
  void onDrawStepsZ(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 153, 148, 194, 161);
      dwinFrameAreaCopy(1,   9, 150,  16, 160, LBLX + 44, MBASE(line));      // Z
    }
    onDrawPFloatMenu(menuitem, line);
  }
#endif

#if HAS_HOTEND

  void onDrawStepsE(MenuItem* menuitem, int8_t line) {
    if (hmiIsChinese()) {
      menuitem->setFrame(1, 153, 148, 194, 161);
      dwinFrameAreaCopy(1,  18, 150,  25, 160, LBLX + 44, MBASE(line));    // E
    }
    onDrawPFloatMenu(menuitem, line);
  }

#endif

// Menu Creation and Drawing functions ======================================================

frame_rect_t selrect(frame_rect_t) {
  return hmiIsChinese() ? frame_rect_t({ 133, 1, 28, 13 }) : frame_rect_t({ 0 });
}

void drawPrepareMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(prepareMenu, selrect({133, 1, 28, 13}), MSG_PREPARE, 12 + PREHEAT_COUNT)) {
    BACK_ITEM(gotoMainMenu);
    MENU_ITEM(ICON_FilMan, MSG_FILAMENT_MAN, onDrawSubMenu, drawFilamentManMenu);
    MENU_ITEM(ICON_Axis, MSG_MOVE_AXIS, onDrawMoveSubMenu, drawMoveMenu);
    #if ENABLED(LCD_BED_TRAMMING)
      MENU_ITEM(ICON_Tram, MSG_BED_TRAMMING, onDrawSubMenu, drawTrammingMenu);
    #endif
    MENU_ITEM(ICON_CloseMotor, MSG_DISABLE_STEPPERS, onDrawDisableMotors, disableMotors);
    #if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)
      MENU_ITEM(ICON_Homing, MSG_HOMING, onDrawSubMenu, drawHomingMenu);
    #else
      MENU_ITEM(ICON_Homing, MSG_AUTO_HOME, onDrawAutoHome, autoHome);
    #endif
    #if ENABLED(MESH_BED_LEVELING)
      MENU_ITEM(ICON_ManualMesh, MSG_MANUAL_MESH, onDrawSubMenu, drawManualMeshMenu);
    #elif HAS_BED_PROBE
      MENU_ITEM(ICON_Level, MSG_AUTO_MESH, onDrawMenuItem, autoLevel);
    #endif
    #if HAS_ZOFFSET_ITEM
      #if HAS_BED_PROBE
        MENU_ITEM(ICON_SetZOffset, MSG_PROBE_WIZARD, onDrawSubMenu, drawZOffsetWizMenu);
      #elif ENABLED(BABYSTEPPING)
        EDIT_ITEM(ICON_Zoffset, MSG_HOME_OFFSET_Z, onDrawPFloat2Menu, setZOffset, &BABY_Z_VAR);
      #endif
    #endif
    #if HAS_PREHEAT
      #define _ITEM_PREHEAT(N) MENU_ITEM(ICON_Preheat##N, MSG_PREHEAT_##N, onDrawMenuItem, DoPreheat##N);
      REPEAT_1(PREHEAT_COUNT, _ITEM_PREHEAT)
    #endif
    MENU_ITEM(ICON_Cool, MSG_COOLDOWN, onDrawCooldown, doCoolDown);
    #if ALL(PROUI_TUNING_GRAPH, PROUI_ITEM_PLOT)
      MENU_ITEM(ICON_PIDNozzle, MSG_HOTEND_TEMP_GRAPH, onDrawMenuItem, drawHPlot);
      MENU_ITEM(ICON_PIDBed, MSG_BED_TEMP_GRAPH, onDrawMenuItem, drawBPlot);
    #endif
    MENU_ITEM(ICON_Language, MSG_UI_LANGUAGE, onDrawLanguage, setLanguage);
  }
  ui.reset_status(true);
  updateMenu(prepareMenu);
}

#if ENABLED(LCD_BED_TRAMMING)

  void drawTrammingMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(trammingMenu, MSG_BED_TRAMMING, 8)) {
      BACK_ITEM(drawPrepareMenu);
      #if HAS_BED_PROBE && HAS_MESH
        MENU_ITEM(ICON_Tram, MSG_TRAMMING_WIZARD, onDrawMenuItem, trammingwizard);
        EDIT_ITEM(ICON_Version, MSG_BED_TRAMMING_MANUAL, onDrawChkbMenu, setManualTramming, &hmiData.fullManualTramming);
      #elif !HAS_BED_PROBE && HAS_ZOFFSET_ITEM
        MENU_ITEM_F(ICON_MoveZ0, "Home Z and disable", onDrawMenuItem, homeZAndDisable);
      #endif
      MENU_ITEM(ICON_AxisBL, MSG_TRAM_FL, onDrawMenuItem, []{ (void)tram(0); });
      MENU_ITEM(ICON_AxisBR, MSG_TRAM_FR, onDrawMenuItem, []{ (void)tram(1); });
      MENU_ITEM(ICON_AxisTR, MSG_TRAM_BR, onDrawMenuItem, []{ (void)tram(2); });
      MENU_ITEM(ICON_AxisTL, MSG_TRAM_BL, onDrawMenuItem, []{ (void)tram(3); });
      #if ENABLED(BED_TRAMMING_INCLUDE_CENTER)
        MENU_ITEM(ICON_AxisC, MSG_TRAM_C, onDrawMenuItem, []{ (void)tram(4); });
      #endif
    }
    updateMenu(trammingMenu);
  }

#endif // LCD_BED_TRAMMING

void drawControlMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(controlMenu, selrect({103, 1, 28, 14}), MSG_CONTROL, 11)) {
    BACK_ITEM(gotoMainMenu);
    MENU_ITEM(ICON_Temperature, MSG_TEMPERATURE, onDrawTempSubMenu, drawTemperatureMenu);
    MENU_ITEM(ICON_Motion, MSG_MOTION, onDrawMotionSubMenu, drawMotionMenu);
    #if ENABLED(CASE_LIGHT_MENU)
      #if CASELIGHT_USES_BRIGHTNESS
        enableLiveCaseLightBrightness = true;  // Allow live update of brightness in control menu
        MENU_ITEM(ICON_CaseLight, MSG_CASE_LIGHT, onDrawSubMenu, drawCaseLightMenu);
      #else
        EDIT_ITEM(ICON_CaseLight, MSG_CASE_LIGHT, onDrawChkbMenu, setCaseLight, &caselight.on);
      #endif
    #endif
    #if ENABLED(LED_CONTROL_MENU)
      enableLiveLedColor = true;  // Allow live update of color in control menu
      MENU_ITEM(ICON_LedControl, MSG_LED_CONTROL, onDrawSubMenu, drawLedControlMenu);
    #endif
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawWriteEeprom, writeEEPROM);
      MENU_ITEM(ICON_ReadEEPROM, MSG_LOAD_EEPROM, onDrawReadEeprom, readEEPROM);
      MENU_ITEM(ICON_ResetEEPROM, MSG_RESTORE_DEFAULTS, onDrawResetEeprom, resetEEPROM);
    #endif
    MENU_ITEM(ICON_Reboot, MSG_RESET_PRINTER, onDrawMenuItem, rebootPrinter);
    MENU_ITEM(ICON_Info, MSG_INFO_SCREEN, onDrawInfoSubMenu, gotoInfoMenu);
  }
  ui.reset_status(true);
  updateMenu(controlMenu);
}

void drawAdvancedSettingsMenu() {
  checkkey = ID_Menu;
  if (SET_MENU(advancedSettingsMenu, MSG_ADVANCED_SETTINGS, 24)) {
    BACK_ITEM(gotoMainMenu);
    #if ENABLED(EEPROM_SETTINGS)
      MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawMenuItem, writeEEPROM);
    #endif
    #if HAS_MESH
      MENU_ITEM(ICON_Mesh, MSG_MESH_LEVELING, onDrawSubMenu, drawMeshSetMenu);
    #endif
    #if HAS_BED_PROBE
      MENU_ITEM(ICON_Probe, MSG_ZPROBE_SETTINGS, onDrawSubMenu, drawProbeSetMenu);
    #endif
    #if HAS_HOME_OFFSET
      MENU_ITEM(ICON_HomeOffset, MSG_SET_HOME_OFFSETS, onDrawSubMenu, drawHomeOffsetMenu);
    #endif
    MENU_ITEM(ICON_FilSet, MSG_FILAMENT_SET, onDrawSubMenu, drawFilSetMenu);
    #if ENABLED(PIDTEMP) && ANY(PID_AUTOTUNE_MENU, PID_EDIT_MENU)
      MENU_ITEM_F(ICON_PIDNozzle, STR_HOTEND_PID " Settings", onDrawSubMenu, drawHotendPIDMenu);
    #endif
    #if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)
      MENU_ITEM_F(ICON_MPCNozzle, "MPC Settings", onDrawSubMenu, drawHotendMPCMenu);
    #endif
    #if ENABLED(PIDTEMPBED) && ANY(PID_AUTOTUNE_MENU, PID_EDIT_MENU)
      MENU_ITEM_F(ICON_PIDBed, STR_BED_PID " Settings", onDrawSubMenu, drawBedPIDMenu);
    #endif
    #if HAS_TRINAMIC_CONFIG
      MENU_ITEM(ICON_TMCSet, MSG_TMC_DRIVERS, onDrawSubMenu, drawTrinamicConfigMenu);
    #endif
    #if HAS_ESDIAG
      MENU_ITEM_F(ICON_esDiag, "End-stops diag.", onDrawSubMenu, drawEndStopDiag);
    #endif
    #if ENABLED(PRINTCOUNTER)
      MENU_ITEM(ICON_PrintStats, MSG_INFO_STATS_MENU, onDrawSubMenu, gotoPrintStats);
      MENU_ITEM(ICON_PrintStatsReset, MSG_INFO_PRINT_COUNT_RESET, onDrawSubMenu, printStatsReset);
    #endif
    #if HAS_LOCKSCREEN
      MENU_ITEM(ICON_Lock, MSG_LOCKSCREEN, onDrawMenuItem, dwinLockScreen);
    #endif
    #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
      EDIT_ITEM(ICON_RemainTime, MSG_SCREEN_TIMEOUT, onDrawPIntMenu, setTimer, &ui.backlight_timeout_minutes);
    #endif
    #if ENABLED(SOUND_MENU_ITEM)
      EDIT_ITEM(ICON_Sound, MSG_SOUND_ENABLE, onDrawChkbMenu, setEnableSound, &ui.sound_on);
    #endif
    #if ENABLED(POWER_LOSS_RECOVERY)
      EDIT_ITEM(ICON_Pwrlossr, MSG_OUTAGE_RECOVERY, onDrawChkbMenu, setPwrLossr, &recovery.enabled);
    #endif
    #if HAS_GCODE_PREVIEW
      EDIT_ITEM(ICON_File, MSG_HAS_PREVIEW, onDrawChkbMenu, setPreview, &hmiData.enablePreview);
    #endif
    #if ENABLED(PROUI_MEDIASORT)
      EDIT_ITEM(ICON_File, MSG_MEDIA_SORT, onDrawChkbMenu, setMediaSort, &hmiData.mediaSort);
    #endif
    EDIT_ITEM(ICON_File, MSG_MEDIA_UPDATE, onDrawChkbMenu, setMediaAutoMount, &hmiData.mediaAutoMount);
    #if ENABLED(BAUD_RATE_GCODE)
      EDIT_ITEM_F(ICON_SetBaudRate, "115K baud", onDrawChkbMenu, setBaudRate, &hmiData.baud115K);
    #endif
    #if HAS_LCD_BRIGHTNESS
      EDIT_ITEM(ICON_Brightness, MSG_BRIGHTNESS, onDrawPInt8Menu, setBrightness, &ui.brightness);
      MENU_ITEM(ICON_Box, MSG_BRIGHTNESS_OFF, onDrawMenuItem, turnOffBacklight);
    #endif
    #if HAS_CUSTOM_COLORS
      MENU_ITEM(ICON_Scolor, MSG_COLORS_SELECT, onDrawSubMenu, drawSelectColorsMenu);
    #endif
  }
  ui.reset_status(true);
  updateMenu(advancedSettingsMenu);
}

void drawMoveMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(moveMenu, selrect({192, 1, 42, 14}), MSG_MOVE_AXIS, 6)) {
    BACK_ITEM(drawPrepareMenu);
    EDIT_ITEM(ICON_Axis, MSG_LIVE_MOVE, onDrawChkbMenu, setLiveMove, &enableLiveMove);
    #if HAS_X_AXIS
      EDIT_ITEM(ICON_MoveX, MSG_MOVE_X, onDrawMoveX, setMoveX, &current_position.x);
    #endif
    #if HAS_Y_AXIS
      EDIT_ITEM(ICON_MoveY, MSG_MOVE_Y, onDrawMoveY, setMoveY, &current_position.y);
    #endif
    #if HAS_Z_AXIS
      EDIT_ITEM(ICON_MoveZ, MSG_MOVE_Z, onDrawMoveZ, setMoveZ, &current_position.z);
    #endif
    #if HAS_HOTEND
      gcode.process_subcommands_now(F("G92E0"));  // Reset extruder position
      EDIT_ITEM(ICON_Extruder, MSG_MOVE_E, onDrawMoveE, setMoveE, &current_position.e);
    #endif
  }
  updateMenu(moveMenu);
  if (!all_axes_trusted()) LCD_MESSAGE_F("WARNING: Current position unknown. Home axes.");
}

#if HAS_HOME_OFFSET

  void drawHomeOffsetMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(homeOffsetMenu, MSG_SET_HOME_OFFSETS, 4)) {
      BACK_ITEM(drawAdvancedSettingsMenu);
      #if HAS_X_AXIS
        EDIT_ITEM(ICON_HomeOffsetX, MSG_HOME_OFFSET_X, onDrawPFloatMenu, setHomeOffsetX, &home_offset.x);
      #endif
      #if HAS_Y_AXIS
        EDIT_ITEM(ICON_HomeOffsetY, MSG_HOME_OFFSET_Y, onDrawPFloatMenu, setHomeOffsetY, &home_offset.y);
      #endif
      #if HAS_Z_AXIS
        EDIT_ITEM(ICON_HomeOffsetZ, MSG_HOME_OFFSET_Z, onDrawPFloatMenu, setHomeOffsetZ, &home_offset.z);
      #endif
    }
    updateMenu(homeOffsetMenu);
  }

#endif // HAS_HOME_OFFSET

#if HAS_BED_PROBE

  void drawProbeSetMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(probeSettingsMenu, MSG_ZPROBE_SETTINGS, 9)) {
      BACK_ITEM(drawAdvancedSettingsMenu);
      #if HAS_X_AXIS
        EDIT_ITEM(ICON_ProbeOffsetX, MSG_ZPROBE_XOFFSET, onDrawPFloatMenu, setProbeOffsetX, &probe.offset.x);
      #endif
      #if HAS_Y_AXIS
        EDIT_ITEM(ICON_ProbeOffsetY, MSG_ZPROBE_YOFFSET, onDrawPFloatMenu, setProbeOffsetY, &probe.offset.y);
      #endif
      #if HAS_Z_AXIS
        EDIT_ITEM(ICON_ProbeOffsetZ, MSG_ZPROBE_ZOFFSET, onDrawPFloat2Menu, setProbeOffsetZ, &probe.offset.z);
      #endif
      #if ENABLED(BLTOUCH)
        MENU_ITEM(ICON_ProbeStow, MSG_MANUAL_STOW, onDrawMenuItem, probeStow);
        MENU_ITEM(ICON_ProbeDeploy, MSG_MANUAL_DEPLOY, onDrawMenuItem, probeDeploy);
        MENU_ITEM(ICON_BLTouchReset, MSG_BLTOUCH_RESET, onDrawMenuItem, bltouch._reset);
        #if HAS_BLTOUCH_HS_MODE
          EDIT_ITEM(ICON_HSMode, MSG_ENABLE_HS_MODE, onDrawChkbMenu, setHSMode, &bltouch.high_speed_mode);
        #endif
      #endif
      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        MENU_ITEM(ICON_ProbeTest, MSG_M48_TEST, onDrawMenuItem, probeTest);
      #endif
    }
    updateMenu(probeSettingsMenu);
  }

#endif // HAS_BED_PROBE

void drawFilSetMenu() {
  checkkey = ID_Menu;
  if (SET_MENU(filSetMenu, MSG_FILAMENT_SET, 9)) {
    BACK_ITEM(drawAdvancedSettingsMenu);
    #if HAS_FILAMENT_SENSOR
      EDIT_ITEM(ICON_Runout, MSG_RUNOUT_SENSOR, onDrawChkbMenu, setRunoutEnable, &runout.enabled);
    #endif
    #if HAS_FILAMENT_RUNOUT_DISTANCE
      EDIT_ITEM(ICON_Runout, MSG_RUNOUT_DISTANCE_MM, onDrawPFloatMenu, setRunoutDistance, &runout.runout_distance());
    #endif
    #if ENABLED(PREVENT_COLD_EXTRUSION)
      EDIT_ITEM(ICON_ExtrudeMinT, MSG_EXTRUDER_MIN_TEMP, onDrawPIntMenu, setExtMinT, &hmiData.extMinT);
    #endif
    #if ENABLED(CONFIGURE_FILAMENT_CHANGE)
      EDIT_ITEM(ICON_FilLoad, MSG_FILAMENT_LOAD, onDrawPFloatMenu, setFilLoad, &fc_settings[0].load_length);
      EDIT_ITEM(ICON_FilUnload, MSG_FILAMENT_UNLOAD, onDrawPFloatMenu, setFilUnload, &fc_settings[0].unload_length);
    #endif
    #if ENABLED(FWRETRACT)
      MENU_ITEM(ICON_FWRetract, MSG_FWRETRACT, onDrawSubMenu, drawFWRetractMenu);
    #endif
  }
  updateMenu(filSetMenu);
}

#if ALL(CASE_LIGHT_MENU, CASELIGHT_USES_BRIGHTNESS)

  void drawCaseLightMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(caseLightMenu, MSG_CASE_LIGHT, 3)) {
      BACK_ITEM(drawControlMenu);
      EDIT_ITEM(ICON_CaseLight, MSG_CASE_LIGHT, onDrawChkbMenu, setCaseLight, &caselight.on);
      EDIT_ITEM(ICON_Brightness, MSG_CASE_LIGHT_BRIGHTNESS, onDrawPInt8Menu, setCaseLightBrightness, &caselight.brightness);
    }
    updateMenu(caseLightMenu);
  }

#endif

#if ENABLED(LED_CONTROL_MENU)

  void drawLedControlMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(ledControlMenu, MSG_LED_CONTROL, 10)) {
      BACK_ITEM((currentMenu == tuneMenu) ? drawTuneMenu : drawControlMenu);
      #if !ALL(CASE_LIGHT_MENU, CASE_LIGHT_USE_NEOPIXEL)
        EDIT_ITEM(ICON_LedControl, MSG_LIGHTS, onDrawChkbMenu, setLedStatus, &leds.lights_on);
      #endif
      #if HAS_COLOR_LEDS
        #if ENABLED(LED_COLOR_PRESETS)
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_WHITE, onDrawMenuItem,  leds.set_white);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_RED, onDrawMenuItem,    leds.set_red);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_ORANGE, onDrawMenuItem, leds.set_orange);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_YELLOW, onDrawMenuItem, leds.set_yellow);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_GREEN, onDrawMenuItem,  leds.set_green);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_BLUE, onDrawMenuItem,   leds.set_blue);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_INDIGO, onDrawMenuItem, leds.set_indigo);
          MENU_ITEM(ICON_LedControl, MSG_SET_LEDS_VIOLET, onDrawMenuItem, leds.set_violet);
        #else
          EDIT_ITEM(ICON_LedControl, MSG_COLORS_RED, onDrawPInt8Menu, setLEDColorR, &leds.color.r);
          EDIT_ITEM(ICON_LedControl, MSG_COLORS_GREEN, onDrawPInt8Menu, setLEDColorG, &leds.color.g);
          EDIT_ITEM(ICON_LedControl, MSG_COLORS_BLUE, onDrawPInt8Menu, setLEDColorB, &leds.color.b);
          #if ENABLED(HAS_WHITE_LED)
            EDIT_ITEM(ICON_LedControl, MSG_COLORS_WHITE, onDrawPInt8Menu, setLEDColorW, &leds.color.w);
          #endif
        #endif
      #endif
    }
    updateMenu(ledControlMenu);
  }

#endif // LED_CONTROL_MENU

void drawTuneMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(tuneMenu, selrect({73, 2, 28, 12}), MSG_TUNE, 20)) {
    BACK_ITEM(gotoPrintProcess);
    EDIT_ITEM(ICON_Speed, MSG_SPEED, onDrawSpeedItem, setSpeed, &feedrate_percentage);
    #if HAS_HOTEND
      hotendTargetItem = EDIT_ITEM(ICON_HotendTemp, MSG_UBL_SET_TEMP_HOTEND, onDrawHotendTemp, setHotendTemp, &thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      bedTargetItem = EDIT_ITEM(ICON_BedTemp, MSG_UBL_SET_TEMP_BED, onDrawBedTemp, setBedTemp, &thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      fanSpeedItem = EDIT_ITEM(ICON_FanSpeed, MSG_FAN_SPEED, onDrawFanSpeed, setFanSpeed, &thermalManager.fan_speed[0]);
    #endif
    #if ALL(HAS_ZOFFSET_ITEM, HAS_BED_PROBE, BABYSTEP_ZPROBE_OFFSET, BABYSTEPPING)
      EDIT_ITEM(ICON_Zoffset, MSG_BABYSTEP_PROBE_Z, onDrawZOffset, setZOffset, &BABY_Z_VAR);
    #elif ALL(HAS_ZOFFSET_ITEM, MESH_BED_LEVELING, BABYSTEPPING)
      EDIT_ITEM(ICON_Zoffset, MSG_HOME_OFFSET_Z, onDrawPFloat2Menu, setZOffset, &BABY_Z_VAR);
    #endif
    EDIT_ITEM(ICON_Flow, MSG_FLOW, onDrawPIntMenu, setFlow, &planner.flow_percentage[0]);
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      MENU_ITEM(ICON_FilMan, MSG_FILAMENTCHANGE, onDrawMenuItem, changeFilament);
    #endif
    #if HAS_FILAMENT_SENSOR
      EDIT_ITEM(ICON_Runout, MSG_RUNOUT_SENSOR, onDrawChkbMenu, setRunoutEnable, &runout.enabled);
    #endif
    #if ENABLED(PROUI_ITEM_PLR)
      EDIT_ITEM(ICON_Pwrlossr, MSG_OUTAGE_RECOVERY, onDrawChkbMenu, setPwrLossr, &recovery.enabled);
    #endif
    #if ENABLED(FWRETRACT)
      MENU_ITEM(ICON_FWRetract, MSG_FWRETRACT, onDrawSubMenu, drawFWRetractMenu);
    #endif
    #if ENABLED(PROUI_ITEM_JD)
      EDIT_ITEM(ICON_JDmm, MSG_JUNCTION_DEVIATION, onDrawPFloat3Menu, setJDmm, &planner.junction_deviation_mm);
    #endif
    #if ENABLED(PROUI_ITEM_ADVK)
      EDIT_ITEM(ICON_MaxAccelerated, MSG_ADVANCE_K, onDrawPFloat3Menu, setLA_K, &planner.extruder_advance_K[0]);
    #endif
    #if HAS_LOCKSCREEN
      MENU_ITEM(ICON_Lock, MSG_LOCKSCREEN, onDrawMenuItem, dwinLockScreen);
    #endif
    #if HAS_LCD_BRIGHTNESS
      EDIT_ITEM(ICON_Brightness, MSG_BRIGHTNESS, onDrawPInt8Menu, setBrightness, &ui.brightness);
      MENU_ITEM(ICON_Box, MSG_BRIGHTNESS_OFF, onDrawMenuItem, turnOffBacklight);
    #endif
    #if ENABLED(EDITABLE_DISPLAY_TIMEOUT)
      EDIT_ITEM(ICON_RemainTime, MSG_SCREEN_TIMEOUT, onDrawPIntMenu, setTimer, &ui.backlight_timeout_minutes);
    #endif
    #if ALL(PROUI_TUNING_GRAPH, PROUI_ITEM_PLOT)
      MENU_ITEM(ICON_PIDNozzle, MSG_HOTEND_TEMP_GRAPH, onDrawMenuItem, drawHPlot);
      MENU_ITEM(ICON_PIDBed, MSG_BED_TEMP_GRAPH, onDrawMenuItem, drawBPlot);
    #endif
    #if ENABLED(CASE_LIGHT_MENU)
      EDIT_ITEM(ICON_CaseLight, MSG_CASE_LIGHT, onDrawChkbMenu, setCaseLight, &caselight.on);
      #if CASELIGHT_USES_BRIGHTNESS
        // Avoid heavy interference with print job disabling live update of brightness in tune menu
        enableLiveCaseLightBrightness = false;
        EDIT_ITEM(ICON_Brightness, MSG_CASE_LIGHT_BRIGHTNESS, onDrawPInt8Menu, setCaseLightBrightness, &caselight.brightness);
      #endif
      #if ENABLED(LED_CONTROL_MENU)
        // Avoid heavy interference with print job disabling live update of color in tune menu
        enableLiveLedColor = false;
        MENU_ITEM(ICON_LedControl, MSG_LED_CONTROL, onDrawSubMenu, drawLedControlMenu);
      #endif
    #elif ENABLED(LED_CONTROL_MENU) && DISABLED(CASE_LIGHT_USE_NEOPIXEL)
      EDIT_ITEM(ICON_LedControl, MSG_LIGHTS, onDrawChkbMenu, setLedStatus, &leds.lights_on);
    #endif
  }
  updateMenu(tuneMenu);
}

#if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
  void setAdaptiveStepSmoothing() {
    toggleCheckboxLine(stepper.adaptive_step_smoothing_enabled);
  }
#endif

#if ENABLED(SHAPING_MENU)
  void applyShapingFreq() { stepper.set_shaping_frequency(hmiValue.axis, menuData.value / 100); }
  void applyShapingZeta() { stepper.set_shaping_damping_ratio(hmiValue.axis, menuData.value / 100); }

  #if ENABLED(INPUT_SHAPING_X)
    void onDrawShapingXFreq(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_frequency(X_AXIS)); }
    void onDrawShapingXZeta(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_damping_ratio(X_AXIS)); }
    void setShapingXFreq() { hmiValue.axis = X_AXIS; setFloatOnClick(0, 200, 2, stepper.get_shaping_frequency(X_AXIS), applyShapingFreq); }
    void setShapingXZeta() { hmiValue.axis = X_AXIS; setFloatOnClick(0, 1, 2, stepper.get_shaping_damping_ratio(X_AXIS), applyShapingZeta); }
  #endif

  #if ENABLED(INPUT_SHAPING_Y)
    void onDrawShapingYFreq(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_frequency(Y_AXIS)); }
    void onDrawShapingYZeta(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_damping_ratio(Y_AXIS)); }
    void setShapingYFreq() { hmiValue.axis = Y_AXIS; setFloatOnClick(0, 200, 2, stepper.get_shaping_frequency(Y_AXIS), applyShapingFreq); }
    void setShapingYZeta() { hmiValue.axis = Y_AXIS; setFloatOnClick(0, 1, 2, stepper.get_shaping_damping_ratio(Y_AXIS), applyShapingZeta); }
  #endif

  #if ENABLED(INPUT_SHAPING_Z)
    void onDrawShapingZFreq(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_frequency(Z_AXIS)); }
    void onDrawShapingZZeta(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, stepper.get_shaping_damping_ratio(Z_AXIS)); }
    void setShapingZFreq() { hmiValue.axis = Z_AXIS; setFloatOnClick(0, 200, 2, stepper.get_shaping_frequency(Z_AXIS), applyShapingFreq); }
    void setShapingZZeta() { hmiValue.axis = Z_AXIS; setFloatOnClick(0, 1, 2, stepper.get_shaping_damping_ratio(Z_AXIS), applyShapingZeta); }
  #endif

  void drawInputShaping_menu() {
    checkkey = ID_Menu;
    if (SET_MENU(inputShapingMenu, MSG_INPUT_SHAPING, 1 PLUS_TERN0(INPUT_SHAPING_X, 2) PLUS_TERN0(INPUT_SHAPING_Y, 2) PLUS_TERN0(INPUT_SHAPING_Z, 2))) {
      BACK_ITEM(drawMotionMenu);
      #if ENABLED(INPUT_SHAPING_X)
        MENU_ITEM(ICON_ShapingX, MSG_SHAPING_A_FREQ, onDrawShapingXFreq, setShapingXFreq);
        MENU_ITEM(ICON_ShapingX, MSG_SHAPING_A_ZETA, onDrawShapingXZeta, setShapingXZeta);
      #endif
      #if ENABLED(INPUT_SHAPING_Y)
        MENU_ITEM(ICON_ShapingY, MSG_SHAPING_B_FREQ, onDrawShapingYFreq, setShapingYFreq);
        MENU_ITEM(ICON_ShapingY, MSG_SHAPING_B_ZETA, onDrawShapingYZeta, setShapingYZeta);
      #endif
      #if ENABLED(INPUT_SHAPING_Z)
        MENU_ITEM(ICON_ShapingZ, MSG_SHAPING_C_FREQ, onDrawShapingZFreq, setShapingZFreq);
        MENU_ITEM(ICON_ShapingZ, MSG_SHAPING_C_ZETA, onDrawShapingZZeta, setShapingZZeta);
      #endif
    }
    updateMenu(inputShapingMenu);
  }
#endif

#if HAS_TRINAMIC_CONFIG
  #if AXIS_IS_TMC(X)
    void setXTMCCurrent() { setPIntOnClick(MIN_TMC_CURRENT, MAX_TMC_CURRENT, []{ stepperX.refresh_stepper_current(); }); }
  #endif
  #if AXIS_IS_TMC(Y)
    void setYTMCCurrent() { setPIntOnClick(MIN_TMC_CURRENT, MAX_TMC_CURRENT, []{ stepperY.refresh_stepper_current(); }); }
  #endif
  #if AXIS_IS_TMC(Z)
    void setZTMCCurrent() { setPIntOnClick(MIN_TMC_CURRENT, MAX_TMC_CURRENT, []{ stepperZ.refresh_stepper_current(); }); }
  #endif
  #if AXIS_IS_TMC(E0)
    void setETMCCurrent() { setPIntOnClick(MIN_TMC_CURRENT, MAX_TMC_CURRENT, []{ stepperE0.refresh_stepper_current(); }); }
  #endif

  void drawTrinamicConfigMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(trinamicConfigMenu, MSG_TMC_DRIVERS, 5)) {
      BACK_ITEM(drawAdvancedSettingsMenu);
      #if AXIS_IS_TMC(X)
        EDIT_ITEM(ICON_TMCXSet, MSG_TMC_ACURRENT, onDrawPIntMenu, setXTMCCurrent, &stepperX.val_mA);
      #endif
      #if AXIS_IS_TMC(Y)
        EDIT_ITEM(ICON_TMCYSet, MSG_TMC_BCURRENT, onDrawPIntMenu, setYTMCCurrent, &stepperY.val_mA);
      #endif
      #if AXIS_IS_TMC(Z)
        EDIT_ITEM(ICON_TMCZSet, MSG_TMC_CCURRENT, onDrawPIntMenu, setZTMCCurrent, &stepperZ.val_mA);
      #endif
      #if AXIS_IS_TMC(E0)
        EDIT_ITEM(ICON_TMCESet, MSG_TMC_ECURRENT, onDrawPIntMenu, setETMCCurrent, &stepperE0.val_mA);
      #endif
    }
    updateMenu(trinamicConfigMenu);
  }
#endif

void drawMotionMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(motionMenu, selrect({1, 16, 28, 13}), MSG_MOTION, 11)) {
    BACK_ITEM(drawControlMenu);
    MENU_ITEM(ICON_MaxSpeed, MSG_SPEED, onDrawSpeed, drawMaxSpeedMenu);
    MENU_ITEM(ICON_MaxAccelerated, MSG_ACCELERATION, onDrawAcc, drawMaxAccelMenu);
    #if ENABLED(CLASSIC_JERK)
      MENU_ITEM(ICON_MaxJerk, MSG_JERK, onDrawJerk, drawMaxJerkMenu);
    #elif HAS_JUNCTION_DEVIATION
      EDIT_ITEM(ICON_JDmm, MSG_JUNCTION_DEVIATION, onDrawPFloat3Menu, setJDmm, &planner.junction_deviation_mm);
    #endif
    #if ENABLED(EDITABLE_STEPS_PER_UNIT)
      MENU_ITEM(ICON_Step, MSG_STEPS_PER_MM, onDrawSteps, drawStepsMenu);
    #endif
    #if ENABLED(EDITABLE_HOMING_FEEDRATE)
      MENU_ITEM(ICON_Homing, MSG_HOMING_FEEDRATE, onDrawSubMenu, drawHomingFRMenu);
    #endif
    #if ENABLED(LIN_ADVANCE)
      EDIT_ITEM(ICON_MaxAccelerated, MSG_ADVANCE_K, onDrawPFloat3Menu, setLA_K, &planner.extruder_advance_K[0]);
    #endif
    #if ENABLED(SHAPING_MENU)
      MENU_ITEM(ICON_InputShaping, MSG_INPUT_SHAPING, onDrawSubMenu, drawInputShaping_menu);
    #endif
    #if ENABLED(ADAPTIVE_STEP_SMOOTHING_TOGGLE)
      EDIT_ITEM(ICON_UBLActive, MSG_STEP_SMOOTHING, onDrawChkbMenu, setAdaptiveStepSmoothing, &stepper.adaptive_step_smoothing_enabled);
    #endif
    EDIT_ITEM(ICON_Flow, MSG_FLOW, onDrawPIntMenu, setFlow, &planner.flow_percentage[0]);
    EDIT_ITEM(ICON_Speed, MSG_SPEED, onDrawPIntMenu, setSpeed, &feedrate_percentage);
  }
  updateMenu(motionMenu);
}

#if ALL(ADVANCED_PAUSE_FEATURE, HAS_PREHEAT)

    void drawPreheatHotendMenu() {
      checkkey = ID_Menu;
      if (SET_MENU(preheatHotendMenu, MSG_PREHEAT_HOTEND, 1 + PREHEAT_COUNT)) {
        BACK_ITEM(drawFilamentManMenu);
        #define _ITEM_PREHEAT_HE(N) MENU_ITEM(ICON_Preheat##N, MSG_PREHEAT_##N, onDrawMenuItem, DoPreheatHotend##N);
        REPEAT_1(PREHEAT_COUNT, _ITEM_PREHEAT_HE)
      }
      updateMenu(preheatHotendMenu);
    }

#endif

void drawFilamentManMenu() {
  checkkey = ID_Menu;
  if (SET_MENU(filamentMenu, MSG_FILAMENT_MAN, 6)) {
    BACK_ITEM(drawPrepareMenu);
    #if ENABLED(NOZZLE_PARK_FEATURE)
      MENU_ITEM(ICON_Park, MSG_FILAMENT_PARK_ENABLED, onDrawMenuItem, parkHead);
    #endif
    #if ENABLED(ADVANCED_PAUSE_FEATURE)
      #if HAS_PREHEAT
        MENU_ITEM(ICON_SetEndTemp, MSG_PREHEAT_HOTEND, onDrawSubMenu, drawPreheatHotendMenu);
      #endif
      MENU_ITEM(ICON_FilMan, MSG_FILAMENTCHANGE, onDrawMenuItem, changeFilament);
    #endif
    #if ENABLED(FILAMENT_LOAD_UNLOAD_GCODES)
      MENU_ITEM(ICON_FilUnload, MSG_FILAMENTUNLOAD, onDrawMenuItem, unloadFilament);
      MENU_ITEM(ICON_FilLoad, MSG_FILAMENTLOAD, onDrawMenuItem, loadFilament);
    #endif
  }
  updateMenu(filamentMenu);
}

#if ENABLED(MESH_BED_LEVELING)

  void drawManualMeshMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(manualMeshMenu, MSG_UBL_MANUAL_MESH, 6)) {
      BACK_ITEM(drawPrepareMenu);
      MENU_ITEM(ICON_ManualMesh, MSG_LEVEL_BED, onDrawMenuItem, manualMeshStart);
      mMeshMoveZItem = EDIT_ITEM(ICON_Zoffset, MSG_MOVE_Z, onDrawMMeshMoveZ, setMMeshMoveZ, &current_position.z);
      MENU_ITEM(ICON_Axis, MSG_UBL_CONTINUE_MESH, onDrawMenuItem, manualMeshContinue);
      MENU_ITEM(ICON_MeshViewer, MSG_MESH_VIEW, onDrawSubMenu, dwinMeshViewer);
      MENU_ITEM(ICON_MeshSave, MSG_UBL_SAVE_MESH, onDrawMenuItem, manualMeshSave);
    }
    updateMenu(manualMeshMenu);
  }

#endif // MESH_BED_LEVELING

#if HAS_PREHEAT

  void drawPreheatMenu(const bool notCurrent) {
    checkkey = ID_Menu;
    if (notCurrent) {
      BACK_ITEM(drawTemperatureMenu);
      #if HAS_HOTEND
        EDIT_ITEM(ICON_HotendTemp, MSG_UBL_SET_TEMP_HOTEND, onDrawSetPreheatHotend, setPreheatEndTemp, &ui.material_preset[hmiValue.select].hotend_temp);
      #endif
      #if HAS_HEATED_BED
        EDIT_ITEM(ICON_BedTemp, MSG_UBL_SET_TEMP_BED, onDrawSetPreheatBed, setPreheatBedTemp, &ui.material_preset[hmiValue.select].bed_temp);
      #endif
      #if HAS_FAN
        EDIT_ITEM(ICON_FanSpeed, MSG_FAN_SPEED, onDrawSetPreheatFan, setPreheatFanSpeed, &ui.material_preset[hmiValue.select].fan_speed);
      #endif
      #if ENABLED(EEPROM_SETTINGS)
        MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawWriteEeprom, writeEEPROM);
      #endif
    }
    updateMenu(preheatMenu);
  }

  #define _preheatMenu(N) \
    void drawPreheat## N ##Menu() { \
      hmiValue.select = (N) - 1; \
      drawPreheatMenu(SET_MENU(preheatMenu, MSG_PREHEAT_## N ##_SETTINGS, 5)); \
    }
  REPEAT_1(PREHEAT_COUNT, _preheatMenu)

#endif // HAS_PREHEAT

void drawTemperatureMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(temperatureMenu, selrect({236, 2, 28, 12}), MSG_TEMPERATURE, 4 + PREHEAT_COUNT)) {
    BACK_ITEM(drawControlMenu);
    #if HAS_HOTEND
      hotendTargetItem = EDIT_ITEM(ICON_HotendTemp, MSG_UBL_SET_TEMP_HOTEND, onDrawHotendTemp, setHotendTemp, &thermalManager.temp_hotend[0].target);
    #endif
    #if HAS_HEATED_BED
      bedTargetItem = EDIT_ITEM(ICON_BedTemp, MSG_UBL_SET_TEMP_BED, onDrawBedTemp, setBedTemp, &thermalManager.temp_bed.target);
    #endif
    #if HAS_FAN
      fanSpeedItem = EDIT_ITEM(ICON_FanSpeed, MSG_FAN_SPEED, onDrawFanSpeed, setFanSpeed, &thermalManager.fan_speed[0]);
    #endif
    #if HAS_PREHEAT
      #define _ITEM_SETPREHEAT(N) MENU_ITEM(ICON_SetPreheat##N, MSG_PREHEAT_## N ##_SETTINGS, onDrawSubMenu, drawPreheat## N ##Menu);
      REPEAT_1(PREHEAT_COUNT, _ITEM_SETPREHEAT)
    #endif
  }
  updateMenu(temperatureMenu);
}

void drawMaxSpeedMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(maxSpeedMenu, selrect({1, 16, 28, 13}), MSG_MAX_SPEED, 5)) {
    BACK_ITEM(drawMotionMenu);
    #if HAS_X_AXIS
      EDIT_ITEM(ICON_MaxSpeedX, MSG_VMAX_A, onDrawMaxSpeedX, setMaxSpeedX, &planner.settings.max_feedrate_mm_s[X_AXIS]);
    #endif
    #if HAS_Y_AXIS
      EDIT_ITEM(ICON_MaxSpeedY, MSG_VMAX_B, onDrawMaxSpeedY, setMaxSpeedY, &planner.settings.max_feedrate_mm_s[Y_AXIS]);
    #endif
    #if HAS_Z_AXIS
      EDIT_ITEM(ICON_MaxSpeedZ, MSG_VMAX_C, onDrawMaxSpeedZ, setMaxSpeedZ, &planner.settings.max_feedrate_mm_s[Z_AXIS]);
    #endif
    #if HAS_HOTEND
      EDIT_ITEM(ICON_MaxSpeedE, MSG_VMAX_E, onDrawMaxSpeedE, setMaxSpeedE, &planner.settings.max_feedrate_mm_s[E_AXIS]);
    #endif
  }
  updateMenu(maxSpeedMenu);
}

void drawMaxAccelMenu() {
  checkkey = ID_Menu;
  if (SET_MENU_R(maxAccelMenu, selrect({1, 16, 28, 13}), MSG_AMAX_EN, 5)) {
    BACK_ITEM(drawMotionMenu);
    #if HAS_X_AXIS
      EDIT_ITEM(ICON_MaxAccX, MSG_AMAX_A, onDrawMaxAccelX, setMaxAccelX, &planner.settings.max_acceleration_mm_per_s2[X_AXIS]);
    #endif
    #if HAS_Y_AXIS
      EDIT_ITEM(ICON_MaxAccY, MSG_AMAX_B, onDrawMaxAccelY, setMaxAccelY, &planner.settings.max_acceleration_mm_per_s2[Y_AXIS]);
    #endif
    #if HAS_Z_AXIS
      EDIT_ITEM(ICON_MaxAccZ, MSG_AMAX_C, onDrawMaxAccelZ, setMaxAccelZ, &planner.settings.max_acceleration_mm_per_s2[Z_AXIS]);
    #endif
    #if HAS_HOTEND
      EDIT_ITEM(ICON_MaxAccE, MSG_AMAX_E, onDrawMaxAccelE, setMaxAccelE, &planner.settings.max_acceleration_mm_per_s2[E_AXIS]);
    #endif
  }
  updateMenu(maxAccelMenu);
}

#if ENABLED(CLASSIC_JERK)

  void drawMaxJerkMenu() {
    checkkey = ID_Menu;
    if (SET_MENU_R(maxJerkMenu, selrect({1, 16, 28, 13}), MSG_JERK, 5)) {
      BACK_ITEM(drawMotionMenu);
      #if HAS_X_AXIS
        EDIT_ITEM(ICON_MaxSpeedJerkX, MSG_VA_JERK, onDrawMaxJerkX, setMaxJerkX, &planner.max_jerk.x);
      #endif
      #if HAS_Y_AXIS
        EDIT_ITEM(ICON_MaxSpeedJerkY, MSG_VB_JERK, onDrawMaxJerkY, setMaxJerkY, &planner.max_jerk.y);
      #endif
      #if HAS_Z_AXIS
        EDIT_ITEM(ICON_MaxSpeedJerkZ, MSG_VC_JERK, onDrawMaxJerkZ, setMaxJerkZ, &planner.max_jerk.z);
      #endif
      #if HAS_HOTEND
        EDIT_ITEM(ICON_MaxSpeedJerkE, MSG_VE_JERK, onDrawMaxJerkE, setMaxJerkE, &planner.max_jerk.e);
      #endif
    }
    updateMenu(maxJerkMenu);
  }

#endif // CLASSIC_JERK

#if ENABLED(EDITABLE_STEPS_PER_UNIT)

  void drawStepsMenu() {
    checkkey = ID_Menu;
    if (SET_MENU_R(stepsMenu, selrect({1, 16, 28, 13}), MSG_STEPS_PER_MM, 5)) {
      BACK_ITEM(drawMotionMenu);
      #if HAS_X_AXIS
        EDIT_ITEM(ICON_StepX, MSG_A_STEPS, onDrawStepsX, setStepsX, &planner.settings.axis_steps_per_mm[X_AXIS]);
      #endif
      #if HAS_Y_AXIS
        EDIT_ITEM(ICON_StepY, MSG_B_STEPS, onDrawStepsY, setStepsY, &planner.settings.axis_steps_per_mm[Y_AXIS]);
      #endif
      #if HAS_Z_AXIS
        EDIT_ITEM(ICON_StepZ, MSG_C_STEPS, onDrawStepsZ, setStepsZ, &planner.settings.axis_steps_per_mm[Z_AXIS]);
      #endif
      #if HAS_HOTEND
        EDIT_ITEM(ICON_StepE, MSG_E_STEPS, onDrawStepsE, setStepsE, &planner.settings.axis_steps_per_mm[E_AXIS]);
      #endif
    }
    updateMenu(stepsMenu);
  }

#endif

#if ENABLED(EDITABLE_HOMING_FEEDRATE)

  void drawHomingFRMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(homingFRMenu, MSG_HOMING_FEEDRATE, 4)) {
      BACK_ITEM(drawMotionMenu);
      #if HAS_X_AXIS
        uint16_t xhome = static_cast<uint16_t>(homing_feedrate_mm_m.x);
        EDIT_ITEM(ICON_MaxSpeedJerkX, MSG_HOMING_FEEDRATE_X, onDrawPIntMenu, setHomingX, &xhome);
      #endif
      #if HAS_Y_AXIS
        uint16_t yhome = static_cast<uint16_t>(homing_feedrate_mm_m.y);
        EDIT_ITEM(ICON_MaxSpeedJerkY, MSG_HOMING_FEEDRATE_Y, onDrawPIntMenu, setHomingY, &yhome);
      #endif
      #if HAS_Z_AXIS
        uint16_t zhome = static_cast<uint16_t>(homing_feedrate_mm_m.z);
        EDIT_ITEM(ICON_MaxSpeedJerkZ, MSG_HOMING_FEEDRATE_Z, onDrawPIntMenu, setHomingZ, &zhome);
      #endif
    }
    updateMenu(homingFRMenu);
  }

#endif

//=============================================================================
// UI editable custom colors
//=============================================================================

#if HAS_CUSTOM_COLORS

  void restoreDefaultColors() {
    dwinSetColorDefaults();
    DWINUI::setColors(hmiData.colorText, hmiData.colorBackground, hmiData.colorStatusBg);
    dwinRedrawScreen();
  }

  void selColor() {
    menuData.intPtr = (int16_t*)static_cast<MenuItemPtr*>(currentMenu->selectedItem())->value;
    hmiValue.color.r = GetRColor(*menuData.intPtr); // Red
    hmiValue.color.g = GetGColor(*menuData.intPtr); // Green
    hmiValue.color.b = GetBColor(*menuData.intPtr); // Blue
    drawGetColorMenu();
  }

  void liveRGBColor() {
    hmiValue.color[currentMenu->line() - 2] = menuData.value;
    const uint16_t color = RGB(hmiValue.color.r, hmiValue.color.g, hmiValue.color.b);
    dwinDrawRectangle(1, color, 20, 315, DWIN_WIDTH - 20, 335);
  }
  void setRGBColor() {
    const uint8_t color = static_cast<MenuItem*>(currentMenu->selectedItem())->icon;
    setIntOnClick(0, (color == 1) ? 63 : 31, hmiValue.color[color], nullptr, liveRGBColor);
  }

  void dwinApplyColor() {
    *menuData.intPtr = RGB(hmiValue.color.r, hmiValue.color.g, hmiValue.color.b);
    DWINUI::setColors(hmiData.colorText, hmiData.colorBackground, hmiData.colorStatusBg);
    drawSelectColorsMenu();
    hash_changed = true;
    LCD_MESSAGE(MSG_COLORS_APPLIED);
    dwinDrawDashboard();
  }

  void drawSelectColorsMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(selectColorMenu, MSG_COLORS_SELECT, 20)) {
      BACK_ITEM(drawAdvancedSettingsMenu);
      MENU_ITEM(ICON_StockConfiguration, MSG_RESTORE_DEFAULTS, onDrawMenuItem, restoreDefaultColors);
      EDIT_ITEM_F(0, "Screen Background", onDrawSelColorItem, selColor, &hmiData.colorBackground);
      EDIT_ITEM_F(0, "Cursor", onDrawSelColorItem, selColor, &hmiData.colorCursor);
      EDIT_ITEM_F(0, "Title Background", onDrawSelColorItem, selColor, &hmiData.colorTitleBg);
      EDIT_ITEM_F(0, "Title Text", onDrawSelColorItem, selColor, &hmiData.colorTitleTxt);
      EDIT_ITEM_F(0, "Text", onDrawSelColorItem, selColor, &hmiData.colorText);
      EDIT_ITEM_F(0, "Selected", onDrawSelColorItem, selColor, &hmiData.colorSelected);
      EDIT_ITEM_F(0, "Split Line", onDrawSelColorItem, selColor, &hmiData.colorSplitLine);
      EDIT_ITEM_F(0, "Highlight", onDrawSelColorItem, selColor, &hmiData.colorHighlight);
      EDIT_ITEM_F(0, "Status Background", onDrawSelColorItem, selColor, &hmiData.colorStatusBg);
      EDIT_ITEM_F(0, "Status Text", onDrawSelColorItem, selColor, &hmiData.colorStatusTxt);
      EDIT_ITEM_F(0, "Popup Background", onDrawSelColorItem, selColor, &hmiData.colorPopupBg);
      EDIT_ITEM_F(0, "Popup Text", onDrawSelColorItem, selColor, &hmiData.colorPopupTxt);
      EDIT_ITEM_F(0, "Alert Background", onDrawSelColorItem, selColor, &hmiData.colorAlertBg);
      EDIT_ITEM_F(0, "Alert Text", onDrawSelColorItem, selColor, &hmiData.colorAlertTxt);
      EDIT_ITEM_F(0, "Percent Text", onDrawSelColorItem, selColor, &hmiData.colorPercentTxt);
      EDIT_ITEM_F(0, "Bar Fill", onDrawSelColorItem, selColor, &hmiData.colorBarfill);
      EDIT_ITEM_F(0, "Indicator value", onDrawSelColorItem, selColor, &hmiData.colorIndicator);
      EDIT_ITEM_F(0, "Coordinate value", onDrawSelColorItem, selColor, &hmiData.colorCoordinate);
    }
    updateMenu(selectColorMenu);
  }

  void drawGetColorMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(getColorMenu, MSG_COLORS_GET, 5)) {
      BACK_ITEM(dwinApplyColor);
      MENU_ITEM(ICON_Cancel, MSG_BUTTON_CANCEL, onDrawMenuItem, drawSelectColorsMenu);
      MENU_ITEM(0, MSG_COLORS_RED,   onDrawGetColorItem, setRGBColor);
      MENU_ITEM(1, MSG_COLORS_GREEN, onDrawGetColorItem, setRGBColor);
      MENU_ITEM(2, MSG_COLORS_BLUE,  onDrawGetColorItem, setRGBColor);
    }
    updateMenu(getColorMenu);
    dwinDrawRectangle(1, *menuData.intPtr, 20, 315, DWIN_WIDTH - 20, 335);
  }

#endif // HAS_CUSTOM_COLORS

//=============================================================================
// Nozzle and Bed PID/MPC
//=============================================================================

#if ANY(MPC_EDIT_MENU, MPC_AUTOTUNE_MENU)

  #if ENABLED(MPC_EDIT_MENU)
    void setHeaterPower() { setPFloatOnClick(1, 200, 1); }
    void setBlkHeatCapacity() { setPFloatOnClick(0, 40, 2); }
    void setSensorResponse() { setPFloatOnClick(0, 1, 4); }
    void setAmbientXfer() { setPFloatOnClick(0, 1, 4); }
    #if ENABLED(MPC_INCLUDE_FAN)
      void onDrawFanAdj(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 4, thermalManager.temp_hotend[0].fanCoefficient()); }
      void applyFanAdj() { thermalManager.temp_hotend[0].applyFanAdjustment(menuData.value / POW(10, 4)); }
      void setFanAdj() { setFloatOnClick(0, 1, 4, thermalManager.temp_hotend[0].fanCoefficient(), applyFanAdj); }
    #endif
  #endif

  void drawHotendMPCMenu() {
    checkkey = ID_Menu;
    if (SET_MENU_F(hotendMPCMenu, "MPC Settings", 7)) {
      MPC_t &mpc = thermalManager.temp_hotend[0].mpc;
      BACK_ITEM(drawAdvancedSettingsMenu);
      #if ENABLED(MPC_AUTOTUNE_MENU)
        MENU_ITEM(ICON_MPCNozzle, MSG_MPC_AUTOTUNE, onDrawMenuItem, []{ thermalManager.MPC_autotune(active_extruder, Temperature::MPCTuningType::AUTO); });
      #endif
      #if ENABLED(MPC_EDIT_MENU)
        EDIT_ITEM(ICON_MPCHeater, MSG_MPC_POWER, onDrawPFloatMenu, setHeaterPower, &mpc.heater_power);
        EDIT_ITEM(ICON_MPCHeatCap, MSG_MPC_BLOCK_HEAT_CAPACITY, onDrawPFloat2Menu, setBlkHeatCapacity, &mpc.block_heat_capacity);
        EDIT_ITEM(ICON_MPCValue, MSG_SENSOR_RESPONSIVENESS, onDrawPFloat4Menu, setSensorResponse, &mpc.sensor_responsiveness);
        EDIT_ITEM(ICON_MPCValue, MSG_MPC_AMBIENT_XFER_COEFF, onDrawPFloat4Menu, setAmbientXfer, &mpc.ambient_xfer_coeff_fan0);
        #if ENABLED(MPC_INCLUDE_FAN)
          EDIT_ITEM(ICON_MPCFan, MSG_MPC_AMBIENT_XFER_COEFF_FAN, onDrawFanAdj, setFanAdj, &mpc.fan255_adjustment);
        #endif
      #endif
    }
    updateMenu(hotendMPCMenu);
  }

#endif // MPC_EDIT_MENU || MPC_AUTOTUNE_MENU

#if HAS_PID_HEATING

  #if ENABLED(PID_AUTOTUNE_MENU)
    void setPID(celsius_t t, heater_id_t h) {
      gcode.process_subcommands_now(
        MString<60>(F("G28OXY\nG0Z5F300\nG0X"), X_CENTER, F("Y"), Y_CENTER, F("F5000\nM84\nM400"))
      );
      thermalManager.PID_autotune(t, h, hmiData.pidCycles, true);
    }
    void setPIDCycles() { setPIntOnClick(3, 50); }
  #endif

  #if ENABLED(PID_EDIT_MENU)
    void setKp() { setPFloatOnClick(0, 1000, 2); }
    void applyPIDi() {
      *menuData.floatPtr = scalePID_i(menuData.value / POW(10, 2));
      TERN_(PIDTEMP, thermalManager.updatePID());
    }
    void applyPIDd() {
      *menuData.floatPtr = scalePID_d(menuData.value / POW(10, 2));
      TERN_(PIDTEMP, thermalManager.updatePID());
    }
    void setKi() {
      menuData.floatPtr = (float*)static_cast<MenuItemPtr*>(currentMenu->selectedItem())->value;
      const float value = unscalePID_i(*menuData.floatPtr);
      setFloatOnClick(0, 1000, 2, value, applyPIDi);
    }
    void setKd() {
      menuData.floatPtr = (float*)static_cast<MenuItemPtr*>(currentMenu->selectedItem())->value;
      const float value = unscalePID_d(*menuData.floatPtr);
      setFloatOnClick(0, 1000, 2, value, applyPIDd);
    }
    void onDrawPIDi(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, unscalePID_i(*(float*)static_cast<MenuItemPtr*>(menuitem)->value)); }
    void onDrawPIDd(MenuItem* menuitem, int8_t line) { onDrawFloatMenu(menuitem, line, 2, unscalePID_d(*(float*)static_cast<MenuItemPtr*>(menuitem)->value)); }
  #endif // PID_EDIT_MENU

#endif // HAS_PID_HEATING

#if ANY(PID_AUTOTUNE_MENU, PID_EDIT_MENU)

  #if ENABLED(PIDTEMP)

    #if ENABLED(PID_AUTOTUNE_MENU)
      void hotendPID() { setPID(hmiData.hotendPIDT, H_E0); }
      void setHotendPIDT() { setPIntOnClick(MIN_ETEMP, MAX_ETEMP); }
    #endif

    void drawHotendPIDMenu() {
      checkkey = ID_Menu;
      if (SET_MENU_F(hotendPIDMenu, STR_HOTEND_PID " Settings", 8)) {
        BACK_ITEM(drawAdvancedSettingsMenu);
        #if ENABLED(PID_AUTOTUNE_MENU)
          MENU_ITEM_F(ICON_PIDNozzle, STR_HOTEND_PID, onDrawMenuItem, hotendPID);
          EDIT_ITEM(ICON_Temperature, MSG_TEMPERATURE, onDrawPIntMenu, setHotendPIDT, &hmiData.hotendPIDT);
          EDIT_ITEM(ICON_PIDCycles, MSG_PID_CYCLE, onDrawPIntMenu, setPIDCycles, &hmiData.pidCycles);
        #endif
        #if ENABLED(PID_EDIT_MENU)
          EDIT_ITEM_F(ICON_PIDValue, "Set Kp: ", onDrawPFloat2Menu, setKp, &thermalManager.temp_hotend[0].pid.Kp);
          EDIT_ITEM_F(ICON_PIDValue, "Set Ki: ", onDrawPIDi, setKi, &thermalManager.temp_hotend[0].pid.Ki);
          EDIT_ITEM_F(ICON_PIDValue, "Set Kd: ", onDrawPIDd, setKd, &thermalManager.temp_hotend[0].pid.Kd);
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawMenuItem, writeEEPROM);
        #endif
      }
      updateMenu(hotendPIDMenu);
    }

  #endif // PIDTEMP

  #if ENABLED(PIDTEMPBED)

    #if ENABLED(PID_AUTOTUNE_MENU)
      void bedPID() { setPID(hmiData.bedPIDT, H_BED); }
      void setBedPIDT() { setPIntOnClick(MIN_BEDTEMP, MAX_BEDTEMP); }
    #endif

    void drawBedPIDMenu() {
      checkkey = ID_Menu;
      if (SET_MENU_F(bedPIDMenu, STR_BED_PID " Settings", 8)) {
        BACK_ITEM(drawAdvancedSettingsMenu);
        #if ENABLED(PID_AUTOTUNE_MENU)
          MENU_ITEM_F(ICON_PIDBed, STR_BED_PID, onDrawMenuItem,bedPID);
          EDIT_ITEM(ICON_Temperature, MSG_TEMPERATURE, onDrawPIntMenu, setBedPIDT, &hmiData.bedPIDT);
          EDIT_ITEM(ICON_PIDCycles, MSG_PID_CYCLE, onDrawPIntMenu, setPIDCycles, &hmiData.pidCycles);
        #endif
        #if ENABLED(PID_EDIT_MENU)
          EDIT_ITEM_F(ICON_PIDValue, "Set Kp: ", onDrawPFloat2Menu, setKp, &thermalManager.temp_bed.pid.Kp);
          EDIT_ITEM_F(ICON_PIDValue, "Set Ki: ", onDrawPIDi, setKi, &thermalManager.temp_bed.pid.Ki);
          EDIT_ITEM_F(ICON_PIDValue, "Set Kd: ", onDrawPIDd, setKd, &thermalManager.temp_bed.pid.Kd);
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawMenuItem, writeEEPROM);
        #endif
      }
      updateMenu(bedPIDMenu);
    }

  #endif // PIDTEMPBED

  #if ENABLED(PIDTEMPCHAMBER)

    #if ENABLED(PID_AUTOTUNE_MENU)
      void chamberPID() { setPID(hmiData.chamberPIDT, H_CHAMBER); }
      void setChamberPIDT() { setPIntOnClick(MIN_CHAMBERTEMP, MAX_CHAMBERTEMP); }
    #endif

    void drawChamberPIDMenu() {
      checkkey = ID_Menu;
      if (SET_MENU_F(chamberPIDMenu, STR_CHAMBER_PID " Settings", 8)) {
        BACK_ITEM(drawAdvancedSettingsMenu);
        #if ENABLED(PID_AUTOTUNE_MENU)
          MENU_ITEM_F(ICON_PIDChamber, STR_CHAMBER_PID, onDrawMenuItem,chamberPID);
          EDIT_ITEM(ICON_Temperature, MSG_TEMPERATURE, onDrawPIntMenu, setChamberPIDT, &hmiData.chamberPIDT);
          EDIT_ITEM(ICON_PIDCycles, MSG_PID_CYCLE, onDrawPIntMenu, setPIDCycles, &hmiData.pidCycles);
        #endif
        #if ENABLED(PID_EDIT_MENU)
          EDIT_ITEM_F(ICON_PIDValue, "Set Kp: ", onDrawPFloat2Menu, setKp, &thermalManager.temp_chamber.pid.Kp);
          EDIT_ITEM_F(ICON_PIDValue, "Set Ki: ", onDrawPIDi, setKi, &thermalManager.temp_chamber.pid.Ki);
          EDIT_ITEM_F(ICON_PIDValue, "Set Kd: ", onDrawPIDd, setKd, &thermalManager.temp_chamber.pid.Kd);
        #endif
        #if ENABLED(EEPROM_SETTINGS)
          MENU_ITEM(ICON_WriteEEPROM, MSG_STORE_EEPROM, onDrawMenuItem, writeEEPROM);
        #endif
      }
      updateMenu(chamberPIDMenu);
    }

  #endif // PIDTEMPCHAMBER

#endif // PID_AUTOTUNE_MENU || PID_EDIT_MENU

//=============================================================================

#if HAS_BED_PROBE

  void drawZOffsetWizMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(zOffsetWizMenu, MSG_PROBE_WIZARD, 4)) {
      BACK_ITEM(drawPrepareMenu);
      MENU_ITEM(ICON_Homing, MSG_AUTO_HOME, onDrawMenuItem, autoHome);
      MENU_ITEM(ICON_AxisD, MSG_MOVE_NOZZLE_TO_BED, onDrawMenuItem, setMoveZto0);
      EDIT_ITEM(ICON_Zoffset, MSG_BABYSTEP_PROBE_Z, onDrawPFloat2Menu, setZOffset, &BABY_Z_VAR);
    }
    updateMenu(zOffsetWizMenu);
    if (!axis_is_trusted(Z_AXIS)) LCD_MESSAGE_F("WARNING: Z position unknown, move Z to home");
  }

#endif

#if ENABLED(INDIVIDUAL_AXIS_HOMING_SUBMENU)

  void drawHomingMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(homingMenu, MSG_HOMING, 6)) {
      BACK_ITEM(drawPrepareMenu);
      MENU_ITEM(ICON_Homing, MSG_AUTO_HOME, onDrawMenuItem, autoHome);
      #if HAS_X_AXIS
        MENU_ITEM(ICON_HomeX, MSG_AUTO_HOME_X, onDrawMenuItem, homeX);
      #endif
      #if HAS_Y_AXIS
        MENU_ITEM(ICON_HomeY, MSG_AUTO_HOME_Y, onDrawMenuItem, homeY);
      #endif
      #if HAS_Z_AXIS
        MENU_ITEM(ICON_HomeZ, MSG_AUTO_HOME_Z, onDrawMenuItem, homeZ);
      #endif
      #if ENABLED(Z_STEPPER_AUTO_ALIGN)
        MENU_ITEM(ICON_HomeZ, MSG_AUTO_Z_ALIGN, onDrawMenuItem, autoZAlign);
      #endif
      #if ENABLED(MESH_BED_LEVELING)
        EDIT_ITEM(ICON_ZAfterHome, MSG_Z_AFTER_HOME, onDrawPInt8Menu, setZAfterHoming, &hmiData.zAfterHoming);
      #endif
    }
    updateMenu(homingMenu);
  }

#endif // INDIVIDUAL_AXIS_HOMING_SUBMENU

#if ENABLED(FWRETRACT)

  void drawFWRetractMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(fwRetractMenu, MSG_FWRETRACT, 6)) {
      BACK_ITEM(returnFWRetractMenu);
      EDIT_ITEM(ICON_FWRetLength, MSG_CONTROL_RETRACT, onDrawPFloatMenu, setRetractLength, &fwretract.settings.retract_length);
      EDIT_ITEM(ICON_FWRetSpeed, MSG_SINGLENOZZLE_RETRACT_SPEED, onDrawPFloatMenu, setRetractSpeed, &fwretract.settings.retract_feedrate_mm_s);
      EDIT_ITEM(ICON_FWRetZRaise, MSG_CONTROL_RETRACT_ZHOP, onDrawPFloat2Menu, setZRaise, &fwretract.settings.retract_zraise);
      EDIT_ITEM(ICON_FWRecSpeed, MSG_SINGLENOZZLE_UNRETRACT_SPEED, onDrawPFloatMenu, setRecoverSpeed, &fwretract.settings.retract_recover_feedrate_mm_s);
      EDIT_ITEM(ICON_FWRecExtra, MSG_CONTROL_RETRACT_RECOVER, onDrawPFloatMenu, setAddRecover, &fwretract.settings.retract_recover_extra);
    }
    updateMenu(fwRetractMenu);
  }

#endif

//=============================================================================
// Mesh Bed Leveling
//=============================================================================

#if HAS_MESH

  void applyMeshFadeHeight() { set_z_fade_height(planner.z_fade_height); }
  void setMeshFadeHeight() { setPFloatOnClick(0, 100, 1, applyMeshFadeHeight); }

  void setMeshActive() {
    set_bed_leveling_enabled(!planner.leveling_active);
    drawCheckboxLine(currentMenu->line(), planner.leveling_active);
    dwinUpdateLCD();
  }

  #if ENABLED(PREHEAT_BEFORE_LEVELING)
    void setBedLevT() { setPIntOnClick(MIN_BEDTEMP, MAX_BEDTEMP); }
  #endif

  #if ENABLED(PROUI_MESH_EDIT)
    void liveEditMesh() { ((MenuItemPtr*)editZValueItem)->value = &bedlevel.z_values[hmiValue.select ? bedLevelTools.mesh_x : menuData.value][hmiValue.select ? menuData.value : bedLevelTools.mesh_y]; editZValueItem->redraw(); }
    void applyEditMeshX() { bedLevelTools.mesh_x = menuData.value; }
    void applyEditMeshY() { bedLevelTools.mesh_y = menuData.value; }
    void resetMesh() { bedLevelTools.meshReset(); LCD_MESSAGE(MSG_MESH_RESET); }
    void setEditMeshX() { hmiValue.select = 0; setIntOnClick(0, GRID_MAX_POINTS_X - 1, bedLevelTools.mesh_x, applyEditMeshX, liveEditMesh); }
    void setEditMeshY() { hmiValue.select = 1; setIntOnClick(0, GRID_MAX_POINTS_Y - 1, bedLevelTools.mesh_y, applyEditMeshY, liveEditMesh); }
    void setEditZValue() { setPFloatOnClick(Z_OFFSET_MIN, Z_OFFSET_MAX, 3); }
  #endif

#endif // HAS_MESH

#if ENABLED(AUTO_BED_LEVELING_UBL)

  void applyUBLSlot() { bedlevel.storage_slot = menuData.value; }
  void setUBLSlot() { setIntOnClick(0, settings.calc_num_meshes() - 1, bedlevel.storage_slot, applyUBLSlot); }
  void onDrawUBLSlot(MenuItem* menuitem, int8_t line) {
    NOLESS(bedlevel.storage_slot, 0);
    onDrawIntMenu(menuitem, line, bedlevel.storage_slot);
  }

  void applyUBLTiltGrid() { bedLevelTools.tilt_grid = menuData.value; }
  void setUBLTiltGrid() { setIntOnClick(1, 3, bedLevelTools.tilt_grid, applyUBLTiltGrid); }

  void ublMeshTilt() {
    NOLESS(bedlevel.storage_slot, 0);
    if (bedLevelTools.tilt_grid > 1)
      gcode.process_subcommands_now(TS(F("G29J"), bedLevelTools.tilt_grid));
    else
      gcode.process_subcommands_now(F("G29J"));
    LCD_MESSAGE(MSG_UBL_MESH_TILTED);
  }

  void ublSmartFillMesh() {
    for (uint8_t x = 0; x < GRID_MAX_POINTS_Y; ++x) bedlevel.smart_fill_mesh();
    LCD_MESSAGE(MSG_UBL_MESH_FILLED);
  }

  void ublMeshSave() {
    NOLESS(bedlevel.storage_slot, 0);
    settings.store_mesh(bedlevel.storage_slot);
    ui.status_printf(0, GET_TEXT_F(MSG_MESH_SAVED), bedlevel.storage_slot);
    DONE_BUZZ(true);
  }

  void ublMeshLoad() {
    NOLESS(bedlevel.storage_slot, 0);
    settings.load_mesh(bedlevel.storage_slot);
  }

#endif // AUTO_BED_LEVELING_UBL

#if HAS_MESH

  void drawMeshSetMenu() {
    checkkey = ID_Menu;
    if (SET_MENU(meshMenu, MSG_MESH_LEVELING, 14)) {
      BACK_ITEM(drawAdvancedSettingsMenu);
      #if ENABLED(PREHEAT_BEFORE_LEVELING)
        EDIT_ITEM(ICON_Temperature, MSG_UBL_SET_TEMP_BED, onDrawPIntMenu, setBedLevT, &hmiData.bedLevT);
      #endif
      EDIT_ITEM(ICON_SetZOffset, MSG_Z_FADE_HEIGHT, onDrawPFloatMenu, setMeshFadeHeight, &planner.z_fade_height);
      EDIT_ITEM(ICON_UBLActive, MSG_ACTIVATE_MESH, onDrawChkbMenu, setMeshActive, &planner.leveling_active);
      #if HAS_BED_PROBE
        MENU_ITEM(ICON_Level, MSG_AUTO_MESH, onDrawMenuItem, autoLevel);
      #endif
      #if ENABLED(AUTO_BED_LEVELING_UBL)
        EDIT_ITEM(ICON_UBLSlot, MSG_UBL_STORAGE_SLOT, onDrawUBLSlot, setUBLSlot, &bedlevel.storage_slot);
        MENU_ITEM(ICON_UBLMeshSave, MSG_UBL_SAVE_MESH, onDrawMenuItem, ublMeshSave);
        MENU_ITEM(ICON_UBLMeshLoad, MSG_UBL_LOAD_MESH, onDrawMenuItem, ublMeshLoad);
        EDIT_ITEM(ICON_UBLTiltGrid, MSG_UBL_TILTING_GRID, onDrawPInt8Menu, setUBLTiltGrid, &bedLevelTools.tilt_grid);
        MENU_ITEM(ICON_UBLTiltGrid, MSG_UBL_TILT_MESH, onDrawMenuItem, ublMeshTilt);
        MENU_ITEM(ICON_UBLSmartFill, MSG_UBL_SMART_FILLIN, onDrawMenuItem, ublSmartFillMesh);
      #endif
      #if ENABLED(PROUI_MESH_EDIT)
        MENU_ITEM(ICON_MeshReset, MSG_MESH_RESET, onDrawMenuItem, resetMesh);
        MENU_ITEM(ICON_MeshEdit, MSG_EDIT_MESH, onDrawSubMenu, drawEditMeshMenu);
      #endif
      MENU_ITEM(ICON_MeshViewer, MSG_MESH_VIEW, onDrawSubMenu, dwinMeshViewer);
    }
    updateMenu(meshMenu);
  }

  #if ENABLED(PROUI_MESH_EDIT)
    void drawEditMeshMenu() {
      if (!leveling_is_valid()) { LCD_MESSAGE(MSG_UBL_MESH_INVALID); return; }
      set_bed_leveling_enabled(false);
      checkkey = ID_Menu;
      if (SET_MENU(editMeshMenu, MSG_EDIT_MESH, 4)) {
        bedLevelTools.mesh_x = bedLevelTools.mesh_y = 0;
        BACK_ITEM(drawMeshSetMenu);
        EDIT_ITEM(ICON_MeshEditX, MSG_MESH_X, onDrawPInt8Menu, setEditMeshX, &bedLevelTools.mesh_x);
        EDIT_ITEM(ICON_MeshEditY, MSG_MESH_Y, onDrawPInt8Menu, setEditMeshY, &bedLevelTools.mesh_y);
        editZValueItem = EDIT_ITEM(ICON_MeshEditZ, MSG_MESH_EDIT_Z, onDrawPFloat2Menu, setEditZValue, &bedlevel.z_values[bedLevelTools.mesh_x][bedLevelTools.mesh_y]);
      }
      updateMenu(editMeshMenu);
    }
  #endif

#endif // HAS_MESH

#endif // DWIN_LCD_PROUI
