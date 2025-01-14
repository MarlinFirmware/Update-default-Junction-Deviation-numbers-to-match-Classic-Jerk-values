/*********************
 * status_screen.cpp *
 *********************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *   Written By Brian Kahl      2023 - FAME3D.                              *
 *                                                                          *
 *   This program is free software: you can redistribute it and/or modify   *
 *   it under the terms of the GNU General Public License as published by   *
 *   the Free Software Foundation, either version 3 of the License, or      *
 *   (at your option) any later version.                                    *
 *                                                                          *
 *   This program is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the          *
 *   GNU General Public License for more details.                           *
 *                                                                          *
 *   To view a copy of the GNU General Public License, go to the following  *
 *   location: <https://www.gnu.org/licenses/>.                             *
 ****************************************************************************/

#include "../config.h"
#include "../screens.h"
#include "../screen_data.h"

#ifdef FTDI_STATUS_SCREEN

#include "../archim2-flash/flash_storage.h"

using namespace FTDI;
using namespace Theme;
using namespace ExtUI;

#define GRID_COLS 9
#define GRID_ROWS 16

void StatusScreen::draw_axis_position(draw_mode_t what) {
  CommandProcessor cmd;

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define X_LBL_POS             BTN_POS(1,7), BTN_SIZE(2,2)
    #define Y_LBL_POS             BTN_POS(4,7), BTN_SIZE(2,2)
    #define Z_LBL_POS             BTN_POS(7,7), BTN_SIZE(2,2)
    #define X_VAL_POS             BTN_POS(2,7), BTN_SIZE(2,2)
    #define Y_VAL_POS             BTN_POS(5,7), BTN_SIZE(2,2)
    #define Z_VAL_POS             BTN_POS(8,7), BTN_SIZE(2,2)
    #define ALL_VAL_POS           BTN_POS(1,7), BTN_SIZE(9,2)
  #else
    #define X_LBL_POS   BTN_POS(1, 9), BTN_SIZE(1, 2)
    #define Y_LBL_POS   BTN_POS(2, 9), BTN_SIZE(1, 2)
    #define Z_LBL_POS   BTN_POS(3, 9), BTN_SIZE(1, 2)
    #define X_VAL_POS   BTN_POS(1,11), BTN_SIZE(1, 2)
    #define Y_VAL_POS   BTN_POS(2,11), BTN_SIZE(1, 2)
    #define Z_VAL_POS   BTN_POS(3,11), BTN_SIZE(1, 2)
    #define ALL_VAL_POS BTN_POS(1, 9), BTN_SIZE(3,11)
  #endif

  #define _UNION_POS(x1,y1,w1,h1,x2,y2,w2,h2) x1,y1,max(x1+w1,x2+w2)-x1,max(y1+h1,y2+h2)-y1
  #define UNION_POS(p1, p2) _UNION_POS(p1, p2)

  if (what & BACKGROUND) {
    cmd.tag(0)
       .colors(normal_btn)
       .font(Theme::font_medium)
       .text  ( X_LBL_POS, GET_TEXT_F(MSG_AXIS_X))
       .text  ( Y_LBL_POS, GET_TEXT_F(MSG_AXIS_Y))
       .text  ( Z_LBL_POS, GET_TEXT_F(MSG_AXIS_Z));

    if (!ExtUI::isOngoingPrintJob()) {
    cmd.tag(6)
        .button(X_VAL_POS, F(""), OPT_FLAT)
        .button(Y_VAL_POS, F(""), OPT_FLAT)
        .button(Z_VAL_POS, F(""), OPT_FLAT)
        .button(ALL_VAL_POS, F(""));
    }
  }

  if (what & FOREGROUND) {
    using namespace ExtUI;
    char x_str[15];
    char y_str[15];
    char z_str[15];

    if (isAxisPositionKnown(X))
      format_position(x_str, getAxisPosition_mm(X), 0);
    else
      strcpy_P(x_str, PSTR("?"));

    if (isAxisPositionKnown(Y))
      format_position(y_str, getAxisPosition_mm(Y), 0);
    else
      strcpy_P(y_str, PSTR("?"));

    if (isAxisPositionKnown(Z))
      format_position(z_str, getAxisPosition_mm(Z), 1);
    else
      strcpy_P(z_str, PSTR("?"));

    if (!ExtUI::isOngoingPrintJob()) {
      cmd.tag(6)
         .colors(normal_text)
         .font(Theme::font_medium)
         .text(X_VAL_POS, x_str)
         .text(Y_VAL_POS, y_str)
         .text(Z_VAL_POS, z_str)
         .text(X_LBL_POS, GET_TEXT_F(MSG_AXIS_X))
         .text(Y_LBL_POS, GET_TEXT_F(MSG_AXIS_Y))
         .text(Z_LBL_POS, GET_TEXT_F(MSG_AXIS_Z));
    }
    else {
      cmd.tag(0)
         .colors(temp_btn)
         .font(Theme::font_medium)
         .text(X_VAL_POS, x_str)
         .text(Y_VAL_POS, y_str)
         .text(Z_VAL_POS, z_str)
         .text(X_LBL_POS, GET_TEXT_F(MSG_AXIS_X))
         .text(Y_LBL_POS, GET_TEXT_F(MSG_AXIS_Y))
         .text(Z_LBL_POS, GET_TEXT_F(MSG_AXIS_Z));
    }
  }
}

#undef GRID_COLS
#define GRID_COLS 6

void StatusScreen::draw_temperature(draw_mode_t what) {
  using namespace Theme;

  #define BACKGROUND_POS      BTN_POS(1,1),  BTN_SIZE(6,4)
  #define TEMP_RECT_E0        BTN_POS(1,1),  BTN_SIZE(3,2)
  #define TEMP_RECT_E1        BTN_POS(4,1),  BTN_SIZE(3,2)
  #define TEMP_RECT_BED       BTN_POS(1,3),  BTN_SIZE(3,2)
  #define NOZ_1_POS           BTN_POS(1,1),  BTN_SIZE(3,2)
  #define NOZ_2_POS           BTN_POS(4,1),  BTN_SIZE(3,2)
  #define BED_POS             BTN_POS(1,3),  BTN_SIZE(3,2)
  #define FAN_POS             BTN_POS(4,3),  BTN_SIZE(3,2)
  #define ALL_TEMP_POS        BTN_POS(1,1),  BTN_SIZE(6,4)

  #define _ICON_POS(x,y,w,h) x, y, w/3, h
  #define _TEXT_POS(x,y,w,h) x + w/3, y, w - w/3, h
  #define ICON_POS(pos) _ICON_POS(pos)
  #define TEXT_POS(pos) _TEXT_POS(pos)

  CommandProcessor cmd;

  if (what & BACKGROUND) {

    // Draw Extruder Bitmap on Extruder Temperature Button
    cmd.font(Theme::font_small).tag(5)
       .cmd (BITMAP_SOURCE(Extruder_Icon_Info))
       .cmd (BITMAP_LAYOUT(Extruder_Icon_Info))
       .cmd (BITMAP_SIZE  (Extruder_Icon_Info))
       .icon(ICON_POS(NOZ_1_POS), Extruder_Icon_Info, icon_scale)
       .icon(ICON_POS(NOZ_2_POS), Extruder_Icon_Info, icon_scale);

    // Draw Bed Heat Bitmap on Bed Heat Button
    cmd.cmd (BITMAP_SOURCE(Bed_Heat_Icon_Info))
       .cmd (BITMAP_LAYOUT(Bed_Heat_Icon_Info))
       .cmd (BITMAP_SIZE  (Bed_Heat_Icon_Info))
       .icon(ICON_POS(BED_POS), Bed_Heat_Icon_Info, icon_scale);

    // Draw Fan Percent Bitmap on Bed Heat Button
    cmd.cmd (BITMAP_SOURCE(Fan_Icon_Info))
       .cmd (BITMAP_LAYOUT(Fan_Icon_Info))
       .cmd (BITMAP_SIZE  (Fan_Icon_Info))
       .icon(ICON_POS(FAN_POS), Fan_Icon_Info, icon_scale);

    TERN_(TOUCH_UI_USE_UTF8, load_utf8_bitmaps(cmd)); // Restore font bitmap handles
  }

  if (what & FOREGROUND) {
    using namespace ExtUI;
    char e0_str[20], e1_str[20], bed_str[20], fan_str[20];

    sprintf_P(fan_str, PSTR("%-3d %%"), int8_t(getActualFan_percent(FAN0)));

    if (isHeaterIdle(BED))
      format_temp_and_idle(bed_str, getActualTemp_celsius(BED));
    else
      format_temp_and_temp(bed_str, getActualTemp_celsius(BED), getTargetTemp_celsius(BED));

    if (isHeaterIdle(H0))
      format_temp_and_idle(e0_str, getActualTemp_celsius(H0));
    else
      format_temp_and_temp(e0_str, getActualTemp_celsius(H0), getTargetTemp_celsius(H0));

    #if HAS_MULTI_EXTRUDER
      if (isHeaterIdle(H1))
        format_temp_and_idle(e1_str, getActualTemp_celsius(H1));
      else
        format_temp_and_temp(e1_str, getActualTemp_celsius(H1), getTargetTemp_celsius(H1));
    #else
      strcpy_P(e1_str, PSTR("N/A"));
    #endif

    if (getTargetTemp_celsius(H0) > 0) {
      cmd.fgcolor(temp_button);
    }
    else {
      cmd.colors(normal_btn);
    }
    cmd.tag(5).font(font_medium).button(TEXT_POS(NOZ_1_POS), e0_str);

    if (getTargetTemp_celsius(BED) > 0) {
      cmd.fgcolor(temp_button);
    }
    else {
      cmd.colors(normal_btn);
    }
    cmd.tag(5).font(font_medium).button(TEXT_POS(BED_POS), bed_str);

    if (getActualFan_percent(FAN0) > 0) {
      cmd.fgcolor(temp_button);
    }
    else {
      cmd.colors(normal_btn);
    }
    cmd.tag(5).font(font_medium).button(TEXT_POS(FAN_POS), fan_str);

    if DISABLED(HAS_MULTI_HOTEND) {
      cmd.font(font_xsmall).fgcolor(gray_color_1);
    }
    else if (getTargetTemp_celsius(H1) > 0) {
      cmd.font(font_medium).fgcolor(temp_button);
    }
    else {
      cmd.font(font_medium).colors(normal_btn);
    }
    cmd.tag(5).button(TEXT_POS(NOZ_2_POS), e1_str);
  }
}

void StatusScreen::_format_time(char *outstr, uint32_t time) {
  const uint8_t hrs = time / 3600,
                min = (time / 60) % 60,
                sec = time % 60;
  if (hrs)
    sprintf_P(outstr, PSTR("%02d:%02d"), hrs, min);
  else
    sprintf_P(outstr, PSTR("%02d:%02ds"), min, sec);
}

void StatusScreen::draw_progress(draw_mode_t what) {
  using namespace ExtUI;
  using namespace Theme;

  CommandProcessor cmd;

 #undef GRID_COLS

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define GRID_COLS 2
    #define PROGRESSZONE_POS   BTN_POS(1,9), BTN_SIZE(2,2)
    #define PROGRESSZONE_POS_1 BTN_POS(1,9), BTN_SIZE(1,2)
    #define PROGRESSZONE_POS_2 BTN_POS(2,9), BTN_SIZE(1,2)
    #define TIME_POS_X         BTN_X(1)
    #define TIME_POS_W         BTN_W(1)
    //#define REMAINING_POS_X  BTN_X(2)
    //#define REMAINING_POS_W  BTN_W(1)
    #define PROGRESS_POS_X     BTN_X(2)
    #define PROGRESS_POS_W     BTN_W(1)
    #define PROGRESSZONE_FIRSTLINE_Y BTN_Y(9)
    #define PROGRESSBAR_POS    BTN_POS(1,8), BTN_SIZE(2,2)
    #define CLEAR_PROGRESS_POS BTN_POS(0,6.75), BTN_SIZE(4,5)
  #else
    #define GRID_COLS 6
    #define PROGRESSZONE_POS BTN_POS(5,1), BTN_SIZE(2,4)
    #define PROGRESSZONE_POS_1 BTN_POS(1,9), BTN_SIZE(1,2)
    #define PROGRESSZONE_POS_2 BTN_POS(2,9), BTN_SIZE(1,2)
    #define CLEAR_PROGRESS_POS BTN_POS(0,6.75), BTN_SIZE(4,5)
    #if ENABLED(SHOW_REMAINING_TIME)
      #define TIME_POS       BTN_POS(5,1), BTN_SIZE(1,2)
      #define REMAINING_POS  BTN_POS(6,1), BTN_SIZE(1,2)
    #else
      #define TIME_POS       BTN_POS(5,1), BTN_SIZE(2,2)
    #endif
    #define PROGRESS_POS     BTN_POS(5,3), BTN_SIZE(2,2)
    #define PROGRESSBAR_POS  BTN_POS(5,2), BTN_SIZE(2,2)
  #endif

  if (ExtUI::isOngoingPrintJob()|| ExtUI::isPrintingPaused()) {

    if (what & FOREGROUND) {
      cmd.colors(temp_btn)
       .tag(0).button(CLEAR_PROGRESS_POS, F(""), OPT_FLAT);

      const uint32_t elapsed = getProgress_seconds_elapsed();
      char elapsed_str[10];
      _format_time(elapsed_str, elapsed);

      #if ENABLED(SHOW_REMAINING_TIME)
        const uint32_t remaining = getProgress_seconds_remaining();
        char remaining_str[10];
        _format_time(remaining_str, remaining);
      #endif

      const uint16_t current_progress = TERN(HAS_PRINT_PROGRESS_PERMYRIAD, getProgress_permyriad(), getProgress_percent() * 100);
      constexpr uint16_t progress_range = 10000U;

      const bool show_progress_bar = current_progress < progress_range + 1;
      if (show_progress_bar) {
        cmd.tag(0).font(font_medium)
          .bgcolor(progress)
          .progress(PROGRESSBAR_POS, current_progress, progress_range, OPT_FLAT);
      }

      char progress_str[10];
      sprintf_P(progress_str,
        #if ENABLED(PRINT_PROGRESS_SHOW_DECIMALS)
          PSTR("%3d.%02d%%"), uint8_t(current_progress / 100), current_progress % 100
        #else
          PSTR("%3d%%"), uint8_t(current_progress / 100)
        #endif
      );

      #if ENABLED(TOUCH_UI_PORTRAIT)
        const uint16_t texts_pos_h = show_progress_bar ? (BTN_H(2)) : (BTN_H(2));
        cmd.font(font_medium)
          .tag(0).text(TIME_POS_X, PROGRESSZONE_FIRSTLINE_Y, TIME_POS_W, texts_pos_h, elapsed_str)
          #if ENABLED(SHOW_REMAINING_TIME)
            .text(REMAINING_POS_X, PROGRESSZONE_FIRSTLINE_Y, REMAINING_POS_W, texts_pos_h, remaining_str)
          #endif
          .text(PROGRESS_POS_X, PROGRESSZONE_FIRSTLINE_Y, PROGRESS_POS_W, texts_pos_h, progress_str);
      #else
        cmd.font(font_medium)
          .tag(0).text(TIME_POS, elapsed_str)
          #if ENABLED(SHOW_REMAINING_TIME)
            .text(REMAINING_POS, remaining_str)
          #endif
          .text(PROGRESS_POS, progress_str);
      #endif
    }
  }
  else {
    cmd.colors(normal_btn)
       .font(font_medium)
       .tag(9).button(PROGRESSZONE_POS_1, GET_TEXT_F(MSG_HOME_ALL))
       .tag(16).button(PROGRESSZONE_POS_2, GET_TEXT_F(MSG_PRESENT_BED));
  }
}

void StatusScreen::draw_interaction_buttons(draw_mode_t what) {
  #undef GRID_COLS
  #define GRID_COLS 2
  if (what & FOREGROUND) {
    using namespace ExtUI;

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define TOOL_HEAD_POS         BTN_POS(1,11), BTN_SIZE(1,2)
    #define CHANGE_FILAMENT_POS   BTN_POS(2,11), BTN_SIZE(1,2)
    #define PREHEAT_POS           BTN_POS(1,13), BTN_SIZE(1,2)
    #define COOLDOWN_OFFSET_POS   BTN_POS(2,13), BTN_SIZE(1,2)
    #define MEDIA_BTN_POS         BTN_POS(1,15), BTN_SIZE(1,2)
    #define MENU_BTN_POS          BTN_POS(2,15), BTN_SIZE(1,2)
  #else
    #define TOOL_HEAD_POS         BTN_POS(1,11), BTN_SIZE(1,2)
    #define CHANGE_FILAMENT_POS   BTN_POS(2,11), BTN_SIZE(1,2)
    #define PREHEAT_POS           BTN_POS(1,13), BTN_SIZE(1,2)
    #define COOLDOWN_OFFSET_POS   BTN_POS(2,13), BTN_SIZE(1,2)
    #define MEDIA_BTN_POS         BTN_POS(1,13), BTN_SIZE(1,4)
    #define MENU_BTN_POS          BTN_POS(2,13), BTN_SIZE(1,4)
  #endif

    const bool has_media = isMediaMounted() && !isPrintingFromMedia();

    CommandProcessor cmd;
    if (ExtUI::isOngoingPrintJob() || ExtUI::isPrintingPaused()) {
      cmd.colors(normal_btn)
          .font(font_medium)
          .tag(!ExtUI::isPrintingPaused() ? 17 : 18)
          .button(TOOL_HEAD_POS, !ExtUI::isPrintingPaused() ? GET_TEXT_F(MSG_BUTTON_PAUSE) : GET_TEXT_F(MSG_BUTTON_RESUME))
          .tag(!ExtUI::isPrintingPaused() ? 7 : 14)
          .button(CHANGE_FILAMENT_POS, !ExtUI::isPrintingPaused() ? GET_TEXT_F(MSG_SPEED) : F(""));
      #if HAS_FILAMENT_SENSOR
        cmd.tag(8).button(PREHEAT_POS, GET_TEXT_F(MSG_SENSOR));
      #endif
      if (ExtUI::isPrintingPaused())
        draw_text_box(cmd, CHANGE_FILAMENT_POS, F("Change\nFilament"), OPT_CENTER, font_medium);
    }
    else {
      cmd.colors(normal_btn)
          .font(font_medium)
          .tag(14).button(CHANGE_FILAMENT_POS, F(""));
          draw_text_box(cmd, CHANGE_FILAMENT_POS, F("Change\nFilament"), OPT_CENTER, font_medium);
      cmd.colors(normal_btn)
          .font(font_medium)
          .tag(20).button(PREHEAT_POS, GET_TEXT_F(MSG_PREHEAT));
      cmd.enabled(ENABLED(CUSTOM_MENU_MAIN)).tag(10).button(TOOL_HEAD_POS, F(""));
      draw_text_box(cmd, TOOL_HEAD_POS, F("" CUSTOM_MENU_MAIN_TITLE "\n "), OPT_CENTER, font_medium);
    }

    cmd.colors(normal_btn)
       .font(Theme::font_medium)
       .tag(15).button(COOLDOWN_OFFSET_POS, isOngoingPrintJob() ? GET_TEXT_F(MSG_ZOFFSET) : GET_TEXT_F(MSG_COOLDOWN))
       .colors(has_media ? action_btn : normal_btn)
       .enabled(has_media || isOngoingPrintJob())
       .tag(isOngoingPrintJob() ? 19 : 3).button(MEDIA_BTN_POS, isOngoingPrintJob() ? GET_TEXT_F(MSG_BUTTON_CANCEL) : GET_TEXT_F(MSG_BUTTON_PRINT))
       .colors(!has_media ? action_btn : normal_btn)
       .tag(4).button(MENU_BTN_POS, GET_TEXT_F(MSG_BUTTON_MENU));
  }
}

void StatusScreen::draw_status_message(draw_mode_t what, const char *message) {
  #undef  GRID_COLS
  #define GRID_COLS 1

  #if ENABLED(TOUCH_UI_PORTRAIT)
    #define STATUS_POS  BTN_POS(1,5), BTN_SIZE(1,2)
  #else
    #define STATUS_POS  BTN_POS(1,5), BTN_SIZE(1,4)
  #endif

  if (what & BACKGROUND) {
    CommandProcessor cmd;
    cmd.colors(temp_btn)
       .tag(0)
       .button(STATUS_POS, F(""), OPT_FLAT);

    draw_text_box(cmd, STATUS_POS, message, OPT_CENTER, font_large);
  }
}

void StatusScreen::setStatusMessage(FSTR_P fmsg) {
  #ifdef __AVR__
    char buff[strlen_P(FTOP(fmsg)) + 1];
    strcpy_P(buff, FTOP(fmsg));
    setStatusMessage((const char *)buff);
  #else
    setStatusMessage(FTOP(fmsg));
  #endif
}

void StatusScreen::setStatusMessage(const char *message) {
  if (CommandProcessor::is_processing()) {
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("Cannot update status message, command processor busy");
    #endif
    return;
  }

  CommandProcessor cmd;
  cmd.cmd(CMD_DLSTART)
     .cmd(CLEAR_COLOR_RGB(Theme::bg_color))
     .cmd(CLEAR(true,true,true));

  draw_temperature(BACKGROUND);
  draw_status_message(BACKGROUND, message);
  draw_interaction_buttons(BACKGROUND);
  draw_progress(BACKGROUND);
  draw_axis_position(BACKGROUND);

  storeBackground();

  #if ENABLED(TOUCH_UI_DEBUG)
    SERIAL_ECHO_MSG("New status message: ", message);
  #endif

  if (AT_SCREEN(StatusScreen)) {
    current_screen.onRefresh();
  }
}

void StatusScreen::loadBitmaps() {
  // Load the bitmaps for the status screen
  using namespace Theme;
  constexpr uint32_t base = ftdi_memory_map::RAM_G;
  CLCD::mem_write_xbm(base + TD_Icon_Info.RAMG_offset,       TD_Icon,       sizeof(TD_Icon));
  CLCD::mem_write_xbm(base + Extruder_Icon_Info.RAMG_offset, Extruder_Icon, sizeof(Extruder_Icon));
  CLCD::mem_write_xbm(base + Bed_Heat_Icon_Info.RAMG_offset, Bed_Heat_Icon, sizeof(Bed_Heat_Icon));
  CLCD::mem_write_xbm(base + Fan_Icon_Info.RAMG_offset,      Fan_Icon,      sizeof(Fan_Icon));
  CLCD::mem_write_pgm(base + Home_icon_Info.RAMG_offset,     Home_icon,     sizeof(Home_icon));

  // Load fonts for internationalization
  #if ENABLED(TOUCH_UI_USE_UTF8)
    load_utf8_data(base + UTF8_FONT_OFFSET);
  #endif
}

void StatusScreen::onStartup() {
  UIFlashStorage::initialize();
}

void StatusScreen::onRedraw(draw_mode_t what) {
  if (what & FOREGROUND) {
    draw_temperature(FOREGROUND);
    draw_progress(FOREGROUND);
    draw_axis_position(FOREGROUND);
    draw_interaction_buttons(FOREGROUND);
  }
}

void StatusScreen::onEntry() {
  BaseScreen::onEntry();
  onRefresh();
}

void StatusScreen::onIdle() {
  if (refresh_timer.elapsed(STATUS_UPDATE_INTERVAL)) {
    onRefresh();
    refresh_timer.start();
  }
  BaseScreen::onIdle();
}

bool StatusScreen::onTouchEnd(uint8_t tag) {
  using namespace ExtUI;

  switch (tag) {
    #if HAS_MEDIA
      case 3: GOTO_SCREEN(FilesScreen); break;
    #endif
    case 4:
      if (isPrinting()) {
        GOTO_SCREEN(TuneMenu);
      }
      else {
        GOTO_SCREEN(MainMenu);
      }
      break;
    case 5:  GOTO_SCREEN(TemperatureScreen); break;
    case 6:
      if (isPrinting()) {
        #if ENABLED(BABYSTEPPING)
          GOTO_SCREEN(NudgeNozzleScreen);
        #elif HAS_BED_PROBE
          GOTO_SCREEN(ZOffsetScreen);
        #else
          return false;
        #endif
      }
      else {
        GOTO_SCREEN(MoveAxisScreen);
      }
      break;
    case 7:  GOTO_SCREEN(FeedratePercentScreen); break;
    #if HAS_FILAMENT_SENSOR
      case 8:  GOTO_SCREEN(FilamentRunoutScreen); break;
    #endif
    case 9:  injectCommands(F("G28")); break;
    #if ENABLED(CUSTOM_MENU_MAIN)
      case 10:  GOTO_SCREEN(CustomUserMenus); break;
    #endif
    if (!ExtUI::isOngoingPrintJob()) {
    case 11: injectCommands(F("G28X")); break;
    case 12: injectCommands(F("G28Y")); break;
    case 13: injectCommands(F("G28Z")); break;
    }
    case 14: GOTO_SCREEN(ChangeFilamentScreen);  break;
    #if ALL(HAS_LEVELING, HAS_BED_PROBE)
    case 15:
      if (ExtUI::isOngoingPrintJob()|| ExtUI::isPrintingPaused()) {
        #if EXTRUDERS > 1
          GOTO_SCREEN(NudgeNozzleScreen); break;
        #else
          GOTO_SCREEN(ZOffsetScreen); break;
        #endif
      }
      else {
        coolDown();
        TERN_(HAS_HEATED_CHAMBER, setTargetTemp_celsius(0, CHAMBER));
        GOTO_SCREEN(StatusScreen);
        break;
      }
    #endif
    #ifdef PRESENT_BED_GCODE
    case 16: injectCommands(F(PRESENT_BED_GCODE)); break;
    #endif
    case 17: injectCommands(F("M117 Print Paused")); pausePrint();  break;
    case 18: injectCommands(F("M117 Print Resumed")); resumePrint(); break;
    case 19:
      GOTO_SCREEN(ConfirmAbortPrintDialogBox);
      current_screen.forget();
      PUSH_SCREEN(StatusScreen);
      break;
    #ifdef PREHEAT_1_COMMAND
    case 20: injectCommands_P(PSTR(PREHEAT_1_COMMAND)); break;
    #endif
    default:
      return true;
  }
  // If a passcode is enabled, the LockScreen will prevent the
  // user from proceeding.
  LockScreen::check_passcode();
  return true;
}

void StatusScreen::onMediaMounted() {
  if (AT_SCREEN(StatusScreen))
    setStatusMessage(GET_TEXT_F(MSG_MEDIA_INSERTED));
}

void StatusScreen::onMediaRemoved() {
  if (AT_SCREEN(StatusScreen) || ExtUI::isPrintingFromMedia())
    setStatusMessage(GET_TEXT_F(MSG_MEDIA_REMOVED));
}

#endif // FTDI_STATUS_SCREEN
