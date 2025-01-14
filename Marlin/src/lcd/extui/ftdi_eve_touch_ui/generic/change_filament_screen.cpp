/******************************
 * change_filament_screen.cpp *
 ******************************/

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

#ifdef FTDI_CHANGE_FILAMENT_SCREEN

using namespace ExtUI;
using namespace FTDI;
using namespace Theme;

constexpr static ChangeFilamentScreenData &mydata = screen_data.ChangeFilamentScreen;

#ifdef TOUCH_UI_PORTRAIT
  #define GRID_COLS 4
  #define GRID_ROWS 10
  #define E0_TEMP_POS          BTN_POS(1,2),  BTN_SIZE(2,1)
  #define E1_TEMP_POS          BTN_POS(3,2),  BTN_SIZE(2,1)
  #define UNLD_LABL_POS        BTN_POS(3,7),  BTN_SIZE(2,2)
  #define LOAD_LABL_POS        BTN_POS(1,7),  BTN_SIZE(2,2)
  #define FILAMENT_SWAP_POS    BTN_POS(1,9),  BTN_SIZE(4,1)
  #define BACK_POS             BTN_POS(1,10), BTN_SIZE(4,1)
#else
  #define GRID_COLS 4
  #define GRID_ROWS 6
  #define E0_TEMP_POS          BTN_POS(2,2),  BTN_SIZE(1,1)
  #define E1_TEMP_POS          BTN_POS(3,2),  BTN_SIZE(1,1)
  #define FILAMENT_SWAP_POS    BTN_POS(1,9),  BTN_SIZE(4,1)
  #define UNLD_LABL_POS        BTN_POS(3,3),  BTN_SIZE(1,1)
  #define LOAD_LABL_POS        BTN_POS(4,3),  BTN_SIZE(1,1)
  #define UNLD_MOMN_POS        BTN_POS(3,4),  BTN_SIZE(1,1)
  #define LOAD_MOMN_POS        BTN_POS(4,4),  BTN_SIZE(1,1)
  #define UNLD_CONT_POS        BTN_POS(3,5),  BTN_SIZE(1,1)
  #define LOAD_CONT_POS        BTN_POS(4,5),  BTN_SIZE(1,1)
  #define BACK_POS             BTN_POS(3,6),  BTN_SIZE(2,1)
#endif
#define GROUP_1_POS            BTN_POS(1,3),  BTN_SIZE(4,1)
#define GROUP_1_LABL_POS       BTN_POS(1,3),  BTN_SIZE(3,1)
#define GROUP_1_TEMP_POS       BTN_POS(4,3),  BTN_SIZE(1,1)
#define GROUP_2_POS            BTN_POS(1,4),  BTN_SIZE(4,1)
#define GROUP_2_LABL_POS       BTN_POS(1,4),  BTN_SIZE(3,1)
#define GROUP_2_TEMP_POS       BTN_POS(4,4),  BTN_SIZE(1,1)
#define GROUP_3_POS            BTN_POS(1,5),  BTN_SIZE(4,1)
#define GROUP_3_LABL_POS       BTN_POS(1,5),  BTN_SIZE(3,1)
#define GROUP_3_TEMP_POS       BTN_POS(4,5),  BTN_SIZE(1,1)
#define GROUP_4_POS            BTN_POS(1,6),  BTN_SIZE(4,1)
#define GROUP_4_LABL_POS       BTN_POS(1,6),  BTN_SIZE(3,1)
#define GROUP_4_TEMP_POS       BTN_POS(4,6),  BTN_SIZE(1,1)
#define E1_SEL_POS             BTN_POS(1,1),  BTN_SIZE(2,1)
#define E2_SEL_POS             BTN_POS(3,1),  BTN_SIZE(2,1)

#define COOL_TEMP  40
#define LOW_TEMP  180
#define MED_TEMP  220
#define HIGH_TEMP 240
#define GROUP_1_TEMP  180
#define GROUP_2_TEMP  200
#define GROUP_3_TEMP  220
#define GROUP_4_TEMP  240

#define _ICON_POS(x,y,w,h) x, y, w/3, h
#define _TEXT_POS(x,y,w,h) x + w/3, y, w - w/3, h
#define ICON_POS(pos) _ICON_POS(pos)
#define TEXT_POS(pos) _TEXT_POS(pos)

/****************** COLOR SCALE ***********************/

uint32_t ChangeFilamentScreen::getWarmColor(uint16_t temp, uint16_t cool, uint16_t low, uint16_t med, uint16_t high) {
  rgb_t R0, R1, mix;

  float t;
  if (temp < cool) {
    R0 = cool_rgb;
    R1 = low_rgb;
    t  = 0;
  }
  else if (temp < low) {
    R0 = cool_rgb;
    R1 = low_rgb;
    t = (float(temp)-cool)/(low-cool);
  }
  else if (temp < med) {
    R0 = low_rgb;
    R1 = med_rgb;
    t = (float(temp)-low)/(med-low);
  }
  else if (temp < high) {
    R0 = med_rgb;
    R1 = high_rgb;
    t = (float(temp)-med)/(high-med);
  }
  else if (temp >= high) {
    R0 = med_rgb;
    R1 = high_rgb;
    t = 1;
  }
  rgb_t::lerp(t, R0, R1, mix);
  return mix;
}

void ChangeFilamentScreen::drawTempGradient(uint16_t x, uint16_t y, uint16_t w, uint16_t h) {
  CommandProcessor cmd;
  cmd.cmd(SCISSOR_XY   (x, y))
     .cmd(SCISSOR_SIZE (w, h/2))
     .gradient         (x, y,     high_rgb, x, y+h/2, med_rgb)
     .cmd(SCISSOR_XY   (x, y+h/2))
     .cmd(SCISSOR_SIZE (w, h/2))
     .gradient         (x, y+h/2, med_rgb,  x, y+h, low_rgb)
     .cmd(SCISSOR_XY   ())
     .cmd(SCISSOR_SIZE ());
}

void ChangeFilamentScreen::onEntry() {
  BaseScreen::onEntry();
  mydata.e_tag = ExtUI::getActiveTool() + 10;
  mydata.t_tag = 0;
  mydata.repeat_tag = 0;
  mydata.saved_extruder = getActiveTool();
  #if FILAMENT_UNLOAD_PURGE_LENGTH > 0
    mydata.need_purge = true;
  #endif
}

void ChangeFilamentScreen::onExit() {
  setActiveTool(mydata.saved_extruder, true);
}

void ChangeFilamentScreen::onRedraw(draw_mode_t what) {
  CommandProcessor cmd;

  if (what & BACKGROUND) {
    cmd.cmd(CLEAR_COLOR_RGB(bg_color))
       .cmd(CLEAR(true,true,true))
       .cmd(COLOR_RGB(bg_text_enabled))
       .tag(0)
       .font(TERN(TOUCH_UI_PORTRAIT, font_large, font_medium));
  }

  if (what & FOREGROUND) {
    char e0_str[20], e1_str[20];
    const extruder_t e = getExtruder();

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
      cmd.cmd(COLOR_RGB(temp_button));
    }
    else {
      cmd.cmd(COLOR_RGB(gray_color_1));
    }
    cmd.tag(15)
       .rectangle(E0_TEMP_POS)
       .font(font_medium)
       .colors(normal_btn)
       .text(TEXT_POS(E0_TEMP_POS), e0_str)
       .colors(normal_btn);

    if DISABLED(HAS_MULTI_HOTEND) {
      cmd.font(font_small).cmd(COLOR_RGB(gray_color_1));
    }
    else if (getTargetTemp_celsius(H1) > 0) {
      cmd.font(font_medium).cmd(COLOR_RGB(temp_button));
    }
    else {
      cmd.font(font_medium).cmd(COLOR_RGB(gray_color_1));
    }
    cmd.tag(15)
       .rectangle(E1_TEMP_POS)
       .colors(normal_btn)
       .text(TEXT_POS(E1_TEMP_POS), e1_str)
       .colors(normal_btn);

    cmd.tag(5)
       .cmd (BITMAP_SOURCE(Extruder_Icon_Info))
       .cmd (BITMAP_LAYOUT(Extruder_Icon_Info))
       .cmd (BITMAP_SIZE  (Extruder_Icon_Info))
       .icon(ICON_POS(E0_TEMP_POS), Extruder_Icon_Info, icon_scale)
       .icon(ICON_POS(E1_TEMP_POS), Extruder_Icon_Info, icon_scale);

    const bool t_ok = getActualTemp_celsius(e) > getSoftenTemp() - 30;

    #define TOG_STYLE(A) colors(A ? action_btn : normal_btn)

    const bool tog2  = mydata.t_tag == 2;
    const bool tog3  = mydata.t_tag == 3;
    const bool tog4  = mydata.t_tag == 4;
    const bool tog5  = mydata.t_tag == 5;
    const bool tog10 = mydata.e_tag == 10;
    #if HAS_MULTI_HOTEND
      const bool tog11 = mydata.e_tag == 11;
    #endif

    cmd.TOG_STYLE(tog10)
       .tag(10).font(font_large).button (E1_SEL_POS, F("Extruder 1"))
    #if HOTENDS < 2
       .tag(0).fgcolor(gray_color_1)
    #else
       .TOG_STYLE(tog11).tag(11)
    #endif
       .button (E2_SEL_POS, F("Extruder 2"));

    if (!t_ok) reset_menu_timeout();

    const bool tog7 = mydata.repeat_tag == 7;
    const bool tog8 = mydata.repeat_tag == 8;

    cmd.colors(normal_btn)
       .font(font_medium)
       .tag(0)
       .button(GROUP_1_POS, F(""), OPT_FLAT)
       .button(GROUP_2_POS, F(""), OPT_FLAT)
       .button(GROUP_3_POS, F(""), OPT_FLAT)
       .button(GROUP_4_POS, F(""), OPT_FLAT);
    {
      char str[30];
      format_temp(str, GROUP_1_TEMP);
      cmd.tag(2) .TOG_STYLE(tog2).button (GROUP_1_TEMP_POS,  F( STRINGIFY(GROUP_1_TEMP)));

      format_temp(str, GROUP_2_TEMP);
      cmd.tag(3) .TOG_STYLE(tog3).button (GROUP_2_TEMP_POS, F( STRINGIFY(GROUP_2_TEMP)));

      format_temp(str, GROUP_3_TEMP);
      cmd.tag(4) .TOG_STYLE(tog4).button (GROUP_3_TEMP_POS, F( STRINGIFY(GROUP_3_TEMP)));

      format_temp(str, GROUP_4_TEMP);
      cmd.tag(5) .TOG_STYLE(tog5).button (GROUP_4_TEMP_POS, F( STRINGIFY(GROUP_4_TEMP)));
    }

    cmd.colors(normal_btn)
       .font(font_medium)
       .tag(0)
       .text(GROUP_1_LABL_POS, F("PLA, PVA, PVB"))
       .text(GROUP_2_LABL_POS, F("TPU, Flexibles"))
       .text(GROUP_3_LABL_POS, F("ABS, PETg, ASA"))
       .text(GROUP_4_LABL_POS, F("Nylon, PC"));

    cmd.cmd(COLOR_RGB(t_ok ? bg_text_enabled : bg_text_disabled))
       .colors(normal_btn)
       .font(font_xlarge)
       .tag(7).TOG_STYLE(tog7).enabled(t_ok).button (UNLD_LABL_POS, GET_TEXT_F(MSG_UNLOAD))
       .tag(8).TOG_STYLE(tog8).enabled(t_ok).button (LOAD_LABL_POS, GET_TEXT_F(MSG_LOAD))
       .font(font_medium)
       .tag(1).colors(action_btn).button (BACK_POS, GET_TEXT_F(MSG_BUTTON_DONE));

       if (ExtUI::isPrintingPaused()) {
        cmd.colors(normal_btn)
           .font(font_medium)
           .tag(16).colors(normal_btn).button(FILAMENT_SWAP_POS, GET_TEXT_F(MSG_RESUME_PRINT));
       }
       else {
        #ifdef PARKING_COMMAND_GCODE
        cmd.colors(normal_btn)
           .font(font_medium)
           .tag(17).colors(normal_btn).button(FILAMENT_SWAP_POS, GET_TEXT_F(MSG_FILAMENT_SWAP));
        #endif
       }
  }
}

void ChangeFilamentScreen::loadBitmaps() {
  // Load the bitmaps for the status screen
  using namespace Theme;
  constexpr uint32_t base = ftdi_memory_map::RAM_G;
  CLCD::mem_write_xbm(base + Extruder_Icon_Info.RAMG_offset, Extruder_Icon, sizeof(Extruder_Icon));

  // Load fonts for internationalization
  #if ENABLED(TOUCH_UI_USE_UTF8)
    load_utf8_data(base + UTF8_FONT_OFFSET);
  #endif
}

uint8_t ChangeFilamentScreen::getSoftenTemp() {
  switch (mydata.t_tag) {
    case 2:  return GROUP_1_TEMP;
    case 3:  return GROUP_2_TEMP;
    case 4:  return GROUP_3_TEMP;
    case 5:  return GROUP_4_TEMP;
    default: return EXTRUDE_MINTEMP;
  }
}

ExtUI::extruder_t ChangeFilamentScreen::getExtruder() {
  switch (mydata.e_tag) {
    case 13: return ExtUI::E3;
    case 12: return ExtUI::E2;
    case 11: return ExtUI::E1;
    default: return ExtUI::E0;
  }
}

void ChangeFilamentScreen::doPurge() {
  #if FILAMENT_UNLOAD_PURGE_LENGTH > 0
    constexpr float purge_distance_mm = FILAMENT_UNLOAD_PURGE_LENGTH;
    if (mydata.need_purge) {
      mydata.need_purge = false;
      MoveAxisScreen::setManualFeedrate(getExtruder(), purge_distance_mm);
      ExtUI::setAxisPosition_mm(ExtUI::getAxisPosition_mm(getExtruder()) + purge_distance_mm, getExtruder());
    }
  #endif
}

bool ChangeFilamentScreen::onTouchEnd(uint8_t tag) {
  using namespace ExtUI;
  switch (tag) {
    case 1: GOTO_PREVIOUS(); break;
    case 2:
    case 3:
    case 4:
    case 5:
      // Change temperature
      mydata.t_tag = tag;
      setTargetTemp_celsius(getSoftenTemp(), getExtruder());
      break;
    case 7:
      mydata.repeat_tag = (mydata.repeat_tag == 7) ? 0 : 7;
      break;
    case 8:
      mydata.repeat_tag = (mydata.repeat_tag == 8) ? 0 : 8;
      break;
    case 10:
    case 11:
      // Change extruder
      mydata.e_tag = tag;
      mydata.t_tag = 0;
      mydata.repeat_tag = 0;
      #if FILAMENT_UNLOAD_PURGE_LENGTH > 0
        mydata.need_purge = true;
      #endif
      setActiveTool(getExtruder(), true);
      break;
    case 15: GOTO_SCREEN(TemperatureScreen); break;
    case 16:
      if (ExtUI::isPrintingPaused()) {
        injectCommands(F("M117 Print Resumed")); resumePrint(); GOTO_SCREEN(StatusScreen);
      }
      break;
    #ifdef PARKING_COMMAND_GCODE
      case 17: injectCommands(F(PARKING_COMMAND_GCODE)); break;
    #endif
  }
  return true;
}

bool ChangeFilamentScreen::onTouchHeld(uint8_t tag) {
  if (ExtUI::isMoving()) return false; // Don't allow moves to accumulate
  constexpr float increment = 1;
  #define UI_INCREMENT_AXIS(axis) UI_INCREMENT(AxisPosition_mm, axis);
  #define UI_DECREMENT_AXIS(axis) UI_DECREMENT(AxisPosition_mm, axis);
  switch (tag) {
    case 7: UI_DECREMENT_AXIS(getExtruder()); break;
    case 6: case 8: UI_INCREMENT_AXIS(getExtruder()); break;
    default: return false;
  }
  #undef UI_DECREMENT_AXIS
  #undef UI_INCREMENT_AXIS
  return false;
}

void ChangeFilamentScreen::onIdle() {
  reset_menu_timeout();
  if (mydata.repeat_tag) onTouchHeld(mydata.repeat_tag);
  if (refresh_timer.elapsed(STATUS_UPDATE_INTERVAL)) {
    onRefresh();
    refresh_timer.start();
  }
  BaseScreen::onIdle();
}

#endif // FTDI_CHANGE_FILAMENT_SCREEN
