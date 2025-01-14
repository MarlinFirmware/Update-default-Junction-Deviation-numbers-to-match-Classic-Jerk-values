/*********************************
 * cocoa_press/status_screen.cpp *
 *********************************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2019 - Cocoa Press                          *
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
 *   location: <https://www.gnu.org/licenses/>.                              *
 ****************************************************************************/

#include "../config.h"
#include "../screens.h"
#include "../screen_data.h"

#ifdef COCOA_STATUS_SCREEN

#include "cocoa_press_ui.h"
#include "cocoa_press_bitmap.h"

#define POLY(A) PolyUI::poly_reader_t(A, sizeof(A)/sizeof(A[0]))
#define ICON_POS(x,y,w,h) x,     y,     h, h
#define TEXT_POS(x,y,w,h) x + h, y, w - h, h

using namespace FTDI;
using namespace Theme;
using namespace ExtUI;

const uint8_t shadow_depth = 5;

constexpr static StatusScreenData &mydata = screen_data.StatusScreen;

// Format for background image

constexpr uint8_t  format   = RGB332;
constexpr uint16_t bitmap_w = 800;
constexpr uint16_t bitmap_h = 480;

void StatusScreen::_format_time(char *outstr, uint32_t time) {
  const uint8_t hrs = time / 3600,
                min = (time / 60) % 60,
                sec = time % 60;
  if (hrs)
    sprintf_P(outstr, PSTR("%02d:%02d"), hrs, min);
  else
    sprintf_P(outstr, PSTR("%02d:%02ds"), min, sec);
}

void StatusScreen::loadBitmaps() {
  // Load the bitmaps for the status screen
  using namespace Theme;
  constexpr uint32_t base = ftdi_memory_map::RAM_G;
  CLCD::mem_write_xbm(base + Light_Bulb_Info.RAMG_offset,    Light_Bulb,    sizeof(Light_Bulb));
  CLCD::mem_write_xbm(base + Chamber_Icon_Info.RAMG_offset,  Chamber_Icon,  sizeof(Chamber_Icon));
  CLCD::mem_write_xbm(base + Clock_Icon_Info.RAMG_offset,    Clock_Icon,    sizeof(Clock_Icon));
  CLCD::mem_write_xbm(base + File_Icon_Info.RAMG_offset,     File_Icon,     sizeof(File_Icon));
  CLCD::mem_write_xbm(base + TD_Icon_Info.RAMG_offset,       TD_Icon,       sizeof(TD_Icon));
  CLCD::mem_write_xbm(base + Extruder_Icon_Info.RAMG_offset, Extruder_Icon, sizeof(Extruder_Icon));
  CLCD::mem_write_xbm(base + Bed_Heat_Icon_Info.RAMG_offset, Bed_Heat_Icon, sizeof(Bed_Heat_Icon));
  CLCD::mem_write_xbm(base + Fan_Icon_Info.RAMG_offset,      Fan_Icon,      sizeof(Fan_Icon));

  // Load fonts for internationalization
  #if ENABLED(TOUCH_UI_USE_UTF8)
    load_utf8_data(base + UTF8_FONT_OFFSET);
  #endif
}

void StatusScreen::draw_bkgnd(draw_mode_t what) {
  if (what & BACKGROUND) {
    constexpr float scale_w = float(FTDI::display_width)/bitmap_w;
    constexpr float scale_h = float(FTDI::display_height)/bitmap_h;
    uint16_t linestride;
    uint32_t color;
    switch (format) {
      case RGB565: linestride = bitmap_w * 2; color = 0xFFFFFF; break;
      case RGB332: linestride = bitmap_w    ; color = 0xFFFFFF; break;
      case L1:     linestride = bitmap_w/8  ; color = 0x000000; break;
      case L2:     linestride = bitmap_w/4  ; color = 0x000000; break;
      case L4:     linestride = bitmap_w/2  ; color = 0x000000; break;
      case L8:     linestride = bitmap_w    ; color = 0x000000; break;
    }
    CommandProcessor cmd;
    cmd.cmd(COLOR_RGB(color))
       .cmd(BITMAP_SOURCE(BACKGROUND_OFFSET))
       .tag(0)
       .bitmap_layout(format, linestride, bitmap_h)
       .bitmap_size(NEAREST, BORDER, BORDER, bitmap_w*scale_w, bitmap_h*scale_h)
       .cmd(BITMAP_TRANSFORM_A(uint32_t(float(256)/scale_w)))
       .cmd(BITMAP_TRANSFORM_E(uint32_t(float(256)/scale_h)))
       .cmd(BEGIN(BITMAPS))
       .cmd(VERTEX2II(0, 0, 0, 0))
       .cmd(BITMAP_TRANSFORM_A(256))
       .cmd(BITMAP_TRANSFORM_E(256))
       .cmd(COLOR_RGB(bg_text_enabled));
  }
}

void StatusScreen::send_buffer(CommandProcessor &cmd, const void *data, uint16_t len) {
  const char *ptr = (const char*) data;
  constexpr uint16_t block_size = 512;
  char               block[block_size];
  for (;len > 0;) {
    const uint16_t nBytes = min(len, block_size);
    memcpy_P(block, ptr, nBytes);
    cmd.write((const void*)block, nBytes);
    cmd.execute();
    if (cmd.has_fault()) {
      SERIAL_ECHOLNPGM("Recovering from fault: ");
      cmd.reset();
      delay(1000);
      return;
    }
    ptr += nBytes;
    len -= nBytes;
  }
}

void StatusScreen::load_background(const void *data, uint16_t len) {
  CommandProcessor cmd;
  cmd.inflate(BACKGROUND_OFFSET)
     .execute();
  send_buffer(cmd, data, len);
  cmd.wait();
}

void StatusScreen::draw_time(draw_mode_t what) {
  CommandProcessor cmd;
  PolyUI ui(cmd, what);

  int16_t x, y, w, h;
  ui.bounds(POLY(print_time_hms), x, y, w, h);

  if (what & BACKGROUND) {
    cmd.cmd(COLOR_RGB(bg_text_enabled))
       .cmd (BITMAP_SOURCE(Clock_Icon_Info))
       .cmd (BITMAP_LAYOUT(Clock_Icon_Info))
       .cmd (BITMAP_SIZE  (Clock_Icon_Info))
       .icon(ICON_POS(x, y, w, h), Clock_Icon_Info, icon_scale)
       .cmd(COLOR_RGB(bg_text_enabled));
  }

  if (what & FOREGROUND) {
    const uint32_t elapsed = getProgress_seconds_elapsed();
    char elapsed_str[10];
    _format_time(elapsed_str, elapsed);

    cmd.font(font_medium)
       .cmd(COLOR_RGB(bg_text_enabled))
       .text(TEXT_POS(x, y, w, h), elapsed_str);
  }
}

void StatusScreen::draw_percent(draw_mode_t what) {
  CommandProcessor cmd;
  PolyUI ui(cmd, what);

  int16_t x, y, w, h;
  ui.bounds(POLY(print_time_pct), x, y, w, h);

  if (what & FOREGROUND) {
    const uint16_t current_progress = TERN(HAS_PRINT_PROGRESS_PERMYRIAD, getProgress_permyriad(), getProgress_percent() * 100);
    char progress_str[10];
    sprintf_P(progress_str,
      #if ENABLED(PRINT_PROGRESS_SHOW_DECIMALS)
        PSTR("%3d.%02d%%"), uint8_t(current_progress / 100), current_progress % 100
      #else
        PSTR("%3d%%"), uint8_t(current_progress / 100)
      #endif
    );

    cmd.font(font_medium)
       .cmd(COLOR_RGB(bg_text_enabled))
       .text(TEXT_POS(x, y, w, h), progress_str);
  }
}

void StatusScreen::draw_temperature(draw_mode_t what) {
  CommandProcessor cmd;
  PolyUI ui(cmd, what);

  int16_t x, y, w, h;

  if (what & BACKGROUND) {
    cmd.cmd(COLOR_RGB(bg_text_enabled));
    cmd.font(font_medium).tag(0);

    ui.bounds(POLY(h0_label), x, y, w, h);
    cmd.text(x, y, w, h, GET_TEXT_F(MSG_NOZZLE));

    ui.bounds(POLY(h1_label), x, y, w, h);
    cmd.text(x, y, w, h, GET_TEXT_F(MSG_BODY));

    #if ENABLED(TOUCH_UI_USE_UTF8)
      load_utf8_bitmaps(cmd); // Restore font bitmap handles
    #endif
  }

  if (what & FOREGROUND) {
    char str[15];
    cmd.font(font_medium).colors(normal_btn).tag(10);

    // Show the actual temperatures

    format_temp(str, getActualTemp_celsius(E0));
    ui.bounds(POLY(h0_temp), x, y, w, h);
    cmd.button(x, y, w, h, str);

    format_temp(str, getActualTemp_celsius(E1));
    ui.bounds(POLY(h1_temp), x, y, w, h);
    cmd.button(x, y, w, h, str);
  }
}

void StatusScreen::draw_buttons(draw_mode_t what) {
  if (what & FOREGROUND) {
    int16_t x, y, w, h;

    const bool can_print        = !isPrinting() && isMediaMounted() && isFileSelected();
    const bool can_select       = !isPrinting() && isMediaMounted();
    const bool sdOrHostPrinting = ExtUI::isPrinting();
    const bool sdOrHostPaused   = ExtUI::isPrintingPaused();

    CommandProcessor cmd;
    PolyUI ui(cmd, what);

    cmd.font(font_medium).colors(normal_btn);

    ui.bounds(POLY(load_chocolate_btn), x, y, w, h);
    cmd.tag(1).button(x, y, w, h, GET_TEXT_F(MSG_LOAD_UNLOAD));

    ui.bounds(POLY(extrude_btn), x, y, w, h);
    cmd.tag(2).button(x, y, w, h, GET_TEXT_F(MSG_EXTRUDE));

    ui.bounds(POLY(preheat_chocolate_btn), x, y, w, h);
    cmd.tag(3).button(x, y, w, h, GET_TEXT_F(MSG_PREHEAT_CHOCOLATE));

    ui.bounds(POLY(menu_btn), x, y, w, h);
    cmd.tag(4).button(x, y, w, h, GET_TEXT_F(MSG_BUTTON_MENU));

    ui.bounds(POLY(media_btn), x, y, w, h);
    cmd.tag(5).enabled(can_select).button(x, y, w, h, GET_TEXT_F(MSG_BUTTON_MEDIA));

    ui.bounds(POLY(print_btn), x, y, w, h);
    cmd.tag(6).colors(action_btn).enabled(can_print).button(x, y, w, h, GET_TEXT_F(MSG_BUTTON_PRINT));

    ui.bounds(POLY(pause_btn), x, y, w, h);
    cmd.tag(sdOrHostPaused ? 8 : 7).enabled(sdOrHostPrinting).button(x, y, w, h, sdOrHostPaused ? GET_TEXT_F(MSG_BUTTON_RESUME) : GET_TEXT_F(MSG_BUTTON_PAUSE));

    ui.bounds(POLY(stop_btn), x, y, w, h);
    cmd.tag(9).enabled(sdOrHostPrinting).button(x, y, w, h, GET_TEXT_F(MSG_BUTTON_STOP));
  }
}

// When visible, the file name occupies the same space as the status
// message and must be drawn opaque.
void StatusScreen::draw_file(draw_mode_t what) {
  if (mydata.gotMessage) return;

  if (what & FOREGROUND) {
    int16_t x, y, w, h;

    CommandProcessor cmd;
    PolyUI ui(cmd, what);
    ui.bounds(POLY(file_name), x, y, w, h);

    cmd.tag(5)
       .cmd (COLOR_RGB(bg_color))
       .rectangle(x, y, w, h)
       .cmd (COLOR_RGB(bg_text_enabled))
       .cmd (BITMAP_SOURCE(File_Icon_Info))
       .cmd (BITMAP_LAYOUT(File_Icon_Info))
       .cmd (BITMAP_SIZE  (File_Icon_Info))
       .icon(ICON_POS(x, y, w, h), File_Icon_Info, icon_scale);

    if (!isMediaMounted())
      draw_text_with_ellipsis(cmd, TEXT_POS(x, y, w, h), F("No media present"), OPT_CENTERY, font_small);
    else if (isFileSelected()) {
      FileList list;
      draw_text_with_ellipsis(cmd, TEXT_POS(x, y, w, h), list.filename(), OPT_CENTERY, font_small);
    }
    else
      draw_text_with_ellipsis(cmd, TEXT_POS(x, y, w, h), F("No file selected"), OPT_CENTERY, font_small);
  }
}

// The message will be drawn on the background and may be obscured by
// the filename.
void StatusScreen::draw_message(draw_mode_t what, const char *message) {
  if (what & BACKGROUND) {
    int16_t x, y, w, h;

    CommandProcessor cmd;
    PolyUI ui(cmd, what);
    ui.bounds(POLY(file_name), x, y, w, h);

    cmd.cmd(COLOR_RGB(bg_text_enabled));
    draw_text_box(cmd, TEXT_POS(x, y, w, h), message, OPT_CENTERY, font_small);
  }
}

bool StatusScreen::isFileSelected() {
  if (!isMediaMounted()) return false;
  FileList list;
  if (list.isDir()) return false;
  const char *filename = list.filename();
  if (filename[0] == '\0') return false;
  return true;
}

void StatusScreen::onRedraw(draw_mode_t what) {
  if (what & FOREGROUND) {
    draw_bkgnd(what);
    draw_file(what);
    draw_time(what);
    draw_percent(what);
    draw_temperature(what);
    draw_buttons(what);
  }
}

bool StatusScreen::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case  1: GOTO_SCREEN(LoadChocolateScreen); break;
    case  3: GOTO_SCREEN(PreheatMenu); break;
    case  4: GOTO_SCREEN(MainMenu); break;
    case  5: GOTO_SCREEN(FilesScreen); break;
    case  6: GOTO_SCREEN(ConfirmStartPrintDialogBox); break;
    case  7:
      sound.play(twinkle, PLAY_ASYNCHRONOUS);
      if (ExtUI::isPrintingFromMedia())
        ExtUI::pausePrint();
      #ifdef ACTION_ON_PAUSE
        else hostui.pause();
      #endif
      GOTO_SCREEN(StatusScreen);
      break;
    case  8:
      sound.play(twinkle, PLAY_ASYNCHRONOUS);
      if (ExtUI::isPrintingFromMedia())
        ExtUI::resumePrint();
      #ifdef ACTION_ON_RESUME
        else hostui.resume();
      #endif
      GOTO_SCREEN(StatusScreen);
      break;
    case  9:
      GOTO_SCREEN(ConfirmAbortPrintDialogBox);
      current_screen.forget();
      PUSH_SCREEN(StatusScreen);
      break;

    case 10: GOTO_SCREEN(TemperatureScreen); break;
    default: return false;
  }
  // If a passcode is enabled, the LockScreen will prevent the
  // user from proceeding.
  LockScreen::check_passcode();
  return true;
}

bool StatusScreen::onTouchHeld(uint8_t tag) {
  if (tag == 2 && !ExtUI::isMoving()) {
    float increment;
    LoadChocolateScreen::setManualFeedrateAndIncrement(0.25, increment);
    UI_INCREMENT(AxisPosition_mm, E0);
  }
  return false;
}

void StatusScreen::setStatusMessage(FSTR_P message) {
  char buff[strlen_P((const char * const)message)+1];
  strcpy_P(buff, (const char * const) message);
  setStatusMessage((const char *) buff);
}

void StatusScreen::setStatusMessage(const char * const message) {
  if (CommandProcessor::is_processing()) {
    #if ENABLED(TOUCH_UI_DEBUG)
      SERIAL_ECHO_MSG("Cannot update status message, command processor busy");
    #endif
    return;
  }

  CommandProcessor cmd;
  cmd.cmd(CMD_DLSTART)
     .cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true));

  const draw_mode_t what = BACKGROUND;
  draw_bkgnd(what);
  draw_message(what, message);
  draw_time(what);
  draw_percent(what);
  draw_temperature(what);
  draw_buttons(what);

  storeBackground();

  #if ENABLED(TOUCH_UI_DEBUG)
    SERIAL_ECHO_MSG("New status message: ", message);
  #endif

  mydata.gotMessage = true;

  if (AT_SCREEN(StatusScreen))
    current_screen.onRefresh();
}

void StatusScreen::onEntry() {
  mydata.gotMessage = false;
  load_background(cocoa_press_ui, sizeof(cocoa_press_ui));
}

void StatusScreen::onIdle() {
  reset_menu_timeout();
  if (refresh_timer.elapsed(STATUS_UPDATE_INTERVAL)) {
    if (!EventLoop::is_touch_held()) onRefresh();
    refresh_timer.start();
  }
}

void StatusScreen::onMediaMounted() {
  if (AT_SCREEN(StatusScreen))
    setStatusMessage(GET_TEXT_F(MSG_MEDIA_INSERTED));
}

void StatusScreen::onMediaRemoved() {
  if (AT_SCREEN(StatusScreen) || ExtUI::isPrintingFromMedia())
    setStatusMessage(GET_TEXT_F(MSG_MEDIA_REMOVED));
}

#endif // COCOA_STATUS_SCREEN
