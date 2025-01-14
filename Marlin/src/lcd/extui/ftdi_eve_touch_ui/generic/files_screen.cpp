/********************
 * files_screen.cpp *
 ********************/

/****************************************************************************
 *   Written By Mark Pelletier  2017 - Aleph Objects, Inc.                  *
 *   Written By Marcio Teixeira 2018 - Aleph Objects, Inc.                  *
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

#ifdef FTDI_FILES_SCREEN

#if ENABLED(TOUCH_UI_PORTRAIT)
  #define GRID_COLS  6
  #define GRID_ROWS 15
  #define FILES_PER_PAGE 11
  #define PREV_DIR LEFT
  #define NEXT_DIR RIGHT

  #define PREV_POS BTN_POS(1,1),  BTN_SIZE(1,2)
  #define HEAD_POS BTN_POS(2,1),  BTN_SIZE(4,2)
  #define NEXT_POS BTN_POS(6,1),  BTN_SIZE(1,2)
  #define LIST_POS BTN_POS(1,3),  BTN_SIZE(6,FILES_PER_PAGE)
  #define BTN1_POS BTN_POS(1,14), BTN_SIZE(3,2)
  #define BTN2_POS BTN_POS(4,14), BTN_SIZE(3,2)
#else
  #define GRID_COLS 12
  #define GRID_ROWS  8
  #define FILES_PER_PAGE 6
  #define PREV_DIR UP
  #define NEXT_DIR DOWN

  #define PREV_POS BTN_POS(12,2), BTN_SIZE(1,3)
  #define HEAD_POS BTN_POS( 1,1), BTN_SIZE(12,1)
  #define NEXT_POS BTN_POS(12,5), BTN_SIZE(1,4)
  #define LIST_POS BTN_POS( 1,2), BTN_SIZE(11,FILES_PER_PAGE)
  #define BTN1_POS BTN_POS( 1,8), BTN_SIZE(6,1)
  #define BTN2_POS BTN_POS( 7,8), BTN_SIZE(5,1)
#endif

using namespace FTDI;
using namespace ExtUI;
using namespace Theme;

constexpr static FilesScreenData &mydata = screen_data.FilesScreen;

void FilesScreen::onEntry() {
  mydata.cur_page        = 0;
  mydata.selected_tag    = 0xFF;
  #if ENABLED(SCROLL_LONG_FILENAMES) && (FTDI_API_LEVEL >= 810)
    CLCD::mem_write_32(CLCD::REG::MACRO_0,DL::NOP);
  #endif
  gotoPage(0);
  BaseScreen::onEntry();
}

const char *FilesScreen::getSelectedFilename(bool shortName) {
  FileList files;
  files.seek(getSelectedFileIndex(), true);
  return shortName ? files.shortFilename() : files.filename();
}

void FilesScreen::drawSelectedFile() {
  if (mydata.selected_tag == 0xFF) return;
  FileList files;
  files.seek(getSelectedFileIndex(), true);
  mydata.flags.is_dir = files.isDir();
  drawFileButton(
    files.filename(),
    mydata.selected_tag,
    mydata.flags.is_dir,
    true
  );
}

uint16_t FilesScreen::getSelectedFileIndex() {
  return getFileForTag(mydata.selected_tag);
}

uint16_t FilesScreen::getFileForTag(uint8_t tag) {
  return mydata.cur_page * FILES_PER_PAGE + tag - 2;
}

void FilesScreen::drawFileButton(int x, int y, int w, int h, const char *filename, uint8_t tag, bool is_dir, bool is_highlighted) {
  #define SUB_COLS 6
  #define SUB_ROWS FILES_PER_PAGE

  const int bx = SUB_X(1);
  const int by = SUB_Y(getLineForTag(tag)+1);
  const int bw = SUB_W(6);
  const int bh = SUB_H(1);

  CommandProcessor cmd;
  cmd.tag(tag);
  cmd.cmd(COLOR_RGB(is_highlighted ? fg_action : bg_color));
  cmd.font(font_medium).rectangle(bx, by, bw, bh);
  cmd.cmd(COLOR_RGB(is_highlighted ? normal_btn.rgb : bg_text_enabled));
  if (TERN0(SCROLL_LONG_FILENAMES, is_highlighted)) {
    cmd.cmd(SAVE_CONTEXT());
    cmd.cmd(SCISSOR_XY(x,y));
    cmd.cmd(SCISSOR_SIZE(w,h));
    cmd.cmd(MACRO(0));
    cmd.text(bx, by, bw, bh, filename, OPT_CENTERY | OPT_NOFIT);
  }
  else
    draw_text_with_ellipsis(cmd, bx, by, bw - (is_dir ? 20 : 0), bh, filename, OPT_CENTERY, font_medium);
  if (is_dir && !is_highlighted) cmd.text(bx, by, bw, bh, F("> "),  OPT_CENTERY | OPT_RIGHTX);
  if (TERN0(SCROLL_LONG_FILENAMES, is_highlighted)) cmd.cmd(RESTORE_CONTEXT());
}

void FilesScreen::drawFileList() {
  FileList files;
  mydata.num_page = max(1,ceil(float(files.count()) / FILES_PER_PAGE));
  mydata.cur_page = min(mydata.cur_page, mydata.num_page-1);
  mydata.flags.is_root  = files.isAtRootDir();
  mydata.flags.is_empty = true;

  uint16_t fileIndex = mydata.cur_page * FILES_PER_PAGE;
  for (uint8_t i = 0; i < FILES_PER_PAGE; i++, fileIndex++) {
    if (!files.seek(fileIndex)) break;
    drawFileButton(files.filename(), getTagForLine(i), files.isDir(), false);
    mydata.flags.is_empty = false;
  }
}

void FilesScreen::drawHeader() {
  char str[16];
  sprintf_P(str, PSTR("Page %d of %d"), mydata.cur_page + 1, mydata.num_page);

  CommandProcessor cmd;
  cmd.colors(normal_btn)
     .font(font_small)
     .tag(0).button(HEAD_POS, str, OPT_CENTER | OPT_FLAT);
}

void FilesScreen::drawArrows() {
  const bool prev_enabled = mydata.cur_page > 0;
  const bool next_enabled = mydata.cur_page < (mydata.num_page - 1);

  CommandProcessor cmd;
  cmd.colors(normal_btn);
  cmd.tag(242).enabled(prev_enabled).button(PREV_POS, F("")); if (prev_enabled) drawArrow(PREV_POS, PREV_DIR);
  cmd.tag(243).enabled(next_enabled).button(NEXT_POS, F("")); if (next_enabled) drawArrow(NEXT_POS, NEXT_DIR);
}

void FilesScreen::drawFooter() {
  const bool has_selection = mydata.selected_tag != 0xFF;

  CommandProcessor cmd;
  cmd.colors(normal_btn)
     .font(font_medium)
     .colors(has_selection ? normal_btn : action_btn);

  if (mydata.flags.is_root)
    cmd.tag(240).button(BTN2_POS, GET_TEXT_F(MSG_BUTTON_DONE));
  else
    cmd.tag(245).button(BTN2_POS, F("Back"));

  cmd.enabled(has_selection)
     .colors(has_selection ? action_btn : normal_btn);

  if (mydata.flags.is_dir)
    cmd.tag(244).button(BTN1_POS, GET_TEXT_F(MSG_BUTTON_OPEN));
  else
    cmd.tag(241).button(BTN1_POS, GET_TEXT_F(MSG_BUTTON_PRINT));
}

void FilesScreen::drawFileButton(const char *filename, uint8_t tag, bool is_dir, bool is_highlighted) {
  #undef  MARGIN_L
  #undef  MARGIN_R
  #define MARGIN_L 0
  #define MARGIN_R 0
  drawFileButton(LIST_POS, filename, tag, is_dir, is_highlighted);
}

void FilesScreen::onRedraw(draw_mode_t what) {
  if (what & FOREGROUND) {
    drawHeader();
    drawArrows();
    drawSelectedFile();
    drawFooter();
  }
}

void FilesScreen::gotoPage(uint8_t page) {
  mydata.selected_tag = 0xFF;
  mydata.cur_page     = page;
  CommandProcessor cmd;
  cmd.cmd(CMD_DLSTART)
     .cmd(CLEAR_COLOR_RGB(bg_color))
     .cmd(CLEAR(true,true,true))
     .colors(normal_btn);
  drawFileList();
  storeBackground();
}

bool FilesScreen::onTouchEnd(uint8_t tag) {
  switch (tag) {
    case 240: // Done button
      GOTO_PREVIOUS();
      return true;
    case 241: // Print highlighted file
      printFile(getSelectedShortFilename());
      GOTO_SCREEN(StatusScreen);
      return true;
    case 242: // Previous page
      if (mydata.cur_page > 0) {
        gotoPage(mydata.cur_page-1);
      }
      break;
    case 243: // Next page
      if (mydata.cur_page < (mydata.num_page-1)) {
        gotoPage(mydata.cur_page+1);
      }
      break;
    case 244: // Select directory
      {
        FileList files;
        files.changeDir(getSelectedShortFilename());
        gotoPage(0);
      }
      break;
    case 245: // Up directory
      {
        FileList files;
        files.upDir();
        gotoPage(0);
      }
      break;
    default: // File selected
      if (tag < 240) {
        mydata.selected_tag = tag;
        #if ENABLED(SCROLL_LONG_FILENAMES) && (FTDI_API_LEVEL >= 810)
          mydata.scroll_pos = 0;
          mydata.scroll_max = 0;
          if (FTDI::ftdi_chip >= 810) {
            const char *filename = getSelectedFilename();
            if (filename[0]) {
              CommandProcessor cmd;
              constexpr int dim[4] = {LIST_POS};
              const uint16_t text_width = cmd.font(font_medium).text_width(filename);
              if (text_width > dim[2])
                mydata.scroll_max = text_width - dim[2] + MARGIN_L + MARGIN_R + 10;
            }
          }
        #endif
      }
      break;
  }
  return true;
}

void FilesScreen::onIdle() {
  #if ENABLED(SCROLL_LONG_FILENAMES) && (FTDI_API_LEVEL >= 810)
    if (FTDI::ftdi_chip >= 810) {
      CLCD::mem_write_32(CLCD::REG::MACRO_0,
        VERTEX_TRANSLATE_X(-int32_t(mydata.scroll_pos)));
      if (mydata.scroll_pos < mydata.scroll_max * 16)
        mydata.scroll_pos++;
    }
  #endif
}

void FilesScreen::onMediaRemoved() {
  if (AT_SCREEN(FilesScreen)) GOTO_SCREEN(StatusScreen);
}

#endif // FTDI_FILES_SCREEN
