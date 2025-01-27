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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_TFT_LVGL_UI

#include "draw_ui.h"
#include <lv_conf.h>
//#include "../lvgl/src/lv_objx/lv_imgbtn.h"
//#include "../lvgl/src/lv_objx/lv_img.h"
//#include "../lvgl/src/lv_core/lv_disp.h"
//#include "../lvgl/src/lv_core/lv_refr.h"

#include "../../../sd/cardreader.h"
#include "../../../inc/MarlinConfig.h"

static lv_obj_t *scr;
extern lv_group_t*  g;

static lv_obj_t *buttonPageUp, *buttonPageDown, *buttonBack,
                *buttonGcode[FILE_BTN_CNT], *labelPageUp[FILE_BTN_CNT], *buttonText[FILE_BTN_CNT];

enum {
  ID_P_UP = 7,
  ID_P_DOWN,
  ID_P_RETURN
};

int8_t curDirLever = 0;
LIST_FILE list_file;
DIR_OFFSET dir_offset[10];

extern uint8_t public_buf[513];
extern char public_buf_m[100];

uint8_t sel_id = 0;

#if HAS_MEDIA

  static uint8_t search_file() {
    int valid_name_cnt = 0;
    //char tmp[SHORT_NEME_LEN*MAX_DIR_LEVEL+1];

    list_file.Sd_file_cnt = 0;
    //list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_first_offset;

    //root2.rewind();
    //SERIAL_ECHOLN(list_file.curDirPath);

    if (curDirLever != 0)
      card.cd(list_file.curDirPath);
    else
      card.cdroot();

    const int16_t fileCnt = card.get_num_items();

    for (int16_t i = 0; i < fileCnt; i++) {
      if (list_file.Sd_file_cnt == list_file.Sd_file_offset) {
        card.selectFileByIndexSorted(i);

        list_file.IsFolder[valid_name_cnt] = card.flag.filenameIsDir;
        strcpy(list_file.file_name[valid_name_cnt], list_file.curDirPath);
        strcat_P(list_file.file_name[valid_name_cnt], PSTR("/"));
        strcat(list_file.file_name[valid_name_cnt], card.filename);
        strcpy(list_file.long_name[valid_name_cnt], card.longest_filename());

        valid_name_cnt++;
        if (valid_name_cnt == 1)
          dir_offset[curDirLever].cur_page_first_offset = list_file.Sd_file_offset;
        if (valid_name_cnt >= FILE_NUM) {
          dir_offset[curDirLever].cur_page_last_offset = list_file.Sd_file_offset;
          list_file.Sd_file_offset++;
          break;
        }
        list_file.Sd_file_offset++;
      }
      list_file.Sd_file_cnt++;
    }
    //card.closefile(false);
    return valid_name_cnt;
  }

#endif // HAS_MEDIA

bool have_pre_pic(char *path) {
  #if HAS_MEDIA
    char *ps1, *ps2, *cur_name = strrchr(path, '/');
    card.openFileRead(cur_name);
    card.read(public_buf, 512);
    ps1 = strstr((char *)public_buf, ";simage:");
    card.read(public_buf, 512);
    ps2 = strstr((char *)public_buf, ";simage:");
    card.closefile();
    if (ps1 || ps2) return true;
  #endif

  return false;
}

static void event_handler(lv_obj_t *obj, lv_event_t event) {
  if (event != LV_EVENT_RELEASED) return;
  uint8_t i, file_count = 0;
  if (obj->mks_obj_id == ID_P_UP) {
    if (dir_offset[curDirLever].curPage > 0) {
      // 2015.05.19
      list_file.Sd_file_cnt = 0;

      if (dir_offset[curDirLever].cur_page_first_offset >= FILE_NUM)
        list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_first_offset - FILE_NUM;

      TERN_(HAS_MEDIA, file_count = search_file());
      if (file_count != 0) {
        dir_offset[curDirLever].curPage--;
        lv_clear_print_file();
        disp_gcode_icon(file_count);
      }
    }
  }
  else if (obj->mks_obj_id == ID_P_DOWN) {
    if (dir_offset[curDirLever].cur_page_last_offset > 0) {
      list_file.Sd_file_cnt    = 0;
      list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_last_offset + 1;
      TERN_(HAS_MEDIA, file_count = search_file());
      if (file_count != 0) {
        dir_offset[curDirLever].curPage++;
        lv_clear_print_file();
        disp_gcode_icon(file_count);
      }
      if (file_count < FILE_NUM)
        dir_offset[curDirLever].cur_page_last_offset = 0;
    }
  }
  else if (obj->mks_obj_id == ID_P_RETURN) {
    if (curDirLever > 0) {
      int8_t *ch = (int8_t *)strrchr(list_file.curDirPath, '/');
      if (ch) {
        *ch = 0;
        TERN_(HAS_MEDIA, card.cdup());
        dir_offset[curDirLever].curPage               = 0;
        dir_offset[curDirLever].cur_page_first_offset = 0;
        dir_offset[curDirLever].cur_page_last_offset  = 0;
        curDirLever--;
        list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_first_offset;
        TERN_(HAS_MEDIA, file_count = search_file());
        lv_clear_print_file();
        disp_gcode_icon(file_count);
      }
    }
    else {
      lv_clear_print_file();
      TERN(HAS_MULTI_VOLUME, lv_draw_media_select(), lv_draw_ready_print());
    }
  }
  else {
    for (i = 0; i < FILE_BTN_CNT; i++) {
      if (obj->mks_obj_id == (i + 1)) {
        if (list_file.file_name[i][0] != 0) {
          if (list_file.IsFolder[i]) {
            strcpy(list_file.curDirPath, list_file.file_name[i]);
            curDirLever++;
            list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_first_offset;
            TERN_(HAS_MEDIA, file_count = search_file());
            lv_clear_print_file();
            disp_gcode_icon(file_count);
          }
          else {
            sel_id = i;
            lv_clear_print_file();
            lv_draw_dialog(DIALOG_TYPE_PRINT_FILE);
          }
          break;
        }
      }
    }
  }
}

void lv_draw_print_file() {
  //uint8_t i;
  uint8_t file_count;

  curDirLever = 0;
  dir_offset[curDirLever].curPage = 0;

  list_file.Sd_file_offset = 0;
  list_file.Sd_file_cnt = 0;

  ZERO(dir_offset);
  ZERO(list_file.IsFolder);
  ZERO(list_file.curDirPath);

  list_file.Sd_file_offset = dir_offset[curDirLever].cur_page_first_offset;
  #if HAS_MEDIA
    card.mount();
    file_count = search_file();
  #endif
  disp_gcode_icon(file_count);

  //lv_obj_t *labelPageUp = lv_label_create_empty(buttonPageUp);
  //lv_obj_t *labelPageDown = lv_label_create_empty(buttonPageDown);
  //lv_obj_t *label_Back = lv_label_create_empty(buttonBack);

  /*
  if (gCfgItems.multiple_language) {
    lv_label_set_text(labelPageUp, tool_menu.preheat);
    lv_obj_align(labelPageUp, buttonPageUp, LV_ALIGN_IN_BOTTOM_MID,0, BUTTON_TEXT_Y_OFFSET);

    lv_label_set_text(labelPageDown, tool_menu.extrude);
    lv_obj_align(labelPageDown, buttonPageDown, LV_ALIGN_IN_BOTTOM_MID,0, BUTTON_TEXT_Y_OFFSET);

    lv_label_set_text(label_Back, common_menu.text_back);
    lv_obj_align(label_Back, buttonBack, LV_ALIGN_IN_BOTTOM_MID,0, BUTTON_TEXT_Y_OFFSET);
  }
  */
}
static char test_public_buf_l[40];
void disp_gcode_icon(uint8_t file_num) {
  uint8_t i;

  // TODO: set current media title?!
  scr = lv_screen_create(PRINT_FILE_UI, "");

  // Create image buttons
  buttonPageUp   = lv_imgbtn_create(scr, "F:/bmp_pageUp.bin", OTHER_BTN_XPIEL * 3 + INTERVAL_V * 4, titleHeight, event_handler, ID_P_UP);
  buttonPageDown = lv_imgbtn_create(scr, "F:/bmp_pageDown.bin", OTHER_BTN_XPIEL * 3 + INTERVAL_V * 4, titleHeight + OTHER_BTN_YPIEL + INTERVAL_H, event_handler, ID_P_DOWN);
  buttonBack     = lv_imgbtn_create(scr, "F:/bmp_back.bin", OTHER_BTN_XPIEL * 3 + INTERVAL_V * 4, titleHeight + OTHER_BTN_YPIEL * 2 + INTERVAL_H * 2, event_handler, ID_P_RETURN);

  // Create labels on the image buttons
  for (i = 0; i < FILE_BTN_CNT; i++) {
    /*
    if (seq) {
      j = (FILE_BTN_CNT-1) - i;
      back_flg = 1;
    }
    else {
      j = i;
      back_flg = 0;
    }
    */
    if (i >= file_num) break;

    #ifdef TFT35
      buttonGcode[i] = lv_imgbtn_create(scr, nullptr);

      lv_imgbtn_use_label_style(buttonGcode[i]);
      lv_obj_clear_protect(buttonGcode[i], LV_PROTECT_FOLLOW);
      lv_btn_set_layout(buttonGcode[i], LV_LAYOUT_OFF);

      ZERO(public_buf_m);
      cutFileName((char *)list_file.long_name[i], 16, 8, (char *)public_buf_m);

      if (list_file.IsFolder[i]) {
        lv_obj_set_event_cb_mks(buttonGcode[i], event_handler, (i + 1), "", 0);
        lv_imgbtn_set_src_both(buttonGcode[i], "F:/bmp_dir.bin");
        if (i < 3)
          lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * i + INTERVAL_V * (i + 1), titleHeight);
        else
          lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * (i - 3) + INTERVAL_V * ((i - 3) + 1), BTN_Y_PIXEL + INTERVAL_H + titleHeight);

        labelPageUp[i] = lv_label_create(buttonGcode[i], public_buf_m);
        lv_obj_align(labelPageUp[i], buttonGcode[i], LV_ALIGN_IN_BOTTOM_MID, 0, -5);
      }
      else {
        if (have_pre_pic((char *)list_file.file_name[i])) {

          //lv_obj_set_event_cb_mks(buttonGcode[i], event_handler, (i + 1), list_file.file_name[i], 1);

          strcpy(test_public_buf_l, "S:");
          strcat(test_public_buf_l, list_file.file_name[i]);
          char *temp = strstr(test_public_buf_l, ".GCO");
          if (temp) strcpy(temp, ".bin");
          lv_obj_set_event_cb_mks(buttonGcode[i], event_handler, (i + 1), test_public_buf_l, 0);
          lv_imgbtn_set_src_both(buttonGcode[i], buttonGcode[i]->mks_pic_name);
          if (i < 3) {
            lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * i + INTERVAL_V * (i + 1) + FILE_PRE_PIC_X_OFFSET, titleHeight + FILE_PRE_PIC_Y_OFFSET);
            buttonText[i] = lv_btn_create(scr, nullptr);
            //lv_obj_set_event_cb(buttonText[i], event_handler);

            lv_btn_use_label_style(buttonText[i]);
            lv_obj_clear_protect(buttonText[i], LV_PROTECT_FOLLOW);
            lv_btn_set_layout(buttonText[i], LV_LAYOUT_OFF);
            //lv_obj_set_event_cb_mks(buttonText[i], event_handler,(i+10),"", 0);
            lv_obj_set_pos(buttonText[i], BTN_X_PIXEL * i + INTERVAL_V * (i + 1) + FILE_PRE_PIC_X_OFFSET, titleHeight + FILE_PRE_PIC_Y_OFFSET + 100);
            lv_obj_set_size(buttonText[i], 100, 40);
          }
          else {
            lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * (i - 3) + INTERVAL_V * ((i - 3) + 1) + FILE_PRE_PIC_X_OFFSET, BTN_Y_PIXEL + INTERVAL_H + titleHeight + FILE_PRE_PIC_Y_OFFSET);
            buttonText[i] = lv_btn_create(scr, nullptr);
            //lv_obj_set_event_cb(buttonText[i], event_handler);

            lv_btn_use_label_style(buttonText[i]);
            lv_obj_clear_protect(buttonText[i], LV_PROTECT_FOLLOW);
            lv_btn_set_layout(buttonText[i], LV_LAYOUT_OFF);
            //lv_obj_set_event_cb_mks(buttonText[i], event_handler,(i+10),"", 0);
            lv_obj_set_pos(buttonText[i], BTN_X_PIXEL * (i - 3) + INTERVAL_V * ((i - 3) + 1) + FILE_PRE_PIC_X_OFFSET, BTN_Y_PIXEL + INTERVAL_H + titleHeight + FILE_PRE_PIC_Y_OFFSET + 100);
            lv_obj_set_size(buttonText[i], 100, 40);
          }
          labelPageUp[i] = lv_label_create(buttonText[i], public_buf_m);
          lv_obj_align(labelPageUp[i], buttonText[i], LV_ALIGN_IN_BOTTOM_MID, 0, 0);
        }
        else {
          lv_obj_set_event_cb_mks(buttonGcode[i], event_handler, (i + 1), "", 0);
          lv_imgbtn_set_src_both(buttonGcode[i], "F:/bmp_file.bin");
          if (i < 3)
            lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * i + INTERVAL_V * (i + 1), titleHeight);
          else
            lv_obj_set_pos(buttonGcode[i], BTN_X_PIXEL * (i - 3) + INTERVAL_V * ((i - 3) + 1), BTN_Y_PIXEL + INTERVAL_H + titleHeight);

          labelPageUp[i] = lv_label_create(buttonGcode[i], public_buf_m);
          lv_obj_align(labelPageUp[i], buttonGcode[i], LV_ALIGN_IN_BOTTOM_MID, 0, -5);
        }
      }
      #if HAS_ROTARY_ENCODER
        if (gCfgItems.encoder_enable) lv_group_add_obj(g, buttonGcode[i]);
      #endif

    #else // !TFT35
    #endif // !TFT35
  }
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) {
      lv_group_add_obj(g, buttonPageUp);
      lv_group_add_obj(g, buttonPageDown);
      lv_group_add_obj(g, buttonBack);
    }
  #endif
}

uint32_t lv_open_gcode_file(char *path) {
  #if HAS_MEDIA
    uint32_t *ps4;
    uintptr_t pre_sread_cnt = UINTPTR_MAX;
    char *cur_name;

    cur_name = strrchr(path, '/');

    card.openFileRead(cur_name);
    card.read(public_buf, 512);
    ps4 = (uint32_t *)strstr((char *)public_buf, ";simage:");
    // Ignore the beginning message of G-code file
    if (ps4) {
      pre_sread_cnt = (uintptr_t)ps4 - (uintptr_t)((uint32_t *)(&public_buf[0]));
      card.setIndex(pre_sread_cnt);
    }
    return pre_sread_cnt;
  #endif // HAS_MEDIA
}

int ascii2dec_test(char *ascii) {
  int result = 0;
  if (ascii == 0) return 0;

  if (*(ascii) >= '0' && *(ascii) <= '9')
    result = *(ascii) - '0';
  else if (*(ascii) >= 'a' && *(ascii) <= 'f')
    result = *(ascii) - 'a' + 0x0A;
  else if (*(ascii) >= 'A' && *(ascii) <= 'F')
    result = *(ascii) - 'A' + 0x0A;
  else
    return 0;

  return result;
}

void lv_gcode_file_read(uint8_t *data_buf) {
  #if HAS_MEDIA
    uint16_t i = 0, j = 0, k = 0, row_1 = 0;
    bool ignore_start = true;
    char temp_test[200];
    volatile uint16_t *p_index;

    hal.watchdog_refresh();
    memset(public_buf, 0, 200);

    while (card.isFileOpen()) {
      if (ignore_start) card.read(temp_test, 8); // line start -> ignore
      card.read(temp_test, 200); // data
      // \r;;gimage: we got the bit img, so stop here
      if (temp_test[1] == ';') {
        card.closefile();
        break;
      }
      for (i = 0; i < 200;) {
        public_buf[row_1 * 200 + 100 * k + j] = (char)(ascii2dec_test(&temp_test[i]) << 4 | ascii2dec_test(&temp_test[i + 1]));
        j++;
        i += 2;
      }

      uint16_t c = card.get();
      // check for more data or end of line (CR or LF)
      if (ISEOL(c)) {
        c = card.get(); // more eol?
        if (!ISEOL(c)) card.setIndex(card.getIndex() - 1);
        break;
      }
      card.setIndex(card.getIndex() - 1);
      k++;
      j = 0;
      ignore_start = false;
      if (k > 1) {
        card.closefile();
        break;
      }
    }
    for (i = 0; i < 200;) {
      p_index = (uint16_t *)(&public_buf[i]);
      //Color = (*p_index >> 8);
      //*p_index = Color | ((*p_index & 0xFF) << 8);
      i += 2;
      if (*p_index == 0x0000) *p_index = LV_COLOR_BACKGROUND.full;
    }
    memcpy(data_buf, public_buf, 200);
  #endif // HAS_MEDIA
}

void lv_close_gcode_file() {TERN_(HAS_MEDIA, card.closefile());}

void lv_gcode_file_seek(uint32_t pos) {
  card.setIndex(pos);
}

void cutFileName(char *path, int len, int bytePerLine, char *outStr) {
  #if _LFN_UNICODE
    TCHAR *tmpFile;
    TCHAR *strIndex1 = 0, *strIndex2 = 0, *beginIndex;
    TCHAR secSeg[10]   = {0};
    TCHAR gFileTail[4] = {'~', '.', 'g', '\0'};
  #else
    char *tmpFile;
    char *strIndex1 = 0, *strIndex2 = 0, *beginIndex;
    char secSeg[10] = {0};
  #endif

  if (path == 0 || len <= 3 || outStr == 0) return;

  tmpFile = path;
  #if _LFN_UNICODE
    strIndex1 = (WCHAR *)wcsstr((const WCHAR *)tmpFile, (const WCHAR *)'/');
    strIndex2 = (WCHAR *)wcsstr((const WCHAR *)tmpFile, (const WCHAR *)'.');
  #else
    strIndex1 = (char *)strrchr(tmpFile, '/');
    strIndex2 = (char *)strrchr(tmpFile, '.');
  #endif

  beginIndex = (strIndex1 != 0
                //&& (strIndex2 != 0) && (strIndex1 < strIndex2)
                ) ? strIndex1 + 1 : tmpFile;

  if (strIndex2 == 0 || (strIndex1 > strIndex2)) { // not G-code file
    #if _LFN_UNICODE
      if (wcslen(beginIndex) > len)
        wcsncpy(outStr, beginIndex, len);
      else
        wcscpy(outStr, beginIndex);
    #else
      if ((int)strlen(beginIndex) > len)
        strlcpy(outStr, beginIndex, len + 1);
      else
        strcpy(outStr, beginIndex);
    #endif
  }
  else { // G-code file
    if (strIndex2 - beginIndex > (len - 2)) {
      #if _LFN_UNICODE
        wcsncpy(outStr, (const WCHAR *)beginIndex, len - 3);
        wcscat(outStr, (const WCHAR *)gFileTail);
      #else
        strlcpy(outStr, beginIndex, len - 3);
        strcat_P(outStr, PSTR("~.g"));
      #endif
    }
    else {
      const size_t strsize = strIndex2 - beginIndex + 1;
      #if _LFN_UNICODE
        wcsncpy(outStr, (const WCHAR *)beginIndex, strsize);
        wcscat(outStr, (const WCHAR *)&gFileTail[3]);
      #else
        strlcpy(outStr, beginIndex, strsize + 1);
        strcat_P(outStr, PSTR("g"));
      #endif
    }
  }

  #if _LFN_UNICODE
    if (wcslen(outStr) > bytePerLine) {
      wcscpy(secSeg, (const WCHAR *)&outStr[bytePerLine]);
      outStr[bytePerLine]     = '\n';
      outStr[bytePerLine + 1] = '\0';
      wcscat(outStr, (const WCHAR *)secSeg);
    }
  #else
    if ((int)strlen(outStr) > bytePerLine) {
      strcpy(secSeg, &outStr[bytePerLine]);
      outStr[bytePerLine]     = '\n';
      outStr[bytePerLine + 1] = '\0';
      strcat(outStr, secSeg);
    }
    else {
      strcat_P(outStr, PSTR("\n"));
    }
  #endif
}

void lv_clear_print_file() {
  #if HAS_ROTARY_ENCODER
    if (gCfgItems.encoder_enable) lv_group_remove_all_objs(g);
  #endif
  lv_obj_del(scr);
}

#endif // HAS_TFT_LVGL_UI
