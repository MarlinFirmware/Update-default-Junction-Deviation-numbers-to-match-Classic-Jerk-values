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
#include "../../../sd/SdFatConfig.h"

#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

typedef struct {
  int cur_page_first_offset;
  int cur_page_last_offset;
  int curPage;
} DIR_OFFSET;
extern DIR_OFFSET dir_offset[10];

#define FILE_NUM 6

#define NAME_CUT_LEN 23

#define MAX_DIR_LEVEL  10

typedef struct {
  char file_name[FILE_NUM][FILENAME_LENGTH * MAX_DIR_LEVEL + 1];
  char long_name[FILE_NUM][TERN(LONG_FILENAME_WRITE_SUPPORT, LONG_FILENAME_LENGTH, FILENAME_LENGTH * 2) + 1];
  char curDirPath[FILENAME_LENGTH * MAX_DIR_LEVEL + 1];
  bool IsFolder[FILE_NUM];
  char Sd_file_cnt;
  char sd_file_index;
  char Sd_file_offset;
} LIST_FILE;
extern LIST_FILE list_file;

void disp_gcode_icon(uint8_t file_num);
void lv_draw_print_file();
uint32_t lv_open_gcode_file(char *path);
void lv_gcode_file_read(uint8_t *data_buf);
void lv_close_gcode_file();
void cutFileName(char *path, int len, int bytePerLine, char *outStr);
int ascii2dec_test(char *ascii);
void lv_clear_print_file();
void lv_gcode_file_seek(uint32_t pos);

#ifdef __cplusplus
  } /* C-declarations for C++ */
#endif
