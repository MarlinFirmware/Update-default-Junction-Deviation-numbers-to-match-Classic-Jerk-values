/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include <inttypes.h>
#include "../../../sd/SdFatConfig.h"

#define DGUS_E3S1PRO_BASIC_SDCARD_MAX_FILES      20
#define DGUS_E3S1PRO_BASIC_SDCARD_FILES_PER_PAGE  5

#define DGUS_PAGE_TO_SCREEN(page)   (DGUS_ScreenID)((uint8_t)DGUS_ScreenID::FILE1 + ((uint8_t)page - (uint8_t)DGUS_SDCardHandler::page_t::PAGE_1))
#define DGUS_SCREEN_TO_PAGE(screen) (DGUS_SDCardHandler::page_t)((uint8_t)DGUS_SDCardHandler::page_t::PAGE_1 + ((uint8_t)screen - (uint8_t)DGUS_ScreenID::FILE1))

#define DGUS_PAGE_TO_INDEX(page)    (uint8_t)((uint8_t)page - (uint8_t)DGUS_SDCardHandler::page_t::PAGE_1)
#define DGUS_FILE_TO_INDEX(file)    (uint8_t)((uint8_t)file - (uint8_t)DGUS_SDCardHandler::file_t::FILE_1)

#define DGUS_PAGE_FROM_INDEX(index) (DGUS_SDCardHandler::page_t)(index + (uint8_t)DGUS_SDCardHandler::page_t::PAGE_1)
#define DGUS_FILE_FROM_INDEX(index) (DGUS_SDCardHandler::file_t)(index + (uint8_t)DGUS_SDCardHandler::file_t::FILE_1)

class DGUS_SDCardHandler {
  public:
    enum class page_t : uint8_t {
      PAGE_1 = 1,
      PAGE_2,
      PAGE_3,
      PAGE_4
    };

    enum class file_t : uint8_t {
      FILE_1 = 1,
      FILE_2,
      FILE_3,
      FILE_4,
      FILE_5,
      FILE_Max = FILE_5,
      FILE_Count
    };

  static char filenames[5][LONG_FILENAME_LENGTH];

public:
  static void Reset();

  static void onPageLoad(page_t page);
  static bool onFileSelect(file_t file);

  static page_t onFirstPage();
  static page_t onLastPage();
  static page_t onPreviousPage();
  static page_t onNextPage();

private:
  static void setFilename(file_t file, const char* filename, bool isDir = false) PROGMEM;
};

extern DGUS_SDCardHandler dgus_sdcard_handler;
