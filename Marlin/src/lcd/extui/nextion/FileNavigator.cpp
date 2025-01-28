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

/* ****************************************
 * lcd/extui/nextion/FileNavigator.cpp
 * ****************************************
 * Extensible_UI implementation for Nextion
 * https://github.com/Skorpi08
 * ***************************************/

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(NEXTION_TFT)

#include "FileNavigator.h"
#include "nextion_tft.h"

using namespace ExtUI;

#define DEBUG_OUT NEXDEBUGLEVEL
#include "../../../core/debug_out.h"

FileList  FileNavigator::filelist;                      // ExtUI file API
char      FileNavigator::currentDirPath[MAX_PATH_LEN];  // Current folder path
uint16_t  FileNavigator::lastindex;
uint8_t   FileNavigator::folderdepth;
uint16_t  FileNavigator::currentindex;                  // override the panel request

FileNavigator filenavigator;

FileNavigator::FileNavigator() { reset(); }

void FileNavigator::reset() {
  currentDirPath[0] = '\0';
  folderdepth  = 0;
  currentindex = 0;
  lastindex    = 0;
  // Start at root folder
  while (!filelist.isAtRootDir()) filelist.upDir();
  refresh();
}

void FileNavigator::refresh() { filelist.refresh(); }

void FileNavigator::getFiles(uint16_t index) {
  uint16_t files = 7, fseek = 0, fcnt  = 0;
  if (index == 0)
    currentindex = 0;
  else {
    // Each time we change folder we reset the file index to 0 and keep track
    // of the current position as the TFT panel isn't aware of folder trees.
    --currentindex; // go back a file to take account of the .. added to the root.
    if (index > lastindex)
      currentindex += files + 1;
    else if (currentindex >= files)
      currentindex -= files - 1;
    else
      currentindex = 0;
  }
  lastindex = index;

  #if NEXDEBUG(AC_FILE)
    DEBUG_ECHOLNPGM("index=", index, " currentindex=", currentindex);
  #endif

  if (currentindex == 0 && folderdepth > 0) { // Add a link to go up a folder
    nextion.tftSend(F("vis p0,1"));
    nextion.tftSend(F("\xFF\xFF\xFF"));
    SEND_VAL("tmpUP", "0");
    files--;
  }
  else {
    nextion.tftSend(F("vis p0,0"));
    nextion.tftSend(F("\xFF\xFF\xFF"));
  }

  for (uint16_t seek = currentindex; seek < currentindex + files; seek++) {
    if (filelist.seek(seek)) {
      nextion.tftSend(F("s"));
      LCD_SERIAL.print(fcnt);
      nextion.tftSend(F(".txt=\""));
      if (filelist.isDir()) {
        LCD_SERIAL.print(filelist.shortFilename());
        nextion.tftSend(F("/\""));
        nextion.tftSend(F("\xFF\xFF\xFF"));

        nextion.tftSend(F("l"));
        LCD_SERIAL.print(fcnt);
        nextion.tftSend(F(".txt=\""));
        LCD_SERIAL.print(filelist.filename());
        nextion.tftSend(F("\""));
        nextion.tftSend(F("\xFF\xFF\xFF"));
        SEND_PCO2("l", fcnt, "1055");
      }
      else {
        LCD_SERIAL.print(currentDirPath);
        LCD_SERIAL.print(filelist.shortFilename());
        nextion.tftSend(F("\""));
        nextion.tftSend(F("\xFF\xFF\xFF"));

        nextion.tftSend(F("l"));
        LCD_SERIAL.print(fcnt);
        nextion.tftSend(F(".txt=\""));
        LCD_SERIAL.print(filelist.longFilename());
        nextion.tftSend(F("\""));
        nextion.tftSend(F("\xFF\xFF\xFF"));
      }
      fcnt++;
      fseek = seek;
      #if NEXDEBUG(AC_FILE)
        DEBUG_ECHOLNPGM("-", seek, " '", filelist.longFilename(), "' '", currentDirPath, "", filelist.shortFilename(), "'\n");
      #endif
    }
  }
  SEND_VAL("n0", filelist.count());
  SEND_VAL("n1", fseek + 1);
}

void FileNavigator::changeDIR(char *folder) {
  #if NEXDEBUG(AC_FILE)
    DEBUG_ECHOLNPGM("currentfolder: ", currentDirPath, "  New: ", folder);
  #endif
  if (folderdepth >= MAX_FOLDER_DEPTH) return; // limit the folder depth
  strcat(currentDirPath, folder);
  strcat(currentDirPath, "/");
  filelist.changeDir(folder);
  refresh();
  folderdepth++;
  currentindex = 0;
}

void FileNavigator::upDIR() {
  filelist.upDir();
  refresh();
  folderdepth--;
  currentindex = 0;
  // Remove the last child folder from the stored path
  if (folderdepth == 0) {
    currentDirPath[0] = '\0';
    reset();
  }
  else {
    char *pos = nullptr;
    for (uint_fast8_t f = 0; f < folderdepth; f++)
      pos = strchr(currentDirPath, '/');
    pos[1] = '\0';
  }
  #if NEXDEBUG(AC_FILE)
    DEBUG_ECHOLNPGM("depth: ", folderdepth, " currentDirPath: ", currentDirPath);
  #endif
}

char* FileNavigator::getCurrentDirPath() { return currentDirPath; }

#endif // NEXTION_TFT
