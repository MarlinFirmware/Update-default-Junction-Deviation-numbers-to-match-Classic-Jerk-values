/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * lcd/extui/ia_creality/FileNavigator.cpp
 * ****************************************
 * Extensible_UI implementation for Creality DWIN
 * 10SPro, Max, CR10V2
 * Based on implementations for Anycubic Chiron and Nextion by Nick Wells and Skorpi08
 * Written by Insanity Automation
 * ***************************************/

#include "../../../inc/MarlinConfigPre.h"

#if DGUS_LCD_UI_IA_CREALITY

#include "ia_creality_rts.h"
#include "FileNavigator.h"

#include <WString.h>

ExtUI::FileList FileNavigator::filelist;                // ExtUI file API
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

bool FileNavigator::getIndexisDir(uint16_t index){
  filelist.seek(index);
  return filelist.isDir();
}

const char *FileNavigator::getIndexName(uint16_t index){
  filelist.seek(index);
  return filelist.shortFilename();
}

uint16_t FileNavigator::maxFiles() {
  return filelist.count();
}

void FileNavigator::getFiles(uint16_t index) {
  uint16_t files = DISPLAY_FILES, fcnt  = 0;
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

  // Clear currently drawn screen
  for (int i = 0; i < DISPLAY_FILES; i++) {
    for (int j = 0; j < 20; j++)
      rts.sendData(0, SDFILE_ADDR + (i * 20) + j);
  }

  for (int j = 0; j < 10; j++) {
    rts.sendData(0, Printfilename + j);  // clear screen.
    rts.sendData(0, Choosefilename + j); // clear filename
  }
  for (int j = 0; j < 8; j++)
    rts.sendData(0, FilenameCount + j);
  for (int j = 1; j <= DISPLAY_FILES; j++) {
    rts.sendData(10, FilenameIcon + j);
    rts.sendData(10, FilenameIcon1 + j);
  }

  if (currentindex == 0 && folderdepth > 0) { // Add a link to go up a folder
    files--;
    rts.sendData("Up Directory", SDFILE_ADDR);
    fcnt++;
  }
  else if (currentindex == DISPLAY_FILES && folderdepth > 0)
    currentindex--;

  for (uint16_t seek = currentindex; seek < currentindex + files; seek++) {
    if (filelist.seek(seek)) {
      const int filelen = strlen(filelist.filename());
      if (filelen > 20) {
        char *buf = (char *)filelist.filename();
        buf[18] = '\0'; // cutoff at screen edge
        rts.sendData(buf, (SDFILE_ADDR + (fcnt * 20)));
      }
      else
        rts.sendData(filelist.filename(), (SDFILE_ADDR + (fcnt * 20)));

      if (filelist.isDir()) {
        rts.sendData((uint8_t)4, FilenameIcon + (fcnt+1));
        rts.sendData((unsigned long)0x041F, (FilenameNature + ((1+fcnt) * 16))); // Change BG of selected line to Blue
      }
      else {
        rts.sendData((uint8_t)0, FilenameIcon + (fcnt+1));
        rts.sendData((unsigned long)0xFFFF, (FilenameNature + ((1+fcnt) * 16))); // white
      }
      SERIAL_ECHOLNPGM("-", seek, " '", filelist.filename(), "' '", currentDirPath, "", filelist.shortFilename(), "'\n");
      fcnt++;
    }
  }
}

void FileNavigator::changeDIR(char *folder) {
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
}

char* FileNavigator::getCurrentDirPath() { return currentDirPath; }

#endif // DGUS_LCD_UI_IA_CREALITY
