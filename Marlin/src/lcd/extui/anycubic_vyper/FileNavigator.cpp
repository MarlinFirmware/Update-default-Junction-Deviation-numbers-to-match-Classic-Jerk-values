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

/**
 * lcd/extui/anycubic_vyper/FileNavigator.cpp
 */

/***************************************************************************
 * The AC panel wants files in block of 4 and can only display a flat list *
 * This library allows full folder traversal.                              *
 ***************************************************************************/

#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(ANYCUBIC_LCD_VYPER)

#include "FileNavigator.h"
#include "dgus_tft.h"

using namespace ExtUI;

namespace Anycubic {

  FileList  FileNavigator::filelist;                          // ExtUI file API
  char      FileNavigator::currentDirPath[MAX_PATH_LEN + 1];  // Current folder path
  uint16_t  FileNavigator::lastindex;
  uint8_t   FileNavigator::folderdepth;
  uint16_t  FileNavigator::currentindex;                      // override the panel request

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
    uint8_t files = 5;
    if (index == 0) currentindex = 0;

    // Each time we change folder we reset the file index to 0 and keep track
    // of the current position as the TFT panel isnt aware of folders trees.
    if (index > 0) {
      //--currentindex; // go back a file to take account off the .. we added to the root.
      if (index > lastindex)
        currentindex += files;
      else
        currentindex = currentindex < 5 ? 0 : currentindex - files;
    }
    lastindex = index;

    #if ACDEBUG(AC_FILE)
      SERIAL_ECHOLNPGM("index=", index, " currentindex=", currentindex, " lastindex=", lastindex);
    #endif

    uint8_t file_num = 0;
    for (uint16_t _seek = currentindex; _seek < currentindex + files; _seek++) {

      #if ACDEBUG(AC_FILE)
        SERIAL_ECHOLNPGM("_seek: ", _seek, " currentindex: ", currentindex, " files: ", files);
      #endif

      if (filelist.seek(_seek)) {
        //sendFile();

        DgusTFT::sendTxtToTFT(filelist.longFilename(), TXT_FILE_0 + file_num*0x30);

        #if ACDEBUG(AC_FILE)
          SERIAL_ECHOLNPGM("seek: ", _seek, " '", filelist.longFilename(), "' '", currentDirPath, "", filelist.shortFilename(), "'\n");
        #endif
      }
      else {
        #if ACDEBUG(AC_FILE)
          SERIAL_ECHOLNPGM("over seek: ", _seek);
        #endif

        DgusTFT::sendTxtToTFT("\0", TXT_FILE_0 + file_num*0x30);
      }

      file_num++;
    }
  }

  void FileNavigator::sendFile() {
    // Send the file and folder info to the panel.
    // This info will be returned when the file is selected.
    // Permitted special characters in file name: -_*#~
    // Panel can display 22 characters per line.
    if (!filelist.isDir())
      DgusTFT::sendTxtToTFT(filelist.longFilename(), TXT_FILE_0);
  }

  void FileNavigator::changeDIR(char *folder) {
    #if ACDEBUG(AC_FILE)
      SERIAL_ECHOLNPGM("currentfolder: ", currentDirPath, "  New: ", folder);
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

      *(pos + 1) = '\0';
    }
    #if ACDEBUG(AC_FILE)
      SERIAL_ECHOLNPGM("depth: ", folderdepth, " currentDirPath: ", currentDirPath);
    #endif
  }

  char* FileNavigator::getCurrentDirPath() { return currentDirPath; }

  uint16_t FileNavigator::getFileNum() { return filelist.count(); }
}

#endif // ANYCUBIC_LCD_VYPER
