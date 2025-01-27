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
#pragma once

/**
 * cardreader.h - SD card / USB flash drive file handling interface
 */

#include "../inc/MarlinConfig.h"

#if HAS_MEDIA

extern const char M23_STR[], M24_STR[];

#if ENABLED(SDCARD_SORT_ALPHA)
  #if ENABLED(SDSORT_DYNAMIC_RAM)
    #define SD_RESORT 1
  #endif
  #ifndef SDSORT_FOLDERS
    #define SDSORT_FOLDERS 0
  #endif
  #if SDSORT_FOLDERS || ENABLED(SDSORT_GCODE)
    #define HAS_FOLDER_SORTING 1
  #endif
#endif

#define MAX_DIR_DEPTH     10       // Maximum folder depth
#define MAXDIRNAMELENGTH   8       // DOS folder name size
#define MAXPATHNAMELENGTH  (1 + (MAXDIRNAMELENGTH + 1) * (MAX_DIR_DEPTH) + 1 + FILENAME_LENGTH) // "/" + N * ("ADIRNAME/") + "filename.ext"

#include "SdFile.h"
#include "disk_io_driver.h"

#if HAS_USB_FLASH_DRIVE
  #include "usb_flashdrive/Sd2Card_FlashDrive.h"
#endif

#if NEED_SD2CARD_SDIO
  #include "Sd2Card_sdio.h"
#elif NEED_SD2CARD_SPI
  #include "Sd2Card.h"
#endif

#if HAS_MULTI_VOLUME
  #define SV_SD_ONBOARD      1
  #define SV_USB_FLASH_DRIVE 2
  #define _VOLUME_ID(N) _CAT(SV_, N)
  #define SHARED_VOLUME_IS(N) (DEFAULT_SHARED_VOLUME == _VOLUME_ID(N))
  #if !SHARED_VOLUME_IS(SD_ONBOARD) && !SHARED_VOLUME_IS(USB_FLASH_DRIVE)
    #error "DEFAULT_SHARED_VOLUME must be either SD_ONBOARD or USB_FLASH_DRIVE."
  #endif
#else
  #define SHARED_VOLUME_IS(...) 0
#endif

typedef struct {
  bool saving:1,                // Receiving a G-code file or logging commands during a print
       logging:1,               // Log enqueued commands to the open file. See GCodeQueue::advance()
       sdprinting:1,            // Actively printing from the open file
       sdprintdone:1,           // The active job has reached the end, 100%
       mounted:1,               // The card or flash drive is mounted and ready to read/write
       filenameIsDir:1,         // The working item is a directory
       workDirIsRoot:1,         // The working directory is / so there's no parent
       abort_sd_printing:1      // Abort by calling abortSDPrinting() at the main loop()
       #if DO_LIST_BIN_FILES
         , filenameIsBin:1      // The working item is a BIN file
       #endif
       #if ENABLED(BINARY_FILE_TRANSFER)
         , binary_mode:1        // Use the serial line buffer as BinaryStream input
       #endif
    ;
} card_flags_t;

enum ListingFlags : uint8_t { LS_LONG_FILENAME, LS_ONLY_BIN, LS_TIMESTAMP };
enum SortFlag : int8_t { AS_REV = -1, AS_OFF, AS_FWD, AS_ALSO_REV };

#if ENABLED(AUTO_REPORT_SD_STATUS)
  #include "../libs/autoreport.h"
#endif

class CardReader {
public:
  static card_flags_t flag;                         // Flags (above)
  static char filename[FILENAME_LENGTH],            // DOS 8.3 filename of the selected item
              longFilename[LONG_FILENAME_LENGTH];   // Long name of the selected item

  // Fast! binary file transfer
  #if ENABLED(BINARY_FILE_TRANSFER)
    #if HAS_MULTI_SERIAL
      static serial_index_t transfer_port_index;
    #else
      static constexpr serial_index_t transfer_port_index = 0;
    #endif
  #endif

  CardReader();

  static void changeMedia(DiskIODriver *_driver) { driver = _driver; }

  static MediaFile getroot() { return root; }

  static void mount();
  static void release();
  static bool isMounted() { return flag.mounted; }

  // Handle media insert/remove
  static void manage_media();

  // SD Card Logging
  static void openLogFile(const char * const path);
  static void write_command(char * const buf);

  #if DISABLED(NO_SD_AUTOSTART)     // Auto-Start auto#.g file handling
    static uint8_t autofile_index;  // Next auto#.g index to run, plus one. Ignored by autofile_check when zero.
    static void autofile_begin();   // Begin check. Called automatically after boot-up.
    static bool autofile_check();   // Check for the next auto-start file and run it.
    static void autofile_cancel() { autofile_index = 0; }
  #endif

  #if ENABLED(ONE_CLICK_PRINT)
    static bool one_click_check();  // Check for the newest file and prompt to run it.
    static void diveToNewestFile(MediaFile parent, uint32_t &compareDateTime, MediaFile &outdir, char * const outname);
    static bool selectNewestFile();
  #endif

  // Basic file ops
  static void openFileRead(const char * const path, const uint8_t subcall=0);
  static void openFileWrite(const char * const path);
  static void closefile(const bool store_location=false);
  static bool fileExists(const char * const name);
  static void removeFile(const char * const name);

  static char* longest_filename() { return longFilename[0] ? longFilename : filename; }
  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    static void printLongPath(char * const path);   // Used by M33
    static void getLongPath(char * const pathLong, char * const pathShort); // Used by anycubic_vyper
  #endif

  // Working Directory for SD card menu
  static void cdroot();
  static void cd(const char *relpath);
  static int8_t cdup();
  static int16_t get_num_items();

  // Select a file
  static void selectFileByIndex(const int16_t nr);
  static void selectFileByName(const char * const match);  // (working directory only)

  // Print job
  static void report_status(TERN_(QUIETER_AUTO_REPORT_SD_STATUS, const bool isauto=false));
  static void getAbsFilenameInCWD(char *dst);
  static void printSelectedFilename();
  static void openAndPrintFile(const char *name);   // (working directory or full path)
  static void startOrResumeFilePrinting();
  static void endFilePrintNow(TERN_(SD_RESORT, const bool re_sort=false));
  static void abortFilePrintNow(TERN_(SD_RESORT, const bool re_sort=false));
  static void fileHasFinished();
  static void abortFilePrintSoon() { flag.abort_sd_printing = isFileOpen(); }
  static void pauseSDPrint()       { flag.sdprinting = false; }
  static bool isPrinting()         { return flag.sdprinting; }
  static bool isStillPrinting()    { return flag.sdprinting && !flag.abort_sd_printing; }
  static bool isPaused()           { return isFileOpen() && !isPrinting(); }
  #if HAS_PRINT_PROGRESS_PERMYRIAD
    static uint16_t permyriadDone() {
      if (flag.sdprintdone) return 10000;
      if (isFileOpen() && filesize) return sdpos / ((filesize + 9999) / 10000);
      return 0;
    }
  #endif
  static uint8_t percentDone() {
    if (flag.sdprintdone) return 100;
    if (isFileOpen() && filesize) return sdpos / ((filesize + 99) / 100);
    return 0;
  }

  /**
   * Dive down to a relative or absolute path.
   * Relative paths apply to the workDir.
   *
   * update_cwd: Pass 'true' to update the workDir on success.
   *   inDirPtr: On exit your pointer points to the target MediaFile.
   *             A nullptr indicates failure.
   *       path: Start with '/' for abs path. End with '/' to get a folder ref.
   *       echo: Set 'true' to print the path throughout the loop.
   */
  static const char* diveToFile(const bool update_cwd, MediaFile* &inDirPtr, const char * const path, const bool echo=false);

  #if ENABLED(SDCARD_SORT_ALPHA)
    static void presort();
    static void selectFileByIndexSorted(const int16_t nr);
    #if ENABLED(SDSORT_GCODE)
      FORCE_INLINE static void setSortOn(const SortFlag f) { sort_alpha = (f == AS_ALSO_REV) ? AS_REV : f; presort(); }
      FORCE_INLINE static void setSortFolders(const int8_t i) { sort_folders = i; presort(); }
      //FORCE_INLINE static void setSortReverse(bool b) { sort_reverse = b; }
    #endif
  #else
    FORCE_INLINE static void selectFileByIndexSorted(const int16_t nr) {
      selectFileByIndex(TERN(SDCARD_RATHERRECENTFIRST, get_num_items() - 1 - nr, (nr)));
    }
  #endif

  static void ls(const uint8_t lsflags=0);

  #if ENABLED(POWER_LOSS_RECOVERY)
    static bool jobRecoverFileExists();
    static void openJobRecoveryFile(const bool read);
    static void removeJobRecoveryFile();
  #endif

  // Binary flag for the current file
  static bool fileIsBinary() { return TERN0(DO_LIST_BIN_FILES, flag.filenameIsBin); }
  static void setBinFlag(const bool bin) { TERN(DO_LIST_BIN_FILES, flag.filenameIsBin = bin, UNUSED(bin)); }

  // Current Working Dir - Set by cd, cdup, cdroot, and diveToFile(true, ...)
  static char* getWorkDirName()  { workDir.getDosName(filename); return filename; }
  static MediaFile& getWorkDir()    { return workDir.isOpen() ? workDir : root; }

  // Print File stats
  static uint32_t getFileSize()  { return filesize; }
  static uint32_t getIndex()     { return sdpos; }
  static bool isFileOpen()       { return isMounted() && file.isOpen(); }
  static bool eof()              { return getIndex() >= getFileSize(); }

  // File data operations
  static int16_t get()                            { int16_t out = (int16_t)file.read(); sdpos = file.curPosition(); return out; }
  static int16_t read(void *buf, uint16_t nbyte)  { return file.isOpen() ? file.read(buf, nbyte) : -1; }
  static int16_t write(void *buf, uint16_t nbyte) { return file.isOpen() ? file.write(buf, nbyte) : -1; }
  static void setIndex(const uint32_t index)      { file.seekSet((sdpos = index)); }

  // TODO: rename to diskIODriver()
  static DiskIODriver* diskIODriver() { return driver; }

  #if ENABLED(AUTO_REPORT_SD_STATUS)
    //
    // SD Auto Reporting
    //
    struct AutoReportSD { static void report() { report_status(TERN_(QUIETER_AUTO_REPORT_SD_STATUS, true)); } };
    static AutoReporter<AutoReportSD> auto_reporter;
  #endif

  #if SHARED_VOLUME_IS(USB_FLASH_DRIVE) || HAS_USB_FLASH_DRIVE
    #define HAS_USB_FLASH_DRIVE 1
    static DiskIODriver_USBFlash media_driver_usbFlash;
  #endif

  #if NEED_SD2CARD_SDIO || NEED_SD2CARD_SPI
    typedef TERN(NEED_SD2CARD_SDIO, DiskIODriver_SDIO, DiskIODriver_SPI_SD) sdcard_driver_t;
    static sdcard_driver_t media_driver_sdcard;
  #endif

private:
  //
  // Working directory and parents
  //
  static MediaFile root, workDir, workDirParents[MAX_DIR_DEPTH];
  static uint8_t workDirDepth;
  static int16_t nrItems; // Cache the total count

  //
  // Alphabetical file and folder sorting
  //
  #if ENABLED(SDCARD_SORT_ALPHA)
    static int16_t sort_count;    // Count of sorted items in the current directory
    #if ENABLED(SDSORT_GCODE)
      static SortFlag sort_alpha; // Sorting: REV, OFF, FWD
      static int8_t sort_folders; // Folder sorting before/none/after
      //static bool sort_reverse; // Flag to enable / disable reverse sorting
    #endif

    // By default the sort index is statically allocated
    #if ENABLED(SDSORT_DYNAMIC_RAM)
      static uint8_t *sort_order;
    #else
      static uint8_t sort_order[SDSORT_LIMIT];
    #endif

    #if ALL(SDSORT_USES_RAM, SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)
      #define SORTED_LONGNAME_MAXLEN (SDSORT_CACHE_VFATS) * (FILENAME_LENGTH)
      #define SORTED_LONGNAME_STORAGE (SORTED_LONGNAME_MAXLEN + 1)
    #else
      #define SORTED_LONGNAME_MAXLEN LONG_FILENAME_LENGTH
      #define SORTED_LONGNAME_STORAGE SORTED_LONGNAME_MAXLEN
    #endif

    // Cache filenames to speed up SD menus.
    #if ENABLED(SDSORT_USES_RAM)

      // If using dynamic ram for names, allocate on the heap.
      #if ENABLED(SDSORT_CACHE_NAMES)
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          static char **sortshort, **sortnames;
        #else
          static char sortshort[SDSORT_LIMIT][FILENAME_LENGTH];
        #endif
      #endif

      #if (ENABLED(SDSORT_CACHE_NAMES) && DISABLED(SDSORT_DYNAMIC_RAM)) || NONE(SDSORT_CACHE_NAMES, SDSORT_USES_STACK)
        static char sortnames[SDSORT_LIMIT][SORTED_LONGNAME_STORAGE];
      #endif

      // Folder sorting uses an isDir array when caching items.
      #if HAS_FOLDER_SORTING
        #if ENABLED(SDSORT_DYNAMIC_RAM)
          static uint8_t *isDir;
        #elif ENABLED(SDSORT_CACHE_NAMES) || DISABLED(SDSORT_USES_STACK)
          static uint8_t isDir[(SDSORT_LIMIT + 7) >> 3];
        #endif
      #endif

    #endif // SDSORT_USES_RAM

  #endif // SDCARD_SORT_ALPHA

  static DiskIODriver *driver;
  static MarlinVolume volume;
  static MediaFile file;

  static uint32_t filesize, // Total size of the current file, in bytes
                  sdpos;    // Index most recently read (one behind file.getPos)

  //
  // Procedure calls to other files
  //
  #if HAS_MEDIA_SUBCALLS
    static uint8_t file_subcall_ctr;
    static uint32_t filespos[SD_PROCEDURE_DEPTH];
    static char proc_filenames[SD_PROCEDURE_DEPTH][MAXPATHNAMELENGTH];
  #endif

  //
  // Directory items
  //
  static bool is_visible_entity(const dir_t &p OPTARG(CUSTOM_FIRMWARE_UPLOAD, const bool onlyBin=false));
  static int16_t countVisibleItems(MediaFile dir);
  static void selectByIndex(MediaFile dir, const int16_t index);
  static void selectByName(MediaFile dir, const char * const match);
  static void printListing(
    MediaFile parent, const char * const prepend, const uint8_t lsflags
    OPTARG(LONG_FILENAME_HOST_SUPPORT, const char * const prependLong=nullptr)
  );

  #if ENABLED(SDCARD_SORT_ALPHA)
    static void flush_presort();
  #endif
};

#if HAS_USB_FLASH_DRIVE
  #define IS_SD_INSERTED() DiskIODriver_USBFlash::isInserted()
#elif HAS_SD_DETECT
  #define IS_SD_INSERTED() (READ(SD_DETECT_PIN) == SD_DETECT_STATE)
#else
  // No card detect line? Assume the card is inserted.
  #define IS_SD_INSERTED() true
#endif

#define IS_SD_MOUNTED()   card.isMounted()
#define IS_SD_PRINTING()  card.isStillPrinting()
#define IS_SD_FETCHING()  (!card.flag.sdprintdone && card.isStillPrinting())
#define IS_SD_PAUSED()    card.isPaused()
#define IS_SD_FILE_OPEN() card.isFileOpen()

extern CardReader card;

#else // !HAS_MEDIA

#define IS_SD_MOUNTED()   false
#define IS_SD_PRINTING()  false
#define IS_SD_FETCHING()  false
#define IS_SD_PAUSED()    false
#define IS_SD_FILE_OPEN() false

#define LONG_FILENAME_LENGTH 0

#endif // !HAS_MEDIA
