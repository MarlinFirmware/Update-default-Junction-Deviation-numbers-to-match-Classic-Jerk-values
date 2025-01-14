/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * mmu2_reporting.h
 */

#include "../../MarlinCore.h"

#include <stdint.h>
#include "mmu_hw/error_codes.h"
#include "mmu_hw/progress_codes.h"

namespace MMU3 {

  enum CommandInProgress : uint8_t {
    NoCommand = 0,
    CutFilament = 'K',
    EjectFilament = 'E',
    Homing = 'H',
    LoadFilament = 'L',
    Reset = 'X',
    ToolChange = 'T',
    UnloadFilament = 'U',
  };

  /**
   * Data class for MMU operation statistics.
   *
   * This is used to load/save data from/to EEPROM.
   * The data is initialized by the settings.load() method.
  */
  class OperationStatistics {
    public:
    void increment_load_fails();
    void increment_mmu_fails();
    void increment_tool_change_counter();
    bool reset_per_print_stats(); // Reset only the per print stats.
    bool reset_fail_stats(); // Reset only fail stats and keep tool change counters
    bool reset_stats(); // Reset MMU stats and update EEPROM

    static uint16_t fail_total_num;               // total failures
    static uint8_t fail_num;                      // fails during print
    static uint16_t load_fail_total_num;          // total load failures
    static uint8_t load_fail_num;                 // load failures during print
    static uint16_t tool_change_counter;          // number of tool changes during print
    static uint32_t tool_change_total_counter;    // number of total tool changes
    static int fail_total_num_addr;               // total failures EEPROM addr
    static int fail_num_addr;                     // fails during print EEPROM addr
    static int load_fail_total_num_addr;          // total load failures EEPROM addr
    static int load_fail_num_addr;                // load failures during print EEPROM addr
    static int tool_change_counter_addr;          // number of tool changes EEPROM addr
    static int tool_change_total_counter_addr;    // number of total tool changes EEPROM addr
  };

  extern OperationStatistics operation_statistics;

  // Called at the begin of every MMU operation
  void BeginReport(CommandInProgress cip, ProgressCode ec);

  // Called at the end of every MMU operation
  void EndReport(CommandInProgress cip, ProgressCode ec);

  // Checks for error screen user input, if the error screen is open
  void CheckErrorScreenUserInput();

  // Return true if the error screen is sleeping in the background
  // Error screen sleeps when the firmware is rendering complementary
  // UI to resolve the error screen, for example tuning Idler Stallguard Threshold
  bool TuneMenuEntered();

  // @brief Called when the MMU or MK3S sends operation error (even repeatedly).
  // Render MMU error screen on the LCD. This must be non-blocking
  // and allow the MMU and printer to communicate with each other.
  // @param[in] ec error code
  // @param[in] es error source
  void ReportErrorHook(CommandInProgress cip, ErrorCode ec, uint8_t es);

  // Called when the MMU sends operation progress update
  void ReportProgressHook(CommandInProgress cip, ProgressCode ec);

  struct TryLoadUnloadReporter {
    TryLoadUnloadReporter(float delta_mm);
    ~TryLoadUnloadReporter();
    void Progress(bool sensorState);
    void DumpToSerial();

    private:
    // @brief Add one block to the progress bar
    // @param col pixel position on the LCD status line, should range from 0 to (LCD_WIDTH - 1)
    // @param sensorState if true, filament is not present, else filament is present. This controls which character to render
    void Render(uint8_t col, bool sensorState);

    uint8_t dpixel0, dpixel1;
    uint8_t lcd_cursor_col;
    // The total length is twice delta_mm. Divide that length by number of pixels
    // available to get length per pixel.
    // Note: Below is the reciprocal of (2 * delta_mm) / LCD_WIDTH [mm/pixel]
    float pixel_per_mm;
  };

  // Remders the sensor status line. Also used by the "resume temperature" screen.
  void ReportErrorHookDynamicRender();

  // Renders the static part of the sensor state line. Also used by "resuming temperature screen"
  void ReportErrorHookSensorLineRender();

  // @return true if the MMU is communicating and available. Can change at runtime.
  //bool MMUAvailable();

  // Global Enable/Disable use MMU (to be stored in EEPROM)
  //bool UseMMU();

  // Disable MMU in EEPROM
  //void DisableMMUInSettings();

  // Increments EEPROM cell - number of failed loads into the nozzle
  // Note: technically, this is not an MMU error but an error of the printer.
  void IncrementLoadFails();

  // Increments EEPROM cell - number of MMU errors
  void IncrementMMUFails();

  // @return true when Cutter is enabled in the menus
  bool cutter_enabled();

  // Beware: enum values intentionally chosen to match the 8bit FW to save code size
  enum SoundType {
    Prompt = 2,
    Confirm = 3
  };

  void MakeSound(SoundType s);

  void fullScreenMsgCut(uint8_t slot);
  void fullScreenMsgEject(uint8_t slot);
  void fullScreenMsgTest(uint8_t slot);
  void fullScreenMsgLoad(uint8_t slot);
  void fullScreenMsgRestoringTemperature();

  void ScreenUpdateEnable();
  void ScreenClear();

  void tuneIdlerStallguardThreshold();

  void write_register_and_return_to_status_menu(uint8_t address, uint8_t value);

} // MMU3
