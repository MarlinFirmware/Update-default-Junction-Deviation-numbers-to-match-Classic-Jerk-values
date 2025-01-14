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

#include "../../inc/MarlinConfigPre.h"

#if HAS_GRAPHICAL_TFT

#include "tft_image.h"
#include "ui_common.h"

const tImage NoLogo = { nullptr, 0, 0, NOCOLORS };

const tImage images[imgCount] = {
  TERN(SHOW_BOOTSCREEN, BOOTSCREEN_LOGO, NoLogo), // imgBootScreen
  HotEnd_64x64x4,                                 // imgHotEnd
  Bed_64x64x4,                                    // imgBed
  Bed_Heated_64x64x4,                             // imgBedHeated
  Chamber_64x64x4,                                // imgChamber
  Chamber_Heated_64x64x4,                         // imgChamberHeated
  Fan0_64x64x4,                                   // imgFanIdle
  Fan_Slow0_64x64x4,                              // imgFanSlow0
  Fan_Slow1_64x64x4,                              // imgFanSlow1
  Fan_Fast0_64x64x4,                              // imgFanFast0
  Fan_Fast1_64x64x4,                              // imgFanFast1
  Feedrate_32x32x4,                               // imgFeedRate
  Flowrate_32x32x4,                               // imgFlowRate
  SD_64x64x4,                                     // imgSD
  Menu_64x64x4,                                   // imgMenu
  Settings_64x64x4,                               // imgSettings
  Directory_32x32x4,                              // imgDirectory
  Confirm_64x64x4,                                // imgConfirm
  Cancel_64x64x4,                                 // imgCancel
  Increase_64x64x4,                               // imgIncrease
  Decrease_64x64x4,                               // imgDecrease
  Back_32x32x4,                                   // imgBack
  Up_32x32x4,                                     // imgUp
  Down_32x32x4,                                   // imgDown
  Left_32x32x4,                                   // imgLeft
  Right_32x32x4,                                  // imgRight
  Refresh_32x32x4,                                // imgRefresh
  Leveling_32x32x4,                               // imgLeveling
  Slider8x16x4,                                   // imgSlider
  Home_64x64x4,                                   // imgHome
  BtnRounded_64x52x4,                             // imgBtn52Rounded
  BtnRounded_42x39x4,                             // imgBtn39Rounded
  Time_Elapsed_32x32x4,                           // imgTimeElapsed
  Time_Remaining_32x32x4,                         // imgTimeRemaining
};

#endif // HAS_GRAPHICAL_TFT
