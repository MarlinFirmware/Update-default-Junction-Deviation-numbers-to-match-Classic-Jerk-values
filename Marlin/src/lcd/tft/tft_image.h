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

#include "../../inc/MarlinConfigPre.h"

#if ENABLED(COMPACT_MARLIN_BOOT_LOGO)
  #define MARLIN_LOGO_CHOSEN(W,H) { (void *)marlin_logo_##W##x##H##x16_rle16, W, H, RLE16 }
#else
  #define MARLIN_LOGO_CHOSEN(W,H) { (void *)marlin_logo_##W##x##H##x16, W, H, HIGHCOLOR }
#endif

enum MarlinImage : uint8_t {
  imgBootScreen = 0x00, // BOOTSCREEN_LOGO / NoLogo
  imgHotEnd,            // HotEnd_64x64x4
  imgBed,               // Bed_64x64x4
  imgBedHeated,         // Bed_Heated_64x64x4
  imgChamber,           // Chamber_64x64x4
  imgChamberHeated,     // Chamber_Heated_64x64x4
  imgFanIdle,           // Fan0_64x64x4
  imgFanSlow0,          // Fan_Slow0_64x64x4
  imgFanSlow1,          // Fan_Slow1_64x64x4
  imgFanFast0,          // Fan_Fast0_64x64x4
  imgFanFast1,          // Fan_Fast1_64x64x4
  imgFeedRate,          // Feedrate_32x32x4
  imgFlowRate,          // Flowrate_32x32x4
  imgSD,                // SD_64x64x4
  imgMenu,              // Menu_64x64x4
  imgSettings,          // Settings_64x64x4
  imgDirectory,         // Directory_32x32x4
  imgConfirm,           // Confirm_64x64x4
  imgCancel,            // Cancel_64x64x4
  imgIncrease,          // Increase_64x64x4
  imgDecrease,          // Decrease_64x64x4
  imgBack,              // Back_32x32x4
  imgUp,                // Up_32x32x4
  imgDown,              // Down_32x32x4
  imgLeft,              // Left_32x32x4
  imgRight,             // Right_32x32x4
  imgRefresh,           // Refresh_32x32x4
  imgLeveling,          // Leveling_32x32x4
  imgSlider,            // Slider8x16x4
  imgHome,              // Home_64x64x4
  imgBtn52Rounded,      // BtnRounded_64x52x4
  imgBtn39Rounded,      // BtnRounded_42x39x4
  imgTimeElapsed,       // Time_Elapsed_32x32x4
  imgTimeRemaining,     // Time_Remaining_32x32x4
  // Special values - must be at the end!
  imgCount,
  noImage = imgCount,
  imgPageUp = imgLeft,
  imgPageDown = imgRight
};

enum colorMode_t : uint8_t {
  NOCOLORS = 0x00,
  MONOCHROME = 0x01,  // 1 bit per pixel
  GREYSCALE1 = 0x01,  // 1 bit per pixel
  GREYSCALE2,         // 2 bits per pixel
  GREYSCALE4,         // 4 bits per pixel
  HIGHCOLOR           // 16 bits per pixel
  #if ENABLED(COMPACT_MARLIN_BOOT_LOGO)
    , RLE16           // RLE16 Compressed 16 bits per pixel row
  #endif
};

typedef colorMode_t ColorMode;

#ifdef __AVR__
  #define IMG_PACKED __attribute__((__packed__))
#else
  #define IMG_PACKED
#endif

typedef struct IMG_PACKED {
  void *data;
  uint16_t width;
  uint16_t height;
  colorMode_t colorMode;
} tImage;

extern const tImage NoLogo;

#if ENABLED(SHOW_BOOTSCREEN)
  extern const tImage MarlinLogo112x38x1;
  extern const tImage MarlinLogo228x255x2;
  extern const tImage MarlinLogo228x255x4;
  extern const tImage MarlinLogo195x59x16;
  extern const tImage MarlinLogo240x250x16;
  extern const tImage MarlinLogo320x240x16;
  extern const tImage MarlinLogo480x320x16;
  extern const tImage MarlinLogo576x478x16;
  extern const tImage MarlinLogo1024x600x16;
#endif
extern const tImage Background320x30x16;

extern const tImage HotEnd_64x64x4;
extern const tImage Bed_64x64x4;
extern const tImage Bed_Heated_64x64x4;
extern const tImage Chamber_64x64x4;
extern const tImage Chamber_Heated_64x64x4;
extern const tImage Fan0_64x64x4;
extern const tImage Fan1_64x64x4;
extern const tImage Fan_Slow0_64x64x4;
extern const tImage Fan_Slow1_64x64x4;
extern const tImage Fan_Fast0_64x64x4;
extern const tImage Fan_Fast1_64x64x4;
extern const tImage SD_64x64x4;
extern const tImage Home_64x64x4;
extern const tImage BtnRounded_64x52x4;
extern const tImage BtnRounded_42x39x4;
extern const tImage Menu_64x64x4;
extern const tImage Settings_64x64x4;
extern const tImage Confirm_64x64x4;
extern const tImage Cancel_64x64x4;
extern const tImage Increase_64x64x4;
extern const tImage Decrease_64x64x4;
extern const tImage Pause_64x64x4;

extern const tImage Feedrate_32x32x4;
extern const tImage Flowrate_32x32x4;
extern const tImage Directory_32x32x4;
extern const tImage Back_32x32x4;
extern const tImage Up_32x32x4;
extern const tImage Down_32x32x4;
extern const tImage Left_32x32x4;
extern const tImage Right_32x32x4;
extern const tImage Refresh_32x32x4;
extern const tImage Leveling_32x32x4;
extern const tImage Time_Elapsed_32x32x4;
extern const tImage Time_Remaining_32x32x4;

extern const tImage Slider8x16x4;

extern const tImage images[imgCount];
