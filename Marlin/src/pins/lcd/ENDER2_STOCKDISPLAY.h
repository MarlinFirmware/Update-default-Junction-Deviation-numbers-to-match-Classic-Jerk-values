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

/**
 * pins/lcd/ENDER2_STOCKDISPLAY.h
 */

// U8GLIB_MINI12864_2X_HAL : u8g_dev_uc1701_mini12864_HAL_2x_sw_spi, u8g_dev_uc1701_mini12864_HAL_2x_hw_spi

// MINIPANEL
//  DOGLCD
//   HAS_MARLINUI_U8GLIB > (128x64)
//  IS_ULTIPANEL
//   HAS_WIRED_LCD
//     HAS_DISPLAY
//       HAS_STATUS_MESSAGE
//   IS_NEWPANEL
//   HAS_MARLINUI_MENU
//     HAS_MANUAL_MOVE_MENU

/**
 *        1     3    (5)    7     9
 * LCD1:  BEEP  NC    NC    LCDA0 GND
 *        ENC   NC    LCDCS NC    VCC
 *        2     4     6     8     10
 *
 *        1     3    (5)    7     9
 * LCD2: <MISO  EN2   EN1  <SDDET GND
 *        SCK  <SDSS  MOSI  KILL  NC
 *        2     4     6     8     10
 */

// When using the adapter, BEEPER is not available.
// Instead this pin gets used for DOGLCD_SCK.
#if !defined(BEEPER_PIN) && LCD2_02_PIN != EXP1_01_PIN && LCD2_02_PIN != EXP3_01_PIN
  #define BEEPER_PIN      LCD1_01_PIN
#endif

#define BTN_ENC           LCD1_02_PIN
#define BTN_EN1           LCD2_03_PIN
#define BTN_EN2           LCD2_05_PIN

#define DOGLCD_CS         LCD1_06_PIN
#define DOGLCD_A0         LCD1_07_PIN

// When using the single-plug adapter
// these will be re-routed from 1-1 and 1-8
#if PIN_EXISTS(LCD2_02)
  #define DOGLCD_SCK      LCD2_02_PIN  // usually SD_SCK_PIN
  #define DOGLCD_MOSI     LCD2_06_PIN  // usually SD_MOSI_PIN
#endif

// Defined by some pins files, but extraneous
//#define LCD_PINS_RS     LCD1_06_PIN  // ST7920 CS
//#define LCD_PINS_D4     LCD1_07_PIN  // ST7920 CLK (SCK)
//#define LCD_PINS_EN     LCD2_06_PIN  // ST7920 DAT (MOSI)

//#if DOGLCD_SCK != SD_SCK_PIN
//  #define FORCE_SOFT_SPI
//#endif
