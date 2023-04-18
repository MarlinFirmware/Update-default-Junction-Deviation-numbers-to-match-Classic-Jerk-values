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
 * pins/lcd/MINIPANEL.h
 * https://reprap.org/wiki/Mini_panel
 */

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
 * Standard Minipanel has a single 20-pin connector which is mapped onto EXP1/2
 * for controllers that also define 'MINIPANEL', so the real MINIPANEL is just
 * mapped onto EXP1/2 as well.
 *
 * Those Creality and MKS displays may have been using 'MINIPANEL' as a way to
 * get more board support "for free" so that association may be removed after
 * reviewing the history.
 *
 *     SDD   MOSI  SDSS       RESET       LCDSS  A0    KILL  ENC
 *     2-7   2-6   2-2                           1-1   2-3   2-5
 *    ----------------------------- -----------------------------
 *   |  2     4     6     8    10  | 12    14    16    18    20  |
 *   |  1     3     5     7     9  | 11    13    15    17    19  |
 *    ----------------------------- -----------------------------
 *                 2-2                     2-7   1-1   2-3   2-5
 *     5V    MISO  SCK   GND   3V3         BL    BEEP  EN1   EN2
 *
 * LCD1:  BEEP   --     --     A0     GND
 *        1      3     (5)     7      9
 *        2      4      6      8      10
 *        ENC    --    LCDCS   --     VCC
 *
 * LCD2: <MISO   EN2    EN1   <SDDET  GND
 *        1      3     (5)     7      9
 *        2      4      6      8      10
 *        SCK   <SDSS   MOSI   KILL   --
 */

// When using a single-plug adapter, BEEPER is not available.
// Instead its usual pin gets mapped to DOGLCD_SCK.
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
#if PINS_EXIST(LCD2_02, LCD2_06)
  #define DOGLCD_SCK      LCD2_02_PIN  // usually SD_SCK_PIN
  #define DOGLCD_MOSI     LCD2_06_PIN  // usually SD_MOSI_PIN
#endif

#define LCD_BACKLIGHT_PIN LCD1_08_PIN
#define KILL_PIN          LCD2_08_PIN

#ifndef SD_DETECT_PIN
  #define SD_DETECT_PIN   LCD2_07_PIN
#endif

//#if DOGLCD_SCK != SD_SCK_PIN
//  #define FORCE_SOFT_SPI
//#endif
