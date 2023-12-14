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

#if ENABLED(LCD_ON_EXP3)

  // Remap "EXP3" to "LCD1" for displays that only have a single 10-pin connector

  #define LCD1_01_PIN EXP3_01_PIN
  #define LCD1_02_PIN EXP3_02_PIN
  #define LCD1_03_PIN EXP3_03_PIN
  #define LCD1_04_PIN EXP3_04_PIN
  #define LCD1_05_PIN EXP3_05_PIN
  #define LCD1_06_PIN EXP3_06_PIN
  #define LCD1_07_PIN EXP3_07_PIN
  #define LCD1_08_PIN EXP3_08_PIN

#elif ENABLED(ADAPTER_MELZI_CREALITY)

  /** Creality "Melzi" LCD single-to-dual adapter
   *                   ------
   *   LCD1-1 <= BEEP | 1  2 | ENC   => LCD1-2
   *   LCD2-3 <=  EN1 | 3  4 | RESET => LCD1-4
   *   LCD2-5 <=  EN2   5  6 | A0    => LCD1-7
   *   LCD1-6 <=   CS | 7  8 | MOSI  => LCD2-6
   *              GND | 9 10 | 5V
   *                   ------
   */
  #define LCD1_01_PIN EXP1_01_PIN  // BEEPER
  #define LCD1_02_PIN EXP1_02_PIN  // ENC
  #define LCD1_06_PIN EXP1_07_PIN  // CS / RS / ST9720 CS
  #define LCD1_07_PIN EXP1_06_PIN  // A0 / D4 / ST9720 CLK / SCK

  #define LCD2_03_PIN EXP1_03_PIN  // EN1
  #define LCD2_05_PIN EXP1_05_PIN  // EN2
  #define LCD2_06_PIN EXP1_08_PIN  //      EN / ST9720 DAT / MOSI

#elif ENABLED(ADAPTER_MINIPANEL_DUAL)

  // Map old-style MINIPANEL connectors to LCD1/2.
  // Contrast with Single-to-mini and Dual-to-mini adapters,
  // if such adapters exist for MINIPANEL.

  /**
   *     SDD   MOSI  SDSS       RESET       LCDSS  A0    KILL  ENC
   *     2-7   2-6   2-2                     1-6   1-1   2-3   2-5
   *    ----------------------------- -----------------------------
   *   |  2     4     6     8    10  | 12    14    16    18    20  |
   *   |  1     3     5     7     9  | 11    13    15    17    19  |
   *    ----------------------------- -----------------------------
   *                 2-2                     2-7   1-1   2-3   2-5
   *     5V    MISO  SCK   GND   3V3         BL    BEEP  EN1   EN2
   */

  #define LCD1_01_PIN MINI_15 // BEEP
  #define LCD1_02_PIN MINI_20 // ENC
  #define LCD1_06_PIN MINI_14 // LCDSS
  #define LCD1_07_PIN MINI_16 // A0
  #define LCD1_08_PIN MINI_13 // BACKLIGHT

  #define LCD2_02_PIN MINI_05 // SCK
  #define LCD2_03_PIN MINI_17 // EN1
  #define LCD2_05_PIN MINI_19 // EN2
  #define LCD2_06_PIN MINI_04 // MOSI
  #define LCD2_07_PIN MINI_02 // SD_DETECT
  #define LCD2_08_PIN MINI_18 // KILL

#elif ENABLED(ADAPTER_CREALITY_PIN27)

  // A "Pin 27" Adapter frees EXP1_01_PIN for other uses. This assumes a single EXP plug.
  // The generic "ADAPTER_PIN27" below simply frees LCD1_01_PIN after LCD mapping.
  #define LCD1_02_PIN EXP1_02_PIN  // ENC
  #define LCD1_03_PIN EXP1_03_PIN  // EN1
  #define LCD1_04_PIN EXP1_04_PIN  // RESET
  #define LCD1_05_PIN EXP1_05_PIN  // EN2
  #define LCD1_06_PIN EXP1_06_PIN  // A0 / ST9720 CLK
  #define LCD1_07_PIN EXP1_07_PIN  // CS / ST9720 CS
  #define LCD1_08_PIN EXP1_08_PIN  //      ST9720 DAT

#elif ANY(ADAPTER_CREALITY_DUAL, ADAPTER_BTT_DUAL)

  /** Common LCD single-to-dual adapter
   *                   ------
   *    LCD1-1 <= SCK | 1  2 | ENC   => LCD1-2
   *    LCD2-5 <= EN1 | 3  4 | RESET => LCD1-4
   *    LCD2-3 <= EN2   5  6 | A0    => LCD1-7
   *    LCD1-6 <=  CS | 7  8 | MOSI  => LCD2-6
   *              GND | 9 10 | 5V
   *                   ------
   */
  #define LCD1_02_PIN EXP1_02_PIN  // ENC
  #define LCD1_04_PIN EXP1_04_PIN  // RESET
  #define LCD1_06_PIN EXP1_07_PIN  // CS
  #define LCD1_07_PIN EXP1_06_PIN  // A0

  #define LCD2_02_PIN EXP1_01_PIN  // SCK  - Soft SPI
  #define LCD2_03_PIN EXP1_05_PIN  // EN2
  #define LCD2_05_PIN EXP1_03_PIN  // EN1
  #define LCD2_06_PIN EXP1_08_PIN  // MOSI - Soft SPI

  #define FORCE_SOFT_SPI

#elif ANY(ADAPTER_TH3D_DUAL, ADAPTER_MKS_DUAL)

  /**
   * https://www.th3dstudio.com/product/ezboard-creality-dual-lcd-adapter-board/
   *                   ------
   *    LCD1-1 <= SCK | 1  2 | ENC   => LCD1-2
   *    LCD2-5 <= EN1 | 3  4 | RESET => LCD1-4
   *    LCD2-3 <= EN2   5  6 | A0    => LCD1-7
   *    LCD1-6 <=  CS | 7  8 | MOSI  => LCD2-6
   *              GND | 9 10 | 5V
   *                   ------
   */
  #define LCD1_02_PIN EXP1_02_PIN  // ENC
  #define LCD1_04_PIN EXP1_04_PIN  // RESET
  #define LCD1_06_PIN EXP1_07_PIN  // CS
  #define LCD1_07_PIN EXP1_06_PIN  // A0

  #define LCD2_02_PIN EXP1_01_PIN  // SCK  - Soft SPI
  #define LCD2_03_PIN EXP1_05_PIN  // EN2
  #define LCD2_05_PIN EXP1_03_PIN  // EN1
  #define LCD2_06_PIN EXP1_08_PIN  // MOSI - Soft SPI

  #define FORCE_SOFT_SPI

#elif ENABLED(ADAPTER_CUSTOM_E3_DIP_TFT)

  /**
   * FYSETC_MINI_12864_2_1 / MKS_MINI_12864_V3 / BTT_MINI_12864 / BEEZ_MINI_12864 display pinout
   *
   *                         Board
   *        ------                           ---
   *   NEO | 1  2 | ENC                     | 1 | RST
   *   EN2 | 3  4 | RESET                   | 2 | RX2
   *   EN1   5  6 | LCD_RS                  | 3 | TX2
   *    A0 | 7  8 | CS                      | 4 | GND
   *   GND | 9 10 | 5V                      | 5 | 5V
   *        ------                           ---
   *         EXP1                            TFT
   *                        Display
   *        ------                           ------
   *    5V |10  9 | GND                  -- |10  9 | --
   *    -- | 8  7 | --                RESET | 8  7 | SD_DET
   *   NEO | 6  5   LCD_RS             MOSI | 6  5   EN2
   *    A0 | 4  3 | CS                   -- | 4  3 | EN1
   *   ENC | 2  1 | BEEP                SCK | 2  1 | --
   *        ------                           ------
   *         LCD1                             LCD2
   *
   * Needs custom cable:
   *
   *  EXP1-1  <-->  LCD1-6  NEO
   *  EXP1-2  <-->  LCD1-2  ENC
   *  EXP1-3  <-->  LCD2-5  EN2
   *  EXP1-4  <-->  LCD2-8  RESET
   *  EXP1-5  <-->  LCD2-3  EN1
   *  EXP1-6  <-->  LCD1-8  LCD_RS
   *  EXP1-7  <-->  LCD1-4  A0
   *  EXP1-8  <-->  LCD1-3  CS
   *   TFT-2  <-->  LCD2-6  MOSI
   *   TFT-3  <-->  LCD2-2  SCK
   */

  #define LCD1_02_PIN              EXP1_02_PIN  // ENC
  #define LCD1_03_PIN              EXP1_08_PIN  // CS
  #define LCD1_04_PIN              EXP1_07_PIN  // A0
  #define LCD1_05_PIN              EXP1_06_PIN  // LCD_RS
  #define LCD1_06_PIN              EXP1_01_PIN  // NEOPIXEL

  #define LCD2_02_PIN                   TFT_02  // SCK
  #define LCD2_03_PIN              EXP1_03_PIN  // EN1
  #define LCD2_05_PIN              EXP1_05_PIN  // EN2
  #define LCD2_06_PIN                   TFT_03  // MOSI

  #define FORCE_SOFT_SPI

#elif ENABLED(ADAPTER_CUSTOM_MINI_E3_TFT)

  /**
   * FYSETC_MINI_12864_2_1 / MKS_MINI_12864_V3 / BTT_MINI_12864 / BEEZ_MINI_12864 display pinout
   *
   *                    Board
   *        ------                   ---
   *    A0 | 1  2 | ENC         RST | 1 |
   *    CS | 3  4 | RESET      MOSI | 2 | (RX2)
   *    -- | 5  6 | EN1         SCK | 3 | (TX2)
   *   NEO | 7  8 | EN2         GND | 4 |
   *   GND | 9 10 | 5V           5V | 5 |
   *        ------                   ---
   *         EXP1                    TFT
   *
   *                    Display
   *        ------                  ------
   *  BEEP |10  9 | ENC       MISO |10  9 | SCK
   *    CS | 8  7 | A0         EN1 | 8  7 | SS
   * RESET | 6  5 | RED        EN2 | 6  5 | MOSI
   * GREEN | 4  3 | BLUE        CD | 4  3 | RST
   *   GND | 2  1 | 5V         GND | 2  1 | KILL
   *        ------                  ------
   *         LCD1                    LCD2
   *
   * Needs custom cable:
   *
   *  EXP1-10 <-->  LCD1-1  5V
   *  EXP1-9  <-->  LCD1-2  GND
   *  EXP1-8  <-->  LCD2-6  EN2
   *  EXP1-7  <-->  LCD1-5  RED
   *  EXP1-6  <-->  LCD2-8  EN1
   *  EXP1-5        n/c
   *  EXP1-4  <-->  LCD1-6  RESET
   *  EXP1-3  <-->  LCD1-8  CS
   *  EXP1-2  <-->  LCD1-9  ENC
   *  EXP1-1  <-->  LCD1-7  A0
   *   TFT-2  <-->  LCD2-5  MOSI
   *   TFT-3  <-->  LCD2-9  SCK
   *
   * For backlight configuration see steps 2 (V2.1) and 3 at https://wiki.fysetc.com/Mini12864_Panel/
   */
  #define LCD1_05_PIN              EXP1_07_PIN // RED
  #define LCD1_06_PIN              EXP1_04_PIN // RESET
  #define LCD1_07_PIN              EXP1_01_PIN // A0
  #define LCD1_08_PIN              EXP1_03_PIN // CS
  #define LCD1_09_PIN              EXP1_02_PIN // ENC

  #define LCD2_05_PIN                   TFT_02 // MOSI
  #define LCD2_06_PIN              EXP1_08_PIN // EN2
  #define LCD2_08_PIN              EXP1_06_PIN // EN1
  #define LCD2_09_PIN                   TFT_03 // SCK

  #define FORCE_SOFT_SPI

#elif ENABLED(ADAPTER_CUSTOM_CREALITY_V4_DEBUG_HEADER)
  /**
   *          Board (RET6 12864 LCD)
   *       ------
   *  EN1 | 1  2 | ENC             ---
   *   CS | 3  4 | LCD_RS         | 1 | VCC
   *   A0   5  6 | EN2            | 2 | NEO
   *  SCK | 7  8 | MOSI           | 3 | --
   *  GND | 9 10 | 5V             | 4 | GND
   *       ------                  ---
   *       "EXP3"               Debug port
   *
   *                  Display
   *       ------                  ------
   *   5V |10  9 | GND         -- |10  9 | --
   *   -- | 8  7 | --       RESET | 8  7 | --
   *  NEO | 6  5   LCD_RS    MOSI | 6  5   EN2
   *   A0 | 4  3 | CS          -- | 4  3 | EN1
   *  ENC | 2  1 | --         SCK | 2  1 | --
   *       ------                  ------
   *        LCD1                    LCD2
   *
   * Needs custom cable. Connect EN2-EN2, LCD_CS-LCD_CS and so on.
   * Debug port is just above EXP1. You may need to add pins.
   */

  #define LCD1_02_PIN              EXP3_02_PIN  // ENC
  #define LCD1_03_PIN              EXP3_03_PIN  // CS
  #define LCD1_04_PIN              EXP3_05_PIN  // A0
  #define LCD1_05_PIN              EXP3_04_PIN  // RESET
  #define LCD1_06_PIN             DEBUG_02_PIN  // NEOPIXEL

  #define LCD2_02_PIN              EXP3_07_PIN  // SCK
  #define LCD2_03_PIN              EXP3_01_PIN  // EN1
  #define LCD2_05_PIN              EXP3_06_PIN  // EN2
  #define LCD2_06_PIN              EXP3_08_PIN  // MOSI

  #define FORCE_SOFT_SPI

#elif ENABLED(ADAPTER_SKR_MINI_SCREEN)

  /**
   * See https://github.com/VoronDesign/Voron-Hardware/tree/master/SKR-Mini_Screen_Adaptor/SRK%20Mini%20E3%20V3.0
   *
   *                     Board
   *        ------                   ------
   *   ENC | 1  2 | NEO          5V | 1  2 | GND
   *   EN1 | 3  4 | RESET        CS | 3  4 | SCK
   *   EN2   5  6 | LCD_RS     MOSI | 5  6 | MISO
   *    CS | 7  8 | A0          3V3 | 7  8 | GND
   *   GND | 9 10 | 5V               ------
   *        ------                    SPI
   *         EXP1
   *
   *           SKR Mini Screen "Adaptor"
   *        ------                   ------
   *    -- | 1  2 | ENC        MISO | 1  2 | SCK
   *    CS | 3  4 | A0          EN1 | 3  4 | --
   * RESET   5  6 | DIN         EN2   5  6 | MOSI
   *    -- | 7  8 | --           -- | 7  8 | --
   *   GND | 9 10 | 5V          GND | 9 10 | RESET (KILL)
   *        ------                   ------
   *         EXP1                     EXP2
   */

  // TODO: Ideally, adapters should provide standard display-agnostic connectors
  #if ENABLED(FYSETC_MINI_12864_2_1)

    #define LCD1_02_PIN            EXP1_01_PIN  // ENC
    #define LCD1_03_PIN            EXP1_07_PIN  // DOGLCD_CS
    #define LCD1_04_PIN            EXP1_08_PIN  // DOGLCD_A0
    #define LCD1_05_PIN            EXP1_06_PIN  // LCD_RESET_PIN
    #define LCD1_06_PIN            EXP1_02_PIN  // NEOPIXEL_PIN

    #define LCD2_02_PIN            SPI1_04_PIN  // DOGLCD_SCK
    #define LCD2_03_PIN            EXP1_03_PIN  // EN1
    #define LCD2_05_PIN            EXP1_05_PIN  // EN2
    #define LCD2_06_PIN            SPI1_05_PIN  // DOGLCD_MOSI

    #define FORCE_SOFT_SPI

  #else
    #error "Only FYSETC_MINI_12864_2_1, MKS_MINI_12864_V3, and BTT_MINI_12864 are supported with ADAPTER_SKR_MINI_SCREEN."
  #endif

#elif ENABLED(ADAPTER_CUSTOM_E3_V3_NEO)

  /**
   *                     Board
   *       ------
   *  EN2 | 1  2 | ENC                ---
   *   CS | 3  4 | RESET             | 3 |
   *   A0   5  6 | EN1               | 2 | (DIN)
   *  SCK | 7  8 | MOSI              | 1 |
   *  GND | 9 10 | 5V                 ---
   *       ------                   NeoPixel
   *        EXP1
   *                     Display
   *       ------                     ------
   *   5V |10  9 | GND            -- |10  9 | --
   *   -- | 8  7 | --          RESET | 8  7 | --
   *  DIN | 6  5   RESET        MOSI | 6  5   EN2
   *   A0 | 4  3 | CS             -- | 4  3 | EN1
   *  ENC | 2  1 | --            SCK | 2  1 | --
   *       ------                     ------
   *        EXP1                       EXP2
   *
   * Needs custom cable. Connect EN2-EN2, LCD_CS-LCD_CS and so on.
   *
   * Check the index/notch position twice!!!
   * On BTT boards pins from IDC10 connector are numbered in unusual order.
   */

  #define LCD1_02_PIN              EXP1_02_PIN  // ENC
  #define LCD1_03_PIN              EXP1_03_PIN  // CS
  #define LCD1_04_PIN              EXP1_05_PIN  // A0
  #define LCD1_05_PIN              EXP1_04_PIN  // LCD_RS
  #define LCD1_06_PIN             NEOPIXEL_PIN  // NEOPIXEL

  #define LCD2_02_PIN              EXP1_07_PIN  // SCK
  #define LCD2_03_PIN              EXP1_06_PIN  // EN1
  #define LCD2_05_PIN              EXP1_01_PIN  // EN2
  #define LCD2_06_PIN              EXP1_08_PIN  // MOSI

  #define FORCE_SOFT_SPI

#endif

// Provide One-to-One mapping by default

#define _LCD_SET(N,I) && (LCD##I##_0##N##_PIN == 0)

#if defined(EXP1_01_PIN) REPEAT2_S(1,9,_LCD_SET,1)
  #define LCD1_01_PIN EXP1_01_PIN
  #define LCD1_02_PIN EXP1_02_PIN
  #define LCD1_03_PIN EXP1_03_PIN
  #define LCD1_04_PIN EXP1_04_PIN
  #define LCD1_05_PIN EXP1_05_PIN
  #define LCD1_06_PIN EXP1_06_PIN
  #define LCD1_07_PIN EXP1_07_PIN
  #define LCD1_08_PIN EXP1_08_PIN
#endif

#if defined(EXP2_01_PIN) REPEAT2_S(1,9,_LCD_SET,2)
  #define LCD2_01_PIN EXP2_01_PIN
  #define LCD2_02_PIN EXP2_02_PIN
  #define LCD2_03_PIN EXP2_03_PIN
  #define LCD2_04_PIN EXP2_04_PIN
  #define LCD2_05_PIN EXP2_05_PIN
  #define LCD2_06_PIN EXP2_06_PIN
  #define LCD2_07_PIN EXP2_07_PIN
  #define LCD2_08_PIN EXP2_08_PIN
#endif

// Define "LCD3" pins if EXP3 is not remapped to LCD1
// May not be needed if we just always remap "LCD1" to EXP3

#if defined(EXP3_01_PIN) REPEAT2_S(1,9,_LCD_SET,3)
  #define LCD3_01_PIN EXP3_01_PIN
  #define LCD3_02_PIN EXP3_02_PIN
  #define LCD3_03_PIN EXP3_03_PIN
  #define LCD3_04_PIN EXP3_04_PIN
  #define LCD3_05_PIN EXP3_05_PIN
  #define LCD3_06_PIN EXP3_06_PIN
  #define LCD3_07_PIN EXP3_07_PIN
  #define LCD3_08_PIN EXP3_08_PIN
#endif

#undef _LCD_SET

#if ENABLED(ADAPTER_PIN27)
  #undef LCD1_01_PIN
#endif

#ifndef LCD1_01_PIN
  #define LCD1_01_PIN -1
#endif
#ifndef LCD1_02_PIN
  #define LCD1_02_PIN -1
#endif
#ifndef LCD1_03_PIN
  #define LCD1_03_PIN -1
#endif
#ifndef LCD1_04_PIN
  #define LCD1_04_PIN -1
#endif
#ifndef LCD1_05_PIN
  #define LCD1_05_PIN -1
#endif
#ifndef LCD1_06_PIN
  #define LCD1_06_PIN -1
#endif
#ifndef LCD1_07_PIN
  #define LCD1_07_PIN -1
#endif
#ifndef LCD1_08_PIN
  #define LCD1_08_PIN -1
#endif

#ifndef LCD2_01_PIN
  #define LCD2_01_PIN -1
#endif
#ifndef LCD2_02_PIN
  #define LCD2_02_PIN -1
#endif
#ifndef LCD2_03_PIN
  #define LCD2_03_PIN -1
#endif
#ifndef LCD2_04_PIN
  #define LCD2_04_PIN -1
#endif
#ifndef LCD2_05_PIN
  #define LCD2_05_PIN -1
#endif
#ifndef LCD2_06_PIN
  #define LCD2_06_PIN -1
#endif
#ifndef LCD2_07_PIN
  #define LCD2_07_PIN -1
#endif
#ifndef LCD2_08_PIN
  #define LCD2_08_PIN -1
#endif
