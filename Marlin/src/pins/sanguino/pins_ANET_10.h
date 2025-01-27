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
 * Anet V1.0 board pin assignments
 * Schematic: https://github.com/ralf-e/ANET-3D-Board-V1.0/blob/master/ANET3D_Board_Schematic.pdf
 */

/**
 * Rev B    16 JUN 2017
 *
 * 1) no longer uses Sanguino files to define some of the pins
 * 2) added pointers to useable Arduino IDE extensions
 */

/**
 * The standard Arduino IDE extension (Boards Manager) for this board
 * is located at https://github.com/SkyNet3D/anet-board.
 *
 * Installation instructions are on that page.
 *
 * After copying the files to the appropriate location, restart Arduino and
 * you'll see "Anet V1.0" and "Anet V1.0 (Optiboot)" in the boards list.
 *
 * "Anet V1.0" uses the bootloader that was installed on the board when
 * it shipped from the factory.
 *
 * "Anet V1.0 (Optiboot)" frees up another 3K of FLASH.  You'll need to burn
 * a new bootloader to the board to be able to automatically download a
 * compiled image.
 */

/**
 * Requires this Arduino IDE extension for Boards Manager:
 * https://github.com/Lauszus/Sanguino
 *
 * Follow the installation instructions at https://learn.sparkfun.com/pages/CustomBoardsArduino
 * Just use this JSON URL instead of Sparkfun's:
 * https://raw.githubusercontent.com/Lauszus/Sanguino/master/package_lauszus_sanguino_index.json
 *
 * Once installed select the SANGUINO board and then select the CPU.
 */

/**
 *  To burn a new bootloader:
 *
 *   1. Connect your programmer to the board.
 *   2. In the Arduino IDE select the board and then select the programmer.
 *   3. In the Arduino IDE click on "burn bootloader". Don't worry about the "verify failed at 1F000" error message.
 *   4. The programmer is no longer needed. Remove it.
 */

/**
 * Additional info:
 *
 *   Anet Schematics                    - https://github.com/ralf-e/ANET-3D-Board-V1.0
 *   Wiring RRDFG Smart Controller      - https://www.thingiverse.com/thing:2103748
 *   SkyNet3D Anet software development - https://github.com/SkyNet3D/Marlin/
 *   Anet Users / Skynet SW on Facebook - https://www.facebook.com/skynet3ddevelopment/
 *
 *   Many thanks to Hans Raaf (@oderwat) for developing the Anet-specific software and supporting the Anet community.
 */

/**
 * OptiBoot Bootloader:
 *   Optiboot is an alternative bootloader that can be flashed on the board to free up space for a larger firmware build.
 *   See https://github.com/Optiboot/optiboot for more information.
 *
 * Install Marlin with Arduino IDE:
 *   For a board with the stock bootloader, select 'Sanguino' in 'Tools > Board' and 'ATmega1284P' in 'Tools > Processor.'
 *   For a board with OptiBoot, select 'Sanguino (Optiboot)' in 'Tools > Board' and 'ATmega1284P' in 'Tools > Processor.'
 *
 * Install Marlin with PlatformIO IDE:
 *   (NOTE: You can set a default build environment by editing the value of 'default_env' in 'platformio.ini'.
 *          For the best user experience install the "Auto Build Marlin" extension.)
 *   For a board with the stock bootloader use Build / Upload under the 'sanguino1284p' or 'sanguino1284p_optimized' target.
 *   For a board with OptiBoot, use Build / Upload under the 'melzi_optiboot' target.
 */

#if NOT_TARGET(__AVR_ATmega1284P__)
  #error "Oops! Select 'Sanguino' in 'Tools > Board' and 'ATmega1284P' in 'Tools > Processor.' (For PlatformIO, use 'sanguino1284p' or 'sanguino1284p_optimized'. With optiboot, use 'melzi_optiboot.')"
#endif

#define BOARD_INFO_NAME "Anet 1.0"

//
// Limit Switches
//
#define X_STOP_PIN                            18
#define Y_STOP_PIN                            19
#define Z_STOP_PIN                            20

//
// Steppers
//
#define X_STEP_PIN                            15
#define X_DIR_PIN                             21
#define X_ENABLE_PIN                          14

#define Y_STEP_PIN                            22
#define Y_DIR_PIN                             23
#define Y_ENABLE_PIN                          14

#define Z_STEP_PIN                             3
#define Z_DIR_PIN                              2
#define Z_ENABLE_PIN                          26

#define E0_STEP_PIN                            1
#define E0_DIR_PIN                             0
#define E0_ENABLE_PIN                         14

//
// Temperature Sensors
//
#define TEMP_0_PIN                             7  // Analog Input (pin 33 extruder)
#define TEMP_BED_PIN                           6  // Analog Input (pin 34 bed)

//
// Heaters / Fans
//
#define HEATER_0_PIN                          13  // (extruder)
#define HEATER_BED_PIN                        12  // (bed)

#ifndef FAN0_PIN
  #define FAN0_PIN                             4
#endif

//
// Misc. Functions
//
#define SD_SS_PIN                             31
#define LED_PIN                               -1

/**
 * Connector pinouts
 *
 *            ------                            ------                             ----
 * (SDA)  17 | 1  2 | 30 (A1)              3V3 | 1  2 | 4 (SS)              J3_RX |1  2| J3_TX
 * (SCL)  16 | 3  4 | 29 (A2)              GND | 3  4 | RESET            (TXO)  9 |3  4| 8 (RX0)
 *        11 | 5  6   28 (A3)       (MOSI)   5 | 5  6   7 (SCK)            USB_RX |5  6| USB_TX
 *        10 | 7  8 | 27 (A4)               5V | 7  8 | 6 (MISO)                   ----
 *        5V | 9 10 | GND                J3_RX | 9 10 | J3_TX                     USB_BLE
 *            ------                            ------
 *             LCD                                J3
 */
#define EXP1_01_PIN                           17  // BEEPER / ENC
#define EXP1_02_PIN                           30  // LCD_D4 / SERVO
#define EXP1_03_PIN                           16  // ENC    / LCD_EN
#define EXP1_04_PIN                           29  // SERVO  / LCD_RS
#define EXP1_05_PIN                           11  // EN1    / LCD_D4
#define EXP1_06_PIN                           28  // LCD_EN / EN1
#define EXP1_07_PIN                           10  // EN2
#define EXP1_08_PIN                           27  // LCD_RS / BEEPER

/**
 * LCD / Controller
 *
 * Only the following displays are supported:
 *  ZONESTAR_LCD
 *  ANET_FULL_GRAPHICS_LCD
 *  CTC_A10S_A13
 *  REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
 */

#if HAS_WIRED_LCD

  #define LCD_SDSS_PIN               EXP1_06_PIN

  #if HAS_ADC_BUTTONS

    #define SERVO0_PIN               EXP1_08_PIN  // free for BLTouch/3D-Touch
    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_04_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                     1

  #elif IS_RRD_FG_SC

    // Pin definitions for the Anet A6 Full Graphics display and the RepRapDiscount Full Graphics
    // display using an adapter board. See https://aisler.net/benlye/anet-lcd-adapter/pcb
    // See below for alternative pin definitions for use with https://www.thingiverse.com/thing:2103748

    #if ENABLED(CTC_A10S_A13)

      /**
       * CTC_A10S_A13 pinout
       *
       *           ------
       *      GND | 1  2 | 5V
       *   BEEPER | 3  4 | BTN_EN2
       *  BTN_EN1   5  6 | LCD_D4
       *   LCD_RS | 7  8 | LCD_EN
       *   SERVO0 | 9 10 | BTN_ENC
       *           ------
       *            LCD
       */
      #define SERVO0_PIN             EXP1_02_PIN

      #define BEEPER_PIN             EXP1_08_PIN

      #define BTN_ENC                EXP1_01_PIN
      #define BTN_EN1                EXP1_06_PIN
      #define BTN_EN2                EXP1_07_PIN

      #define LCD_PINS_RS            EXP1_04_PIN
      #define LCD_PINS_EN            EXP1_03_PIN
      #define LCD_PINS_D4            EXP1_05_PIN

      #define BOARD_ST7920_DELAY_1           250
      #define BOARD_ST7920_DELAY_2           250
      #define BOARD_ST7920_DELAY_3           250

    #else

      /**
       * ANET_FULL_GRAPHICS_LCD pinouts
       *
       *          ------                      ------
       *     GND | 1  2 | 5V               - | 1  2 | -
       *  LCD_RS | 3  4 | BTN_EN2          - | 3  4 | 5V
       *  LCD_EN   5  6 | BTN_EN1          -   5  6 | -
       *  SERVO0 | 7  8 | BTN_ENC      RESET | 7  8 | GND
       *  LCD_D4 | 9 10 | BEEPER_PIN       - | 9 10 | 3V3
       *          ------                      ------
       *           LCD                          J3
       */
      #define SERVO0_PIN             EXP1_04_PIN  // Free for BLTouch/3D-Touch

      #define BEEPER_PIN             EXP1_01_PIN

      #define BTN_ENC                EXP1_03_PIN
      #define BTN_EN1                EXP1_05_PIN
      #define BTN_EN2                EXP1_07_PIN

      #define LCD_PINS_RS            EXP1_08_PIN
      #define LCD_PINS_EN            EXP1_06_PIN
      #define LCD_PINS_D4            EXP1_02_PIN

      #define BOARD_ST7920_DELAY_1           125
      #define BOARD_ST7920_DELAY_2            63
      #define BOARD_ST7920_DELAY_3           125

    #endif

  #endif

#else
  #define SERVO0_PIN                 EXP1_08_PIN
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN              SERVO0_PIN
#endif

/**
 * ====================================================================
 * =============== Alternative RepRapDiscount Wiring ==================
 * ====================================================================
 *
 * An alternative wiring scheme for the RepRapDiscount Full Graphics Display is
 * published by oderwat on Thingiverse at https://www.thingiverse.com/thing:2103748.
 *
 * Using that adapter requires changing the pin definition as follows:
 *   #define SERVO0_PIN   27   // free for BLTouch/3D-Touch
 *   #define BEEPER_PIN   28
 *   #define LCD_PINS_RS  30
 *   #define LCD_PINS_EN  29
 *   #define LCD_PINS_D4  17
 *
 * The BLTouch pin becomes LCD:3
 */

/**
 * ====================================================================
 * ===================== LCD PINOUTS ==================================
 * ====================================================================
 *
 *   Anet V1.0 controller           | ZONESTAR_LCD      | ANET_FULL_   | RepRapDiscount Full     | Thingiverse RepRap wiring
 *   physical   logical   alt       |                   | GRAPHICS_LCD | Graphics Display Wiring | https://www.thingiverse.com/thing:2103748
 *     pin        pin     functions |                   |              |                         |
 *--------------------------------------------------------------------------------------------------------------------
 *   ANET-J3.1    8 ***             | N/A               | J3_TX ***    |                         |
 *   ANET-J3.2    9 ***             | N/A               | J3_RX ***    |                         |
 *   ANET-J3.3    6       MISO      | N/A               | MISO ***     | EXP2.1   MISO           | EXP2.1   MISO
 *   ANET-J3.4    +5V               | N/A               | +5V          |                         |
 *   ANET-J3.5    7       SCK       | N/A               | SCK ***      | EXP2.2   SCK            | EXP2.2   SCK
 *   ANET-J3.6    5       MOSI      | N/A               | MOSI ***     | EXP2.6   MOSI           | EXP2.6   MOSI
 *   ANET-J3.7    !RESET            | N/A               | button       | EXP2.8   panel button   | EXP2.8   panel button
 *   ANET-J3.8    GND               | N/A               | GND          | EXP2.9   GND            | EXP2.9   GND
 *   ANET-J3.9    4       Don't use | N/A               | N/C          |                         |
 *   ANET-J3.10   +3.3V             | N/A               | +3.3V ***    |                         |
 *                                  |                   |              |                         |
 *                                  |                   |              |                         |
 *   ANET-LCD.1   GND               | GND               | GND          | EXP1.9   GND            | EXP1.9   GND
 *   ANET-LCD.2   +5V               | +5V               | +5V          | EXP1.10  +5V            | EXP1.10  +5V
 *   ANET-LCD.3   27      A4        | N/C *             | LCD_PINS_RS  | EXP1.4   LCD_PINS_RS    | EXP2.4   SDSS or N/C *
 *   ANET-LCD.4   10                | LCD_PINS_D4       | BTN_EN2      | EXP2.3   BTN_EN2        | EXP2.3   BTN_EN2
 *   ANET-LCD.5   28      A3        | LCD_PINS_RS       | LCD_PINS_EN  | EXP1.3   LCD_PINS_EN    | EXP1.1   BEEPER_PIN
 *   ANET-LCD.6   11                | LCD_PINS_D5       | BTN_EN1      | EXP2.5   BTN_EN1        | EXP2.5   BTN_EN1
 *   ANET-LCD.7   29      A2        | LCD_PINS_EN       | N/C *        | EXP2.4   SDSS or N/C *  | EXP1.3   LCD_PINS_EN
 *   ANET-LCD.8   16      SCL       | LCD_PINS_D6       | BTN_ENC      | EXP1.2   BTN_ENC        | EXP1.2   BTN_ENC
 *   ANET-LCD.9   30      A1        | ADC_KEYPAD_PIN ** | LCD_PINS_D4  | EXP1.5   LCD_PINS_D4    | EXP1.4   LCD_PINS_RS
 *   ANET-LCD.10  17      SDA       | LCD_PINS_D7       | BEEPER_PIN   | EXP1.1   BEEPER_PIN     | EXP1.5   LCD_PINS_D4
 *
 *                 N/C * - if not connected to the LCD can be used for BLTouch servo input
 *                 ** - analog pin -WITHOUT a pullup
 *                 *** - only connected to something if the Bluetooth module is populated
 */

/**
 *   REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER
 *   physical pin  function
 *   EXP1.1        BEEPER
 *   EXP1.2        BTN_ENC
 *   EXP1.3        LCD_PINS_EN
 *   EXP1.4        LCD_PINS_RS
 *   EXP1.5        LCD_PINS_D4
 *   EXP1.6        LCD_PINS_D5 (not used)
 *   EXP1.7        LCD_PINS_D6 (not used)
 *   EXP1.8        LCD_PINS_D7 (not used)
 *   EXP1.9        GND
 *   EXP1.10       VCC
 *
 *
 *   EXP2.1        MISO
 *   EXP2.2        SCK
 *   EXP2.3        BTN_EN2
 *   EXP2.4        SDSS
 *   EXP2.5        BTN_EN1
 *   EXP2.6        MOSI
 *   EXP2.7        SD_DETECT_PIN
 *   EXP2.8        button
 *   EXP2.9        GND
 *   EXP2.10       NC
 */
