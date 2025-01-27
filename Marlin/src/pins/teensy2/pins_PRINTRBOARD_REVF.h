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
 *  Rev B  2 JUN 2017
 *
 *  Converted to Arduino pin numbering
 *
 *  Schematic (RevF): https://github.com/lwalkera/printrboard/raw/revF/Printrboard.sch
 *  Schematic (RevF2): https://raw.githubusercontent.com/lwalkera/printrboard/revF2/Printrboard.sch
 *  Schematic (RevF3): https://raw.githubusercontent.com/lwalkera/printrboard/revF3/Printrboard.sch
 *  Schematic (RevF4): https://raw.githubusercontent.com/lwalkera/printrboard/revF4/Printrboard.sch
 *  Schematic (RevF5): https://raw.githubusercontent.com/lwalkera/printrboard/revF5/Printrboard.sch
 *  Schematic (RevF6): https://raw.githubusercontent.com/lwalkera/printrboard/revF6/Printrboard.sch
 */

/**
 *  There are two Arduino IDE extensions that are compatible with this board
 *  and with the mainstream Marlin software.
 *
 *  Teensyduino -  https://www.pjrc.com/teensy/teensyduino.html
 *  Installation - https://www.pjrc.com/teensy/td_download.html
 *
 *    Select Teensy++ 2.0 in Arduino IDE from the 'Tools > Board' menu
 *
 *    Note: With Teensyduino extension, the Arduino IDE will report 130048 bytes of program storage space available,
 *    but there is actually only 122880 bytes due to the larger DFU bootloader shipped by default on all Printrboard RevF.
 *
 *  Printrboard - https://github.com/scwimbush/Printrboard-HID-Arduino-IDE-Support
 *
 *    Installation:
 *
 *       1. Go to the above URL, click on the "Clone or Download" button and then
 *          click on "Download ZIP" button.
 *       2. Unzip the file, find the "printrboard" directory and then copy it to the
 *          hardware directory in Arduino.  The Arduino hardware directory will probably
 *          be located in a path similar to this: C:\Program Files (x86)\Arduino\hardware.
 *       3. Restart Arduino.
 *       4. Select "Printrboard" from the 'Tools > Board' menu.
 *
 *  Teensyduino is the most popular and easiest option.
 */

/**
 *  To burn the bootloader that comes with Printrboard HID extension:
 *
 *   1. Connect your programmer to the board.
 *   2. In the Arduino IDE select "Printrboard" and then select the programmer.
 *   3. In the Arduino IDE click on "burn bootloader". Don't worry about the "verify failed at 1F000" error message.
 *   4. The programmer is no longer needed. Remove it.
 */

#include "env_validate.h"

#if !defined(__MARLIN_DEPS__) && !defined(USBCON)
  #error "USBCON should be defined by the platform for this board."
#endif

#define BOARD_INFO_NAME "Printrboard Rev.F"

// Disable JTAG pins so EXP1 pins work correctly
// (Its pins are used for the Extrudrboard and filament sensor, for example).
#define DISABLE_JTAG

/**
 * Note that REV F6 of the Printrboard stole the A HOTEND pin and
 * reassigned it to a second fan for the extruder heater.  It's
 * recommended that you swap the A and B outputs on the Extrudrboard
 * so EXTRUDERS=2 will still work on F6, using B for E1/HEATER_1/TEMP_1.
 * See https://printrbot.zendesk.com/hc/en-us/articles/115003072346
 *
 * If you have REV F6 you probably also want to set E0_AUTO_FAN_PIN
 * to PRINTRBOARD_F6_HEATER_FAN_PIN (44).
 *
 * Define NO_EXTRUDRBOARD if you don't have an EXTRUDRBOARD and wish to
 * reassign different functions to EXP1.
 *
 * Define NO_EXTRUDRBOARD_OUTPUT_SWAP if you have a REV F5 or lower and
 * want to use EXTRUDRBOARD A for E1 and EXTRUDRBOARD B for E2.
 */
//#define NO_EXTRUDRBOARD
//#define NO_EXTRUDRBOARD_OUTPUT_SWAP

//
// Limit Switches
//
#define X_STOP_PIN                            47  // E3
#define Y_STOP_PIN                            24  // B4 PWM2A
#define Z_STOP_PIN                            36  // E4

//
// Steppers
//
#define X_STEP_PIN                            28  // A0
#define X_DIR_PIN                             29  // A1
#define X_ENABLE_PIN                          19  // E7

#define Y_STEP_PIN                            30  // A2
#define Y_DIR_PIN                             31  // A3
#define Y_ENABLE_PIN                          18  // E6

#define Z_STEP_PIN                            32  // A4
#define Z_DIR_PIN                             33  // A5
#define Z_ENABLE_PIN                          17  // C7

#define E0_STEP_PIN                           34  // A6
#define E0_DIR_PIN                            35  // A7
#define E0_ENABLE_PIN                         13  // C3

#if DISABLED(NO_EXTRUDRBOARD)
#if DISABLED(NO_EXTRUDRBOARD_OUTPUT_SWAP)
  #define E1_STEP_PIN                         25  // B5
  #define E1_DIR_PIN                          37  // E5
  #define E1_ENABLE_PIN                       42  // F4

  #define E2_STEP_PIN                          2  // D2
  #define E2_DIR_PIN                           3  // D3
  #define E2_ENABLE_PIN                       43  // F5
#else
  #define E1_STEP_PIN                          2  // D2
  #define E1_DIR_PIN                           3  // D3
  #define E1_ENABLE_PIN                       43  // F5

  #define E2_STEP_PIN                         25  // B5
  #define E2_DIR_PIN                          37  // E5
  #define E2_ENABLE_PIN                       42  // F4
#endif
#endif // NO_EXTRUDRBOARD

// Enable control of stepper motor currents with the I2C based MCP4728 DAC used on Printrboard REVF
#define HAS_MOTOR_CURRENT_DAC 1

// Set default drive strength percents if not already defined - X, Y, Z, E axis
#ifndef DAC_MOTOR_CURRENT_DEFAULT
  #define DAC_MOTOR_CURRENT_DEFAULT { 70, 70, 50, 70 }
#endif

// Number of channels available for DAC
#define DAC_STEPPER_ORDER { 3, 2, 1, 0 }

#define DAC_STEPPER_SENSE    0.11
#define DAC_STEPPER_ADDRESS                    0
#define DAC_STEPPER_MAX                      3520
#define DAC_STEPPER_VREF                       1  // internal Vref, gain 1x = 2.048V
#define DAC_STEPPER_GAIN                       0
#define DAC_OR_ADDRESS                      0x00

//
// Temperature Sensors
//
#define TEMP_0_PIN                             1  // Analog Input (Extruder)
#define TEMP_BED_PIN                           0  // Analog Input (Bed)

#if DISABLED(NO_EXTRUDRBOARD)
#if DISABLED(NO_EXTRUDRBOARD_OUTPUT_SWAP)
  #define TEMP_1_PIN                           2  // Analog Input (Extrudrboard A THERM)
  #define TEMP_2_PIN                           3  // Analog Input (Extrudrboard B THERM)
#else
  #define TEMP_1_PIN                           3  // Analog Input (Extrudrboard B THERM)
  #define TEMP_2_PIN                           2  // Analog Input (Extrudrboard A THERM)
#endif
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          15  // C5 PWM3B - Extruder
#define HEATER_BED_PIN                        14  // C4 PWM3C

#if DISABLED(NO_EXTRUDRBOARD)
#if DISABLED(NO_EXTRUDRBOARD_OUTPUT_SWAP)
  #define HEATER_1_PIN                        44  // F6 - Extrudrboard A HOTEND
  #define HEATER_2_PIN                        45  // F7 - Extrudrboard B HOTEND
#else
  #define HEATER_1_PIN                        45  // F7 - Extrudrboard B HOTEND
  #define HEATER_2_PIN                        44  // F6 - Extrudrboard A HOTEND
#endif
#endif

#ifndef FAN0_PIN
  #define FAN0_PIN                            16  // C6 PWM3A
#endif

//
// LCD / Controller
//
//#define USE_INTERNAL_SD

#if HAS_WIRED_LCD
  #define LCD_PINS_RS                          9  // E1       JP11-11
  #define LCD_PINS_EN                          8  // E0       JP11-10
  #define LCD_PINS_D4                          7  // D7       JP11-8
  #define LCD_PINS_D5                          6  // D6       JP11-7
  #define LCD_PINS_D6                          5  // D5       JP11-6
  #define LCD_PINS_D7                          4  // D4       JP11-5

  #if ANY(VIKI2, miniVIKI)

    #define BEEPER_PIN                         8  // E0       JP11-10
    #define DOGLCD_A0                         40  // F2       JP2-2
    #define DOGLCD_CS                         41  // F3       JP2-4

    #define BTN_EN1                            2  // D2 TX1   JP2-5
    #define BTN_EN2                            3  // D3 RX1   JP2-7
    #define BTN_ENC                           45  // F7 TDI   JP2-12

    #define SD_SS_PIN                          3  // F5 TMS   JP2-8

    #define STAT_LED_RED_PIN                  12  // C2       JP11-14
    #define STAT_LED_BLUE_PIN                 10  // C0       JP11-12

    #define LCD_SCREEN_ROTATE                180  // 0, 90, 180, 270

  #elif ENABLED(MINIPANEL)

    #if DISABLED(USE_INTERNAL_SD)
      //      PIN       FASTIO PIN#  ATUSB90 PIN# Teensy2.0++ PIN#  Printrboard RevF Conn.   MKSLCD12864 PIN#
      #define SD_SS_PIN                       11  //      36               C1                EXP2-13             EXP2-07
      #define SD_DETECT_PIN                    9  //      34               E1                EXP2-11             EXP2-04
    #endif

    //      PIN       FASTIO PIN#  ATUSB90 PIN# Teensy2.0++ PIN#  Printrboard RevF Conn.   MKSLCD12864 PIN#
    #define DOGLCD_A0                          4  //       29               D4                EXP2-05             EXP1-04
    #define DOGLCD_CS                          5  //       30               D5                EXP2-06             EXP1-05
    #define BTN_ENC                            6  //       31               D6                EXP2-07             EXP1-09
    #define BEEPER_PIN                         7  //       32               D7                EXP2-08             EXP1-10
    #define KILL_PIN                           8  //       33               E0                EXP2-10             EXP2-03
    #define BTN_EN1                           10  //       35               C0                EXP2-12             EXP2-06
    #define BTN_EN2                           12  //       37               C2                EXP2-14             EXP2-08
    //#define LCD_BACKLIGHT_PIN               43  //    56               F5                EXP1-12     Not Implemented
    //#define SCK                             21  //         11               B1                ICSP-04             EXP2-09
    //#define MOSI                            22  //         12               B2                ICSP-03             EXP2-05
    //#define MISO                            23  //         13               B3                ICSP-06             EXP2-05

    // Alter timing for graphical display
    #define BOARD_ST7920_DELAY_1           313
    #define BOARD_ST7920_DELAY_2           313
    #define BOARD_ST7920_DELAY_3           313

  #else

    #define BTN_EN1                           10  // C0       JP11-12
    #define BTN_EN2                           11  // C1       JP11-13
    #define BTN_ENC                           12  // C2       JP11-14

  #endif

#endif

//
// Misc. Functions
//
//      PIN       FASTIO PIN#  ATUSB90 PIN# Teensy2.0++ PIN#  Printrboard RevF Conn.
#ifndef SDSS
  #define SD_SS_PIN                           20  //       10               B0
#endif

/**
 * This is EXP1-2, which is also the TEMP_A_PIN for the Extrudrboard.
 * If using w/ Extrudrboard, cut off pin 2 on the Extrudrboard male
 * connector to ensure this is disconnected from the A THERM pullups.
 * You probably want to set EXTRUDERS=2 and swap the Extrudrboard outputs,
 * which will let you use Channel B on the Extrudrboard as E1.
 */
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         2  // Analog Input
#endif
