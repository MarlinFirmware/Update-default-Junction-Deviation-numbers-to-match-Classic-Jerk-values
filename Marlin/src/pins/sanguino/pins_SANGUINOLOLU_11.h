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
 * Sanguinololu board pin assignments
 * Schematic (0.1): https://github.com/mosfet/Sanguinololu/blob/master/rev0.1/sanguinololu.sch
 * Schematic (0.6): https://github.com/mosfet/Sanguinololu/blob/master/rev0.6/images/schematic.jpg
 * Schematic (0.7): https://github.com/mosfet/Sanguinololu/blob/master/rev0.7/images/schematic.jpg
 * Schematic (1.0): https://reprap.org/wiki/File:Sanguinololu-schematic.jpg
 * Schematic (1.1): https://github.com/mosfet/Sanguinololu/blob/master/rev1.1/sanguinololu.sch
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

#define ALLOW_MEGA644P
#include "env_validate.h"

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Sanguinololu <1.2"
#endif

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

#define Y_STEP_PIN                            22
#define Y_DIR_PIN                             23

#define Z_STEP_PIN                             3
#define Z_DIR_PIN                              2

#define E0_STEP_PIN                            1
#define E0_DIR_PIN                             0

//
// Temperature Sensors
//
#define TEMP_0_PIN                             7  // Analog Input (pin 33 extruder)
#define TEMP_BED_PIN                           6  // Analog Input (pin 34 bed)

//
// Heaters / Fans
//
#define HEATER_0_PIN                          13  // (extruder)

#ifndef FAN0_PIN
  #define FAN0_PIN                             4  // Works for Panelolu2 too
#endif

#if DISABLED(SANGUINOLOLU_V_1_2)
  #define HEATER_BED_PIN                      14  // (bed)
  #define X_ENABLE_PIN                         4
  #define Y_ENABLE_PIN                         4
  #ifndef Z_ENABLE_PIN
    #define Z_ENABLE_PIN                       4
  #endif
  #define E0_ENABLE_PIN                        4
#else
  #if !HAS_CUTTER && !ALL(HAS_WIRED_LCD, IS_NEWPANEL) // Use IO Header
    #define CASE_LIGHT_PIN                     4  // Hardware PWM  - see if IO Header is available
  #endif
#endif

//
// Misc. Functions
//

/**
 * In some versions of the Sanguino libraries the pin
 * definitions are wrong, with SDSS = 24 and LED_PIN = 28 (Melzi).
 * If you encounter issues with these pins, upgrade your
 * Sanguino libraries! See #368.
 */
//#define SDSS                                24
#define SDSS                             AUX1_09

#if IS_MELZI
  #define LED_PIN                        AUX1_01
#elif MB(STB_11)
  #define LCD_BACKLIGHT_PIN              AUX1_04  // LCD backlight LED
#endif

/**
 * Sanguinololu 1.4 AUX pins:
 *
 *           PWM  TX1  RX1  SDA  SCL
 *  12V  5V  D12  D11  D10  D17  D16
 *  GND GND  D31  D30  D29  D28  D27
 *            A4   A3   A2   A1   A0
 */
#define AUX1_01                               27  // A0
#define AUX1_02                               16  // SCL
#define AUX1_03                               28  // A1
#define AUX1_04                               17  // SDA
#define AUX1_05                               29  // A2
#define AUX1_06                               10  // RX1
#define AUX1_07                               30  // A3
#define AUX1_08                               11  // TX1
#define AUX1_09                               31  // A4
#define AUX1_10                               12  // PWM

//
// LCD / Controller
//

#ifdef LCD_PINS_DEFINED

  // LCD pins already defined by including header

#elif HAS_WIRED_LCD

  #define SD_DETECT_PIN                       -1

  #if HAS_MARLINUI_U8GLIB

    #if ENABLED(LCD_FOR_MELZI)

      #define LCD_PINS_RS                AUX1_04
      #define LCD_PINS_EN                AUX1_02
      #define LCD_PINS_D4                AUX1_08
      #define KILL_PIN                   AUX1_06
      #define BEEPER_PIN                 AUX1_01

    #elif IS_U8GLIB_ST7920                        // SPI GLCD 12864 ST7920 ( like [www.digole.com] ) For Melzi V2.0

      #if IS_MELZI
        #define LCD_PINS_RS              AUX1_07  // CS chip select /SS chip slave select
        #define LCD_PINS_EN              AUX1_05  // SID (MOSI)
        #define LCD_PINS_D4              AUX1_04  // SCK (CLK) clock
        // Pin 27 is taken by LED_PIN, but Melzi LED does nothing with
        // Marlin so this can be used for BEEPER_PIN. You can use this pin
        // with M42 instead of BEEPER_PIN.
        #define BEEPER_PIN               AUX1_01
      #else // Sanguinololu >=1.3
        #define LCD_PINS_RS                    4
        #define LCD_PINS_EN              AUX1_04
        #define LCD_PINS_D4              AUX1_07
        #define LCD_PINS_D5              AUX1_05
        #define LCD_PINS_D6              AUX1_03
        #define LCD_PINS_D7              AUX1_01
      #endif

    #else

      #define DOGLCD_A0                  AUX1_07

      #if ENABLED(MAKRPANEL)

        #define BEEPER_PIN               AUX1_05
        #define DOGLCD_CS                AUX1_04
        #define LCD_BACKLIGHT_PIN        AUX1_03  // PA3

      #elif IS_MELZI

        #define BEEPER_PIN               AUX1_01
        #ifndef DOGLCD_CS
          #define DOGLCD_CS              AUX1_03
        #endif

      #else

        #ifndef DOGLCD_CS
          #define DOGLCD_CS              AUX1_05
        #endif

      #endif

    #endif

    //#define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

  #elif ENABLED(ZONESTAR_LCD)                     // For the Tronxy Melzi boards

    #define LCD_PINS_RS                  AUX1_03
    #define LCD_PINS_EN                  AUX1_05
    #define LCD_PINS_D4                  AUX1_06
    #define LCD_PINS_D5                  AUX1_08
    #define LCD_PINS_D6                  AUX1_02
    #define LCD_PINS_D7                  AUX1_04

  #else

    #define LCD_PINS_RS                        4
    #define LCD_PINS_EN                  AUX1_04
    #define LCD_PINS_D4                  AUX1_07
    #define LCD_PINS_D5                  AUX1_05
    #define LCD_PINS_D6                  AUX1_03
    #define LCD_PINS_D7                  AUX1_01

  #endif

  #if ENABLED(LCD_FOR_MELZI)

    #define BTN_ENC                      AUX1_03
    #define BTN_EN1                      AUX1_05
    #define BTN_EN2                      AUX1_07

  #elif ENABLED(ZONESTAR_LCD)                     // For the Tronxy Melzi boards

    #define ADC_KEYPAD_PIN                     1
    #define BTN_EN1                           -1
    #define BTN_EN2                           -1

  #elif ENABLED(LCD_I2C_PANELOLU2)

    #if IS_MELZI
      #define BTN_ENC                    AUX1_05
      #ifndef LCD_SDSS
        #define LCD_SDSS                 AUX1_07  // Panelolu2 SD card reader rather than the Melzi
      #endif
    #else
      #define BTN_ENC                    AUX1_07
    #endif
    #undef LCD_PINS_EN                            // not used, causes false pin conflict report

  #else // !LCD_FOR_MELZI && !ZONESTAR_LCD && !LCD_I2C_PANELOLU2

    #define BTN_ENC                      AUX1_02
    #ifndef LCD_SDSS
      #define LCD_SDSS                   AUX1_03  // Smart Controller SD card reader rather than the Melzi
    #endif

  #endif

  #if IS_NEWPANEL && !defined(BTN_EN1)
    #define BTN_EN1                      AUX1_08
    #define BTN_EN2                      AUX1_06
  #endif

#endif // HAS_WIRED_LCD

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER
  #if !MB(AZTEEG_X1) && ENABLED(SANGUINOLOLU_V_1_2) && !ALL(HAS_WIRED_LCD, IS_NEWPANEL) // try to use IO Header

    #define SPINDLE_LASER_PWM_PIN              4  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN        AUX1_06  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN              AUX1_08

  #elif !MB(MELZI)                                // use X stepper motor socket

    /**
     *  To control the spindle speed and have an LCD you must sacrifice
     *  the Extruder and pull some signals off the X stepper driver socket.
     *
     *  The following assumes:
     *   - The X stepper driver socket is empty
     *   - The extruder driver socket has a driver board plugged into it
     *   - The X stepper wires are attached the the extruder connector
     */

    /**
     *  Where to get the spindle signals
     *
     *      spindle signal          socket name       socket name
     *                                         -------
     *                               /ENABLE  O|     |O  VMOT
     *                                   MS1  O|     |O  GND
     *                                   MS2  O|     |O  2B
     *                                   MS3  O|     |O  2A
     *                                /RESET  O|     |O  1A
     *                                /SLEEP  O|     |O  1B
     *  SPINDLE_LASER_PWM_PIN           STEP  O|     |O  VDD
     *  SPINDLE_LASER_ENA_PIN            DIR  O|     |O  GND
     *                                         -------
     *
     *  Note: Socket names vary from vendor to vendor.
     */
    #undef X_DIR_PIN
    #undef X_ENABLE_PIN
    #undef X_STEP_PIN
    #define X_DIR_PIN                          0
    #define X_ENABLE_PIN                      14
    #define X_STEP_PIN                         1
    #define SPINDLE_LASER_PWM_PIN             15  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN             21  // Pullup!
    #define SPINDLE_DIR_PIN                   -1  // No pin available on the socket for the direction pin
  #endif
#endif
