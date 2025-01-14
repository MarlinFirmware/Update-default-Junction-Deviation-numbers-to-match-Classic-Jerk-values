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
 * TT OSCAR by YM Tech.LTD
 *
 * ATmega2560
 */

#include "env_validate.h"

#if HOTENDS > 5 || E_STEPPERS > 5
  #error "TT OSCAR supports up to 5 hotends / E steppers."
#endif

#define BOARD_INFO_NAME      "TT OSCAR"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

//
// Servos
//
#define SERVO0_PIN                            11
#define SERVO1_PIN                            12
#define SERVO2_PIN                             5
#define SERVO3_PIN                             4

//
// Limit Switches
//
#define X_MIN_PIN                              3
#define X_MAX_PIN                              2
#define Y_MIN_PIN                             14
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                             18
#define Z_MAX_PIN                             19

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN             SERVO3_PIN
#endif

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#define X_CS_PIN                              57

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#define Y_CS_PIN                              58

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62
#define Z_CS_PIN                              53  // EXP2-4

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#define E0_CS_PIN                             49  // EXP2-7

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#define E1_CS_PIN                      E0_CS_PIN

#define E2_STEP_PIN                           63
#define E2_DIR_PIN                            22
#define E2_ENABLE_PIN                         59
#define E2_CS_PIN                      E0_CS_PIN

#define E3_STEP_PIN                           32
#define E3_DIR_PIN                            40
#define E3_ENABLE_PIN                         39
#define E3_CS_PIN                      E0_CS_PIN

#define E4_STEP_PIN                           43
#define E4_DIR_PIN                            42
#define E4_ENABLE_PIN                         47
#define E4_CS_PIN                      E0_CS_PIN

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1

  #define X_SERIAL_TX_PIN                     -1  // 59
  //#define X_SERIAL_RX_PIN                   -1  // 63
  #define X2_SERIAL_TX_PIN                    -1

  #define Y_SERIAL_TX_PIN                     -1  // 64
  //#define Y_SERIAL_RX_PIN                   -1  // 40
  #define Y2_SERIAL_TX_PIN                    -1

  #define Z_SERIAL_TX_PIN                     -1  // 44
  //#define Z_SERIAL_RX_PIN                   -1  // 42
  #define Z2_SERIAL_TX_PIN                    -1

  #define E0_SERIAL_TX_PIN                    -1  // 66
  //#define E0_SERIAL_RX_PIN                  -1  // 65
  #define E1_SERIAL_TX_PIN                    -1
  #define E2_SERIAL_TX_PIN                    -1
  #define E3_SERIAL_TX_PIN                    -1
  #define E4_SERIAL_TX_PIN                    -1
  #define E5_SERIAL_TX_PIN                    -1
  #define E6_SERIAL_TX_PIN                    -1
  #define E7_SERIAL_TX_PIN                    -1
#endif

//
// Default pins for TMC software SPI
//
//#ifndef TMC_SPI_MOSI
//  #define TMC_SPI_MOSI                      66
//#endif
//#ifndef TMC_SPI_MISO
//  #define TMC_SPI_MISO                      44
//#endif
//#ifndef TMC_SPI_SCK
//  #define TMC_SPI_SCK                       64
//#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                            13
#define TEMP_1_PIN                            15
#define TEMP_2_PIN                            10
#define TEMP_3_PIN                            11
#define TEMP_BED_PIN                          14

#if TEMP_SENSOR_CHAMBER > 0
  #define TEMP_CHAMBER_PIN                    12
#else
  #define TEMP_4_PIN                          12
#endif

// SPI for MAX Thermocouple
//#if !HAS_MEDIA
//  #define TEMP_0_CS_PIN    66   // Don't use 53 if using Display/SD card
//#else
//  #define TEMP_0_CS_PIN    66   // Don't use 49 (SD_DETECT_PIN)
//#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           7
#define HEATER_2_PIN                          44
#define HEATER_BED_PIN                         8

#define FAN0_PIN                               9

#if EXTRUDERS >= 5
  #define HEATER_4_PIN                         6
#else
  #define FAN1_PIN                             6
#endif

#if EXTRUDERS >= 4
  #define HEATER_3_PIN                        45
#else
  #define FAN2_PIN                            45
#endif

//
// Misc. Functions
//
#define SDSS                                  53  // EXP2-4
#define LED_PIN                               13

//#ifndef FILWIDTH_PIN
//  #define FILWIDTH_PIN                       5  // Analog Input
//#endif

// DIO 4 (Servos plug) for the runout sensor.
//#define FIL_RUNOUT_PIN              SERVO3_PIN

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER && !PIN_EXISTS(SPINDLE_LASER_ENA)
  #if !NUM_SERVOS                                 // Prefer the servo connector
    #define SPINDLE_LASER_PWM_PIN              6  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN              4  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN                    5
  #elif HAS_FREE_AUX2_PINS                        // Try to use AUX 2
    #define SPINDLE_LASER_PWM_PIN             44  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN             40  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN                   65
  #endif
#endif

//
// Case Light
//
#if ENABLED(CASE_LIGHT_ENABLE) && !PIN_EXISTS(CASE_LIGHT) && !defined(SPINDLE_LASER_ENA_PIN)
  #if !NUM_SERVOS                                 // Prefer the servo connector
    #define CASE_LIGHT_PIN                     6  // Hardware PWM
  #elif HAS_FREE_AUX2_PINS                        // Try to use AUX 2
    #define CASE_LIGHT_PIN                    44  // Hardware PWM
  #endif
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#if 0 && HAS_PRUSA_MMU1
  #ifndef E_MUX0_PIN
    #define E_MUX0_PIN                        58  // Y_CS_PIN
  #endif
  #ifndef E_MUX1_PIN
    #define E_MUX1_PIN                        53  // EXP2-4
  #endif
  #ifndef E_MUX2_PIN
    #define E_MUX2_PIN                        49  // EXP2-7
  #endif
#endif

/**           TT OSCAR Expansion Headers
 *         ------
 *     -- | 1  2 | --
 *     --   3  4 | --
 *     --   5  6 | --
 *     49 | 7  8 | --
 *         ------
 *          AUX1
 *
 *         ------                     ------
 *     37 | 1  2 | 35      (MISO) 44 | 1  2 | 52 (SCK)
 *     17 | 3  4 | 41?            35 | 3  4 | 53
 *     23   5  6 | 25             31   5  6 | 51 (MOSI)
 *     27 | 7  8 | 29             49 | 7  8 | 41
 *    GND | 9 10 | 5V            GND | 9 10 | --
 *         ------                     ------
 *          EXP1                       EXP2
 */
#define EXP1_01_PIN                           37  // BEEPER
#define EXP1_02_PIN                           35  // ENC
#define EXP1_03_PIN                           17  // ENC1
#define EXP1_04_PIN                           41  // RESET
#define EXP1_05_PIN                           23  // ENC2
#define EXP1_06_PIN                           25  // D4
#define EXP1_07_PIN                           27  // RS
#define EXP1_08_PIN                           29  // EN

#define EXP2_01_PIN                           44  // MISO
#define EXP2_02_PIN                           52  // SCK
#define EXP2_03_PIN                           35  // EN2 / EN1
#define EXP2_04_PIN                           53  // SDSS
#define EXP2_05_PIN                           31  // EN1 / EN2
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           49  // SD_DET
#define EXP2_08_PIN                           41  // KILL / RESET

//
// LCD / Controller
//

#if HAS_WIRED_LCD

  //
  // LCD Display output pins
  //
  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS              EXP2_07_PIN  // CS chip select /SS chip slave select
    #define LCD_PINS_EN              EXP2_06_PIN  // SID (MOSI)
    #define LCD_PINS_D4              EXP2_02_PIN  // SCK (CLK) clock

  #elif ALL(IS_NEWPANEL, PANEL_ONE)

    #define LCD_PINS_RS                       40
    #define LCD_PINS_EN                       42
    #define LCD_PINS_D4                       65
    #define LCD_PINS_D5                       66
    #define LCD_PINS_D6                       44
    #define LCD_PINS_D7                       64

  #elif ENABLED(ZONESTAR_LCD)

    #define LCD_PINS_RS                       64
    #define LCD_PINS_EN                       44
    #define LCD_PINS_D4                       63
    #define LCD_PINS_D5                       40
    #define LCD_PINS_D6                       42
    #define LCD_PINS_D7                       65
    #define ADC_KEYPAD_PIN                    12

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS                     27
      #define LCD_PINS_EN                     29
      #define LCD_PINS_D4                     25

      #if !IS_NEWPANEL
        #define BEEPER_PIN                    37
      #endif

    #else

      #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC                   25  // Set as output on init
        #define LCD_PINS_RS                   27  // Pull low for 1s to init
        // DOGM SPI LCD Support
        #define DOGLCD_CS                     16
        #define DOGLCD_MOSI                   17
        #define DOGLCD_SCK                    23
        #define DOGLCD_A0            LCD_PINS_DC
      #else
        #define LCD_PINS_RS                   16
        #define LCD_PINS_EN                   17
        #define LCD_PINS_D4                   23
        #define LCD_PINS_D5                   25
        #define LCD_PINS_D6                   27
      #endif

      #define LCD_PINS_D7                     29

      #if !IS_NEWPANEL
        #define BEEPER_PIN                    33
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN                 38
      //#define SHIFT_LD_PIN                  42
      //#define SHIFT_OUT_PIN                 40
      //#define SHIFT_EN_PIN                  17
    #endif

  #endif

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if IS_RRD_SC

      #define BEEPER_PIN                      37

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1                       17
        #define BTN_EN2                       23
      #else
        #define BTN_EN1                       31
        #define BTN_EN2                       33
      #endif

      #define BTN_ENC                         35
      #define SD_DETECT_PIN          EXP2_07_PIN
      //#define KILL_PIN                      41

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        #define LCD_BACKLIGHT_PIN             39
      #endif

    #elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

      #define BTN_EN1                         64
      #define BTN_EN2                         59
      #define BTN_ENC                         63
      #define SD_DETECT_PIN                   42

    #elif ENABLED(LCD_I2C_PANELOLU2)

      #define BTN_EN1                         47
      #define BTN_EN2                         43
      #define BTN_ENC                         32
      #define LCD_SDSS               EXP2_04_PIN
      //#define KILL_PIN                      41
      #undef LCD_PINS_EN                          // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                         22  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      #define BTN_EN2                          7  // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
      #define BTN_ENC                         -1

      #define LCD_SDSS               EXP2_04_PIN
      #define SD_DETECT_PIN          EXP2_07_PIN

    #elif ANY(VIKI2, miniVIKI)

      #define DOGLCD_CS                       45
      #define DOGLCD_A0                       44

      #define BEEPER_PIN                      33
      #define STAT_LED_RED_PIN                32
      #define STAT_LED_BLUE_PIN               35

      #define BTN_EN1                         22
      #define BTN_EN2                          7
      #define BTN_ENC                         39

      #define SDSS                   EXP2_04_PIN
      #define SD_DETECT_PIN                   -1  // Pin 49 for display SD interface, 72 for easy adapter board
      //#define KILL_PIN                      31

      #define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS                       29
      #define DOGLCD_A0                       27

      #define BEEPER_PIN                      23
      #define LCD_BACKLIGHT_PIN               33

      #define BTN_EN1                         35
      #define BTN_EN2                         37
      #define BTN_ENC                         31

      #define LCD_SDSS               EXP2_04_PIN
      #define SD_DETECT_PIN          EXP2_07_PIN
      //#define KILL_PIN                      41

    #elif ENABLED(MKS_MINI_12864)

      #define DOGLCD_A0                       27
      #define DOGLCD_CS                       25

      #define BEEPER_PIN                      37

      #define LCD_BACKLIGHT_PIN               65  // backlight LED on A11/D65

      #define BTN_EN1                         31
      #define BTN_EN2                         33
      #define BTN_ENC                         35
      //#define SDSS                 EXP2_04_PIN
      #define SD_DETECT_PIN          EXP2_07_PIN
      //#define KILL_PIN                      64

      //#define LCD_CONTRAST_INIT            190
      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270

    #elif ENABLED(MINIPANEL)

      #define BEEPER_PIN                      42
      // not connected to a pin
      #define LCD_BACKLIGHT_PIN               65  // backlight LED on A11/D65

      #define DOGLCD_A0                       44
      #define DOGLCD_CS                       66

      #define BTN_EN1                         40
      #define BTN_EN2                         63
      #define BTN_ENC                         59

      #define SDSS                   EXP2_04_PIN
      #define SD_DETECT_PIN          EXP2_07_PIN
      //#define KILL_PIN                      64

      //#define LCD_CONTRAST_INIT            190
      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270

    #else

      // Beeper on AUX-4
      #define BEEPER_PIN                      33

      // Buttons are directly attached to AUX-2
      #if IS_RRW_KEYPAD
        #define SHIFT_OUT_PIN                 40
        #define SHIFT_CLK_PIN                 44
        #define SHIFT_LD_PIN                  42
        #define BTN_EN1                       64
        #define BTN_EN2                       59
        #define BTN_ENC                       63
      #elif ENABLED(PANEL_ONE)
        #define BTN_EN1                       59  // AUX2 PIN 3
        #define BTN_EN2                       63  // AUX2 PIN 4
        #define BTN_ENC                       49  // AUX3 PIN 7
      #else
        #define BTN_EN1                       37
        #define BTN_EN2                       35
        #define BTN_ENC                       31
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN        EXP2_07_PIN
        //#define KILL_PIN                    41
      #endif

    #endif

  #endif // IS_NEWPANEL

#endif
