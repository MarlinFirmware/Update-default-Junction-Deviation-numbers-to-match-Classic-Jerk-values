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
 * Native with a RAMPS like board with additional pins
 */

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "RAMPS Native"
#endif

#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "SimRap 1.4"
#endif

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                           151
#define SERVO1_PIN                           152
#define SERVO2_PIN                           153
#ifndef SERVO3_PIN
  #define SERVO3_PIN                         154
#endif

//
// Limit Switches
//
#define X_MIN_PIN                            155
#ifndef X_MAX_PIN
  #define X_MAX_PIN                          156
#endif
#define Y_MIN_PIN                            157
#define Y_MAX_PIN                            158
#define Z_MIN_PIN                            159
#define Z_MAX_PIN                            160

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            53
#endif

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            49
#endif

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            40
#endif

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           42
#endif

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           44
#endif

#define E2_STEP_PIN                          100
#define E2_DIR_PIN                           101
#define E2_ENABLE_PIN                        102
#ifndef E2_CS_PIN
  #define E2_CS_PIN                          103
#endif

#define E3_STEP_PIN                          104
#define E3_DIR_PIN                           105
#define E3_ENABLE_PIN                        106
#ifndef E3_CS_PIN
  #define E3_CS_PIN                          107
#endif

#define E4_STEP_PIN                          108
#define E4_DIR_PIN                           109
#define E4_ENABLE_PIN                        110
#ifndef E4_CS_PIN
  #define E4_CS_PIN                          111
#endif

#define E5_STEP_PIN                          112
#define E5_DIR_PIN                           113
#define E5_ENABLE_PIN                        114
#ifndef E5_CS_PIN
  #define E5_CS_PIN                          115
#endif

#define E6_STEP_PIN                          116
#define E6_DIR_PIN                           117
#define E6_ENABLE_PIN                        118
#ifndef E6_CS_PIN
  #define E6_CS_PIN                          119
#endif

#define E7_STEP_PIN                          120
#define E7_DIR_PIN                           121
#define E7_ENABLE_PIN                        122
#ifndef E7_CS_PIN
  #define E7_CS_PIN                          123
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                             0  // Analog Input
#define TEMP_1_PIN                             1  // Analog Input
#define TEMP_2_PIN                             2  // Analog Input
#define TEMP_3_PIN                             3  // Analog Input
#define TEMP_4_PIN                             4  // Analog Input
#define TEMP_5_PIN                             5  // Analog Input
#define TEMP_6_PIN                             6  // Analog Input
#define TEMP_7_PIN                             7  // Analog Input
#define TEMP_BED_PIN                           8  // Analog Input

// SPI for MAX Thermocouple
#if !HAS_MEDIA
  #define TEMP_0_CS_PIN                       66  // Don't use 53 if using Display/SD card
#else
  #define TEMP_0_CS_PIN                       66  // Don't use 49 (SD_DETECT_PIN)
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           9
#define HEATER_2_PIN                           8
#define HEATER_3_PIN                         125
#define HEATER_4_PIN                         126
#define HEATER_5_PIN                         127
#define HEATER_6_PIN                         128
#define HEATER_7_PIN                         129
#define HEATER_BED_PIN                       108

#ifndef FAN0_PIN
  #define FAN0_PIN                           161  // IO pin. Buffer needed
#endif
#define FAN1_PIN                             162  // IO pin. Buffer needed
#define FAN2_PIN                             163  // IO pin. Buffer needed
#define FAN3_PIN                             164  // IO pin. Buffer needed
#define FAN4_PIN                             165  // IO pin. Buffer needed
#define FAN5_PIN                             166  // IO pin. Buffer needed
#define FAN6_PIN                             167  // IO pin. Buffer needed
#define FAN7_PIN                             168  // IO pin. Buffer needed

//
// Misc. Functions
//
#define SDSS                                  53
#define LED_PIN                               13
#define BOARD_NEOPIXEL_PIN                    71

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         5  // Analog Input on AUX2
#endif

// define digital pin 4 for the filament runout sensor. Use the RAMPS 1.4 digital input 4 on the servos connector
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      21
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                           12
#endif

#if ENABLED(CASE_LIGHT_ENABLE) && !defined(CASE_LIGHT_PIN) && !defined(SPINDLE_LASER_ENA_PIN)
  #if NUM_SERVOS <= 1                             // Prefer the servo connector
    #define CASE_LIGHT_PIN                     6  // Hardware PWM
  #elif HAS_FREE_AUX2_PINS                        // try to use AUX 2
    #define CASE_LIGHT_PIN                    44  // Hardware PWM
  #endif
#endif

//
// M3/M4/M5 - Spindle/Laser Control
//
#if HAS_CUTTER && !PIN_EXISTS(SPINDLE_LASER_ENA)
  #if !defined(NUM_SERVOS) || NUM_SERVOS == 0     // Prefer the servo connector
    #define SPINDLE_LASER_PWM_PIN              6  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN              4  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN                    5
  #elif HAS_FREE_AUX2_PINS                        // try to use AUX 2
    #define SPINDLE_LASER_PWM_PIN             44  // Hardware PWM
    #define SPINDLE_LASER_ENA_PIN             40  // Pullup or pulldown!
    #define SPINDLE_DIR_PIN                   65
  #endif
#endif

//
// Průša i3 MK2 Multiplexer Support
//
#if HAS_PRUSA_MMU1
  #ifndef E_MUX0_PIN
    #define E_MUX0_PIN                        40  // Z_CS_PIN
  #endif
  #ifndef E_MUX1_PIN
    #define E_MUX1_PIN                        42  // E0_CS_PIN
  #endif
  #ifndef E_MUX2_PIN
    #define E_MUX2_PIN                        44  // E1_CS_PIN
  #endif
#endif

/**
 * Default pins for TMC SPI
 */
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                        66
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                        44
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                         64
#endif

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
  //#define E4_HARDWARE_SERIAL Serial1

  /**
   * Software serial
   */

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                   40
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                   63
  #endif
  #ifndef X2_SERIAL_TX_PIN
    #define X2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef X2_SERIAL_RX_PIN
    #define X2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                   59
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                   64
  #endif
  #ifndef Y2_SERIAL_TX_PIN
    #define Y2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Y2_SERIAL_RX_PIN
    #define Y2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                   42
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                   65
  #endif
  #ifndef Z2_SERIAL_TX_PIN
    #define Z2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Z2_SERIAL_RX_PIN
    #define Z2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  44
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                  66
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E1_SERIAL_RX_PIN
    #define E1_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_TX_PIN
    #define E2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E2_SERIAL_RX_PIN
    #define E2_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_TX_PIN
    #define E3_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E3_SERIAL_RX_PIN
    #define E3_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_TX_PIN
    #define E4_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E4_SERIAL_RX_PIN
    #define E4_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_TX_PIN
    #define E5_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E5_SERIAL_RX_PIN
    #define E5_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_TX_PIN
    #define E6_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E6_SERIAL_RX_PIN
    #define E6_SERIAL_RX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_TX_PIN
    #define E7_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E7_SERIAL_RX_PIN
    #define E7_SERIAL_RX_PIN                  -1
  #endif
#endif

/**                    Faux Expansion Headers
 *               ------                             ------
 *    (BEEP) 37 | 1  2 | 35 (ENC)        (MISO) 50 | 1  2 | 52 (SCK)
 *  (LCD_EN) 17 | 3  4 | 16 (LCD_RS)      (EN1) 31 | 3  4 | 53 (SDSS)
 *  (LCD_D4) 23   5  6 | 25 (LCD_D5)      (EN2) 33   5  6 | 51 (MOSI)
 *  (LCD_D6) 27 | 7  8 | 29 (LCD_D7)   (SD_DET) 49 | 7  8 | 41 (KILL)
 *           -- | 9 10 | --                     -- | 9 10 | --
 *               ------                             ------
 *                EXP1                               EXP2
 */
#define EXP1_01_PIN                           37  // BEEPER
#define EXP1_02_PIN                           35  // ENC
#define EXP1_03_PIN                           17  // LCD_EN
#define EXP1_04_PIN                           16  // LCD_RS
#define EXP1_05_PIN                           23  // LCD_D4
#define EXP1_06_PIN                           25  // LCD_D5
#define EXP1_07_PIN                           27  // LCD_D6
#define EXP1_08_PIN                           29  // LCD_D7

#define EXP2_01_PIN                           50  // MISO
#define EXP2_02_PIN                           52  // SCK
#define EXP2_03_PIN                           31  // EN1
#define EXP2_04_PIN                           53  // SDSS
#define EXP2_05_PIN                           33  // EN2
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           49  // SD_DET
#define EXP2_08_PIN                           41  // KILL

//
// LCD / Controller
//

#if ANY(TFT_COLOR_UI, TFT_CLASSIC_UI, TFT_LVGL_UI)

  #define TFT_CS_PIN                          49
  #define TFT_DC_PIN                          43
  #define TFT_A0_PIN                  TFT_DC_PIN
  #define TFT_SCK_PIN                 SD_SCK_PIN
  #define TFT_MISO_PIN               SD_MISO_PIN
  #define TFT_MOSI_PIN               SD_MOSI_PIN
  #define LCD_USE_DMA_SPI

  #define BEEPER_PIN                          42

  #define BTN_ENC                             59
  #define BTN_EN1                             40
  #define BTN_EN2                             63

  #define TOUCH_CS_PIN                        33

  #define SD_DETECT_PIN                       41

  #define SPI_FLASH
  #if ENABLED(SPI_FLASH)
    #define SPI_DEVICE                         1  // Maple
    #define SPI_FLASH_SIZE             0x1000000  // 16MB
    #define SPI_FLASH_CS_PIN                  31
    #define SPI_FLASH_SCK_PIN         SD_SCK_PIN
    #define SPI_FLASH_MISO_PIN       SD_MISO_PIN
    #define SPI_FLASH_MOSI_PIN       SD_MOSI_PIN
  #endif

  #define TFT_BUFFER_WORDS                0xFFFF
  #ifndef TFT_DRIVER
    #define TFT_DRIVER                    ST7796
  #endif
  #if DISABLED(TOUCH_SCREEN_CALIBRATION)
    #if ENABLED(TFT_RES_320x240)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X        20525
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        15335
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X                -1
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y                 0
      #endif
    #elif ENABLED(TFT_RES_480x272)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X        30715
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        17415
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X                 0
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y                -1
      #endif
    #elif ENABLED(TFT_RES_480x320)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X        30595
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        20415
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X                 2
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y                 1
      #endif
    #elif ENABLED(TFT_RES_1024x600)
      #ifndef TOUCH_CALIBRATION_X
        #define TOUCH_CALIBRATION_X        65533
      #endif
      #ifndef TOUCH_CALIBRATION_Y
        #define TOUCH_CALIBRATION_Y        38399
      #endif
      #ifndef TOUCH_OFFSET_X
        #define TOUCH_OFFSET_X                 2
      #endif
      #ifndef TOUCH_OFFSET_Y
        #define TOUCH_OFFSET_Y                 1
      #endif
    #endif
  #endif

  #define BTN_BACK                            70

#elif HAS_WIRED_LCD

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

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      #define LCD_PINS_RS            EXP1_07_PIN
      #define LCD_PINS_EN            EXP1_08_PIN
      #define LCD_PINS_D4            EXP1_06_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP1_01_PIN
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      #define LCD_PINS_RS                     64
      #define LCD_PINS_EN                     44
      #define LCD_PINS_D4                     63
      #define LCD_PINS_D5                     40
      #define LCD_PINS_D6                     42
      #define LCD_PINS_D7                     65

    #else

      #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
        #define LCD_PINS_DC          EXP1_06_PIN  // Set as output on init
        #define LCD_PINS_RS          EXP1_07_PIN  // Pull low for 1s to init
        // DOGM SPI LCD Support
        #define DOGLCD_CS            EXP1_04_PIN
        #define DOGLCD_MOSI          EXP1_03_PIN
        #define DOGLCD_SCK           EXP1_05_PIN
        #define DOGLCD_A0            LCD_PINS_DC
      #else
        #define LCD_PINS_RS          EXP1_04_PIN
        #define LCD_PINS_EN          EXP1_03_PIN
        #define LCD_PINS_D4          EXP1_05_PIN
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
      #endif

      #define LCD_PINS_D7            EXP1_08_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP2_05_PIN
      #endif

    #endif
  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)

      #define BEEPER_PIN             EXP1_01_PIN

      #if ENABLED(CR10_STOCKDISPLAY)
        #define BTN_EN1              EXP1_03_PIN
        #define BTN_EN2              EXP1_05_PIN
      #else
        #define BTN_EN1              EXP2_03_PIN
        #define BTN_EN2              EXP2_05_PIN
      #endif

      #define BTN_ENC                EXP1_02_PIN
      #define SD_DETECT_PIN          EXP2_07_PIN
      #define KILL_PIN               EXP2_08_PIN

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
      #define LCD_SDSS                      SDSS
      #define KILL_PIN                        41
      #undef LCD_PINS_EN                          // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      #define BTN_EN1                         22  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      #define BTN_EN2                          7  // 22/7 are unused on RAMPS_14. 22 is unused and 7 the SERVO0_PIN on RAMPS_13.
      #define BTN_ENC                         -1

      #define LCD_SDSS                      SDSS
      #define SD_DETECT_PIN                   49

    #elif ANY(VIKI2, miniVIKI)

      #define DOGLCD_CS                       45
      #define DOGLCD_A0                       44

      #define BEEPER_PIN                      33
      #define STAT_LED_RED_PIN                32
      #define STAT_LED_BLUE_PIN               35

      #define BTN_EN1                         22
      #define BTN_EN2                          7
      #define BTN_ENC                         39

      #define SD_DETECT_PIN                   -1  // Pin 49 for display sd interface, 72 for easy adapter board
      #define KILL_PIN                        31

      #define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      #define DOGLCD_CS              EXP1_08_PIN
      #define DOGLCD_A0              EXP1_07_PIN

      #define BEEPER_PIN             EXP1_05_PIN
      #define LCD_BACKLIGHT_PIN      EXP2_05_PIN

      #define BTN_EN1                EXP1_02_PIN
      #define BTN_EN2                EXP1_01_PIN
      #define BTN_ENC                EXP2_03_PIN

      #define LCD_SDSS                      SDSS
      #define SD_DETECT_PIN          EXP2_07_PIN
      #define KILL_PIN               EXP2_08_PIN

    #elif ENABLED(MKS_MINI_12864)

      #define DOGLCD_A0              EXP1_07_PIN
      #define DOGLCD_CS              EXP1_06_PIN

      #define BEEPER_PIN             EXP1_01_PIN
      // not connected to a pin
      #define LCD_BACKLIGHT_PIN               65  // backlight LED on A11/D65

      #define BTN_EN1                EXP2_03_PIN
      #define BTN_EN2                EXP2_05_PIN
      #define BTN_ENC                EXP1_02_PIN

      #define SD_DETECT_PIN          EXP2_07_PIN
      #define KILL_PIN                        64

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

      #define SD_DETECT_PIN                   49
      #define KILL_PIN                        64

      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270

    #elif ENABLED(ZONESTAR_LCD)

      #define ADC_KEYPAD_PIN                  12

    #elif ENABLED(AZSMZ_12864)

      // Pins only defined for RAMPS_SMART currently

    #else

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
        #define BTN_EN1              EXP1_01_PIN
        #define BTN_EN2              EXP1_02_PIN
        #define BTN_ENC              EXP2_03_PIN
        #define SD_DETECT_PIN        EXP2_08_PIN
      #endif

      #if ENABLED(G3D_PANEL)
        #define SD_DETECT_PIN        EXP2_07_PIN
        #define KILL_PIN             EXP2_08_PIN
      #endif
    #endif

    // CUSTOM SIMULATOR INPUTS
    #define BTN_BACK                          70

  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD

#ifndef KILL_PIN
  #define KILL_PIN                   EXP2_08_PIN
#endif
