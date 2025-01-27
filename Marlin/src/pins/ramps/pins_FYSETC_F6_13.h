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

//
// FYSETC F6 1.3 (and 1.4) pin assignments
// Schematic: https://github.com/FYSETC/FYSETC-F6/blob/master/Hardware/V1.3/F6_V13.pdf
// ATmega2560
//

#if NOT_TARGET(__AVR_ATmega2560__)
  #error "Oops! Select 'FYSETC F6' in 'Tools > Board.'"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "FYSETC F6 1.3"
#endif

#define RESET_PIN                             30
#define SPI_FLASH_CS_PIN                      83

//
// Servos
//
#define SERVO0_PIN                            13
#define SERVO1_PIN                            11  // (PS_ON_PIN)
#define SERVO2_PIN                            10  // (FIL_RUNOUT_PIN)
#define SERVO3_PIN                             4  // (RGB_LED_G_PIN)

//
// Limit Switches
//
#define X_MIN_PIN                             63
#define X_MAX_PIN                             64
#define Y_MIN_PIN                             14
#define Y_MAX_PIN                             15
#define Z_MIN_PIN                             12
#ifndef Z_MAX_PIN
  #define Z_MAX_PIN                            9
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN              SERVO2_PIN
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                      9  // Servos pin
#endif

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38
#ifndef X_CS_PIN
  #define X_CS_PIN                            70
#endif

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56
#ifndef Y_CS_PIN
  #define Y_CS_PIN                            39
#endif

#define Z_STEP_PIN                            43
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          58
#ifndef Z_CS_PIN
  #define Z_CS_PIN                            74
#endif

#define E0_STEP_PIN                           26
#define E0_DIR_PIN                            28
#define E0_ENABLE_PIN                         24
#ifndef E0_CS_PIN
  #define E0_CS_PIN                           47
#endif

#define E1_STEP_PIN                           36
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         30
#ifndef E1_CS_PIN
  #define E1_CS_PIN                           32
#endif

#define E2_STEP_PIN                           59
#define E2_DIR_PIN                            57
#define E2_ENABLE_PIN                         40
#ifndef E2_CS_PIN
  #define E2_CS_PIN                           42
#endif

//
// Sensorless homing DIAG pin is not directly connected to the MCU. Close
// the jumper next to the limit switch socket when using sensorless homing.
//

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Software serial communication pins.
   * At the moment, F6 rx pins are not pc interrupt pins
   */
  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN                   72
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN                   -1  // 71
  #endif
  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN                   75
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN                   -1  // 73
  #endif
  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN                   79
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN                   -1  // 78
  #endif
  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  77
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                  -1  // 76
  #endif
  #ifndef E1_SERIAL_TX_PIN
    #define E1_SERIAL_TX_PIN                  81
  #endif
  #ifndef E1_SERIAL_RX_PIN
    #define E1_SERIAL_RX_PIN                  -1  // 80
  #endif
  #ifndef E2_SERIAL_TX_PIN
    #define E2_SERIAL_TX_PIN                  82
  #endif
  #ifndef E2_SERIAL_RX_PIN
    #define E2_SERIAL_RX_PIN                  -1  // 22
  #endif
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                            12  // Analog Input
#define TEMP_1_PIN                            13  // Analog Input
#define TEMP_2_PIN                            14  // Analog Input
#define TEMP_BED_PIN                          15  // Analog Input

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                         9  // Analog Input on X+ endstop
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                           5
#define HEATER_1_PIN                           6
#define HEATER_2_PIN                           7
#define HEATER_BED_PIN                         8

#define FAN0_PIN                              44
#define FAN1_PIN                              45
#define FAN2_PIN                              46

//
// Misc. Functions
//
#define LED_PIN                               13
#define KILL_PIN                              41

#ifndef PS_ON_PIN
  #define PS_ON_PIN                   SERVO1_PIN
#endif

/**
 *                ------                                    ------
 *  (BEEPER) D37 | 1  2 | D35 (BTN_ENC)    (MISO)      D50 | 1  2 | D52 (SCK)
 *  (LCD_EN) D17 | 3  4 | D16 (LCD_RS)     (BTN_EN1)   D31 | 3  4 | D53 (SD_SS)
 *  (LCD_D4) D23   5  6 | D25 (LCD_D5)     (BTN_EN2)   D33   5  6 | D51 (MOSI)
 *  (LCD_D6) D27 | 7  8 | D29 (LCD_D7)     (SD_DETECT) D49 | 7  8 | RESET
 *           GND | 9 10 | 5V                           GND | 9 10 | 5V / D41
 *                ------                                    ------
 *                 EXP1                                      EXP2
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
#define EXP2_04_PIN                           53  // SD_SS
#define EXP2_05_PIN                           33  // EN2
#define EXP2_06_PIN                           51  // MOSI
#define EXP2_07_PIN                           49  // SD_DETECT
#define EXP2_08_PIN                           -1  // RESET

//
// SD Card
//

#define SD_SS_PIN                    EXP2_04_PIN
#define SD_DETECT_PIN                EXP2_07_PIN

//
// LCD / Controller
//

#if ENABLED(FYSETC_242_OLED_12864)
  #define BTN_EN1                    EXP1_01_PIN
  #define BTN_EN2                    EXP1_08_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define BEEPER_PIN                 EXP2_03_PIN

  #define LCD_PINS_DC                EXP1_06_PIN
  #define LCD_PINS_RS                EXP2_05_PIN
  #define DOGLCD_CS                  EXP1_04_PIN
  #define DOGLCD_MOSI                EXP1_05_PIN
  #define DOGLCD_SCK                 EXP1_03_PIN
  #define DOGLCD_A0                  LCD_PINS_DC

  #undef KILL_PIN
  #define BOARD_NEOPIXEL_PIN         EXP1_07_PIN

#else
  #define BEEPER_PIN                 EXP1_01_PIN

  #if ENABLED(FYSETC_MINI_12864)
    //
    // See https://wiki.fysetc.com/Mini12864_Panel/
    //
    #define DOGLCD_A0                EXP1_04_PIN
    #define DOGLCD_CS                EXP1_03_PIN

    #if ENABLED(FYSETC_GENERIC_12864_1_1)
      #define LCD_BACKLIGHT_PIN      EXP1_07_PIN
    #endif

    #define LCD_RESET_PIN            EXP1_05_PIN  // Must be high or open for LCD to operate normally.
                                                  // Seems to work best if left open.

    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN        EXP1_06_PIN
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN        EXP1_07_PIN
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN        EXP1_08_PIN
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN           EXP1_06_PIN
    #endif

  #elif HAS_MARLINUI_U8GLIB || HAS_MARLINUI_HD44780

    #define LCD_PINS_RS              EXP1_04_PIN
    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN
    #define LCD_PINS_D5              EXP1_06_PIN
    #define LCD_PINS_D6              EXP1_07_PIN
    #define LCD_PINS_D7              EXP1_08_PIN

    #if ENABLED(MKS_MINI_12864)
      #define DOGLCD_CS              EXP1_06_PIN
      #define DOGLCD_A0              EXP1_07_PIN
    #endif

    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      #define BTN_ENC_EN             LCD_PINS_D7  // Detect the presence of the encoder
    #endif

  #endif

  #if IS_NEWPANEL
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN
    #define BTN_ENC                  EXP1_02_PIN
  #endif
#endif

#ifndef RGB_LED_R_PIN
  #define RGB_LED_R_PIN                        3
#endif
#ifndef RGB_LED_G_PIN
  #define RGB_LED_G_PIN                        4
#endif
#ifndef RGB_LED_B_PIN
  #define RGB_LED_B_PIN                        9
#endif
#ifndef RGB_LED_W_PIN
  #define RGB_LED_W_PIN                       -1
#endif
