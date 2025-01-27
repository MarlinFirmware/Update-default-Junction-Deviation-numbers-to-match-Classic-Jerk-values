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

/****************************************************************************************
* Teensy 3.5 (MK64FX512) and Teensy 3.6 (MK66FX1M0) Breadboard pin assignments
* Requires the Teensyduino software with Teensy 3.5 or Teensy 3.6 selected in Arduino IDE!
* https://www.pjrc.com/teensy/teensyduino.html
****************************************************************************************/

#if NOT_TARGET(IS_TEENSY_35_36)
  #error "Oops! Select 'Teensy 3.5' or 'Teensy 3.6' in 'Tools > Board.'"
#endif

#if IS_TEENSY35
  #define BOARD_INFO_NAME "Teensy3.5"
#elif IS_TEENSY36
  #define BOARD_INFO_NAME "Teensy3.6"
#endif

/**
 * Plan for Teensy 3.5 and Teensy 3.6:
 *                                                       USB
 *                                            GND |-----#####-----| VIN 5V
 *        X_STEP_PIN          MOSI1   RX1       0 |     #####     | Analog GND
 *        X_DIR_PIN           MISO1   TX1       1 |               | 3.3V
 *        Y_STEP_PIN                       PWM  2 | *NC     AREF* | 23  A9 PWM
 *        Y_DIR_PIN           SCL2 CAN0TX  PWM  3 | *A26     A10* | 22  A8 PWM
 *        Z_STEP_PIN          SDA2 CAN0RX  PWM  4 | *A25     A11* | 21  A7 PWM  CS0   MOSI1  RX1
 *        Z_DIR_PIN           MISO1   TX1  PWM  5 | *GND * * 57   | 20  A6 PWM  CS0   SCK1        FILWIDTH_PIN
 *        X_ENABLE_PIN                     PWM  6 | *GND * * 56   | 19  A5            SCL0        E0_STEP_PIN
 *        Y_ENABLE_PIN  SCL0  MOSI0   RX3  PWM  7 |      * * 55   | 18  A4            SDA0        E0_DIR_PIN
 *        Z_ENABLE_PIN  SDA0  MISO0   TX3  PWM  8 |      * * 54   | 17  A3            SDA0        E0_ENABLE_PIN
 *                            CS0     RX2  PWM  9 |               | 16  A2            SCL0        TEMP_0_PIN
 *                            CS0     TX2  PWM 10 |               | 15  A1      CS0               TEMP_BED_PIN
 *        X_STOP_PIN          MOSI0            11 |               | 14  A0 PWM  CS0               TEMP_1_PIN
 *        Y_STOP_PIN          MISO0            12 |               | 13 LED            SCK0        LED_PIN
 *                                           3.3V |               | GND
 *        Z_STOP_PIN                           24 |   40 * * 53   |    A22 DAC1
 *  AUX2                                       25 |   41 * * 52   |    A21 DAC0
 *  AUX2  FAN0_PIN            SCL2    TX1      26 |   42 * * 51   | 39 A20      MISO0             SDSS
 *  AUX2  Z-PROBE PWR         SCK0    RX1      27 | *  *  *  *  * | 38 A19 PWM        SDA1
 *  AUX2  SOL1_PIN            MOSI0            28 |   43 * * 50   | 37 A18 PWM        SCL1
 *  D10   CONTROLLER_FAN_PIN  CAN0TX       PWM 29 |   44 * * 49   | 36 A17 PWM
 *  D9    HEATER_0_PIN        CAN0RX       PWM 30 |   45 * * 48   | 35 A16 PWM                    E1_ENABLE_PIN
 *  D8    HEATER_BED_PIN      CS1     RX4  A12 31 |   46 * * 47   | 34 A15 PWM        SDA0  RX5   E1_DIR_PIN
 *                            SCK1    TX4  A13 32 |__GND_*_*_3.3V_| 33 A14 PWM        SCL0  TX5   E1_STEP_PIN
 *
 *            Interior pins:
 *                            LCD_PINS_RS             40 * * 53   SCK2
 *                            LCD_PINS_EN             41 * * 52   MOSI2
 *                            LCD_PINS_D4             42 * * 51   MISO2
 *                            LCD_PINS_D5     CS2     43 * * 50   A24
 *                            LCD_PINS_D6     MOSI2   44 * * 49   A23
 *                            LCD_PINS_D7     MISO2   45 * * 48   TX6   SDA0  BTN_ENC
 *                            BTN_EN1         SCK2    46 * * 47   RX6   SCL0  BTN_EN2
 *                                                   GND * * 3.3V
 */

//
// Limit Switches
//
#define X_STOP_PIN                            24
#define Y_STOP_PIN                            26
#define Z_STOP_PIN                            28

//
// Steppers
//
#define X_STEP_PIN                            22
#define X_DIR_PIN                             21
#define X_ENABLE_PIN                          39

#define Y_STEP_PIN                            19
#define Y_DIR_PIN                             18
#define Y_ENABLE_PIN                          20

#define Z_STEP_PIN                            38
#define Z_DIR_PIN                             37
#define Z_ENABLE_PIN                          17

#define E0_STEP_PIN                           31
#define E0_DIR_PIN                            30
#define E0_ENABLE_PIN                         32

#define E1_STEP_PIN                           33
#define E1_DIR_PIN                            34
#define E1_ENABLE_PIN                         35

#define HEATER_0_PIN                          30
#define HEATER_1_PIN                          36
#define HEATER_BED_PIN                        31
#ifndef FAN0_PIN
  #define FAN0_PIN                             2
#endif

#define TEMP_0_PIN                             2  // Extruder / Analog pin numbering: 2 => A2
#define TEMP_1_PIN                             0
#define TEMP_BED_PIN                           1  // Bed / Analog pin numbering

//
// Misc. Functions
//
#define LED_PIN                               13
#define PS_ON_PIN                              1
#define FILWIDTH_PIN                           6
#define SOL1_PIN                              28

//
// SD Card
//
#define SD_SS_PIN                             39  // 8

#if HAS_WIRED_LCD
  #define LCD_PINS_RS                         40
  #define LCD_PINS_EN                         41
  #define LCD_PINS_D4                         42
  #define LCD_PINS_D5                         43
  #define LCD_PINS_D6                         44
  #define LCD_PINS_D7                         45
#endif

#if IS_NEWPANEL
  #define BTN_EN1                             46
  #define BTN_EN2                             47
  #define BTN_ENC                             48
#endif

#if IS_RRW_KEYPAD
  #define SHIFT_OUT_PIN                       40
  #define SHIFT_CLK_PIN                       44
  #define SHIFT_LD_PIN                        42
#endif
