/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * ZONESTAR ZRIB V5.3 Based on MKS BASE v1.4 with A4982 stepper drivers and digital micro-stepping
 * ATmega2560, ATmega1280
 */

#include "env_validate.h"

#if HOTENDS > 2
  #error "ZRIB V5.3 supports up to 2 hotends."
#elif E_STEPPERS > 3
  #error "ZRIB V5.3 supports up to 3 E steppers."
#endif

#define BOARD_INFO_NAME "ZRIB V5.3"

//
// PIN 12 Connector
//
#define PIN_12_PIN                            12

//
// XS1 Connector
//
#define XS1_01_PIN                            42
#define XS1_03_PIN                            43
#define XS1_05_PIN                            44
#define XS1_07_PIN                            45
#define XS1_08_PIN                            47

//
// XS6 Connector
//
#define XS6_01_PIN                            20
#define XS6_03_PIN                            52
#define XS6_05_PIN                            51
#define XS6_07_PIN                            50
#define XS6_08_PIN                            21

//
// Servos / XS3 Connector
//
#ifndef SERVO0_PIN
  #define SERVO0_PIN                          65  // PWM
#endif
#ifndef SERVO1_PIN
  #define SERVO1_PIN                          66  // PWM
#endif

//
// Limit Switches
//
#ifndef X_STOP_PIN
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                          3
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                          2
  #endif
#endif
#ifndef Y_STOP_PIN
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                         14
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                         15
  #endif
#endif
#ifndef Z_STOP_PIN
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                         18
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                         19
  #endif
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     19
#endif

//
// Steppers
//
#define X_STEP_PIN                            54
#define X_DIR_PIN                             55
#define X_ENABLE_PIN                          38

#define Y_STEP_PIN                            60
#define Y_DIR_PIN                             61
#define Y_ENABLE_PIN                          56

#define Z_STEP_PIN                            46
#define Z_DIR_PIN                             48
#define Z_ENABLE_PIN                          62

#if NUM_Z_STEPPERS == 2
  #define Z2_STEP_PIN                         26  // E0 connector
  #define Z2_DIR_PIN                          28
  #define Z2_ENABLE_PIN                       24

  #define E0_STEP_PIN                         36  // E1 connector
  #define E0_DIR_PIN                          34
  #define E0_ENABLE_PIN                       30

  #define E1_STEP_PIN                          4  // E2 connector
  #define E1_DIR_PIN                           5
  #define E1_ENABLE_PIN                       22
#else
  #define E0_STEP_PIN                         26
  #define E0_DIR_PIN                          28
  #define E0_ENABLE_PIN                       24

  #define E1_STEP_PIN                         36
  #define E1_DIR_PIN                          34
  #define E1_ENABLE_PIN                       30

  #define E2_STEP_PIN                          4
  #define E2_DIR_PIN                           5
  #define E2_ENABLE_PIN                       22
#endif

//
// Temperature Sensors
//
#ifndef TEMP_0_PIN
  #define TEMP_0_PIN                          13  // Analog Input
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN                          15  // Analog Input
#endif
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                        14  // Analog Input
#endif

//
// Heaters / Fans Connectors
//

#define HEATER_0_PIN                          10
#define HEATER_1_PIN                           7
#define FAN0_PIN                               9
#define HEATER_BED_PIN                         8
#define FAN1_PIN                               6

//
// Misc. Functions
//
#ifndef SD_SS_PIN
  #define SD_SS_PIN                           53
#endif

#define LED_PIN                               13

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN              XS1_01_PIN
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                   XS1_03_PIN
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   *
   * Serial2 -- AUX-4 Pin 18 (D16 TX2) and AUX-4 Pin 17 (D17 RX2)
   * Serial1 -- Pins D18 and D19 are used for Z-MIN and Z-MAX
   */
  //#define X_HARDWARE_SERIAL Serial1
  //#define X2_HARDWARE_SERIAL Serial1
  //#define Y_HARDWARE_SERIAL Serial1
  //#define Y2_HARDWARE_SERIAL Serial1
  //#define Z_HARDWARE_SERIAL Serial1
  //#define Z2_HARDWARE_SERIAL Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1
  //#define E2_HARDWARE_SERIAL Serial1
  //#define E3_HARDWARE_SERIAL Serial1
  //#define E4_HARDWARE_SERIAL Serial1

  #ifndef X_SERIAL_TX_PIN
    #define X_SERIAL_TX_PIN           XS1_08_PIN
  #endif
  #ifndef X_SERIAL_RX_PIN
    #define X_SERIAL_RX_PIN           PIN_12_PIN
  #endif
  #ifndef X2_SERIAL_TX_PIN
    #define X2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef X2_SERIAL_RX_PIN
    #define X2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Y_SERIAL_TX_PIN
    #define Y_SERIAL_TX_PIN           XS1_08_PIN
  #endif
  #ifndef Y_SERIAL_RX_PIN
    #define Y_SERIAL_RX_PIN           PIN_12_PIN
  #endif
  #ifndef Y2_SERIAL_TX_PIN
    #define Y2_SERIAL_TX_PIN                  -1
  #endif
  #ifndef Y2_SERIAL_RX_PIN
    #define Y2_SERIAL_RX_PIN                  -1
  #endif

  #ifndef Z_SERIAL_TX_PIN
    #define Z_SERIAL_TX_PIN           XS1_08_PIN
  #endif
  #ifndef Z_SERIAL_RX_PIN
    #define Z_SERIAL_RX_PIN           PIN_12_PIN
  #endif
  #ifndef Z2_SERIAL_TX_PIN
    #define Z2_SERIAL_TX_PIN          XS1_08_PIN
  #endif
  #ifndef Z2_SERIAL_RX_PIN
    #define Z2_SERIAL_RX_PIN          PIN_12_PIN
  #endif

  #ifndef E0_SERIAL_TX_PIN
    #define E0_SERIAL_TX_PIN                  -1
  #endif
  #ifndef E0_SERIAL_RX_PIN
    #define E0_SERIAL_RX_PIN                  -1
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

/**
 * LCD adapter. NOTE: These come in two variants. The socket keys can be
 * on either side, and may be backwards on some displays.
 *         ------                     ------
 *    D37 | 1  2 | D35    (MISO) D50 | 1  2 | D52 (SCK)
 *    D17 | 3  4 | D16           D31 | 3  4 | D53
 *    D23   6  5   D25           D33   6  5   D51 (MOSI)
 *    D27 | 7  8 | D29           D49 | 7  8 | D41
 *    GND | 9 10 | 5V            GND | 9 10 | --
 *         ------                     ------
 *          EXP1                       EXP2
 */

#ifndef EXP1_08_PIN
  #define EXP1_01_PIN                         37
  #define EXP1_02_PIN                         35
  #define EXP1_03_PIN                         17
  #define EXP1_04_PIN                         16
  #define EXP1_05_PIN                         23
  #define EXP1_06_PIN                         25
  #define EXP1_07_PIN                         27
  #define EXP1_08_PIN                         29

  #define EXP2_01_PIN                 XS6_07_PIN
  #define EXP2_02_PIN                 XS6_03_PIN
  #define EXP2_03_PIN                         31
  #define EXP2_04_PIN                         53
  #define EXP2_05_PIN                         33
  #define EXP2_06_PIN                 XS6_05_PIN
  #define EXP2_07_PIN                         49
  #define EXP2_08_PIN                         41
#endif

//
// LCD / Controller
//

#if ENABLED(ZONESTAR_12864LCD)
  #define LCDSCREEN_NAME "ZONESTAR LCD12864"
  #define LCD_SDSS_PIN                        16
  #define LCD_PINS_RS                         16  // ST7920 CS          (LCD-4)
  #define LCD_PINS_EN                         23  // ST7920 DAT LCD-R/W (LCD-5)
  #define LCD_PINS_D4                         17  // ST7920 CLK LCD-ENA (LCD-6)
  #define BTN_EN2                             25
  #define BTN_EN1                             27
  #define BTN_ENC                             29
  #define BEEPER_PIN                          37
  #define KILL_PIN                            35
#elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define LCDSCREEN_NAME "Reprap LCD12864"
  // Use EXP1 & EXP2 connector
  #define LCD_PINS_RS                         16  // ST7920 CS
  #define LCD_PINS_EN                         17  // ST7920 DAT
  #define LCD_PINS_D4                         23  // ST7920 CLK LCD-R/W
  #define BTN_EN1                             31
  #define BTN_EN2                             33
  #define BTN_ENC                             35
  #define BEEPER_PIN                          37
  #define KILL_PIN                            41
#endif

//================================================================================
// OLED 128x64
//================================================================================

#if ANY(ZONESTAR_12864OLED, ZONESTAR_12864OLED_SSD1306)
  #define LCDSCREEN_NAME "ZONESTAR 12864OLED"
  #define LCD_SDSS_PIN                        16
  #define LCD_PINS_RS                         23  // RESET Pull low for 1s to init
  #define LCD_PINS_DC                         17
  #define DOGLCD_CS                           16  // CS
  #define BTN_EN2                             25
  #define BTN_EN1                             27
  #define BTN_ENC                             29
  #define BEEPER_PIN                          -1
  #define KILL_PIN                            -1
  #if ANY(OLED_HW_IIC, OLED_HW_SPI)
    #error "Oops! You must choose SW SPI for ZRIB V53 board and connect the OLED screen to EXP1 connector."
  #else // SW_SPI
    #define DOGLCD_A0                LCD_PINS_DC
    #define DOGLCD_MOSI                       35  // SDA
    #define DOGLCD_SCK                        37  // SCK
  #endif
#endif // OLED 128x64

//================================================================================
// LCD 2004 KEYPAD
//================================================================================

#if ENABLED(ZONESTAR_LCD)
  #define LCDSCREEN_NAME "LCD2004 ADCKEY"
  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN
  #define LCD_PINS_D6                EXP1_07_PIN
  #define LCD_PINS_D7                EXP1_08_PIN
  #define ADC_KEYPAD_PIN                      10  // A10 for ADCKEY
  #define BEEPER_PIN                 EXP1_01_PIN
#endif

/**
 * ZRIB V5.3 Main Board
 *
 * Available connectors on ZRIB V5.3
 *
 *  =======
 *  | GND |
 *  |-----|   E0
 *  |  10 |                (10)  PB4 ** Pin23 ** PWM10
 *  |-----|
 *  | GND |
 *  |-----|   E1
 *  |  7  |                ( 7)  PH4 ** Pin16 ** PWM7
 *  |-----|
 *  | GND |
 *  |-----|   FAN
 *  |  9  |                ( 9)  PH6 ** Pin18 ** PWM9
 *  =======
 *  =======
 *  | GND |
 *  |-----|   Heated Bed
 *  |  8  |                ( 8)  PH5 ** Pin17 ** PWM8
 *  =======
 *  ==========
 *  | 12-24V |
 *  |--------|   Power
 *  |  GND   |
 *  ==========
 * Servos / XS3 Connector
 *  =================
 *  | 65 | GND | 5V |      (65)  PK3 ** Pin86 ** A11
 *  |----|-----|----|
 *  | 66 | GND | 5V |      (66)  PK4 ** Pin85 ** A12
 *  =================
 * ICSP
 *  =================
 *  | 5V | 51 | GND |      (51)  PB2 ** Pin21 ** SPI_MOSI
 *  |----|----|-----|
 *  | 50 | 52 | RST |      (50)  PB3 ** Pin22 ** SPI_MISO
 *  =================      (52)  PB1 ** Pin20 ** SPI_SCK
 * XS6 Connector
 *  ======================
 *  | 5V | GND | NC | 20 | (20)  PD1 ** Pin44 ** I2C_SDA
 *  |----|-----|----|----|
 *  | 50 |  51 | 52 | 21 | (50)  PB3 ** Pin22 ** SPI_MISO
 *  ====================== (51)  PB2 ** Pin21 ** SPI_MOSI
 *                         (52)  PB1 ** Pin20 ** SPI_SCK
 *                         (21)  PD0 ** Pin43 ** I2C_SCL
 * XS1 Connector
 *  ======================
 *  | 5V | GND | NC | 47 | (47)  PL2 ** Pin37 ** D47
 *  |----|-----|----|----|
 *  | 42 |  43 | 44 | 45 | (45)  PL4 ** Pin39 ** D45
 *  ====================== (44)  PL5 ** Pin40 ** D44
 *                         (43)  PL6 ** Pin41 ** D43
 *                         (42)  PL7 ** Pin42 ** D42
 * Temperature
 *  ==================================
 *  | GND | 69 | GND | 68 | GND | 67 |
 *  ==================================
 *                         (69)  PK7 ** Pin82 ** A15
 *                         (68)  PK6 ** Pin83 ** A14
 *                         (67)  PK5 ** Pin84 ** A13
 * Limit Switches
 *  ============
 *  |  2 | GND | X+        ( 2)  PE4 ** Pin6  ** PWM2
 *  |----|-----|
 *  |  3 | GND | X-        ( 3)  PE5 ** Pin7  ** PWM3
 *  |----|-----|
 *  | 15 | GND | Y+        (15)  PJ0 ** Pin63 ** USART3_RX
 *  |----|-----|
 *  | 14 | GND | Y-        (14)  PJ1 ** Pin64 ** USART3_TX
 *  |----|-----|
 *  | 19 | GND | Z+        (19)  PD2 ** Pin45 ** USART1_RX
 *  |----|-----|
 *  | 18 | GND | Z-        (18)  PD3 ** Pin46 ** USART1_TX
 *  ============
 * EXP1
 *  ============
 *  |  37 | 35 |           (37)  PC0 ** Pin53 ** D37
 *  |-----|----|           (35)  PC2 ** Pin55 ** D35
 *  |  17 | 16 |           (17)  PH0 ** Pin12 ** USART2_RX
 *  |-----|----|           (16)  PH1 ** Pin13 ** USART2_TX
 *  |  23 | 25 |           (23)  PA1 ** Pin77 ** D23
 *  |-----|----|           (25)  PA3 ** Pin75 ** D25
 *  |  27 | 29 |           (27)  PA5 ** Pin73 ** D27
 *  |-----|----|           (29)  PA7 ** Pin71 ** D29
 *  | GND | 5V |
 *  ============
 * EXP2
 *  ============
 *  |  50 | 52 |           (50)  PB3 ** Pin22 ** SPI_MISO
 *  |-----|----|           (52)  PB1 ** Pin20 ** SPI_SCK
 *  |  31 | 53 |           (31)  PC6 ** Pin59 ** D31
 *  |-----|----|           (53)  PB0 ** Pin19 ** SPI_SS
 *  |  33 | 51 |           (33)  PC4 ** Pin57 ** D33
 *  |-----|----|           (51)  PB2 ** Pin21 ** SPI_MOSI
 *  |  49 | 41 |           (49)  PL0 ** Pin35 ** D49
 *  |-----|----|           (41)  PG0 ** Pin51 ** D41
 *  | GND | NC |
 *  ============
 *
 * PIN 12
 *   ======
 *   | 12 |                (12)  PB6 ** Pin25 ** D12
 *   ======
 */
