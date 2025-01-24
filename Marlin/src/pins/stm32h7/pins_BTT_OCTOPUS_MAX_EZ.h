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

#include "env_validate.h"

#define BOARD_INFO_NAME      "OCTOPUS MAX EZ V1.0"

#define USES_DIAG_JUMPERS

// Onboard I2C EEPROM
#if ANY(NO_EEPROM_SELECTED, I2C_EEPROM)
  #undef NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C
  #define I2C_SCL_PIN                       PB10
  #define I2C_SDA_PIN                       PB11
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4KB
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PB14

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PF0   // M1-STOP
#define Y_DIAG_PIN                          PF2   // M2-STOP
#define Z_DIAG_PIN                          PF4   // M3-STOP
#define Z2_DIAG_PIN                         PF3   // M4-DET
#define E0_DIAG_PIN                         PF1   // M5-DET
#define E1_DIAG_PIN                         PC15  // M6-DET
#define E2_DIAG_PIN                         PF12  // PWRDET

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-DET
  #else
    #define X_MIN_PIN                E0_DIAG_PIN  // M4-DET
  #endif
#elif NEEDS_X_MINMAX
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-DET
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-DET
  #else
    #define Y_MIN_PIN                E1_DIAG_PIN  // M5-DET
  #endif
#elif NEEDS_Y_MINMAX
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-DET
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // PWRDET
  #else
    #define Z_MIN_PIN                E2_DIAG_PIN  // PWRDET
  #endif
#elif NEEDS_Z_MINMAX
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // PWRDET
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // Z-STOP
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB15
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PF1   // M5-DET
#define FIL_RUNOUT2_PIN                     PC15  // M6-DET

//
// Steppers
//
#define X_STEP_PIN                          PC13  // MOTOR 1
#define X_DIR_PIN                           PC14
#define X_ENABLE_PIN                        PE6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PG14
#endif

#define Y_STEP_PIN                          PE4   // MOTOR 2
#define Y_DIR_PIN                           PE5
#define Y_ENABLE_PIN                        PE3
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PG13
#endif

#define Z_STEP_PIN                          PE1   // MOTOR 3
#define Z_DIR_PIN                           PE0
#define Z_ENABLE_PIN                        PE2
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PG12
#endif

#define Z2_STEP_PIN                         PB8   // MOTOR 4
#define Z2_DIR_PIN                          PB9
#define Z2_ENABLE_PIN                       PB7
#ifndef Z2_CS_PIN
  #define Z2_CS_PIN                         PG11
#endif

#define E0_STEP_PIN                         PB5   // MOTOR 5
#define E0_DIR_PIN                          PB4
#define E0_ENABLE_PIN                       PB6
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PG10
#endif

#define E1_STEP_PIN                         PG15  // MOTOR 6
#define E1_DIR_PIN                          PB3
#define E1_ENABLE_PIN                       PD5
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PG9
#endif

#define E2_STEP_PIN                         PD3   // MOTOR 7
#define E2_DIR_PIN                          PD2
#define E2_ENABLE_PIN                       PD4
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD7
#endif

#define E3_STEP_PIN                         PA10  // MOTOR 8
#define E3_DIR_PIN                          PA9
#define E3_ENABLE_PIN                       PA15
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD6
#endif

#define E4_STEP_PIN                         PA8   // MOTOR 9
#define E4_DIR_PIN                          PC7
#define E4_ENABLE_PIN                       PC9
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PG8
#endif

#define E5_STEP_PIN                         PG6   // MOTOR 10
#define E5_DIR_PIN                          PC6
#define E5_ENABLE_PIN                       PC8
#ifndef E5_CS_PIN
  #define E5_CS_PIN                         PG7
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PB1   // TB
#define TEMP_0_PIN                          PB0   // TH0
#define TEMP_1_PIN                          PC5   // TH1
#define TEMP_2_PIN                          PC4   // TH2
#define TEMP_3_PIN                          PA7   // TH3

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PF5   // Hotbed
#define HEATER_0_PIN                        PF6   // Heater0
#define HEATER_1_PIN                        PA0   // Heater1
#define HEATER_2_PIN                        PF9   // Heater2
#define HEATER_3_PIN                        PF7   // Heater3

#define FAN0_PIN                            PA6   // Fan0
#define FAN1_PIN                            PA5   // Fan1
#define FAN2_PIN                            PA4   // Fan2
#define FAN3_PIN                            PA3   // Fan3
#define FAN4_PIN                            PA1   // 4 wire Fan4
#define FAN5_PIN                            PF8   // 4 wire Fan5
#define FAN6_PIN                            PA2   // 4 wire Fan6

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PF13  // PS-ON
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PF12  // PWRDET
#endif

//
// Misc. Functions
//
#define LED_PIN                             PA14
#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                      PC0
#endif
#ifndef FILWIDTH2_PIN
  #define FILWIDTH2_PIN                     PF10
#endif

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #if HAS_WIRED_LCD && DISABLED(NO_LCD_SDCARD)
    #define SDCARD_CONNECTION                LCD
  #else
    #define SDCARD_CONNECTION            ONBOARD
  #endif
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PE14
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PE13
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PE12
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

  #define X_SERIAL_TX_PIN                   PG14
  #define Y_SERIAL_TX_PIN                   PG13
  #define Z_SERIAL_TX_PIN                   PG12
  #define Z2_SERIAL_TX_PIN                  PG11
  #define E0_SERIAL_TX_PIN                  PG10
  #define E1_SERIAL_TX_PIN                  PG9
  #define E2_SERIAL_TX_PIN                  PD7
  #define E3_SERIAL_TX_PIN                  PD6
  #define E4_SERIAL_TX_PIN                  PG8
  #define E4_SERIAL_RX_PIN      E3_SERIAL_RX_PIN
  #define E5_SERIAL_TX_PIN                  PG7
  #define E5_SERIAL_RX_PIN      E3_SERIAL_RX_PIN

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

/**
 *            18-pin FPC Connector
 *                    ----
 *       (MISO) PE13 | 1  |
 *        (SCK) PE12 | 2  |
 *    (BTN_EN1)  PG5 | 3  |
 *      (SD_SS) PE11 | 4  |
 *    (BTN_EN2)  PG4 | 5  |
 *       (MOSI) PE14 | 6  |
 *  (SD_DETECT)  PG3 | 7  |
 *             RESET | 8  |
 *     (BEEPER)  PG2 | 9  |
 *    (BTN_ENC) PD15 | 10 |
 *     (LCD_EN) PD14 | 11 |
 *     (LCD_RS) PD13 | 12 |
 *     (LCD_D4) PD12 | 13 |
 *     (LCD_D5) PD11 | 14 |
 *     (LCD_D6) PD10 | 15 |
 *     (LCD_D7) PE15 | 16 |
 *               GND | 17 |
 *                5V | 18 |
 *                    ----
 *                    LCD
 */

#define EXP1_01_PIN                         PG2
#define EXP1_02_PIN                         PD15
#define EXP1_03_PIN                         PD14
#define EXP1_04_PIN                         PD13
#define EXP1_05_PIN                         PD12
#define EXP1_06_PIN                         PD11
#define EXP1_07_PIN                         PD10
#define EXP1_08_PIN                         PE15

#define EXP2_01_PIN                         PE13
#define EXP2_02_PIN                         PE12
#define EXP2_03_PIN                         PG5
#define EXP2_04_PIN                         PE11
#define EXP2_05_PIN                         PG4
#define EXP2_06_PIN                         PE14
#define EXP2_07_PIN                         PG3
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #ifndef SD_DETECT_STATE
    #define SD_DETECT_STATE HIGH
  #elif SD_DETECT_STATE == LOW
    #error "BOARD_BTT_OCTOPUS_MAX_EZ onboard SD requires SD_DETECT_STATE set to HIGH."
  #endif
  #define SD_SS_PIN                         PB12
  #define SD_SCK_PIN                        PE12
  #define SD_MISO_PIN                       PE13
  #define SD_MOSI_PIN                       PE14
  #define SD_DETECT_PIN                     PB13
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(LCD)
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
#endif

//
// LCD / Controller
//

#if ENABLED(BTT_MINI_12864)                       // BTT Mini 12864 V2.0 connected via 18-pin FPC cable

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define LCD_PINS_RS                EXP1_04_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN
  #define LCD_PINS_D6                EXP1_07_PIN
  #define LCD_PINS_D7                EXP1_08_PIN

  #define DOGLCD_CS                  EXP1_03_PIN
  #define DOGLCD_A0                  EXP1_04_PIN
  #define DOGLCD_SCK                 EXP2_02_PIN
  #define DOGLCD_MOSI                EXP2_06_PIN

  #define SOFTWARE_SPI
  #define FORCE_SOFT_SPI                          // Use this if Hardware SPI causes display problems.
                                                  // Results in LCD Software SPI mode 3, SD Software SPI mode 0.

  //#define LCD_BACKLIGHT_PIN               -1
  #define LCD_RESET_PIN              EXP1_05_PIN  // Must be high or open for LCD to operate normally.
  #define NEOPIXEL_PIN               EXP1_06_PIN

#elif HAS_WIRED_LCD
  #error "Only BTT_MINI_12864 (BTT Mini 12864 V2.0 with FPC cable) is currently supported on the Octopus Max EZ."
#endif

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PE10
#endif
#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PE9
#endif

//
// WiFi
//
#if ENABLED(WIFISUPPORT)
  /**
   *                 --------
   *           GND  |  9   8 | 3.3V
   *  (ESP-CS) PG1  | 10   7 | PB15 (ESP-MOSI)
   *           3.3V | 11   6 | PB14 (ESP-MISO)
   * (ESP-IO0) PG0  | 12   5 | PB13 (ESP-CLK)
   * (ESP-IO4) PF15 | 13   4 | --
   *             -- | 14   3 | 3.3V (ESP-EN)
   *  (ESP-RX) PE7  | 15   2 | --
   *  (ESP-TX) PE8  | 16   1 | PB2 (ESP-RST)
   *                 --------
   *                   WIFI
   */
  #define ESP_WIFI_MODULE_COM                  7  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PB2
  #define ESP_WIFI_MODULE_GPIO0_PIN         PG0
  #define ESP_WIFI_MODULE_GPIO4_PIN         PF15
#endif
