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

#define BOARD_INFO_NAME   "BTT Manta M8P V2.0"
#define BOARD_WEBSITE_URL "github.com/bigtreetech/Manta-M8P/tree/master/V2.0"

#define USES_DIAG_JUMPERS

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #undef NO_EEPROM_SELECTED
  #ifndef FLASH_EEPROM_EMULATION
    #define FLASH_EEPROM_EMULATION                // Use Flash-based EEPROM emulation
  #endif
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PD12  // PROBE

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PF4   // M1-STOP
#define Y_DIAG_PIN                          PF3   // M2-STOP
#define Z_DIAG_PIN                          PF2   // M3-STOP
#define E0_DIAG_PIN                         PF1   // M4-STOP
#define E1_DIAG_PIN                         PF0   // M5-STOP
#define E2_DIAG_PIN                         PC15  // M6-STOP
#define E3_DIAG_PIN                         -1    // No DIAG or STOP pin for M7
#define E4_DIAG_PIN                         -1    // No DIAG or STOP pin for M8

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN  // M1-STOP
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-STOP
  #else
    #define X_MIN_PIN                E0_DIAG_PIN  // M4-STOP
  #endif
#elif NEEDS_X_MINMAX
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // M1-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // M4-STOP
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // M1-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN  // M2-STOP
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-STOP
  #else
    #define Y_MIN_PIN                E1_DIAG_PIN  // M5-STOP
  #endif
#elif NEEDS_Y_MINMAX
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // M2-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // M5-STOP
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // M2-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN  // M3-STOP
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // M6-STOP
  #else
    #define Z_MIN_PIN                E2_DIAG_PIN  // M6-STOP
  #endif
#elif NEEDS_Z_MINMAX
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // M3-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // M6-STOP
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // M3-STOP
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PD13  // PROBE
  //#define Z_MIN_PROBE_PIN                 PD8   // "FAN IND" - Shared inductive probe / fan connector
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
#define FIL_RUNOUT_PIN               E0_DIAG_PIN  // M4-STOP
#define FIL_RUNOUT2_PIN              E1_DIAG_PIN  // M5-STOP
#define FIL_RUNOUT3_PIN              E2_DIAG_PIN  // M6-STOP

//
// Steppers
//
#define X_STEP_PIN                          PE6   // MOTOR 1
#define X_DIR_PIN                           PE5
#define X_ENABLE_PIN                        PC14
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC13
#endif

#define Y_STEP_PIN                          PE2   // MOTOR 2
#define Y_DIR_PIN                           PE1
#define Y_ENABLE_PIN                        PE4
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE3
#endif

#define Z_STEP_PIN                          PB8   // MOTOR 3
#define Z_DIR_PIN                           PB7
#define Z_ENABLE_PIN                        PE0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB9
#endif

#define E0_STEP_PIN                         PB4   // MOTOR 4
#define E0_DIR_PIN                          PB3
#define E0_ENABLE_PIN                       PB6
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB5
#endif

#define E1_STEP_PIN                         PG13  // MOTOR 5
#define E1_DIR_PIN                          PG12
#define E1_ENABLE_PIN                       PG15
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PG14
#endif

#define E2_STEP_PIN                         PG9   // MOTOR 6
#define E2_DIR_PIN                          PD7
#define E2_ENABLE_PIN                       PG11
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PG10
#endif

#define E3_STEP_PIN                         PD4   // MOTOR 7
#define E3_DIR_PIN                          PD3
#define E3_ENABLE_PIN                       PD6
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD5
#endif

#define E4_STEP_PIN                         PC7   // MOTOR 8
#define E4_DIR_PIN                          PC8
#define E4_ENABLE_PIN                       PD2
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PC6
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB0   // TH0
#define TEMP_1_PIN                          PC5   // TH1
#define TEMP_2_PIN                          PC4   // TH2
#define TEMP_3_PIN                          PA7   // TH3
#define TEMP_BED_PIN                        PB1   // TB

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PF5   // BED-OUT
#define HEATER_0_PIN                        PA0   // HE0
#define HEATER_1_PIN                        PA1   // HE1
#define HEATER_2_PIN                        PA3   // HE2
#define HEATER_3_PIN                        PA5   // HE3

#define FAN0_PIN                            PF7   // FAN0 (2 wire)
#if ENABLED(FOURWIRES_FANS)
  // Only FAN5 and FAN6 have tachometer pins, so swap them with FAN1 and FAN2
  #define FAN1_PIN                          PA6   // FAN5 (4 wire)
  #define FAN2_PIN                          PA2   // FAN6 (4 wire)
  #define FAN5_PIN                          PF9   // FAN1 (2 wire)
  #define FAN6_PIN                          PF6   // FAN2 (2 wire)
#else
  #define FAN1_PIN                          PF9   // FAN1 (2 wire)
  #define FAN2_PIN                          PF6   // FAN2 (2 wire)
  #define FAN5_PIN                          PA6   // FAN5 (4 wire)
  #define FAN6_PIN                          PA2   // FAN6 (4 wire)
#endif
#define FAN3_PIN                            PF8   // FAN3 (2 wire)
#define FAN4_PIN                            PA4   // FAN4 (2 wire)
#define FAN7_PIN                            PE9   // MOTOR (3 wire)
//#define FAN8_PIN                          PB11  // "FAN IND" (3 wire) - Shared inductive probe / fan connector

#if ENABLED(FOURWIRES_FANS)
  #ifndef E0_FAN_TACHO_PIN
    #define E0_FAN_TACHO_PIN                PC2   // FAN5 (4 wire)
  #endif
  #ifndef E1_FAN_TACHO_PIN
    #define E1_FAN_TACHO_PIN                PC1   // FAN6 (4 wire)
  #endif
#endif

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN            FAN7_PIN  // MOTOR (3 wire)
#endif

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PD14  // PS-ON
#endif

//
// Misc. Functions
//
#define I2C_SDA_PIN                         PC9   // I2C
#define I2C_SCL_PIN                         PA8   // I2C

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                      PC0   // FWS
#endif
#ifndef FILWIDTH2_PIN
  #define FILWIDTH2_PIN                     PF10  // FWS
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
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PG6
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PG7
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PG8
  #endif
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

  #define X_SERIAL_TX_PIN                   PC13
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PB9
  #define Z2_SERIAL_TX_PIN                  PB5
  #define E0_SERIAL_TX_PIN                  PG14
  #define E1_SERIAL_TX_PIN                  PG10
  #define E2_SERIAL_TX_PIN                  PD5
  #define E3_SERIAL_TX_PIN                  PC6

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
 *    (BTN_EN1) PE15 | 3  |
 *      (SD_SS) PE11 | 4  |
 *    (BTN_EN2) PE10 | 5  |
 *       (MOSI) PE14 | 6  |
 *  (SD_DETECT)  PE8 | 7  |
 *             RESET | 8  |
 *     (BEEPER)  PE7 | 9  |
 *    (BTN_ENC)  PG1 | 10 |
 *     (LCD_EN)  PG0 | 11 |
 *     (LCD_RS) PF15 | 12 |
 *     (LCD_D4) PF14 | 13 |
 *     (LCD_D5) PF13 | 14 |
 *     (LCD_D6) PF12 | 15 |
 *     (LCD_D7) PF11 | 16 |
 *               GND | 17 |
 *                5V | 18 |
 *                    ----
 *                    LCD
 */

#define EXP1_01_PIN                         PE7
#define EXP1_02_PIN                         PG1
#define EXP1_03_PIN                         PG0
#define EXP1_04_PIN                         PF15
#define EXP1_05_PIN                         PF14
#define EXP1_06_PIN                         PF13
#define EXP1_07_PIN                         PF12
#define EXP1_08_PIN                         PF11

#define EXP2_01_PIN                         PE13
#define EXP2_02_PIN                         PE12
#define EXP2_03_PIN                         PE15
#define EXP2_04_PIN                         PE11
#define EXP2_05_PIN                         PE10
#define EXP2_06_PIN                         PE14
#define EXP2_07_PIN                         PE8
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #ifndef SD_DETECT_STATE
    #define SD_DETECT_STATE HIGH
  #elif SD_DETECT_STATE == LOW
    #error "BOARD_BTT_MANTA_M8P_V2_0 onboard SD requires SD_DETECT_STATE set to HIGH."
  #endif
  #define SD_SS_PIN                         PB12
  #define SD_SCK_PIN                        PB13
  #define SD_MISO_PIN                       PB14
  #define SD_MOSI_PIN                       PB15
  #define SD_DETECT_PIN                     -1
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(LCD)
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for BOARD_BTT_MANTA_M8P_V2_0."
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
  #error "Only BTT_MINI_12864 (BTT Mini 12864 V2.0 with FPC cable) is currently supported on BOARD_BTT_MANTA_M8P_V2_0."
#endif

//
// NeoPixel LED
// The BTT_MINI_12864 uses one of the EXP pins for NeoPixels
//
#if DISABLED(BTT_MINI_12864) && !defined(NEOPIXEL_PIN)
  #define NEOPIXEL_PIN                      PD15  // RGB
#elif ENABLED(BTT_MINI_12864) && !defined(NEOPIXEL2_PIN)
  // Allow dedicated RGB (NeoPixel) pin to be used for a NeoPixel strip
  #define NEOPIXEL2_PIN                     PD15  // RGB
#endif
