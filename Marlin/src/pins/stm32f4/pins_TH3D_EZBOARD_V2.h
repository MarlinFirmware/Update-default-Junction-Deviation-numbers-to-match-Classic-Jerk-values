/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "TH3D EZBoard only supports 1 hotend / E stepper."
#endif

#define BOARD_INFO_NAME   "TH3D EZBoard V2"
#define BOARD_WEBSITE_URL "th3dstudio.com"

//#define V3_EZABL_ON_SERVO                       // As in TH3D Firmware Config

#define DISABLE_JTAGSWD                           // Disabling J-tag and Debug via SWD

// Onboard I2C EEPROM
#if NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4K
  #define I2C_SCL_PIN                       PB6
  #define I2C_SDA_PIN                       PB7
  #undef NO_EEPROM_SELECTED
#endif

//
// NeoPixel
//
#define BOARD_NEOPIXEL_PIN                  PA8

//
// Servos
//
#if ENABLED(V3_EZABL_ON_SERVO)
  #define SERVO0_PIN                        -1
#else
  #define SERVO0_PIN                        PA2
#endif

//
// Limit Switches
//
#if ANY(SENSORLESS_HOMING, SENSORLESS_PROBING)
  // Sensorless homing pins
  #if ENABLED(X_AXIS_SENSORLESS_HOMING)
    #define X_STOP_PIN                      PB4
  #else
    #define X_STOP_PIN                      PC1
  #endif

  #if ENABLED(Y_AXIS_SENSORLESS_HOMING)
    #define Y_STOP_PIN                      PB9
  #else
    #define Y_STOP_PIN                      PC2
  #endif

  //#define Z_STOP_PIN                      PC15  // Don't use sensorless homing on Z!

  #define E_STOP_PIN                        PB10
#else
  // Standard Endstop Pins
  #define X_STOP_PIN                        PC1
  #define Y_STOP_PIN                        PC2
#endif

#if ENABLED(V3_EZABL_ON_SERVO)
  #define Z_STOP_PIN                        PA2
#else
  #define Z_STOP_PIN                        PC3
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PC0
#endif

//
// Steppers
//
#define X_STEP_PIN                          PB3
#define X_DIR_PIN                           PD2
#define X_ENABLE_PIN                        PB5

#define Y_STEP_PIN                          PB8
#define Y_DIR_PIN                           PC13
#define Y_ENABLE_PIN                        PC12

#define Z_STEP_PIN                          PA3
#define Z_DIR_PIN                           PB1
#define Z_ENABLE_PIN                        PC14

#define E0_STEP_PIN                         PA15
#define E0_DIR_PIN                          PB11
#define E0_ENABLE_PIN                       PB2

#if HAS_TMC_UART
  //
  // Hardware Serial on UART4, Single Wire, 0-3 addresses
  //
  #define X_SERIAL_TX_PIN                   PC10
  #define X_SERIAL_RX_PIN                   PC11

  #define Y_SERIAL_TX_PIN                   PC10
  #define Y_SERIAL_RX_PIN                   PC11

  #define Z_SERIAL_TX_PIN                   PC10
  #define Z_SERIAL_RX_PIN                   PC11

  #define E0_SERIAL_TX_PIN                  PC10
  #define E0_SERIAL_RX_PIN                  PC11

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_TH3D_EZBOARD_V2.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_TH3D_EZBOARD_V2.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_TH3D_EZBOARD_V2.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_TH3D_EZBOARD_V2.");

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temp Sensors
//  3.3V max when defined as an Analog Input!
//
#define TEMP_0_PIN                          PA1   // Analog Input PA1
#define TEMP_BED_PIN                        PA0   // Analog Input PA0

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PC9
#define HEATER_0_PIN                        PC8
#ifndef FAN0_PIN
  #define FAN0_PIN                          PC6
#endif
#define FAN1_PIN                            PC7

//
// Auto fans
//
#define AUTO_FAN_PIN                        PC7
#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN           AUTO_FAN_PIN
#endif

//
// SD Card
//

#define SDCARD_CONNECTION                ONBOARD

//#define SOFTWARE_SPI
#define CUSTOM_SPI_PINS
#define ONBOARD_SD_CS_PIN                   PA4
//#define SD_DETECT_PIN                     -1

#define SD_SS_PIN              ONBOARD_SD_CS_PIN
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7

//
// LCD / Controller
//

/**
 *        ------
 *  PA14 | 1  2 | PB0
 *  PC4  | 3  4 | --
 *  PC5  | 5  6   PB13
 *  PB12 | 7  8 | PB15
 *   GND | 9 10 | 5V
 *        ------
 *         EXP1
 *
 * LCD_PINS_D5, D6, and D7 are not present in the EXP1 connector, and will need to be
 * defined to use the REPRAP_DISCOUNT_SMART_CONTROLLER.
 *
 * A remote SD card is currently not supported because the pins routed to the EXP2
 * connector are shared with the onboard SD card.
 */
#define EXP1_01_PIN                         PA14
#define EXP1_02_PIN                         PB0
#define EXP1_03_PIN                         PC4
//#define EXP1_04_PIN                       -1
#define EXP1_05_PIN                         PC5
#define EXP1_06_PIN                         PB13
#define EXP1_07_PIN                         PB12
#define EXP1_08_PIN                         PB15

#if ENABLED(CR10_STOCKDISPLAY)
  /**          ------
   *   BEEPER | 1  2 | ENC
   *   EN1    | 3  4 | RESET
   *   EN2    | 5  6   LCD_D4
   *   LCD_RS | 7  8 | LCD_EN
   *      GND | 9 10 | 5V
   *           ------
   */
  #ifdef DISABLE_JTAGSWD
    #define BEEPER_PIN               EXP1_01_PIN  // Not connected in dev board
  #endif
  #define LCD_PINS_RS                EXP1_07_PIN
  #define LCD_PINS_EN                EXP1_08_PIN
  #define LCD_PINS_D4                EXP1_06_PIN
  //#define KILL_PIN                        -1

  #define BOARD_ST7920_DELAY_1           600
  #define BOARD_ST7920_DELAY_2           750
  #define BOARD_ST7920_DELAY_3           750

#elif ENABLED(MKS_MINI_12864)
  /**          ------
   *      SCK | 1  2 | ENC
   *      EN1 | 3  4 | --
   *      EN2 | 5  6   A0
   *      CS  | 7  8 | MOSI
   *      GND | 9 10 | 5V
   *           ------
   */
  #define DOGLCD_CS                  EXP1_07_PIN
  #define DOGLCD_A0                  EXP1_06_PIN
  #define DOGLCD_SCK                 EXP1_01_PIN
  #define DOGLCD_MOSI                EXP1_08_PIN
  #define LCD_CONTRAST_INIT                  160
  #define LCD_CONTRAST_MIN                   120
  #define LCD_CONTRAST_MAX                   180
  #define FORCE_SOFT_SPI
  #define LCD_BACKLIGHT_PIN                 -1

#elif HAS_WIRED_LCD

  #error "Only CR10_STOCKDISPLAY or MKS_MINI_12864 are supported with TH3D EZBoard V2."

#endif

#if ANY(CR10_STOCKDISPLAY, MKS_MINI_12864)
  #define BTN_EN1                    EXP1_03_PIN
  #define BTN_EN2                    EXP1_05_PIN
  #define BTN_ENC                    EXP1_02_PIN
#endif
