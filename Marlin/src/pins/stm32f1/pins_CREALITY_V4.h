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
 * Creality 4.2.x (STM32F103RE / STM32F103RC) board pin assignments
 */

#include "env_validate.h"

#if !E_ERROR && (HAS_MULTI_HOTEND || E_STEPPERS > 1)
  #error "Creality v4 only supports 1 hotend / E stepper."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME      "Creality V4"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME "Ender-3 V2"
#endif

#define BOARD_NO_NATIVE_USB

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#ifndef DISABLE_DEBUG
  #define DISABLE_DEBUG
#endif

//
// EEPROM
//
#if NO_EEPROM_SELECTED
  #define IIC_BL24CXX_EEPROM                      // EEPROM on I2C-0
  //#define SDCARD_EEPROM_EMULATION
  #undef NO_EEPROM_SELECTED
#endif

#if ENABLED(IIC_BL24CXX_EEPROM)
  #ifndef IIC_EEPROM_SDA
    #define IIC_EEPROM_SDA                  PA11
  #endif
  #ifndef IIC_EEPROM_SCL
    #define IIC_EEPROM_SCL                  PA12
  #endif
  #define MARLIN_EEPROM_SIZE               0x800  // 2K (24C16)
#elif ENABLED(SDCARD_EEPROM_EMULATION)
  #define MARLIN_EEPROM_SIZE               0x800  // 2K
#endif

//
// Servos
//
#ifndef SERVO0_PIN
  #ifndef HAS_PIN_27_BOARD
    #define SERVO0_PIN                      PB0   // BLTouch OUT
  #else
    #define SERVO0_PIN                      PC6
  #endif
#endif

//
// Limit Switches
//
#ifndef X_STOP_PIN
  #define X_STOP_PIN                        PA5
#endif
#ifndef Y_STOP_PIN
  #define Y_STOP_PIN                        PA6
#endif
#ifndef Z_STOP_PIN
  #define Z_STOP_PIN                        PA7
#endif

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB1   // BLTouch IN
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
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // "Pulled-high"
#endif

//
// Steppers
//
#ifndef X_STEP_PIN
  #define X_STEP_PIN                        PC2
#endif
#ifndef X_DIR_PIN
  #define X_DIR_PIN                         PB9
#endif
#define X_ENABLE_PIN                        PC3   // Shared

#ifndef Y_STEP_PIN
  #define Y_STEP_PIN                        PB8
#endif
#ifndef Y_DIR_PIN
  #define Y_DIR_PIN                         PB7
#endif
#define Y_ENABLE_PIN                X_ENABLE_PIN

#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                        PB6
#endif
#ifndef Z_DIR_PIN
  #define Z_DIR_PIN                         PB5
#endif
#define Z_ENABLE_PIN                X_ENABLE_PIN

#ifndef E0_STEP_PIN
  #define E0_STEP_PIN                       PB4
#endif
#ifndef E0_DIR_PIN
  #define E0_DIR_PIN                        PB3
#endif
#define E0_ENABLE_PIN               X_ENABLE_PIN

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // TH1
#define TEMP_BED_PIN                        PC4   // TB1

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PA1   // HEATER1
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PA2   // HOT BED
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                          PA0   // FAN
#endif
#define FAN_SOFT_PWM_REQUIRED

//
// SD Card
//
#define SD_DETECT_PIN                       PC7
#define SDCARD_CONNECTION ONBOARD
#define ONBOARD_SDIO
#define NO_SD_HOST_DRIVE                          // This board's SD is only seen by the printer

/**    Debug port
 *       -----
 *       | 1 | VCC
 *       | 2 | PA13
 *       | 3 | PA14
 *       | 4 | GND
 *       -----
 */

#define DEBUG_02_PIN                        PA13
#define DEBUG_03_PIN                        PA14

#if ANY(RET6_12864_LCD, HAS_DWIN_E3V2, IS_DWIN_MARLINUI)

  /**
   *    RET6 12864 LCD
   *        ------
   *  PC6  | 1  2 | PB2
   *  PB10 | 3  4 | PB11
   *  PB14   5  6 | PB13
   *  PB12 | 7  8 | PB15
   *   GND | 9 10 | 5V
   *        ------
   */
  #ifndef EXP3_01_PIN
    #define EXP3_01_PIN                     PC6
  #endif
  #ifndef EXP3_02_PIN
    #define EXP3_02_PIN                     PB2
  #endif
  #ifndef EXP3_03_PIN
    #define EXP3_03_PIN                     PB10
  #endif
  #ifndef EXP3_04_PIN
    #define EXP3_04_PIN                     PB11
  #endif
  #ifndef EXP3_05_PIN
    #define EXP3_05_PIN                     PB14
  #endif
  #ifndef EXP3_06_PIN
    #define EXP3_06_PIN                     PB13
  #endif
  #ifndef EXP3_07_PIN
    #define EXP3_07_PIN                     PB12
  #endif
  #ifndef EXP3_08_PIN
    #define EXP3_08_PIN                     PB15
  #endif

#elif ANY(VET6_12864_LCD, DWIN_VET6_CREALITY_LCD)

  /**
   *    VET6 12864 LCD
   *        ------
   *  ?    | 1  2 | PC5
   *  PB10 | 3  4 | ?
   *  PA6    5  6 | PA5
   *  PA4  | 7  8 | PA7
   *   GND | 9 10 | 5V
   *        ------
   */
  #define EXP3_01_PIN                       -1
  #define EXP3_02_PIN                       PC5
  #define EXP3_03_PIN                       PB10
  #define EXP3_04_PIN                       -1
  #define EXP3_05_PIN                       PA6
  #define EXP3_06_PIN                       PA5
  #define EXP3_07_PIN                       PA4
  #define EXP3_08_PIN                       PA7

#elif ANY(CR10_STOCKDISPLAY, FYSETC_MINI_12864_2_1)
  #error "Define RET6_12864_LCD or VET6_12864_LCD to select pins for the LCD with the Creality V4 controller."
#endif

#if ENABLED(CR10_STOCKDISPLAY)

  #define LCD_PINS_RS                EXP3_07_PIN
  #define LCD_PINS_EN                EXP3_08_PIN
  #define LCD_PINS_D4                EXP3_06_PIN

  #define BTN_ENC                    EXP3_02_PIN
  #define BTN_EN1                    EXP3_03_PIN
  #define BTN_EN2                    EXP3_05_PIN

  #ifndef HAS_PIN_27_BOARD
    #define BEEPER_PIN               EXP3_01_PIN
  #endif

#elif ANY(HAS_DWIN_E3V2, IS_DWIN_MARLINUI, DWIN_VET6_CREALITY_LCD)

  #define BTN_ENC                    EXP3_05_PIN
  #define BTN_EN1                    EXP3_08_PIN
  #define BTN_EN2                    EXP3_07_PIN

  #ifndef BEEPER_PIN
    #define BEEPER_PIN               EXP3_06_PIN
  #endif

#elif ENABLED(FYSETC_MINI_12864_2_1)

  CONTROLLER_WARNING("CREALITY_V4", "FYSETC_MINI_12864_2_1 and clones")

  #if SD_CONNECTION_IS(LCD)
    #error "The LCD SD Card is not connected with this configuration."
  #endif

  /**
   *
   *                 Board (RET6 12864 LCD)              Display
   *                 ------                               ------
   *  (EN1)    PC6  | 1  2 | PB2  (BTN_ENC)           5V |10  9 | GND
   *  (LCD_CS) PB10 | 3  4 | PB11 (LCD RESET)         -- | 8  7 | --
   *  (LCD_A0) PB14   5  6 | PB13 (EN2)           (DIN)  | 6  5   (LCD RESET)
   *  (LCD_SCK)PB12 | 7  8 | PB15 (MOSI)        (LCD_A0) | 4  3 | (LCD_CS)
   *            GND | 9 10 | 5V                (BTN_ENC) | 2  1 | --
   *                 ------                               ------
   *                  EXP1                                 EXP1
   *
   *                                                      ------
   *                -----                             -- |10  9 | --
   *                | 1 | VCC                    (RESET) | 8  7 | --
   *                | 2 | PA13 (DIN)             (MOSI)  | 6  5   (EN2)
   *                | 3 | PA14                        -- | 4  3 | (EN1)
   *                | 4 | GND                   (LCD_SCK)| 2  1 | --
   *                -----                                 ------
   *              Debug port                               EXP2
   *
   * Needs custom cable. Connect EN2-EN2, LCD_CS-LCD_CS and so on.
   * Debug port is just above EXP1. You need to add pins.
   *
   */

  #define BTN_ENC                    EXP3_02_PIN
  #define BTN_EN1                    EXP3_01_PIN
  #define BTN_EN2                    EXP3_06_PIN
  #define BEEPER_PIN                        -1

  #define DOGLCD_CS                  EXP3_03_PIN
  #define DOGLCD_A0                  EXP3_05_PIN
  #define DOGLCD_SCK                 EXP3_07_PIN
  #define DOGLCD_MOSI                EXP3_08_PIN
  #define LCD_RESET_PIN              EXP3_04_PIN

  #define FORCE_SOFT_SPI
  #define LCD_BACKLIGHT_PIN                 -1
  #define NEOPIXEL_PIN                      PA13

#endif

// Pins for documentation and sanity checks only.
// Changing these will not change the pin they are on.

// Hardware UART pins
#define UART1_TX_PIN                        PA9   // default uses CH340 RX
#define UART1_RX_PIN                        PA10  // default uses CH340 TX
#define UART2_TX_PIN                        PA2   // default uses HEATER_BED_PIN
#define UART2_RX_PIN                        PA3   // not connected
#define UART3_TX_PIN                        PB10  // default uses LCD connector
#define UART3_RX_PIN                        PB11  // default uses LCD connector
#define UART4_TX_PIN                        PC10  // default uses sdcard SDIO_D2
#define UART4_RX_PIN                        PC11  // default uses sdcard SDIO_D3
#define UART5_TX_PIN                        PC12  // default uses sdcard SDIO_CK
#define UART5_RX_PIN                        PD2   // default uses sdcard SDIO_CMD
