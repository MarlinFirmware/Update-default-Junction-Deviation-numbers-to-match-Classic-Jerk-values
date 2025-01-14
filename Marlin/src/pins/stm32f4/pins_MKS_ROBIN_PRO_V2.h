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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Robin Nano V3 supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS Robin PRO V2"

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 10

// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION                   // Use BackSRAM-based EEPROM emulation
//#define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#define I2C_EEPROM
#define MARLIN_EEPROM_SIZE                0x1000  // 4K

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
//#define DISABLE_DEBUG

//
// Note: MKS Robin board is using SPI2 interface.
//
//#define SPI_MODULE 2

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches
//
#define X_DIAG_PIN                          PA15
#define Y_DIAG_PIN                          PA12
#define Z_DIAG_PIN                          PA11
#define E0_DIAG_PIN                         PC4
#define E1_DIAG_PIN                         PE7

#define X_STOP_PIN                          PA15
#define Y_STOP_PIN                          PA12
#define Z_MIN_PIN                           PA11
#define Z_MAX_PIN                           PC4

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4   // MT_DET
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PE4
#define X_STEP_PIN                          PE3
#define X_DIR_PIN                           PE2
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD5
#endif

#define Y_ENABLE_PIN                        PE1
#define Y_STEP_PIN                          PE0
#define Y_DIR_PIN                           PB9
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD7
#endif

#define Z_ENABLE_PIN                        PB8
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB4
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif

#define E0_ENABLE_PIN                       PB3
#define E0_STEP_PIN                         PD6
#define E0_DIR_PIN                          PD3
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD9
#endif

#define E1_ENABLE_PIN                       PA3
#define E1_STEP_PIN                         PD15
#define E1_DIR_PIN                          PA1
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD8
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PD14
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PD1
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PD0
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

  #define X_SERIAL_TX_PIN                   PD5
  #define Y_SERIAL_TX_PIN                   PD7
  #define Z_SERIAL_TX_PIN                   PD4
  #define E0_SERIAL_TX_PIN                  PD9
  #define E1_SERIAL_TX_PIN                  PD8

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC3   // HEATER1
#define HEATER_1_PIN                        PB0   // HEATER2
#define HEATER_BED_PIN                      PA0   // HOT BED

#define FAN0_PIN                            PB1   // FAN

//
// Thermocouples
//
//#define TEMP_0_CS_PIN                     PE5   // TC1 - CS1
//#define TEMP_0_CS_PIN                     PE6   // TC2 - CS2

//
// Misc. Functions
//
//#define POWER_LOSS_PIN                    PA2   // PW_DET
//#define PS_ON_PIN                         PA3   // PW_OFF

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #define SUICIDE_PIN                       PB2
  #define KILL_PIN                          PA2
  #define KILL_PIN_STATE                    LOW
#else
  #define LED_PIN                           PB2
#endif

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION ONBOARD
#endif

//#define USE_NEW_SPI_API 1

//
// Onboard SD card
// NOT compatible with LCD
//
// Detect pin doesn't work when ONBOARD and NO_SD_HOST_DRIVE disabled
#if SD_CONNECTION_IS(ONBOARD)
  #if USE_NEW_SPI_API
    #define SD_SPI MARLIN_SPI(HardwareSPI3, PC9)
  #else
    #define ENABLE_SPI3
    #define SD_SS_PIN                       -1
    #define SDSS                            PC9
    #define SD_SCK_PIN                      PC10
    #define SD_MISO_PIN                     PC11
    #define SD_MOSI_PIN                     PC12
  #endif
  #define SD_DETECT_PIN                     PD12
#endif

/**               ------                                      ------
 * (BEEPER) PC5  | 1  2 | PE13 (BTN_ENC)    (SPI1 MISO) PA6  | 1  2 | PA5  (SPI1 SCK)
 * (LCD_EN) PD13 | 3  4 | PC6  (LCD_RS)       (BTN_EN1) PE8  | 3  4 | PE10 (SPI1 CS)
 * (LCD_D4) PE14   5  6 | PE15 (LCD_D5)       (BTN_EN2) PE11   5  6 | PA7  (SPI1 MOSI)
 * (LCD_D6) PD11 | 7  8 | PD10 (LCD_D7)       (SPI DET) PE12 | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | 3.3V
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PC5
#define EXP1_02_PIN                         PE13
#define EXP1_03_PIN                         PD13
#define EXP1_04_PIN                         PC6
#define EXP1_05_PIN                         PE14
#define EXP1_06_PIN                         PE15
#define EXP1_07_PIN                         PD11
#define EXP1_08_PIN                         PD10

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PE8
#define EXP2_04_PIN                         PE10
#define EXP2_05_PIN                         PE11
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PE12
#define EXP2_08_PIN                         -1    // RESET

//
// LCD SD
//
/*
#if SD_CONNECTION_IS(LCD)
  #define ENABLE_SPI1
  #define SDSS                       EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#endif
*/

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_DEVICE                           2  // Maple
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif

//
// LCD / Controller
//

#if ANY(TFT_COLOR_UI, TFT_LVGL_UI, TFT_CLASSIC_UI)
  #ifndef TOUCH_CALIBRATION_X
    #define TOUCH_CALIBRATION_X           -17253
  #endif
  #ifndef TOUCH_CALIBRATION_Y
    #define TOUCH_CALIBRATION_Y            11579
  #endif
  #ifndef TOUCH_OFFSET_X
    #define TOUCH_OFFSET_X                   514
  #endif
  #ifndef TOUCH_OFFSET_Y
    #define TOUCH_OFFSET_Y                   -24
  #endif
  #ifndef TOUCH_ORIENTATION
    #define TOUCH_ORIENTATION    TOUCH_LANDSCAPE
  #endif

  #define TFT_CS_PIN                 EXP1_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN
  #define TFT_DC_PIN                 EXP1_08_PIN
  #define TFT_A0_PIN                  TFT_DC_PIN

  #define TFT_RESET_PIN              EXP1_04_PIN
  #define LCD_BACKLIGHT_PIN          EXP1_03_PIN
  #define TFT_BACKLIGHT_PIN    LCD_BACKLIGHT_PIN

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          1

  #ifndef TFT_WIDTH
    #define TFT_WIDTH                        480
  #endif
  #ifndef TFT_HEIGHT
    #define TFT_HEIGHT                       320
  #endif

  #define TOUCH_CS_PIN               EXP1_05_PIN  // SPI1_NSS
  #define TOUCH_SCK_PIN              EXP2_02_PIN  // SPI1_SCK
  #define TOUCH_MISO_PIN             EXP2_01_PIN  // SPI1_MISO
  #define TOUCH_MOSI_PIN             EXP2_06_PIN  // SPI1_MOSI

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #define LCD_READ_ID                       0xD3
  #define LCD_USE_DMA_SPI

  //#define TFT_DRIVER                    ST7796
  #define TFT_BUFFER_WORDS                 14400

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define LCD_BACKLIGHT_PIN                 -1

  // MKS MINI12864 and MKS LCD12864B; If using MKS LCD12864A (Need to remove RPK2 resistor)
  #if ENABLED(MKS_MINI_12864)
    //#define LCD_BACKLIGHT_PIN             -1
    //#define LCD_RESET_PIN                 -1
    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_06_PIN
    //#define DOGLCD_SCK             EXP2_02_PIN
    //#define DOGLCD_MOSI            EXP2_06_PIN

    // Required for MKS_MINI_12864 with this board
    //#define MKS_LCD12864B

  #else // !MKS_MINI_12864

    #define LCD_PINS_D4              EXP1_05_PIN
    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
    #endif

    #define BOARD_ST7920_DELAY_1              96
    #define BOARD_ST7920_DELAY_2              48
    #define BOARD_ST7920_DELAY_3             600

  #endif // !MKS_MINI_12864

#endif // HAS_WIRED_LCD
