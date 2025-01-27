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

#include "env_validate.h"

#define BOARD_INFO_NAME "ZONESTAR ZM3E4 V2.0"

//#define DISABLE_DEBUG
#define DISABLE_JTAG

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//#define OPTION_DUALZ_DRIVE
//#define OPTION_Z2_ENDSTOP
//#define OPTION_REPEAT_PRINTING
//#define SWITCH_EXTRUDER_SEQUENCE

//=============================================================================
// Zonestar ZM3E4 V2.0 (STM32F103VET6) board pin assignments
//=============================================================================
//  PA0                 |  PB0     HEAT_1          |  PC0         AUX_SDA
//  PA1                 |  PB1     FAN1            |  PC1         TEMP_E1
//  PA2     HEAT_BED    |  PB2     BOOT1           |  PC2         TEMP_E0
//  PA3     PWR_HOLD    |  PB3     SPI3_SCK        |  PC3         TEMP_BED
//  PA4     SD_CS       |  PB4     SPI3_MISO       |  PC4         SD_DETECT
//  PA5     SD_SCK      |  PB5     SPI3_MOSI       |  PC5         HEAT_0
//  PA6     SD_MISO     |  PB6     SERVO3          |  PC6         E1_STEP
//  PA7     SD_MOSI     |  PB7     SERVO2          |  PC7         E1_EN
//  PA8     X_DIR       |  PB8     FAN2/SERVO1     |  PC8         FIL_RUNOUT_PIN
//  PA9     UART1_RX    |  PB9     SERVO0          |  PC9         E0_DIR
//  PA10    UART1_TX    |  PB10    TX3             |  PC10        E0_EN
//  PA11    USB_D-      |  PB11    RX3             |  PC11        Z2_EN
//  PA12    USB_D+      |  PB12    LED             |  PC12        Z2_STEP
//  PA13    SWD_SDO     |  PB13    Z1_MAX          |  PC13        X_MIN
//  PA14    SWD_SCK     |  PB14    Y_MAX           |  PC14        WIFI_RST
//  PA15    SPI3_CS     |  PB15    PWR_DET         |  PC15        WIFI_CS
//  PD0     Z2_DIR      |  PE0     Y_EN
//  PD1     Z2_MIN      |  PE1     Y_STEP
//  PD2     Z1_EN       |  PE2     Y_DIR
//  PD3     Z1_STEP     |  PE3     Y_MIN
//  PD4     Z1_DIR      |  PE4     X_DIR
//  PD5     WIFI_RXD    |  PE5     X_STEP
//  PD6     WIFI_TXD    |  PE6     X_EN
//  PD7     Z1_MIN      |  PE7     AUX_SCL
//  PD8     X_MAX       |  PE8     BTN_EN1
//  PD9     E3_DIR      |  PE9     LCD_SCK
//  PD10    E3_STEP     |  PE10    LCD_MOSI
//  PD11    E3_EN       |  PE11    BEEPER
//  PD12    E2_DIR      |  PE12    LCD_EN
//  PD13    E2_STEP     |  PE13    KILL
//  PD14    E2_EN       |  PE14    BTN_EN2
//  PD15    E1_DIR      |  PE15    BTN_ENC

//=============================================================================
// EXP1 connector
//     MARK     I/O     ZONESTAR_LCD12864   REPRAPDISCOUNT_LCD12864
//  10 RS       PE13    KILL                BTN_ENC
//   9 BP       PE11    BEEP                BEEP
//   8 EN       PE12    DOGLCD_CS           LCDRS
//   7 MOSI     PE10    DOGLCD_SCK          LCDE
//   6 EN1      PE8     BTN_EN1             NC
//   5 SCK      PE9     DOGLCD_MOSI         LCD4
//   4 ENC      PE15    BTN_ENC             NC
//   3 EN2      PE14    BTN_EN2             NC
//   2 +5V
//   1 GND

#define EXP1_01_PIN                         PE13
#define EXP1_02_PIN                         PE11
#define EXP1_03_PIN                         PE12
#define EXP1_04_PIN                         PE10
#define EXP1_05_PIN                         PE8
#define EXP1_06_PIN                         PE9
#define EXP1_07_PIN                         PE15
#define EXP1_08_PIN                         PE14

// EXP2 connector
//     MARK     I/O     ZONESTAR_LCD12864   REPRAPDISCOUNT_LCD12864
//  10 SDA      PC0
//   9 SCL      PE7
//   8 RX1      PA9     UART1_RX
//   7 TX1      PA10    UART1_TX            BTN_EN2
//   6 CS3      PA15
//   5 MISO3    PB4                         BTN_EN1
//   4 MOSI3    PB5                         KILL
//   3 SCK3     PB3
//   2 +5V
//   1 GND

#define EXP2_01_PIN                         PC0
#define EXP2_02_PIN                         PE7
#define EXP2_03_PIN                         PA9
#define EXP2_04_PIN                         PA10
#define EXP2_05_PIN                         PA15
#define EXP2_06_PIN                         PB4
#define EXP2_07_PIN                         PB5
#define EXP2_08_PIN                         PB3

// AUX1 connector
//  1 +5V
//  2 GND
//  3 RX3       PB11    UART3_RX
//  4 TX3       PB10    UART3_TX
//  5 SCL       PE7
//  6 SDA       PC0

// WiFi
//  1 +5V
//  2 GND
//  3 WIFI_TXD  PD5     UART2_RX
//  4 WIFI_RXD  PD6     UART2_TX
//  5 WIFI_RST  PC14
//  6 WIFI_CS   PC15
//=============================================================================

//
// Servos
//
#define SERVO0_PIN                          PB9
//#define SERVO1_PIN                        PB8
#define SERVO2_PIN                          PB7
#define SERVO3_PIN                          PB6

//
// Limit Switches
//
#define X_MIN_PIN                           PC13
#define Y_MIN_PIN                           PE3
#define Z_MIN_PIN                           PD7
#define X_MAX_PIN                           PD8
#define Y_MAX_PIN                           PB14
#define Z_MAX_PIN                           PB13

// TODO: Use different pin names for auto-remapping
#ifdef OPTION_Z2_ENDSTOP
  #define Z2_MIN_PIN                        PD1
  #define Z2_MAX_PIN                        PB12
#endif

//
// Steppers
//
#if ENABLED(COREXY)
  #define X_ENABLE_PIN                      PE0
  #define X_STEP_PIN                        PE1
  #define X_DIR_PIN                         PE2

  #define Y_ENABLE_PIN                      PE6
  #define Y_STEP_PIN                        PE5
  #define Y_DIR_PIN                         PE4
#else
  #define X_ENABLE_PIN                      PE6
  #define X_STEP_PIN                        PE5
  #define X_DIR_PIN                         PE4

  #define Y_ENABLE_PIN                      PE0
  #define Y_STEP_PIN                        PE1
  #define Y_DIR_PIN                         PE2
#endif

#define Z_ENABLE_PIN                        PD2
#define Z_STEP_PIN                          PD3
#define Z_DIR_PIN                           PD4

#ifdef OPTION_DUALZ_DRIVE
  #define Z2_ENABLE_PIN                     PC11
  #define Z2_STEP_PIN                       PC12
  #define Z2_DIR_PIN                        PD0
#endif

#ifdef OPTION_REPEAT_PRINTING
  #define REPRINT_STOP_PIN                  PD8   // X_MAX_PIN
  #define FORWARD_PIN                       PA13
  #define BACK_PIN                          PA14
#endif

#ifdef SWITCH_EXTRUDER_SEQUENCE
  #define E3_ENABLE_PIN                     PC10
  #define E3_STEP_PIN                       PA8
  #define E3_DIR_PIN                        PC9

  #define E2_STEP_PIN                       PC6
  #define E2_DIR_PIN                        PD15
  #define E2_ENABLE_PIN                     PC7

  #define E1_STEP_PIN                       PD13
  #define E1_DIR_PIN                        PD12
  #define E1_ENABLE_PIN                     PD14

  #define E0_STEP_PIN                       PD10
  #define E0_DIR_PIN                        PD9
  #define E0_ENABLE_PIN                     PD11
#else
  #define E0_ENABLE_PIN                     PC10
  #define E0_STEP_PIN                       PA8
  #define E0_DIR_PIN                        PC9

  #define E1_STEP_PIN                       PC6
  #define E1_DIR_PIN                        PD15
  #define E1_ENABLE_PIN                     PC7

  #define E2_STEP_PIN                       PD13
  #define E2_DIR_PIN                        PD12
  #define E2_ENABLE_PIN                     PD14

  #define E3_STEP_PIN                       PD10
  #define E3_DIR_PIN                        PD9
  #define E3_ENABLE_PIN                     PD11
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC2   // TH0
//#define TEMP_1_PIN                        PC1   // TH1
#define TEMP_BED_PIN                        PC3   // TB1

//
// Heaters
//
#define HEATER_0_PIN                        PC5   // HEATER0
//#define HEATER_1_PIN                      PB0   // HEATER1
#define HEATER_BED_PIN                      PA2   // HOT BED

//
// Fans
//
#define FAN0_PIN                            PB1   // FAN1
#define FAN1_PIN                            PB8   // FAN2

//
// Misc. Functions
//
//#define POWER_LOSS_PIN                    PB15
#define LED_PIN                             PA0
#define SUICIDE_PIN                         PA3
#define FIL_RUNOUT_PIN                      PC8

//
// SD card
//
#define ENABLE_SPI1
#define SD_DETECT_PIN                       PC4
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7
#define SD_SS_PIN                           PA4

// WiFi Functions
#define WIFI_RST                            PC15
#define WIFI_EN                             PC14

#if ENABLED(ZONESTAR_12864LCD)
  #define LCDSCREEN_NAME "ZONESTAR LCD12864"
  #define LCD_PINS_RS                EXP1_03_PIN  // 7 CS make sure for zonestar zm3e4!
  #define LCD_PINS_EN                EXP1_06_PIN  // 6 DATA make sure for zonestar zm3e4!
  #define LCD_PINS_D4                EXP1_04_PIN  // 8 SCK make sure for zonestar zm3e4!
  #define BEEPER_PIN                 EXP1_02_PIN
  #define KILL_PIN                          -1    // EXP1_01_PIN
  #define BTN_EN1                    EXP1_05_PIN
  #define BTN_EN2                    EXP1_08_PIN
  #define BTN_ENC                    EXP1_07_PIN
#elif ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
  #define LCDSCREEN_NAME "REPRAPDISCOUNT LCD12864"
  #define LCD_PINS_RS                EXP2_03_PIN  // 7 CS make sure for zonestar zm3e4!
  #define LCD_PINS_EN                EXP2_06_PIN  // 6 DATA make sure for zonestar zm3e4!
  #define LCD_PINS_D4                EXP2_04_PIN  // 8 SCK make sure for zonestar zm3e4!
  #define BEEPER_PIN                 EXP2_01_PIN
  #define KILL_PIN                   EXP2_02_PIN
  #define BTN_EN1                    EXP2_08_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP2_07_PIN
#elif ENABLED(ZONESTAR_DWIN_LCD)
  // Connect to EXP2 connector
  #define LCDSCREEN_NAME "ZONESTAR DWIN LCD"
  #define BEEPER_PIN                 EXP2_05_PIN  // PE11
  #define KILL_PIN                          -1    // EXP1_01_PIN
  #define BTN_EN2                    EXP2_07_PIN  // PE8
  #define BTN_EN1                    EXP2_08_PIN  // PE14
  #define BTN_ENC                    EXP2_06_PIN  // PE15
#endif

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               200  // Tclk_fall <200ns
  #define BOARD_ST7920_DELAY_2               250  // Tdata_width >200ns
  #define BOARD_ST7920_DELAY_3               200  // Tclk_rise <200ns
#endif

// Remap SERVO0 PIN for BLTouch
#if ENABLED(BLTOUCH_ON_EXP1)
  // BLTouch connected to EXP1
  #define BLTOUCH_PROBE_PIN          EXP1_05_PIN
  #define BLTOUCH_GND_PIN            EXP1_07_PIN
  #undef SERVO0_PIN
  #define SERVO0_PIN                 EXP1_08_PIN
#elif ENABLED(BLTOUCH_ON_EXP2)
  // BLTouch connected to EXP2
  #define BLTOUCH_PROBE_PIN          EXP2_08_PIN
  #define BLTOUCH_GND_PIN            EXP2_07_PIN
  #undef SERVO0_PIN
  #define SERVO0_PIN                 EXP2_05_PIN
#else
  #define BLTOUCH_PROBE_PIN                 PB13  // Z1_MAX
#endif
