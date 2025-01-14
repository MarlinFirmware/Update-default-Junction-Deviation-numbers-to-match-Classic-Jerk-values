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
 * MKS Robin Pro (STM32F103ZET6) board pin assignments
 * Schematic: https://github.com/makerbase-mks/MKS-Robin-Pro/blob/master/hardware/MKS%20Robin%20Pro%20V1.0_001/MKS%20Robin%20Pro%20V1.0_001%20SCH.pdf
 */

#include "env_validate.h"

#if HOTENDS > 3 || E_STEPPERS > 3
  #error "MKS Robin pro supports up to 3 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS Robin pro"

#define BOARD_NO_NATIVE_USB

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// Onboard I2C EEPROM
//
#define IIC_BL24CXX_EEPROM                // Use I2C EEPROM onboard IC (AT24C04C, Size 4K, PageSize 16B)
#define MARLIN_EEPROM_SIZE                0x1000  // 4K
#define IIC_EEPROM_SDA                      PB7
#define IIC_EEPROM_SCL                      PB6
#define EEPROM_DEVICE_ADDRESS             0xA0

//
// SPI
//
#define SPI_DEVICE                             2  // Maple

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches
//
#define X_MIN_PIN                           PA15
#define X_MAX_PIN                           PG7
#define Y_MIN_PIN                           PA12
#define Y_MAX_PIN                           PG8
#define Z_MIN_PIN                           PA11
#define Z_MAX_PIN                           PC4

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
  #define X_CS_PIN                          PF8
#endif

#define Y_ENABLE_PIN                        PE1
#define Y_STEP_PIN                          PE0
#define Y_DIR_PIN                           PB9
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PF3
#endif

#define Z_ENABLE_PIN                        PB8
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB4
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PF6
#endif

#define E0_ENABLE_PIN                       PB3
#define E0_STEP_PIN                         PD6
#define E0_DIR_PIN                          PD3
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PG15
#endif

#define E1_ENABLE_PIN                       PA3
#define E1_STEP_PIN                         PA6
#define E1_DIR_PIN                          PA1
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PG10
#endif

#define E2_ENABLE_PIN                       PF0
#define E2_STEP_PIN                         PF2
#define E2_DIR_PIN                          PF1
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PG9
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB15
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB14
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB13
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  MSerial1
  //#define Y_HARDWARE_SERIAL  MSerial1
  //#define Z_HARDWARE_SERIAL  MSerial1
  //#define E0_HARDWARE_SERIAL MSerial1
  //#define E1_HARDWARE_SERIAL MSerial1
  //#define E2_HARDWARE_SERIAL MSerial1

  #define X_SERIAL_TX_PIN                   PF7
  #define X_SERIAL_RX_PIN                   PF8

  #define Y_SERIAL_TX_PIN                   PF4
  #define Y_SERIAL_RX_PIN                   PF3

  #define Z_SERIAL_TX_PIN                   PF5
  #define Z_SERIAL_RX_PIN                   PF6

  #define E0_SERIAL_TX_PIN                  PG13
  #define E0_SERIAL_RX_PIN                  PG15

  #define E1_SERIAL_TX_PIN                  PG12
  #define E1_SERIAL_RX_PIN                  PG10

  #define E2_SERIAL_TX_PIN                  PC13
  #define E2_SERIAL_RX_PIN                  PG9
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_2_PIN                          PC3   // TH3
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PF10  // +HE0-
#define HEATER_1_PIN                        PB0   // +HE1-
#define HEATER_2_PIN                        PF9   // +HE2-
#define HEATER_BED_PIN                      PA0   // +HOT-BED-
#define FAN0_PIN                            PB1   // +FAN-

//
// Note: Using SPI2 interface. Make sure stm32duino is configured accordingly
//
//#define TEMP_0_CS_PIN                     PE5   // TC1 - CS1
//#define TEMP_0_CS_PIN                     PF11  // TC2 - CS2

#define POWER_LOSS_PIN                      PA2   // PW_DET
#define FIL_RUNOUT_PIN                      PA4   // MT_DET1
#define FIL_RUNOUT2_PIN                     PE6   // MT_DET2
#define FIL_RUNOUT3_PIN                     PG14  // MT_DET3

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #if ENABLED(TFT_LVGL_UI)
    #if ENABLED(PSU_CONTROL)
      #error "PSU_CONTROL is incompatible with MKS_PWC plus TFT_LVGL_UI."
    #endif
    #undef MKS_PWC
    #define SUICIDE_PIN                     PG11
    #define SUICIDE_PIN_STATE               LOW
  #else
    #define PS_ON_PIN                       PG11  // PW_OFF
  #endif
  #define KILL_PIN                          PA2
  #define KILL_PIN_STATE                    HIGH
#endif

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD)
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_SS_PIN                  EXP2_04_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #define ONBOARD_SDIO
  #define SD_DETECT_PIN                     PD12
  #define ONBOARD_SD_CS_PIN                 PC11
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

/**               ------                                      ------
 * (BEEPER) PC5  | 1  2 | PG2  (BTN_ENC)         (MISO) PB14 | 1  2 | PB13 (SCK)
 * (LCD_EN) PG0  | 3  4 | PG1  (LCD_RS)       (BTN_EN1) PG5  | 3  4 | PG6  (SD_SS)
 * (LCD_D4) PF14   5  6 | PF15 (LCD_D5)       (BTN_EN2) PG4    5  6 | PB15 (MOSI)
 * (LCD_D6) PF12 | 7  8 | PF13 (LCD_D7)     (SD_DETECT) PG3  | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PC5
#define EXP1_02_PIN                         PG2
#define EXP1_03_PIN                         PG0
#define EXP1_04_PIN                         PG1
#define EXP1_05_PIN                         PF14
#define EXP1_06_PIN                         PF15
#define EXP1_07_PIN                         PF12
#define EXP1_08_PIN                         PF13

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PG5
#define EXP2_04_PIN                         PG6
#define EXP2_05_PIN                         PG4
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PG3
#define EXP2_08_PIN                         -1    // RESET

/**                -------
 *                |     0 | DGND-|
 *           3V3  |  1  2 | DGND-|
 *      (D0) PD14 |  3  4 | PD15 (D1)
 *      (D2) PD0  |  5  6 | PD1  (D3)
 *      (D4) PE7  |  7  8 | PE8  (D5)
 *      (D6) PE9  |  9 10 | PE10 (D7)
 *      (D8) PE11 | 11 12 | PE12 (D9)
 *     (D10) PE13 | 13 14 | PE14 (D11)
 *     (D12) PE15 | 15 16 | PD8  (D13)
 *     (D14) PD9  | 17 18 | PD10 (D15)
 *     (NE4) PD7  | 19 20 | PD11 (A0)
 *     (NWE) PD5  | 21 22 | PD4  (NOE)
 *     (RST) PC6? | 23 24 | PD13?(LIGHT)
 *   (MISO2) PB14 | 25 26 | ---  (INT)
 *   (MOSI2) PB15 | 27 28 | PC5  (BEEPER)
 *    (SCK2) PB13 | 29 30 | VCC
 *    (NSS2) PA7  | 31 32 | DGND
 *                 -------
 *                  FSMC
 */
//#define FSMC_00_PIN                       -1    // GND
//#define FSMC_01_PIN                       -1    // 3.3V
//#define FSMC_02_PIN                       -1    // GND
#define FSMC_03_PIN                         PD14  // D0
#define FSMC_04_PIN                         PD15  // D1
#define FSMC_05_PIN                         PD0   // D2
#define FSMC_06_PIN                         PD1   // D3
#define FSMC_07_PIN                         PE7   // D4
#define FSMC_08_PIN                         PE8   // D5
#define FSMC_09_PIN                         PE9   // D6
#define FSMC_10_PIN                         PE10  // D7
#define FSMC_11_PIN                         PE11  // D8
#define FSMC_12_PIN                         PE12  // D9
#define FSMC_13_PIN                         PE13  // D10
#define FSMC_14_PIN                         PE14  // D11
#define FSMC_15_PIN                         PE15  // D12
#define FSMC_16_PIN                         PD8   // D13
#define FSMC_17_PIN                         PD9   // D14
#define FSMC_18_PIN                         PD10  // D15
#define FSMC_19_PIN                         PD7   // NE4
#define FSMC_20_PIN                         PD11  // A0
#define FSMC_21_PIN                         PD5   // NWE
#define FSMC_22_PIN                         PD4   // NOE
#define FSMC_23_PIN                         PC6   // RST
#define FSMC_24_PIN                         PD13  // LIGHT
#define FSMC_25_PIN                         PB14  // MISO2
#define FSMC_26_PIN                         -1    // INT
#define FSMC_27_PIN                         PB15  // MOSI2
#define FSMC_28_PIN                         PC5   // BEEPER
#define FSMC_29_PIN                         PB13  // SCK2
//#define FSMC_30_PIN                       -1    // VCC
#define FSMC_31_PIN                         PA7   // NSS2
//#define FSMC_32_PIN                       -1    // GND

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers
   * Supported screens are based on the ILI9341, ST7789V and ILI9328 (320x240)
   * ILI9488 is not supported
   * Define init sequences for other screens in u8g_dev_tft_320x240_upscale_from_128x64.cpp
   *
   * If the screen stays white, disable 'TFT_RESET_PIN' to let the bootloader init the screen.
   *
   * Setting a 'TFT_RESET_PIN' may cause a flicker when switching menus
   * because Marlin uses the reset as a failsafe to revive a glitchy LCD.
   */
  #define TFT_RESET_PIN              FSMC_23_PIN
  #define TFT_BACKLIGHT_PIN          FSMC_24_PIN

  #define LCD_USE_DMA_FSMC
  #define FSMC_CS_PIN                FSMC_19_PIN  // NE4
  #define FSMC_RS_PIN                FSMC_20_PIN  // A0
  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TFT_BUFFER_WORDS                 14400

  #if NEED_TOUCH_PINS
    #define TOUCH_BUTTONS_HW_SPI
    #define TOUCH_BUTTONS_HW_SPI_DEVICE        2
    #define TOUCH_CS_PIN             FSMC_31_PIN  // SPI2_NSS
    #define TOUCH_SCK_PIN            FSMC_29_PIN  // SPI2_SCK
    #define TOUCH_MISO_PIN           FSMC_25_PIN  // SPI2_MISO
    #define TOUCH_MOSI_PIN           FSMC_27_PIN  // SPI2_MOSI
  #else
    #define BEEPER_PIN               FSMC_28_PIN
    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN
  #endif

#elif IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS                      PG5
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  // MKS MINI12864 and MKS LCD12864B. If using MKS LCD12864A (Need to remove RPK2 resistor)
  #if ANY(ENDER2_STOCKDISPLAY, MKS_MINI_12864)

    #define LCD_BACKLIGHT_PIN               -1
    #define LCD_RESET_PIN                   -1
    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_06_PIN
    #define DOGLCD_SCK               EXP2_02_PIN
    #define DOGLCD_MOSI              EXP2_06_PIN

  #else // !ENDER2_STOCKDISPLAY && !MKS_MINI_12864

    #define LCD_PINS_D4              EXP1_05_PIN
    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN

      #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               125
  #define BOARD_ST7920_DELAY_2               125
  #define BOARD_ST7920_DELAY_3               125
#endif

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define SPI_FLASH_SIZE               0x1000000  // 16MB
  #define SPI_FLASH_CS_PIN                  PB12  // Flash chip-select
  #define SPI_FLASH_SCK_PIN                 PB13
  #define SPI_FLASH_MISO_PIN                PB14
  #define SPI_FLASH_MOSI_PIN                PB15
#endif

//
// MKS WIFI pins
//
#if ENABLED(MKS_WIFI_MODULE)
  #define WIFI_RESET_PIN                    PA5
  #define WIFI_SERIAL_PORT                     1  // USART1
  #define WIFI_IO1_PIN                      PC7
  #define WIFI_IO0_PIN                      -1    // N.C.
#endif
