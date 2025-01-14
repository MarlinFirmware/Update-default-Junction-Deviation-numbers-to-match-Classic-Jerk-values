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

//#define BOARD_CUSTOM_BUILD_FLAGS -DTONE_CHANNEL=4 -DTONE_TIMER=4 -DTIMER_TONE=4

#include "env_validate.h"

#if HAS_MULTI_HOTEND || E_STEPPERS > 1
  #error "BTT SKR Mini E3 V3.0.1 supports up to 1 hotend / E stepper."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT SKR Mini E3 V3.0.1"
#endif

#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#ifndef NEOPIXEL_LED
  #define LED_PIN                           PA14
#endif

// Onboard I2C EEPROM
#if ANY(NO_EEPROM_SELECTED, I2C_EEPROM)
  #undef NO_EEPROM_SELECTED
  #define I2C_EEPROM
  #define SOFT_I2C_EEPROM                         // Force the use of Software I2C
  #define I2C_SCL_PIN                       PB8
  #define I2C_SDA_PIN                       PB9
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA0   // SERVOS

//
// Limit Switches
//
#define X_STOP_PIN                          PB5   // X-STOP
#define Y_STOP_PIN                          PB6   // Y-STOP
#define Z_STOP_PIN                          PB7   // Z-STOP

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PA1   // PROBE

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
  #define FIL_RUNOUT_PIN                    PC15  // E0-STOP
#endif

//
// Power-loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PC13  // Power Loss Detection: PWR-DET
#endif

#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PA14  // LED driving pin
#endif

#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PC14  // Power Supply Control
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC10
#define X_STEP_PIN                          PC11
#define X_DIR_PIN                           PC12

#define Y_ENABLE_PIN                        PB13
#define Y_STEP_PIN                          PB12
#define Y_DIR_PIN                           PB10

#define Z_ENABLE_PIN                        PB2
#define Z_STEP_PIN                          PB1
#define Z_DIR_PIN                           PB0

#define E0_ENABLE_PIN                       PC3
#define E0_STEP_PIN                         PC2
#define E0_DIR_PIN                          PC1

#if HAS_TMC_UART
  /**
   * TMC220x stepper drivers
   * Hardware serial communication ports
   */
  #define X_HARDWARE_SERIAL  MSerial6
  #define Y_HARDWARE_SERIAL  MSerial6
  #define Z_HARDWARE_SERIAL  MSerial6
  #define E0_HARDWARE_SERIAL MSerial6

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                    0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                    2
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                    1
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                   3
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_BTT_SKR_MINI_E3_V3_0_1.");
  static_assert(Y_SLAVE_ADDRESS == 2, "Y_SLAVE_ADDRESS must be 2 for BOARD_BTT_SKR_MINI_E3_V3_0_1.");
  static_assert(Z_SLAVE_ADDRESS == 1, "Z_SLAVE_ADDRESS must be 1 for BOARD_BTT_SKR_MINI_E3_V3_0_1.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_BTT_SKR_MINI_E3_V3_0_1.");
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC5   // Analog Input "TH0"
#define TEMP_BED_PIN                        PC4   // Analog Input "TB0"

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA15  // "HE"
#define HEATER_BED_PIN                      PB3   // "HB"
#define FAN0_PIN                            PC9   // "FAN0"
#define FAN1_PIN                            PA8   // "FAN1"
#define FAN2_PIN                            PC8   // "FAN2"

/**
 *              SKR Mini E3 V3.0.1
 *                  ------
 *  (BEEPER)  PB15 | 1  2 | PB14 (BTN_ENC)
 *  (BTN_EN1) PA9  | 3  4 | RESET
 *  (BTN_EN2) PA10   5  6 | PB4  (LCD_D4)
 *  (LCD_RS)  PD2  | 7  8 | PC0  (LCD_EN)
 *             GND | 9 10 | 5V
 *                  ------
 *                   EXP1
 */
#define EXP1_01_PIN                         PB15
#define EXP1_02_PIN                         PB14
#define EXP1_03_PIN                         PA9
#define EXP1_04_PIN                         -1    // RESET
#define EXP1_05_PIN                         PA10
#define EXP1_06_PIN                         PB4
#define EXP1_07_PIN                         PD2
#define EXP1_08_PIN                         PC0

#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  /**
   *        ------                ------                ------
   * (ENT) | 1  2 | (BEEP)       |10  9 |              |10  9 |
   *  (RX) | 3  4 |         (RX) | 8  7 | (TX)      RX | 8  7 | TX
   *  (TX)   5  6 |        (ENT)   6  5 | (BEEP)   ENT | 6  5 | BEEP
   *   (B) | 7  8 | (A)      (B) | 4  3 | (A)        B | 4  3 | A
   *   GND | 9 10 | (VCC)    GND | 2  1 | VCC      GND | 2  1 | VCC
   *        ------                ------                ------
   *         EXP1                  DWIN               DWIN (plug)
   *
   * All pins are labeled as printed on DWIN PCB. Connect TX-TX, A-A and so on.
   */

  #error "DWIN_CREALITY_LCD requires a custom cable, see diagram above this line. Comment out this line to continue."

  #define BEEPER_PIN                 EXP1_02_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN
  #define BTN_ENC                    EXP1_01_PIN

#elif HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN               EXP1_01_PIN
    #define BTN_ENC                  EXP1_02_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

    CONTROLLER_WARNING("BTT_SKR_MINI_E3_V3_0_1", "ZONESTAR_LCD")

    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_02_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                  PA1   // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

  #elif ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define DOGLCD_CS                EXP1_07_PIN
    #define DOGLCD_A0                EXP1_06_PIN
    #define DOGLCD_SCK               EXP1_01_PIN
    #define DOGLCD_MOSI              EXP1_08_PIN

    #define FORCE_SOFT_SPI
    #define LCD_BACKLIGHT_PIN               -1

  #elif IS_TFTGLCD_PANEL

    #if ENABLED(TFTGLCD_PANEL_SPI)

      CONTROLLER_WARNING("BTT_SKR_MINI_E3_V3_0_1", "TFTGLCD_PANEL_SPI")

      /**
       * TFTGLCD_PANEL_SPI display pinout
       *
       *                Board                                 Display
       *                ------                                ------
       * (BEEPER) PB6  | 1  2 | PB15 (SD_DET)             5V |10  9 | GND
       *         RESET | 3  4 | PA9  (MOD_RESET)          -- | 8  7 | (SD_DET)
       *          PB4    5  6 | PA10 (SD_CS)         (MOSI)  | 6  5 | --
       *          PB7  | 7  8 | PD2  (LCD_CS)        (SD_CS) | 4  3 | (LCD_CS)
       *           GND | 9 10 | 5V                   (SCK)   | 2  1 | (MISO)
       *                ------                                ------
       *                 EXP1                                  EXP1
       *
       * Needs custom cable:
       *
       *    Board             Display
       *
       *   EXP1-10 ---------- EXP1-10
       *   EXP1-9 ----------- EXP1-9
       *   SPI1-4 ----------- EXP1-6
       *   EXP1-7 ----------- FREE
       *   SPI1-3 ----------- EXP1-2
       *   EXP1-5 ----------- EXP1-4
       *   EXP1-4 ----------- FREE
       *   EXP1-3 ----------- EXP1-3
       *   SPI1-1 ----------- EXP1-1
       *   EXP1-1 ----------- EXP1-7
       */

      #define TFTGLCD_CS             EXP1_03_PIN

    #endif

  #else
    #error "Only CR10_STOCKDISPLAY, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, and TFTGLCD_PANEL_(SPI|I2C) are currently supported on the SKR Mini E3."
  #endif

#endif // HAS_WIRED_LCD

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("BTT_SKR_MINI_E3_V3_0_1", "LCD_FYSETC_TFT81050")

  /**
   * FYSETC TFT TFT81050 display pinout
   *
   *                   Board                             Display
   *                   ------                            ------
   * (SD_DET)    PB15 | 1  2 | PB6 (BEEPER)          5V |10  9 | GND
   * (MOD_RESET) PA9  | 3  4 | RESET            (RESET) | 8  7 | (SD_DET)
   * (SD_CS)     PA10   5  6 | PB4 (FREE)       (MOSI)  | 6  5 | (LCD_CS)
   * (LCD_CS)    PD2  | 7  8 | PB7 (FREE)       (SD_CS) | 4  3 | (MOD_RESET)
   *               5V | 9 10 | GND              (SCK)   | 2  1 | (MISO)
   *                   ------                            ------
   *                    EXP1                              EXP1
   *
   * Needs custom cable:
   *
   *    Board   Adapter   Display
   *           _________
   *   EXP1-10 ---------- EXP1-10
   *   EXP1-9 ----------- EXP1-9
   *   SPI1-4 ----------- EXP1-6
   *   EXP1-7 ----------- EXP1-5
   *   SPI1-3 ----------- EXP1-2
   *   EXP1-5 ----------- EXP1-4
   *   EXP1-4 ----------- EXP1-8
   *   EXP1-3 ----------- EXP1-3
   *   SPI1-1 ----------- EXP1-1
   *   EXP1-1 ----------- EXP1-7
   */

  #define CLCD_SPI_BUS                         1  // SPI1 connector

  #define BEEPER_PIN                 EXP1_02_PIN

  #define CLCD_MOD_RESET             EXP1_03_PIN
  #define CLCD_SPI_CS                EXP1_07_PIN

#endif // TOUCH_UI_FTDI_EVE && LCD_FYSETC_TFT81050

//
// SD Support
//

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD) && (ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050) || IS_TFTGLCD_PANEL)
  #define SD_DETECT_PIN              EXP1_01_PIN
  #define SD_SS_PIN                  EXP1_05_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with SKR Mini E3."
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1 -> used only by HAL/STM32F1...
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card

#define ENABLE_SPI1
#define SDSS                   ONBOARD_SD_CS_PIN
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7
