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
 * MKS Robin E3 & E3D (STM32F103RCT6) common board pin assignments
 */

#include "env_validate.h"

#define BOARD_WEBSITE_URL "github.com/makerbase-mks"

#define BOARD_NO_NATIVE_USB
#define USES_DIAG_JUMPERS

//#define DISABLE_DEBUG
#define DISABLE_JTAG

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA3

//
// Limit Switches
//
#define X_STOP_PIN                          PA12
#define Y_STOP_PIN                          PA11
#define Z_STOP_PIN                          PC6

//
// Z Probe
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PB1
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
#define X_STEP_PIN                          PC0
#define X_DIR_PIN                           PB2
#define X_ENABLE_PIN                        PC13

#define Y_STEP_PIN                          PC2
#define Y_DIR_PIN                           PB9
#define Y_ENABLE_PIN                        PB12

#ifndef Z_STEP_PIN
  #define Z_STEP_PIN                        PB7
#endif
#ifndef Z_DIR_PIN
  #define Z_DIR_PIN                         PB6
#endif
#define Z_ENABLE_PIN                        PB8

#define E0_STEP_PIN                         PB4
#define E0_DIR_PIN                          PB3
#define E0_ENABLE_PIN                       PB5

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

  #define X_SERIAL_TX_PIN                   PC7
  #define Y_SERIAL_TX_PIN                   PD2
  #define Z_SERIAL_TX_PIN                   PC12
  #define E0_SERIAL_TX_PIN                  PC11

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Heaters 0,1 / Fans / Bed
//
#define HEATER_0_PIN                        PC9
#define FAN0_PIN                            PA8
#define HEATER_BED_PIN                      PC8

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PA1   // TB
#define TEMP_0_PIN                          PA0   // TH1

#define FIL_RUNOUT_PIN                      PB10  // MT_DET

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #define PS_ON_PIN                         PA14  // PW_OFF
  #define KILL_PIN                          PB10  // PW_DET
  #define KILL_PIN_STATE                    HIGH
#endif

/**
 *                ------                                    ------                                  ------
 *  (BEEPER) PC1 | 1  2 | PC3 (BTN_ENC)        (MISO) PB14 | 1  2 | PB13 (SD_SCK)    (BEEPER) PC1  | 1  2 | PC3 (BTN_ENC)
 *  (LCD_EN) PA4 | 3  4 | PA5 (LCD_RS)      (BTN_EN1) PB11 | 3  4 | PA15 (SD_SS)    (BTN_EN1) PB11 | 3  4 | RESET
 *  (LCD_D4) PA6   5  6 | PA7 (LCD_D5)      (BTN_EN2)  PB0   5  6 | PB15 (SD_MOSI)  (BTN_EN2) PB0    5  6 | PA6  (LCD_D4)
 *  (LCD_D6) PC4 | 7  8 | PC5 (LCD_D7)    (SD_DETECT) PC10 | 7  8 | RESET            (LCD_RS) PA5  | 7  8 | PA4  (LCD_EN)
 *           GND | 9 10 | 5V                           GND | 9 10 | --                         GND | 9 10 | 5V
 *                ------                                    ------                                  ------
 *                 EXP1                                      EXP2                                "Ender-3 EXP1"
 */
#define EXP1_01_PIN                         PC1
#define EXP1_02_PIN                         PC3
#define EXP1_03_PIN                         PA4
#define EXP1_04_PIN                         PA5
#define EXP1_05_PIN                         PA6
#define EXP1_06_PIN                         PA7
#define EXP1_07_PIN                         PC4
#define EXP1_08_PIN                         PC5

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PB11
#define EXP2_04_PIN                         PA15
#define EXP2_05_PIN                         PB0
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PC10
#define EXP2_08_PIN                         -1    // RESET

// "Ender-3 EXP1"
#define EXP3_01_PIN                         PC1
#define EXP3_02_PIN                         PC3
#define EXP3_03_PIN                         PB11
#define EXP3_04_PIN                         -1    // RESET
#define EXP3_05_PIN                         PB0
#define EXP3_06_PIN                         PA6
#define EXP3_07_PIN                         PA5
#define EXP3_08_PIN                         PA4

#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  // MKS MINI12864 and MKS LCD12864B; If using MKS LCD12864A (Need to remove RPK2 resistor)
  #if ENABLED(MKS_MINI_12864)

    #define LCD_BACKLIGHT_PIN               -1
    #define LCD_RESET_PIN                   -1
    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_06_PIN
    #define DOGLCD_SCK               EXP2_02_PIN
    #define DOGLCD_MOSI              EXP2_06_PIN

  #elif ENABLED(FYSETC_MINI_12864_2_1)

    #define LCD_PINS_DC              EXP1_04_PIN
    #define DOGLCD_CS                EXP1_03_PIN
    #define DOGLCD_A0                LCD_PINS_DC
    #define LCD_BACKLIGHT_PIN               -1
    #define LCD_RESET_PIN            EXP1_05_PIN
    #define NEOPIXEL_PIN             EXP1_06_PIN
    #define DOGLCD_MOSI              EXP2_06_PIN
    #define DOGLCD_SCK               EXP2_02_PIN
    #define FORCE_SOFT_SPI
    #define SOFTWARE_SPI
    //#define LCD_SCREEN_ROTATE              180  // 0, 90, 180, 270

  #else // !FYSETC_MINI_12864_2_1

    #define LCD_PINS_D4              EXP1_05_PIN
    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_06_PIN
      #define LCD_PINS_D6            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN

      #if !defined(BTN_ENC_EN) && ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
        #define BTN_ENC_EN           LCD_PINS_D7  // Detect the presence of the encoder
      #endif

    #endif

  #endif // !FYSETC_MINI_12864_2_1

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #ifndef BOARD_ST7920_DELAY_1
    #define BOARD_ST7920_DELAY_1             125
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2             125
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3             125
  #endif
#endif

// LED driving pin
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PA2
#endif

//
// SD Card
//
#define SDCARD_CONNECTION                ONBOARD
#define ONBOARD_SPI_DEVICE                     2  // Maple
#define ONBOARD_SD_CS_PIN            EXP2_04_PIN
#define SD_DETECT_PIN                EXP2_07_PIN
#define NO_SD_HOST_DRIVE

// TODO: This is the only way to set SPI for SD on STM32 (for now)
#define ENABLE_SPI2
#define SD_SCK_PIN                   EXP2_02_PIN
#define SD_MISO_PIN                  EXP2_01_PIN
#define SD_MOSI_PIN                  EXP2_06_PIN
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
