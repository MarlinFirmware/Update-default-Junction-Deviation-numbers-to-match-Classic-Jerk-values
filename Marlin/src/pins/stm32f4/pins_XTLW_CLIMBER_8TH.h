/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define BOARD_INFO_NAME   "XTLW Climber-8th"

// Use one of these or SDCard-based Emulation will be used
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER                            10

//
// Servos
//
#define SERVO0_PIN                          PC2

//
// Limit Switches
//
#define X_MIN_PIN                           PC6
#define X_MAX_PIN                           PC7
#define Y_STOP_PIN                          PD10
#define Z_STOP_PIN                          PD11

//
// Z Probe
//
#define Z_MIN_PROBE_PIN                     PC3

//
// Steppers
//
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PC0
#define X_ENABLE_PIN                        PC1
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC15
#endif

#define X2_STEP_PIN                         PE4
#define X2_DIR_PIN                          PC13
#define X2_ENABLE_PIN                       PC14
#ifndef X2_CS_PIN
  #define X2_CS_PIN                         PA8
#endif

#define Y_STEP_PIN                          PE3
#define Y_DIR_PIN                           PB7
#define Y_ENABLE_PIN                        PB8
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PB6
#endif

#define Z_STEP_PIN                          PE2
#define Z_DIR_PIN                           PD6
#define Z_ENABLE_PIN                        PD7
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD5
#endif

#define E0_STEP_PIN                         PE1
#define E0_DIR_PIN                          PD3
#define E0_ENABLE_PIN                       PD4
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD1
#endif

#define E1_STEP_PIN                         PE0
#define E1_DIR_PIN                          PA15
#define E1_ENABLE_PIN                       PD0
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PE9
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC5
#define TEMP_0_PIN                          PC4
#define TEMP_1_PIN                          PA3

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PD12
#define HEATER_1_PIN                        PD13
#define HEATER_BED_PIN                      PB0

#define FAN0_PIN                            PA2
#define FAN1_PIN                            PA0
#define FAN2_PIN                            PA1
#define FAN3_PIN                            PB9
#define FAN4_PIN                            PE6

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                      PE10
#define FIL_RUNOUT2_PIN                     PE12

//
// Software SPI pins for TMC2130 stepper drivers
//
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PB5
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PB4
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PB3
  #endif
#endif

#if HAS_TMC_UART

  #define X_SERIAL_TX_PIN                   PC15
  #define X2_SERIAL_TX_PIN                  PA8
  #define Y_SERIAL_TX_PIN                   PB6
  #define Z_SERIAL_TX_PIN                   PD5
  #define E0_SERIAL_TX_PIN                  PD1
  #define E1_SERIAL_TX_PIN                  PE9

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    19200
#endif

//
// SD Connection
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION                 ONBOARD
#endif

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #define SDIO_SUPPORT                            // Use SDIO for onboard SD
  #define SDIO_D0_PIN                       PC8
  #define SDIO_D1_PIN                       PC9
  #define SDIO_D2_PIN                       PC10
  #define SDIO_D3_PIN                       PC11
  #define SDIO_CK_PIN                       PC12
  #define SDIO_CMD_PIN                      PD2
#elif SD_CONNECTION_IS(LCD)
  #define CUSTOM_SPI_PINS
  #define SD_SS_PIN                         PA4
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_DETECT_PIN                     PE13
#endif

/**
 *               -----                                             -----
 *           NC | 1 2 | GND                                    5V | 1 2 | GND
 *        RESET | 3 4 | PE13 (SD_DETECT)            (LCD_D7) PA14 | 3 4 | PA13 (LCD_D6)
 *   (MOSI) PA7 | 5 6   PD14 (BTN_EN1)              (LCD_D5) PE14 | 5 6   PE15 (LCD_D4)
 *  (SD_SS) PA4 | 7 8 | PD15 (BTN_EN2)              (LCD_RS)  PE8 | 7 8 | PB2  (LCD_EN)
 *    (SCK) PA5 | 9 10| PA6  (MISO)                (BTN_ENC)  PE7 | 9 10| PB1  (BEEPER)
 *               -----                                             -----
 *               EXP2                                              EXP1
 */

#define EXP1_03_PIN                         PA14
#define EXP1_04_PIN                         PA13
#define EXP1_05_PIN                         PE14
#define EXP1_06_PIN                         PE15
#define EXP1_07_PIN                         PE8
#define EXP1_08_PIN                         PB2
#define EXP1_09_PIN                         PE7
#define EXP1_10_PIN                         PB1

#define EXP2_03_PIN                         -1
#define EXP2_04_PIN                         PE13
#define EXP2_05_PIN                         PA7
#define EXP2_06_PIN                         PD14
#define EXP2_07_PIN                         PA4
#define EXP2_08_PIN                         PD15
#define EXP2_09_PIN                         PA5
#define EXP2_10_PIN                         PA6

//
// LCDs and Controllers
//
#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_08_PIN
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_10_PIN
  #define BTN_ENC                    EXP1_09_PIN

  #if ENABLED(MKS_MINI_12864)

    #define DOGLCD_A0                EXP1_04_PIN
    #define DOGLCD_CS                EXP1_05_PIN
    #define BTN_EN1                  EXP2_06_PIN
    #define BTN_EN2                  EXP2_08_PIN

  #else

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP2_06_PIN
    #define BTN_EN2                  EXP2_08_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_08_PIN
      #define DOGLCD_A0              EXP1_07_PIN
      //#define LCD_BACKLIGHT_PIN           -1
      #define LCD_RESET_PIN          EXP1_06_PIN  // Must be high or open for LCD to operate normally.
      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_05_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_04_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_03_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_05_PIN
      #endif
    #endif // !FYSETC_MINI_12864

    #if IS_ULTIPANEL
      #define LCD_PINS_D5            EXP1_05_PIN
      #define LCD_PINS_D6            EXP1_04_PIN
      #define LCD_PINS_D7            EXP1_03_PIN
    #endif

  #endif

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if ENABLED(U8GLIB_ST7920)
  #ifndef BOARD_ST7920_DELAY_1
    #define BOARD_ST7920_DELAY_1             120
  #endif
  #ifndef BOARD_ST7920_DELAY_2
    #define BOARD_ST7920_DELAY_2              80
  #endif
  #ifndef BOARD_ST7920_DELAY_3
    #define BOARD_ST7920_DELAY_3             580
  #endif
#endif
