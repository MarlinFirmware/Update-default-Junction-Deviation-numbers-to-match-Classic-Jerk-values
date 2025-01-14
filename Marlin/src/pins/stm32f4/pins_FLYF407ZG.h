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

#if HOTENDS > 6 || E_STEPPERS > 6
  #error "FLYF407ZG supports up to 6 hotends / E steppers."
#endif

#define BOARD_INFO_NAME      "FLYF407ZG"
#define BOARD_WEBSITE_URL    "github.com/FLYmaker/FLYF407ZG"
#define DEFAULT_MACHINE_NAME BOARD_INFO_NAME

// Avoid conflict with fans and TIMER_TONE
#define TEMP_TIMER  3
#define STEP_TIMER  5

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
  //#define SRAM_EEPROM_EMULATION
  //#define I2C_EEPROM
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across
  // the 128kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#elif ENABLED(I2C_EEPROM)
  #define MARLIN_EEPROM_SIZE              0x2000  // 8K
#endif

#ifndef MARLIN_EEPROM_SIZE
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                          PE11

//
// Limit Switches
//
#define X_MIN_PIN                           PC3
#define X_MAX_PIN                           PC2
#define Y_MIN_PIN                           PF2
#define Y_MAX_PIN                           PF1
#define Z_MIN_PIN                           PF0
#define Z_MAX_PIN                           PC15

//
// Z Probe (when not Z_MIN_PIN)
//
#define Z_MIN_PROBE_PIN                     PC14  // Z3_PIN

//
// Steppers
//

#define X_STEP_PIN                          PB9
#define X_DIR_PIN                           PE0
#define X_ENABLE_PIN                        PE1
#ifndef X_CS_PIN
  #define X_CS_PIN                          PG13
#endif

#define Y_STEP_PIN                          PB8
#define Y_DIR_PIN                           PG11
#define Y_ENABLE_PIN                        PG12
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PG10
#endif

#define Z_STEP_PIN                          PA8
#define Z_DIR_PIN                           PD6
#define Z_ENABLE_PIN                        PD7
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD5
#endif

#define E0_STEP_PIN                         PC7
#define E0_DIR_PIN                          PD3
#define E0_ENABLE_PIN                       PD4
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD1
#endif

#define E1_STEP_PIN                         PC6
#define E1_DIR_PIN                          PA15
#define E1_ENABLE_PIN                       PD0
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PA14
#endif

#define E2_STEP_PIN                         PD15
#define E2_DIR_PIN                          PG7
#define E2_ENABLE_PIN                       PG8
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PG6
#endif

#define E3_STEP_PIN                         PD14
#define E3_DIR_PIN                          PG4
#define E3_ENABLE_PIN                       PG5
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PG3
#endif

#define E4_STEP_PIN                         PD13
#define E4_DIR_PIN                          PD11
#define E4_ENABLE_PIN                       PG2
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PD10
#endif

#define E5_STEP_PIN                         PD12
#define E5_DIR_PIN                          PD8
#define E5_ENABLE_PIN                       PD9
#ifndef E5_CS_PIN
  #define E5_CS_PIN                         PB12
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // Analog Input
#define TEMP_1_PIN                          PC1   // Analog Input
#define TEMP_2_PIN                          PC0   // Analog Input
#define TEMP_3_PIN                          PF10  // Analog Input
#define TEMP_4_PIN                          PF5   // Analog Input
#define TEMP_5_PIN                          PF4   // Analog Input
#define TEMP_BED_PIN                        PF3   // Analog Input

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PF7
#define HEATER_1_PIN                        PF6
#define HEATER_2_PIN                        PE6
#define HEATER_3_PIN                        PE5
#define HEATER_4_PIN                        PE4
#define HEATER_5_PIN                        PE3
#define HEATER_BED_PIN                      PE2

#ifndef FAN0_PIN
  #define FAN0_PIN                          PF8
#endif
#define FAN1_PIN                            PF9
#define FAN2_PIN                            PA2
#define FAN3_PIN                            PA1
#define FAN4_PIN                            PE13
#define FAN5_PIN                            PB11

/**
 *        ------                ------
 *  PB10 | 1  2 | PE15    PB14 | 1  2 | PB13
 *  PE14 | 3  4 | PE12    PC5  | 3  4 | PF11
 *  PE10   5  6 | PE9     PC4    5  6 | PB15
 *  PE8  | 7  8 | PE7     PB2  | 7  8 | RESET
 *   GND | 9 10 | 5V       GND | 9 10 | --
 *        ------                ------
 *         EXP1                  EXP2
 */
#define EXP1_01_PIN                         PB10
#define EXP1_02_PIN                         PE15
#define EXP1_03_PIN                         PE14
#define EXP1_04_PIN                         PE12
#define EXP1_05_PIN                         PE10
#define EXP1_06_PIN                         PE9
#define EXP1_07_PIN                         PE8
#define EXP1_08_PIN                         PE7

#define EXP2_01_PIN                         PB14
#define EXP2_02_PIN                         PB13
#define EXP2_03_PIN                         PC5
#define EXP2_04_PIN                         PF11
#define EXP2_05_PIN                         PC4
#define EXP2_06_PIN                         PB15
#define EXP2_07_PIN                         PB2
#define EXP2_08_PIN                         -1    // RESET

//
// Onboard SD support
//
//#define SD_CARD_DETECT_PIN                PC13

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)

  #define ONBOARD_SDIO                            // Use SDIO for onboard SD

  #if DISABLED(ONBOARD_SDIO)
    #define SOFTWARE_SPI                          // Use soft SPI for onboard SD
    #define SDSS                            PC11
    #define SD_SCK_PIN                      PC12
    #define SD_MISO_PIN                     PC8
    #define SD_MOSI_PIN                     PD2
  #endif

#elif SD_CONNECTION_IS(LCD)

  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SDSS                       EXP2_04_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN

#endif

//
// Trinamic SPI
//
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                EXP2_02_PIN
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO               EXP2_01_PIN
#endif
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI               EXP2_06_PIN
#endif

//
// Trinamic Software Serial
//

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PG13
  #define Y_SERIAL_TX_PIN                   PG10
  #define Z_SERIAL_TX_PIN                   PD5
  #define E0_SERIAL_TX_PIN                  PD1
  #define E1_SERIAL_TX_PIN                  PA14
  #define E2_SERIAL_TX_PIN                  PG6
  #define E3_SERIAL_TX_PIN                  PG3
  #define E4_SERIAL_TX_PIN                  PD10
  #define E5_SERIAL_TX_PIN                  PB12
#endif

//
// LCD / Controller
//

#if IS_RRD_SC
  #define BEEPER_PIN                 EXP1_01_PIN
  #define LCD_PINS_RS                EXP1_04_PIN
  #define LCD_PINS_EN                EXP1_03_PIN
  #define LCD_PINS_D4                EXP1_05_PIN
  #define LCD_PINS_D5                EXP1_06_PIN
  #define LCD_PINS_D6                EXP1_07_PIN
  #define LCD_PINS_D7                EXP1_08_PIN
  #define BTN_EN1                    EXP2_05_PIN
  #define BTN_EN2                    EXP2_03_PIN
  #define BTN_ENC                    EXP1_02_PIN
#endif

//
// Filament runout
//

#define FIL_RUNOUT_PIN                      PA3

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1                96
  #define BOARD_ST7920_DELAY_2                48
  #define BOARD_ST7920_DELAY_3               715
#endif
