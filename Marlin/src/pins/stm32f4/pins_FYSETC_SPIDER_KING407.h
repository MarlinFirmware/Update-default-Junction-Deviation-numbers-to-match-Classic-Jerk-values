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

#if HOTENDS > 5 || E_STEPPERS > 5
  #error "FYSETC SPIDER KING supports up to 5 hotends / E-steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "FYSETC SPIDER KING"
#endif
#ifndef DEFAULT_MACHINE_NAME
  #define DEFAULT_MACHINE_NAME BOARD_INFO_NAME
#endif

//
// EEPROM Emulation
//
#if NO_EEPROM_SELECTED
  #undef NO_EEPROM_SELECTED
  //#define FLASH_EEPROM_EMULATION
  //#define SRAM_EEPROM_EMULATION
  #define I2C_EEPROM
#endif

#if ENABLED(I2C_EEPROM)
  #define I2C_EEPROM
  #define I2C_SCL_PIN                       PF1
  #define I2C_SDA_PIN                       PF0
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4KB
#endif

//
// Servos
//
#define SERVO0_PIN                          PA1

//
// Software SPI pins for TMC2130 stepper drivers
//
#define TMC_USE_SW_SPI
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PE14
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PE13
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PE12
  #endif
#endif

//
// Limit Switches
//
#define X_MIN_PIN                           PC5
#define Y_MIN_PIN                           PC4
#define Z_MIN_PIN                           PB6
#define X_MAX_PIN                           PB5
#define Y_MAX_PIN                           PF13
#define Z_MAX_PIN                           PF14

#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PA0
#endif

//
// Steppers
//
#define X_STEP_PIN                          PG7   // "MOT1"
#define X_DIR_PIN                           PG6
#define X_ENABLE_PIN                        PE11
#define X_CS_PIN                            PD2

#define X2_STEP_PIN                         PD11  // "MOT2"
#define X2_DIR_PIN                          PD10
#define X2_ENABLE_PIN                       PG10
#define X2_CS_PIN                           PE15

#define Y_STEP_PIN                          PG14  // "MOT3"
#define Y_DIR_PIN                           PG12
#define Y_ENABLE_PIN                        PG15
#define Y_CS_PIN                            PD8

#define Z_STEP_PIN                          PD4   // "MOT4"
#define Z_DIR_PIN                           PD6
#define Z_ENABLE_PIN                        PD5
#define Z_CS_PIN                            PD7

#define Z2_STEP_PIN                         PE5   // "MOT5"
#define Z2_DIR_PIN                          PC13
#define Z2_ENABLE_PIN                       PE6
#define Z2_CS_PIN                           PC14

#define E0_STEP_PIN                         PE3   // "MOT6"
#define E0_DIR_PIN                          PE4
#define E0_ENABLE_PIN                       PE2
#define E0_CS_PIN                           PC15

#define E1_STEP_PIN                         PG13  // "MOT7"
#define E1_DIR_PIN                          PG8
#define E1_ENABLE_PIN                       PG9
#define E1_CS_PIN                           PG3

#define E2_STEP_PIN                         PE1   // "MOT8"
#define E2_DIR_PIN                          PE0
#define E2_ENABLE_PIN                       PB2
#define E2_CS_PIN                           PD9

#define E3_STEP_PIN                         PF4   // "MOT9"
#define E3_DIR_PIN                          PF3
#define E3_ENABLE_PIN                       PF2
#define E3_CS_PIN                           PF5

#define E4_STEP_PIN                         PF15  // "MOT10"
#define E4_DIR_PIN                          PG0
#define E4_ENABLE_PIN                       PG5
#define E4_CS_PIN                           PG11

#if HAS_TMC_UART
  //
  // TMC2208/TMC2209 stepper drivers
  //
  #define X_SERIAL_TX_PIN                   PD2
  #define X2_SERIAL_TX_PIN                  PE15
  #define Y_SERIAL_TX_PIN                   PD8
  #define Z_SERIAL_TX_PIN                   PD7
  #define Z2_SERIAL_TX_PIN                  PC14
  #define E0_SERIAL_TX_PIN                  PC15
  #define E1_SERIAL_TX_PIN                  PG3
  #define E2_SERIAL_TX_PIN                  PD9
  #define E3_SERIAL_TX_PIN                  PF5
  #define E4_SERIAL_TX_PIN                  PG11

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1
#define TEMP_1_PIN                          PF9
#define TEMP_2_PIN                          PC3
#define TEMP_3_PIN                          PC2
#define TEMP_4_PIN                          PC0
//#define TEMP_CHAMBER_PIN                  PC0
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                      PF10
#endif

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PB4
#endif
#ifndef HEATER_1_PIN
  #define HEATER_1_PIN                      PB0
#endif
#ifndef HEATER_2_PIN
  #define HEATER_2_PIN                      PD13
#endif
#ifndef HEATER_3_PIN
  #define HEATER_3_PIN                      PC8
#endif
#ifndef HEATER_4_PIN
  #define HEATER_4_PIN                      PA15
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PE10
#endif

#ifndef FAN0_PIN
  #define FAN0_PIN                          PE8
#endif
#ifndef FAN1_PIN
  #define FAN1_PIN                          PE9
#endif
#ifndef FAN2_PIN
  #define FAN2_PIN                          PD15
#endif
#ifndef FAN3_PIN
  #define FAN3_PIN                          PD12
#endif
#ifndef FAN4_PIN
  #define FAN4_PIN                          PD14
#endif

/**
 *          ------                ------                 ------
 *     PA2 |10  9 | PA3      PA6 |10  9 | PA5       PC9 |10  9 | PA8
 *      -1 | 8  7 | PB10     PC6 | 8  7 | PA4      PC12 | 8  7 | PD0
 *     PA6   6  5 | PA7      PC7 | 6  5   PA7      PC10 | 6  5   PC11
 *     PA5 | 4  3 | PA4     PB10 | 4  3 | RESET     PG4 | 4  3 | PE7
 *     GND | 2  1 | 5V       GND | 2  1 | 5V        GND | 2  1 | 5V
 *          ------                ------                 ------
 *           EXP3                  EXP2                   EXP1
 */
#define EXP1_03_PIN                         PE7   // LCD_D7
#define EXP1_04_PIN                         PG4   // LCD_D6
#define EXP1_05_PIN                         PC11  // LCD_D5
#define EXP1_06_PIN                         PC10  // LCD_D4
#define EXP1_07_PIN                         PD0   // LCD_RS
#define EXP1_08_PIN                         PC12  // LCD_EN
#define EXP1_09_PIN                         PA8   // BTN_ENC
#define EXP1_10_PIN                         PC9   // BEEP

#define EXP2_03_PIN                         -1    // RESET
#define EXP2_04_PIN                         PB10  // CD
#define EXP2_05_PIN                         PA7   // MOSI
#define EXP2_06_PIN                         PC7   // BTN_EN2
#define EXP2_07_PIN                         PA4   // SS
#define EXP2_08_PIN                         PC6   // BTN_EN1
#define EXP2_09_PIN                         PA5   // SCK
#define EXP2_10_PIN                         PA6   // MISO

#define EXP3_03_PIN                         PA4   // SS
#define EXP3_04_PIN                         PA5   // SCK
#define EXP3_05_PIN                         PA7   // MOSI
#define EXP3_06_PIN                         PA6   // MISO
#define EXP3_07_PIN                         PB10  // CD
#define EXP3_08_PIN                         -1    // 3V3
#define EXP3_09_PIN                         PA3   // LCD_RX/RX
#define EXP3_10_PIN                         PA2   // LCD_TX/TX

//
// SPI / SD Card
//
#define SD_SCK_PIN                   EXP2_09_PIN
#define SD_MISO_PIN                  EXP2_10_PIN
#define SD_MOSI_PIN                  EXP2_05_PIN

#define SD_SS_PIN                    EXP2_07_PIN
#define SD_DETECT_PIN                EXP2_04_PIN

//
// LCD / Controller
//

#if ENABLED(FYSETC_242_OLED_12864)

  #define BEEPER_PIN                 EXP2_08_PIN

  #define BTN_EN1                    EXP1_10_PIN
  #define BTN_EN2                    EXP1_03_PIN
  #define BTN_ENC                    EXP1_09_PIN

  #define LCD_PINS_DC                EXP1_05_PIN
  #define LCD_PINS_RS                EXP2_06_PIN  // LCD_RST
  #define DOGLCD_CS                  EXP1_07_PIN
  #define DOGLCD_MOSI                EXP1_06_PIN
  #define DOGLCD_SCK                 EXP1_08_PIN
  #define DOGLCD_A0                  LCD_PINS_DC
  #define FORCE_SOFT_SPI

  //#define KILL_PIN                        -1    // NC
  #define NEOPIXEL_PIN               EXP1_04_PIN

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_10_PIN
  #define BTN_ENC                    EXP1_09_PIN

  #define LCD_PINS_RS                EXP1_07_PIN

  #define BTN_EN1                    EXP2_08_PIN
  #define BTN_EN2                    EXP2_06_PIN

  #define LCD_SDSS_PIN               EXP2_07_PIN

  #define LCD_PINS_EN                EXP1_08_PIN
  #define LCD_PINS_D4                EXP1_06_PIN

  #if ENABLED(FYSETC_MINI_12864)
    // See https://wiki.fysetc.com/Mini12864_Panel
    #define DOGLCD_CS                EXP1_08_PIN
    #define DOGLCD_A0                EXP1_07_PIN
    #if ENABLED(FYSETC_GENERIC_12864_1_1)
      #define LCD_BACKLIGHT_PIN      EXP1_04_PIN
    #endif
    #define LCD_RESET_PIN            EXP1_06_PIN  // Must be high or open for LCD to operate normally.
    #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      #ifndef RGB_LED_R_PIN
        #define RGB_LED_R_PIN        EXP1_05_PIN
      #endif
      #ifndef RGB_LED_G_PIN
        #define RGB_LED_G_PIN        EXP1_04_PIN
      #endif
      #ifndef RGB_LED_B_PIN
        #define RGB_LED_B_PIN        EXP1_03_PIN
      #endif
    #elif ENABLED(FYSETC_MINI_12864_2_1)
      #define NEOPIXEL_PIN           EXP1_05_PIN
    #endif
  #endif

  #if IS_ULTIPANEL
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_04_PIN
    #define LCD_PINS_D7              EXP1_03_PIN
    #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
      #define BTN_ENC_EN             LCD_PINS_D7  // Detect the presence of the encoder
    #endif
  #endif

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1              96
  #define BOARD_ST7920_DELAY_2              48
  #define BOARD_ST7920_DELAY_3              640
#endif

//
// Wifi module
//
#define ESP_WIFI_MODULE_COM                 1     // Set either SERIAL_PORT or SERIAL_PORT_2 to this
#define ESP_WIFI_MODULE_BAUDRATE        BAUDRATE  // Use the same BAUDRATE as SERIAL_PORT or SERIAL_PORT_2
#define ESP_WIFI_MODULE_RESET_PIN           PB3
#define ESP_WIFI_MODULE_ENABLE_PIN          PD1   // PC8
#define ESP_WIFI_MODULE_GPIO0_PIN           PG2   // PB4
#define ESP_WIFI_MODULE_GPIO4_PIN           PG1   // PB7

//
// NeoPixel LED
//
#ifndef NEOPIXEL_PIN
  #define NEOPIXEL_PIN                      PD3
#endif
