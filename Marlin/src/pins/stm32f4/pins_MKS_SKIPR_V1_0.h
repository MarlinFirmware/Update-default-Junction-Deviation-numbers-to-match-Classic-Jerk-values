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

#include "env_validate.h"

#if HOTENDS > 4 || E_STEPPERS > 4
  #error "MKS SKIPR supports up to 4 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS SKIPR V1.0"

// Valid SERIAL_PORT values: -1 (USB-C), 1 (direct to RK3328), 3 (USART3 header)

#define USES_DIAG_JUMPERS

// Onboard I2C EEPROM
#define I2C_EEPROM
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K (AT24C32)
#define I2C_SCL_PIN                         PB8
#define I2C_SDA_PIN                         PB9

//
// Servos
//
#define SERVO0_PIN                          PA8

//
// Trinamic Stallguard pins                       // Connector labels
#define X_DIAG_PIN                          PA14  // X-
#define Y_DIAG_PIN                          PA15  // Y-
#define Z_DIAG_PIN                          PB15  // Z-
#define E0_DIAG_PIN                         PA13  // MT-DET
#define E1_DIAG_PIN                         PC5   // NEOPIXEL
#define E2_DIAG_PIN                         PB14  // Z+

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN  // X-
#elif NEEDS_X_MINMAX
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // X-
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // MT-DET
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // X-
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN  // Y-
#elif NEEDS_Y_MINMAX
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // Y-
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // NEOPIXEL
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // Y-
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN  // Z-
#elif NEEDS_Z_MINMAX
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // Z-
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // Z+
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // Z-
#endif

#if DISABLED(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN) || ENABLED(USE_PROBE_FOR_Z_HOMING)
  #ifndef Z_MIN_PROBE_PIN
    #define Z_MIN_PROBE_PIN          E2_DIAG_PIN  // defaults to 'Z+' connector
  #endif
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
#define X_STEP_PIN                          PC14
#define X_DIR_PIN                           PC13
#define X_ENABLE_PIN                        PC15
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE6
#endif

#define Y_STEP_PIN                          PE5
#define Y_DIR_PIN                           PE4
#define Y_ENABLE_PIN                        PD14
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE3
#endif

#define Z_STEP_PIN                          PE1   // "Z1"
#define Z_DIR_PIN                           PE0
#define Z_ENABLE_PIN                        PE2
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB7
#endif

#define E0_STEP_PIN                         PB5
#define E0_DIR_PIN                          PB4
#define E0_ENABLE_PIN                       PB6
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB3
#endif

#define E1_STEP_PIN                         PD6   // "Z2"
#define E1_DIR_PIN                          PD5
#define E1_ENABLE_PIN                       PD7
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD4
#endif

#define E2_STEP_PIN                         PD2   // "Z3"
#define E2_DIR_PIN                          PD1
#define E2_ENABLE_PIN                       PD3
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD0
#endif

#define E3_STEP_PIN                         PC7   // "Z4"
#define E3_DIR_PIN                          PC6
#define E3_ENABLE_PIN                       PC8
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD15
#endif

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PC0   // TB
#define TEMP_0_PIN                          PC1   // TH0
#define TEMP_1_PIN                          PC2   // TH1
#define TEMP_2_PIN                          PC3   // TH2

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PD12  // Hotbed
#define HEATER_0_PIN                        PB1   // Heater0
#define HEATER_1_PIN                        PB0   // Heater1
#define HEATER_2_PIN                        PA3   // Heater2

#define FAN0_PIN                            PA2   // Fan0
#define FAN1_PIN                            PA1   // Fan1
#define FAN2_PIN                            PA0   // Fan2

//
// Default pins for TMC software SPI
// This board only supports SW SPI for stepper drivers
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #define TMC_SPI_MOSI                      PE14
  #define TMC_SPI_MISO                      PE13
  #define TMC_SPI_SCK                       PE12
#endif

//
// TMC2208/TMC2209 stepper drivers
// This board is routed for one-wire software serial
//
#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PE6
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PB7
  #define E0_SERIAL_TX_PIN                  PB3
  #define E1_SERIAL_TX_PIN                  PD4
  #define E2_SERIAL_TX_PIN                  PD0
  #define E3_SERIAL_TX_PIN                  PD15

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

/**               ------                                      ------
 * (BEEPER) PB2  | 1  2 | PE10 (BTN_ENC)         (MISO) PA6  | 1  2 | PA5  (SCK)
 * (LCD_EN) PE11 | 3  4 | PD10 (LCD_RS)       (BTN_EN1) PE9  | 3  4 | PA4  (SD_SS)
 * (LCD_D4) PD9  | 5  6   PD8  (LCD_D5)       (BTN_EN2) PE8  | 5  6   PA7  (MOSI)
 * (LCD_D6) PE15 | 7  8 | PE7  (LCD_D7)     (SD_DETECT) PD13 | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PB2
#define EXP1_02_PIN                         PE10
#define EXP1_03_PIN                         PE11
#define EXP1_04_PIN                         PD10
#define EXP1_05_PIN                         PD9
#define EXP1_06_PIN                         PD8
#define EXP1_07_PIN                         PE15
#define EXP1_08_PIN                         PE7

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PE9
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PE8
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PD13
#define EXP2_08_PIN                         -1    // connected to MCU reset

//
// SD Support
// Onboard SD card use hardware SPI3 (defined in variant), LCD SD card use hardware SPI1
//
#if HAS_MEDIA
  #ifndef SDCARD_CONNECTION
    #if ENABLED(NO_LCD_SDCARD)
      #define SDCARD_CONNECTION          ONBOARD
    #else
      #define SDCARD_CONNECTION              LCD
    #endif
  #endif
  #if SD_CONNECTION_IS(ONBOARD)
    //#define SOFTWARE_SPI
    //#define SD_SPI_SPEED        SPI_HALF_SPEED
    #undef SD_DETECT_STATE
    #define SD_DETECT_STATE                  LOW
    #define SD_DETECT_PIN                   PC4
  #elif SD_CONNECTION_IS(LCD)
    //#define SOFTWARE_SPI
    //#define SD_SPI_SPEED     SPI_QUARTER_SPEED
    #define SD_SS_PIN                EXP2_04_PIN
    #define SD_SCK_PIN               EXP2_02_PIN
    #define SD_MISO_PIN              EXP2_01_PIN
    #define SD_MOSI_PIN              EXP2_06_PIN
    #define SD_DETECT_PIN            EXP2_07_PIN
  #elif SD_CONNECTION_IS(CUSTOM_CABLE)
    #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for this board"
  #endif
#endif

//
// LCD / Controller
//

#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)

    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

    #if ENABLED(FYSETC_MINI_12864)
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      //#define LCD_BACKLIGHT_PIN           -1
      #define LCD_RESET_PIN          EXP1_05_PIN  // Must be high or open for LCD to operate normally.
      #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
        #ifndef RGB_LED_R_PIN
          #define RGB_LED_R_PIN      EXP1_06_PIN
        #endif
        #ifndef RGB_LED_G_PIN
          #define RGB_LED_G_PIN      EXP1_07_PIN
        #endif
        #ifndef RGB_LED_B_PIN
          #define RGB_LED_B_PIN      EXP1_08_PIN
        #endif
      #elif ENABLED(FYSETC_MINI_12864_2_1)
        #define NEOPIXEL_PIN         EXP1_06_PIN
      #endif
    #endif // !FYSETC_MINI_12864

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
  #define BOARD_ST7920_DELAY_1               120
  #define BOARD_ST7920_DELAY_2                80
  #define BOARD_ST7920_DELAY_3               580
#endif

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PC5
#endif

//
// MAX31865
//
#if HAS_MAX31865
  #define TEMP_0_CS_PIN                     PD11
  #define TEMP_0_SCK_PIN                    PE12
  #define TEMP_0_MISO_PIN                   PE13
  #define TEMP_0_MOSI_PIN                   PE14
#endif
