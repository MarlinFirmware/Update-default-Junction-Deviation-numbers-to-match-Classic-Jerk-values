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

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Fly RRF E3 V1"
#endif

#define USES_DIAG_JUMPERS // Requires Mellow/Fly TMC2209 with DIAG disable dip switch

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION
#endif
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K

//
// Servos
//
#define SERVO0_PIN                          PB0   // SERVOS

//
// Limit Switches
//
#define X_STOP_PIN                          PA2   // X-STOP
#define Y_STOP_PIN                          PA1   // Y-STOP
#define Z_STOP_PIN                          PC5   // Z-STOP

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PC4   // PROBE

//
// Steppers
//

#define X_ENABLE_PIN                        PC15
#define X_STEP_PIN                          PC13
#define X_DIR_PIN                           PC14

#define Y_ENABLE_PIN                        PE5
#define Y_STEP_PIN                          PE3
#define Y_DIR_PIN                           PE4

#define Z_ENABLE_PIN                        PE1
#define Z_STEP_PIN                          PB9
#define Z_DIR_PIN                           PE0

#define E0_ENABLE_PIN                       PB5
#define E0_STEP_PIN                         PB3
#define E0_DIR_PIN                          PB4

/**
 * TMC2208/TMC2209 stepper drivers
 */
#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PE6
  #define Y_SERIAL_TX_PIN                   PE2
  #define Z_SERIAL_TX_PIN                   PB8
  #define E0_SERIAL_TX_PIN                  PD7

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif
#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_BED_PIN                        PA3   // Analog Input "TB"
#define TEMP_0_PIN                          PA4   // Analog Input "TH0"
//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PA0   // "HB"
#define HEATER_0_PIN                        PA5   // "HE0"

#define FAN0_PIN                            PA7   // "FAN0"

#ifndef CONTROLLER_FAN_PIN
  #define CONTROLLER_FAN_PIN                PA6   // "FAN1"
#endif

/**
 *                 ------
 * (BEEPER)  PE12 | 1  2 | PE11  (BTN_ENC)
 * (BTN_EN1) PE10 | 3  4 | RESET
 * (BTN_EN2) PE9    5  6 | PE8 (LCD_D4)
 * (LCD_RS)  PE7  | 7  8 | PB1 (LCD_EN)
 *           GND  | 9 10 | 5V
 *                 ------
 *                  EXP1
 */
#define EXP1_01_PIN                         PE12
#define EXP1_02_PIN                         PE11
#define EXP1_03_PIN                         PE10
#define EXP1_04_PIN                         -1    // RESET
#define EXP1_05_PIN                         PE9
#define EXP1_06_PIN                         PE8
#define EXP1_07_PIN                         PE7
#define EXP1_08_PIN                         PB1

#if HAS_WIRED_LCD

  #if ANY(CR10_STOCKDISPLAY, LCD_FOR_MELZI)

    #define BEEPER_PIN               EXP1_01_PIN

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_D4              EXP1_06_PIN
    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN

    #if ENABLED(LCD_FOR_MELZI)

      CONTROLLER_WARNING("FLY_RRF_E3_V1", "LCD_FOR_MELZI")

      /** LCD_FOR_MELZI display pinout
       *
       *              Fly RRF E3 V1                                 Display Ribbon
       *                 ------                                         ------
       * (BEEPER)  PE12 | 1  2 | PE11  (BTN_ENC)                   GND |10  9 | 5V
       * (BTN_EN1) PE10 | 3  4 | RESET                          BEEPER | 8  7 | ESTOP    (RESET)
       * (BTN_EN2) PE9    5  6 | PE8 (LCD_D4)        (BTN_ENC) ENC_BTN | 6  5 | LCD_SCLK (LCD_D4)
       * (LCD_RS)  PE7  | 7  8 | PB1 (LCD_EN)        (BTN_EN2) ENC_A   | 4  3 | LCD_DATA (LCD_EN)
       *           GND  | 9 10 | 5V                  (BTN_EN1) ENC_B   | 2  1 | LCD_CS   (LCD_RS)
       *                 ------                                         ------
       *                  EXP1                                           LCD
       *
       * Needs custom cable:
       *
       *    Board   Adapter   Display Ribbon (coming from display)
       *  ----------------------------------
       *  EXP1-10 ---------- LCD-9   5V
       *  EXP1-9 ----------- LCD-10  GND
       *  EXP1-8 ----------- LCD-3   LCD_EN
       *  EXP1-7 ----------- LCD-1   LCD_RS
       *  EXP1-6 ----------- LCD-5   LCD_D4
       *  EXP1-5 ----------- LCD-4   EN2
       *  EXP1-4 ----------- LCD-7   RESET
       *  EXP1-3 ----------- LCD-2   EN1
       *  EXP1-2 ----------- LCD-6   BTN
       *  EXP1-1 ----------- LCD-8   BEEPER
       */

    #endif

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

    CONTROLLER_WARNING("FLY_RRF_E3_V1", "ZONESTAR_LCD")

    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_02_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                  PB0   // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

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

      CONTROLLER_WARNING("FLY_RRF_E3_V1", "TFTGLCD_PANEL_SPI")

      /**
       * TFTGLCD_PANEL_SPI display pinout
       *
       *                    Board                       Display
       *                   ------                       ------
       * (SD_DET)    PE12 | 1  2 | PE11 (BEEPER)    5V |10  9 | GND
       * (MOD_RESET) PE10 | 3  4 | RESET            -- | 8  7 | (SD_DET)
       * (SD_CS)     PE9    5  6 | PE8          (MOSI)   6  5 | --
       * (LCD_CS)    PE7  | 7  8 | PB1         (SD_CS) | 4  3 | (LCD_CS)
       *             GND  | 9 10 | 5V          (SCK)   | 2  1 | (MISO)
       *                   ------                       ------
       *                   EXP1                         EXP1
       *
       * Needs custom cable:
       *
       *    Board   Adapter   Display
       *   ----------------------------------
       *   EXP1-10 ---------- EXP1-10  5V
       *   EXP1-9 ----------- EXP1-9   GND
       *   SPI1-4 ----------- EXP1-6   MOSI
       *   EXP1-7 ----------- n/c
       *   SPI1-3 ----------- EXP1-2   SCK
       *   EXP1-5 ----------- EXP1-4   SD_CS
       *   EXP1-4 ----------- n/c
       *   EXP1-3 ----------- EXP1-3   LCD_CS
       *   SPI1-1 ----------- EXP1-1   MISO
       *   EXP1-1 ----------- EXP1-7   SD_DET
       */

      #define TFTGLCD_CS             EXP1_03_PIN

    #endif

  #else
    #error "Only CR10_STOCKDISPLAY, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, LCD_FOR_MELZI, and TFTGLCD_PANEL_(SPI|I2C) are currently supported on the FLY_RRF_E3_V1."
  #endif

  // Alter timing for graphical display
  #if ENABLED(LCD_FOR_MELZI)                      // LCD_FOR_MELZI default timing is too fast. This works but may be reduced.
    #define BOARD_ST7920_DELAY_1             200
    #define BOARD_ST7920_DELAY_2             400
    #define BOARD_ST7920_DELAY_3            1200
  #elif IS_U8GLIB_ST7920
    #define BOARD_ST7920_DELAY_1              96
    #define BOARD_ST7920_DELAY_2              48
    #define BOARD_ST7920_DELAY_3             600
  #endif

#endif // HAS_WIRED_LCD

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("FLY_RRF_E3_V1", "LCD_FYSETC_TFT81050")

  /** FYSETC TFT TFT81050 display pinout
   *
   *                    Board                          Display
   *                   ------                          ------
   * (SD_DET)    PE12 | 1  2 | PE11 (BEEPER)       5V |10  9 | GND
   * (MOD_RESET) PE10 | 3  4 | RESET            RESET | 8  7 | (SD_DET)
   * (SD_CS)     PE9    5  6 | PE8            (MOSI)  | 6  5 | (LCD_CS)
   * (LCD_CS)    PE7  | 7  8 | PB1            (SD_CS) | 4  3 | (MOD_RESET)
   *             GND  | 9 10 | 5V             (SCK)   | 2  1 | (MISO)
   *                   ------                          ------
   *                   EXP1                            EXP1
   *
   * Needs custom cable:
   *
   *    Board   Adapter   Display
   *   ----------------------------------
   *   EXP1-10 ---------- EXP1-10  5V
   *   EXP1-9 ----------- EXP1-9   GND
   *   SPI1-4 ----------- EXP1-6   MOSI
   *   EXP1-7 ----------- EXP1-5   LCD_CS
   *   SPI1-3 ----------- EXP1-2   SCK
   *   EXP1-5 ----------- EXP1-4   SD_CS
   *   EXP1-4 ----------- EXP1-8   RESET
   *   EXP1-3 ----------- EXP1-3   MOD_RST
   *   SPI1-1 ----------- EXP1-1   MISO
   *   EXP1-1 ----------- EXP1-7   SD_DET
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

#if SD_CONNECTION_IS(ONBOARD)
  #define ONBOARD_SDIO                            // Use SDIO for onboard SD
  //#define SDIO_CLOCK                  48000000
  #define SD_DETECT_PIN                     PA15
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with Fly RRF E3 V1."
#endif

//
// WIFI
//

#if ENABLED(WIFISUPPORT)
  #define ESP_WIFI_MODULE_COM                  3  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PE15
  #define ESP_WIFI_MODULE_ENABLE_PIN        -1    // EN is constantly pulled up to 3.3V on this board
  #define ESP_WIFI_MODULE_GPIO0_PIN         PE13
#endif
