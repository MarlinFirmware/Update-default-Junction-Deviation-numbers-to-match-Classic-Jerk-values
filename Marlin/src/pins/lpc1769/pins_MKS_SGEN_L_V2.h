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
 * MKS SGen-L V2 pin assignments
 * Schematic: https://github.com/makerbase-mks/MKS-SGEN_L-V2/blob/master/Hardware/MKS%20SGEN_L%20V2.0_003/MKS%20SGEN_L%20V2.0_003%20SCH.pdf
 */

#include "env_validate.h"

#define BOARD_INFO_NAME   "MKS SGEN_L V2"
#define BOARD_WEBSITE_URL "github.com/makerbase-mks"

#define USES_DIAG_JUMPERS

//
// EEPROM, MKS SGEN_L V2.0 hardware has 4K EEPROM on the board
//
#if NO_EEPROM_SELECTED
  #define I2C_EEPROM                              // AT24C32
  #define MARLIN_EEPROM_SIZE              0x1000  // 4K
#endif

//
// Servos
//
#define SERVO0_PIN                         P1_23  // SERVO P1.23
#define SERVO1_PIN                         P2_00  // SERVO P2.0

//
// Trinamic Stallguard pins, can connect or disconnect by jumpers cap on the board
//
#define X_DIAG_PIN                         P1_29  // X-
#define Y_DIAG_PIN                         P1_27  // Y-
#define Z_DIAG_PIN                         P1_25  // Z-
#define E0_DIAG_PIN                        P1_28  // X+
#define E1_DIAG_PIN                        P1_26  // Y+

//
// Limit Switches
//
#if X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                      P1_28  // X+
  #else
    #define X_MIN_PIN                      P1_28  // X+
  #endif
#else
  #define X_MIN_PIN                        P1_29  // X-
  #define X_MAX_PIN                        P1_28  // X+
#endif

#if Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                      P1_26  // Y+
  #else
    #define Y_MIN_PIN                      P1_26  // Y+
  #endif
#else
  #define Y_MIN_PIN                        P1_27  // Y-
  #define Y_MAX_PIN                        P1_26  // Y+
#endif

#if Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                      P1_24  // Z+
  #else
    #define Z_MIN_PIN                      P1_24  // Z+
  #endif
#else
  #define Z_MIN_PIN                        P1_25  // Z-
  #define Z_MAX_PIN                        P1_24  // Z+
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                  P1_24
#endif

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Filament Runout Sensor
//
#define FIL_RUNOUT_PIN                     P1_28  // X+
#define FIL_RUNOUT2_PIN                    P1_26  // Y+

//
// Steppers
//
#define X_STEP_PIN                         P2_02
#define X_DIR_PIN                          P2_03
#define X_ENABLE_PIN                       P2_01
#ifndef X_CS_PIN
  #define X_CS_PIN                         P1_01
#endif

#define Y_STEP_PIN                         P0_19
#define Y_DIR_PIN                          P0_20
#define Y_ENABLE_PIN                       P2_08
#ifndef Y_CS_PIN
  #define Y_CS_PIN                         P1_08
#endif

#define Z_STEP_PIN                         P0_22
#define Z_DIR_PIN                          P2_11
#define Z_ENABLE_PIN                       P0_21
#ifndef Z_CS_PIN
  #define Z_CS_PIN                         P1_10
#endif

#define E0_STEP_PIN                        P2_13
#define E0_DIR_PIN                         P0_11
#define E0_ENABLE_PIN                      P2_12
#ifndef E0_CS_PIN
  #define E0_CS_PIN                        P1_15
#endif

#define E1_STEP_PIN                        P1_09
#define E1_DIR_PIN                         P1_14
#define E1_ENABLE_PIN                      P0_10
#ifndef E1_CS_PIN
  #define E1_CS_PIN                        P1_17
#endif

//
// Default pins for TMC software SPI
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                     P1_16
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                     P0_05
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                      P0_04
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

  #define X_SERIAL_TX_PIN                  P1_01
  #define Y_SERIAL_TX_PIN                  P1_08
  #define Z_SERIAL_TX_PIN                  P1_10
  #define E0_SERIAL_TX_PIN                 P1_15
  #define E1_SERIAL_TX_PIN                 P1_17

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
// 3.3V max when defined as an analog input
//
#define TEMP_0_PIN                      P0_23_A0  // Analog Input A0 (TH1)
#define TEMP_BED_PIN                    P0_24_A1  // Analog Input A1 (TB)
#define TEMP_1_PIN                      P0_25_A2  // Analog Input A2 (TH2)
#define TEMP_2_PIN                      P0_26_A3  // Analog Input A3 (P0.26, No pull up)

#if HOTENDS == 1 && !REDUNDANT_TEMP_MATCH(SOURCE, E1)
  #if TEMP_SENSOR_PROBE
    #define TEMP_PROBE_PIN            TEMP_1_PIN
  #elif TEMP_SENSOR_CHAMBER
    #define TEMP_CHAMBER_PIN          TEMP_1_PIN
  #endif
#endif

//
// Heaters / Fans
//
#define HEATER_BED_PIN                     P2_05
#define HEATER_0_PIN                       P2_07
#if HAS_MULTI_HOTEND
  #ifndef HEATER_1_PIN
    #define HEATER_1_PIN                   P2_06
  #endif
#else
  #ifndef FAN2_PIN
    #define FAN2_PIN                       P2_06  // HE1 for FAN3
  #endif
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                         P2_04  // FAN1
#endif
#ifndef FAN1_PIN
  #define FAN1_PIN                         P1_04  // FAN2
#endif

//
// Misc. Functions
//
#define LED_PIN                            P1_18  // Used as a status indicator

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #define PS_ON_PIN                        P2_00  // SERVO1
  #define KILL_PIN                         P1_24  // Z+
  #define KILL_PIN_STATE                    HIGH
#endif

//
// RGB LED
//
#if ENABLED(RGB_LED)
  #ifndef RGB_LED_R_PIN
    #define RGB_LED_R_PIN                  P1_19
  #endif
  #ifndef RGB_LED_G_PIN
    #define RGB_LED_G_PIN                  P1_20
  #endif
  #ifndef RGB_LED_B_PIN
    #define RGB_LED_B_PIN                  P1_21
  #endif
#else
  #define LED2_PIN                         P1_19  // Initialized by HAL/LPC1768/main.cpp
  #define LED3_PIN                         P1_20
  #define LED4_PIN                         P1_21
#endif

/**                ------                                     ------
 *  (BEEPER) 1.31 | 1  2 | 1.30 (BTN_ENC)        (MISO) 0.8  | 1  2 | 0.7  (SD_SCK)
 *  (LCD_EN) 0.18 | 3  4 | 0.16 (LCD_RS)      (BTN_EN1) 3.25 | 3  4 | 0.28 (SD_CS2)
 *  (LCD_D4) 0.15   5  6 | 0.17 (LCD_D5)      (BTN_EN2) 3.26   5  6 | 0.9  (SD_MOSI)
 *  (LCD_D6)  1.0 | 7  8 | 1.22 (LCD_D7)    (SD_DETECT) 0.27 | 7  8 | RST
 *            GND | 9 10 | 5V                            GND | 9 10 | --
 *                 ------                                     ------
 *                  EXP1                                       EXP2
 */
#define EXP1_01_PIN                        P1_31
#define EXP1_02_PIN                        P1_30
#define EXP1_03_PIN                        P0_18
#define EXP1_04_PIN                        P0_16
#define EXP1_05_PIN                        P0_15
#define EXP1_06_PIN                        P0_17
#define EXP1_07_PIN                        P1_00
#define EXP1_08_PIN                        P1_22

#define EXP2_01_PIN                        P0_08
#define EXP2_02_PIN                        P0_07
#define EXP2_03_PIN                        P3_25
#define EXP2_04_PIN                        P0_28
#define EXP2_05_PIN                        P3_26
#define EXP2_06_PIN                        P0_09
#define EXP2_07_PIN                        P0_27
#define EXP2_08_PIN                        -1     // RESET

#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

  #define SD_DETECT_PIN              EXP2_07_PIN

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

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_SDSS                 EXP2_04_PIN

    #if ENABLED(MKS_12864OLED_SSD1306)

      #define LCD_PINS_DC            EXP1_06_PIN
      #define DOGLCD_CS              EXP1_04_PIN
      #define DOGLCD_A0              LCD_PINS_DC
      #define DOGLCD_SCK             EXP1_05_PIN
      #define DOGLCD_MOSI            EXP1_03_PIN

      #define LCD_PINS_RS            EXP1_07_PIN
      #define LCD_PINS_D7            EXP1_08_PIN
      #define KILL_PIN                     -1     // NC

    #elif HAS_SPI_TFT                             // Config for Classic UI (emulated DOGM) and Color UI

      #define TFT_CS_PIN             EXP1_07_PIN
      #define TFT_DC_PIN             EXP1_08_PIN
      #define TFT_A0_PIN              TFT_DC_PIN
      #define TFT_MISO_PIN           EXP2_01_PIN
      #define TFT_BACKLIGHT_PIN      EXP1_03_PIN
      #define TFT_RESET_PIN          EXP1_04_PIN

      #define LCD_USE_DMA_SPI

      #define TOUCH_INT_PIN          EXP1_06_PIN
      #define TOUCH_CS_PIN           EXP1_05_PIN
      #define TOUCH_BUTTONS_HW_SPI
      #define TOUCH_BUTTONS_HW_SPI_DEVICE      2

      // Disable any LCD related PINs config
      #define LCD_PINS_EN                  -1
      #define LCD_PINS_RS                  -1

      #ifndef TFT_BUFFER_WORDS
        #define TFT_BUFFER_WORDS            1200
      #endif
      #ifndef TFT_QUEUE_SIZE
        #define TFT_QUEUE_SIZE              6144
      #endif

    #else // !MKS_12864OLED_SSD1306

      #define LCD_PINS_RS            EXP1_04_PIN

      #define LCD_PINS_EN            EXP1_03_PIN
      #define LCD_PINS_D4            EXP1_05_PIN

      #if ENABLED(FYSETC_MINI_12864)

        #define DOGLCD_CS            EXP1_03_PIN
        #define DOGLCD_A0            EXP1_04_PIN
        #define DOGLCD_SCK           EXP2_02_PIN
        #define DOGLCD_MOSI          EXP2_06_PIN

        #define LCD_BACKLIGHT_PIN          -1

        #define FORCE_SOFT_SPI                    // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        #define LCD_RESET_PIN        EXP1_05_PIN  // Must be high or open for LCD to operate normally.

        #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            #define RGB_LED_R_PIN    EXP1_06_PIN
          #endif
          #ifndef RGB_LED_G_PIN
            #define RGB_LED_G_PIN    EXP1_07_PIN
          #endif
          #ifndef RGB_LED_B_PIN
            #define RGB_LED_B_PIN    EXP1_08_PIN
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          #define NEOPIXEL_PIN       EXP1_06_PIN
        #endif

      #else // !FYSETC_MINI_12864

        #if ENABLED(MKS_MINI_12864)
          #define DOGLCD_CS          EXP1_06_PIN
          #define DOGLCD_A0          EXP1_07_PIN
        #endif

        #if IS_ULTIPANEL
          #define LCD_PINS_D5        EXP1_06_PIN
          #define LCD_PINS_D6        EXP1_07_PIN
          #define LCD_PINS_D7        EXP1_08_PIN

          #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
            #define BTN_ENC_EN       LCD_PINS_D7  // Detect the presence of the encoder
          #endif

        #endif

      #endif // !FYSETC_MINI_12864

    #endif // !MKS_12864OLED_SSD1306

  #endif // !CR10_STOCKDISPLAY

#endif // HAS_WIRED_LCD

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD) || SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #if SD_CONNECTION_IS(ONBOARD)
    #define ONBOARD_SD_CS_PIN              P0_06  // Chip select for "System" SD card
    #define SD_SS_PIN          ONBOARD_SD_CS_PIN
  #else
    #define SD_SS_PIN                EXP2_04_PIN
  #endif
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

//
// Other Pins
//
//#define PIN_P0_02                        P0_02  // AUX1 (Interrupt Capable/ADC/Serial Port 0)
//#define PIN_P0_03                        P0_03  // AUX1 (Interrupt Capable/ADC/Serial Port 0)
//#define PS_ON_PIN                        P1_23  // SERVO0 P1.23
