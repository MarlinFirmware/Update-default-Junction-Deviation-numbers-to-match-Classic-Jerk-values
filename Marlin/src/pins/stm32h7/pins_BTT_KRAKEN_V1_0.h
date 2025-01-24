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

#define BOARD_INFO_NAME   "BTT Kraken V1.0"
#define BOARD_WEBSITE_URL "github.com/bigtreetech/BIGTREETECH-Kraken"

#define USES_DIAG_JUMPERS

//
// EEPROM
//
#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #undef NO_EEPROM_SELECTED
  #ifndef FLASH_EEPROM_EMULATION
    #define FLASH_EEPROM_EMULATION
  #endif
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 8

//
// Servos
//
#define SERVO0_PIN                          PE9   // PROBE
#define SERVO1_PIN                          PE7   // MOTOR

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC15  // MIN1
#define Y_DIAG_PIN                          PF0   // MIN2
#define Z_DIAG_PIN                          PF1   // MIN3
#define E0_DIAG_PIN                         PF2   // MIN4
#define E1_DIAG_PIN                         PF3   // MIN5
#define E2_DIAG_PIN                         PF4   // MIN6
#define E3_DIAG_PIN                         PF10  // MIN7
#define E4_DIAG_PIN                         PC0   // MIN8

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                E0_DIAG_PIN  // MIN4
  #else
    #define X_MIN_PIN                E0_DIAG_PIN  // MIN4
  #endif
#elif NEEDS_X_MINMAX
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                 X_DIAG_PIN  // MIN1
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                E0_DIAG_PIN  // MIN4
  #endif
#else
  #define X_STOP_PIN                  X_DIAG_PIN  // MIN1
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // MIN5
  #else
    #define Y_MIN_PIN                E1_DIAG_PIN  // MIN5
  #endif
#elif NEEDS_Y_MINMAX
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                 Y_DIAG_PIN  // MIN2
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                E1_DIAG_PIN  // MIN5
  #endif
#else
  #define Y_STOP_PIN                  Y_DIAG_PIN  // MIN2
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // MIN6
  #else
    #define Z_MIN_PIN                E2_DIAG_PIN  // MIN6
  #endif
#elif NEEDS_Z_MINMAX
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                 Z_DIAG_PIN  // MIN3
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                E2_DIAG_PIN  // MIN6
  #endif
#else
  #define Z_STOP_PIN                  Z_DIAG_PIN  // MIN3
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PG1   // PROBE (BLTouch, MicroProbe, etc.)
  //#define Z_MIN_PROBE_PIN                 PD11  // IND-DET (Inductive probe)
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
#define FIL_RUNOUT_PIN               E0_DIAG_PIN  // MIN4
#define FIL_RUNOUT2_PIN              E1_DIAG_PIN  // MIN5
#define FIL_RUNOUT3_PIN              E2_DIAG_PIN  // MIN6
#define FIL_RUNOUT4_PIN              E3_DIAG_PIN  // MIN7
#define FIL_RUNOUT5_PIN              E4_DIAG_PIN  // MIN8

//
// Steppers
//
#define X_STEP_PIN                          PC14  // S1 (Motor 1)
#define X_DIR_PIN                           PC13
#define X_ENABLE_PIN                        PE6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PD6
#endif

#define Y_STEP_PIN                          PE5   // S2 (Motor 2)
#define Y_DIR_PIN                           PE4
#define Y_ENABLE_PIN                        PE3
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD5
#endif

#define Z_STEP_PIN                          PE2   // S3 (Motor 3)
#define Z_DIR_PIN                           PE1
#define Z_ENABLE_PIN                        PE0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD4
#endif

#define E0_STEP_PIN                         PB9   // S4 (Motor 4)
#define E0_DIR_PIN                          PB8
#define E0_ENABLE_PIN                       PB7
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD3
#endif

#define E1_STEP_PIN                         PG9   // S5 (Motor 5)
#define E1_DIR_PIN                          PG10
#define E1_ENABLE_PIN                       PG13
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD2
#endif

#define E2_STEP_PIN                         PG11  // S6 (Motor 6)
#define E2_DIR_PIN                          PD7
#define E2_ENABLE_PIN                       PG12
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PA15
#endif

#define E3_STEP_PIN                         PB4   // S7 (Motor 7)
#define E3_DIR_PIN                          PB3
#define E3_ENABLE_PIN                       PB5
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PA9
#endif

#define E4_STEP_PIN                         PG15  // S8 (Motor 8)
#define E4_DIR_PIN                          PB6
#define E4_ENABLE_PIN                       PG14
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PA10
#endif

//
// Integrated TMC2160 driver defaults
//
#if  (HAS_X_AXIS && !AXIS_DRIVER_TYPE_X(TMC2160)) \
  || (HAS_Y_AXIS && !AXIS_DRIVER_TYPE_Y(TMC2160)) \
  || (NUM_Z_STEPPERS >= 1 && !AXIS_DRIVER_TYPE_Z(TMC2160)) \
  || (NUM_Z_STEPPERS >= 2 && !AXIS_DRIVER_TYPE_Z2(TMC2160)) \
  || (NUM_Z_STEPPERS >= 3 && !AXIS_DRIVER_TYPE_Z3(TMC2160)) \
  || (NUM_Z_STEPPERS >= 4 && !AXIS_DRIVER_TYPE_Z4(TMC2160)) \
  || (EXTRUDERS >= 1 && !AXIS_DRIVER_TYPE_E0(TMC2160)) \
  || (EXTRUDERS >= 2 && !AXIS_DRIVER_TYPE_E1(TMC2160)) \
  || (EXTRUDERS >= 3 && !AXIS_DRIVER_TYPE_E2(TMC2160)) \
  || (EXTRUDERS >= 4 && !AXIS_DRIVER_TYPE_E3(TMC2160)) \
  || (EXTRUDERS >= 5 && !AXIS_DRIVER_TYPE_E4(TMC2160))
  #error "All DRIVER TYPEs must be TMC2160 for BOARD_BTT_KRAKEN_V1_0."
#endif

// RSENSE defaults
#if HAS_X_AXIS
  static_assert(X_RSENSE == 0.022, "X_RSENSE (S1 / Motor 1) must be 0.022 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if HAS_Y_AXIS
  static_assert(Y_RSENSE == 0.022, "Y_RSENSE (S2 / Motor 2) must be 0.022 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if NUM_Z_STEPPERS >= 1
  static_assert(Z_RSENSE == 0.022, "Z_RSENSE (S3 / Motor 3) must be 0.022 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if NUM_Z_STEPPERS >= 2
  static_assert(Z2_RSENSE == 0.075, "Z2_RSENSE (S5 / Motor 5) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if NUM_Z_STEPPERS >= 3
  static_assert(Z3_RSENSE == 0.075, "Z3_RSENSE (S6 / Motor 6) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if NUM_Z_STEPPERS >= 4
  static_assert(Z4_RSENSE == 0.075, "Z4_RSENSE (S7 / Motor 7) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if EXTRUDERS >= 1
  static_assert(E0_RSENSE == 0.022, "E0_RSENSE (S4 / Motor 4) must be 0.022 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if EXTRUDERS >= 2
  static_assert(E1_RSENSE == 0.075, "E1_RSENSE (S5 / Motor 5) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if EXTRUDERS >= 3
  static_assert(E2_RSENSE == 0.075, "E2_RSENSE (S6 / Motor 6) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if EXTRUDERS >= 4
  static_assert(E3_RSENSE == 0.075, "E3_RSENSE (S7 / Motor 7) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif
#if EXTRUDERS >= 5
  static_assert(E4_RSENSE == 0.075, "E4_RSENSE (S8 / Motor 8) must be 0.075 for BOARD_BTT_KRAKEN_V1_0.");
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PB1   // TH0
#define TEMP_1_PIN                          PC5   // TH1
#define TEMP_2_PIN                          PC4   // TH2
#define TEMP_3_PIN                          PA7   // TH3
#define TEMP_BED_PIN                        PB0   // THB

#if TEMP_SENSOR_0_IS_MAX31865
  #define TEMP_0_CS_PIN                     PC9   // 31865 (1)
  #define TEMP_0_SCK_PIN                    PC10
  #define TEMP_0_MISO_PIN                   PC11
  #define TEMP_0_MOSI_PIN                   PC12
  #define SOFTWARE_SPI                            // MAX31865 and LCD SD share a set of SPIs, Set SD to software SPI
  #define FORCE_SOFT_SPI
#endif
#if TEMP_SENSOR_1_IS_MAX31865
  #define TEMP_1_CS_PIN                     PA8   // 31865 (2)
  #define TEMP_1_SCK_PIN                    PC10
  #define TEMP_1_MISO_PIN                   PC11
  #define TEMP_1_MOSI_PIN                   PC12
#endif

//
// Heaters / Fans
//
#define HEATER_BED_PIN                      PF5   // BED-OUT
#define HEATER_0_PIN                        PF6   // HE0
#define HEATER_1_PIN                        PF7   // HE1
#define HEATER_2_PIN                        PF9   // HE2
#define HEATER_3_PIN                        PF8   // HE3

#define FAN0_PIN                            PA0   // FAN0 (3 wire)
#define FAN1_PIN                            PA1   // FAN1 (3 wire)
#define FAN2_PIN                            PA2   // FAN2 (3 wire)
#define FAN3_PIN                            PA3   // FAN3 (3 wire)
#define FAN4_PIN                            PA4   // FAN4 (3 wire)
#define FAN5_PIN                            PA5   // FAN5 (3 wire)
#define FAN6_PIN                            PA6   // FAN6 (4 wire)
#define FAN7_PIN                            PE8   // FAN7 (4 wire)

#ifndef E0_FAN_TACHO_PIN
  //#define E0_FAN_TACHO_PIN                PC1   // FAN6
#endif
#ifndef E1_FAN_TACHO_PIN
  //#define E1_FAN_TACHO_PIN                PG0   // FAN7
#endif

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PD10  // PS-ON
#endif

//
// Misc. Functions
//
#define LED_PIN                             PA14

#ifndef FILWIDTH_PIN
  #define FILWIDTH_PIN                      PC2
#endif
#ifndef FILWIDTH2_PIN
  #define FILWIDTH2_PIN                     PC3
#endif

//
// SD Support
//
#ifndef SDCARD_CONNECTION
  #if HAS_WIRED_LCD && DISABLED(NO_LCD_SDCARD)
    #define SDCARD_CONNECTION                LCD
  #else
    #define SDCARD_CONNECTION            ONBOARD
  #endif
#endif

//
// Default pins for TMC software SPI
// This board only supports SW SPI for stepper drivers
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PC8
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PC7
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PC6
  #endif
#endif

/**               ------                                      ------
 * (BEEPER) PG5  | 1  2 | PG4  (BTN_ENC)         (MISO) PE13 | 1  2 | PE12 (SCK)
 * (LCD_EN) PG3  | 3  4 | PG2  (LCD_RS)       (BTN_EN1) PG8  | 3  4 | PE11 (SD_SS)
 * (LCD_D4) PD15   5  6 | PD14 (LCD_D5)       (BTN_EN2) PG7    5  6 | PE14 (MOSI)
 * (LCD_D6) PD13 | 7  8 | PD12 (LCD_D7)     (SD_DETECT) PG6  | 7  8 | RESET
 *           GND | 9 10 | 5V                             GND | 9 10 | --
 *                ------                                      ------
 *                 EXP1                                        EXP2
 */
#define EXP1_01_PIN                         PG5
#define EXP1_02_PIN                         PG4
#define EXP1_03_PIN                         PG3
#define EXP1_04_PIN                         PG2
#define EXP1_05_PIN                         PD15
#define EXP1_06_PIN                         PD14
#define EXP1_07_PIN                         PD13
#define EXP1_08_PIN                         PD12

#define EXP2_01_PIN                         PE13
#define EXP2_02_PIN                         PE12
#define EXP2_03_PIN                         PG8
#define EXP2_04_PIN                         PE11
#define EXP2_05_PIN                         PG7
#define EXP2_06_PIN                         PE14
#define EXP2_07_PIN                         PG6
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #ifndef SD_DETECT_STATE
    #define SD_DETECT_STATE HIGH
  #elif SD_DETECT_STATE == LOW
    #error "BOARD_BTT_KRAKEN_V1_0 onboard SD requires SD_DETECT_STATE set to HIGH."
  #endif
  #define SD_SS_PIN                         PB12
  #define SD_SCK_PIN                        PB13
  #define SD_MISO_PIN                       PB14
  #define SD_MOSI_PIN                       PB15
  #define SD_DETECT_PIN                     PE15
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(LCD)
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SOFTWARE_SPI
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for BOARD_BTT_KRAKEN_V1_0."
#endif

#if ENABLED(BTT_MOTOR_EXPANSION)
  /**
   *         ------                  ------
   * M3DIAG | 1  2 | M3RX     M3STP | 1  2 | M3DIR
   * M2DIAG | 3  4 | M2RX     M2STP | 3  4 | M2DIR
   * M1DIAG   5  6 | M1RX     M1DIR   5  6 | M1STP
   *   M3EN | 7  8 | M2EN      M1EN | 7  8 | --
   *    GND | 9 10 | --         GND | 9 10 | --
   *        ------                   ------
   *         EXP1                     EXP2
   */

  // M1 on Driver Expansion Module
  #define E4_STEP_PIN                EXP2_06_PIN
  #define E4_DIR_PIN                 EXP2_05_PIN
  #define E4_ENABLE_PIN              EXP2_07_PIN
  #define E4_DIAG_PIN                EXP1_05_PIN
  #define E4_CS_PIN                  EXP1_06_PIN
  #if HAS_TMC_UART
    #define E4_SERIAL_TX_PIN         EXP1_06_PIN
  #endif

  // M2 on Driver Expansion Module
  #define E5_STEP_PIN                EXP2_03_PIN
  #define E5_DIR_PIN                 EXP2_04_PIN
  #define E5_ENABLE_PIN              EXP1_08_PIN
  #define E5_DIAG_PIN                EXP1_03_PIN
  #define E5_CS_PIN                  EXP1_04_PIN
  #if HAS_TMC_UART
    #define E5_SERIAL_TX_PIN         EXP1_04_PIN
  #endif

  // M3 on Driver Expansion Module
  #define E6_STEP_PIN                EXP2_01_PIN
  #define E6_DIR_PIN                 EXP2_02_PIN
  #define E6_ENABLE_PIN              EXP1_07_PIN
  #define E6_DIAG_PIN                EXP1_01_PIN
  #define E6_CS_PIN                  EXP1_02_PIN
  #if HAS_TMC_UART
    #define E6_SERIAL_TX_PIN         EXP1_02_PIN
  #endif

#endif // BTT_MOTOR_EXPANSION

//
// LCD / Controller
//

#if IS_TFTGLCD_PANEL

  #if ENABLED(TFTGLCD_PANEL_SPI)
    #define TFTGLCD_CS               EXP2_03_PIN
  #endif

#elif HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  /**
   *        ------                 ------            ---
   *       | 1  2 |               | 1  2 |            1 |
   *       | 3  4 |            RX | 3  4 | TX       | 2 | RX
   *   ENT   5  6 | BEEP      ENT   5  6 | BEEP     | 3 | TX
   *     B | 7  8 | A           B | 7  8 | A        | 4 |
   *   GND | 9 10 | VCC       GND | 9 10 | VCC        5 |
   *        ------                 ------            ---
   *         EXP1                   DWIN             TFT
   *
   * DWIN pins are labeled as printed on DWIN PCB. GND, VCC, A, B, ENT & BEEP can be connected in the same
   * orientation as the existing plug/DWIN to EXP1. TX/RX need to be connected to the TFT port, with TX->RX, RX->TX.
   */

  CONTROLLER_WARNING("BTT_KRAKEN_V1_0", "Ender-3 V2 display")

  #define BEEPER_PIN                 EXP1_06_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN
  #define BTN_ENC                    EXP1_05_PIN

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

      #define FORCE_SOFT_SPI

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

#if HAS_SPI_TFT                                   // Config for Classic UI (emulated DOGM) and Color UI

  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN

  #ifndef TFT_WIDTH
    #define TFT_WIDTH                        480
  #endif
  #ifndef TFT_HEIGHT
    #define TFT_HEIGHT                       320
  #endif

  #if ENABLED(BTT_TFT35_SPI_V1_0)

    /**
     *            ------                       ------
     *    BEEPER | 1  2 | LCD-BTN        MISO | 1  2 | CLK
     *    T_MOSI | 3  4 | T_CS       LCD-ENCA | 3  4 | TFTCS
     *     T_CLK | 5  6   T_MISO     LCD-ENCB | 5  6   MOSI
     *    PENIRQ | 7  8 | F_CS             RS | 7  8 | RESET
     *       GND | 9 10 | VCC             GND | 9 10 | NC
     *            ------                       ------
     *             EXP1                         EXP2
     *
     * 480x320, 3.5", SPI Display with Rotary Encoder.
     * Stock Display for the BIQU B1 SE Series.
     * Schematic: https://github.com/bigtreetech/TFT35-SPI/blob/master/v1/Hardware/BTT%20TFT35-SPI%20V1-SCH.pdf
     */
    #define TFT_CS_PIN               EXP2_04_PIN
    #define TFT_DC_PIN               EXP2_07_PIN
    #define TFT_A0_PIN                TFT_DC_PIN

    #define TOUCH_CS_PIN             EXP1_04_PIN
    #define TOUCH_SCK_PIN            EXP1_05_PIN
    #define TOUCH_MISO_PIN           EXP1_06_PIN
    #define TOUCH_MOSI_PIN           EXP1_03_PIN
    #define TOUCH_INT_PIN            EXP1_07_PIN

    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X          17540
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y         -11388
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 -21
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                 337
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION TOUCH_LANDSCAPE
    #endif

  #elif ENABLED(MKS_TS35_V2_0)

    /**                      ------                                   ------
     *               BEEPER | 1  2 | BTN_ENC               SPI1_MISO | 1  2 | SPI1_SCK
     *     TFT_BKL / LCD_EN | 3  4 | TFT_RESET / LCD_RS      BTN_EN1 | 3  4 | SPI1_CS
     *    TOUCH_CS / LCD_D4 | 5  6   TOUCH_INT / LCD_D5      BTN_EN2 | 5  6   SPI1_MOSI
     *     SPI1_CS / LCD_D6 | 7  8 | SPI1_RS / LCD_D7       SPI1_RS  | 7  8 | RESET
     *                  GND | 9 10 | VCC                         GND | 9 10 | VCC
     *                       ------                                   ------
     *                        EXP1                                     EXP2
     */
    #define TFT_CS_PIN               EXP1_07_PIN  // SPI1_CS
    #define TFT_DC_PIN               EXP1_08_PIN  // SPI1_RS
    #define TFT_A0_PIN                TFT_DC_PIN

    #define TFT_RESET_PIN            EXP1_04_PIN

    #define LCD_BACKLIGHT_PIN        EXP1_03_PIN
    #define TFT_BACKLIGHT_PIN  LCD_BACKLIGHT_PIN

    #define TOUCH_BUTTONS_HW_SPI
    #define TOUCH_BUTTONS_HW_SPI_DEVICE        1

    #define TOUCH_CS_PIN             EXP1_05_PIN  // SPI1_NSS
    #define TOUCH_SCK_PIN            EXP2_02_PIN  // SPI1_SCK
    #define TOUCH_MISO_PIN           EXP2_01_PIN  // SPI1_MISO
    #define TOUCH_MOSI_PIN           EXP2_06_PIN  // SPI1_MOSI

    #define LCD_READ_ID                   0xD3
    #define LCD_USE_DMA_SPI

    #define TFT_BUFFER_WORDS               14400

    #ifndef TOUCH_CALIBRATION_X
      #define TOUCH_CALIBRATION_X         -17253
    #endif
    #ifndef TOUCH_CALIBRATION_Y
      #define TOUCH_CALIBRATION_Y          11579
    #endif
    #ifndef TOUCH_OFFSET_X
      #define TOUCH_OFFSET_X                 514
    #endif
    #ifndef TOUCH_OFFSET_Y
      #define TOUCH_OFFSET_Y                 -24
    #endif
    #ifndef TOUCH_ORIENTATION
      #define TOUCH_ORIENTATION TOUCH_LANDSCAPE
    #endif

  #endif

#endif

//
// NeoPixel LED
//
#if DISABLED(FYSETC_MINI_12864_2_1) && !defined(NEOPIXEL_PIN)
  #define NEOPIXEL_PIN                      PF12  // RGB1
#endif
#ifndef NEOPIXEL2_PIN
  #define NEOPIXEL2_PIN                     PF11  // RGB2
#endif
