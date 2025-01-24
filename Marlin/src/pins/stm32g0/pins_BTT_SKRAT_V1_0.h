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

/**
 * BigTreeTech SKRat 1.0 pin assignments
 * Schematic: https://github.com/bigtreetech/SKRat/blob/master/Hardware/BTT_SKRat_V1.0_SCH.pdf
 * Pins: https://github.com/bigtreetech/SKRat/blob/master/Hardware/BIGTREETECH%20SKRat%20V1.0-Pin.pdf
 */

#include "env_validate.h"

//#define BOARD_CUSTOM_BUILD_FLAGS -DTONE_CHANNEL=4 -DTONE_TIMER=4 -DTIMER_TONE=4

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "BTT SKRat V1.0"
#endif

#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#define LED_PIN                             PE2

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

//
// Servos
//
#define SERVO0_PIN                          PE6   // SERVOS

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PB5   // X-STOP
#define Y_DIAG_PIN                          PC1   // Y-STOP
#define Z_DIAG_PIN                          PC0   // Z-STOP
#define E0_DIAG_PIN                         PF4   // E0_DET
#define E1_DIAG_PIN                         PF5   // E1_DET

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                       PF4   // E0_DET
  #else
    #define X_MIN_PIN                       PF4   // E0_DET
  #endif
#elif ENABLED(X_DUAL_ENDSTOPS)
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                       PB5   // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                       PF4   // E0_DET
  #endif
#else
  #define X_STOP_PIN                        PB5   // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                       PF5   // E1_DET
  #else
    #define Y_MIN_PIN                       PF5   // E1_DET
  #endif
#elif ENABLED(Y_DUAL_ENDSTOPS)
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                       PC1   // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                       PF5   // E1_DET
  #endif
#else
  #define Y_STOP_PIN                        PC1   // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                       PE12  // PWR_DET
  #else
    #define Z_MIN_PIN                       PE12  // PWR_DET
  #endif
#elif ENABLED(Z_MULTI_ENDSTOPS)
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                       PC0   // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                       PE12  // PWR_DET
  #endif
#else
  #ifndef Z_STOP_PIN
    #define Z_STOP_PIN                      PC0   // Z-STOP
  #endif
#endif

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                   PE5   // Probe
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PF4   // E0_DET
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                   PF5   // E1_DET
#endif

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PF6   // PS_ON
#endif

//
// Power-loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PE12  // PWR_DET
#endif

//
// Steppers
//
#define X_STEP_PIN                          PF9   // XM
#define X_DIR_PIN                           PD7
#define X_ENABLE_PIN                        PD6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PF10
#endif

#define Y_STEP_PIN                          PD3   // YM
#define Y_DIR_PIN                           PD2
#define Y_ENABLE_PIN                        PD5
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD4
#endif

#define Z_STEP_PIN                          PA15  // ZMA / ZMB
#define Z_DIR_PIN                           PF8
#define Z_ENABLE_PIN                        PC9
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PC8
#endif

#define E0_STEP_PIN                         PC7   // E0M
#define E0_DIR_PIN                          PC6
#define E0_ENABLE_PIN                       PD9
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD8
#endif

#define E1_STEP_PIN                         PB10  // E1M
#define E1_DIR_PIN                          PE15
#define E1_ENABLE_PIN                       PA8
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PB11
#endif

//
// Temperature Sensors
//
#if TEMP_SENSOR_0_IS_MAX31865
  #define TEMP_0_CS_PIN                     PA4   // Max31865 CS
  #define TEMP_0_SCK_PIN                    PA5
  #define TEMP_0_MISO_PIN                   PA6
  #define TEMP_0_MOSI_PIN                   PA7
  #define SOFTWARE_SPI                            // Max31865 and LCD SD share a set of SPIs, Set SD to softwareSPI for Max31865
  #define FORCE_SOFT_SPI
#else
  #ifndef TEMP_0_PIN
    #define TEMP_0_PIN                      PA3   // TH0
  #endif
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN                        PA2   // TH1
#endif
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                      PB2   // TB
#endif
#ifndef TEMP_SENSOR_PROBE
  #define TEMP_PROBE_PIN                    PA1   // TH2
#endif
#ifndef TEMP_SENSOR_CHAMBER
  #define TEMP_CHAMBER_PIN                  PA0   // TH3
#endif

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PE11  // HE0
#endif
#ifndef HEATER_1_PIN
  #define HEATER_1_PIN                      PE13  // HE1
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PB3   // HB
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                          PD15  // FAN0
  //#define FAN0_PIN                        PE9   // "Four-Wire FAN0"
#endif

#if HAS_CUTTER
  #ifndef SPINDLE_LASER_PWM_PIN
    #define SPINDLE_LASER_PWM_PIN           PD13  // FAN2
  #endif
  #ifndef SPINDLE_LASER_ENA_PIN
    #define SPINDLE_LASER_ENA_PIN           PD14  // FAN1
  #endif
#else
  #ifndef FAN1_PIN
    #define FAN1_PIN                        PD14  // FAN1
    //#define FAN1_PIN                      PE14  // "Four-Wire FAN1"
  #endif
  #ifndef FAN2_PIN
    #define FAN2_PIN                        PD13  // FAN2
  #endif
  #ifndef FAN3_PIN
    #define FAN3_PIN                        PD12  // FAN3
  #endif

  #ifndef E0_FAN_TACHO_PIN
    //#define E0_FAN_TACHO_PIN              PD10  // "Four-Wire FAN0"
  #endif
  #ifndef E1_FAN_TACHO_PIN
    //#define E1_FAN_TACHO_PIN              PA11  // "Four-Wire FAN1"
  #endif

#endif // SPINDLE_FEATURE || LASER_FEATURE

//
// Default pins for TMC software SPI
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
  #ifndef TMC_SPI_MOSI
    #define TMC_SPI_MOSI                    PC12
  #endif
  #ifndef TMC_SPI_MISO
    #define TMC_SPI_MISO                    PC11
  #endif
  #ifndef TMC_SPI_SCK
    #define TMC_SPI_SCK                     PC10
  #endif
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial1

  //
  // Software serial
  //
  //#define X_SERIAL_TX_PIN                 PF10
  //#define Y_SERIAL_TX_PIN                 PD4
  //#define Z_SERIAL_TX_PIN                 PC8
  //#define E0_SERIAL_TX_PIN                PD8
  //#define E1_SERIAL_TX_PIN                PB11

  // Reduce baud rate to improve software serial reliability
  //#ifndef TMC_BAUD_RATE
  //  #define TMC_BAUD_RATE                19200
  //#endif

  #error "UART-based drivers are not supported on this board."

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

/**
 *                ------                                   ------
 * (BEEPER) PC13 | 1  2 | PF3  (BTN_ENC)  (MISO)      PA6 | 1  2 | PA5 (SCK)
 * (LCD_EN) PC3  | 3  4 | PC2  (LCD_RS)   (BTN_EN1)   PE7 | 3  4 | PF7 (SD_SS)
 * (LCD_D4) PB1  | 5  6   PB0  (LCD_D5)   (BTN_EN2)   PE8 | 5  6   PA7 (MOSI)
 * (LCD_D6) PC5  | 7  8 | PC4  (LCD_D7)  (SD_DETECT) PE10 | 7  8 | RESET
 *           GND | 9 10 | 5V                          GND | 9 10 | --
 *                ------                                   ------
 *                 EXP1                                     EXP2
 */
#define EXP1_01_PIN                         PC13
#define EXP1_02_PIN                         PF3
#define EXP1_03_PIN                         PC3
#define EXP1_04_PIN                         PC2
#define EXP1_05_PIN                         PB1
#define EXP1_06_PIN                         PB0
#define EXP1_07_PIN                         PC5
#define EXP1_08_PIN                         PC4

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PE7
#define EXP2_04_PIN                         PF7
#define EXP2_05_PIN                         PE8
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PE10
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PE3
#elif SD_CONNECTION_IS(LCD) && (ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050) || IS_TFTGLCD_PANEL)
  #define SD_DETECT_PIN              EXP1_01_PIN
  #define SD_SS_PIN                  EXP1_05_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1 -> used only by HAL/STM32F1...
#define ONBOARD_SD_CS_PIN                   PB8   // Chip select for "System" SD card

#define ENABLE_SPI1
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7

#if ENABLED(BTT_MOTOR_EXPANSION)
  /**        ------                  ------
   * M3DIAG | 1  2 | M3RX     M3STP | 1  2 | M3DIR
   * M2DIAG | 3  4 | M2RX     M2STP | 3  4 | M2DIR
   * M1DIAG   5  6 | M1RX     M1DIR   5  6 | M1STP
   * M3EN   | 7  8 | M2EN     M1EN  | 7  8 |    --
   * GND    | 9 10 |   --     GND   | 9 10 |    --
   *         ------                  ------
   *          EXP1                    EXP2
   *
   * NB In EXP_MOT_USE_EXP2_ONLY mode EXP1 is not used and M2EN and M3EN need to be jumpered to M1EN
   */

  // M1 on Driver Expansion Module
  #define E2_STEP_PIN                EXP2_06_PIN
  #define E2_DIR_PIN                 EXP2_05_PIN
  #define E2_ENABLE_PIN              EXP2_07_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E2_DIAG_PIN              EXP1_05_PIN
    #define E2_CS_PIN                EXP1_06_PIN
    #if HAS_TMC_UART
      #define E2_SERIAL_TX_PIN       EXP1_06_PIN
    #endif
  #endif

  // M2 on Driver Expansion Module
  #define E3_STEP_PIN                EXP2_03_PIN
  #define E3_DIR_PIN                 EXP2_04_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E3_ENABLE_PIN            EXP1_08_PIN
    #define E3_DIAG_PIN              EXP1_03_PIN
    #define E3_CS_PIN                EXP1_04_PIN
    #if HAS_TMC_UART
      #define E3_SERIAL_TX_PIN       EXP1_04_PIN
    #endif
  #else
    #define E3_ENABLE_PIN            EXP2_07_PIN
  #endif

  // M3 on Driver Expansion Module
  #define E4_STEP_PIN                EXP2_01_PIN
  #define E4_DIR_PIN                 EXP2_02_PIN
  #if !EXP_MOT_USE_EXP2_ONLY
    #define E4_ENABLE_PIN            EXP1_07_PIN
    #define E4_DIAG_PIN              EXP1_01_PIN
    #define E4_CS_PIN                EXP1_02_PIN
    #if HAS_TMC_UART
      #define E4_SERIAL_TX_PIN       EXP1_02_PIN
    #endif
  #else
    #define E4_ENABLE_PIN            EXP2_07_PIN
  #endif

#endif // BTT_MOTOR_EXPANSION

//
// LCDs and Controllers
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

  #elif ENABLED(MKS_MINI_12864)

    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_06_PIN
    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

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

#if HAS_SPI_TFT
  //
  // e.g., BTT_TFT35_SPI_V1_0 (480x320, 3.5", SPI Stock Display with Rotary Encoder in BIQU B1 SE)
  //
  #define TFT_CS_PIN                 EXP2_04_PIN
  #define TFT_A0_PIN                 EXP2_07_PIN
  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define TOUCH_INT_PIN              EXP1_07_PIN
  #define TOUCH_MISO_PIN             EXP1_06_PIN
  #define TOUCH_MOSI_PIN             EXP1_03_PIN
  #define TOUCH_SCK_PIN              EXP1_05_PIN
  #define TOUCH_CS_PIN               EXP1_04_PIN

  #define BTN_EN1                    EXP2_03_PIN
  #define BTN_EN2                    EXP2_05_PIN
  #define BTN_ENC                    EXP1_02_PIN
#endif

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                 PE4  // RGB
#endif
