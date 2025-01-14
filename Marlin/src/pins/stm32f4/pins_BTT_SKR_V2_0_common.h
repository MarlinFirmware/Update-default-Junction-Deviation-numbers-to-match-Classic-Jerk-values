/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define USES_DIAG_JUMPERS

// If you have the BigTreeTech driver expansion module, enable BTT_MOTOR_EXPANSION
// https://github.com/bigtreetech/BTT-Expansion-module/tree/master/BTT%20EXP-MOT
//#define BTT_MOTOR_EXPANSION

#if ALL(HAS_WIRED_LCD, BTT_MOTOR_EXPANSION)
  #if ANY(CR10_STOCKDISPLAY, ENDER2_STOCKDISPLAY)
    #define EXP_MOT_USE_EXP2_ONLY 1
  #else
    #error "You can't use both an LCD and a Motor Expansion Module on EXP1/EXP2 at the same time."
  #endif
#endif

// Use one of these or SDCard-based Emulation will be used
#if NO_EEPROM_SELECTED
  //#define SRAM_EEPROM_EMULATION                 // Use BackSRAM-based EEPROM emulation
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  #define FLASH_EEPROM_LEVELING
#endif

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 10

//
// Servos
//
#define SERVO0_PIN                          PE5

//
// Trinamic Stallguard pins
//
#define X_DIAG_PIN                          PC1   // X-STOP
#define Y_DIAG_PIN                          PC3   // Y-STOP
#define Z_DIAG_PIN                          PC0   // Z-STOP
#define E0_DIAG_PIN                         PC2   // E0DET
#define E1_DIAG_PIN                         PA0   // E1DET

//
// Limit Switches
//
#ifdef X_STALL_SENSITIVITY
  #define X_STOP_PIN                  X_DIAG_PIN
  #if X_HOME_TO_MIN
    #define X_MAX_PIN                       PC2   // E0DET
  #else
    #define X_MIN_PIN                       PC2   // E0DET
  #endif
#elif ENABLED(X_DUAL_ENDSTOPS)
  #ifndef X_MIN_PIN
    #define X_MIN_PIN                       PC1   // X-STOP
  #endif
  #ifndef X_MAX_PIN
    #define X_MAX_PIN                       PC2   // E0DET
  #endif
#else
  #define X_STOP_PIN                        PC1   // X-STOP
#endif

#ifdef Y_STALL_SENSITIVITY
  #define Y_STOP_PIN                  Y_DIAG_PIN
  #if Y_HOME_TO_MIN
    #define Y_MAX_PIN                       PA0   // E1DET
  #else
    #define Y_MIN_PIN                       PA0   // E1DET
  #endif
#elif ENABLED(Y_DUAL_ENDSTOPS)
  #ifndef Y_MIN_PIN
    #define Y_MIN_PIN                       PC3   // Y-STOP
  #endif
  #ifndef Y_MAX_PIN
    #define Y_MAX_PIN                       PA0   // E1DET
  #endif
#else
  #define Y_STOP_PIN                        PC3   // Y-STOP
#endif

#ifdef Z_STALL_SENSITIVITY
  #define Z_STOP_PIN                  Z_DIAG_PIN
  #if Z_HOME_TO_MIN
    #define Z_MAX_PIN                       PC15  // PWRDET
  #else
    #define Z_MIN_PIN                       PC15  // PWRDET
  #endif
#elif ENABLED(Z_MULTI_ENDSTOPS)
  #ifndef Z_MIN_PIN
    #define Z_MIN_PIN                       PC0   // Z-STOP
  #endif
  #ifndef Z_MAX_PIN
    #define Z_MAX_PIN                       PC15  // PWRDET
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
  #define Z_MIN_PROBE_PIN                   PE4
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
#define FIL_RUNOUT_PIN                      PC2   // E0DET
#define FIL_RUNOUT2_PIN                     PA0   // E1DET

//
// Power Supply Control
//
#ifndef PS_ON_PIN
  #define PS_ON_PIN                         PE8   // PS-ON
#endif

//
// Power Loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PC15  // PWRDET
#endif

//
// Control pin of driver/heater/fan power supply
//
#define SAFE_POWER_PIN                      PC13

//
// Steppers
//
#define X_STEP_PIN                          PE2
#define X_DIR_PIN                           PE1
#define X_ENABLE_PIN                        PE3
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE0
#endif

#define Y_STEP_PIN                          PD5
#define Y_DIR_PIN                           PD4
#define Y_ENABLE_PIN                        PD6
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PD3
#endif

#define Z_STEP_PIN                          PA15
#define Z_DIR_PIN                           PA8
#define Z_ENABLE_PIN                        PD1
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PD0
#endif

#ifndef E0_STEP_PIN
  #define E0_STEP_PIN                       PD15
#endif
#ifndef E0_DIR_PIN
  #define E0_DIR_PIN                        PD14
#endif
#ifndef E0_ENABLE_PIN
  #define E0_ENABLE_PIN                     PC7
#endif
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PC6
#endif

#ifndef E1_STEP_PIN
  #define E1_STEP_PIN                       PD11
#endif
#ifndef E1_DIR_PIN
  #define E1_DIR_PIN                        PD10
#endif
#ifndef E1_ENABLE_PIN
  #define E1_ENABLE_PIN                     PD13
#endif
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD12
#endif

//
// Temperature Sensors
//
#ifndef TEMP_0_PIN
  #define TEMP_0_PIN                        PA2   // TH0
#endif
#ifndef TEMP_1_PIN
  #define TEMP_1_PIN                        PA3   // TH1
#endif
#ifndef TEMP_BED_PIN
  #define TEMP_BED_PIN                      PA1   // TB
#endif

#if HOTENDS == 1 && DISABLED(HEATERS_PARALLEL)
  #if TEMP_SENSOR_PROBE
    #define TEMP_PROBE_PIN            TEMP_1_PIN
  #elif TEMP_SENSOR_CHAMBER
    #define TEMP_CHAMBER_PIN          TEMP_1_PIN
  #endif
#endif

//
// Heaters / Fans
//
#ifndef HEATER_0_PIN
  #define HEATER_0_PIN                      PB3   // Heater0
#endif
#ifndef HEATER_1_PIN
  #define HEATER_1_PIN                      PB4   // Heater1
#endif
#ifndef HEATER_BED_PIN
  #define HEATER_BED_PIN                    PD7   // Hotbed
#endif
#ifndef FAN0_PIN
  #define FAN0_PIN                          PB7   // Fan0
#endif

#if HAS_CUTTER
  #ifndef SPINDLE_LASER_PWM_PIN
    #define SPINDLE_LASER_PWM_PIN           PB5
  #endif
  #ifndef SPINDLE_LASER_ENA_PIN
    #define SPINDLE_LASER_ENA_PIN           PB6
  #endif
#else
  #ifndef FAN1_PIN
    #define FAN1_PIN                        PB6   // Fan1
  #endif
  #ifndef FAN2_PIN
    #define FAN2_PIN                        PB5   // Fan2
  #endif
#endif // SPINDLE_FEATURE || LASER_FEATURE

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PE14
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PA14
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PE15
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

  //
  // Software serial
  //
  #define X_SERIAL_TX_PIN                   PE0
  #define Y_SERIAL_TX_PIN                   PD3
  #define Z_SERIAL_TX_PIN                   PD0
  #define E0_SERIAL_TX_PIN                  PC6
  #define E1_SERIAL_TX_PIN                  PD12

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// SD Connection
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

/**
 *                ------                                   ------
 * (BEEPER) PC5  | 1  2 | PB0  (BTN_ENC)  (MISO)      PA6 | 1  2 | PA5 (SCK)
 * (LCD_EN) PB1  | 3  4 | PE9  (LCD_RS)   (BTN_EN1)   PE7 | 3  4 | PA4 (SD_SS)
 * (LCD_D4) PE10 | 5  6   PE11 (LCD_D5)   (BTN_EN2)   PB2 | 5  6   PA7 (MOSI)
 * (LCD_D6) PE12 | 7  8 | PE13 (LCD_D7)   (SD_DETECT) PC4 | 7  8 | RESET
 *           GND | 9 10 | 5V                          GND | 9 10 | --
 *                ------                                   ------
 *                 EXP1                                     EXP2
 */
#define EXP1_01_PIN                         PC5
#define EXP1_02_PIN                         PB0
#define EXP1_03_PIN                         PB1
#define EXP1_04_PIN                         PE9
#define EXP1_05_PIN                         PE10
#define EXP1_06_PIN                         PE11
#define EXP1_07_PIN                         PE12
#define EXP1_08_PIN                         PE13

#define EXP2_01_PIN                         PA6
#define EXP2_02_PIN                         PA5
#define EXP2_03_PIN                         PE7
#define EXP2_04_PIN                         PA4
#define EXP2_05_PIN                         PB2
#define EXP2_06_PIN                         PA7
#define EXP2_07_PIN                         PC4
#define EXP2_08_PIN                         -1

//
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(LCD)
  #define SDSS                       EXP2_04_PIN
  #define SD_SS_PIN                         SDSS
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #define ONBOARD_SDIO                            // Use SDIO for onboard SD
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "No custom SD drive cable defined for this board."
#endif

#if ENABLED(BTT_MOTOR_EXPANSION)
  /**       -----                        -----
   *    -- | . . | GND               -- | . . | GND
   *    -- | . . | M1EN            M2EN | . . | M3EN
   * M1STP | . .   M1DIR           M1RX | . .   M1DIAG
   * M2DIR | . . | M2STP           M2RX | . . | M2DIAG
   * M3DIR | . . | M3STP           M3RX | . . | M3DIAG
   *        -----                        -----
   *        EXP2                         EXP1
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

#if HAS_SPI_TFT

  #define TFT_SCK_PIN                EXP2_02_PIN
  #define TFT_MISO_PIN               EXP2_01_PIN
  #define TFT_MOSI_PIN               EXP2_06_PIN

  #define BTN_ENC                    EXP1_02_PIN
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

    #define LCD_READ_ID                     0xD3
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

#endif // HAS_SPI_TFT

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

//
// NeoPixel LED
//
#ifndef BOARD_NEOPIXEL_PIN
  #define BOARD_NEOPIXEL_PIN                PE6
#endif

#if ENABLED(WIFISUPPORT)
  //
  // WIFI
  //

  /**
   *                      -------
   *            GND | 9  |       | 8 | 3.3V
   *  (ESP-CS) PB12 | 10 |       | 7 | PB15 (ESP-MOSI)
   *           3.3V | 11 |       | 6 | PB14 (ESP-MISO)
   * (ESP-IO0) PB10 | 12 |       | 5 | PB13 (ESP-CLK)
   * (ESP-IO4) PB11 | 13 |       | 4 | --
   *             -- | 14 |       | 3 | 3.3V (ESP-EN)
   *  (ESP-RX)  PD8 | 15 |       | 2 | --
   *  (ESP-TX)  PD9 | 16 |       | 1 | PC14 (ESP-RST)
   *                      -------
   *                       WIFI
   */
  #define ESP_WIFI_MODULE_COM                  3  // Must also set either SERIAL_PORT or SERIAL_PORT_2 to this
  #define ESP_WIFI_MODULE_BAUDRATE      BAUDRATE  // Must use same BAUDRATE as SERIAL_PORT & SERIAL_PORT_2
  #define ESP_WIFI_MODULE_RESET_PIN         PC14
  #define ESP_WIFI_MODULE_GPIO0_PIN         PB10
  #define ESP_WIFI_MODULE_GPIO4_PIN         PB11
#endif
