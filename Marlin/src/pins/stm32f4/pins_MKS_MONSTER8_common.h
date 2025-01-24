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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 3 || E_STEPPERS > 5
  #error "MKS Monster supports up to 3 hotends and 5 E steppers."
#elif HAS_FSMC_TFT
  #error "MKS Monster doesn't support FSMC-based TFT displays."
#endif

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support
#define USES_DIAG_JUMPERS

//#define DISABLE_DEBUG

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 10

// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION                   // Use BackSRAM-based EEPROM emulation
//#define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#define I2C_EEPROM                                // Need use jumpers set i2c for EEPROM
#define MARLIN_EEPROM_SIZE               0x1000U  // 4K
#define I2C_SCL_PIN                         PB8   // I2C_SCL and CAN_RX
#define I2C_SDA_PIN                         PB9   // I2C_SDA and CAN_TX

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches for diag signals
//
#define X_DIAG_PIN                          PA14  // Driver0 diag signal is connected to X-
#define Y_DIAG_PIN                          PA15  // Driver1 diag signal is connected to Y-
#define Z_DIAG_PIN                          PB13  // Driver2 diag signal is connected to Z-
#define E0_DIAG_PIN                         PA13  // Driver3 diag signal is connected to X+
#define E1_DIAG_PIN                         PC5   // Driver4 diag signal is connected to Y+
#define E2_DIAG_PIN                         PB12  // Driver5 diag signal is connected to Z+
#define E3_DIAG_PIN                         -1    // Driver6 diag signal is not connected
#define E4_DIAG_PIN                         -1    // Driver7 diag signal is not connected

// Limit Switches for endstops
#define Z_MIN_PIN                           PB13
#define Z_MAX_PIN                           PB12

//
// Probe enable
//
#if ENABLED(PROBE_ENABLE_DISABLE) && !defined(PROBE_ENABLE_PIN)
  #define PROBE_ENABLE_PIN            SERVO0_PIN
#endif

//
// Filament Runout Sensor
//
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PC5   // Y+
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                   PB12  // Z+
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC15  // Driver0
#define X_STEP_PIN                          PC14
#define X_DIR_PIN                           PC13
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE6
#endif

#define Y_ENABLE_PIN                        PC15  // Driver1
#define Y_STEP_PIN                          PE5
#define Y_DIR_PIN                           PE4
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE3
#endif

#define Z_ENABLE_PIN                        PE2   // Driver2
#define Z_STEP_PIN                          PE1
#define Z_DIR_PIN                           PE0
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB7
#endif

#define E0_ENABLE_PIN                       PB6   // Driver3
#define E0_STEP_PIN                         PB5
#define E0_DIR_PIN                          PB4
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB3
#endif

#define E1_ENABLE_PIN                       PD7   // Driver4
#define E1_STEP_PIN                         PD6
#define E1_DIR_PIN                          PD5
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD4
#endif

#define E2_ENABLE_PIN                       PD3   // Driver5
#define E2_STEP_PIN                         PD2
#define E2_DIR_PIN                          PD1
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD0
#endif

#define E3_ENABLE_PIN                       PC8   // Driver6
#define E3_STEP_PIN                         PC7
#define E3_DIR_PIN                          PC6
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD15
#endif

#define E4_STEP_PIN                         PD13  // Driver7
#define E4_DIR_PIN                          PD12
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PD11
#endif

//
// Default pins for TMC software SPI
// This board only supports SW SPI for stepper drivers
//
#if HAS_TMC_SPI
  #define TMC_USE_SW_SPI
#endif
#if !defined(TMC_SPI_MOSI) || TMC_SPI_MOSI == -1
  #undef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PE14
#endif
#if !defined(TMC_SPI_MISO) || TMC_SPI_MISO == -1
  #undef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PE13
#endif
#if !defined(TMC_SPI_SCK) || TMC_SPI_SCK == -1
  #undef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PE12
#endif

#if HAS_TMC_UART
  //
  // Software serial
  // No Hardware serial for steppers
  //
  #define X_SERIAL_TX_PIN                   PE6
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PB7
  #define E0_SERIAL_TX_PIN                  PB3
  #define E1_SERIAL_TX_PIN                  PD4
  #define E2_SERIAL_TX_PIN                  PD0
  #define E3_SERIAL_TX_PIN                  PD15
  #define E4_SERIAL_TX_PIN                  PD11

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH0
#define TEMP_1_PIN                          PC2   // TH1
#define TEMP_2_PIN                          PC3   // TH2
#define TEMP_BED_PIN                        PC0   // TB

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB1   // HE0
#define HEATER_1_PIN                        PB0   // HE1
#define HEATER_2_PIN                        PA3   // HE2
#define HEATER_BED_PIN                      PB10  // H-BED

#define FAN0_PIN                            PA2   // FAN0
#define FAN1_PIN                            PA1   // FAN1
#define FAN2_PIN                            PA0   // FAN2

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #define PS_ON_PIN                       PW_OFF
  #define KILL_PIN                        PW_DET
  #define KILL_PIN_STATE                    HIGH
#endif

/**
 *                  ------                                      ------
 *   (BEEPER) PB2  | 1  2 | PE10 (BTN_ENC)    (SPI1 MISO) PA6  | 1  2 | PA5 (SPI1 SCK)
 *   (LCD_EN) PE11 | 3  4 | PD10 (LCD_RS)       (BTN_EN1) PE9  | 3  4 | PA4 (SPI1 CS)
 *   (LCD_D4) PD9    5  6 | PD8  (LCD_D5)       (BTN_EN2) PE8    5  6 | PA7 (SPI1 MOSI)
 *   (LCD_D6) PE15 | 7  8 | PE7  (LCD_D7)       (SPI1_RS) PB11 | 7  8 | RESET
 *             GND | 9 10 | 5V                             GND | 9 10 | 3.3V
 *                  ------                                      ------
 *                   EXP1                                        EXP2
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
#define EXP2_07_PIN                         PB11
#define EXP2_08_PIN                         -1    // RESET

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
// Onboard SD card
// Must use soft SPI because Marlin's default hardware SPI is tied to LCD's EXP2
//
#if SD_CONNECTION_IS(ONBOARD)
  #define ENABLE_SPI3
  #define SD_SS_PIN                         PC9
  #define SD_SCK_PIN                        PC10
  #define SD_MISO_PIN                       PC11
  #define SD_MOSI_PIN                       PC12
  #define SD_DETECT_PIN                     PC4   // SD_DETECT_PIN doesn't work with NO_SD_HOST_DRIVE disabled
#elif SD_CONNECTION_IS(LCD)
  #define ENABLE_SPI1
  #define SD_SS_PIN                  EXP2_04_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_DETECT_PIN              EXP2_07_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "CUSTOM_CABLE is not a supported SDCARD_CONNECTION for BOARD_MKS_MONSTER8_V1/V2."
#endif

#if HAS_WIRED_LCD

  #define BEEPER_PIN                 EXP1_01_PIN
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #else

    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    // MKS MINI12864 and MKS LCD12864B; If using MKS LCD12864A (Need to remove RPK2 resistor)
    #if ENABLED(MKS_MINI_12864)

      #define ENABLE_SPI1
      #define FORCE_SOFT_SPI
      #define DOGLCD_A0              EXP1_07_PIN
      #define DOGLCD_CS              EXP1_06_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN
      //#define LCD_BACKLIGHT_PIN           -1
      //#define LCD_RESET_PIN               -1

    #elif ENABLED(FYSETC_MINI_12864_2_1)

      #define LCD_PINS_DC            EXP1_04_PIN
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_A0              LCD_PINS_DC
      #define LCD_BACKLIGHT_PIN             -1
      #define LCD_RESET_PIN          EXP1_05_PIN
      #define NEOPIXEL_PIN           EXP1_06_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #if SD_CONNECTION_IS(ONBOARD)
        #define FORCE_SOFT_SPI
      #endif
      //#define LCD_SCREEN_ROTATE            180  // 0, 90, 180, 270

    #else

      #define LCD_PINS_D4            EXP1_05_PIN
      #if IS_ULTIPANEL
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
        #define LCD_PINS_D7          EXP1_08_PIN
      #endif

      #ifndef BOARD_ST7920_DELAY_1
        #define BOARD_ST7920_DELAY_1            96
      #endif
      #ifndef BOARD_ST7920_DELAY_2
        #define BOARD_ST7920_DELAY_2            48
      #endif
      #ifndef BOARD_ST7920_DELAY_3
        #define BOARD_ST7920_DELAY_3           600
      #endif

    #endif
  #endif
#endif // HAS_WIRED_LCD

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

#endif
