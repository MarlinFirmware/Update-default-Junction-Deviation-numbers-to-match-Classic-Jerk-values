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

/**
 * ORCA 3D Modular Controller (STM32F103VCT6) board pin assignments
 */

#if NOT_TARGET(__STM32F1__, STM32F1)
  #error "Oops! Select an STM32F1 board in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "ORCA 3D Modular Controller"

// Avoid conflict with TIMER_SERVO when using the STM32 HAL
#define TEMP_TIMER  5

//
// Release PB4 (Y_ENABLE_PIN) from JTAG NRST role
//
#define DISABLE_DEBUG

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches
//
#define X_STOP_PIN                          PA15
#define Y_STOP_PIN                          PA14
#define Z_STOP_PIN                          PA12

//
// Steppers
//
#define X_ENABLE_PIN                        PC15
#define X_STEP_PIN                          PE5
#define X_DIR_PIN                           PE6
#ifndef X_CS_PIN
  #define X_CS_PIN                          PE3
#endif

#define Y_ENABLE_PIN                        PC15
#define Y_STEP_PIN                          PC14
#define Y_DIR_PIN                           PC13
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PE4
#endif

#define Z_ENABLE_PIN                        PB4
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB6
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PB3
#endif

#define E0_ENABLE_PIN                       PE2
#define E0_STEP_PIN                         PE1
#define E0_DIR_PIN                          PE0
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PB7
#endif

#define E1_ENABLE_PIN                       PB11
#define E1_STEP_PIN                         PC7
#define E1_DIR_PIN                          PC6
#ifndef E1_CS_PIN
  #define E1_CS_PIN                         PD15
#endif

#define E2_ENABLE_PIN                       PD7
#define E2_STEP_PIN                         PD6
#define E2_DIR_PIN                          PD5
#ifndef E2_CS_PIN
  #define E2_CS_PIN                         PD4
#endif

#define E3_ENABLE_PIN                       PD14
#define E3_STEP_PIN                         PD13
#define E3_DIR_PIN                          PC5
#ifndef E3_CS_PIN
  #define E3_CS_PIN                         PD11
#endif

#define E4_ENABLE_PIN                       PD3
#define E4_STEP_PIN                         PE14
#define E4_DIR_PIN                          PD9
#ifndef E4_CS_PIN
  #define E4_CS_PIN                         PD8
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB15
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB14
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB13
#endif

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   *
   * Hardware serial communication ports.
   * If undefined software serial is used according to the pins below
   */
  //#define X_HARDWARE_SERIAL  MSerial1
  //#define Y_HARDWARE_SERIAL  MSerial1
  //#define Z_HARDWARE_SERIAL  MSerial1
  //#define E0_HARDWARE_SERIAL MSerial1
  //#define E1_HARDWARE_SERIAL MSerial1

  #define X_SERIAL_TX_PIN                   PE3
  #define Y_SERIAL_TX_PIN                   PE4
  #define Z_SERIAL_TX_PIN                   PB3
  #define E0_SERIAL_TX_PIN                  PB7
  #define E1_SERIAL_TX_PIN                  PD15
  #define E2_SERIAL_TX_PIN                  PD11
  #define E3_SERIAL_TX_PIN                  PD8

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PC2   // TH2
#define TEMP_2_PIN                          PC3   // TH2
#define TEMP_3_PIN                          PC4   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB1   // HEATER1
#define HEATER_1_PIN                        PB2   // HEATER2
#define HEATER_2_PIN                        PB10  // HEATER2
#define HEATER_3_PIN                        PB12  // HEATER2
#define HEATER_BED_PIN                      PB0   // HOT BED

#define FAN0_PIN                            PA0   // FAN
#define FAN1_PIN                            PA1   // FAN
#define FAN2_PIN                            PA2   // FAN
#define FAN3_PIN                            PA3   // FAN

//
// LCD / Controller
//

/**
 * Note: ORCA 3D Modular Controller uses various TFT controllers.
 * - For TFT displays use UART0
 * - For 12864 displays use the expansion headers
 */

/**                ------                                     ------
 *             -- | 1  2 | PE10  (BTN_ENC)       (MISO) PA6  | 1  2 | PA5  (SCK)
 *  (LCD_EN) PE11 | 3  4 | PD10  (LCD_RS)     (BTN_EN1) PE8  | 3  4 | PA4  (SD_SS)
 *  (LCD_D4) PE12 | 5  6 | PE13  (LCD_D5)     (BTN_EN2) PE9  | 5  6 | PA7  (MOSI)
 *  (LCD_D6) PE15 | 7  8 | PE7   (LCD_D7)   (SD_DETECT) PD12 | 7  8 | RESET
 *            GND | 9 10 | 5V                           GND  | 9 10 | --
 *                 ------                                     ------
 *                  EXP1                                       EXP2
 */
#define EXP1_01_PIN                         -1    // No BEEPER
#define EXP1_02_PIN                         PE10  // ENC
#define EXP1_03_PIN                         PE11  // LCD_EN
#define EXP1_04_PIN                         PD10  // LCD_RS
#define EXP1_05_PIN                         PE12  // LCD_D4
#define EXP1_06_PIN                         PE13  // LCD_D5
#define EXP1_07_PIN                         PE15  // LCD_D6
#define EXP1_08_PIN                         PE7   // LCD_D7

#define EXP2_01_PIN                         PA6   // MISO
#define EXP2_02_PIN                         PA5   // SCK
#define EXP2_03_PIN                         PE8   // EN1
#define EXP2_04_PIN                         PA4   // SD_SS
#define EXP2_05_PIN                         PE9   // EN2
#define EXP2_06_PIN                         PA7   // MOSI
#define EXP2_07_PIN                         PD12  // SD_DET
#define EXP2_08_PIN                         -1    // RESET

//
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(LCD)
  #define SPI_DEVICE                           1
  #define SD_DETECT_PIN              EXP2_07_PIN
  #define SD_SCK_PIN                 EXP2_02_PIN
  #define SD_MISO_PIN                EXP2_01_PIN
  #define SD_MOSI_PIN                EXP2_06_PIN
  #define SD_SS_PIN                  EXP2_04_PIN
#elif SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PA3
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
  #define SD_SS_PIN                         PA4
#endif
#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                 EXP1_01_PIN  // NC
  #define BTN_ENC                    EXP1_02_PIN

  #if ENABLED(CR10_STOCKDISPLAY)
    #define LCD_PINS_RS              EXP1_07_PIN

    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif IS_TFTGLCD_PANEL

    #undef BEEPER_PIN
    #undef BTN_ENC

    #if ENABLED(TFTGLCD_PANEL_SPI)
      #define TFTGLCD_CS             EXP2_03_PIN
    #endif

    #define SD_DETECT_PIN            EXP2_07_PIN

  #else

    #define LCD_PINS_RS              EXP1_04_PIN

    #define BTN_EN1                  EXP2_03_PIN
    #define BTN_EN2                  EXP2_05_PIN

    #define LCD_PINS_EN              EXP1_03_PIN

    #if ENABLED(FYSETC_MINI_12864)

      #define LCD_BACKLIGHT_PIN             -1
      #define LCD_RESET_PIN          EXP1_05_PIN
      #define DOGLCD_A0              EXP1_04_PIN
      #define DOGLCD_CS              EXP1_03_PIN
      #define DOGLCD_SCK             EXP2_02_PIN
      #define DOGLCD_MOSI            EXP2_06_PIN

      #define FORCE_SOFT_SPI                      // SPI MODE3

      #define LED_PIN                EXP1_06_PIN  // red pwm
      //#define LED_PIN              EXP1_07_PIN  // green
      //#define LED_PIN              EXP1_08_PIN  // blue

      //#if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
      //  #ifndef RGB_LED_R_PIN
      //    #define RGB_LED_R_PIN    EXP1_06_PIN
      //  #endif
      //  #ifndef RGB_LED_G_PIN
      //    #define RGB_LED_G_PIN    EXP1_07_PIN
      //  #endif
      //  #ifndef RGB_LED_B_PIN
      //    #define RGB_LED_B_PIN    EXP1_08_PIN
      //  #endif
      //#elif ENABLED(FYSETC_MINI_12864_2_1)
      //  #define NEOPIXEL_PIN       EXP1_06_PIN
      //#endif

    #else // !FYSETC_MINI_12864

      #define LCD_PINS_D4            EXP1_05_PIN
      #if IS_ULTIPANEL
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
        #define LCD_PINS_D7          EXP1_08_PIN

        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         LCD_PINS_D7  // Detect the presence of the encoder
        #endif

      #endif

    #endif // !FYSETC_MINI_12864

  #endif

#endif // HAS_WIRED_LCD

// Alter timing for graphical display
#if IS_U8GLIB_ST7920
  #define BOARD_ST7920_DELAY_1               125
  #define BOARD_ST7920_DELAY_2               125
  #define BOARD_ST7920_DELAY_3               125
#endif
