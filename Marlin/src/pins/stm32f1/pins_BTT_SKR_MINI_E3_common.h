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

#include "env_validate.h"

// Release PB3/PB4 (E0 STP/DIR) from JTAG pins
#define DISABLE_JTAG

#define USES_DIAG_JUMPERS

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE                0x800U  // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

//
// Servos
//
#define SERVO0_PIN                          PA1   // SERVOS

//
// Limit Switches
//
#define X_STOP_PIN                          PC0   // X-STOP
#define Y_STOP_PIN                          PC1   // Y-STOP
#define Z_STOP_PIN                          PC2   // Z-STOP

//
// Z Probe must be this pin
//
#define Z_MIN_PROBE_PIN                     PC14  // PROBE

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
  #define FIL_RUNOUT_PIN                    PC15  // E0-STOP
#endif

//
// Power-loss Detection
//
#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PC12  // Power Loss Detection: PWR-DET
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PB14
#define X_STEP_PIN                          PB13
#define X_DIR_PIN                           PB12

#define Y_ENABLE_PIN                        PB11
#define Y_STEP_PIN                          PB10
#define Y_DIR_PIN                           PB2

#define Z_ENABLE_PIN                        PB1
#define Z_STEP_PIN                          PB0
#define Z_DIR_PIN                           PC5

#define E0_ENABLE_PIN                       PD2
#define E0_STEP_PIN                         PB3
#define E0_DIR_PIN                          PB4

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA0   // Analog Input "TH0"
#define TEMP_BED_PIN                        PC3   // Analog Input "TB0"

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC8   // "HE"
#define HEATER_BED_PIN                      PC9   // "HB"

#ifndef FAN0_PIN
  #define FAN0_PIN                          PA8   // "FAN0"
#endif

//
// USB connect control
//
#ifndef USB_CONNECT_PIN
  #define USB_CONNECT_PIN                   PC13
#endif

#define USB_CONNECT_INVERTING              false

/**
 *         SKR Mini E3 V1.0, V1.2                    SKR Mini E3 V2.0
 *                 ------                                  ------
 * (BEEPER)  PB5  | 1  2 | PB6 (BTN_ENC)   (BEEPER)  PB5  | 1  2 | PA15 (BTN_ENC)
 * (BTN_EN1) PA9  | 3  4 | RESET           (BTN_EN1) PA9  | 3  4 | RESET
 * (BTN_EN2) PA10   5  6 | PB9  (LCD_D4)   (BTN_EN2) PA10   5  6 | PB9  (LCD_D4)
 * (LCD_RS)  PB8  | 7  8 | PB7  (LCD_EN)   (LCD_RS)  PB8  | 7  8 | PB15 (LCD_EN)
 *            GND | 9 10 | 5V                         GND | 9 10 | 5V
 *                 ------                                  ------
 *                  EXP1                                    EXP1
 */
#ifndef EXP1_08_PIN
  #define EXP1_01_PIN                       PB5
  #define EXP1_02_PIN                       PB6
  #define EXP1_03_PIN                       PA9
  #define EXP1_04_PIN                       -1    // RESET
  #define EXP1_05_PIN                       PA10
  #define EXP1_06_PIN                       PB9
  #define EXP1_07_PIN                       PB8
  #define EXP1_08_PIN                       PB7
#endif

/*   -----
 *   | 1 | RST
 *   | 2 | PA3 RX2
 *   | 3 | PA2 TX2
 *   | 4 | GND
 *   | 5 | 5V
 *   -----
 *    TFT
 */

#define TFT_02                              PA3
#define TFT_03                              PA2

#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI

  CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "Ender-3 V2 display")

  /**
   *        ------                ------                ------
   * (ENT) | 1  2 | (BEEP)       |10  9 |              |10  9 |
   *  (RX) | 3  4 |         (RX) | 8  7 | (TX)      RX | 8  7 | TX
   *  (TX)   5  6 |        (ENT)   6  5 | (BEEP)   ENT | 6  5 | BEEP
   *   (B) | 7  8 | (A)      (B) | 4  3 | (A)        B | 4  3 | A
   *   GND | 9 10 | (VCC)    GND | 2  1 | VCC      GND | 2  1 | VCC
   *        ------                ------                ------
   *         EXP1                  DWIN               DWIN (plug)
   *
   * All pins are labeled as printed on DWIN PCB. Connect TX-TX, A-A and so on.
   */

  #define BEEPER_PIN                 EXP1_02_PIN
  #define BTN_EN1                    EXP1_08_PIN
  #define BTN_EN2                    EXP1_07_PIN
  #define BTN_ENC                    EXP1_01_PIN

#elif HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN               EXP1_01_PIN

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(LCD_FOR_MELZI)

    CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "LCD for Melzi v4", " Requires a custom cable.")

    /**
     * LCD for Melzi v4 needs a custom cable with reversed GND/5V pins; plugging in a standard cable may damage the board or LCD!
     *  1. Swap the LCD's +5V (Pin2) and GND (Pin1) wires. (This is the critical part!)
     *  2. Swap pin 4 on the Melzi LCD to pin 7 on the SKR Mini E3 EXP1 connector (pin 4 on the SKR is a RESET and cannot be used)
     *
     *       LCD for Melzi V4         SKR Mini E3 V2.0
     *            ------                   ------
     *    LCD RS | 1  2 | EN1      LCD RS | 1  2 | EN1
     *    LCD EN | 3  4 | EN2      LCD EN | 3  4 | OPEN (RESET)
     *    LCD D4 | 5  6 | ENC      LCD D4 | 5  6 | ENC
     *    E-Stop | 7  8 | BEEP        EN2 | 7  8 | BEEP
     *        5V | 9 10 | GND         GND | 9 10 | 5V
     *            ------                   ------
     *             EXP1                     EXP1
     */
    #define BEEPER_PIN               EXP1_08_PIN

    #define BTN_ENC                  EXP1_06_PIN
    #define BTN_EN1                  EXP1_02_PIN
    #define BTN_EN2                  EXP1_07_PIN

    #define LCD_PINS_RS              EXP1_01_PIN
    #define LCD_PINS_EN              EXP1_03_PIN
    #define LCD_PINS_D4              EXP1_05_PIN

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

    CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "ZONESTAR_LCD")

    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_02_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                  PA1   // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

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

      CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "TFTGLCD_PANEL_SPI")

      /**
       * TFTGLCD_PANEL_SPI display pinout
       *
       *                   Board                                   Display
       *                   ------                                  ------
       * (SD_DET)    PB5  | 1  2 | PB6 or PA15 (BEEPER)        5V |10  9 | GND
       * (MOD_RESET) PA9  | 3  4 | RESET                       -- | 8  7 | (SD_DET)
       * (SD_CS)     PA10   5  6 | PB9                    (MOSI)  | 6  5 | --
       * (LCD_CS)    PB8  | 7  8 | PB7 or PB15            (SD_CS) | 4  3 | (LCD_CS)
       *              GND | 9 10 | 5V                     (SCK)   | 2  1 | (MISO)
       *                   ------                                  ------
       *                    EXP1                                    EXP1
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

  #elif ENABLED(FYSETC_MINI_12864_2_1)

    CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "FYSETC_MINI_12864_2_1 and clones")

    /**
     * FYSETC_MINI_12864_2_1 / MKS_MINI_12864_V3 / BTT_MINI_12864 / BEEZ_MINI_12864 display pinout
     *
     *       Board                         Display
     *       ------                        ------
     * PB5  | 1  2 | PB6 or PA15   (BEEP) |10  9 | BTN_ENC
     * PA9  | 3  4 | RESET         LCD_CS | 8  7 | LCD A0
     * PA10 | 5  6 | PB9          LCD_RST | 6  5 | RED
     * PB8  | 7  8 | PB7 or PB15  (GREEN) | 4  3 | (BLUE)
     * GND  | 9 10 | 5V               GND | 2  1 | 5V
     *       ------                        ------
     *        EXP1                          EXP1
     *
     *            ---                   ------
     *       RST | 1 |          (MISO) |10  9 | SCK
     * (RX2) PA3 | 2 |         BTN_EN1 | 8  7 | (SS)
     * (TX2) PA2 | 3 |         BTN_EN2 | 6  5 | MOSI
     *       GND | 4 |            (CD) | 4  3 | (RST)
     *        5V | 5 |             GND | 2  1 | (KILL)
     *            ---                   ------
     *            TFT                    EXP2
     *
     * Needs custom cable:
     *
     *    Board             Display
     *
     *   EXP1-10 ---------- EXP1-1   5V
     *   EXP1-9 ----------- EXP1-2   GND
     *   EXP1-8 ----------- EXP2-6   EN2
     *   EXP1-7 ----------- EXP1-5   RED
     *   EXP1-6 ----------- EXP2-8   EN1
     *   EXP1-5 ----------- n/c
     *   EXP1-4 ----------- EXP1-6   RESET
     *   EXP1-3 ----------- EXP1-8   LCD_CS
     *   EXP1-2 ----------- EXP1-9   ENC
     *   EXP1-1 ----------- EXP1-7   LCD_A0
     *
     *    TFT-2 ----------- EXP2-5   MOSI
     *    TFT-3 ----------- EXP2-9   SCK
     *
     * for backlight configuration see steps 2 (V2.1) and 3 in https://wiki.fysetc.com/Mini12864_Panel/
     */

    #define LCD_BACKLIGHT_PIN               -1
    #define NEOPIXEL_PIN             EXP1_07_PIN
    #define LCD_CONTRAST_INIT                255

    #define DOGLCD_CS                EXP1_03_PIN
    #define DOGLCD_A0                EXP1_01_PIN
    #define DOGLCD_SCK                      PA2
    #define DOGLCD_MOSI                     PA3

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_06_PIN
    #define BTN_EN2                  EXP1_08_PIN

    #define FORCE_SOFT_SPI

  #else
    #error "Only CR10_STOCKDISPLAY, LCD_FOR_MELZI, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, TFTGLCD_PANEL_(SPI|I2C), FYSETC_MINI_12864_2_1, MKS_MINI_12864_V3, and BTT_MINI_12864 are currently supported on the SKR Mini E3."
  #endif

#endif // HAS_WIRED_LCD

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("BTT_SKR_MINI_E3_common", "LCD_FYSETC_TFT81050")

  /**
   * FYSETC TFT TFT81050 display pinout
   *
   *                   Board                                   Display
   *                   ------                                  ------
   * (SD_DET)    PB5  | 1  2 | PB6 or PA15 (BEEPER)        5V |10  9 | GND
   * (MOD_RESET) PA9  | 3  4 | RESET                  (RESET) | 8  7 | (SD_DET)
   * (SD_CS)     PA10   5  6 | PB9                    (MOSI)  | 6  5 | (LCD_CS)
   * (LCD_CS)    PB8  | 7  8 | PB7 or PB15            (SD_CS) | 4  3 | (MOD_RESET)
   *              GND | 9 10 | 5V                     (SCK)   | 2  1 | (MISO)
   *                   ------                                  ------
   *                    EXP1                                    EXP1
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

  #define CLCD_SPI_BUS 1                          // SPI1 connector

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
  #define SD_DETECT_PIN                     PC4
#elif SD_CONNECTION_IS(LCD) && (ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050) || IS_TFTGLCD_PANEL)
  #define SD_DETECT_PIN              EXP1_01_PIN
  #define SD_SS_PIN                  EXP1_05_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with SKR Mini E3."
#endif

#define ONBOARD_SPI_DEVICE 1                      // SPI1 -> used only by HAL/STM32F1...
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card

#define ENABLE_SPI1
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7
