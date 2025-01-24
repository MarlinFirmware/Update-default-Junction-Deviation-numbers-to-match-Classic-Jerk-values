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

#define BOARD_INFO_NAME "BTT SKR E3 DIP V1.x"

// Release PB3/PB4 (TMC_SW Pins) from JTAG pins
#define DISABLE_JTAG

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
#define X_STOP_PIN                          PC1   // X-STOP
#define Y_STOP_PIN                          PC0   // Y-STOP
#define Z_STOP_PIN                          PC15  // Z-STOP

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
  #define FIL_RUNOUT_PIN                    PC2   // E0-STOP
#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC7
#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PB15
#ifndef X_CS_PIN
  #define X_CS_PIN                          PC10
#endif

#define Y_ENABLE_PIN                        PB14
#define Y_STEP_PIN                          PB13
#define Y_DIR_PIN                           PB12
#ifndef Y_CS_PIN
  #define Y_CS_PIN                          PC11
#endif

#define Z_ENABLE_PIN                        PB11
#define Z_STEP_PIN                          PB10
#define Z_DIR_PIN                           PB2
#ifndef Z_CS_PIN
  #define Z_CS_PIN                          PC12
#endif

#define E0_ENABLE_PIN                       PB1
#define E0_STEP_PIN                         PB0
#define E0_DIR_PIN                          PC5
#ifndef E0_CS_PIN
  #define E0_CS_PIN                         PD2
#endif

//
// SPI pins for TMC2130 stepper drivers
//
#ifndef TMC_SPI_MOSI
  #define TMC_SPI_MOSI                      PB5
#endif
#ifndef TMC_SPI_MISO
  #define TMC_SPI_MISO                      PB4
#endif
#ifndef TMC_SPI_SCK
  #define TMC_SPI_SCK                       PB3
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

  #define X_SERIAL_TX_PIN                   PC10
  #define Y_SERIAL_TX_PIN                   PC11
  #define Z_SERIAL_TX_PIN                   PC12
  #define E0_SERIAL_TX_PIN                  PD2

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

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
#define FAN0_PIN                            PA8   // "FAN0"

//
// USB connect control
//
#define USB_CONNECT_PIN                     PC13
#define USB_CONNECT_INVERTING              false

/**
 *                 ------
 * (BEEPER)  PA15 | 1  2 | PB6 (BTN_ENC)
 * (BTN_EN1) PA9  | 3  4 | RESET
 * (BTN_EN2) PA10   5  6 | PB9 (LCD_D4)
 * (LCD_RS)  PB8  | 7  8 | PB7 (LCD_EN)
 *            GND | 9 10 | 5V
 *                 ------
 *                  EXP1
 */

#define EXP1_01_PIN                         PA15
#define EXP1_02_PIN                         PB6
#define EXP1_03_PIN                         PA9
#define EXP1_04_PIN                         -1    // RESET
#define EXP1_05_PIN                         PA10
#define EXP1_06_PIN                         PB9
#define EXP1_07_PIN                         PB8
#define EXP1_08_PIN                         PB7

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

#if HAS_WIRED_LCD

  #if ENABLED(CR10_STOCKDISPLAY)

    #define BEEPER_PIN               EXP1_01_PIN

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define LCD_PINS_RS              EXP1_07_PIN
    #define LCD_PINS_EN              EXP1_08_PIN
    #define LCD_PINS_D4              EXP1_06_PIN

  #elif ENABLED(ZONESTAR_LCD)                     // ANET A8 LCD Controller - Must convert to 3.3V - CONNECTING TO 5V WILL DAMAGE THE BOARD!

    CONTROLLER_WARNING("BTT_SKR_E3_DIP", "ZONESTAR_LCD")

    #define LCD_PINS_RS              EXP1_06_PIN
    #define LCD_PINS_EN              EXP1_02_PIN
    #define LCD_PINS_D4              EXP1_07_PIN
    #define LCD_PINS_D5              EXP1_05_PIN
    #define LCD_PINS_D6              EXP1_03_PIN
    #define LCD_PINS_D7              EXP1_01_PIN
    #define ADC_KEYPAD_PIN                  PA1   // Repurpose servo pin for ADC - CONNECTING TO 5V WILL DAMAGE THE BOARD!

  #elif ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

    /** Creality Ender-2 display pinout
     *                   ------
     *   (SCK)     PA15 | 1  2 | PB6 (BTN_ENC)
     *   (BTN_EN1) PA9  | 3  4 | RESET
     *   (BTN_EN2) PA10   5  6 | PB9 (LCD_A0)
     *   (LCD_RS)  PB8  | 7  8 | PB7 (MOSI)
     *              GND | 9 10 | 5V
     *                   ------
     *                    EXP1
     */

    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN

    #define DOGLCD_CS                EXP1_07_PIN
    #define DOGLCD_A0                EXP1_06_PIN
    #define DOGLCD_SCK               EXP1_01_PIN
    #define DOGLCD_MOSI              EXP1_08_PIN
    #define FORCE_SOFT_SPI
    #define LCD_BACKLIGHT_PIN               -1

  #elif ENABLED(FYSETC_MINI_12864_2_1)

    CONTROLLER_WARNING("BTT_SKR_E3_DIP", "FYSETC_MINI_12864_2_1 and clones")

    #if SD_CONNECTION_IS(LCD)
      #error "The LCD SD Card is not supported with this configuration."
    #endif

    /**
     * FYSETC_MINI_12864_2_1 / MKS_MINI_12864_V3 / BTT_MINI_12864 / BEEZ_MINI_12864 display pinout
     *
     *                   Board                               Display
     *                   ------                               ------
     * (NEOPIXEL)  PA15 | 1  2 | PB6 (BTN_ENC)            5V |10  9 | GND
     * (BTN_EN2)   PA9  | 3  4 | RESET                    -- | 8  7 | --
     * (BTN_EN1)   PA10   5  6 | PB9 (LCD_RESET)    NEOPIXEL | 6  5   LCD RESET
     * (LCD_A0)    PB8  | 7  8 | PB7 (LCD_CS)         LCD_A0 | 4  3 | LCD_CS
     *              GND | 9 10 | 5V                  BTN_ENC | 2  1 | BEEP
     *                   ------                               ------
     *                    EXP1                                 EXP1
     *
     *
     *                   ---                                  ------
     *                  | 1 | RST                         -- |10  9 | --
     *                  | 2 | PA3 RX2              RESET_BTN | 8  7 | SD_DETECT
     *                  | 3 | PA2 TX2               LCD_MOSI | 6  5   EN2
     *                  | 4 | GND                         -- | 4  3 | EN1
     *                  | 5 | 5V                     LCD_SCK | 2  1 | --
     *                   ---                                  ------
     *                   TFT                                   EXP2

     *
     * Needs custom cable.
     *
     *    BOARD EXP1 NEOPIXEL   <-->  LCD EXP1 NEOPIXEL
     *    BOARD EXP1 BTN_ENC    <-->  LCD EXP1 BTN_ENC
     *    BOARD EXP1 BTN_EN2    <-->  LCD EXP2 EN2
     *    BOARD EXP1 RESET      <-->  LCD EXP2 RESET_BTN
     *    BOARD EXP1 BTN_EN1    <-->  LCD EXP2 EN1
     *    BOARD EXP1 LCD_RESET  <-->  LCD EXP1 LCD RESET
     *    BOARD EXP1 LCD_A0     <-->  LCD EXP1 LCD_A0
     *    BOARD EXP1 LCD_CS     <-->  LCD EXP1 LCD_CS
     *    BOARD TFT RX2         <-->  LCD EXP2 LCD_MOSI
     *    BOARD TFT TX2         <-->  LCD EXP2 LCD_SCK
     */

    #define NEOPIXEL_PIN             EXP1_01_PIN
    #define BTN_ENC                  EXP1_02_PIN
    #define BTN_EN1                  EXP1_03_PIN
    #define BTN_EN2                  EXP1_05_PIN
    #define LCD_RESET_PIN            EXP1_06_PIN
    #define BEEPER_PIN                      -1
    #define DOGLCD_A0                EXP1_07_PIN
    #define DOGLCD_CS                EXP1_08_PIN

    #define DOGLCD_SCK                      PA2
    #define DOGLCD_MOSI                     PA3

    #define LCD_BACKLIGHT_PIN               -1
    #define FORCE_SOFT_SPI

  #else
    #error "Only CR10_STOCKDISPLAY, ZONESTAR_LCD, ENDER2_STOCKDISPLAY, MKS_MINI_12864, FYSETC_MINI_12864_2_1 and MKS_LCD12864A/B are currently supported on the SKR E3 DIP."
  #endif

#endif // HAS_WIRED_LCD

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("BTT_SKR_E3_DIP", "LCD_FYSETC_TFT81050")

  /** FYSETC TFT TFT81050 display pinout
   *
   *                   Board                                   Display
   *                   ------                                  ------
   * (SD_DET)    PA15 | 1  2 | PB6 (BEEPER)                5V |10  9 | GND
   * (MOD_RESET) PA9  | 3  4 | RESET                  (RESET) | 8  7 | (SD_DET)
   * (SD_CS)     PA10   5  6 | PB9                    (MOSI)    6  5 | (LCD_CS)
   * (LCD_CS)    PB8  | 7  8 | PB7                    (SD_CS) | 4  3 | (MOD_RESET)
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
   *   SP11-3 ----------- EXP1-2   SCK
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
// SD Card
//
#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

#if SD_CONNECTION_IS(ONBOARD)
  #define SD_DETECT_PIN                     PC4
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
#elif SD_CONNECTION_IS(LCD) && ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)
  #define SD_DETECT_PIN              EXP1_01_PIN
  #define SD_SS_PIN                  EXP1_05_PIN
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with SKR E3 DIP."
#endif

#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card
#define SD_SS_PIN              ONBOARD_SD_CS_PIN
