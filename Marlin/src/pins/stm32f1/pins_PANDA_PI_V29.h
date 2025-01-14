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

#define BOARD_INFO_NAME "PANDA PI V2.9"

// Release PB3/PB4 (TMC_SW Pins) from JTAG pins
#define DISABLE_JTAG

// Ignore temp readings during development.
//#define BOGUS_TEMPERATURE_GRACE_PERIOD    2000

#if ANY(NO_EEPROM_SELECTED, FLASH_EEPROM_EMULATION)
  #define FLASH_EEPROM_EMULATION
  #define EEPROM_PAGE_SIZE     (0x800U)           // 2K
  #define EEPROM_START_ADDRESS (0x8000000UL + (STM32_FLASH_SIZE) * 1024UL - (EEPROM_PAGE_SIZE) * 2UL)
  #define MARLIN_EEPROM_SIZE    EEPROM_PAGE_SIZE  // 2K
#endif

#if ENABLED(BD_SENSOR)
  #define I2C_BD_SDA_PIN                    PC6
  #define I2C_BD_SCL_PIN                    PB2
  #define I2C_BD_DELAY 10                         // (seconds)
#endif

//
// Servos
//
#define SERVO0_PIN                          PA11  // SERVOS

//
// Limit Switches
//
#define X_STOP_PIN                          PA3   // X-STOP
#define Y_STOP_PIN                          PC9   // Y-STOP
#define Z_STOP_PIN                          PA1   // Z-STOP

//
// Z Probe must be this pin
//
//#define Z_MIN_PROBE_PIN                   PA1   // PROBE

//
// Filament Runout Sensor
//
//#ifndef FIL_RUNOUT_PIN
//  #define FIL_RUNOUT_PIN                  PC2   // E0-STOP
//#endif

//
// Steppers
//
#define X_ENABLE_PIN                        PC12
#define X_STEP_PIN                          PC11
#define X_DIR_PIN                           PB6

#define Y_ENABLE_PIN                        PC12
#define Y_STEP_PIN                          PB5
#define Y_DIR_PIN                           PB4

#define Z_ENABLE_PIN                        PC12
#define Z_STEP_PIN                          PB3
#define Z_DIR_PIN                           PA15

#define E0_ENABLE_PIN                       PC12
#define E0_STEP_PIN                         PB15
#define E0_DIR_PIN                          PB14

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
#define TEMP_0_PIN                          PB0   // Analog Input "TH0"
#define TEMP_BED_PIN                        PB1   // Analog Input "TB0"
#define TEMP_1_PIN                          PA2

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PB12  // "HE"
#define HEATER_BED_PIN                      PB13  // "HB"
#define FAN0_PIN                            PA8   // "FAN0"
#define HEATER_1_PIN                        PA12

//
// SD Support
//
#define ONBOARD_SPI_DEVICE                     1  // SPI1
#define ONBOARD_SD_CS_PIN                   PA4   // Chip select for "System" SD card
#define SDSS                   ONBOARD_SD_CS_PIN

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif
#if SD_CONNECTION_IS(ONBOARD)
  //#define SD_DETECT_PIN                   PA4
  #define SD_SCK_PIN                        PA5
  #define SD_MISO_PIN                       PA6
  #define SD_MOSI_PIN                       PA7
#elif SD_CONNECTION_IS(CUSTOM_CABLE)
  #error "SD CUSTOM_CABLE is not compatible with SKR E3 DIP."
#endif

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BTN_ENC                           PA0
  #define BTN_EN1                           PC4
  #define BTN_EN2                           PC5

  #define LCD_PINS_RS                       PC0
  #define LCD_PINS_EN                       PC2
  #define LCD_PINS_D4                       PC1
#endif

#if ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)

  CONTROLLER_WARNING("PANDA_PI_V29", "LCD_FYSETC_TFT81050")

  /** FYSETC TFT TFT81050 display pinout
   *
   *                   Board                                   Display
   *                   ------                                  ------
   * (SD_DET)    PA15 | 1  2 | PB6 (BEEPER)  (SPI1-MISO) MISO | 1  2 | SCK   (SPI1-SCK)
   * (MOD_RESET) PA9  | 3  4 | RESET                MOD_RESET | 3  4 | SD_CS
   * (SD_CS)     PA10   5  6 | PB9 (FREE)              LCD_CS | 5  6   MOSI  (SPI1-MOSI)
   * (LCD_CS)    PB8  | 7  8 | PB7 (FREE)              SD_DET | 7  8 | RESET
   *             GND  | 9 10 |  5V                        GND | 9 10 | 5V
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

  #define BEEPER_PIN                        PB6

  #define CLCD_MOD_RESET                    PA9
  #define CLCD_SPI_CS                       PB8

  #if SD_CONNECTION_IS(LCD) && ALL(TOUCH_UI_FTDI_EVE, LCD_FYSETC_TFT81050)
    #define SD_DETECT_PIN                   PA15
    #define SD_SS_PIN                       PA10
  #endif

#elif HAS_WIRED_LCD

  #define BEEPER_PIN                        PC3

#endif
