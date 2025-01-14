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

#if NOT_TARGET(STM32F767xx)
  #error "Oops! Select an STM32F767 environment"
#endif

#define BOARD_INFO_NAME      "NUCLEO-F767ZI"
#define DEFAULT_MACHINE_NAME "Prototype Board"

#if NO_EEPROM_SELECTED
  #define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  // Decrease delays and flash wear by spreading writes across the
  // 128 kB sector allocated for EEPROM emulation.
  // Not yet supported on F7 hardware
  //#define FLASH_EEPROM_LEVELING
#endif

/**
 * Timer assignments
 *
 * TIM1 -
 * TIM2 - Hardware PWM (Fan/Heater Pins)
 * TIM3 - Hardware PWM (Servo Pins)
 * TIM4 - STEP_TIMER (Marlin)
 * TIM5 -
 * TIM6 - TIMER_TONE (variant.h)
 * TIM7 - TIMER_SERVO (variant.h)
 * TIM9 - TIMER_SERIAL (platformio.ini)
 * TIM10 - For some reason trips Watchdog when used for SW Serial
 * TIM11 -
 * TIM12 -
 * TIM13 -
 * TIM14 - TEMP_TIMER (Marlin)
 *
 */
#define STEP_TIMER  4
#define TEMP_TIMER 14

/**
 * These pin assignments are arbitrary and intending for testing purposes.
 * Assignments may not be ideal, and not every assignment has been tested.
 * Proceed at your own risk.
 *                                                            _CN7_
 *                                              (X_STEP) PC6 | · · | PB8 (X_EN)
 *                                              (X_DIR) PB15 | · · | PB9 (X_CS)
 *                                             (LCD_D4) PB13 | · · | AVDD
 *                 _CN8_                                PB12 | · · | GND
 *             -- | · · | PC8                (HEATER_0) PA15 | · · | PA5  (SCLK)
 *          IOREF | · · | PC9                   (BEEPER) PC7 | · · | PA6  (MISO)
 *          RESET | · · | PC10              (SERVO1_PIN) PB5 | · · | PA7  (MOSI)
 *          +3.3V | · · | PC11              (HEATER_BED) PB3 | · · | PD14 (SD_DETECT)
 *            +5V | · · | PC12                    (SDSS) PA4 | · · | PD15 (LCD_ENABLE)
 *            GND | · · | PD2               (SERVO0_PIN) PB4 | · · | PF12 (LCD_RS)
 *            GND | · · | PG2                                 ￣￣￣
 *            VIN | · · | PG3
 *                 ￣￣￣                                      _CN10
 *                                                      AVDD | · · | PF13 (BTN_EN1)
 *                 _CN9_                                AGND | · · | PE9  (BTN_EN2)
 *   (TEMP_0) PA3 | · · | PD7                            GND | · · | PE11 (BTN_ENC)
 * (TEMP_BED) PC0 | · · | PD6                            PB1 | · · | PF14
 *            PC3 | · · | PD5                            PC2 | · · | PE13
 *            PF3 | · · | PD4                            PF4 | · · | PF15
 *            PF5 | · · | PD3                   (E_STEP) PB6 | · · | PG14 (E_EN)
 *           PF10 | · · | GND                    (E_DIR) PB2 | · · | PG9  (E_CS)
 *             -- | · · | PE2                            GND | · · | PE8
 *            PA7 | · · | PE4                           PD13 | · · | PE7
 *            PF2 | · · | PE5                           PD12 | · · | GND
 *   (Y_STEP) PF1 | · · | PE6 (Y_EN)           (Z_STEP) PD11 | · · | PE10 (Z_EN)
 *    (Y_DIR) PF0 | · · | PE3 (Y_CS)             (Z_DIR) PE2 | · · | PE12 (Z_CS)
 *            GND | · · | PF8                            GND | · · | PE14
 *    (Z_MAX) PD0 | · · | PF7 (X_MIN)                    PA0 | · · | PE15
 *    (Z_MIN) PD1 | · · | PF9 (X_MAX)                    PB0 | · · | PB10 (FAN)
 *    (Y_MAX) PG0 | · · | PG1 (Y_MIN)                    PE0 | · · | PB11 (FAN1)
 *                 ￣￣￣                                     ￣￣￣￣
 */

#define X_MIN_PIN                           PF7
#define X_MAX_PIN                           PF9
#define Y_MIN_PIN                           PG1
#define Y_MAX_PIN                           PG0
#define Z_MIN_PIN                           PD1
#define Z_MAX_PIN                           PD0

//
// Steppers
//
#define X_STEP_PIN                          PC6
#define X_DIR_PIN                           PB15
#define X_ENABLE_PIN                        PB8
#define X_CS_PIN                            PB9

#define Y_STEP_PIN                          PF1
#define Y_DIR_PIN                           PF0
#define Y_ENABLE_PIN                        PE6
#define Y_CS_PIN                            PE3

#define Z_STEP_PIN                          PD11
#define Z_DIR_PIN                           PE2
#define Z_ENABLE_PIN                        PE10
#define Z_CS_PIN                            PE12

#define E0_STEP_PIN                         PB6
#define E0_DIR_PIN                          PB2
#define E0_ENABLE_PIN                       PG14
#define E0_CS_PIN                           PG9

#if HAS_TMC_UART
  #define X_SERIAL_TX_PIN                   PB9
  #define Y_SERIAL_TX_PIN                   PE3
  #define Z_SERIAL_TX_PIN                   PE12
  #define E0_SERIAL_TX_PIN                  PG9
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PA3
#define TEMP_BED_PIN                        PC0

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PA15  // PWM Capable, TIM2_CH1
#define HEATER_BED_PIN                      PB3   // PWM Capable, TIM2_CH2

#ifndef FAN0_PIN
  #define FAN0_PIN                          PB10  // PWM Capable, TIM2_CH3
#endif
#define FAN1_PIN                            PB11  // PWM Capable, TIM2_CH4

#ifndef E0_AUTO_FAN_PIN
  #define E0_AUTO_FAN_PIN               FAN1_PIN
#endif

//
// Servos
//
#define SERVO0_PIN                          PB4   // PWM Capable, TIM3_CH1
#define SERVO1_PIN                          PB5   // PWM Capable, TIM3_CH2

// SPI for external SD Card (Not entirely sure this will work)
#define SD_SCK_PIN                          PA5
#define SD_MISO_PIN                         PA6
#define SD_MOSI_PIN                         PA7
#define SD_SS_PIN                           PA4
#define SDSS                                PA4

#define LED_PIN                         LED_BLUE

//
// LCD / Controller
//

#if IS_RRD_FG_SC
  #define BEEPER_PIN                        PC7   // LCD_BEEPER
  #define BTN_ENC                           PE11  // BTN_ENC
  #define SD_DETECT_PIN                     PD14
  #define LCD_PINS_RS                       PF12  // LCD_RS
  #define LCD_PINS_EN                       PD15  // LCD_EN
  #define LCD_PINS_D4                       PB13  // LCD_D4
  #define BTN_EN1                           PF13  // BTN_EN1
  #define BTN_EN2                           PE9   // BTN_EN2

  #define BOARD_ST7920_DELAY_1               125
  #define BOARD_ST7920_DELAY_2                63
  #define BOARD_ST7920_DELAY_3               780
#endif
