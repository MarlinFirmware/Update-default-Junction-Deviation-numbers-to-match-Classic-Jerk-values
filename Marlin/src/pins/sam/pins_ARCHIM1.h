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
 * ARCHIM1 pin assignment
 *
 * The Archim 1.0 board requires Arduino Archim addons installed.
 *
 * - Add the following URL to Arduino IDE's Additional Boards Manager URLs:
 *   https://raw.githubusercontent.com/ultimachine/ArduinoAddons/master/package_ultimachine_index.json
 *
 * - In the Arduino IDE Boards Manager search for Archim and install the package.
 *
 * - Change your target board to "Archim".
 *
 * Further information on the UltiMachine website...
 *   https://github.com/ultimachine/Archim/wiki
 */

#if NOT_TARGET(__SAM3X8E__)
  #error "Oops! Select 'Archim' in 'Tools > Board.'"
#endif

#define BOARD_INFO_NAME "Archim 1.0"

//
// Timers
//
#define MF_TIMER_STEP  3
#define HAL_STEP_TIMER_ISR()  void TC3_Handler()

//
// Items marked * have been altered from Archim v1.0
//

//
// Servos
//
#define SERVO0_PIN                            20  // D20 PB12 (Header J20 20)
#define SERVO1_PIN                            21  // D21 PB13 (Header J20 19)

//
// Limit Switches
//
#define X_MIN_PIN                             14  // PD4 MIN ES1
#define X_MAX_PIN                             32  // PD10 MAX ES1
#define Y_MIN_PIN                             29  // PD6 MIN ES2
#define Y_MAX_PIN                             15  // PD5 MAX ES2
#define Z_MIN_PIN                             31  // PA7 MIN ES3
#define Z_MAX_PIN                             30  // PD9 MAX ES3

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     32
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      66  // D66 PB15 (Header J20 15)
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                     67  // D67 PB16 (Header J20 16)
#endif

//
// Steppers
//
#define X_STEP_PIN                            40  // PC8 STEP1 *
#define X_DIR_PIN                             59  // PA4 DIR1 *
#define X_ENABLE_PIN                          41  // PC9 EN1

#define Y_STEP_PIN                            49  // PC14 STEP2 *
#define Y_DIR_PIN                             47  // PC16 DIR2 *
#define Y_ENABLE_PIN                          48  // PC15 EN2 *

#define Z_STEP_PIN                            36  // PC4 STEP Z *
#define Z_DIR_PIN                            107  // PB10 DIR Z *
#define Z_ENABLE_PIN                          96  // PC10 EN Z -AddOns *

#define E0_STEP_PIN                           78  // PB23 STEP3 *
#define E0_DIR_PIN                            22  // PB26 DIR3 *
#define E0_ENABLE_PIN                         97  // PB24 EN3 -Addons *

#define E1_STEP_PIN                           26  // PD1 STEP4 *
#define E1_DIR_PIN                            27  // PD2 DIR4 *
#define E1_ENABLE_PIN                         28  // PD3 EN4 *

// Microstepping mode pins *
#define X_MS1_PIN                             39  // PC7 MOD0E1 - As listed in schematic
#define X_MS2_PIN                             38  // PC6 MOD1E1
#define X_MS3_PIN                             37  // PC5 MOD2E1

#define Y_MS1_PIN                             50  // PC13 MODE0E2
#define Y_MS2_PIN                             51  // PC12 MODE1E2
#define Y_MS3_PIN                             92  // PC11 MODE2E2 - AddOns

#define Z_MS1_PIN                             44  // PC19 MOD0E Z
#define Z_MS2_PIN                             45  // PC18 MOD1E Z
#define Z_MS3_PIN                             46  // PC17 MOD2E Z

#define E0_MS1_PIN                           105  // PB22 MOD0E3 - AddOns
#define E0_MS2_PIN                           106  // PC27 MOD1E3 - AddOns
#define E0_MS3_PIN                           104  // PC20 MOD2E3 - AddOns

#define E1_MS1_PIN                            25  // PD0 MOD0E4
#define E1_MS2_PIN                            18  // PA11 MOD1E4
#define E1_MS3_PIN                            19  // PA10 MOD2E4

// Motor current PWM pins *
#define MOTOR_CURRENT_PWM_X_PIN               58  // PA6   X-REF TIOB2
#define MOTOR_CURRENT_PWM_Y_PIN               12  // PD8   Y-REF TIOB8
#define MOTOR_CURRENT_PWM_Z_PIN               10  // PC29  Z-REF TIOB7
#define MOTOR_CURRENT_PWM_E0_PIN               3  // PC28 E1-REF TIOA7
#define MOTOR_CURRENT_PWM_E1_PIN              11  // PD7  E2-REF TIOA8

#define MOTOR_CURRENT_PWM_RANGE              2750 // (3.3 Volts * 100000 Ohms) / (100000 Ohms + 20000 Ohms) = 2.75 Volts (max vref)
#define DEFAULT_PWM_MOTOR_CURRENT  { 1000, 1000, 1000 } //, 1000, 1000}   // X Y Z E0 E1, 1000 = 1000mAh

//
// Temperature Sensors
//
#define TEMP_0_PIN                            10  // D10 PB19 THERM AN1 *
#define TEMP_1_PIN                             9  // D9 PB18 THERM AN2 *
#define TEMP_2_PIN                             8  // D8 PB17 THERM AN4 *
#define TEMP_BED_PIN                          11  // D11 PB20 THERM AN3 *

//
// Heaters / Fans
//
#define HEATER_0_PIN                           6  // D6 PC24 FET_PWM3
#define HEATER_1_PIN                           7  // D7 PC23 FET_PWM4
#define HEATER_2_PIN                           8  // D8 PC22 FET_PWM5
#define HEATER_BED_PIN                         9  // D9 PC21 BED_PWM

#ifndef FAN0_PIN
  #define FAN0_PIN                             4  // D4 PC26 FET_PWM1
#endif
#define FAN1_PIN                               5  // D5 PC25 FET_PWM2

//
// Misc. Functions
//

// Internal MicroSD card reader on the PCB
#define INT_SCK_PIN                           42  // D42 PA19/MCCK
#define INT_MISO_PIN                          43  // D43 PA20/MCCDA
#define INT_MOSI_PIN                          73  // D73 PA21/MCDA0
#define INT_SDSS                              55  // D55 PA24/MCDA3

// External SD card reader on SC2
#define SD_SCK_PIN                            76  // D76 PA27
#define SD_MISO_PIN                           74  // D74 PA25
#define SD_MOSI_PIN                           75  // D75 PA26
#define SD_SS_PIN                             87  // D87 PA29

// 2MB SPI Flash
#define SPI_FLASH_SS                          52  // D52 PB21

//
// LCD / Controller
//

#if HAS_WIRED_LCD
  #define BEEPER_PIN                          23  // D24 PA15_CTS1
  #define LCD_PINS_RS                         17  // D17 PA12_RXD1
  #define LCD_PINS_EN                         24  // D23 PA14_RTS1
  #define LCD_PINS_D4                         69  // D69 PA0_CANTX0
  #define LCD_PINS_D5                         54  // D54 PA16_SCK1
  #define LCD_PINS_D6                         68  // D68 PA1_CANRX0
  #define LCD_PINS_D7                         34  // D34 PC2_PWML0

  #define SD_DETECT_PIN                        2  // D2  PB25_TIOA0

  #if IS_NEWPANEL
    // Buttons on AUX-2
    #define BTN_EN1                           60  // D60 PA3_TIOB1
    #define BTN_EN2                           13  // D13 PB27_TIOB0
    #define BTN_ENC                           16  // D16 PA13_TXD1
  #endif // IS_NEWPANEL

  #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
    #define BTN_ENC_EN               LCD_PINS_D7  // Detect the presence of the encoder
  #endif

#endif // HAS_WIRED_LCD
