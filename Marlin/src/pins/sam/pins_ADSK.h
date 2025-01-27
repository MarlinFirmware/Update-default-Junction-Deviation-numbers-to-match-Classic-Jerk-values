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
 * Arduino DUE Shield Kit (ADSK) pin assignments
 */

#define BOARD_INFO_NAME "ADSK"

#define ALLOW_MEGA1280
#define ALLOW_MEGA2560
#include "env_validate.h"

/* CNC shield modifications:
FROM THE BOTTOM CUT THE 5V PIN THAT GOES TO ARDUINO!!!
On the top put jumper between 5V and 3V3 pins,
    jumper between D12 and A.STEP, jumper between D13 and A.DIR
*/

/* CNC shield 3D printer connections:
X,Y,Z steppers as normal
A stepper for E0 extruder
(X-)&(GND) - X limit
(Y-)&(GND) - Y limit
(Z-)&(GND) - Z limit
(Abort)&(GND) - Extruder thermistor (also require pullup resistor 4.7K between "Abort" and
                                      Vcc (now "5V" on the board but actual 3.3V because of jumper))
(Hold)&(GND) - Bed thermistor (also require pullup resistor 4.7K between "Hold" and
                                 Vcc (now "5V" on the board but actual 3.3V because of jumper))
(CoolEn) - 3.3v signal to control extruder heater MOSFET
(Resume) - 3.3v signal to control heatbed MOSFET
(SDA) - 3.3v signal to control extruder fan
(SCL) - 3.3v signal to control extruder cooling fan
*/

/* CNC Shield pinout
"Name on the board": DUE pin
"Abort":    Analog pin 0 or Digital pin 54
"Hold":     Analog pin 1 or Digital pin 55
"Resume":   Analog pin 2 or Digital pin 56
"CoolEn":   Analog pin 3 or Digital pin 57
"SDA":      Analog pin 4 or Digital pin 58
"SCL":      Analog pin 5 or Digital pin 59
"E-STOP":   Reset pin
"RX":       Digital pin 0
"TX":       Digital pin 1
"X.STEP":   Digital pin 2
"Y.STEP":   Digital pin 3
"Z.STEP":   Digital pin 4
"X.DIR":    Digital pin 5
"Y.DIR":    Digital pin 6
"Z.DIR":    Digital pin 7
"EN":       Digital pin 8
"X+","X-":  Digital pin 9
"Y+","Y-":  Digital pin 10
"Z+","Z-":  Digital pin 11
"SpinEn":   Digital pin 12 -> will be connected to A.STEP with jumper
"SpinDir":  Digital pin 13 -> will be connected to A.DIR with jumper
*/

//
// Servos
//
#define SERVO0_PIN                            61  // Analog pin 7, Digital pin 61

//
// Limit Switches
//
#define X_STOP_PIN                             9
#define Y_STOP_PIN                            10
#define Z_STOP_PIN                            11

#define Z_MIN_PROBE_PIN                       62  // Analog pin 8, Digital pin 62

//
// Steppers
//
#define X_STEP_PIN                             2
#define X_DIR_PIN                              5
#define X_ENABLE_PIN                           8

#define Y_STEP_PIN                             3
#define Y_DIR_PIN                              6
#define Y_ENABLE_PIN                           8

#define Z_STEP_PIN                             4
#define Z_DIR_PIN                              7
#define Z_ENABLE_PIN                           8

#define E0_STEP_PIN                           12
#define E0_DIR_PIN                            13
#define E0_ENABLE_PIN                          8

//
// Heaters / Fans
//
#define HEATER_0_PIN                          55  // "Hold":   Analog pin 1, Digital pin 55
#define HEATER_BED_PIN                        57  // "CoolEn": Analog pin 3, Digital pin 57
#define FAN0_PIN                              54  // "Abort":  Analog pin 0, Digital pin 54
#undef E0_AUTO_FAN_PIN
#define E0_AUTO_FAN_PIN                       56  // "Resume": Analog pin 2, Digital pin 56

//
// Temperature Sensors
//
#define TEMP_0_PIN                             4  // "SDA":    Analog pin 4, Digital pin 58
#define TEMP_BED_PIN                           5  // "SCL":    Analog pin 5, Digital pin 59

//
// Misc. Functions
//
#define SD_SS_PIN                             52

#if ENABLED(ZONESTAR_LCD)

  /**
   * The 2004 LCD should be powered with 5V.
   * The next LCD pins RS,D4,D5,D6,D7 have internal pull-ups to 5V and as result the 5V will be on these pins.
   * Luckily these internal pull-ups have really high resistance and adding 33K pull-down resistors will create
   * simple voltage divider that will bring the voltage down just slightly below 3.3V.
   *
   * This LCD also has buttons that connected to the same ADC pin with different voltage divider combinations.
   * On the LCD panel there is internal pull-up resistor of the 4.7K connected to 5V.
   * Connecting another 4.7K pull-down resistor between ADC pin and the GND
   *   will result in scaled values for voltage dividers and will bring them down to be always below 3.3V.
   *
   * For 2004 LCD to work with 3.3V board like Arduino DUE the next required:
   * Pull-down resistors of 33K between each of LCD pins RS,D4,D5,D6,D7 and the GND.
   * Pull-down resistor of 4.7K between ADC_KEYPAD_PIN and the GND
   *
   * All these modifications will still work with 5V based boards but require proper scaled ADC values
   */

  #ifdef __SAM3X8E__
    #define AREF_VOLTS 3.3
  #else
    #define AREF_VOLTS 5.0
  #endif

  //
  // LCD / Controller
  //
  #define LCD_PINS_EN                         14
  #define LCD_PINS_RS                         15
  #define LCD_PINS_D4                         16
  #define LCD_PINS_D5                         17
  #define LCD_PINS_D6                         18
  #define LCD_PINS_D7                         19
  #define ADC_KEYPAD_PIN                       6  //60 // Analog pin 6, Digital pin 60

  /**
   * The below defines will scale all the values to work properly on both
   * 5V (Mega) and 3.3V (DUE) boards with all pull-up resistors added for 3.3V
   */

  #define ADC_BUTTONS_VALUE_SCALE         (5.0/AREF_VOLTS) // The LCD module pullup voltage is 5.0V but ADC reference voltage is 3.3V

  #define ADC_BUTTONS_R_PULLDOWN          4.7     // Moves voltage down to be below 3.3V instead of 5V
  // the resistors values will be scaled because of 4.7K pulldown parallel resistor
  #define _ADC_BUTTONS_R_SCALED(R)        ((R) * (ADC_BUTTONS_R_PULLDOWN) / ((R) + ADC_BUTTONS_R_PULLDOWN))

  // buttons pullup resistor
  #define ADC_BUTTONS_R_PULLUP            4.7     // the resistor on the 2004 LCD panel
  // buttons resistors with scaled values because of parallel pulldown resistor
  #define ADC_BUTTONS_LEFT_R_PULLDOWN     _ADC_BUTTONS_R_SCALED(0.47)
  #define ADC_BUTTONS_RIGHT_R_PULLDOWN    _ADC_BUTTONS_R_SCALED(4.7)
  #define ADC_BUTTONS_UP_R_PULLDOWN       _ADC_BUTTONS_R_SCALED(1.0)
  #define ADC_BUTTONS_DOWN_R_PULLDOWN     _ADC_BUTTONS_R_SCALED(10.0)
  #define ADC_BUTTONS_MIDDLE_R_PULLDOWN   _ADC_BUTTONS_R_SCALED(2.2)

#endif // ZONESTAR_LCD

/**
 * RJ45 8 pins extruder connector
 *
 * 1 - GND (Please do not connect to the same GND as extruder heater to prevent ground offset voltage)
 * 2 - thermistor
 * 3 - SERVO PWM
 * 4 - extruder heater
 * 5 - FAN (print cooling)
 * 6 - FAN (extruder cooling)
 * 7 - Probe signal
 * 8 - 5V
 *
 * Standard ethernet pairs: 1&2, 3&6, 4&5, 7&8
 * Use CAT7 cable to have all pairs shielded
 */
