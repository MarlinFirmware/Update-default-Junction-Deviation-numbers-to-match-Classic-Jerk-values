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

//
// Some of the pin mapping functions of the Arduino IDE Teensduino extension
// function differently from other Arduino extensions.
//

#define TEENSYDUINO_IDE

//digitalPinToTimer(pin) function works like Arduino but Timers are not defined
#define TIMER0B 1
#define TIMER1A 7
#define TIMER1B 8
#define TIMER1C 9
#define TIMER2A 6
#define TIMER2B 2
#define TIMER3A 5
#define TIMER3B 4
#define TIMER3C 3

// digitalPinToPort function just returns the pin number so need to create our own
#define PA 1
#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6

const uint8_t PROGMEM digital_pin_to_port_PGM[] = {
  PD, // 0  - PD0 - INT0 - PWM
  PD, // 1  - PD1 - INT1 - PWM
  PD, // 2  - PD2 - INT2 - RX
  PD, // 3  - PD3 - INT3 - TX
  PD, // 4  - PD4
  PD, // 5  - PD5
  PD, // 6  - PD6
  PD, // 7  - PD7
  PE, // 8  - PE0
  PE, // 9  - PE1
  PC, // 10 - PC0
  PC, // 11 - PC1
  PC, // 12 - PC2
  PC, // 13 - PC3
  PC, // 14 - PC4 - PWM
  PC, // 15 - PC5 - PWM
  PC, // 16 - PC6 - PWM
  PC, // 17 - PC7
  PE, // 18 - PE6 - INT6
  PE, // 19 - PE7 - INT7
  PB, // 20 - PB0
  PB, // 21 - PB1
  PB, // 22 - PB2
  PB, // 23 - PB3
  PB, // 24 - PB4 - PWM
  PB, // 25 - PB5 - PWM
  PB, // 26 - PB6 - PWM
  PB, // 27 - PB7 - PWM
  PA, // 28 - PA0
  PA, // 29 - PA1
  PA, // 30 - PA2
  PA, // 31 - PA3
  PA, // 32 - PA4
  PA, // 33 - PA5
  PA, // 34 - PA6
  PA, // 35 - PA7
  PE, // 36 - PE4 - INT4
  PE, // 37 - PE5 - INT5
  PF, // 38 - PF0 - A0
  PF, // 39 - PF1 - A1
  PF, // 40 - PF2 - A2
  PF, // 41 - PF3 - A3
  PF, // 42 - PF4 - A4
  PF, // 43 - PF5 - A5
  PF, // 44 - PF6 - A6
  PF, // 45 - PF7 - A7
  PE, // 46 - PE2 (not defined in teensyduino)
  PE, // 47 - PE3 (not defined in teensyduino)
};

#define digitalPinToPort(P) pgm_read_byte(digital_pin_to_port_PGM[P])

// digitalPinToBitMask(pin) is OK

#define digitalRead_mod(P)  extDigitalRead(P)   // Teensyduino's version of digitalRead doesn't
                                                // disable the PWMs so we can use it as is

// portModeRegister(pin) is OK
