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
 * Pin mapping (Teensy) for AT90USB646, 647, 1286, and 1287
 *
 *   Logical Pin: 28 29 30 31 32 33 34 35 20 21 22 23 24 25 26 27 10 11 12 13 14 15 16 17 00 01 02 03 04 05 06 07 08 09(46*47)36 37 18 19 38 39 40 41 42 43 44 45
 *   Port:        A0 A1 A2 A3 A4 A5 A6 A7 B0 B1 B2 B3 B4 B5 B6 B7 C0 C1 C2 C3 C4 C5 C6 C7 D0 D1 D2 D3 D4 D5 D6 D7 E0 E1 E2 E3 E4 E5 E6 E7 F0 F1 F2 F3 F4 F5 F6 F7
 *            Logical pins 46-47 aren't supported by Teensyduino, but are supported below as E2 and E3
 */

#include "../fastio.h"

// SPI
#define SS          20  //  8
#define SCK         21  //  9
#define MOSI        22  // 10
#define MISO        23  // 11

// Digital I/O

#define DIO0_PIN    PIND0
#define DIO0_RPORT  PIND
#define DIO0_WPORT  PORTD
#define DIO0_PWM    0
#define DIO0_DDR    DDRD

#define DIO1_PIN    PIND1
#define DIO1_RPORT  PIND
#define DIO1_WPORT  PORTD
#define DIO1_PWM    0
#define DIO1_DDR    DDRD

#define DIO2_PIN    PIND2
#define DIO2_RPORT  PIND
#define DIO2_WPORT  PORTD
#define DIO2_PWM    0
#define DIO2_DDR    DDRD

#define DIO3_PIN    PIND3
#define DIO3_RPORT  PIND
#define DIO3_WPORT  PORTD
#define DIO3_PWM    0
#define DIO3_DDR    DDRD

#define DIO4_PIN    PIND4
#define DIO4_RPORT  PIND
#define DIO4_WPORT  PORTD
#define DIO4_PWM    0
#define DIO4_DDR    DDRD

#define DIO5_PIN    PIND5
#define DIO5_RPORT  PIND
#define DIO5_WPORT  PORTD
#define DIO5_PWM    0
#define DIO5_DDR    DDRD

#define DIO6_PIN    PIND6
#define DIO6_RPORT  PIND
#define DIO6_WPORT  PORTD
#define DIO6_PWM    0
#define DIO6_DDR    DDRD

#define DIO7_PIN    PIND7
#define DIO7_RPORT  PIND
#define DIO7_WPORT  PORTD
#define DIO7_PWM    0
#define DIO7_DDR    DDRD

#define DIO8_PIN    PINE0
#define DIO8_RPORT  PINE
#define DIO8_WPORT  PORTE
#define DIO8_PWM    0
#define DIO8_DDR    DDRE

#define DIO9_PIN    PINE1
#define DIO9_RPORT  PINE
#define DIO9_WPORT  PORTE
#define DIO9_PWM    0
#define DIO9_DDR    DDRE

#define DIO10_PIN   PINC0
#define DIO10_RPORT PINC
#define DIO10_WPORT PORTC
#define DIO10_PWM   0
#define DIO10_DDR   DDRC

#define DIO11_PIN   PINC1
#define DIO11_RPORT PINC
#define DIO11_WPORT PORTC
#define DIO11_PWM   0
#define DIO11_DDR   DDRC

#define DIO12_PIN   PINC2
#define DIO12_RPORT PINC
#define DIO12_WPORT PORTC
#define DIO12_PWM   0
#define DIO12_DDR   DDRC

#define DIO13_PIN   PINC3
#define DIO13_RPORT PINC
#define DIO13_WPORT PORTC
#define DIO13_PWM   0
#define DIO13_DDR   DDRC

#define DIO14_PIN   PINC4
#define DIO14_RPORT PINC
#define DIO14_WPORT PORTC
#define DIO14_PWM   0 // OC3C
#define DIO14_DDR   DDRC

#define DIO15_PIN   PINC5
#define DIO15_RPORT PINC
#define DIO15_WPORT PORTC
#define DIO15_PWM   0 // OC3B
#define DIO15_DDR   DDRC

#define DIO16_PIN   PINC6
#define DIO16_RPORT PINC
#define DIO16_WPORT PORTC
#define DIO16_PWM   0 // OC3A
#define DIO16_DDR   DDRC

#define DIO17_PIN   PINC7
#define DIO17_RPORT PINC
#define DIO17_WPORT PORTC
#define DIO17_PWM   0
#define DIO17_DDR   DDRC

#define DIO18_PIN   PINE6
#define DIO18_RPORT PINE
#define DIO18_WPORT PORTE
#define DIO18_PWM   0
#define DIO18_DDR   DDRE

#define DIO19_PIN   PINE7
#define DIO19_RPORT PINE
#define DIO19_WPORT PORTE
#define DIO19_PWM   0
#define DIO19_DDR   DDRE

#define DIO20_PIN   PINB0
#define DIO20_RPORT PINB
#define DIO20_WPORT PORTB
#define DIO20_PWM   0
#define DIO20_DDR   DDRB

#define DIO21_PIN   PINB1
#define DIO21_RPORT PINB
#define DIO21_WPORT PORTB
#define DIO21_PWM   0
#define DIO21_DDR   DDRB

#define DIO22_PIN   PINB2
#define DIO22_RPORT PINB
#define DIO22_WPORT PORTB
#define DIO22_PWM   0
#define DIO22_DDR   DDRB

#define DIO23_PIN   PINB3
#define DIO23_RPORT PINB
#define DIO23_WPORT PORTB
#define DIO23_PWM   0
#define DIO23_DDR   DDRB

#define DIO24_PIN   PINB4
#define DIO24_RPORT PINB
#define DIO24_WPORT PORTB
#define DIO24_PWM   0 // OC2A
#define DIO24_DDR   DDRB

#define DIO25_PIN   PINB5
#define DIO25_RPORT PINB
#define DIO25_WPORT PORTB
#define DIO25_PWM   0 // OC1A
#define DIO25_DDR   DDRB

#define DIO26_PIN   PINB6
#define DIO26_RPORT PINB
#define DIO26_WPORT PORTB
#define DIO26_PWM   0 // OC1B
#define DIO26_DDR   DDRB

#define DIO27_PIN   PINB7
#define DIO27_RPORT PINB
#define DIO27_WPORT PORTB
#define DIO27_PWM   0 // OC1C
#define DIO27_DDR   DDRB

#define DIO28_PIN   PINA0
#define DIO28_RPORT PINA
#define DIO28_WPORT PORTA
#define DIO28_PWM   0
#define DIO28_DDR   DDRA

#define DIO29_PIN   PINA1
#define DIO29_RPORT PINA
#define DIO29_WPORT PORTA
#define DIO29_PWM   0
#define DIO29_DDR   DDRA

#define DIO30_PIN   PINA2
#define DIO30_RPORT PINA
#define DIO30_WPORT PORTA
#define DIO30_PWM   0
#define DIO30_DDR   DDRA

#define DIO31_PIN   PINA3
#define DIO31_RPORT PINA
#define DIO31_WPORT PORTA
#define DIO31_PWM   0
#define DIO31_DDR   DDRA

#define DIO32_PIN   PINA4
#define DIO32_RPORT PINA
#define DIO32_WPORT PORTA
#define DIO32_PWM   0
#define DIO32_DDR   DDRA

#define DIO33_PIN   PINA5
#define DIO33_RPORT PINA
#define DIO33_WPORT PORTA
#define DIO33_PWM   0
#define DIO33_DDR   DDRA

#define DIO34_PIN   PINA6
#define DIO34_RPORT PINA
#define DIO34_WPORT PORTA
#define DIO34_PWM   0
#define DIO34_DDR   DDRA

#define DIO35_PIN   PINA7
#define DIO35_RPORT PINA
#define DIO35_WPORT PORTA
#define DIO35_PWM   0
#define DIO35_DDR   DDRA

#define DIO36_PIN   PINE4
#define DIO36_RPORT PINE
#define DIO36_WPORT PORTE
#define DIO36_PWM   0
#define DIO36_DDR   DDRE

#define DIO37_PIN   PINE5
#define DIO37_RPORT PINE
#define DIO37_WPORT PORTE
#define DIO37_PWM   0
#define DIO37_DDR   DDRE

#define DIO38_PIN   PINF0
#define DIO38_RPORT PINF
#define DIO38_WPORT PORTF
#define DIO38_PWM   0
#define DIO38_DDR   DDRF

#define DIO39_PIN   PINF1
#define DIO39_RPORT PINF
#define DIO39_WPORT PORTF
#define DIO39_PWM   0
#define DIO39_DDR   DDRF

#define DIO40_PIN   PINF2
#define DIO40_RPORT PINF
#define DIO40_WPORT PORTF
#define DIO40_PWM   0
#define DIO40_DDR   DDRF

#define DIO41_PIN   PINF3
#define DIO41_RPORT PINF
#define DIO41_WPORT PORTF
#define DIO41_PWM   0
#define DIO41_DDR   DDRF

#define DIO42_PIN   PINF4
#define DIO42_RPORT PINF
#define DIO42_WPORT PORTF
#define DIO42_PWM   0
#define DIO42_DDR   DDRF

#define DIO43_PIN   PINF5
#define DIO43_RPORT PINF
#define DIO43_WPORT PORTF
#define DIO43_PWM   0
#define DIO43_DDR   DDRF

#define DIO44_PIN   PINF6
#define DIO44_RPORT PINF
#define DIO44_WPORT PORTF
#define DIO44_PWM   0
#define DIO44_DDR   DDRF

#define DIO45_PIN   PINF7
#define DIO45_RPORT PINF
#define DIO45_WPORT PORTF
#define DIO45_PWM   0
#define DIO45_DDR   DDRF

#define AIO0_PIN    PINF0
#define AIO0_RPORT  PINF
#define AIO0_WPORT  PORTF
#define AIO0_PWM    0
#define AIO0_DDR    DDRF

#define AIO1_PIN    PINF1
#define AIO1_RPORT  PINF
#define AIO1_WPORT  PORTF
#define AIO1_PWM    0
#define AIO1_DDR    DDRF

#define AIO2_PIN    PINF2
#define AIO2_RPORT  PINF
#define AIO2_WPORT  PORTF
#define AIO2_PWM    0
#define AIO2_DDR    DDRF

#define AIO3_PIN    PINF3
#define AIO3_RPORT  PINF
#define AIO3_WPORT  PORTF
#define AIO3_PWM    0
#define AIO3_DDR    DDRF

#define AIO4_PIN    PINF4
#define AIO4_RPORT  PINF
#define AIO4_WPORT  PORTF
#define AIO4_PWM    0
#define AIO4_DDR    DDRF

#define AIO5_PIN    PINF5
#define AIO5_RPORT  PINF
#define AIO5_WPORT  PORTF
#define AIO5_PWM    0
#define AIO5_DDR    DDRF

#define AIO6_PIN    PINF6
#define AIO6_RPORT  PINF
#define AIO6_WPORT  PORTF
#define AIO6_PWM    0
#define AIO6_DDR    DDRF

#define AIO7_PIN    PINF7
#define AIO7_RPORT  PINF
#define AIO7_WPORT  PORTF
#define AIO7_PWM    0
#define AIO7_DDR    DDRF

//-- Begin not supported by Teensyduino
//-- don't use Arduino functions on these pins pinMode/digitalWrite/etc
#define DIO46_PIN   PINE2
#define DIO46_RPORT PINE
#define DIO46_WPORT PORTE
#define DIO46_PWM   0
#define DIO46_DDR   DDRE

#define DIO47_PIN   PINE3
#define DIO47_RPORT PINE
#define DIO47_WPORT PORTE
#define DIO47_PWM   0
#define DIO47_DDR   DDRE

#define TEENSY_E2   46
#define TEENSY_E3   47

//-- end not supported by Teensyduino

#undef PA0
#define PA0_PIN     PINA0
#define PA0_RPORT   PINA
#define PA0_WPORT   PORTA
#define PA0_PWM     0
#define PA0_DDR     DDRA
#undef PA1
#define PA1_PIN     PINA1
#define PA1_RPORT   PINA
#define PA1_WPORT   PORTA
#define PA1_PWM     0
#define PA1_DDR     DDRA
#undef PA2
#define PA2_PIN     PINA2
#define PA2_RPORT   PINA
#define PA2_WPORT   PORTA
#define PA2_PWM     0
#define PA2_DDR     DDRA
#undef PA3
#define PA3_PIN     PINA3
#define PA3_RPORT   PINA
#define PA3_WPORT   PORTA
#define PA3_PWM     0
#define PA3_DDR     DDRA
#undef PA4
#define PA4_PIN     PINA4
#define PA4_RPORT   PINA
#define PA4_WPORT   PORTA
#define PA4_PWM     0
#define PA4_DDR     DDRA
#undef PA5
#define PA5_PIN     PINA5
#define PA5_RPORT   PINA
#define PA5_WPORT   PORTA
#define PA5_PWM     0
#define PA5_DDR     DDRA
#undef PA6
#define PA6_PIN     PINA6
#define PA6_RPORT   PINA
#define PA6_WPORT   PORTA
#define PA6_PWM     0
#define PA6_DDR     DDRA
#undef PA7
#define PA7_PIN     PINA7
#define PA7_RPORT   PINA
#define PA7_WPORT   PORTA
#define PA7_PWM     0
#define PA7_DDR     DDRA

#undef PB0
#define PB0_PIN     PINB0
#define PB0_RPORT   PINB
#define PB0_WPORT   PORTB
#define PB0_PWM     0
#define PB0_DDR     DDRB
#undef PB1
#define PB1_PIN     PINB1
#define PB1_RPORT   PINB
#define PB1_WPORT   PORTB
#define PB1_PWM     0
#define PB1_DDR     DDRB
#undef PB2
#define PB2_PIN     PINB2
#define PB2_RPORT   PINB
#define PB2_WPORT   PORTB
#define PB2_PWM     0
#define PB2_DDR     DDRB
#undef PB3
#define PB3_PIN     PINB3
#define PB3_RPORT   PINB
#define PB3_WPORT   PORTB
#define PB3_PWM     0
#define PB3_DDR     DDRB
#undef PB4
#define PB4_PIN     PINB4
#define PB4_RPORT   PINB
#define PB4_WPORT   PORTB
#define PB4_PWM     0
#define PB4_DDR     DDRB
#undef PB5
#define PB5_PIN     PINB5
#define PB5_RPORT   PINB
#define PB5_WPORT   PORTB
#define PB5_PWM     0
#define PB5_DDR     DDRB
#undef PB6
#define PB6_PIN     PINB6
#define PB6_RPORT   PINB
#define PB6_WPORT   PORTB
#define PB6_PWM     0
#define PB6_DDR     DDRB
#undef PB7
#define PB7_PIN     PINB7
#define PB7_RPORT   PINB
#define PB7_WPORT   PORTB
#define PB7_PWM     0
#define PB7_DDR     DDRB

#undef PC0
#define PC0_PIN     PINC0
#define PC0_RPORT   PINC
#define PC0_WPORT   PORTC
#define PC0_PWM     0
#define PC0_DDR     DDRC
#undef PC1
#define PC1_PIN     PINC1
#define PC1_RPORT   PINC
#define PC1_WPORT   PORTC
#define PC1_PWM     0
#define PC1_DDR     DDRC
#undef PC2
#define PC2_PIN     PINC2
#define PC2_RPORT   PINC
#define PC2_WPORT   PORTC
#define PC2_PWM     0
#define PC2_DDR     DDRC
#undef PC3
#define PC3_PIN     PINC3
#define PC3_RPORT   PINC
#define PC3_WPORT   PORTC
#define PC3_PWM     0
#define PC3_DDR     DDRC
#undef PC4
#define PC4_PIN     PINC4
#define PC4_RPORT   PINC
#define PC4_WPORT   PORTC
#define PC4_PWM     0
#define PC4_DDR     DDRC
#undef PC5
#define PC5_PIN     PINC5
#define PC5_RPORT   PINC
#define PC5_WPORT   PORTC
#define PC5_PWM     0
#define PC5_DDR     DDRC
#undef PC6
#define PC6_PIN     PINC6
#define PC6_RPORT   PINC
#define PC6_WPORT   PORTC
#define PC6_PWM     0
#define PC6_DDR     DDRC
#undef PC7
#define PC7_PIN     PINC7
#define PC7_RPORT   PINC
#define PC7_WPORT   PORTC
#define PC7_PWM     0
#define PC7_DDR     DDRC

#undef PD0
#define PD0_PIN     PIND0
#define PD0_RPORT   PIND
#define PD0_WPORT   PORTD
#define PD0_PWM     0 // OC0B
#define PD0_DDR     DDRD
#undef PD1
#define PD1_PIN     PIND1
#define PD1_RPORT   PIND
#define PD1_WPORT   PORTD
#define PD1_PWM     0 // OC2B
#define PD1_DDR     DDRD
#undef PD2
#define PD2_PIN     PIND2
#define PD2_RPORT   PIND
#define PD2_WPORT   PORTD
#define PD2_PWM     0
#define PD2_DDR     DDRD
#undef PD3
#define PD3_PIN     PIND3
#define PD3_RPORT   PIND
#define PD3_WPORT   PORTD
#define PD3_PWM     0
#define PD3_DDR     DDRD
#undef PD4
#define PD4_PIN     PIND4
#define PD4_RPORT   PIND
#define PD4_WPORT   PORTD
#define PD4_PWM     0
#define PD4_DDR     DDRD
#undef PD5
#define PD5_PIN     PIND5
#define PD5_RPORT   PIND
#define PD5_WPORT   PORTD
#define PD5_PWM     0
#define PD5_DDR     DDRD
#undef PD6
#define PD6_PIN     PIND6
#define PD6_RPORT   PIND
#define PD6_WPORT   PORTD
#define PD6_PWM     0
#define PD6_DDR     DDRD
#undef PD7
#define PD7_PIN     PIND7
#define PD7_RPORT   PIND
#define PD7_WPORT   PORTD
#define PD7_PWM     0
#define PD7_DDR     DDRD

#undef PE0
#define PE0_PIN     PINE0
#define PE0_RPORT   PINE
#define PE0_WPORT   PORTE
#define PE0_PWM     0
#define PE0_DDR     DDRE
#undef PE1
#define PE1_PIN     PINE1
#define PE1_RPORT   PINE
#define PE1_WPORT   PORTE
#define PE1_PWM     0
#define PE1_DDR     DDRE
#undef PE2
#define PE2_PIN     PINE2
#define PE2_RPORT   PINE
#define PE2_WPORT   PORTE
#define PE2_PWM     0
#define PE2_DDR     DDRE
#undef PE3
#define PE3_PIN     PINE3
#define PE3_RPORT   PINE
#define PE3_WPORT   PORTE
#define PE3_PWM     0
#define PE3_DDR     DDRE
#undef PE4
#define PE4_PIN     PINE4
#define PE4_RPORT   PINE
#define PE4_WPORT   PORTE
#define PE4_PWM     0
#define PE4_DDR     DDRE
#undef PE5
#define PE5_PIN     PINE5
#define PE5_RPORT   PINE
#define PE5_WPORT   PORTE
#define PE5_PWM     0
#define PE5_DDR     DDRE
#undef PE6
#define PE6_PIN     PINE6
#define PE6_RPORT   PINE
#define PE6_WPORT   PORTE
#define PE6_PWM     0
#define PE6_DDR     DDRE
#undef PE7
#define PE7_PIN     PINE7
#define PE7_RPORT   PINE
#define PE7_WPORT   PORTE
#define PE7_PWM     0
#define PE7_DDR     DDRE

#undef PF0
#define PF0_PIN     PINF0
#define PF0_RPORT   PINF
#define PF0_WPORT   PORTF
#define PF0_PWM     0
#define PF0_DDR     DDRF
#undef PF1
#define PF1_PIN     PINF1
#define PF1_RPORT   PINF
#define PF1_WPORT   PORTF
#define PF1_PWM     0
#define PF1_DDR     DDRF
#undef PF2
#define PF2_PIN     PINF2
#define PF2_RPORT   PINF
#define PF2_WPORT   PORTF
#define PF2_PWM     0
#define PF2_DDR     DDRF
#undef PF3
#define PF3_PIN     PINF3
#define PF3_RPORT   PINF
#define PF3_WPORT   PORTF
#define PF3_PWM     0
#define PF3_DDR     DDRF
#undef PF4
#define PF4_PIN     PINF4
#define PF4_RPORT   PINF
#define PF4_WPORT   PORTF
#define PF4_PWM     0
#define PF4_DDR     DDRF
#undef PF5
#define PF5_PIN     PINF5
#define PF5_RPORT   PINF
#define PF5_WPORT   PORTF
#define PF5_PWM     0
#define PF5_DDR     DDRF
#undef PF6
#define PF6_PIN     PINF6
#define PF6_RPORT   PINF
#define PF6_WPORT   PORTF
#define PF6_PWM     0
#define PF6_DDR     DDRF
#undef PF7
#define PF7_PIN     PINF7
#define PF7_RPORT   PINF
#define PF7_WPORT   PORTF
#define PF7_PWM     0
#define PF7_DDR     DDRF

/**
 * Some of the pin mapping functions of the Teensduino extension to the Arduino IDE
 * do not function the same as the other Arduino extensions.
 */

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
