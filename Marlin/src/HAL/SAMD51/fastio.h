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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */

/**
 * Fast IO functions for SAMD51
 */

#include "SAMD51.h"

/**
 * Utility functions
 */

#ifndef MASK
  #define MASK(PIN) _BV(PIN)
#endif

/**
 * Magic I/O routines
 *
 * Now you can simply SET_OUTPUT(IO); WRITE(IO, HIGH); WRITE(IO, LOW);
 */

// Read a pin
#define READ(IO)        ((PORT->Group[(EPortType)GET_SAMD_PORT(IO)].IN.reg & MASK(GET_SAMD_PIN(IO))) != 0)

// Write to a pin
#define WRITE(IO,V)     do{                                                     \
                          const EPortType port = (EPortType)GET_SAMD_PORT(IO);  \
                          const uint32_t mask = MASK(GET_SAMD_PIN(IO));         \
                                                                                \
                          if (V) PORT->Group[port].OUTSET.reg = mask;           \
                          else PORT->Group[port].OUTCLR.reg = mask;             \
                        }while(0)

// Toggle a pin
#define TOGGLE(IO)      PORT->Group[(EPortType)GET_SAMD_PORT(IO)].OUTTGL.reg = MASK(GET_SAMD_PIN(IO));

// Set pin as input
#define SET_INPUT(IO)           do{                                                                 \
                                  const EPortType port = (EPortType)GET_SAMD_PORT(IO);              \
                                  const uint32_t pin = GET_SAMD_PIN(IO);                            \
                                                                                                    \
                                  PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN);  \
                                  PORT->Group[port].DIRCLR.reg = MASK(pin);                         \
                                }while(0)
// Set pin as input with pullup
#define SET_INPUT_PULLUP(IO)    do{                                                                                       \
                                  const EPortType port = (EPortType)GET_SAMD_PORT(IO);                                    \
                                  const uint32_t pin = GET_SAMD_PIN(IO);                                                  \
                                  const uint32_t mask = MASK(pin);                                                        \
                                                                                                                          \
                                  PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);   \
                                  PORT->Group[port].DIRCLR.reg = mask;                                                    \
                                  PORT->Group[port].OUTSET.reg = mask;                                                    \
                                }while(0)
// Set pin as input with pulldown
#define SET_INPUT_PULLDOWN(IO)  do{                                                                                       \
                                  const EPortType port = (EPortType)GET_SAMD_PORT(IO);                                    \
                                  const uint32_t pin = GET_SAMD_PIN(IO);                                                  \
                                  const uint32_t mask = MASK(pin);                                                        \
                                                                                                                          \
                                  PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);   \
                                  PORT->Group[port].DIRCLR.reg = mask;                                                    \
                                  PORT->Group[port].OUTCLR.reg = mask;                                                    \
                                }while(0)
// Set pin as output (push pull)
#define SET_OUTPUT(IO)          do{                                                                 \
                                  const EPortType port = (EPortType)GET_SAMD_PORT(IO);              \
                                  const uint32_t pin = GET_SAMD_PIN(IO);                            \
                                                                                                    \
                                  PORT->Group[port].DIRSET.reg = MASK(pin);                         \
                                  PORT->Group[port].PINCFG[pin].reg = 0;                            \
                                }while(0)
// Set pin as output (open drain)
#define SET_OUTPUT_OD(IO)       do{                                                                   \
                                  const EPortType port = (EPortType)GET_SAMD_PORT(IO);                \
                                  const uint32_t pin = GET_SAMD_PIN(IO);                              \
                                                                                                      \
                                  PORT->Group[port].PINCFG[pin].reg = (uint8_t)(PORT_PINCFG_PULLEN);  \
                                  PORT->Group[port].DIRCLR.reg = MASK(pin);                           \
                                }while(0)
// Set pin as PWM (push pull)
#define SET_PWM                 SET_OUTPUT
// Set pin as PWM (open drain)
#define SET_PWM_OD              SET_OUTPUT_OD

// check if pin is an output
#define IS_OUTPUT(IO)            ((PORT->Group[(EPortType)GET_SAMD_PORT(IO)].DIR.reg & MASK(GET_SAMD_PIN(IO))) \
                                 || (PORT->Group[(EPortType)GET_SAMD_PORT(IO)].PINCFG[GET_SAMD_PIN(IO)].reg & (PORT_PINCFG_INEN | PORT_PINCFG_PULLEN)) == PORT_PINCFG_PULLEN)
// check if pin is an input
#define IS_INPUT(IO)            !IS_OUTPUT(IO)

// Shorthand
#define OUT_WRITE(IO,V)         do{ SET_OUTPUT(IO); WRITE(IO,V); }while(0)
#define OUT_WRITE_OD(IO,V)      do{ SET_OUTPUT_OD(IO); WRITE(IO,V); }while(0)

// digitalRead/Write wrappers
#define extDigitalRead(IO)      digitalRead(IO)
#define extDigitalWrite(IO,V)   digitalWrite(IO,V)

/**
 * Ports and functions
 * Added as necessary or if I feel like it- not a comprehensive list!
 */

#ifdef ADAFRUIT_GRAND_CENTRAL_M4

  /**
   * Adafruit Grand Central M4 has a lot of PWMs the availables are listed here.
   * Some of these share the same source and so can't be used in the same time
   */
  #define PWM_PIN(P)        (WITHIN(P, 2, 13) || WITHIN(P, 22, 23) || WITHIN(P, 44, 45) || P == 48)

  // Return fulfilled ADCx->INPUTCTRL.reg
  #define PIN_TO_INPUTCTRL(P)     (  (PIN_TO_AIN(P) == 0) ? ADC_INPUTCTRL_MUXPOS_AIN0   \
                                   : (PIN_TO_AIN(P) == 1) ? ADC_INPUTCTRL_MUXPOS_AIN1   \
                                   : (PIN_TO_AIN(P) == 2) ? ADC_INPUTCTRL_MUXPOS_AIN2   \
                                   : (PIN_TO_AIN(P) == 3) ? ADC_INPUTCTRL_MUXPOS_AIN3   \
                                   : (PIN_TO_AIN(P) == 4) ? ADC_INPUTCTRL_MUXPOS_AIN4   \
                                   : (PIN_TO_AIN(P) == 5) ? ADC_INPUTCTRL_MUXPOS_AIN5   \
                                   : (PIN_TO_AIN(P) == 6) ? ADC_INPUTCTRL_MUXPOS_AIN6   \
                                   : (PIN_TO_AIN(P) == 7) ? ADC_INPUTCTRL_MUXPOS_AIN7   \
                                   : (PIN_TO_AIN(P) == 8) ? ADC_INPUTCTRL_MUXPOS_AIN8   \
                                   : (PIN_TO_AIN(P) == 9) ? ADC_INPUTCTRL_MUXPOS_AIN9   \
                                   : (PIN_TO_AIN(P) == 10) ? ADC_INPUTCTRL_MUXPOS_AIN10 \
                                   : (PIN_TO_AIN(P) == 11) ? ADC_INPUTCTRL_MUXPOS_AIN11 \
                                   : (PIN_TO_AIN(P) == 12) ? ADC_INPUTCTRL_MUXPOS_AIN12 \
                                   : (PIN_TO_AIN(P) == 13) ? ADC_INPUTCTRL_MUXPOS_AIN13 \
                                   : (PIN_TO_AIN(P) == 14) ? ADC_INPUTCTRL_MUXPOS_AIN14 \
                                   : ADC_INPUTCTRL_MUXPOS_AIN15)

  #define ANAPIN_TO_SAMDPIN(P)    (  (P == 0) ? PIN_TO_SAMD_PIN(67)   \
                                   : (P == 1) ? PIN_TO_SAMD_PIN(68)   \
                                   : (P == 2) ? PIN_TO_SAMD_PIN(69)   \
                                   : (P == 3) ? PIN_TO_SAMD_PIN(70)   \
                                   : (P == 4) ? PIN_TO_SAMD_PIN(71)   \
                                   : (P == 5) ? PIN_TO_SAMD_PIN(72)   \
                                   : (P == 6) ? PIN_TO_SAMD_PIN(73)   \
                                   : (P == 7) ? PIN_TO_SAMD_PIN(74)   \
                                   : (P == 8) ? PIN_TO_SAMD_PIN(54)   \
                                   : (P == 9) ? PIN_TO_SAMD_PIN(55)   \
                                   : (P == 10) ? PIN_TO_SAMD_PIN(56)  \
                                   : (P == 11) ? PIN_TO_SAMD_PIN(57)  \
                                   : (P == 12) ? PIN_TO_SAMD_PIN(58)  \
                                   : (P == 13) ? PIN_TO_SAMD_PIN(59)  \
                                   : (P == 14) ? PIN_TO_SAMD_PIN(60)  \
                                   : (P == 15) ? PIN_TO_SAMD_PIN(61)  \
                                   : (P == 16) ? PIN_TO_SAMD_PIN(12)  \
                                   : (P == 17) ? PIN_TO_SAMD_PIN(13)  \
                                   : PIN_TO_SAMD_PIN(9))

  #define digitalPinToAnalogIndex(P) (WITHIN(P, 67, 74) ? (P) - 67 : WITHIN(P, 54, 61) ? 8 + (P) - 54 : WITHIN(P, 12, 13) ? 16 + (P) - 12 : P == 9 ? 18 : -1)

  /**
   * pins
   */

   // PORTA
  #define DIO67_PIN   PIN_PA02    // A0
  #define DIO59_PIN   PIN_PA04    // A13
  #define DIO68_PIN   PIN_PA05    // A1
  #define DIO60_PIN   PIN_PA06    // A14
  #define DIO61_PIN   PIN_PA07    // A15
  #define DIO26_PIN   PIN_PA12
  #define DIO27_PIN   PIN_PA13
  #define DIO28_PIN   PIN_PA14
  #define DIO23_PIN   PIN_PA15
  #define DIO37_PIN   PIN_PA16
  #define DIO36_PIN   PIN_PA17
  #define DIO35_PIN   PIN_PA18
  #define DIO34_PIN   PIN_PA19
  #define DIO33_PIN   PIN_PA20
  #define DIO32_PIN   PIN_PA21
  #define DIO31_PIN   PIN_PA22
  #define DIO30_PIN   PIN_PA23
  // PORTB
  #define DIO12_PIN   PIN_PB00    // A16
  #define DIO13_PIN   PIN_PB01    // A17
  #define DIO9_PIN    PIN_PB02    // A18
  #define DIO69_PIN   PIN_PB03    // A2
  #define DIO74_PIN   PIN_PB04    // A7
  #define DIO54_PIN   PIN_PB05    // A8
  #define DIO55_PIN   PIN_PB06    // A9
  #define DIO56_PIN   PIN_PB07    // A10
  #define DIO57_PIN   PIN_PB08    // A11
  #define DIO58_PIN   PIN_PB09    // A12
  #define DIO18_PIN   PIN_PB12
  #define DIO19_PIN   PIN_PB13
  #define DIO39_PIN   PIN_PB14
  #define DIO38_PIN   PIN_PB15
  #define DIO14_PIN   PIN_PB16
  #define DIO15_PIN   PIN_PB17
  #define DIO8_PIN    PIN_PB18
  #define DIO29_PIN   PIN_PB19
  #define DIO20_PIN   PIN_PB20
  #define DIO21_PIN   PIN_PB21
  #define DIO10_PIN   PIN_PB22
  #define DIO11_PIN   PIN_PB23
  #define DIO1_PIN    PIN_PB24
  #define DIO0_PIN    PIN_PB25
  #define DIO83_PIN   PIN_PB28    // SD_CS
  #define DIO95_PIN   PIN_PB31    // SD_CD
  // PORTC
  #define DIO70_PIN   PIN_PC00    // A3
  #define DIO71_PIN   PIN_PC01    // A4
  #define DIO72_PIN   PIN_PC02    // A5
  #define DIO73_PIN   PIN_PC03    // A6
  #define DIO48_PIN   PIN_PC04
  #define DIO49_PIN   PIN_PC05
  #define DIO46_PIN   PIN_PC06
  #define DIO47_PIN   PIN_PC07
  #define DIO45_PIN   PIN_PC10
  #define DIO44_PIN   PIN_PC11
  #define DIO41_PIN   PIN_PC12
  #define DIO40_PIN   PIN_PC13
  #define DIO43_PIN   PIN_PC14
  #define DIO42_PIN   PIN_PC15
  #define DIO25_PIN   PIN_PC16
  #define DIO24_PIN   PIN_PC17
  #define DIO2_PIN    PIN_PC18
  #define DIO3_PIN    PIN_PC19
  #define DIO4_PIN    PIN_PC20
  #define DIO5_PIN    PIN_PC21
  #define DIO16_PIN   PIN_PC22
  #define DIO17_PIN   PIN_PC23
  #define DIO88_PIN   PIN_PC24    // NEOPIXEL
  // PORTD
  #define DIO53_PIN   PIN_PD10
  #define DIO22_PIN   PIN_PD12
  #define DIO6_PIN    PIN_PD20
  #define DIO7_PIN    PIN_PD21

#endif // ADAFRUIT_GRAND_CENTRAL_M4
