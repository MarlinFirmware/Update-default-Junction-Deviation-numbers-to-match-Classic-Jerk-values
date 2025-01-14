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
 * Pins Debugging for Atmel 8 bit AVR CPUs
 *
 *   - NUMBER_PINS_TOTAL
 *   - MULTI_NAME_PAD
 *   - getPinByIndex(index)
 *   - printPinNameByIndex(index)
 *   - getPinIsDigitalByIndex(index)
 *   - digitalPinToAnalogIndex(pin)
 *   - getValidPinMode(pin)
 *   - isValidPin(pin)
 *   - isAnalogPin(pin)
 *   - digitalRead_mod(pin)
 *   - pwm_status(pin)
 *   - printPinPWM(pin)
 *   - printPinPort(pin)
 *   - printPinNumber(pin)
 *   - printPinAnalog(pin)
 */

#include "../../inc/MarlinConfig.h"

#define NUMBER_PINS_TOTAL NUM_DIGITAL_PINS

#if MB(BQ_ZUM_MEGA_3D, MIGHTYBOARD_REVE, MINIRAMBO, SCOOVO_X9H, TRIGORILLA_14)
  #define AVR_ATmega2560_FAMILY_PLUS_70 1
#endif

#if AVR_AT90USB1286_FAMILY

  // Working with Teensyduino extension so need to re-define some things
  #include "pinsDebug_Teensyduino.h"
  // Can't use the "digitalPinToPort" function from the Teensyduino type IDEs
  // portModeRegister takes a different argument
  #define digitalPinToTimer_DEBUG(P) digitalPinToTimer(P)
  #define digitalPinToBitMask_DEBUG(P) digitalPinToBitMask(P)
  #define digitalPinToPort_DEBUG(P) digitalPinToPort(P)
  #define getValidPinMode(P) (*portModeRegister(P) & digitalPinToBitMask_DEBUG(P))

#elif AVR_ATmega2560_FAMILY_PLUS_70   // So we can access/display all the pins on boards using more than 70

  #include "pinsDebug_plus_70.h"
  #define digitalPinToTimer_DEBUG(P) digitalPinToTimer_plus_70(P)
  #define digitalPinToBitMask_DEBUG(P) digitalPinToBitMask_plus_70(P)
  #define digitalPinToPort_DEBUG(P) digitalPinToPort_plus_70(P)
  bool getValidPinMode(pin_t pin) {return *portModeRegister(digitalPinToPort_DEBUG(pin)) & digitalPinToBitMask_DEBUG(pin); }

#else

  #define digitalPinToTimer_DEBUG(P) digitalPinToTimer(P)
  #define digitalPinToBitMask_DEBUG(P) digitalPinToBitMask(P)
  #define digitalPinToPort_DEBUG(P) digitalPinToPort(P)
  bool getValidPinMode(pin_t pin) {return *portModeRegister(digitalPinToPort_DEBUG(pin)) & digitalPinToBitMask_DEBUG(pin); }
  #define getPinByIndex(x) pgm_read_byte(&pin_array[x].pin)

#endif

#define isValidPin(P) (P >= 0 && P < NUMBER_PINS_TOTAL)
#if AVR_ATmega1284_FAMILY
  #define isAnalogPin(P) WITHIN(P, analogInputToDigitalPin(7), analogInputToDigitalPin(0))
  #define digitalPinToAnalogIndex(P) int(isAnalogPin(P) ? (P) - analogInputToDigitalPin(7) : -1)
#else
  #define _ANALOG1(P) WITHIN(P, analogInputToDigitalPin(0), analogInputToDigitalPin(7))
  #define _ANALOG2(P) WITHIN(P, analogInputToDigitalPin(8), analogInputToDigitalPin(15))
  #define isAnalogPin(P) (_ANALOG1(P) || _ANALOG2(P))
  #define digitalPinToAnalogIndex(P) int(_ANALOG1(P) ? (P) - analogInputToDigitalPin(0) : _ANALOG2(P) ? (P) - analogInputToDigitalPin(8) + 8 : -1)
#endif
#define getPinByIndex(x) pgm_read_byte(&pin_array[x].pin)
#define MULTI_NAME_PAD 26 // space needed to be pretty if not first name assigned to a pin

void printPinNameByIndex(const uint8_t index) {
  PGM_P const name_mem_pointer = (PGM_P)pgm_read_ptr(&pin_array[index].name);
  for (uint8_t y = 0; y < MAX_NAME_LENGTH; ++y) {
    char temp_char = pgm_read_byte(name_mem_pointer + y);
    if (temp_char != 0)
      SERIAL_CHAR(temp_char);
    else {
      for (uint8_t i = 0; i < MAX_NAME_LENGTH - y; ++i) SERIAL_CHAR(' ');
      break;
    }
  }
}

#define getPinIsDigitalByIndex(x)   pgm_read_byte(&pin_array[x].is_digital)

#if defined(__AVR_ATmega1284P__)  // 1284 IDE extensions set this to the number of
  #undef NUM_DIGITAL_PINS         // digital only pins while all other CPUs have it
  #define NUM_DIGITAL_PINS 32     // set to digital only + digital/analog
#endif

#define PWM_PRINT(V) do{ sprintf_P(buffer, PSTR("PWM:  %4d"), V); SERIAL_ECHO(buffer); }while(0)
#define PWM_CASE(N,Z)                                           \
  case TIMER##N##Z:                                             \
    if (TCCR##N##A & (_BV(COM##N##Z##1) | _BV(COM##N##Z##0))) { \
      PWM_PRINT(OCR##N##Z);                                     \
      return true;                                              \
    } else return false

#define ABTEST(N) defined(TCCR##N##A) && defined(COM##N##A1)

/**
 * Print a pin's PWM status.
 * Return true if it's currently a PWM pin.
 */
bool pwm_status(const uint8_t pin) {
  char buffer[20];   // for the sprintf statements

  switch (digitalPinToTimer_DEBUG(pin)) {

    #if ABTEST(0)
      #ifdef TIMER0A
        #if !AVR_AT90USB1286_FAMILY  // not available in Teensyduino type IDEs
          PWM_CASE(0, A);
        #endif
      #endif
      PWM_CASE(0, B);
    #endif

    #if ABTEST(1)
      PWM_CASE(1, A);
      PWM_CASE(1, B);
      #if defined(COM1C1) && defined(TIMER1C)
        PWM_CASE(1, C);
      #endif
    #endif

    #if ABTEST(2)
      PWM_CASE(2, A);
      PWM_CASE(2, B);
    #endif

    #if ABTEST(3)
      PWM_CASE(3, A);
      PWM_CASE(3, B);
      #ifdef COM3C1
        PWM_CASE(3, C);
      #endif
    #endif

    #ifdef TCCR4A
      PWM_CASE(4, A);
      PWM_CASE(4, B);
      PWM_CASE(4, C);
    #endif

    #if ABTEST(5)
      PWM_CASE(5, A);
      PWM_CASE(5, B);
      PWM_CASE(5, C);
    #endif

    case NOT_ON_TIMER:
    default:
      return false;
  }
  SERIAL_ECHO_SP(2);
} // pwm_status

const volatile uint8_t* const PWM_other[][3] PROGMEM = {
    { &TCCR0A, &TCCR0B, &TIMSK0 },
    { &TCCR1A, &TCCR1B, &TIMSK1 },
  #if ABTEST(2)
    { &TCCR2A, &TCCR2B, &TIMSK2 },
  #endif
  #if ABTEST(3)
    { &TCCR3A, &TCCR3B, &TIMSK3 },
  #endif
  #ifdef TCCR4A
    { &TCCR4A, &TCCR4B, &TIMSK4 },
  #endif
  #if ABTEST(5)
    { &TCCR5A, &TCCR5B, &TIMSK5 },
  #endif
};

const volatile uint8_t* const PWM_OCR[][3] PROGMEM = {

  #ifdef TIMER0A
    { &OCR0A, &OCR0B, 0 },
  #else
    { 0, &OCR0B, 0 },
  #endif

  #if defined(COM1C1) && defined(TIMER1C)
   { (const uint8_t*)&OCR1A, (const uint8_t*)&OCR1B, (const uint8_t*)&OCR1C },
  #else
   { (const uint8_t*)&OCR1A, (const uint8_t*)&OCR1B, 0 },
  #endif

  #if ABTEST(2)
    { &OCR2A, &OCR2B, 0 },
  #endif

  #if ABTEST(3)
    #ifdef COM3C1
      { (const uint8_t*)&OCR3A, (const uint8_t*)&OCR3B, (const uint8_t*)&OCR3C },
    #else
      { (const uint8_t*)&OCR3A, (const uint8_t*)&OCR3B, 0 },
    #endif
  #endif

  #ifdef TCCR4A
    { (const uint8_t*)&OCR4A, (const uint8_t*)&OCR4B, (const uint8_t*)&OCR4C },
  #endif

  #if ABTEST(5)
    { (const uint8_t*)&OCR5A, (const uint8_t*)&OCR5B, (const uint8_t*)&OCR5C },
  #endif
};

#define TCCR_A(T)   pgm_read_word(&PWM_other[T][0])
#define TCCR_B(T)   pgm_read_word(&PWM_other[T][1])
#define TIMSK(T)    pgm_read_word(&PWM_other[T][2])
#define CS_0       0
#define CS_1       1
#define CS_2       2
#define WGM_0      0
#define WGM_1      1
#define WGM_2      3
#define WGM_3      4
#define TOIE       0

#define OCR_VAL(T, L)   pgm_read_word(&PWM_OCR[T][L])

void err_is_counter()     { SERIAL_ECHOPGM("   non-standard PWM mode"); }
void err_is_interrupt()   { SERIAL_ECHOPGM("   compare interrupt enabled"); }
void err_prob_interrupt() { SERIAL_ECHOPGM("   overflow interrupt enabled"); }
void print_is_also_tied() { SERIAL_ECHOPGM(" is also tied to this pin"); SERIAL_ECHO_SP(14); }

void com_print(const uint8_t N, const uint8_t Z) {
  const uint8_t *TCCRA = (uint8_t*)TCCR_A(N);
  SERIAL_ECHOPGM("    COM", AS_DIGIT(N));
  SERIAL_CHAR(Z);
  SERIAL_ECHOPGM(": ", int((*TCCRA >> (6 - Z * 2)) & 0x03));
}

void timer_prefix(uint8_t T, char L, uint8_t N) {  // T - timer    L - pwm  N - WGM bit layout
  char buffer[20];   // for the sprintf statements
  const uint8_t *TCCRB = (uint8_t*)TCCR_B(T),
                *TCCRA = (uint8_t*)TCCR_A(T);
  uint8_t WGM = (((*TCCRB & _BV(WGM_2)) >> 1) | (*TCCRA & (_BV(WGM_0) | _BV(WGM_1))));
  if (N == 4) WGM |= ((*TCCRB & _BV(WGM_3)) >> 1);

  SERIAL_ECHOPGM("    TIMER", AS_DIGIT(T));
  SERIAL_CHAR(L);
  SERIAL_ECHO_SP(3);

  if (N == 3) {
    const uint8_t *OCRVAL8 = (uint8_t*)OCR_VAL(T, L - 'A');
    PWM_PRINT(*OCRVAL8);
  }
  else {
    const uint16_t *OCRVAL16 = (uint16_t*)OCR_VAL(T, L - 'A');
    PWM_PRINT(*OCRVAL16);
  }
  SERIAL_ECHOPGM("    WGM: ", WGM);
  com_print(T,L);
  SERIAL_ECHOPGM("    CS: ", (*TCCRB & (_BV(CS_0) | _BV(CS_1) | _BV(CS_2)) ));
  SERIAL_ECHOPGM("    TCCR", AS_DIGIT(T), "A: ", *TCCRA);
  SERIAL_ECHOPGM("    TCCR", AS_DIGIT(T), "B: ", *TCCRB);

  const uint8_t *TMSK = (uint8_t*)TIMSK(T);
  SERIAL_ECHOPGM("    TIMSK", AS_DIGIT(T), ": ", *TMSK);

  const uint8_t OCIE = L - 'A' + 1;
  if (N == 3) { if (WGM == 0 || WGM == 2 || WGM ==  4 || WGM ==  6) err_is_counter(); }
  else        { if (WGM == 0 || WGM == 4 || WGM == 12 || WGM == 13) err_is_counter(); }
  if (TEST(*TMSK, OCIE)) err_is_interrupt();
  if (TEST(*TMSK, TOIE)) err_prob_interrupt();
}

void printPinPWM(const uint8_t pin) {
  switch (digitalPinToTimer_DEBUG(pin)) {

    #if ABTEST(0)
      #ifdef TIMER0A
        #if !AVR_AT90USB1286_FAMILY  // not available in Teensyduino type IDEs
          case TIMER0A: timer_prefix(0, 'A', 3); break;
        #endif
      #endif
      case TIMER0B: timer_prefix(0, 'B', 3); break;
    #endif

    #if ABTEST(1)
      case TIMER1A: timer_prefix(1, 'A', 4); break;
      case TIMER1B: timer_prefix(1, 'B', 4); break;
      #if defined(COM1C1) && defined(TIMER1C)
        case TIMER1C: timer_prefix(1, 'C', 4); break;
      #endif
    #endif

    #if ABTEST(2)
      case TIMER2A: timer_prefix(2, 'A', 3); break;
      case TIMER2B: timer_prefix(2, 'B', 3); break;
    #endif

    #if ABTEST(3)
      case TIMER3A: timer_prefix(3, 'A', 4); break;
      case TIMER3B: timer_prefix(3, 'B', 4); break;
      #ifdef COM3C1
        case TIMER3C: timer_prefix(3, 'C', 4); break;
      #endif
    #endif

    #ifdef TCCR4A
      case TIMER4A: timer_prefix(4, 'A', 4); break;
      case TIMER4B: timer_prefix(4, 'B', 4); break;
      case TIMER4C: timer_prefix(4, 'C', 4); break;
    #endif

    #if ABTEST(5)
      case TIMER5A: timer_prefix(5, 'A', 4); break;
      case TIMER5B: timer_prefix(5, 'B', 4); break;
      case TIMER5C: timer_prefix(5, 'C', 4); break;
    #endif

    case NOT_ON_TIMER: break;

  }
  SERIAL_ECHOPGM("  ");

  // on pins that have two PWMs, print info on second PWM
  #if AVR_ATmega2560_FAMILY || AVR_AT90USB1286_FAMILY
    // looking for port B7 - PWMs 0A and 1C
    if (digitalPinToPort_DEBUG(pin) == 'B' - 64 && 0x80 == digitalPinToBitMask_DEBUG(pin)) {
      #if !AVR_AT90USB1286_FAMILY
        SERIAL_ECHOPGM("\n .");
        SERIAL_ECHO_SP(18);
        SERIAL_ECHOPGM("TIMER1C");
        print_is_also_tied();
        timer_prefix(1, 'C', 4);
      #else
        SERIAL_ECHOPGM("\n .");
        SERIAL_ECHO_SP(18);
        SERIAL_ECHOPGM("TIMER0A");
        print_is_also_tied();
        timer_prefix(0, 'A', 3);
      #endif
    }
  #else
    UNUSED(print_is_also_tied);
  #endif
} // printPinPWM

#ifndef digitalRead_mod                   // Use Teensyduino's version of digitalRead - it doesn't disable the PWMs
  int digitalRead_mod(const pin_t pin) {  // same as digitalRead except the PWM stop section has been removed
    const uint8_t port = digitalPinToPort_DEBUG(pin);
    return (port != NOT_A_PIN) && (*portInputRegister(port) & digitalPinToBitMask_DEBUG(pin)) ? HIGH : LOW;
  }
#endif

void printPinPort(const pin_t pin) {   // print port number
  #ifdef digitalPinToPort_DEBUG
    uint8_t x;
    SERIAL_ECHOPGM("  Port: ");
    #if AVR_AT90USB1286_FAMILY
      x = (pin == 46 || pin == 47) ? 'E' : digitalPinToPort_DEBUG(pin) + 64;
    #else
      x = digitalPinToPort_DEBUG(pin) + 64;
    #endif
    SERIAL_CHAR(x);

    #if AVR_AT90USB1286_FAMILY
      if (pin == 46)
        x = '2';
      else if (pin == 47)
        x = '3';
      else {
        uint8_t temp = digitalPinToBitMask_DEBUG(pin);
        for (x = '0'; x < '9' && temp != 1; x++) temp >>= 1;
      }
    #else
      uint8_t temp = digitalPinToBitMask_DEBUG(pin);
      for (x = '0'; x < '9' && temp != 1; x++) temp >>= 1;
    #endif
    SERIAL_CHAR(x);
  #else
    SERIAL_ECHO_SP(10);
  #endif
}

#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3d "), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)

#undef ABTEST
