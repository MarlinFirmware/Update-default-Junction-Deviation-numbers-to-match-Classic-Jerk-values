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

#include "../inc/MarlinConfig.h"

#define MAX_NAME_LENGTH  39    // one place to specify the format of all the sources of names
                               // "-" left justify, "39" minimum width of name, pad with blanks

/**
 *  This routine minimizes RAM usage by creating a FLASH resident array to
 *  store the pin names, pin numbers and analog/digital flag.
 *
 *  Creating the array in FLASH is a two pass process.  The first pass puts the
 *  name strings into FLASH.  The second pass actually creates the array.
 *
 *  Both passes use the same pin list.  The list contains two macro names. The
 *  actual macro definitions are changed depending on which pass is being done.
 */

// first pass - put the name strings into FLASH

#define _ADD_PIN_2(PIN_NAME, ENTRY_NAME) static const char ENTRY_NAME[] PROGMEM = { PIN_NAME };
#define _ADD_PIN(PIN_NAME, COUNTER) _ADD_PIN_2(PIN_NAME, entry_NAME_##COUNTER)
#define REPORT_NAME_DIGITAL(COUNTER, NAME) _ADD_PIN(#NAME, COUNTER)
#define REPORT_NAME_ANALOG(COUNTER, NAME) _ADD_PIN(#NAME, COUNTER)

#include "pinsDebug_list.h"
#line 49

// manually add pins that have names that are macros which don't play well with these macros
#if ANY(AVR_ATmega2560_FAMILY, AVR_ATmega1284_FAMILY, ARDUINO_ARCH_SAM, TARGET_LPC1768)
  #if SERIAL_IN_USE(0)
    static const char RXD_NAME_0[] PROGMEM = { "RXD0" };
    static const char TXD_NAME_0[] PROGMEM = { "TXD0" };
  #endif
  #if SERIAL_IN_USE(1)
    static const char RXD_NAME_1[] PROGMEM = { "RXD1" };
    static const char TXD_NAME_1[] PROGMEM = { "TXD1" };
  #endif
  #if SERIAL_IN_USE(2)
    static const char RXD_NAME_2[] PROGMEM = { "RXD2" };
    static const char TXD_NAME_2[] PROGMEM = { "TXD2" };
  #endif
  #if SERIAL_IN_USE(3)
    static const char RXD_NAME_3[] PROGMEM = { "RXD3" };
    static const char TXD_NAME_3[] PROGMEM = { "TXD3" };
  #endif
#endif

/////////////////////////////////////////////////////////////////////////////

// second pass - create the array

#undef _ADD_PIN_2
#undef _ADD_PIN
#undef REPORT_NAME_DIGITAL
#undef REPORT_NAME_ANALOG

#define _ADD_PIN_2(ENTRY_NAME, NAME, IS_DIGITAL) { ENTRY_NAME, NAME, IS_DIGITAL },
#define _ADD_PIN(NAME, COUNTER, IS_DIGITAL) _ADD_PIN_2(entry_NAME_##COUNTER, NAME, IS_DIGITAL)
#define REPORT_NAME_DIGITAL(COUNTER, NAME) _ADD_PIN(NAME, COUNTER, true)
#define REPORT_NAME_ANALOG(COUNTER, NAME) _ADD_PIN(analogInputToDigitalPin(NAME), COUNTER, false)

typedef struct {
  PGM_P const name;
  pin_t pin;
  bool is_digital;
} PinInfo;

const PinInfo pin_array[] PROGMEM = {

  /**
   *  [pin name]  [pin number]  [is digital or analog]  1 = digital, 0 = analog
   *  Each entry takes up 6 bytes in FLASH:
   *     2 byte pointer to location of the name string
   *     2 bytes containing the pin number
   *         analog pin numbers were converted to digital when the array was created
   *     2 bytes containing the digital/analog bool flag
   */

  #if SERIAL_IN_USE(0)
    #if ANY(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_0, 0, true },
      { TXD_NAME_0, 1, true },
    #elif AVR_ATmega1284_FAMILY
      { RXD_NAME_0, 8, true },
      { TXD_NAME_0, 9, true },
    #elif defined(TARGET_LPC1768)           // TX P0_02  RX P0_03
      { RXD_NAME_0, 3, true },
      { TXD_NAME_0, 2, true },
    #endif
  #endif

  #if SERIAL_IN_USE(1)
    #if ANY(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_1, 19, true },
      { TXD_NAME_1, 18, true },
    #elif AVR_ATmega1284_FAMILY
      { RXD_NAME_1, 10, true },
      { TXD_NAME_1, 11, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART1_P2_00         // TX P2_00  RX P2_01
        { RXD_NAME_1, 0x41, true },
        { TXD_NAME_1, 0x40, true },
      #else                                 // TX P0_15  RX P0_16
        { RXD_NAME_1, 16, true },
        { TXD_NAME_1, 15, true },
      #endif
    #endif
  #endif

  #if SERIAL_IN_USE(2)
    #if ANY(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_2, 17, true },
      { TXD_NAME_2, 16, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART2_P2_08         // TX P2_08  RX P2_09
        { RXD_NAME_2, 0x49, true },
        { TXD_NAME_2, 0x48, true },
      #else                                 // TX P0_10  RX P0_11
        { RXD_NAME_2, 11, true },
        { TXD_NAME_2, 10, true },
      #endif
    #endif
  #endif

  #if SERIAL_IN_USE(3)
    #if ANY(AVR_ATmega2560_FAMILY, ARDUINO_ARCH_SAM)
      { RXD_NAME_3, 15, true },
      { TXD_NAME_3, 14, true },
    #elif defined(TARGET_LPC1768)
      #ifdef LPC_PINCFG_UART3_P0_25         // TX P0_25  RX P0_26
        { RXD_NAME_3, 0x1A, true },
        { TXD_NAME_3, 0x19, true },
      #elif defined(LPC_PINCFG_UART3_P4_28) // TX P4_28  RX P4_29
        { RXD_NAME_3, 0x9D, true },
        { TXD_NAME_3, 0x9C, true },
      #else                                 // TX P0_00  RX P0_01
        { RXD_NAME_3, 1, true },
        { TXD_NAME_3, 0, true },
      #endif
    #endif
  #endif

  #include "pinsDebug_list.h"
  #line 168

};

#include HAL_PATH(.., pinsDebug.h)  // get the correct support file for this CPU

#ifndef M43_NEVER_TOUCH
  #define M43_NEVER_TOUCH(Q) false
#endif

bool pin_is_protected(const pin_t pin);

static void printPinIOState(const bool isout) {
  SERIAL_ECHO(isout ? F("Output ") : F("Input  "));
}

static void printPinState(const bool state) {
  SERIAL_ECHO(state ? F("HIGH") : F("LOW"));
}

// pretty report with PWM info
inline void printPinStateExt(const pin_t pin, const bool ignore, const bool extended=false, FSTR_P const start_string=nullptr) {
  char buffer[MAX_NAME_LENGTH + 1];   // for the sprintf statements
  bool found = false, multi_name_pin = false;

  auto alt_pin_echo = [](const pin_t &pin) {
    #if AVR_AT90USB1286_FAMILY
      // Use FastIO for pins Teensy doesn't expose
      if (pin == 46) {
        printPinIOState(IS_OUTPUT(46));
        printPinState(READ(46));
        return false;
      }
      else if (pin == 47) {
        printPinIOState(IS_OUTPUT(47));
        printPinState(READ(47));
        return false;
      }
    #endif
    return true;
  };

  for (uint8_t x = 0; x < COUNT(pin_array); ++x)  {    // scan entire array and report all instances of this pin
    if (getPinByIndex(x) == pin) {
      if (!found) {    // report digital and analog pin number only on the first time through
        if (start_string) SERIAL_ECHO(start_string);
        SERIAL_ECHOPGM("PIN: ");
        printPinNumber(pin);
        printPinPort(pin);
        if (int8_t(digitalPinToAnalogIndex(pin)) >= 0) printPinAnalog(pin); // analog pin number
        else SERIAL_ECHO_SP(8);                                                 // add padding if not an analog pin
      }
      else {
        SERIAL_CHAR('.');
        SERIAL_ECHO_SP(MULTI_NAME_PAD + (start_string ? strlen_P(FTOP(start_string)) : 0));  // add padding if not the first instance found
      }
      printPinNameByIndex(x);
      if (extended) {
        if (pin_is_protected(pin) && !ignore)
          SERIAL_ECHOPGM("protected ");
        else {
          if (alt_pin_echo(pin)) {
            if (!getPinIsDigitalByIndex(x)) {
              sprintf_P(buffer, PSTR("Analog in = %5ld"), (long)analogRead(digitalPinToAnalogIndex(pin)));
              SERIAL_ECHO(buffer);
            }
            else {
              if (!getValidPinMode(pin)) {
                //pinMode(pin, INPUT_PULLUP);  // make sure input isn't floating - stopped doing this
                                               // because this could interfere with inductive/capacitive
                                               // sensors (high impedance voltage divider) and with Pt100 amplifier
                printPinIOState(false);
                printPinState(digitalRead_mod(pin));
              }
              else if (pwm_status(pin)) {
                // do nothing
              }
              else {
                printPinIOState(true);
                printPinState(digitalRead_mod(pin));
              }
            }
            if (!multi_name_pin && extended) printPinPWM(pin);  // report PWM capabilities only on the first pass & only if doing an extended report
          }
        }
      }
      SERIAL_EOL();
      multi_name_pin = found;
      found = true;
    }  // end of IF
  } // end of for loop

  if (!found) {
    if (start_string) SERIAL_ECHO(start_string);
    SERIAL_ECHOPGM("PIN: ");
    printPinNumber(pin);
    printPinPort(pin);
    if (int8_t(digitalPinToAnalogIndex(pin)) >= 0) printPinAnalog(pin); // analog pin number
    else SERIAL_ECHO_SP(8);                                                 // add padding if not an analog pin
    SERIAL_ECHOPGM("<unused/unknown>");
    if (extended) {

      if (alt_pin_echo(pin)) {
        if (pwm_status(pin)) {
          // do nothing
        }
        else if (getValidPinMode(pin)) {
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);
          printPinIOState(true);
          printPinState(digitalRead_mod(pin));
        }
        else {
          if (isAnalogPin(pin)) {
            sprintf_P(buffer, PSTR("   Analog in = %5ld"), (long)analogRead(digitalPinToAnalogIndex(pin)));
            SERIAL_ECHO(buffer);
            SERIAL_ECHOPGM("   ");
          }
          else
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);   // add padding if not an analog pin

          printPinIOState(false);
          printPinState(digitalRead_mod(pin));
        }
        //if (!pwm_status(pin)) SERIAL_CHAR(' ');    // add padding if it's not a PWM pin
        if (extended) {
          SERIAL_ECHO_SP(MAX_NAME_LENGTH - 16);
          printPinPWM(pin);  // report PWM capabilities only if doing an extended report
        }
      }
    }
    SERIAL_EOL();
  }
}
