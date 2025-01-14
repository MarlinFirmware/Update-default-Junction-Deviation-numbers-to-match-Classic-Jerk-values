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
 * Pins Debugging for Teensy 3.5 (MK64FX512) and Teensy 3.6 (MK66FX1M0)
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

#define NUMBER_PINS_TOTAL NUM_DIGITAL_PINS
#define MULTI_NAME_PAD 16 // space needed to be pretty if not first name assigned to a pin

#define FTM0_CH0_PIN 22
#define FTM0_CH1_PIN 23
#define FTM0_CH2_PIN  9
#define FTM0_CH3_PIN 10
#define FTM0_CH4_PIN  6
#define FTM0_CH5_PIN 20
#define FTM0_CH6_PIN 21
#define FTM0_CH7_PIN  5
#define FTM1_CH0_PIN  3
#define FTM1_CH1_PIN  4
#define FTM2_CH0_PIN 29
#define FTM2_CH1_PIN 30
#define FTM3_CH0_PIN  2
#define FTM3_CH1_PIN 14
#define FTM3_CH2_PIN  7
#define FTM3_CH3_PIN  8
#define FTM3_CH4_PIN 35
#define FTM3_CH5_PIN 36
#define FTM3_CH6_PIN 37
#define FTM3_CH7_PIN 38
#ifdef __MK66FX1M0__ // Teensy3.6
  #define TPM1_CH0_PIN 16
  #define TPM1_CH1_PIN 17
#endif

#define getPinByIndex(x) pin_array[x].pin
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define getPinIsDigitalByIndex(x) pin_array[x].is_digital
#define digitalPinToAnalogIndex(P) int(P - analogInputToDigitalPin(0))
#define getValidPinMode(P) (isValidPin(P) && IS_OUTPUT(P))
#define isValidPin(P) (P >= 0 && P < pin_t(NUMBER_PINS_TOTAL))
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%02d"), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)

#define isAnalogPin(P) ((P) >= analogInputToDigitalPin(0) && (P) <= analogInputToDigitalPin(9)) || ((P) >= analogInputToDigitalPin(12) && (P) <= analogInputToDigitalPin(20))

void printAnalogPin(char buffer[], int8_t pin) {
  if (pin <= 23)      sprintf_P(buffer, PSTR("(A%2d)  "), int(pin - 14));
  else if (pin <= 39) sprintf_P(buffer, PSTR("(A%2d)  "), int(pin - 19));
}

void analog_pin_state(char buffer[], int8_t pin) {
  if (pin <= 23)      sprintf_P(buffer, PSTR("Analog in =% 5d"), analogRead(pin - 14));
  else if (pin <= 39) sprintf_P(buffer, PSTR("Analog in =% 5d"), analogRead(pin - 19));
}

#define PWM_PRINT(V) do{ sprintf_P(buffer, PSTR("PWM:  %4d"), 22); SERIAL_ECHO(buffer); }while(0)
#define FTM_CASE(N,Z) \
  case FTM##N##_CH##Z##_PIN: \
    if (FTM##N##_C##Z##V) { \
      PWM_PRINT(FTM##N##_C##Z##V); \
      return true; \
    } else return false

/**
 * Print a pin's PWM status.
 * Return true if it's currently a PWM pin.
 */
bool pwm_status(const int8_t pin) {
  char buffer[20];   // for the sprintf statements
  switch (pin) {
    FTM_CASE(0,0);
    FTM_CASE(0,1);
    FTM_CASE(0,2);
    FTM_CASE(0,3);
    FTM_CASE(0,4);
    FTM_CASE(0,5);
    FTM_CASE(0,6);
    FTM_CASE(0,7);
    FTM_CASE(1,0);
    FTM_CASE(1,1);
    FTM_CASE(2,0);
    FTM_CASE(2,1);
    FTM_CASE(3,0);
    FTM_CASE(3,1);
    FTM_CASE(3,2);
    FTM_CASE(3,3);
    FTM_CASE(3,4);
    FTM_CASE(3,5);
    FTM_CASE(3,6);
    FTM_CASE(3,7);

    case NOT_ON_TIMER:
    default:
      return false;
  }
  SERIAL_ECHOPGM("  ");
}

void printPinPWM(const pin_t) { /* TODO */ }

void printPinPort(const pin_t) {}
