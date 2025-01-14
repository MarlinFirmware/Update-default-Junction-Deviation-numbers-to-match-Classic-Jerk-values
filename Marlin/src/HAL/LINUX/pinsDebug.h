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
 * Pins Debugging for Linux Native
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

#define NUMBER_PINS_TOTAL   NUM_DIGITAL_PINS
#define MULTI_NAME_PAD      16 // space needed to be pretty if not first name assigned to a pin

#define getPinByIndex(x) pin_array[x].pin

#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)

// active ADC function/mode/code values for PINSEL registers
constexpr int8_t ADC_pin_mode(const pin_t) { return -1; }

// The pin and index are the same on this platform
bool getPinIsDigitalByIndex(const pin_t pin) {
  return (!isAnalogPin(pin) || get_pin_mode(pin) != ADC_pin_mode(pin));
}

#define isAnalogPin(P) (digitalPinToAnalogIndex(P) >= 0)

#define digitalRead_mod(P) digitalRead(P)

int8_t get_pin_mode(const pin_t pin) { return isValidPin(pin) ? 0 : -1; }

bool getValidPinMode(const pin_t pin) {
  const int8_t pin_mode = get_pin_mode(pin);
  if (pin_mode == -1 || pin_mode == ADC_pin_mode(pin)) // Invalid pin or active analog pin
    return false;

  return (Gpio::getMode(pin) != 0); // Input/output state
}

void printPinPWM(const pin_t) {}
bool pwm_status(const pin_t) { return false; }

void printPinPort(const pin_t) {}

#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3d "), P); SERIAL_ECHO(buffer); }while(0)

#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
