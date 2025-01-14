/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#error "PINS_DEBUGGING is not yet supported for ESP32!"

/**
 * Pins Debugging for ESP32
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

#define digitalRead_mod(P) extDigitalRead(P)
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%02d"), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
#define getPinByIndex(x) pin_array[x].pin
#define getPinIsDigitalByIndex(x) pin_array[x].is_digital
#define isValidPin(P) (P >= 0 && P < pin_t(NUMBER_PINS_TOTAL))
#define digitalPinToAnalogIndex(P) int(P - analogInputToDigitalPin(0))
#define isAnalogPin(P) WITHIN(P, pin_t(analogInputToDigitalPin(0)), pin_t(analogInputToDigitalPin(NUM_ANALOG_INPUTS - 1)))
bool pwm_status(const pin_t) { return false; }

void printPinPort(const pin_t) {}

static bool getValidPinMode(const pin_t pin) {
  return isValidPin(pin) && !IS_INPUT(pin);
}

void printPinPWM(const int32_t pin) {
  if (pwm_status(pin)) {
    //uint32_t chan = g_APinDescription[pin].ulPWMChannel TODO when fast pwm is operative;
    //SERIAL_ECHOPGM("PWM = ", duty);
  }
}
