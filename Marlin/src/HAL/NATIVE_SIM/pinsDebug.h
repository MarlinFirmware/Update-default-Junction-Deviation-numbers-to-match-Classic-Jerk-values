/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Pins Debugging for x86_64
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
#define isValidPin(P) (P >= 0 && P < pin_t(NUMBER_PINS_TOTAL))
#define isAnalogPin(P) (digitalPinToAnalogIndex(P) >= 0)
#define digitalRead_mod(P)  digitalRead(P)
#define getPinByIndex(x) pin_array[x].pin
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3d "), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
#define MULTI_NAME_PAD 16 // space needed to be pretty if not first name assigned to a pin

// Active ADC function/mode/code values for PINSEL registers
int8_t ADC_pin_mode(const pin_t);
int8_t get_pin_mode(const pin_t);
bool getValidPinMode(const pin_t);
bool getPinIsDigitalByIndex(const pin_t);
void printPinPort(const pin_t);
void printPinPWM(const pin_t);
bool pwm_status(const pin_t);
