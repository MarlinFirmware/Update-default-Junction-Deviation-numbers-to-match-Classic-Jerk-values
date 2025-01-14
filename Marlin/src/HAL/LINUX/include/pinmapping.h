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

#include "../../../inc/MarlinConfigPre.h"

#include <stdint.h>
#include "../hardware/Gpio.h"

typedef pin_type pin_t;

#define P_NC -1
constexpr uint16_t NUM_DIGITAL_PINS = Gpio::pin_count;
constexpr uint8_t NUM_ANALOG_INPUTS = 16;

#define HAL_SENSITIVE_PINS

constexpr uint8_t analog_offset = NUM_DIGITAL_PINS - NUM_ANALOG_INPUTS;

// Get the digital pin for an analog index
constexpr pin_t analogInputToDigitalPin(const int8_t a) {
  return (WITHIN(a, 0, NUM_ANALOG_INPUTS - 1) ? analog_offset + a : P_NC);
}

// Get the analog index for a digital pin
constexpr int8_t digitalPinToAnalogIndex(const pin_t pin) {
  return (WITHIN(pin, analog_offset, NUM_DIGITAL_PINS - 1) ? pin - analog_offset : P_NC);
}

// Return the index of a pin number
constexpr int16_t GET_PIN_MAP_INDEX(const pin_t pin) { return pin; }

// Test whether the pin is valid
constexpr bool isValidPin(const pin_t pin) { return WITHIN(pin, 0, NUM_DIGITAL_PINS - 1); }

// Test whether the pin is PWM
constexpr bool PWM_PIN(const pin_t) { return false; }

// Test whether the pin is interruptible
constexpr bool INTERRUPT_PIN(const pin_t) { return false; }

// Get the pin number at the given index
constexpr pin_t GET_PIN_MAP_PIN(const int16_t index) { return pin_t(index); }

// Parse a G-code word into a pin index
int16_t PARSED_PIN_INDEX(const char code, const int16_t dval);
