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
 * Pins Debugging for Maple STM32F1
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

#ifndef BOARD_NR_GPIO_PINS // Only in MAPLE_STM32F1
  #error "Expected BOARD_NR_GPIO_PINS not found"
#endif

#include "fastio.h"

extern const stm32_pin_info PIN_MAP[BOARD_NR_GPIO_PINS];

#define NUM_DIGITAL_PINS BOARD_NR_GPIO_PINS
#define NUMBER_PINS_TOTAL BOARD_NR_GPIO_PINS
#define isValidPin(P) (P >= 0 && P < BOARD_NR_GPIO_PINS)
#define getPinByIndex(x) pin_t(pin_array[x].pin)
#define digitalRead_mod(P) extDigitalRead(P)
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3hd "), int16_t(P)); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define MULTI_NAME_PAD 21 // space needed to be pretty if not first name assigned to a pin

// pins that will cause hang/reset/disconnect in M43 Toggle and Watch utilities
#ifndef M43_NEVER_TOUCH
  #define M43_NEVER_TOUCH(Q) (Q >= 9 && Q <= 12) // SERIAL/USB pins PA9(TX) PA10(RX)
#endif

int8_t get_pin_mode(const pin_t pin) { return isValidPin(pin) ? _GET_MODE(pin) : -1; }

int8_t digitalPinToAnalogIndex(const pin_t pin) {
  if (!isValidPin(pin)) return -1;
  pin_t adc_channel = pin_t(PIN_MAP[pin].adc_channel);
  #ifdef NUM_ANALOG_INPUTS
    if (adc_channel >= NUM_ANALOG_INPUTS) adc_channel = (pin_t)ADCx;
  #endif
  return adc_channel;
}

bool isAnalogPin(const pin_t pin) {
  if (!isValidPin(pin)) return false;
  if (PIN_MAP[pin].adc_channel != ADCx) {
    #ifdef NUM_ANALOG_INPUTS
      if (PIN_MAP[pin].adc_channel >= NUM_ANALOG_INPUTS) return false;
    #endif
    return _GET_MODE(pin) == GPIO_INPUT_ANALOG && !M43_NEVER_TOUCH(pin);
  }
  return false;
}

bool getValidPinMode(const pin_t pin) {
  return isValidPin(pin) && !IS_INPUT(pin);
}

bool getPinIsDigitalByIndex(const int16_t index) {
  const pin_t pin = getPinByIndex(index);
  return (!isAnalogPin(pin)
    #ifdef NUM_ANALOG_INPUTS
      || PIN_MAP[pin].adc_channel >= NUM_ANALOG_INPUTS
    #endif
  );
}

#include "../../inc/MarlinConfig.h" // Allow pins/pins.h to set density

void printPinPWM(const pin_t pin) {
  if (PWM_PIN(pin)) {
    timer_dev * const tdev = PIN_MAP[pin].timer_device;
    const uint8_t channel = PIN_MAP[pin].timer_channel;
    const char num = (
      #if ANY(STM32_HIGH_DENSITY, STM32_XL_DENSITY)
        tdev == &timer8 ? '8' :
        tdev == &timer5 ? '5' :
      #endif
      tdev == &timer4 ? '4' :
      tdev == &timer3 ? '3' :
      tdev == &timer2 ? '2' :
      tdev == &timer1 ? '1' : '?'
    );
    char buffer[10];
    sprintf_P(buffer, PSTR(" TIM%c CH%c"), num, ('0' + channel));
    SERIAL_ECHO(buffer);
  }
}

bool pwm_status(const pin_t pin) { return PWM_PIN(pin); }

void printPinPort(const pin_t pin) {
  const char port = 'A' + char(pin >> 4); // pin div 16
  const int16_t gbit = PIN_MAP[pin].gpio_bit;
  char buffer[8];
  sprintf_P(buffer, PSTR("P%c%hd "), port, gbit);
  if (gbit < 10) SERIAL_CHAR(' ');
  SERIAL_ECHO(buffer);
}
