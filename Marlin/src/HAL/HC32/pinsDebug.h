/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Pins Debugging for HC32
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
#include "fastio.h"
#include <drivers/timera/timera_pwm.h>

#ifndef BOARD_NR_GPIO_PINS
  #error "Expected BOARD_NR_GPIO_PINS not found."
#endif

#define NUM_DIGITAL_PINS BOARD_NR_GPIO_PINS
#define NUMBER_PINS_TOTAL BOARD_NR_GPIO_PINS
#define isValidPin(P) IS_GPIO_PIN(P)

// Note: pin_array is defined in `Marlin/src/pins/pinsDebug.h`, and since this file is included
// after it, it is available in this file as well.
#define getPinByIndex(x) pin_t(pin_array[x].pin)
#define digitalRead_mod(P) extDigitalRead(P)

#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3hd "), int16_t(P)); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)

#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)

#define MULTI_NAME_PAD 21 // Space needed to be pretty if not first name assigned to a pin

//
// Pins that will cause a hang / reset / disconnect in M43 Toggle and Watch utils
//
#ifndef M43_NEVER_TOUCH
  // Don't touch any of the following pins:
  // - Host serial pins, and
  // - Pins that could be connected to oscillators (see datasheet, Table 2.1):
  //   - XTAL = PH0, PH1
  //   - XTAL32 = PC14, PC15
  #define IS_HOST_USART_PIN(Q) (Q == BOARD_USART2_TX_PIN || Q == BOARD_USART2_RX_PIN)
  #define IS_OSC_PIN(Q) (Q == PH0 || Q == PH1 || Q == PC14 || Q == PC15)

  #define M43_NEVER_TOUCH(Q) (IS_HOST_USART_PIN(Q) || IS_OSC_PIN(Q))
#endif

int8_t digitalPinToAnalogIndex(const pin_t pin) {
  if (!isValidPin(pin)) return -1;
  const int8_t adc_channel = int8_t(PIN_MAP[pin].adc_info.channel);
  return pin_t(adc_channel);
}

bool isAnalogPin(pin_t pin) {
  if (!isValidPin(pin)) return false;

  if (PIN_MAP[pin].adc_info.channel != ADC_PIN_INVALID)
    return _GET_MODE(pin) == INPUT_ANALOG && !M43_NEVER_TOUCH(pin);

  return false;
}

bool getValidPinMode(const pin_t pin) {
  return isValidPin(pin) && !IS_INPUT(pin);
}

bool getPinIsDigitalByIndex(const int16_t index) {
  const pin_t pin = getPinByIndex(index);
  return (!isAnalogPin(pin));
}

/**
 * @brief print pin PWM status
 * @return true if pin is currently a PWM pin, false otherwise
 */
bool pwm_status(const pin_t pin) {
  // Get timer assignment for pin
  timera_config_t *unit;
  en_timera_channel_t channel;
  en_port_func_t port_function;
  if (!timera_get_assignment(pin, unit, channel, port_function) || unit == nullptr) {
    // No pwm pin or no unit assigned
    return false;
  }

  // A pin that is PWM output is:
  // - Assigned to a timerA unit (tested above)
  // - Unit is initialized
  // - Channel is active
  // - PinMode is OUTPUT_PWM
  return timera_is_unit_initialized(unit) && timera_is_channel_active(unit, channel) && getPinMode(pin) == OUTPUT_PWM;
}

void printPinPWM(const pin_t pin) {
  // Get timer assignment for pin
  timera_config_t *unit;
  en_timera_channel_t channel;
  en_port_func_t port_function;
  if (!timera_get_assignment(pin, unit, channel, port_function) || unit == nullptr)
    return; // No pwm pin or no unit assigned

  // Print timer assignment of pin, eg. "TimerA1Ch2 Func4"
  SERIAL_ECHOPGM("TimerA", TIMERA_REG_TO_X(unit->peripheral.register_base),
                 "Ch", TIMERA_CHANNEL_TO_X(channel),
                 " Func", int(port_function));
  SERIAL_ECHO_SP(3); // 3 spaces

  // Print timer unit state, eg. "1/16 PERAR=1234" OR "N/A"
  if (timera_is_unit_initialized(unit)) {
    // Unit initialized, print
    // - Timer clock divider
    // - Timer period value (PERAR)
    const uint8_t clock_divider = timera_clk_div_to_n(unit->state.base_init->enClkDiv);
    const uint16_t period = TIMERA_GetPeriodValue(unit->peripheral.register_base);
    SERIAL_ECHOPGM("1/", clock_divider, " PERAR=", period);
  }
  else {
    // Unit not initialized
    SERIAL_ECHOPGM("N/A");
    return;
  }

  SERIAL_ECHO_SP(3); // 3 spaces

  // Print timer channel state, e.g. "CMPAR=1234" OR "N/A"
  if (timera_is_channel_active(unit, channel)) {
    // Channel active, print
    // - Channel compare value
    const uint16_t compare = TIMERA_GetCompareValue(unit->peripheral.register_base, channel);
    SERIAL_ECHOPGM("CMPAR=", compare);
  }
  else {
    // Channel inactive
    SERIAL_ECHOPGM("N/A");
  }
}

void printPinPort(pin_t pin) {
  const char port = 'A' + char(pin >> 4); // Pin div 16
  const int16_t gbit = PIN_MAP[pin].bit_pos;
  char buffer[8];
  sprintf_P(buffer, PSTR("P%c%hd "), port, gbit);
  if (gbit < 10) {
    SERIAL_CHAR(' ');
  }

  SERIAL_ECHO(buffer);
}
