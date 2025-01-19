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

#define CPU_32_BIT

#ifndef F_CPU
  #define F_CPU (XOSC_MHZ * 1000000UL)
#endif

#include "arduino_extras.h"
#include "../../core/macros.h"
#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"
//#include "Servo.h"
#include "watchdog.h"

#include "../../inc/MarlinConfigPre.h"

#include <stdint.h>

//
// Serial Ports
//

#include "MarlinSerial.h"

// ------------------------
// Defines
// ------------------------

/**
 * TODO: review this to return 1 for pins that are not analog input
 */
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) (p)
#endif

#define CRITICAL_SECTION_START()  uint32_t primask = __get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (!primask) __enable_irq()
#define cli() __disable_irq()
#define sei() __enable_irq()

// ------------------------
// Types
// ------------------------

template <bool, class L, class R> struct IFPIN { typedef R type; };
template <class L, class R> struct IFPIN<true, L, R> { typedef L type; };
typedef IFPIN<sizeof(pin_size_t) == 1, int8_t, int16_t>::type pin_t;

class libServo;
typedef libServo hal_servo_t;
#define PAUSE_SERVO_OUTPUT() libServo::pause_all_servos()
#define RESUME_SERVO_OUTPUT() libServo::resume_all_servos()

// ------------------------
// ADC
// ------------------------

#define HAL_ADC_VREF         3.3
#ifdef ADC_RESOLUTION
  #define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
  #define HAL_ADC_RESOLUTION 12
#endif
// ADC index 4 is the MCU temperature
#define HAL_ADC_MCU_TEMP_DUMMY_PIN 127

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

#ifndef PLATFORM_M997_SUPPORT
  #define PLATFORM_M997_SUPPORT
#endif
void flashFirmware(const int16_t);

// Maple Compatibility
typedef void (*systickCallback_t)(void);
void systick_attach_callback(systickCallback_t cb);
void HAL_SYSTICK_Callback();

extern volatile uint32_t systick_uptime_millis;

#define HAL_CAN_SET_PWM_FREQ   // This HAL supports PWM Frequency adjustment
#define PWM_FREQUENCY 1000     // Default PWM frequency when set_pwm_duty() is called without set_pwm_frequency()

// ------------------------
// Class Utilities
// ------------------------

int freeMemory();

// ------------------------
// MarlinHAL Class
// ------------------------

class MarlinHAL {
public:

  // Earliest possible init, before setup()
  MarlinHAL() {}

  // Watchdog
  static void watchdog_init()    IF_DISABLED(USE_WATCHDOG, {});
  static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});

  static void init();          // Called early in setup()
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return !__get_PRIMASK(); }
  static void isr_on()  { __enable_irq(); }
  static void isr_off() { __disable_irq(); }

  static void delay_ms(const int ms) { ::delay(ms); }

  // Tasks, called from idle()
  static void idletask() {}

  // Reset
  static uint8_t get_reset_source();
  static void clear_reset_source() {}

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  // Called by Temperature::init once at startup
  static void adc_init();

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const pin_t pin);

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static void adc_start(const pin_t pin);

  // This ADC runs a periodic task
  static void adc_exclusive_handler();

  // Is the ADC ready for reading?
  static volatile bool adc_has_result;
  static bool adc_ready() { return adc_has_result; }

  // The current value of the ADC register
  static uint16_t adc_value();

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * Optionally invert the duty cycle [default = false]
   * Optionally change the scale of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size=255, const bool invert=false);

  /**
   * Set the frequency of the timer for the given pin as close as
   * possible to the provided desired frequency. Internally calculate
   * the required waveform generation mode, prescaler, and resolution
   * values and set timer registers accordingly.
   * NOTE that the frequency is applied to all pins on the timer (Ex OC3A, OC3B and OC3B)
   * NOTE that there are limitations, particularly if using TIMER2. (see Configuration_adv.h -> FAST_PWM_FAN Settings)
   */
  static void set_pwm_frequency(const pin_t pin, const uint16_t f_desired);
};
