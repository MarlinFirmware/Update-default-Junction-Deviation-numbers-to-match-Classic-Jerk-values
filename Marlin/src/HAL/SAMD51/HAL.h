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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */

#define CPU_32_BIT

#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"
#include "fastio.h"

#ifdef ADAFRUIT_GRAND_CENTRAL_M4
  #include "MarlinSerial_AGCM4.h"

  // Serial ports
  typedef ForwardSerial1Class< decltype(Serial) > DefaultSerial1;
  typedef ForwardSerial1Class< decltype(Serial1) > DefaultSerial2;
  typedef ForwardSerial1Class< decltype(Serial2) > DefaultSerial3;
  typedef ForwardSerial1Class< decltype(Serial3) > DefaultSerial4;
  typedef ForwardSerial1Class< decltype(Serial4) > DefaultSerial5;
  extern DefaultSerial1 MSerial0;
  extern DefaultSerial2 MSerial1;
  extern DefaultSerial3 MSerial2;
  extern DefaultSerial4 MSerial3;
  extern DefaultSerial5 MSerial4;

  #define __MSERIAL(X) MSerial##X
  #define _MSERIAL(X) __MSERIAL(X)
  #define MSERIAL(X) _MSERIAL(INCREMENT(X))

  #if SERIAL_PORT == -1
    #define MYSERIAL1 MSerial0
  #elif WITHIN(SERIAL_PORT, 0, 3)
    #define MYSERIAL1 MSERIAL(SERIAL_PORT)
  #else
    #error "SERIAL_PORT must be from 0 to 3. You can also use -1 if the board supports Native USB."
  #endif

  #ifdef SERIAL_PORT_2
    #if SERIAL_PORT_2 == -1
      #define MYSERIAL2 MSerial0
    #elif WITHIN(SERIAL_PORT_2, 0, 3)
      #define MYSERIAL2 MSERIAL(SERIAL_PORT_2)
    #else
      #error "SERIAL_PORT_2 must be from 0 to 3. You can also use -1 if the board supports Native USB."
    #endif
  #endif

  #ifdef SERIAL_PORT_3
    #if SERIAL_PORT_3 == -1
      #define MYSERIAL3 MSerial0
    #elif WITHIN(SERIAL_PORT_3, 0, 3)
      #define MYSERIAL3 MSERIAL(SERIAL_PORT_3)
    #else
      #error "SERIAL_PORT_3 must be from 0 to 3. You can also use -1 if the board supports Native USB."
    #endif
  #endif

  #ifdef MMU_SERIAL_PORT
    #if MMU_SERIAL_PORT == -1
      #define MMU_SERIAL MSerial0
    #elif WITHIN(MMU_SERIAL_PORT, 0, 3)
      #define MMU_SERIAL MSERIAL(MMU_SERIAL_PORT)
    #else
      #error "MMU_SERIAL_PORT must be from 0 to 3. You can also use -1 if the board supports Native USB."
    #endif
  #endif

  #ifdef LCD_SERIAL_PORT
    #if LCD_SERIAL_PORT == -1
      #define LCD_SERIAL MSerial0
    #elif WITHIN(LCD_SERIAL_PORT, 0, 3)
      #define LCD_SERIAL MSERIAL(LCD_SERIAL_PORT)
    #else
      #error "LCD_SERIAL_PORT must be from 0 to 3. You can also use -1 if the board supports Native USB."
    #endif
  #endif

#endif // ADAFRUIT_GRAND_CENTRAL_M4

typedef int8_t pin_t;

#define SHARED_SERVOS HAS_SERVOS  // Use shared/servos.cpp

class Servo;
typedef Servo hal_servo_t;

//
// Interrupts
//
#define CRITICAL_SECTION_START()  const bool irqon = !__get_PRIMASK(); __disable_irq()
#define CRITICAL_SECTION_END()    if (irqon) __enable_irq()

#define cli() __disable_irq() // Disable interrupts
#define sei() __enable_irq()  // Enable interrupts

//
// ADC
//

//#define HAL_ADC_FILTERED          // Disable Marlin's oversampling. The HAL filters ADC values.
#define HAL_ADC_VREF_MV   3300
#define HAL_ADC_RESOLUTION  10      // ... 12

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

//
// Tone
//
void tone(const pin_t _pin, const unsigned int frequency, const unsigned long duration=0);
void noTone(const pin_t _pin);

// ------------------------
// Class Utilities
// ------------------------

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

#ifdef __cplusplus
  extern "C" {
#endif

char *dtostrf(double __val, signed char __width, unsigned char __prec, char *__s);

extern "C" int freeMemory();

#ifdef __cplusplus
  }
#endif

#pragma GCC diagnostic pop

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
  static void isr_on()  { sei(); }
  static void isr_off() { cli(); }

  static void delay_ms(const int ms) { delay(ms); }

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

  static uint16_t adc_result;

  // Called by Temperature::init once at startup
  static void adc_init();

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const uint8_t ch) {}

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static void adc_start(const pin_t pin);

  // Is the ADC ready for reading?
  static bool adc_ready() { return true; }

  // The current value of the ADC register
  static uint16_t adc_value() { return adc_result; }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * No option to invert the duty cycle [default = false]
   * No option to change the scale of the provided value to enable finer PWM duty control [default = 255]
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t=255, const bool=false) {
    analogWrite(pin, v);
  }

private:
  static void dma_init();
};
