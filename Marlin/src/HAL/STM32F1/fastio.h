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
 * Fast I/O interfaces for STM32F1
 * These use GPIO functions instead of Direct Port Manipulation, as on AVR.
 */

#include <libmaple/gpio.h>

#define READ(IO)                (PIN_MAP[IO].gpio_device->regs->IDR & _BV32(PIN_MAP[IO].gpio_bit) ? HIGH : LOW)
#define WRITE(IO,V)             (PIN_MAP[IO].gpio_device->regs->BSRR = _BV32(PIN_MAP[IO].gpio_bit) << ((V) ? 0 : 16))
#define TOGGLE(IO)              TBI32(PIN_MAP[IO].gpio_device->regs->ODR, PIN_MAP[IO].gpio_bit)

#define _GET_MODE(IO)           gpio_get_mode(PIN_MAP[IO].gpio_device, PIN_MAP[IO].gpio_bit)
#define _SET_MODE(IO,M)         gpio_set_mode(PIN_MAP[IO].gpio_device, PIN_MAP[IO].gpio_bit, M)
#define _SET_OUTPUT(IO)         _SET_MODE(IO, GPIO_OUTPUT_PP)
#define _SET_OUTPUT_OD(IO)      _SET_MODE(IO, GPIO_OUTPUT_OD)

#define OUT_WRITE(IO,V)         do{ _SET_OUTPUT(IO); WRITE(IO,V); }while(0)
#define OUT_WRITE_OD(IO,V)      do{ _SET_OUTPUT_OD(IO); WRITE(IO,V); }while(0)

#define SET_INPUT(IO)           _SET_MODE(IO, GPIO_INPUT_FLOATING)
#define SET_INPUT_PULLUP(IO)    _SET_MODE(IO, GPIO_INPUT_PU)
#define SET_INPUT_PULLDOWN(IO)  _SET_MODE(IO, GPIO_INPUT_PD)
#define SET_OUTPUT(IO)          OUT_WRITE(IO, LOW)
#define SET_OUTPUT_OD(IO)       OUT_WRITE_OD(IO, LOW)
#define SET_PWM(IO)             pinMode(IO, PWM)    // do{ gpio_set_mode(PIN_MAP[pin].gpio_device, PIN_MAP[pin].gpio_bit, GPIO_AF_OUTPUT_PP); timer_set_mode(PIN_MAP[pin].timer_device, PIN_MAP[pin].timer_channel, TIMER_PWM); }while(0)
#define SET_PWM_OD(IO)          pinMode(IO, PWM_OPEN_DRAIN)

#define IS_INPUT(IO)            (_GET_MODE(IO) == GPIO_INPUT_FLOATING || _GET_MODE(IO) == GPIO_INPUT_ANALOG || _GET_MODE(IO) == GPIO_INPUT_PU || _GET_MODE(IO) == GPIO_INPUT_PD)
#define IS_OUTPUT(IO)           (_GET_MODE(IO) == GPIO_OUTPUT_PP || _GET_MODE(IO) == GPIO_OUTPUT_OD)

#define PWM_PIN(IO)             !!PIN_MAP[IO].timer_device

// digitalRead/Write wrappers
#define extDigitalRead(IO)      digitalRead(IO)
#define extDigitalWrite(IO,V)   digitalWrite(IO,V)

//
// Pins Definitions
//
#define PA0  0x00
#define PA1  0x01
#define PA2  0x02
#define PA3  0x03
#define PA4  0x04
#define PA5  0x05
#define PA6  0x06
#define PA7  0x07
#define PA8  0x08
#define PA9  0x09
#define PA10 0x0A
#define PA11 0x0B
#define PA12 0x0C
#define PA13 0x0D
#define PA14 0x0E
#define PA15 0x0F

#define PB0  0x10
#define PB1  0x11
#define PB2  0x12
#define PB3  0x13
#define PB4  0x14
#define PB5  0x15
#define PB6  0x16
#define PB7  0x17 // 36 pins (F103T)
#define PB8  0x18
#define PB9  0x19
#define PB10 0x1A
#define PB11 0x1B
#define PB12 0x1C
#define PB13 0x1D
#define PB14 0x1E
#define PB15 0x1F

#if defined(MCU_STM32F103CB) || defined(MCU_STM32F103C8)
  #define PC13 0x20
  #define PC14 0x21
  #define PC15 0x22
#else
  #define PC0  0x20
  #define PC1  0x21
  #define PC2  0x22
  #define PC3  0x23
  #define PC4  0x24
  #define PC5  0x25
  #define PC6  0x26
  #define PC7  0x27
  #define PC8  0x28
  #define PC9  0x29
  #define PC10 0x2A
  #define PC11 0x2B
  #define PC12 0x2C
  #define PC13 0x2D
  #define PC14 0x2E
  #define PC15 0x2F
#endif

#define PD0  0x30
#define PD1  0x31
#define PD2  0x32 // 64 pins (F103R)
#define PD3  0x33
#define PD4  0x34
#define PD5  0x35
#define PD6  0x36
#define PD7  0x37
#define PD8  0x38
#define PD9  0x39
#define PD10 0x3A
#define PD11 0x3B
#define PD12 0x3C
#define PD13 0x3D
#define PD14 0x3E
#define PD15 0x3F

#define PE0  0x40
#define PE1  0x41
#define PE2  0x42
#define PE3  0x43
#define PE4  0x44
#define PE5  0x45
#define PE6  0x46
#define PE7  0x47
#define PE8  0x48
#define PE9  0x49
#define PE10 0x4A
#define PE11 0x4B
#define PE12 0x4C
#define PE13 0x4D
#define PE14 0x4E
#define PE15 0x4F // 100 pins (F103V)

#define PF0  0x50
#define PF1  0x51
#define PF2  0x52
#define PF3  0x53
#define PF4  0x54
#define PF5  0x55
#define PF6  0x56
#define PF7  0x57
#define PF8  0x58
#define PF9  0x59
#define PF10 0x5A
#define PF11 0x5B
#define PF12 0x5C
#define PF13 0x5D
#define PF14 0x5E
#define PF15 0x5F

#define PG0  0x60
#define PG1  0x61
#define PG2  0x62
#define PG3  0x63
#define PG4  0x64
#define PG5  0x65
#define PG6  0x66
#define PG7  0x67
#define PG8  0x68
#define PG9  0x69
#define PG10 0x6A
#define PG11 0x6B
#define PG12 0x6C
#define PG13 0x6D
#define PG14 0x6E
#define PG15 0x6F // 144 pins (F103Z)
