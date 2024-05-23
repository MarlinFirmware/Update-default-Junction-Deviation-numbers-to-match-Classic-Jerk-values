/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#define PA0     0
#define PA1     1
#define PA2     2
#define PA3     3
#define PA4     4
#define PA5     5
#define PA6     6
#define PA7     7
#define PA8     8
#define PA9     9
#define PA10  10
#define PA11  11
#define PA12  12
#define PA13  13
#define PA14  14
#define PA15  15
#define PB0   16
#define PB1   17
#define PB2   18
#define PB3   19
#define PB4   20
#define PB5   21
#define PB6   22
#define PB7   23
#define PB8   24
#define PB9   25
#define PB10  26
#define PB11  27
#define PB12  28
#define PB13  29
#define PB14  30
#define PB15  31
#define PC0   32
#define PC1   33
#define PC2   34
#define PC3   35
#define PC4   36
#define PC5   37
#define PC6   38
#define PC7   39
#define PC8   40
#define PC9   41
#define PC10  42
#define PC11  43
#define PC12  44
#define PC13  45
#define PC14  46
#define PC15  47
#define PD0   48
#define PD1   49
#define PD2   50
#define PD3   51
#define PD4   52
#define PD5   53
#define PD6   54
#define PD7   55
#define PD8   56
#define PD9   57
#define PD10  58
#define PD11  59
#define PD12  60
#define PD13  61
#define PD14  62
#define PD15  63
#define PE0   64
#define PE1   65
#define PE2   66
#define PE3   67
#define PE4   68
#define PE5   69
#define PE6   70
#define PE7   71
#define PE8   72
#define PE9   73
#define PE10  74
#define PE11  75
#define PE12  76
#define PE13  77
#define PE14  78
#define PE15  79

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS        87
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       7
#define NUM_ANALOG_FIRST        80

//#define ADC_RESOLUTION          12

// PWM resolution
//#define PWM_RESOLUTION          12
#define PWM_FREQUENCY           20000 // >= 20 Khz => inaudible noise for fans
#define PWM_MAX_DUTY_CYCLE      255

// SPI Definitions
#define PIN_SPI_SS              PA4
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5

// I2C Definitions
#define PIN_WIRE_SDA            PB9
#define PIN_WIRE_SCL            PB8

// Timer Definitions
// Do not use timer used by PWM pin. See PinMap_PWM.
#define TIMER_TONE              TIM6  // TIMER_TONE must be defined in this file
#define TIMER_SERVO             TIM5  // TIMER_SERVO must be defined in this file
#define TIMER_SERIAL            TIM7  // TIMER_SERIAL must be defined in this file

// UART Definitions
//#define SERIAL_UART_INSTANCE    1 // Connected to EXP3 header
/* Enable Serial 3 */
#define HAVE_HWSERIAL1
#define HAVE_HWSERIAL3

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

/* HAL configuration */
#define HSE_VALUE               12000000U

#define FLASH_PAGE_SIZE         (4U * 1024U)

#ifdef __cplusplus
} // extern "C"
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
// These serial port names are intended to allow libraries and architecture-neutral
// sketches to automatically default to the correct port name for a particular type
// of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
// the first hardware serial port whose RX/TX pins are not dedicated to another use.
//
// SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
//
// SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
//
// SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
//
// SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
//
// SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
//                            pins are NOT connected to anything by default.
#define SERIAL_PORT_MONITOR     Serial
#define SERIAL_PORT_HARDWARE_OPEN  Serial
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
