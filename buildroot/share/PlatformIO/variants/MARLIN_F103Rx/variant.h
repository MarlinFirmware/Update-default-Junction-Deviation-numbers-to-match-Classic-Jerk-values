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


// *   = F103R8-B-C-D-E-F-G
// **  = F103RC-D-E-F-G
//                      | DIGITAL | ANALOG         | USART                    | TWI       | SPI                   | SPECIAL   |
//                      |---------|----------------|--------------------------|-----------|-----------------------|-----------|
#define PA0  PIN_A0  // | 0       | A0             |                          |           |                       |           |
#define PA1  PIN_A1  // | 1       | A1             |                          |           |                       |           |
#define PA2  PIN_A2  // | 2       | A2             | USART2_TX                |           |                       |           |
#define PA3  PIN_A3  // | 2       | A3, DAC_OUT1** | USART2_RX                |           |                       |           |
#define PA4  PIN_A4  // | 4       | A4, DAC_OUT2** |                          |           | SPI1_SS               |           |
#define PA5  PIN_A5  // | 5       | A5             |                          |           | SPI1_SCK              |           |
#define PA6  PIN_A6  // | 6       | A6             |                          |           | SPI1_MISO             |           |
#define PA7  PIN_A7  // | 7       | A7             |                          |           | SPI1_MOSI             |           |
#define PA8  8       // | 8       |                |                          |           |                       |           |
#define PA9  9       // | 9       |                | USART1_TX                |           |                       |           |
#define PA10 10      // | 10      |                | USART1_RX                |           |                       |           |
#define PA11 11      // | 11      |                |                          |           |                       | USB_DM    |
#define PA12 12      // | 12      |                |                          |           |                       | USB_DP    |
#define PA13 13      // | 13      |                |                          |           |                       | SWD_SWDIO |
#define PA14 14      // | 14      |                |                          |           |                       | SWD_SWCLK |
#define PA15 15      // | 15      |                |                          |           | SPI1_SS/SPI3_SS**     |           |
//                      |---------|----------------|--------------------------|-----------|-----------------------|-----------|
#define PB0  PIN_A8  // | 16      | A8             |                          |           |                       |           |
#define PB1  PIN_A9  // | 17      | A9             |                          |           |                       |           |
#define PB2  18      // | 18      |                |                          |           |                       | BOOT1     |
#define PB3  19      // | 19      |                |                          |           | SPI1_SCK/SPI3_SCK**   |           |
#define PB4  20      // | 20      |                |                          |           | SPI1_MISO/SPI3_MISO** |           |
#define PB5  21      // | 21      |                |                          |           | SPI1_MOSI/SPI3_MOSI** |           |
#define PB6  22      // | 22      |                | USART1_TX                | TWI1_SCL  |                       |           |
#define PB7  23      // | 23      |                | USART1_RX                | TWI1_SDA  |                       |           |
#define PB8  24      // | 24      |                |                          | TWI1_SCL  |                       |           |
#define PB9  25      // | 25      |                |                          | TWI1_SDA  |                       |           |
#define PB10 26      // | 26      |                | USART3_TX*               | TWI2_SCL* |                       |           |
#define PB11 27      // | 27      |                | USART3_RX*               | TWI2_SDA* |                       |           |
#define PB12 28      // | 28      |                |                          |           | SPI2_SS*              |           |
#define PB13 29      // | 29      |                |                          |           | SPI2_SCK*             |           |
#define PB14 30      // | 30      |                |                          |           | SPI2_MISO*            |           |
#define PB15 31      // | 31      |                |                          |           | SPI2_MOSI*            |           |
//                      |---------|----------------|--------------------------|-----------|-----------------------|-----------|
#define PC0  PIN_A10 // | 32      | A10            |                          |           |                       |           |
#define PC1  PIN_A11 // | 33      | A11            |                          |           |                       |           |
#define PC2  PIN_A12 // | 34      | A12            |                          |           |                       |           |
#define PC3  PIN_A13 // | 35      | A13            |                          |           |                       |           |
#define PC4  PIN_A14 // | 36      | A14            |                          |           |                       |           |
#define PC5  PIN_A15 // | 37      | A15            |                          |           |                       |           |
#define PC6  38      // | 38      |                |                          |           |                       |           |
#define PC7  39      // | 39      |                |                          |           |                       |           |
#define PC8  40      // | 40      |                |                          |           |                       |           |
#define PC9  41      // | 41      |                |                          |           |                       |           |
#define PC10 42      // | 42      |                | USART3_TX*/UART4_TX**    |           |                       |           |
#define PC11 43      // | 43      |                | USART3_RX*/UART4_RX**    |           |                       |           |
#define PC12 44      // | 44      |                | UART5_TX**               |           |                       |           |
#define PC13 45      // | 45      |                |                          |           |                       |           |
#define PC14 46      // | 46      |                |                          |           |                       | OSC32_IN  |
#define PC15 47      // | 47      |                |                          |           |                       | OSC32_OUT |
//                      |---------|----------------|--------------------------|-----------|-----------------------|-----------|
#define PD0  48      // | 48      |                |                          |           |                       | OSC_IN    |
#define PD1  49      // | 48      |                |                          |           |                       | OSC_OUT   |
#define PD2  50      // | 50      |                | UART5_RX**               |           |                       |           |
//                      |---------|----------------|--------------------------|-----------|-----------------------|-----------|

// This must be a literal
#define NUM_DIGITAL_PINS        51
// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       16

// On-board LED pin number
#ifndef LED_BUILTIN
#define LED_BUILTIN             PB11
#endif
#define LED_GREEN               LED_BUILTIN

// On-board user button
#ifndef USER_BTN
#define USER_BTN                PC13
#endif

// Override default Arduino configuration

// SPI Definitions
#if DEFAULT_SPI == 3
  #define PIN_SPI_SS            PA15
  #define PIN_SPI_MOSI          PB5
  #define PIN_SPI_MISO          PB4
  #define PIN_SPI_SCK           PB3
#elif DEFAULT_SPI == 2
  #define PIN_SPI_SS            PB12
  #define PIN_SPI_MOSI          PB15
  #define PIN_SPI_MISO          PB14
  #define PIN_SPI_SCK           PB13
#else
  #define PIN_SPI_SS            PA4
  #define PIN_SPI_MOSI          PA7
  #define PIN_SPI_MISO          PA6
  #define PIN_SPI_SCK           PA5
#endif

// I2C Definitions
#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

// Timer Definitions
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM3  // TIMER_TONE must be defined in this file
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM2  // TIMER_SERVO must be defined in this file
#endif

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE    1

// Default pin used for 'Serial1' instance
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

// Extra HAL modules
#if defined(STM32F103xE) || defined(STM32F103xG)
  #ifndef HAL_DAC_MODULE_ENABLED
    #define HAL_DAC_MODULE_ENABLED
  #endif
#endif

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
  #define SERIAL_PORT_HARDWARE    Serial1
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
