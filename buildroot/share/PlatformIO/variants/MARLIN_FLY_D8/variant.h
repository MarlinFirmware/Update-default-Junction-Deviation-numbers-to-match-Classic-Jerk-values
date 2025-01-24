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

                      // | DIGITAL | ANALOG IN  | ANALOG OUT | UART/USART            | TWI                  | SPI                               | SPECIAL   |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PA0  0        // | 0       | A0 (ADC1)  |            | UART4_TX              |                      |                                   |           |
#define PA1  1        // | 1       | A1 (ADC1)  |            | UART4_RX              |                      |                                   |           |
#define PA2  2        // | 2       | A2 (ADC1)  |            | USART2_TX             |                      |                                   |           |
#define PA3  3        // | 3       | A3 (ADC1)  |            | USART2_RX             |                      |                                   |           |
#define PA4  4        // | 4       | A4 (ADC1)  | DAC_OUT1   |                       |                      | SPI1_SS, (SPI3_SS)                |           |
#define PA5  5        // | 5       | A5 (ADC1)  | DAC_OUT2   |                       |                      | SPI1_SCK                          |           |
#define PA6  6        // | 6       | A6 (ADC1)  |            |                       |                      | SPI1_MISO                         |           |
#define PA7  7        // | 7       | A7 (ADC1)  |            |                       |                      | SPI1_MOSI                         |           |
#define PA8  8        // | 8       |            |            |                       | TWI3_SCL             |                                   |           |
#define PA9  9        // | 9       |            |            | USART1_TX             |                      |                                   |           |
#define PA10 10       // | 10      |            |            | USART1_RX             |                      |                                   |           |
#define PA11 11       // | 11      |            |            |                       |                      |                                   |           |
#define PA12 12       // | 12      |            |            |                       |                      |                                   |           |
#define PA13 13       // | 13      |            |            |                       |                      |                                   | SWD_SWDIO |
#define PA14 14       // | 14      |            |            |                       |                      |                                   | SWD_SWCLK |
#define PA15 15       // | 15      |            |            |                       |                      | SPI3_SS, (SPI1_SS)                |           |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PB0  16       // | 16      | A8 (ADC1)  |            |                       |                      |                                   |           |
#define PB1  17       // | 17      | A9 (ADC1)  |            |                       |                      |                                   |           |
#define PB2  18       // | 18      |            |            |                       |                      |                                   | BOOT1     |
#define PB3  19       // | 19      |            |            |                       |                      | SPI3_SCK,  (SPI1_SCK)             |           |
#define PB4  20       // | 20      |            |            |                       |                      | SPI3_MISO, (SPI1_MISO)            |           |
#define PB5  21       // | 21      |            |            |                       |                      | SPI3_MOSI, (SPI1_MOSI)            |           |
#define PB6  22       // | 22      |            |            | USART1_TX             | TWI1_SCL             |                                   |           |
#define PB7  23       // | 23      |            |            | USART1_RX             | TWI1_SDA             |                                   |           |
#define PB8  24       // | 24      |            |            |                       | TWI1_SCL             |                                   |           |
#define PB9  25       // | 25      |            |            |                       | TWI1_SDA             | SPI2_SS                           |           |
#define PB10 26       // | 26      |            |            | USART3_TX, (UART4_TX) | TWI2_SCL             | SPI2_SCK                          |           |
#define PB11 27       // | 27      |            |            | USART3_RX             | TWI2_SDA             |                                   |           |
#define PB12 28       // | 28      |            |            |                       |                      | SPI2_SS                           |           |
#define PB13 29       // | 29      |            |            |                       |                      | SPI2_SCK                          |           |
#define PB14 30       // | 30      |            |            |                       |                      | SPI2_MISO                         |           |
#define PB15 31       // | 31      |            |            |                       |                      | SPI2_MOSI                         |           |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PC0  32       // | 32      | A10 (ADC1) |            |                       |                      |                                   |           |
#define PC1  33       // | 33      | A11 (ADC1) |            |                       |                      |                                   |           |
#define PC2  34       // | 34      | A12 (ADC1) |            |                       |                      | SPI2_MISO                         |           |
#define PC3  35       // | 35      | A13 (ADC1) |            |                       |                      | SPI2_MOSI                         |           |
#define PC4  36       // | 36      | A14 (ADC1) |            |                       |                      |                                   |           |
#define PC5  37       // | 37      | A15 (ADC1) |            | USART3_RX             |                      |                                   |           |
#define PC6  38       // | 38      |            |            | USART6_TX             |                      |                                   |           |
#define PC7  39       // | 39      |            |            | USART6_RX             |                      |                                   |           |
#define PC8  40       // | 40      |            |            |                       |                      |                                   |           |
#define PC9  41       // | 41      |            |            | USART3_TX             | TWI3_SDA             |                                   |           |
#define PC10 42       // | 42      |            |            |                       |                      | SPI3_SCK                          |           |
#define PC11 43       // | 43      |            |            | USART3_RX, (UART4_RX) |                      | SPI3_MISO                         |           |
#define PC12 44       // | 44      |            |            | UART5_TX              |                      | SPI3_MOSI                         |           |
#define PC13 45       // | 45      |            |            |                       |                      |                                   |           |
#define PC14 46       // | 46      |            |            |                       |                      |                                   | OSC32_IN  |
#define PC15 47       // | 47      |            |            |                       |                      |                                   | OSC32_OUT |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PD0  48       // | 48      |            |            |                       |                      |                                   |           |
#define PD1  49       // | 49      |            |            |                       |                      |                                   |           |
#define PD2  50       // | 50      |            |            | UART5_RX              |                      |                                   |           |
#define PD3  51       // | 51      |            |            |                       |                      |                                   |           |
#define PD4  52       // | 52      |            |            |                       |                      |                                   |           |
#define PD5  53       // | 53      |            |            | USART2_TX             |                      |                                   |           |
#define PD6  54       // | 54      |            |            | USART2_RX             |                      |                                   |           |
#define PD7  55       // | 55      |            |            |                       |                      |                                   |           |
#define PD8  56       // | 56      |            |            | USART3_TX             |                      |                                   |           |
#define PD9  57       // | 57      |            |            | USART3_RX             |                      |                                   |           |
#define PD10 58       // | 58      |            |            |                       |                      |                                   |           |
#define PD11 59       // | 59      |            |            |                       |                      |                                   |           |
#define PD12 60       // | 60      |            |            |                       |                      |                                   |           |
#define PD13 61       // | 61      |            |            |                       |                      |                                   |           |
#define PD14 62       // | 62      |            |            |                       |                      |                                   |           |
#define PD15 63       // | 63      |            |            |                       |                      |                                   |           |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PE0  64       // | 64      |            |            |                       |                      |                                   |           |
#define PE1  65       // | 65      |            |            |                       |                      |                                   |           |
#define PE2  66       // | 66      |            |            |                       |                      |                                   |           |
#define PE3  67       // | 67      |            |            |                       |                      |                                   |           |
#define PE4  68       // | 68      |            |            |                       |                      |                                   |           |
#define PE5  69       // | 69      |            |            |                       |                      |                                   |           |
#define PE6  70       // | 70      |            |            |                       |                      |                                   |           |
#define PE7  71       // | 71      |            |            |                       |                      |                                   |           |
#define PE8  72       // | 72      |            |            |                       |                      |                                   |           |
#define PE9  73       // | 73      |            |            |                       |                      |                                   |           |
#define PE10 74       // | 74      |            |            |                       |                      |                                   |           |
#define PE11 75       // | 75      |            |            |                       |                      |                                   |           |
#define PE12 76       // | 76      |            |            |                       |                      |                                   |           |
#define PE13 77       // | 77      |            |            |                       |                      |                                   |           |
#define PE14 78       // | 78      |            |            |                       |                      |                                   |           |
#define PE15 79       // | 79      |            |            |                       |                      |                                   |           |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|
#define PH0  80       // | 80      |            |            |                       |                      |                                   | OSC_IN    |
#define PH1  81       // | 81      |            |            |                       |                      |                                   | OSC_OUT   |
                      // |---------|------------|------------|-----------------------|----------------------|-----------------------------------|-----------|

// This must be a literal
#define NUM_DIGITAL_PINS        82
#define NUM_ANALOG_INPUTS       4

// SPI definitions
#define PIN_SPI_SS              PB12
#define PIN_SPI_MOSI            PB15
#define PIN_SPI_MISO            PB14
#define PIN_SPI_SCK             PB13

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6  // TIMER_TONE must be defined in this file
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7  // TIMER_SERVO must be defined in this file
#endif
// #ifndef TIMER_SERIAL
//   #define TIMER_SERIAL          TIM5  // TIMER_SERIAL must be defined in this file
// #endif

// UART Definitions
#define SERIAL_UART_INSTANCE    1

// Default pin used for 'Serial' instance
// Mandatory for Firmata
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

/* Extra HAL modules */
#ifndef HAL_DAC_MODULE_ENABLED
  #define HAL_DAC_MODULE_ENABLED
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
