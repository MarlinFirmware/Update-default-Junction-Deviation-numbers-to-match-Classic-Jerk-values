/*
 *******************************************************************************
 * Copyright (c) 2019, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */

#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
#define  PA0  0
#define  PA1  1
#define  PA2  2
#define  PA3  3
#define  PA4  4
#define  PA5  5
#define  PA6  6
#define  PA7  7
#define  PA8  8
#define  PA9  9
#define  PA10 10
#define  PA11 11
#define  PA12 12
#define  PA13 13
#define  PA14 14
#define  PA15 15

#define  PB0  16
#define  PB1  17
#define  PB2  18
#define  PB3  19
#define  PB4  20
#define  PB5  21
#define  PB6  22
#define  PB7  23
#define  PB8  24
#define  PB9  25
#define  PB10 26
#define  PB11 27
#define  PB12 28
#define  PB13 29
#define  PB14 30
#define  PB15 31

#define  PC0  32
#define  PC1  33
#define  PC2  34
#define  PC3  35
#define  PC4  36
#define  PC5  37
#define  PC6  38
#define  PC7  39
#define  PC8  40
#define  PC9  41
#define  PC10 42
#define  PC11 43
#define  PC12 44
#define  PC13 45
#define  PC14 46
#define  PC15 47

#define  PD0  48
#define  PD1  49
#define  PD2  50
#define  PD3  51
#define  PD4  52
#define  PD5  53
#define  PD6  54
#define  PD7  55
#define  PD8  56
#define  PD9  57
#define  PD10 58
#define  PD11 59
#define  PD12 60
#define  PD13 61
#define  PD14 62
#define  PD15 63

#define  PE0  64
#define  PE1  65
#define  PE2  66
#define  PE3  67
#define  PE4  68
#define  PE5  69
#define  PE6  70
#define  PE7  71
#define  PE8  72
#define  PE9  73
#define  PE10 74
#define  PE11 75
#define  PE12 76
#define  PE13 77
#define  PE14 78
#define  PE15 79

#define  PF0  80
#define  PF1  81
#define  PF2  82
#define  PF3  83
#define  PF4  84
#define  PF5  85
#define  PF6  86
#define  PF7  87
#define  PF8  88
#define  PF9  89
#define  PF10 90
#define  PF11 91
#define  PF12 92
#define  PF13 93
#define  PF14 94
#define  PF15 95

#define  PG0  96
#define  PG1  97
#define  PG2  98
#define  PG3  99
#define  PG4  100
#define  PG5  101
#define  PG6  102
#define  PG7  103
#define  PG8  104
#define  PG9  105
#define  PG10 106
#define  PG11 107
#define  PG12 108
#define  PG13 109
#define  PG14 110
#define  PG15 111

// This must be a literal
#define NUM_DIGITAL_PINS        112
#define NUM_ANALOG_INPUTS       21

// On-board LED pin number
#ifdef ARDUINO_VCCGND_F103ZET6_MINI
#define LED_BUILTIN             PG15
#elif defined(ARDUINO_VCCGND_F103ZET6)
#define LED_BUILTIN             PC13
#elif !defined(LED_BUILTIN)
#define LED_BUILTIN             PC13
#endif

// On-board user button
#ifndef USER_BTN
#define USER_BTN                PB1
#endif

// SPI Definitions
#ifdef ARDUINO_VCCGND_F103ZET6
#define PIN_SPI_SS              PA15
#define PIN_SPI_MOSI            PB5
#define PIN_SPI_MISO            PB4
#define PIN_SPI_SCK             PB3
#else
#define PIN_SPI_SS              PC4
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5
#endif

// I2C Definitions
#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

// Timer Definitions (optional)
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#define TIMER_TONE              TIM6  // TIMER_TONE must be defined in this file
#define TIMER_SERVO             TIM7  // TIMER_SERVO must be defined in this file

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE    1

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

// Default pin used for 'Serial2' instance
#define PIN_SERIAL2_RX          PA3
#define PIN_SERIAL2_TX          PA2

// Default pin used for 'Serial3' instance
#define PIN_SERIAL3_RX          PB11
#define PIN_SERIAL3_TX          PB10

/* Extra HAL modules */
#ifndef HAL_DAC_MODULE_ENABLED
  #define HAL_DAC_MODULE_ENABLED
#endif
#ifndef HAL_SD_MODULE_ENABLED
  #define HAL_SD_MODULE_ENABLED
#endif
#ifndef HAL_SRAM_MODULE_ENABLED
  #define HAL_SRAM_MODULE_ENABLED
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
