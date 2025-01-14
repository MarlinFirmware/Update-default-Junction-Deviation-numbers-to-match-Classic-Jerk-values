/*
 *******************************************************************************
 * Copyright (c) 2017, STMicroelectronics
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/

#define PA1     0
#define PA2     1
#define PA3     2
#define PA4     3
#define PA5     4
#define PA6     5
#define PA7     6
#define PA8     7
#define PA9     8
#define PA10    9
#define PA11    10
#define PA12    11
#define PA13    12
#define PA14    13
#define PA15    14
#define PB0     15
#define PB1     16
#define PB2     17
#define PB3     18
#define PB4     19
#define PB5     20
#define PB6     21
#define PB7     22
#define PB8     23
#define PB9     24
#define PB10    25
#define PB11    26
#define PB12    27
#define PB13    28
#define PB14    29
#define PB15    30
#define PC2     31
#define PC3     32
#define PC4     33
#define PC5     34
#define PC6     35
#define PC7     36
#define PC8     37
#define PC9     38
#define PC10    39
#define PC11    40
#define PC12    41
#define PC13    42
#define PC14    43
#define PC15    44
#define PD0     45
#define PD1     46
#define PD2     47
#define PD3     48
#define PD4     49
#define PD5     50
#define PD6     51
#define PD7     52
#define PD8     53
#define PD9     54
#define PD10    55
#define PD11    56
#define PD12    57
#define PD13    58
#define PD14    59
#define PD15    60
#define PE0     61
#define PE1     62
#define PE11    63
#define PE3     64
#define PE4     65
#define PE5     66
#define PE6     67
#define PE7     68
#define PE8     69
#define PE9     70
#define PE10    71
#define PE2     72
#define PE12    73
#define PE13    74
#define PE14    75
#define PE15    76
#define PF0     77
#define PF1     78
#define PF2     79
#define PF6     80
#define PF7     81
#define PF8     82
#define PF9     83
#define PF11    84
#define PF12    85
#define PF13    86
#define PF14    87
#define PF15    88
#define PG0     89
#define PG1     90
#define PG2     91
#define PG3     92
#define PG4     93
#define PG5     94
#define PG6     95
#define PG7     96
#define PG8     97
#define PG9     98
#define PG10    99
#define PG11    100
#define PG12    101
#define PG13    102
#define PG14    103
#define PG15    104
#define PH0     105
#define PH1     106
#define PA0     107
#define PC1     108
#define PC0     109
#define PF10    110
#define PF5     111
#define PF4     112
#define PF3     113

// This must be a literal
#define NUM_DIGITAL_PINS        114
// This must be a literal with a value less than or equal to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       7
#define NUM_ANALOG_FIRST        107


// Below SPI and I2C definitions already done in the core
// Could be redefined here if differs from the default one
// SPI Definitions
#define PIN_SPI_SS              PF11
#define PIN_SPI_MOSI            PB15
#define PIN_SPI_MISO            PB14
#define PIN_SPI_SCK             PB13



//max6675
//#define PIN_SPI_SS              PA4
//#define PIN_SPI_SCK             PA5
//#define PIN_SPI_MISO            PA6
//#define PIN_SPI_MOSI            PA7




// I2C Definitions
#define PIN_WIRE_SDA            PB7
#define PIN_WIRE_SCL            PB6

// Timer Definitions
// Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM6  // TIMER_TONE must be defined in this file

// Do not use basic timer: OC is required
#define TIMER_SERVO             TIM1  // TODO: advanced-control timers don't work

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE    1 //ex: 2 for Serial2 (USART2)
// DEBUG_UART could be redefined to print on another instance than 'Serial'
//#define DEBUG_UART              ((USART_TypeDef *) U(S)ARTX) // ex: USART3
// DEBUG_UART baudrate, default: 9600 if not defined
//#define DEBUG_UART_BAUDRATE     x
// DEBUG_UART Tx pin name, default: the first one found in PinMap_UART_TX for DEBUG_UART
//#define DEBUG_PINNAME_TX        PX_n // PinName used for TX

// Default pin used for 'Serial' instance (ex: ST-Link)
// Mandatory for Firmata
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9

/* Extra HAL modules */
#ifndef HAL_DAC_MODULE_ENABLED
  //#define HAL_DAC_MODULE_ENABLED
#endif
#ifndef HAL_SD_MODULE_ENABLED
  #define HAL_SD_MODULE_ENABLED
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
