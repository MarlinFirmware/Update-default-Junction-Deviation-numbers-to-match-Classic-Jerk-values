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

// Left Side
#define PB12 0
#define PB13 1
#define PB14 2
#define PB15 3
#define PD8  4
#define PD9  5
#define PD10 6
#define PD11 7
#define PD12 8
#define PD13 9
#define PD14 10
#define PD15 11
#define PG2  12
#define PG3  13
#define PG4  14
#define PG5  15
#define PG6  16
#define PG7  17
#define PG8  18
#define PC6  19
#define PC7  20
#define PC8  21
#define PC9  22
#define PA8  23
#define PA9  24
#define PA10 25
#define PA11 26 // USB_DM
#define PA12 27 // USB_DP
#define PA13 28
#define PA14 29
#define PA15 30
#define PC10 31
#define PC11 32
#define PC12 33
#define PD0  34
#define PD1  35
#define PD2  36
#define PD3  37
#define PD4  38
#define PD5  39
#define PD6  40
#define PD7  41
#define PG9  42
#define PG10 43
#define PG11 44
#define PG12 45
#define PG13 46
#define PG14 47
#define PG15 48
#define PB3  49
#define PB4  50
#define PB5  51
#define PB6  52
#define PB7  53
#define PB8  54
#define PB9  55

// Right Side
#define PB10 56
#define PB11 57
#define PE14 58
#define PE15 59
#define PE12 60
#define PE13 61
#define PE10 62
#define PE11 63
#define PE8  64
#define PE9  65
#define PG1  66
#define PE7  67
#define PF15 68
#define PG0  69
#define PF13 70
#define PF14 71
#define PF11 72
#define PF12 73
#define PB2  74
#define PB1  75 // A0
#define PC5  76 // A1
#define PB0  77 // A2
#define PA7  78 // A3
#define PC4  79 // A4
#define PA5  80 // A5
#define PA6  81 // A6
#define PA3  82 // A7
#define PA4  83 // A8
#define PA1  84 // A9
#define PA2  85 // A10
#define PC3  86 // A11
#define PA0  87 // A12/PA_0(WK_UP): BUT K_UP)
#define PC1  88 // A13
#define PC2  89 // A14
#define PC0  90 // A15
#define PF8  91 // A16
#define PF6  92 // A17
#define PF7  93 // A18
#define PF9  94 // LED D1 (active low)
#define PF10 95 // LED D2 (active low)
#define PF4  96
#define PF5  97
#define PF2  98
#define PF3  99
#define PF0  100
#define PF1  101
#define PE6  102
#define PC13 103
#define PE4  104 // BUT K0
#define PE5  105 // BUT K1
#define PE2  106
#define PE3  107
#define PE0  108
#define PE1  109
#define PC14  110
#define PC15  111
// This must be a literal
#define NUM_DIGITAL_PINS        112
// This must be a literal with a value less than or equal to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       23
#define NUM_ANALOG_FIRST        75


// Below SPI and I2C definitions already done in the core
// Could be redefined here if differs from the default one
// SPI Definitions

// SPI1 (Winbond on the Lerdge-K)
//#define PIN_SPI_SS            PC4
//#define PIN_SPI_SCK           PA5
//#define PIN_SPI_MISO          PA6
//#define PIN_SPI_MOSI          PA7

// SPI2 (Winbond on the Lerdge-S)
//#define PIN_SPI_SS            PB12
//#define PIN_SPI_SCK           PB13
//#define PIN_SPI_MISO          PB14
//#define PIN_SPI_MOSI          PB15

// I2C Definitions (Software I2C)
//#define PIN_WIRE_SDA          PG13
//#define PIN_WIRE_SCL          PG14

// Timer Definitions
// Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
// FANs may require PWM timers 3 10 11 13
// The LED/RGB connectors timer 4
// Beware: MCU_TIMER_STEP default is 6 and MCU_TIMER_TEMP 14 for the F407
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM8  // TIM3 or TIM8 for SPEAKER compat on the lerdge K (PC6)
#endif                                // TIM4 for that on the Lerdge S (PD11)
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM1  // Ideally TIM2 for Hardware PWM (PB11)
#endif                                // TIM4 on the S (PD12)
#ifndef TIMER_SERIAL
  #define TIMER_SERIAL          TIM7  // Default used in SoftwareSerial lib
#endif

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
#ifndef HAL_SRAM_MODULE_ENABLED
  #define HAL_SRAM_MODULE_ENABLED
#endif
#ifndef HAL_EXTI_MODULE_ENABLED
  #define HAL_EXTI_MODULE_ENABLED // Needed for Endstop (and other external) Interrupts
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

