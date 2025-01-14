/*
 *******************************************************************************
 * Copyright (c) 2020-2021, STMicroelectronics
 * All rights reserved.
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 *******************************************************************************
 */
#pragma once

/*----------------------------------------------------------------------------
 *        STM32 pins number
 *----------------------------------------------------------------------------*/
#define PA0                     PIN_A0
#define PA1                     PIN_A1
#define PA2                     PIN_A2
#define PA3                     PIN_A3
#define PA4                     PIN_A4
#define PA5                     PIN_A5
#define PA6                     PIN_A6
#define PA7                     PIN_A7
#define PA8                     8
#define PA9                     9
#define PA10                    10
#define PA11                    11
#define PA12                    12
#define PA13                    13
#define PA14                    14
#define PA15                    15
#define PB0                     PIN_A8
#define PB1                     PIN_A9
#define PB2                     PIN_A10
#define PB3                     19
#define PB4                     20
#define PB5                     21
#define PB6                     22
#define PB7                     23
#define PB8                     24
#define PB9                     25
#define PB10                    PIN_A11
#define PB11                    PIN_A12
#define PB12                    PIN_A13
#define PB13                    29
#define PB14                    30
#define PB15                    31
#define PC0                     32
#define PC1                     33
#define PC2                     34
#define PC3                     35
#define PC4                     PIN_A14
#define PC5                     PIN_A15
#define PC6                     38
#define PC7                     39
#define PC8                     40
#define PC9                     41
#define PC10                    42
#define PC11                    43
#define PC12                    44
#define PC13                    45
#define PC14                    46
#define PC15                    47
#define PD0                     48
#define PD1                     49
#define PD2                     50
#define PD3                     51
#define PD4                     52
#define PD5                     53
#define PD6                     54
#define PD7                     55
#define PD8                     56
#define PD9                     57
#define PD10                    58
#define PD11                    59
#define PD12                    60
#define PD13                    61
#define PD14                    62
#define PD15                    63
#define PE0                     64
#define PE1                     65
#define PE2                     66
#define PE3                     67
#define PE4                     68
#define PE5                     69
#define PE6                     70
#define PE7                     71
#define PE8                     72
#define PE9                     73
#define PE10                    74
#define PE11                    75
#define PE12                    76
#define PE13                    77
#define PE14                    78
#define PE15                    79
#define PF0                     80
#define PF1                     81
#define PF2                     82
#define PF3                     83
#define PF4                     84
#define PF5                     85
#define PF6                     86
#define PF7                     87
#define PF8                     88
#define PF9                     89
#define PF10                    90
#define PF11                    91
#define PF12                    92
#define PF13                    93
#define PA9_R                   94
#define PA10_R                  95

// Alternate pins number
#define PA1_ALT1                (PA1    | ALT1)
#define PA2_ALT1                (PA2    | ALT1)
#define PA3_ALT1                (PA3    | ALT1)
#define PA4_ALT1                (PA4    | ALT1)
#define PA6_ALT1                (PA6    | ALT1)
#define PA6_ALT2                (PA6    | ALT2)
#define PA7_ALT1                (PA7    | ALT1)
#define PA7_ALT2                (PA7    | ALT2)
#define PA7_ALT3                (PA7    | ALT3)
#define PA9_ALT1                (PA9    | ALT1)
#define PA9_R_ALT1              (PA9_R  | ALT1)
#define PA10_ALT1               (PA10   | ALT1)
#define PA10_R_ALT1             (PA10_R | ALT1)
#define PA14_ALT1               (PA14   | ALT1)
#define PA15_ALT1               (PA15   | ALT1)
#define PB0_ALT1                (PB0    | ALT1)
#define PB1_ALT1                (PB1    | ALT1)
#define PB1_ALT2                (PB1    | ALT2)
#define PB3_ALT1                (PB3    | ALT1)
#define PB4_ALT1                (PB4    | ALT1)
#define PB5_ALT1                (PB5    | ALT1)
#define PB6_ALT1                (PB6    | ALT1)
#define PB6_ALT2                (PB6    | ALT2)
#define PB7_ALT1                (PB7    | ALT1)
#define PB8_ALT1                (PB8    | ALT1)
#define PB9_ALT1                (PB9    | ALT1)
#define PB13_ALT1               (PB13   | ALT1)
#define PB14_ALT1               (PB14   | ALT1)
#define PB15_ALT1               (PB15   | ALT1)
#define PB15_ALT2               (PB15   | ALT2)
#define PC0_ALT1                (PC0    | ALT1)
#define PC1_ALT1                (PC1    | ALT1)
#define PC4_ALT1                (PC4    | ALT1)
#define PC5_ALT1                (PC5    | ALT1)
#define PC6_ALT1                (PC6    | ALT1)
#define PC7_ALT1                (PC7    | ALT1)
#define PC8_ALT1                (PC8    | ALT1)
#define PC9_ALT1                (PC9    | ALT1)
#define PC10_ALT1               (PC10   | ALT1)
#define PC11_ALT1               (PC11   | ALT1)
#define PD4_ALT1                (PD4    | ALT1)
#define PF7_ALT1                (PF7    | ALT1)

#define NUM_DIGITAL_PINS        96
#define NUM_REMAP_PINS          2
#define NUM_ANALOG_INPUTS       16

// On-board LED pin number
#ifndef LED_BUILTIN
  #define LED_BUILTIN           PNUM_NOT_DEFINED
#endif

// On-board user button
#ifndef USER_BTN
  #define USER_BTN              PNUM_NOT_DEFINED
#endif

// SPI definitions
#ifndef PIN_SPI_SS
  #define PIN_SPI_SS            PA4
#endif
#ifndef PIN_SPI_SS1
  #define PIN_SPI_SS1           PA15
#endif
#ifndef PIN_SPI_SS2
  #define PIN_SPI_SS2           PB0
#endif
#ifndef PIN_SPI_SS3
  #define PIN_SPI_SS3           PD9
#endif
#ifndef PIN_SPI_MOSI
  #define PIN_SPI_MOSI          PA2
#endif
#ifndef PIN_SPI_MISO
  #define PIN_SPI_MISO          PA6
#endif
#ifndef PIN_SPI_SCK
  #define PIN_SPI_SCK           PA1
#endif

// I2C definitions
#ifndef PIN_WIRE_SDA
  #define PIN_WIRE_SDA          PA6
#endif
#ifndef PIN_WIRE_SCL
  #define PIN_WIRE_SCL          PA7
#endif

// Timer Definitions
// Use TIM6/TIM7 when possible as servo and tone don't need GPIO output pin
#ifndef TIMER_TONE
  #define TIMER_TONE            TIM6
#endif
#ifndef TIMER_SERVO
  #define TIMER_SERVO           TIM7
#endif

// UART Definitions
#ifndef SERIAL_UART_INSTANCE
  #define SERIAL_UART_INSTANCE  4
#endif

// Default pin used for generic 'Serial' instance
// Mandatory for Firmata
#ifndef PIN_SERIAL_RX
  #define PIN_SERIAL_RX         PA1
#endif
#ifndef PIN_SERIAL_TX
  #define PIN_SERIAL_TX         PA0
#endif

// Extra HAL modules
#ifndef HAL_DAC_MODULE_DISABLED
  #define HAL_DAC_MODULE_ENABLED
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
  #ifndef SERIAL_PORT_MONITOR
    #define SERIAL_PORT_MONITOR   Serial
  #endif
  #ifndef SERIAL_PORT_HARDWARE
    #define SERIAL_PORT_HARDWARE  Serial
  #endif
#endif
