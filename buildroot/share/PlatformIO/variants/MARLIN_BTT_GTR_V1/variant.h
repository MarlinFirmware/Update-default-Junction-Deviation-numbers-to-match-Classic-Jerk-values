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

#ifdef STM32F405RX
  #define STM32F4X_PIN_NUM  64  //64 pins mcu, 51 gpio
  #define STM32F4X_GPIO_NUM 51
  #define STM32F4X_ADC_NUM  16
#elif defined(STM32F407_5VX)
  #define STM32F4X_PIN_NUM  100  //100 pins mcu, 82 gpio
  #define STM32F4X_GPIO_NUM 82
  #define STM32F4X_ADC_NUM  16
#elif defined(STM32F407_5ZX)
  #define STM32F4X_PIN_NUM  144  //144 pins mcu, 114 gpio
  #define STM32F4X_GPIO_NUM 114
  #define STM32F4X_ADC_NUM  24
#elif defined(STM32F407IX)
  #define STM32F4X_PIN_NUM  176  //176 pins mcu, 140 gpio
  #define STM32F4X_GPIO_NUM 140
  #define STM32F4X_ADC_NUM  24
#else
  #error "no match MCU defined"
#endif

#if STM32F4X_PIN_NUM >= 64  //64 pins mcu, 51 gpio
  #define PC13  0
  #define PC14  1 //OSC32_IN
  #define PC15  2 //OSC32_OUT
  #define PH0   3 //OSC_IN
  #define PH1   4 //OSC_OUT
  #define PB2   5 //BOOT1
  #define PB10  6 //1:SPI2_SCK / I2C2_SCL / USART3_TX / TIM2_CH3
  #define PB11  7 //1:I2C2_SDA / USART3_RX / TIM2_CH4
  #define PB12  8 //1:SPI2_NSS / OTG_HS_ID
  #define PB13  9 //1:SPI2_SCK  2:OTG_HS_VBUS
  #define PB14  10 //1:SPI2_MISO / TIM12_CH1 / OTG_HS_DM
  #define PB15  11 //SPI2_MOSI / TIM12_CH2 / OTG_HS_DP
  #define PC6   12 //1:TIM8_CH1 / SDIO_D6 / USART6_TX / TIM3_CH1
  #define PC7   13 //1:TIM8_CH2 / SDIO_D7 / USART6_RX / TIM3_CH2
  #define PC8   14 //1:TIM8_CH3 / SDIO_D0 / TIM3_CH3
  #define PC9   15 //1:TIM8_CH4 / SDIO_D1 / TIM3_CH4
  #define PA8   16 //1:TIM1_CH1 / I2C3_SCL / OTG_FS_SOF
  #define PA9   17 //1:USART1_TX / TIM1_CH2  2:OTG_FS_VBUS
  #define PA10  18 //1:USART1_RX / TIM1_CH3 / OTG_FS_ID
  #define PA11  19 //1:TIM1_CH4 / OTG_FS_DM
  #define PA12  20 //1:OTG_FS_DP
  #define PA13  21 //0:JTMS-SWDIO
  #define PA14  22 //0:JTCK-SWCLK
  #define PA15  23 //0:JTDI  1:SPI3_NSS / SPI1_NSS
  #define PC10  24 //1:UART4_TX / SPI3_SCK / SDIO_D2 / USART3_TX
  #define PC11  25 //1:UART4_RX / SPI3_MISO / SDIO_D3 / USART3_RX
  #define PC12  26 //1:UART5_TX / SPI3_MOSI / SDIO_CK
  #define PD2   27 //1:UART5_RX / SDIO_CMD
  #define PB3   28 //0:JTDO  1:SPI3_SCK / TIM2_CH2 / SPI1_SCK
  #define PB4   29 //0:NJTRST  1:SPI3_MISO / TIM3_CH1 / SPI1_MISO
  #define PB5   30 //1:TIM3_CH2 / SPI1_MOSI / SPI3_MOSI
  #define PB6   31 //1:I2C1_SCL / TIM4_CH1 / USART1_TX
  #define PB7   32 //1:I2C1_SDA / TIM4_CH2 / USART1_RX
  #define PB8   33 //1:I2C1_SCL / TIM4_CH3 / SDIO_D4 / TIM10_CH1
  #define PB9   34 //1:I2C1_SDA / TIM4_CH4 / SDIO_D5 / TIM11_CH1 / SPI2_NSS
  #define PA0   35 //1:UART4_TX / TIM5_CH1  2:ADC123_IN0
  #define PA1   36 //1:UART4_RX / TIM5_CH2 / TIM2_CH2  2:ADC123_IN1
  #define PA2   37 //1:USART2_TX /TIM5_CH3 / TIM9_CH1 / TIM2_CH3  2:ADC123_IN2
  #define PA3   38 //1:USART2_RX /TIM5_CH4 / TIM9_CH2 / TIM2_CH4  2:ADC123_IN3
  #define PA4   39 //NOT FT 1:SPI1_NSS / SPI3_NSS / USART2_CK  2:ADC12_IN4 / DAC_OUT1
  #define PA5   40 //NOT FT 1:SPI1_SCK  2:ADC12_IN5 / DAC_OUT2
  #define PA6   41 //1:SPI1_MISO / TIM13_CH1 / TIM3_CH1  2:ADC12_IN6
  #define PA7   42 //1:SPI1_MOSI / TIM14_CH1 / TIM3_CH2  2:ADC12_IN7
  #define PB0   43 //1:TIM3_CH3  2:ADC12_IN8
  #define PB1   44 //1:TIM3_CH4  2:ADC12_IN9
  #define PC0   45 //1:  2:ADC123_IN10
  #define PC1   46 //1:  2:ADC123_IN11
  #define PC2   47 //1:SPI2_MISO  2:ADC123_IN12
  #define PC3   48 //1:SPI2_MOSI  2:ADC123_IN13
  #define PC4   49 //1:  2:ADC12_IN14
  #define PC5   50 //1:  2:ADC12_IN15
  #if STM32F4X_PIN_NUM >= 144
    #define PF3   51 //1:FSMC_A3  2:ADC3_IN9
    #define PF4   52 //1:FSMC_A4  2:ADC3_IN14
    #define PF5   53 //1:FSMC_A5  2:ADC3_IN15
    #define PF6   54 //1:TIM10_CH1  2:ADC3_IN4
    #define PF7   55 //1:TIM11_CH1  2:ADC3_IN5
    #define PF8   56 //1:TIM13_CH1  2:ADC3_IN6
    #define PF9   57 //1;TIM14_CH1  2:ADC3_IN7
    #define PF10  58 //2:ADC3_IN8
  #endif
#endif
#if STM32F4X_PIN_NUM >= 100  //100 pins mcu, 82 gpio
  #define PE2   (35+STM32F4X_ADC_NUM) //1:FSMC_A23
  #define PE3   (36+STM32F4X_ADC_NUM) //1:FSMC_A19
  #define PE4   (37+STM32F4X_ADC_NUM) //1:FSMC_A20
  #define PE5   (38+STM32F4X_ADC_NUM) //1:FSMC_A21
  #define PE6   (39+STM32F4X_ADC_NUM) //1:FSMC_A22
  #define PE7   (40+STM32F4X_ADC_NUM) //1:FSMC_D4
  #define PE8   (41+STM32F4X_ADC_NUM) //1:FSMC_D5
  #define PE9   (42+STM32F4X_ADC_NUM) //1:FSMC_D6 / TIM1_CH1
  #define PE10  (43+STM32F4X_ADC_NUM) //1:FSMC_D7
  #define PE11  (44+STM32F4X_ADC_NUM) //1:FSMC_D8 / TIM1_CH2
  #define PE12  (45+STM32F4X_ADC_NUM) //1:FSMC_D9
  #define PE13  (46+STM32F4X_ADC_NUM) //1:FSMC_D10 / TIM1_CH3
  #define PE14  (47+STM32F4X_ADC_NUM) //1:FSMC_D11 / TIM1_CH4
  #define PE15  (48+STM32F4X_ADC_NUM) //1:FSMC_D12
  #define PD8   (49+STM32F4X_ADC_NUM) //1:FSMC_D13 / USART3_TX
  #define PD9   (50+STM32F4X_ADC_NUM) //1:FSMC_D14 / USART3_RX
  #define PD10  (51+STM32F4X_ADC_NUM) //1:FSMC_D15
  #define PD11  (52+STM32F4X_ADC_NUM) //1:FSMC_A16
  #define PD12  (53+STM32F4X_ADC_NUM) //1:FSMC_A17 / TIM4_CH1
  #define PD13  (54+STM32F4X_ADC_NUM) //1:FSMC_A18 / TIM4_CH2
  #define PD14  (55+STM32F4X_ADC_NUM) //1:FSMC_D0 / TIM4_CH3
  #define PD15  (56+STM32F4X_ADC_NUM) //1:FSMC_D1 / TIM4_CH4
  #define PD0   (57+STM32F4X_ADC_NUM) //1:FSMC_D2
  #define PD1   (58+STM32F4X_ADC_NUM) //1:FSMC_D3
  #define PD3   (59+STM32F4X_ADC_NUM) //1:FSMC_CLK
  #define PD4   (60+STM32F4X_ADC_NUM) //1:FSMC_NOE
  #define PD5   (61+STM32F4X_ADC_NUM) //1:USART2_TX
  #define PD6   (62+STM32F4X_ADC_NUM) //1:USART2_RX
  #define PD7   (63+STM32F4X_ADC_NUM)
  #define PE0   (64+STM32F4X_ADC_NUM)
  #define PE1   (65+STM32F4X_ADC_NUM)
#endif
#if STM32F4X_PIN_NUM >= 144  //144 pins mcu, 114 gpio
  #define PF0   (66+STM32F4X_ADC_NUM) //1:FSMC_A0 / I2C2_SDA
  #define PF1   (67+STM32F4X_ADC_NUM) //1:FSMC_A1 / I2C2_SCL
  #define PF2   (68+STM32F4X_ADC_NUM) //1:FSMC_A2
  #define PF11  (69+STM32F4X_ADC_NUM)
  #define PF12  (70+STM32F4X_ADC_NUM) //1:FSMC_A6
  #define PF13  (71+STM32F4X_ADC_NUM) //1:FSMC_A7
  #define PF14  (72+STM32F4X_ADC_NUM) //1:FSMC_A8
  #define PF15  (73+STM32F4X_ADC_NUM) //1:FSMC_A9
  #define PG0   (74+STM32F4X_ADC_NUM) //1:FSMC_A10
  #define PG1   (75+STM32F4X_ADC_NUM) //1:FSMC_A11
  #define PG2   (76+STM32F4X_ADC_NUM) //1:FSMC_A12
  #define PG3   (77+STM32F4X_ADC_NUM) //1:FSMC_A13
  #define PG4   (78+STM32F4X_ADC_NUM) //1:FSMC_A14
  #define PG5   (79+STM32F4X_ADC_NUM) //1:FSMC_A15
  #define PG6   (80+STM32F4X_ADC_NUM)
  #define PG7   (81+STM32F4X_ADC_NUM)
  #define PG8   (82+STM32F4X_ADC_NUM)
  #define PG9   (83+STM32F4X_ADC_NUM) //1:USART6_RX
  #define PG10  (84+STM32F4X_ADC_NUM) //1:FSMC_NE3
  #define PG11  (85+STM32F4X_ADC_NUM)
  #define PG12  (86+STM32F4X_ADC_NUM) //1:FSMC_NE4
  #define PG13  (87+STM32F4X_ADC_NUM) //1:FSMC_A24
  #define PG14  (88+STM32F4X_ADC_NUM) //1:FSMC_A25 / USART6_TX
  #define PG15  (89+STM32F4X_ADC_NUM)
#endif
#if STM32F4X_PIN_NUM >= 176  //176 pins mcu, 140 gpio
  #define PI8   (90+STM32F4X_ADC_NUM)
  #define PI9   (91+STM32F4X_ADC_NUM)
  #define PI10  (92+STM32F4X_ADC_NUM)
  #define PI11  (93+STM32F4X_ADC_NUM)
  #define PH2   (94+STM32F4X_ADC_NUM)
  #define PH3   (95+STM32F4X_ADC_NUM)
  #define PH4   (96+STM32F4X_ADC_NUM) //1:I2C2_SCL
  #define PH5   (97+STM32F4X_ADC_NUM) //1:I2C2_SDA
  #define PH6   (98+STM32F4X_ADC_NUM) //1:TIM12_CH1
  #define PH7   (99+STM32F4X_ADC_NUM) //1:I2C3_SCL
  #define PH8   (100+STM32F4X_ADC_NUM) //1:I2C3_SDA
  #define PH9   (101+STM32F4X_ADC_NUM) //1:TIM12_CH2
  #define PH10  (102+STM32F4X_ADC_NUM) //1:TIM5_CH1
  #define PH11  (103+STM32F4X_ADC_NUM) //1:TIM5_CH2
  #define PH12  (104+STM32F4X_ADC_NUM) //1:TIM5_CH3
  #define PH13  (105+STM32F4X_ADC_NUM)
  #define PH14  (106+STM32F4X_ADC_NUM)
  #define PH15  (107+STM32F4X_ADC_NUM)
  #define PI0   (108+STM32F4X_ADC_NUM) //1:TIM5_CH4 / SPI2_NSS
  #define PI1   (109+STM32F4X_ADC_NUM) //1:SPI2_SCK
  #define PI2   (110+STM32F4X_ADC_NUM) //1:TIM8_CH4 /SPI2_MISO
  #define PI3   (111+STM32F4X_ADC_NUM) //1:SPI2_MOS
  #define PI4   (112+STM32F4X_ADC_NUM)
  #define PI5   (113+STM32F4X_ADC_NUM) //1:TIM8_CH1
  #define PI6   (114+STM32F4X_ADC_NUM) //1:TIM8_CH2
  #define PI7   (115+STM32F4X_ADC_NUM) //1:TIM8_CH3
#endif


// This must be a literal
#define NUM_DIGITAL_PINS        (STM32F4X_GPIO_NUM)
// This must be a literal with a value less than or equal to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       (STM32F4X_ADC_NUM)
#define NUM_ANALOG_FIRST        35

// Below ADC, DAC and PWM definitions already done in the core
// Could be redefined here if needed
// ADC resolution is 12bits
//#define ADC_RESOLUTION          12
//#define DACC_RESOLUTION         12

// PWM resolution
/*
 * BEWARE:
 * Changing this value from the default (1000) will affect the PWM output value of analogWrite (to a PWM pin)
 * Since the pin is toggled on capture, if you change the frequency of the timer you have to adapt the compare value (analogWrite thinks you did)
 */
//#define PWM_FREQUENCY           20000
//The bottom values are the default and don't need to be redefined
//#define PWM_RESOLUTION          8
//#define PWM_MAX_DUTY_CYCLE      255

// On-board LED pin number
#define LED_BUILTIN             PA7
#define LED_GREEN               LED_BUILTIN

// Below SPI and I2C definitions already done in the core
// Could be redefined here if differs from the default one
// SPI Definitions
#define PIN_SPI_MOSI            PB15
#define PIN_SPI_MISO            PB14
#define PIN_SPI_SCK             PB13
#define PIN_SPI_SS              PB12

// I2C Definitions
#if STM32F4X_PIN_NUM >= 176
  #define PIN_WIRE_SDA          PH5
  #define PIN_WIRE_SCL          PH4
#else
  #define PIN_WIRE_SDA          PB7
  #define PIN_WIRE_SCL          PB6
#endif

// Timer Definitions
// Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM10 // TIMER_TONE must be defined in this file
#define TIMER_SERVO             TIM5  // TIMER_SERVO must be defined in this file
#define TIMER_SERIAL            TIM7  // TIMER_SERIAL must be defined in this file

// UART Definitions
//#define ENABLE_HWSERIAL1        done automatically by the #define SERIAL_UART_INSTANCE below
#define ENABLE_HWSERIAL3
#define ENABLE_HWSERIAL6

// Define here Serial instance number to map on Serial generic name (if not already used by SerialUSB)
#define SERIAL_UART_INSTANCE    1 //1 for Serial = Serial1 (USART1)

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

// Optional PIN_SERIALn_RX and PIN_SERIALn_TX where 'n' is the U(S)ART number
// Used when the user instantiates a hardware Serial using its peripheral name.
// Example: HardwareSerial mySerial(USART3);
// will use PIN_SERIAL3_RX and PIN_SERIAL3_TX if defined.
#define PIN_SERIAL1_RX          PA10
#define PIN_SERIAL1_TX          PA9
#define PIN_SERIAL3_RX          PD9
#define PIN_SERIAL3_TX          PD8
#define PIN_SERIAL6_RX          PC7
#define PIN_SERIAL6_TX          PC6
//#define PIN_SERIALLP1_RX        x // For LPUART1 RX
//#define PIN_SERIALLP1_TX        x // For LPUART1 TX

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
#define SERIAL_PORT_MONITOR         Serial
#define SERIAL_PORT_HARDWARE        Serial1
#define SERIAL_PORT_HARDWARE_OPEN   Serial3
#define SERIAL_PORT_HARDWARE_OPEN1  Serial6
#endif
