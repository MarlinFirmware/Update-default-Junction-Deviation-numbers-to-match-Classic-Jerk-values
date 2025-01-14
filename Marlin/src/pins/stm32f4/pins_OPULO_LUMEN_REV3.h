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
 * STM32F407VET6 on Opulo Lumen PnP Rev3
 * Website - https://opulo.io/
 */

#define ALLOW_STM32DUINO
#include "env_validate.h"

#define BOARD_INFO_NAME      "Opulo Lumen REV3"
#define DEFAULT_MACHINE_NAME "Opulo Pick-and-Place"

/**
 * By default, the extra stepper motor configuration is:
 * I = Left Head
 * J = Right Head
 * K = Auxiliary (Conveyor belt)
 */

#define SRAM_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE                0x2000  // 8K

// I2C MCP3426 (16-Bit, 240SPS, dual-channel ADC)
#define HAS_MCP3426_ADC

//
// Servos
//
#define SERVO0_PIN                          PB10
#define SERVO1_PIN                          PB11

//
// Limit Switches
//
#define X_STOP_PIN                          PC6
#define Y_STOP_PIN                          PD15
#define Z_STOP_PIN                          PD14

// None of these require limit switches by default, so we leave these commented
// here for your reference.
//#define I_MIN_PIN                         PA8
//#define I_MAX_PIN                         PA8
//#define J_MIN_PIN                         PD13
//#define J_MAX_PIN                         PD13
//#define K_MIN_PIN                         PC9
//#define K_MAX_PIN                         PC9

//
// Steppers
//
#define X_STEP_PIN                          PB15
#define X_DIR_PIN                           PB14
#define X_ENABLE_PIN                        PD9

#define Y_STEP_PIN                          PE15
#define Y_DIR_PIN                           PE14
#define Y_ENABLE_PIN                        PB13

#define Z_STEP_PIN                          PE7
#define Z_DIR_PIN                           PB1
#define Z_ENABLE_PIN                        PE9

#define I_STEP_PIN                          PC4
#define I_DIR_PIN                           PA4
#define I_ENABLE_PIN                        PB0

#define J_STEP_PIN                          PE11
#define J_DIR_PIN                           PE10
#define J_ENABLE_PIN                        PE13

#define K_STEP_PIN                          PD6
#define K_DIR_PIN                           PD7
#define K_ENABLE_PIN                        PA3

#if HAS_TMC_SPI
  /**
   * Make sure to configure the jumpers on the back side of the Mobo according to
   * this diagram: https://github.com/MarlinFirmware/Marlin/pull/23851
   */
  #error "SPI drivers require a custom jumper configuration, see comment above! Comment out this line to continue."

  #if AXIS_HAS_SPI(X)
    #define X_CS_PIN                        PD8
  #endif
  #if AXIS_HAS_SPI(Y)
    #define Y_CS_PIN                        PB12
  #endif
  #if AXIS_HAS_SPI(Z)
    #define Z_CS_PIN                        PE8
  #endif
  #if AXIS_HAS_SPI(I)
    #define I_CS_PIN                        PC5
  #endif
  #if AXIS_HAS_SPI(J)
    #define J_CS_PIN                        PE12
  #endif
  #if AXIS_HAS_SPI(K)
    #define K_CS_PIN                        PA2
  #endif

#elif HAS_TMC_UART

  #define X_SERIAL_TX_PIN                   PD8
  #define Y_SERIAL_TX_PIN                   PB12
  #define Z_SERIAL_TX_PIN                   PE8
  #define I_SERIAL_TX_PIN                   PC5
  #define J_SERIAL_TX_PIN                   PE12
  #define K_SERIAL_TX_PIN                   PA2

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200
  #endif

#endif // HAS_TMC_UART

//
// Heaters / Fans
//
#define FAN0_PIN                            PE2
#define FAN1_PIN                            PE3
#define FAN2_PIN                            PE4
#define FAN3_PIN                            PE5

#define FAN_SOFT_PWM_REQUIRED

//
// NeoPixel
//
#define BOARD_NEOPIXEL_PIN                  PC7
#define NEOPIXEL2_PIN                       PC8

//
// SPI
//
#define MISO_PIN                            PB4
#define MOSI_PIN                            PB5
#define SCK_PIN                             PB3

#define TMC_SPI_MISO                    MISO_PIN
#define TMC_SPI_MOSI                    MOSI_PIN
#define TMC_SPI_SCK                      SCK_PIN

//
// I2C
//
#define I2C_SDA_PIN                         PB7
#define I2C_SCL_PIN                         PB6

/**
 * The index mobo rev03 has 3 aux ports. We define them here so they may be used
 * in other places and to make sure someone doesn't have to go look up the pinout
 * in the board files. Each 12 pin aux port has this pinout:
 *
 * VDC    1   2    GND
 * 3.3V   3   4    SCL  (I2C_SCL_PIN)
 * PWM1   5   6    SDA  (I2C_SDA_PIN)
 * PWM2   7   8    CIPO (MISO_PIN)
 * A1     9  10    COPI (MOSI_PIN)
 * A2     11 12    SCK  (SCK_PIN)
 */
#define INDEX_AUX1_PWM1                     PA15
#define INDEX_AUX1_PWM2                     PA5
#define INDEX_AUX1_A1                       PC0
#define INDEX_AUX1_A2                       PC1

#define INDEX_AUX2_PWM1                     PA6
#define INDEX_AUX2_PWM2                     PA7
#define INDEX_AUX2_A1                       PC2
#define INDEX_AUX2_A2                       PC3

#define INDEX_AUX3_PWM1                     PB8
#define INDEX_AUX3_PWM2                     PB9
#define INDEX_AUX3_A1                       PA0
#define INDEX_AUX3_A2                       PA1

#define RS485_TX_ENABLE_PIN                 PD11
#define RS485_RX_ENABLE_PIN                 PD12
