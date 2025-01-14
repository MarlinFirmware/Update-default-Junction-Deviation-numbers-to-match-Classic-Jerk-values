/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * Pins Debugging for SAMD21
 *
 *   - NUMBER_PINS_TOTAL
 *   - MULTI_NAME_PAD
 *   - getPinByIndex(index)
 *   - printPinNameByIndex(index)
 *   - getPinIsDigitalByIndex(index)
 *   - digitalPinToAnalogIndex(pin)
 *   - getValidPinMode(pin)
 *   - isValidPin(pin)
 *   - isAnalogPin(pin)
 *   - digitalRead_mod(pin)
 *   - pwm_status(pin)
 *   - printPinPWM(pin)
 *   - printPinPort(pin)
 *   - printPinNumber(pin)
 *   - printPinAnalog(pin)
 */

#define NUMBER_PINS_TOTAL PINS_COUNT

#define digitalRead_mod(P) extDigitalRead(P)
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%3d "), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
#define getPinByIndex(x) pin_array[x].pin
#define getPinIsDigitalByIndex(x) pin_array[x].is_digital
#define isValidPin(P) (P >= 0 && P < pin_t(NUMBER_PINS_TOTAL))
#define isAnalogPin(P) (digitalPinToAnalogIndex(P) != -1)
#define pwm_status(P) digitalPinHasPWM(P)
#define MULTI_NAME_PAD 27 // space needed to be pretty if not first name assigned to a pin

// pins that will cause hang/reset/disconnect in M43 Toggle and Watch utilities
// uses pin index
#define M43_NEVER_TOUCH(Q) ((Q) >= 75)

bool getValidPinMode(const int8_t pin) {  // 1: output, 0: input
  const EPortType samdport = g_APinDescription[pin].ulPort;
  const uint32_t samdpin = g_APinDescription[pin].ulPin;
  return PORT->Group[samdport].DIR.reg & MASK(samdpin) || (PORT->Group[samdport].PINCFG[samdpin].reg & (PORT_PINCFG_INEN | PORT_PINCFG_PULLEN)) == PORT_PINCFG_PULLEN;
}

void printPinPWM(const int32_t pin) {
  if (pwm_status(pin)) {
    //uint32_t chan = g_APinDescription[pin].ulPWMChannel TODO when fast pwm is operative;
    //SERIAL_ECHOPGM("PWM = ", duty);
  }
}

void printPinPort(const pin_t) {}

/**
 * SAMD21 Board pin|  PORT  | Label
 * ----------------+--------+-------
 *   0             |  PB25  | "RX0"
 *   1             |  PB24  | "TX0"
 *   2             |  PC18  |
 *   3             |  PC19  |
 *   4             |  PC20  |
 *   5             |  PC21  |
 *   6             |  PD20  |
 *   7             |  PD21  |
 *   8             |  PB18  |
 *   9             |  PB2   |
 *  10             |  PB22  |
 *  11             |  PB23  |
 *  12             |  PB0   | "A16"
 *  13             |  PB1   | LED AMBER "L" / "A17"
 *  14             |  PB16  | "TX3"
 *  15             |  PB17  | "RX3"
 *  16             |  PC22  | "TX2"
 *  17             |  PC23  | "RX2"
 *  18             |  PB12  | "TX1" / "A18"
 *  19             |  PB13  | "RX1"
 *  20             |  PB20  | "SDA"
 *  21             |  PB21  | "SCL"
 *  22             |  PD12  |
 *  23             |  PA15  |
 *  24             |  PC17  |
 *  25             |  PC16  |
 *  26             |  PA12  |
 *  27             |  PA13  |
 *  28             |  PA14  |
 *  29             |  PB19  |
 *  30             |  PA23  |
 *  31             |  PA22  |
 *  32             |  PA21  |
 *  33             |  PA20  |
 *  34             |  PA19  |
 *  35             |  PA18  |
 *  36             |  PA17  |
 *  37             |  PA16  |
 *  38             |  PB15  |
 *  39             |  PB14  |
 *  40             |  PC13  |
 *  41             |  PC12  |
 *  42             |  PC15  |
 *  43             |  PC14  |
 *  44             |  PC11  |
 *  45             |  PC10  |
 *  46             |  PC6   |
 *  47             |  PC7   |
 *  48             |  PC4   |
 *  49             |  PC5   |
 *  50             |  PD11  |
 *  51             |  PD8   |
 *  52             |  PD9   |
 *  53             |  PD10  |
 *  54             |  PB5   | "A8"
 *  55             |  PB6   | "A9"
 *  56             |  PB7   | "A10"
 *  57             |  PB8   | "A11"
 *  58             |  PB9   | "A12"
 *  69             |  PA4   | "A13"
 *  60             |  PA6   | "A14"
 *  61             |  PA7   | "A15"
 *  62             |  PB17  |
 *  63             |  PB20  |
 *  64             |  PD11  |
 *  65             |  PD8   |
 *  66             |  PD9   |
 *  67             |  PA2   | "A0" / "DAC0"
 *  68             |  PA5   | "A1" / "DAC1"
 *  69             |  PB3   | "A2"
 *  70             |  PC0   | "A3"
 *  71             |  PC1   | "A4"
 *  72             |  PC2   | "A5"
 *  73             |  PC3   | "A6"
 *  74             |  PB4   | "A7"
 *  75             |  PC31  | LED GREEN "RX"
 *  76             |  PC30  | LED GREEN "TX"
 *  77             |  PA27  | USB: Host enable
 *  78             |  PA24  | USB: D-
 *  79             |  PA25  | USB: D+
 *  80             |  PB29  | SD: MISO
 *  81             |  PB27  | SD: SCK
 *  82             |  PB26  | SD: MOSI
 *  83             |  PB28  | SD: CS
 *  84             |  PA3   | AREF
 *  85             |  PA2   | DAC0 (Duplicate)
 *  86             |  PA5   | DAC1 (Duplicate)
 *  87             |  PB1   | LED AMBER "L" (Duplicate)
 *  88             |  PC24  | NeoPixel
 *  89             |  PB10  | QSPI: SCK
 *  90             |  PB11  | QSPI: CS
 *  91             |  PA8   | QSPI: IO0
 *  92             |  PA9   | QSPI: IO1
 *  93             |  PA10  | QSPI: IO2
 *  94             |  PA11  | QSPI: IO3
 *  95             |  PB31  | SD: DETECT
 */
