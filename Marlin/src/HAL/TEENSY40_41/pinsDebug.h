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
 * Pins Debugging for Teensy 4.0 (IMXRT1062DVL6A) / 4.1 (IMXRT1062DVJ6A)
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

#warning "PINS_DEBUGGING is not fully supported for Teensy 4.0 / 4.1 so 'M43' may cause hangs."

#define NUMBER_PINS_TOTAL NUM_DIGITAL_PINS

#define getPinByIndex(x) pin_array[x].pin
#define printPinNameByIndex(x) do{ sprintf_P(buffer, PSTR("%-" STRINGIFY(MAX_NAME_LENGTH) "s"), pin_array[x].name); SERIAL_ECHO(buffer); }while(0)
#define getPinIsDigitalByIndex(x) pin_array[x].is_digital
#define digitalPinToAnalogIndex(P) int(P - analogInputToDigitalPin(0))
#define getValidPinMode(P) (isValidPin(P) && IS_OUTPUT(P))
#define isValidPin(P) (P >= 0 && P < pin_t(NUMBER_PINS_TOTAL))
#define isAnalogPin(P) (pin_t(P) >= analogInputToDigitalPin(0) && pin_t(P) <= analogInputToDigitalPin(13)) || (pin_t(P) >= analogInputToDigitalPin(14) && pin_t(P) <= analogInputToDigitalPin(17))
#define digitalRead_mod(P) extDigitalRead(P)  // AVR digitalRead disabled PWM before it read the pin
#define printPinNumber(P) do{ sprintf_P(buffer, PSTR("%02d"), P); SERIAL_ECHO(buffer); }while(0)
#define printPinAnalog(P) do{ sprintf_P(buffer, PSTR(" (A%2d)  "), digitalPinToAnalogIndex(P)); SERIAL_ECHO(buffer); }while(0)
#define MULTI_NAME_PAD 16 // space needed to be pretty if not first name assigned to a pin

struct pwm_pin_info_struct {
  uint8_t type;    // 0=no pwm, 1=flexpwm, 2=quad
  uint8_t module;  // 0-3, 0-3
  uint8_t channel; // 0=X, 1=A, 2=B
  uint8_t muxval;  //
};

#define M(a, b) ((((a) - 1) << 4) | (b))

const struct pwm_pin_info_struct pwm_pin_info[] = {
  {1, M(1, 1), 0, 4},  // FlexPWM1_1_X   0  // AD_B0_03
  {1, M(1, 0), 0, 4},  // FlexPWM1_0_X   1  // AD_B0_02
  {1, M(4, 2), 1, 1},  // FlexPWM4_2_A   2  // EMC_04
  {1, M(4, 2), 2, 1},  // FlexPWM4_2_B   3  // EMC_05
  {1, M(2, 0), 1, 1},  // FlexPWM2_0_A   4  // EMC_06
  {1, M(2, 1), 1, 1},  // FlexPWM2_1_A   5  // EMC_08
  {1, M(2, 2), 1, 2},  // FlexPWM2_2_A   6  // B0_10
  {1, M(1, 3), 2, 6},  // FlexPWM1_3_B   7  // B1_01
  {1, M(1, 3), 1, 6},  // FlexPWM1_3_A   8  // B1_00
  {1, M(2, 2), 2, 2},  // FlexPWM2_2_B   9  // B0_11
  {2, M(1, 0), 0, 1},  // QuadTimer1_0  10  // B0_00
  {2, M(1, 2), 0, 1},  // QuadTimer1_2  11  // B0_02
  {2, M(1, 1), 0, 1},  // QuadTimer1_1  12  // B0_01
  {2, M(2, 0), 0, 1},  // QuadTimer2_0  13  // B0_03
  {2, M(3, 2), 0, 1},  // QuadTimer3_2  14  // AD_B1_02
  {2, M(3, 3), 0, 1},  // QuadTimer3_3  15  // AD_B1_03
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {2, M(3, 1), 0, 1},  // QuadTimer3_1  18  // AD_B1_01
  {2, M(3, 0), 0, 1},  // QuadTimer3_0  19  // AD_B1_00
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(4, 0), 1, 1},  // FlexPWM4_0_A  22  // AD_B1_08
  {1, M(4, 1), 1, 1},  // FlexPWM4_1_A  23  // AD_B1_09
  {1, M(1, 2), 0, 4},  // FlexPWM1_2_X  24  // AD_B0_12
  {1, M(1, 3), 0, 4},  // FlexPWM1_3_X  25  // AD_B0_13
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(3, 1), 2, 1},  // FlexPWM3_1_B  28  // EMC_32
  {1, M(3, 1), 1, 1},  // FlexPWM3_1_A  29  // EMC_31
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {0, M(1, 0), 0, 0},
  {1, M(2, 0), 2, 1},  // FlexPWM2_0_B  33  // EMC_07
  #ifdef ARDUINO_TEENSY40
    {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  34  // SD_B0_03
    {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  35  // SD_B0_02
    {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  36  // SD_B0_01
    {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  37  // SD_B0_00
    {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  38  // SD_B0_05
    {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  39  // SD_B0_04
  #endif
  #ifdef ARDUINO_TEENSY41
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(2, 3), 1, 6},  // FlexPWM2_3_A  36  // B1_00
    {1, M(2, 3), 2, 6},  // FlexPWM2_3_B  37  // B1_01
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {0, M(1, 0), 0, 0},
    {1, M(1, 1), 2, 1},  // FlexPWM1_1_B  42  // SD_B0_03
    {1, M(1, 1), 1, 1},  // FlexPWM1_1_A  43  // SD_B0_02
    {1, M(1, 0), 2, 1},  // FlexPWM1_0_B  44  // SD_B0_01
    {1, M(1, 0), 1, 1},  // FlexPWM1_0_A  45  // SD_B0_00
    {1, M(1, 2), 2, 1},  // FlexPWM1_2_B  46  // SD_B0_05
    {1, M(1, 2), 1, 1},  // FlexPWM1_2_A  47  // SD_B0_04
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_0_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_A
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_2_B
    {1, M(3, 3), 2, 1},  // FlexPWM3_3_B  51  // EMC_22
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_B
    {0, M(1, 0), 0, 0},  // duplicate FlexPWM1_1_A
    {1, M(3, 0), 1, 1},  // FlexPWM3_0_A  53  // EMC_29
  #endif
};

void printAnalogPin(char buffer[], const pin_t pin) {
  if (pin <= 23)      sprintf_P(buffer, PSTR("(A%2d)  "), int(pin - 14));
  else if (pin <= 41) sprintf_P(buffer, PSTR("(A%2d)  "), int(pin - 24));
}

void analog_pin_state(char buffer[], const pin_t pin) {
  if (pin <= 23)      sprintf_P(buffer, PSTR("Analog in =% 5d"), analogRead(pin - 14));
  else if (pin <= 41) sprintf_P(buffer, PSTR("Analog in =% 5d"), analogRead(pin - 24));
}

#define PWM_PRINT(V) do{ sprintf_P(buffer, PSTR("PWM:  %4d"), V); SERIAL_ECHO(buffer); }while(0)

/**
 * Print a pin's PWM status.
 * Return true if it's currently a PWM pin.
 */
bool pwm_status(const pin_t pin) {
  char buffer[20];   // for the sprintf statements
  const struct pwm_pin_info_struct *info;

  if (pin >= CORE_NUM_DIGITAL) return false;

  info = pwm_pin_info + pin;
  if (info->type == 0) return false;

  /* TODO decode pwm value from timers */
  // for now just indicate if output is set as pwm
  PWM_PRINT(*(portConfigRegister(pin)) == info->muxval);
  return (*(portConfigRegister(pin)) == info->muxval);
}

void printPinPWM(const pin_t) { /* TODO */ }

void printPinPort(const pin_t) {}
