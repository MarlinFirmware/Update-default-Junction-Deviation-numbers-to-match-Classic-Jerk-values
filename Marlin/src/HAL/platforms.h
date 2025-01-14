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

#define XSTR(V...) #V

#ifdef __AVR__
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/AVR/NAME)
#elif defined(ARDUINO_ARCH_SAM)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/DUE/NAME)
#elif defined(__MK20DX256__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/TEENSY31_32/NAME)
#elif defined(__MK64FX512__) || defined(__MK66FX1M0__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/TEENSY35_36/NAME)
#elif defined(__IMXRT1062__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/TEENSY40_41/NAME)
#elif defined(TARGET_LPC1768)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/LPC1768/NAME)
#elif defined(ARDUINO_ARCH_HC32)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/HC32/NAME)
#elif defined(__STM32F1__) || defined(TARGET_STM32F1)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/STM32F1/NAME)
#elif defined(ARDUINO_ARCH_STM32)
  #ifndef HAL_STM32
    #define HAL_STM32
  #endif
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/STM32/NAME)
#elif defined(ARDUINO_ARCH_ESP32)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/ESP32/NAME)
#elif defined(__PLAT_LINUX__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/LINUX/NAME)
#elif defined(__PLAT_NATIVE_SIM__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/NATIVE_SIM/NAME)
#elif defined(__SAMD51__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/SAMD51/NAME)
#elif defined(__SAMD21__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/SAMD21/NAME)
#elif defined(__PLAT_RP2040__)
  #define HAL_PATH(PATH, NAME) XSTR(PATH/HAL/RP2040/NAME)
#else
  #error "Unsupported Platform!"
#endif
