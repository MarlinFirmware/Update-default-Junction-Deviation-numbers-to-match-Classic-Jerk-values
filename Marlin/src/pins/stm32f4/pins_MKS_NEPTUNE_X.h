/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "MKS Neptune X supports up to 2 hotends / E steppers."
#endif

#define BOARD_INFO_NAME "MKS Neptune X"

//
// Software SPI pins for TMC2130 stepper drivers
// This board only supports SW SPI for stepper drivers
//
#if HAS_TMC_SPI && DISABLED(TMC_USE_SW_SPI)
  #warning "TMC_USE_SW_SPI is required for MKS Neptune X with TMC drivers."
#endif
#if ENABLED(TMC_USE_SW_SPI)
  #ifndef TMC_SW_MOSI
    #define TMC_SW_MOSI                     PD14
  #endif
  #ifndef TMC_SW_MISO
    #define TMC_SW_MISO                     PD1
  #endif
  #ifndef TMC_SW_SCK
    #define TMC_SW_SCK                      PD0
  #endif
#endif

#include "pins_MKS_NEPTUNE_X_common.h"
