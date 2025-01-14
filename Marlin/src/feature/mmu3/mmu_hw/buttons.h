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

/**
 * buttons.h
 */

#include <stdint.h>

// Helper macros to parse the operations from Btns()
#define BUTTON_OP_RIGHT(X)  ((X & 0xF0) >> 4)
#define BUTTON_OP_MIDDLE(X) (X & 0x0F)

namespace MMU3 {

// Will be mapped onto dialog button responses in the FW
// Those responses have their unique+translated texts as well
enum class ButtonOperations : uint8_t {
  NoOperation = 0,
  Retry       = 1,
  Continue    = 2,
  ResetMMU    = 3,
  Unload      = 4,
  Load        = 5,
  Eject       = 6,
  Tune        = 7,
  StopPrint   = 8,
  DisableMMU  = 9,
  MoreInfo    = 10,
};

// Button codes + extended actions performed on the printer's side
enum class Buttons : uint_least8_t {
  Right = 0,
  Middle,
  Left,

  // Performed on the printer's side
  ResetMMU,
  Load,
  Eject,
  StopPrint,
  DisableMMU,
  TuneMMU, // Printer changes MMU register value

  NoButton = 0xFF // should be kept last
};

constexpr uint_least8_t buttons_to_uint8t(Buttons b) {
  return static_cast<uint8_t>(b);
}

} // MMU3
