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
 * mmu2_error_converter.h
 */

#include <stdint.h>
#include <stddef.h>
#include "mmu_hw/buttons.h"
#include "mmu_hw/error_codes.h"

  namespace MMU3 {

  // Translates MMU3::ErrorCode into an index of Prusa-Error-Codes
  // Basically this is the way to obtain an index into all other functions in this API
  uint8_t PrusaErrorCodeIndex(const ErrorCode ec);

  // @return pointer to a PROGMEM string representing the Title of the Prusa-Error-Codes error
  // @param i index of the error - obtained by calling ErrorCodeIndex
  FSTR_P const PrusaErrorTitle(const uint8_t i);

  // @return pointer to a PROGMEM string representing the multi-page Description of the Prusa-Error-Codes error
  // @param i index of the error - obtained by calling ErrorCodeIndex
  FSTR_P const PrusaErrorDesc(const uint8_t i);

  // @return the actual numerical value of the Prusa-Error-Codes error
  // @param i index of the error - obtained by calling ErrorCodeIndex
  uint16_t PrusaErrorCode(const uint8_t i);

  // @return Btns pair of buttons for a particular Prusa-Error-Codes error
  // @param i index of the error - obtained by calling ErrorCodeIndex
  uint8_t PrusaErrorButtons(const uint8_t i);

  // @return pointer to a PROGMEM string representing the Title of a button
  // @param i index of the error - obtained by calling PrusaErrorButtons + extracting low or high nibble from the Btns pair
  FSTR_P const PrusaErrorButtonTitle(const uint8_t bi);

  // Sets the selected button for later pick-up by the MMU state machine.
  // Used to save the GUI selection/decoupling
  void SetButtonResponse(const ButtonOperations rsp);
  ButtonOperations GetButtonResponse();

  // @return button index/code based on currently processed error/screen
  // Clears the "pressed" button upon exit
  Buttons ButtonPressed(const ErrorCode ec);

  // @return button index/code based on currently processed error/screen
  // Used as a subfunction of ButtonPressed.
  // Does not clear the "pressed" button upon exit
  Buttons ButtonAvailable(const ErrorCode ec);

  } // MMU3
