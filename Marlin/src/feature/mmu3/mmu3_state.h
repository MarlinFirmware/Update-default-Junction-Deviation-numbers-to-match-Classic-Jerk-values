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
 * mmu2_state.h
 */

#include <stdint.h>

namespace MMU3 {
  /**
   * @brief status of mmu
   *
   * States of a printer with the MMU:
   * - Active
   * - Connecting
   * - Stopped
   *
   * When the printer's FW starts, the MMU mode is either Stopped or NotResponding (based on user's preference).
   * When the MMU successfully establishes communication, the state changes to Active.
   */
  enum class xState : uint_fast8_t {
    Active,     //!< MMU has been detected, connected, communicates and is ready to be worked with.
    Connecting, //!< MMU is connected but it doesn't communicate (yet). The user wants the MMU, but it is not ready to be worked with.
    Stopped     //!< The user doesn't want the printer to work with the MMU. The MMU itself is not powered and does not work at all.
  };

} // MMU3
