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

/**
 * mmu2_progress_converter.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "../../core/language.h"
#include "mmu3_progress_converter.h"
#include "mmu_hw/progress_codes.h"
#include "mmu_hw/errors_list.h"

namespace MMU3 {

  FSTR_P const progressTexts[] PROGMEM = {
    GET_TEXT_F(MSG_PROGRESS_OK),              // TODO: Generic messages for Marlin
    GET_TEXT_F(MSG_PROGRESS_ENGAGE_IDLER),    // reused below
    GET_TEXT_F(MSG_PROGRESS_DISENGAGE_IDLER), // reused below
    GET_TEXT_F(MSG_PROGRESS_UNLOAD_FINDA),
    GET_TEXT_F(MSG_PROGRESS_UNLOAD_PULLEY),
    GET_TEXT_F(MSG_PROGRESS_FEED_FINDA),
    GET_TEXT_F(MSG_PROGRESS_FEED_EXTRUDER),
    GET_TEXT_F(MSG_PROGRESS_FEED_NOZZLE),
    GET_TEXT_F(MSG_PROGRESS_AVOID_GRIND),
    GET_TEXT_F(MSG_FINISHING_MOVEMENTS),
    GET_TEXT_F(MSG_PROGRESS_DISENGAGE_IDLER), // err disengaging idler is the same text
    GET_TEXT_F(MSG_PROGRESS_ENGAGE_IDLER),    // engage dtto.
    GET_TEXT_F(MSG_PROGRESS_WAIT_USER),
    GET_TEXT_F(MSG_PROGRESS_ERR_INTERNAL),
    GET_TEXT_F(MSG_PROGRESS_ERR_HELP_FIL),
    GET_TEXT_F(MSG_PROGRESS_ERR_TMC),
    GET_TEXT_F(MSG_UNLOADING_FILAMENT),
    GET_TEXT_F(MSG_LOADING_FILAMENT),
    GET_TEXT_F(MSG_PROGRESS_SELECT_SLOT),
    GET_TEXT_F(MSG_PROGRESS_PREPARE_BLADE),
    GET_TEXT_F(MSG_PROGRESS_PUSH_FILAMENT),
    GET_TEXT_F(MSG_PROGRESS_PERFORM_CUT),
    GET_TEXT_F(MSG_PROGRESSPSTRETURN_SELECTOR),
    GET_TEXT_F(MSG_PROGRESS_PARK_SELECTOR),
    GET_TEXT_F(MSG_PROGRESS_EJECT_FILAMENT),
    GET_TEXT_F(MSG_PROGRESSPSTRETRACT_FINDA),
    GET_TEXT_F(MSG_PROGRESS_HOMING),
    GET_TEXT_F(MSG_PROGRESS_MOVING_SELECTOR),
    GET_TEXT_F(MSG_PROGRESS_FEED_FSENSOR)
  };

  FSTR_P const ProgressCodeToText(const ProgressCode pc) {
    // @@TODO ?? a better fallback option?
    return (int(pc) < COUNT(progressTexts))
     ? static_cast<FSTR_P const>(pgm_read_ptr(&progressTexts[(uint16_t)pc]))
     : static_cast<FSTR_P const>(pgm_read_ptr(&progressTexts[0]));
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
