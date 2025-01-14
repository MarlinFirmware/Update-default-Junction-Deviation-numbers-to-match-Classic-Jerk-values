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
 * errors_list.h
 */

/**
 * Extracted from Prusa-Error-Codes repo
 * Subject to automation and optimization
 * BEWARE - This file should only be included by mmu2_error_converter.cpp!
 */
#include "inttypes.h"
#include "../../../core/language.h"
#include "../../../lcd/marlinui.h"
#ifdef __AVR__
  #include <avr/pgmspace.h>
#endif
#include "buttons.h"
#include "../strlen_cx.h"
#include "../ultralcd.h"

namespace MMU3 {

static constexpr uint8_t ERR_MMU_CODE = 4;

typedef enum : uint16_t {
  ERR_UNDEF = 0,

  ERR_MECHANICAL = 100,
  ERR_MECHANICAL_FINDA_DIDNT_TRIGGER = 101,
  ERR_MECHANICAL_FINDA_FILAMENT_STUCK = 102,
  ERR_MECHANICAL_FSENSOR_DIDNT_TRIGGER = 103,
  ERR_MECHANICAL_FSENSOR_FILAMENT_STUCK = 104,

  ERR_MECHANICAL_PULLEY_CANNOT_MOVE = 105,
  ERR_MECHANICAL_FSENSOR_TOO_EARLY = 106,
  ERR_MECHANICAL_INSPECT_FINDA = 107,
  ERR_MECHANICAL_LOAD_TO_EXTRUDER_FAILED = 108,
  ERR_MECHANICAL_SELECTOR_CANNOT_HOME = 115,
  ERR_MECHANICAL_SELECTOR_CANNOT_MOVE = 116,
  ERR_MECHANICAL_IDLER_CANNOT_HOME = 125,
  ERR_MECHANICAL_IDLER_CANNOT_MOVE = 126,

  ERR_TEMPERATURE = 200,
  ERR_TEMPERATURE_WARNING_TMC_PULLEY_TOO_HOT = 201,
  ERR_TEMPERATURE_WARNING_TMC_SELECTOR_TOO_HOT = 211,
  ERR_TEMPERATURE_WARNING_TMC_IDLER_TOO_HOT = 221,

  ERR_TEMPERATURE_TMC_PULLEY_OVERHEAT_ERROR = 202,
  ERR_TEMPERATURE_TMC_SELECTOR_OVERHEAT_ERROR = 212,
  ERR_TEMPERATURE_TMC_IDLER_OVERHEAT_ERROR = 222,


  ERR_ELECTRICAL = 300,
  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_ERROR = 301,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_ERROR = 311,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_ERROR = 321,

  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_RESET = 302,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_RESET = 312,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_RESET = 322,

  ERR_ELECTRICAL_TMC_PULLEY_UNDERVOLTAGE_ERROR = 303,
  ERR_ELECTRICAL_TMC_SELECTOR_UNDERVOLTAGE_ERROR = 313,
  ERR_ELECTRICAL_TMC_IDLER_UNDERVOLTAGE_ERROR = 323,

  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_SHORTED = 304,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_SHORTED = 314,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_SHORTED = 324,

  ERR_ELECTRICAL_MMU_PULLEY_SELFTEST_FAILED = 305,
  ERR_ELECTRICAL_MMU_SELECTOR_SELFTEST_FAILED = 315,
  ERR_ELECTRICAL_MMU_IDLER_SELFTEST_FAILED = 325,

  ERR_ELECTRICAL_MMU_MCU_ERROR = 306,

  ERR_CONNECT = 400,
  ERR_CONNECT_MMU_NOT_RESPONDING = 401,
  ERR_CONNECT_COMMUNICATION_ERROR = 402,


  ERR_SYSTEM = 500,
  ERR_SYSTEM_FILAMENT_ALREADY_LOADED = 501,
  ERR_SYSTEM_INVALID_TOOL = 502,
  ERR_SYSTEM_QUEUE_FULL = 503,
  ERR_SYSTEM_FW_UPDATE_NEEDED = 504,
  ERR_SYSTEM_FW_RUNTIME_ERROR = 505,
  ERR_SYSTEM_UNLOAD_MANUALLY = 506,
  ERR_SYSTEM_FILAMENT_EJECTED = 507,
  ERR_SYSTEM_FILAMENT_CHANGE = 508,

  ERR_OTHER_UNKNOWN_ERROR = 900
} err_num_t;

// Avr gcc has serious trouble understanding static data structures in PROGMEM
// and inadvertedly falls back to copying the whole structure into RAM (which is obviously unwanted).
// But since this file ought to be generated in the future from yaml prescription,
// it really makes no difference if there are "nice" data structures or plain arrays.
static const constexpr err_num_t errorCodes[] PROGMEM = {
  ERR_MECHANICAL_FINDA_DIDNT_TRIGGER,
  ERR_MECHANICAL_FINDA_FILAMENT_STUCK,
  ERR_MECHANICAL_FSENSOR_DIDNT_TRIGGER,
  ERR_MECHANICAL_FSENSOR_FILAMENT_STUCK,
  ERR_MECHANICAL_PULLEY_CANNOT_MOVE,
  ERR_MECHANICAL_FSENSOR_TOO_EARLY,
  ERR_MECHANICAL_INSPECT_FINDA,
  ERR_MECHANICAL_LOAD_TO_EXTRUDER_FAILED,
  ERR_MECHANICAL_SELECTOR_CANNOT_HOME,
  ERR_MECHANICAL_SELECTOR_CANNOT_MOVE,
  ERR_MECHANICAL_IDLER_CANNOT_HOME,
  ERR_MECHANICAL_IDLER_CANNOT_MOVE,
  ERR_TEMPERATURE_WARNING_TMC_PULLEY_TOO_HOT,
  ERR_TEMPERATURE_WARNING_TMC_SELECTOR_TOO_HOT,
  ERR_TEMPERATURE_WARNING_TMC_IDLER_TOO_HOT,
  ERR_TEMPERATURE_TMC_PULLEY_OVERHEAT_ERROR,
  ERR_TEMPERATURE_TMC_SELECTOR_OVERHEAT_ERROR,
  ERR_TEMPERATURE_TMC_IDLER_OVERHEAT_ERROR,
  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_ERROR,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_ERROR,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_ERROR,
  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_RESET,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_RESET,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_RESET,
  ERR_ELECTRICAL_TMC_PULLEY_UNDERVOLTAGE_ERROR,
  ERR_ELECTRICAL_TMC_SELECTOR_UNDERVOLTAGE_ERROR,
  ERR_ELECTRICAL_TMC_IDLER_UNDERVOLTAGE_ERROR,
  ERR_ELECTRICAL_TMC_PULLEY_DRIVER_SHORTED,
  ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_SHORTED,
  ERR_ELECTRICAL_TMC_IDLER_DRIVER_SHORTED,
  ERR_ELECTRICAL_MMU_PULLEY_SELFTEST_FAILED,
  ERR_ELECTRICAL_MMU_SELECTOR_SELFTEST_FAILED,
  ERR_ELECTRICAL_MMU_IDLER_SELFTEST_FAILED,
  ERR_ELECTRICAL_MMU_MCU_ERROR,
  ERR_CONNECT_MMU_NOT_RESPONDING,
  ERR_CONNECT_COMMUNICATION_ERROR,
  ERR_SYSTEM_FILAMENT_ALREADY_LOADED,
  ERR_SYSTEM_INVALID_TOOL,
  ERR_SYSTEM_QUEUE_FULL,
  ERR_SYSTEM_FW_UPDATE_NEEDED,
  ERR_SYSTEM_FW_RUNTIME_ERROR,
  ERR_SYSTEM_UNLOAD_MANUALLY,
  ERR_SYSTEM_FILAMENT_EJECTED,
  ERR_SYSTEM_FILAMENT_CHANGE,
  ERR_OTHER_UNKNOWN_ERROR
};

FSTR_P const errorTitles[] PROGMEM = {
  GET_TEXT_F(MSG_TITLE_FINDA_DIDNT_TRIGGER),
  GET_TEXT_F(MSG_TITLE_FINDA_FILAMENT_STUCK),
  GET_TEXT_F(MSG_TITLE_FSENSOR_DIDNT_TRIGGER),
  GET_TEXT_F(MSG_TITLE_FSENSOR_FILAMENT_STUCK),
  GET_TEXT_F(MSG_TITLE_PULLEY_CANNOT_MOVE),
  GET_TEXT_F(MSG_TITLE_FSENSOR_TOO_EARLY),
  GET_TEXT_F(MSG_TITLE_INSPECT_FINDA),
  GET_TEXT_F(MSG_TITLE_LOAD_TO_EXTRUDER_FAILED),
  GET_TEXT_F(MSG_TITLE_SELECTOR_CANNOT_HOME),
  GET_TEXT_F(MSG_TITLE_SELECTOR_CANNOT_MOVE),
  GET_TEXT_F(MSG_TITLE_IDLER_CANNOT_HOME),
  GET_TEXT_F(MSG_TITLE_IDLER_CANNOT_MOVE),
  GET_TEXT_F(MSG_TITLE_TMC_WARNING_TMC_TOO_HOT),
  GET_TEXT_F(MSG_TITLE_TMC_WARNING_TMC_TOO_HOT),
  GET_TEXT_F(MSG_TITLE_TMC_WARNING_TMC_TOO_HOT),
  GET_TEXT_F(MSG_TITLE_TMC_OVERHEAT_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_OVERHEAT_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_OVERHEAT_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_RESET),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_RESET),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_RESET),
  GET_TEXT_F(MSG_TITLE_TMC_UNDERVOLTAGE_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_UNDERVOLTAGE_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_UNDERVOLTAGE_ERROR),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_SHORTED),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_SHORTED),
  GET_TEXT_F(MSG_TITLE_TMC_DRIVER_SHORTED),
  GET_TEXT_F(MSG_TITLE_SELFTEST_FAILED),
  GET_TEXT_F(MSG_TITLE_SELFTEST_FAILED),
  GET_TEXT_F(MSG_TITLE_SELFTEST_FAILED),
  GET_TEXT_F(MSG_TITLE_MMU_MCU_ERROR),
  GET_TEXT_F(MSG_TITLE_MMU_NOT_RESPONDING),
  GET_TEXT_F(MSG_TITLE_COMMUNICATION_ERROR),
  GET_TEXT_F(MSG_TITLE_FILAMENT_ALREADY_LOADED),
  GET_TEXT_F(MSG_TITLE_INVALID_TOOL),
  GET_TEXT_F(MSG_TITLE_QUEUE_FULL),
  GET_TEXT_F(MSG_TITLE_FW_UPDATE_NEEDED),
  GET_TEXT_F(MSG_TITLE_FW_RUNTIME_ERROR),
  GET_TEXT_F(MSG_TITLE_UNLOAD_MANUALLY),
  GET_TEXT_F(MSG_TITLE_FILAMENT_EJECTED),
  GET_TEXT_F(MSG_TITLE_FILAMENT_CHANGE),
  GET_TEXT_F(MSG_TITLE_UNKNOWN_ERROR)
};

// @@TODO Looking at the texts, they can be composed of several parts and/or parameterized (could save a lot of space) )
// Moreover, some of them have been disabled in favour of saving some more code size.

FSTR_P const errorDescs[] PROGMEM = {
  GET_TEXT_F(MSG_DESC_FINDA_DIDNT_TRIGGER),
  GET_TEXT_F(MSG_DESC_FINDA_FILAMENT_STUCK),
  GET_TEXT_F(MSG_DESC_FSENSOR_DIDNT_TRIGGER),
  GET_TEXT_F(MSG_DESC_FSENSOR_FILAMENT_STUCK),
  GET_TEXT_F(MSG_DESC_PULLEY_CANNOT_MOVE),
  GET_TEXT_F(MSG_DESC_FSENSOR_TOO_EARLY),
  GET_TEXT_F(MSG_DESC_INSPECT_FINDA),
  GET_TEXT_F(MSG_DESC_LOAD_TO_EXTRUDER_FAILED),
  GET_TEXT_F(MSG_DESC_SELECTOR_CANNOT_HOME),
  GET_TEXT_F(MSG_DESC_CANNOT_MOVE),
  GET_TEXT_F(MSG_DESC_IDLER_CANNOT_HOME),
  GET_TEXT_F(MSG_DESC_CANNOT_MOVE),
  GET_TEXT_F(MSG_DESC_TMC), // WARNING_TMC_PULLEY_TOO_HOT
  GET_TEXT_F(MSG_DESC_TMC), // WARNING_TMC_SELECTOR_TOO_HOT
  GET_TEXT_F(MSG_DESC_TMC), // WARNING_TMC_IDLER_TOO_HOT
  GET_TEXT_F(MSG_DESC_TMC), // TMC_PULLEY_OVERHEAT_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_SELECTOR_OVERHEAT_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_IDLER_OVERHEAT_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_PULLEY_DRIVER_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_SELECTOR_DRIVER_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_IDLER_DRIVER_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_PULLEY_DRIVER_RESET
  GET_TEXT_F(MSG_DESC_TMC), // TMC_SELECTOR_DRIVER_RESET
  GET_TEXT_F(MSG_DESC_TMC), // TMC_IDLER_DRIVER_RESET
  GET_TEXT_F(MSG_DESC_TMC), // TMC_PULLEY_UNDERVOLTAGE_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_SELECTOR_UNDERVOLTAGE_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_IDLER_UNDERVOLTAGE_ERROR
  GET_TEXT_F(MSG_DESC_TMC), // TMC_PULLEY_DRIVER_SHORTED
  GET_TEXT_F(MSG_DESC_TMC), // TMC_SELECTOR_DRIVER_SHORTED
  GET_TEXT_F(MSG_DESC_TMC), // TMC_IDLER_DRIVER_SHORTED
  GET_TEXT_F(MSG_DESC_TMC), // MMU_PULLEY_SELFTEST_FAILED
  GET_TEXT_F(MSG_DESC_TMC), // MMU_SELECTOR_SELFTEST_FAILED
  GET_TEXT_F(MSG_DESC_TMC), // MMU_IDLER_SELFTEST_FAILED
  GET_TEXT_F(MSG_DESC_TMC), // MSG_DESC_MMU_MCU_ERROR
  GET_TEXT_F(MSG_DESC_MMU_NOT_RESPONDING),
  GET_TEXT_F(MSG_DESC_COMMUNICATION_ERROR),
  GET_TEXT_F(MSG_DESC_FILAMENT_ALREADY_LOADED),
  GET_TEXT_F(MSG_DESC_INVALID_TOOL),
  GET_TEXT_F(MSG_DESC_QUEUE_FULL),
  GET_TEXT_F(MSG_DESC_FW_UPDATE_NEEDED),
  GET_TEXT_F(MSG_DESC_FW_RUNTIME_ERROR),
  GET_TEXT_F(MSG_DESC_UNLOAD_MANUALLY),
  GET_TEXT_F(MSG_DESC_FILAMENT_EJECTED),
  GET_TEXT_F(MSG_DESC_FILAMENT_CHANGE),
  GET_TEXT_F(MSG_DESC_UNKNOWN_ERROR)
};

// We have max 3 buttons/operations to select from.
// One of them is "More" to show the explanation text normally hidden in the next screens.
// It is displayed with W (Double down arrow, special character from CGRAM)
// 01234567890123456789
// >bttxt   >bttxt   >W
// Therefore at least some of the buttons, which can occur on the screen together, can only be 8-chars long max @@TODO.
// Beware - we only have space for 2 buttons on the LCD while the MMU has 3 buttons
// -> the left button on the MMU is not used/rendered on the LCD (it is also almost unused on the MMU side)

// Used to parse the buttons from Btns().
FSTR_P const btnOperation[] PROGMEM = {
  GET_TEXT_F(MSG_BTN_RETRY),
  GET_TEXT_F(MSG_DONE),
  GET_TEXT_F(MSG_BTN_RESET_MMU),
  GET_TEXT_F(MSG_BTN_UNLOAD),
  GET_TEXT_F(MSG_BTN_LOAD),
  GET_TEXT_F(MSG_BTN_EJECT),
  GET_TEXT_F(MSG_TUNE),
  GET_TEXT_F(MSG_BTN_STOP),
  GET_TEXT_F(MSG_BTN_DISABLE_MMU)
};

// We have 8 different operations/buttons at this time, so we need at least 4 bits to encode each.
// Since one of the buttons is always "More", we can skip that one.
// Therefore we need just 1 byte to describe the necessary buttons for each screen.
uint8_t constexpr Btns(ButtonOperations bMiddle, ButtonOperations bRight) {
  return ((uint8_t)bRight) << 4 | ((uint8_t)bMiddle);
}

static const uint8_t errorButtons[] PROGMEM = {
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // FINDA_DIDNT_TRIGGER
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // FINDA_FILAMENT_STUCK
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // FSENSOR_DIDNT_TRIGGER
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // FSENSOR_FILAMENT_STUCK

  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // PULLEY_CANNOT_MOVE
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // FSENSOR_TOO_EARLY
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // INSPECT_FINDA
  Btns(ButtonOperations::Continue, ButtonOperations::NoOperation), // LOAD_TO_EXTRUDER_FAILED
  Btns(ButtonOperations::Retry, ButtonOperations::Tune),           // SELECTOR_CANNOT_HOME
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // SELECTOR_CANNOT_MOVE
  Btns(ButtonOperations::Retry, ButtonOperations::Tune),           // IDLER_CANNOT_HOME
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // IDLER_CANNOT_MOVE

  Btns(ButtonOperations::Continue, ButtonOperations::ResetMMU),    // WARNING_TMC_PULLEY_TOO_HOT
  Btns(ButtonOperations::Continue, ButtonOperations::ResetMMU),    // WARNING_TMC_SELECTOR_TOO_HOT
  Btns(ButtonOperations::Continue, ButtonOperations::ResetMMU),    // WARNING_TMC_IDLER_TOO_HOT

  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_PULLEY_OVERHEAT_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_SELECTOR_OVERHEAT_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_IDLER_OVERHEAT_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_PULLEY_DRIVER_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_SELECTOR_DRIVER_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_IDLER_DRIVER_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_PULLEY_DRIVER_RESET
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_SELECTOR_DRIVER_RESET
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_IDLER_DRIVER_RESET
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_PULLEY_UNDERVOLTAGE_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_SELECTOR_UNDERVOLTAGE_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_IDLER_UNDERVOLTAGE_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_PULLEY_DRIVER_SHORTED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_SELECTOR_DRIVER_SHORTED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // TMC_IDLER_DRIVER_SHORTED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // MMU_PULLEY_SELFTEST_FAILED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // MMU_SELECTOR_SELFTEST_FAILED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // MMU_IDLER_SELFTEST_FAILED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // MMU_MCU_ERROR
  Btns(ButtonOperations::ResetMMU, ButtonOperations::DisableMMU),  // MMU_NOT_RESPONDING
  Btns(ButtonOperations::ResetMMU, ButtonOperations::DisableMMU),  // COMMUNICATION_ERROR

  Btns(ButtonOperations::Unload, ButtonOperations::Continue),      // FILAMENT_ALREADY_LOADED
  Btns(ButtonOperations::StopPrint, ButtonOperations::ResetMMU),   // INVALID_TOOL
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // QUEUE_FULL
  Btns(ButtonOperations::ResetMMU, ButtonOperations::DisableMMU),  // FW_UPDATE_NEEDED
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // FW_RUNTIME_ERROR
  Btns(ButtonOperations::Retry, ButtonOperations::NoOperation),    // UNLOAD_MANUALLY
  Btns(ButtonOperations::Continue, ButtonOperations::NoOperation), // FILAMENT_EJECTED
  Btns(ButtonOperations::Eject, ButtonOperations::Load),           // FILAMENT_CHANGE
  Btns(ButtonOperations::ResetMMU, ButtonOperations::NoOperation), // UNKOWN_ERROR
};

static_assert(COUNT(errorCodes) == COUNT(errorDescs));
static_assert(COUNT(errorCodes) == COUNT(errorTitles));
static_assert(COUNT(errorCodes) == COUNT(errorButtons));

} // MMU3
