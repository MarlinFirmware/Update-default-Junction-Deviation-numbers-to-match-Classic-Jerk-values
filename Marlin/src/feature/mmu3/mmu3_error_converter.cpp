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
 * mmu2_error_converter.cpp
 */

#include "../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "../../core/language.h"
#include "mmu3_error_converter.h"
#include "mmu_hw/error_codes.h"
#include "mmu_hw/errors_list.h"

namespace MMU3 {

  static ButtonOperations buttonSelectedOperation = ButtonOperations::NoOperation;

  // we don't have a constexpr find_if in C++17/STL yet
  template <class InputIt, class UnaryPredicate>
  constexpr InputIt find_if_cx(InputIt first, InputIt last, UnaryPredicate p) {
    for (; first != last; ++first) {
      if (p(*first)) return first;
    }
    return last;
  }

  // Making a constexpr FindError should instruct the compiler to optimize the
  // PrusaErrorCodeIndex in such a way that no searching will ever be done at
  // runtime. A call to FindError then compiles to a single instruction even on
  // the AVR.
  // static constexpr uint8_t FindErrorIndex(uint16_t pec) {
  static uint8_t FindErrorIndex(uint16_t pec) {
    constexpr uint16_t errorCodesSize = sizeof(errorCodes) / sizeof(errorCodes[0]);
    constexpr const auto *errorCodesEnd = errorCodes + errorCodesSize;
    const auto *i = find_if_cx(errorCodes, errorCodesEnd, [pec](uint16_t ed) {
      return ed == pec;
    });
    return (i != errorCodesEnd) ? (i - errorCodes) : (errorCodesSize - 1);
  }

  // check that the searching algoritm works
  // static_assert( FindErrorIndex(ERR_MECHANICAL_FINDA_DIDNT_TRIGGER) == 0);
  // static_assert( FindErrorIndex(ERR_MECHANICAL_FINDA_FILAMENT_STUCK) == 1);
  // static_assert( FindErrorIndex(ERR_MECHANICAL_FSENSOR_DIDNT_TRIGGER) == 2);
  // static_assert( FindErrorIndex(ERR_MECHANICAL_FSENSOR_FILAMENT_STUCK) == 3);

  constexpr ErrorCode operator&(ErrorCode a, ErrorCode b) {
    return (ErrorCode)((uint16_t)a & (uint16_t)b);
  }

  constexpr bool ContainsBit(ErrorCode ec, ErrorCode mask) {
    return (uint16_t)ec & (uint16_t)mask;
  }

  uint8_t PrusaErrorCodeIndex(const ErrorCode ec) {
    switch (ec) {
      case ErrorCode::FINDA_DIDNT_SWITCH_ON:
        return FindErrorIndex(ERR_MECHANICAL_FINDA_DIDNT_TRIGGER);
      case ErrorCode::FINDA_DIDNT_SWITCH_OFF:
        return FindErrorIndex(ERR_MECHANICAL_FINDA_FILAMENT_STUCK);
      case ErrorCode::FSENSOR_DIDNT_SWITCH_ON:
        return FindErrorIndex(ERR_MECHANICAL_FSENSOR_DIDNT_TRIGGER);
      case ErrorCode::FSENSOR_DIDNT_SWITCH_OFF:
        return FindErrorIndex(ERR_MECHANICAL_FSENSOR_FILAMENT_STUCK);
      case ErrorCode::FSENSOR_TOO_EARLY:
        return FindErrorIndex(ERR_MECHANICAL_FSENSOR_TOO_EARLY);
      case ErrorCode::FINDA_FLICKERS:
        return FindErrorIndex(ERR_MECHANICAL_INSPECT_FINDA);
      case ErrorCode::LOAD_TO_EXTRUDER_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_LOAD_TO_EXTRUDER_FAILED);
      case ErrorCode::FILAMENT_EJECTED:
        return FindErrorIndex(ERR_SYSTEM_FILAMENT_EJECTED);
      case ErrorCode::FILAMENT_CHANGE:
        return FindErrorIndex(ERR_SYSTEM_FILAMENT_CHANGE);

      case ErrorCode::STALLED_PULLEY:
      case ErrorCode::MOVE_PULLEY_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_PULLEY_CANNOT_MOVE);

      case ErrorCode::HOMING_SELECTOR_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_SELECTOR_CANNOT_HOME);
      case ErrorCode::MOVE_SELECTOR_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_SELECTOR_CANNOT_MOVE);

      case ErrorCode::HOMING_IDLER_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_IDLER_CANNOT_HOME);
      case ErrorCode::MOVE_IDLER_FAILED:
        return FindErrorIndex(ERR_MECHANICAL_IDLER_CANNOT_MOVE);

      case ErrorCode::MMU_NOT_RESPONDING:
        return FindErrorIndex(ERR_CONNECT_MMU_NOT_RESPONDING);
      case ErrorCode::PROTOCOL_ERROR:
        return FindErrorIndex(ERR_CONNECT_COMMUNICATION_ERROR);
      case ErrorCode::FILAMENT_ALREADY_LOADED:
        return FindErrorIndex(ERR_SYSTEM_FILAMENT_ALREADY_LOADED);
      case ErrorCode::INVALID_TOOL:
        return FindErrorIndex(ERR_SYSTEM_INVALID_TOOL);
      case ErrorCode::QUEUE_FULL:
        return FindErrorIndex(ERR_SYSTEM_QUEUE_FULL);
      case ErrorCode::VERSION_MISMATCH:
        return FindErrorIndex(ERR_SYSTEM_FW_UPDATE_NEEDED);
      case ErrorCode::INTERNAL:
        return FindErrorIndex(ERR_SYSTEM_FW_RUNTIME_ERROR);
      case ErrorCode::FINDA_VS_EEPROM_DISREPANCY:
        return FindErrorIndex(ERR_SYSTEM_UNLOAD_MANUALLY);
      case ErrorCode::MCU_UNDERVOLTAGE_VCC:
        return FindErrorIndex(ERR_ELECTRICAL_MMU_MCU_ERROR);
      default: break;
    }

    // Electrical issues which can be detected somehow.
    // Need to be placed before TMC-related errors in order to process couples of error bits between single ones
    // and to keep the code size down.
    if (ContainsBit(ec, ErrorCode::TMC_PULLEY_BIT)) {
      if ((ec & ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION) == ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION)
        return FindErrorIndex(ERR_ELECTRICAL_MMU_PULLEY_SELFTEST_FAILED);
    }
    else if (ContainsBit(ec, ErrorCode::TMC_SELECTOR_BIT)) {
      if ((ec & ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION) == ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION)
        return FindErrorIndex(ERR_ELECTRICAL_MMU_SELECTOR_SELFTEST_FAILED);
    }
    else if (ContainsBit(ec, ErrorCode::TMC_IDLER_BIT)) {
      if ((ec & ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION) == ErrorCode::MMU_SOLDERING_NEEDS_ATTENTION)
        return FindErrorIndex(ERR_ELECTRICAL_MMU_IDLER_SELFTEST_FAILED);
    }

    // TMC-related errors - multiple of these can occur at once
    // - in such a case we report the first which gets found/converted into Prusa-Error-Codes (usually the fact, that one TMC has an issue is serious enough)
    // By carefully ordering the checks here we can prioritize the errors being reported to the user.
    if (ContainsBit(ec, ErrorCode::TMC_PULLEY_BIT)) {
      if (ContainsBit(ec, ErrorCode::TMC_IOIN_MISMATCH))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_PULLEY_DRIVER_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_RESET))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_PULLEY_DRIVER_RESET);
      if (ContainsBit(ec, ErrorCode::TMC_UNDERVOLTAGE_ON_CHARGE_PUMP))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_PULLEY_UNDERVOLTAGE_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_SHORT_TO_GROUND))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_PULLEY_DRIVER_SHORTED);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_WARN))
        return FindErrorIndex(ERR_TEMPERATURE_WARNING_TMC_PULLEY_TOO_HOT);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_ERROR))
        return FindErrorIndex(ERR_TEMPERATURE_TMC_PULLEY_OVERHEAT_ERROR);
    }
    else if (ContainsBit(ec, ErrorCode::TMC_SELECTOR_BIT)) {
      if (ContainsBit(ec, ErrorCode::TMC_IOIN_MISMATCH))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_RESET))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_RESET);
      if (ContainsBit(ec, ErrorCode::TMC_UNDERVOLTAGE_ON_CHARGE_PUMP))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_SELECTOR_UNDERVOLTAGE_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_SHORT_TO_GROUND))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_SHORTED);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_WARN))
        return FindErrorIndex(ERR_TEMPERATURE_WARNING_TMC_SELECTOR_TOO_HOT);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_ERROR))
        return FindErrorIndex(ERR_TEMPERATURE_TMC_SELECTOR_OVERHEAT_ERROR);
    }
    else if (ContainsBit(ec, ErrorCode::TMC_IDLER_BIT)) {
      if (ContainsBit(ec, ErrorCode::TMC_IOIN_MISMATCH))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_IDLER_DRIVER_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_RESET))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_IDLER_DRIVER_RESET);
      if (ContainsBit(ec, ErrorCode::TMC_UNDERVOLTAGE_ON_CHARGE_PUMP))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_IDLER_UNDERVOLTAGE_ERROR);
      if (ContainsBit(ec, ErrorCode::TMC_SHORT_TO_GROUND))
        return FindErrorIndex(ERR_ELECTRICAL_TMC_IDLER_DRIVER_SHORTED);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_WARN))
        return FindErrorIndex(ERR_TEMPERATURE_WARNING_TMC_IDLER_TOO_HOT);
      if (ContainsBit(ec, ErrorCode::TMC_OVER_TEMPERATURE_ERROR))
        return FindErrorIndex(ERR_TEMPERATURE_TMC_IDLER_OVERHEAT_ERROR);
    }

    // if nothing got caught, return a generic runtime error
    return FindErrorIndex(ERR_OTHER_UNKNOWN_ERROR);
  }

  uint16_t PrusaErrorCode(const uint8_t i) { return (uint16_t)pgm_read_word(&errorCodes[i]); }

  FSTR_P const PrusaErrorTitle(const uint8_t i) { return (FSTR_P const)pgm_read_ptr(&errorTitles[i]); }
  FSTR_P const PrusaErrorDesc(const uint8_t i) { return (FSTR_P const)pgm_read_ptr(&errorDescs[i]); }

  uint8_t PrusaErrorButtons(const uint8_t i) { return pgm_read_byte(errorButtons + i); }

  FSTR_P const PrusaErrorButtonTitle(const uint8_t bi) {
    // -1 represents the hidden NoOperation button which is not drawn in any way
    return (FSTR_P const)pgm_read_ptr(&btnOperation[bi - 1]);
  }

  Buttons ButtonPressed(const ErrorCode ec) {
    if (buttonSelectedOperation == ButtonOperations::NoOperation || buttonSelectedOperation == ButtonOperations::MoreInfo)
      return Buttons::NoButton; // no button

    const auto result = ButtonAvailable(ec);
    buttonSelectedOperation = ButtonOperations::NoOperation; // Reset operation

    return result;
  }

  Buttons ButtonAvailable(const ErrorCode ec) {
    uint8_t ei = PrusaErrorCodeIndex(ec);

    // The list of responses which occur in mmu error dialogs
    // Return button index or perform some action on the MK3 by itself (like Reset MMU)
    // Based on Prusa-Error-Codes errors_list.h
    // So far hardcoded, but should be generated in the future
    switch (PrusaErrorCode(ei)) {
      case ERR_MECHANICAL_FINDA_DIDNT_TRIGGER:
      case ERR_MECHANICAL_FINDA_FILAMENT_STUCK:
      case ERR_MECHANICAL_FSENSOR_DIDNT_TRIGGER:
      case ERR_MECHANICAL_FSENSOR_FILAMENT_STUCK:
      case ERR_MECHANICAL_FSENSOR_TOO_EARLY:
      case ERR_MECHANICAL_INSPECT_FINDA:
      case ERR_MECHANICAL_SELECTOR_CANNOT_MOVE:
      case ERR_MECHANICAL_IDLER_CANNOT_MOVE:
      case ERR_MECHANICAL_PULLEY_CANNOT_MOVE:
      case ERR_SYSTEM_UNLOAD_MANUALLY:
        switch (buttonSelectedOperation) {
          // may be allow move selector right and left in the future
          case ButtonOperations::Retry: // "Repeat action"
            return Buttons::Middle;
          default:
            break;
        }
        break;
      case ERR_MECHANICAL_SELECTOR_CANNOT_HOME:
      case ERR_MECHANICAL_IDLER_CANNOT_HOME:
        switch (buttonSelectedOperation) {
          // may be allow move selector right and left in the future
          case ButtonOperations::Tune: // Tune Stallguard threshold
            return Buttons::TuneMMU;
          case ButtonOperations::Retry: // "Repeat action"
            return Buttons::Middle;
          default:
            break;
        }
        break;
      case ERR_MECHANICAL_LOAD_TO_EXTRUDER_FAILED:
      case ERR_SYSTEM_FILAMENT_EJECTED:
        switch (buttonSelectedOperation) {
          case ButtonOperations::Continue: // User solved the serious mechanical problem by hand - there is no other way around
            return Buttons::Middle;
          default:
            break;
        }
        break;
      case ERR_SYSTEM_FILAMENT_CHANGE:
        switch (buttonSelectedOperation) {
          case ButtonOperations::Load:
            return Buttons::Load;
          case ButtonOperations::Eject:
            return Buttons::Eject;
          default:
            break;
        }
        break;
      case ERR_TEMPERATURE_WARNING_TMC_PULLEY_TOO_HOT:
      case ERR_TEMPERATURE_WARNING_TMC_SELECTOR_TOO_HOT:
      case ERR_TEMPERATURE_WARNING_TMC_IDLER_TOO_HOT:
        switch (buttonSelectedOperation) {
          case ButtonOperations::Continue: // "Continue"
            return Buttons::Left;
          case ButtonOperations::ResetMMU: // "Reset MMU"
            return Buttons::ResetMMU;
          default:
            break;
        }
        break;

      case ERR_TEMPERATURE_TMC_PULLEY_OVERHEAT_ERROR:
      case ERR_TEMPERATURE_TMC_SELECTOR_OVERHEAT_ERROR:
      case ERR_TEMPERATURE_TMC_IDLER_OVERHEAT_ERROR:

      case ERR_ELECTRICAL_TMC_PULLEY_DRIVER_ERROR:
      case ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_ERROR:
      case ERR_ELECTRICAL_TMC_IDLER_DRIVER_ERROR:

      case ERR_ELECTRICAL_TMC_PULLEY_DRIVER_RESET:
      case ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_RESET:
      case ERR_ELECTRICAL_TMC_IDLER_DRIVER_RESET:

      case ERR_ELECTRICAL_TMC_PULLEY_UNDERVOLTAGE_ERROR:
      case ERR_ELECTRICAL_TMC_SELECTOR_UNDERVOLTAGE_ERROR:
      case ERR_ELECTRICAL_TMC_IDLER_UNDERVOLTAGE_ERROR:

      case ERR_ELECTRICAL_TMC_PULLEY_DRIVER_SHORTED:
      case ERR_ELECTRICAL_TMC_SELECTOR_DRIVER_SHORTED:
      case ERR_ELECTRICAL_TMC_IDLER_DRIVER_SHORTED:

      case ERR_ELECTRICAL_MMU_PULLEY_SELFTEST_FAILED:
      case ERR_ELECTRICAL_MMU_SELECTOR_SELFTEST_FAILED:
      case ERR_ELECTRICAL_MMU_IDLER_SELFTEST_FAILED:

      case ERR_SYSTEM_QUEUE_FULL:
      case ERR_SYSTEM_FW_RUNTIME_ERROR:
      case ERR_ELECTRICAL_MMU_MCU_ERROR:
        switch (buttonSelectedOperation) {
          case ButtonOperations::ResetMMU: // "Reset MMU"
            return Buttons::ResetMMU;
          default:
            break;
        }
        break;
      case ERR_CONNECT_MMU_NOT_RESPONDING:
      case ERR_CONNECT_COMMUNICATION_ERROR:
      case ERR_SYSTEM_FW_UPDATE_NEEDED:
        switch (buttonSelectedOperation) {
          case ButtonOperations::DisableMMU: // "Disable"
            return Buttons::DisableMMU;
          case ButtonOperations::ResetMMU: // "ResetMMU"
            return Buttons::ResetMMU;
          default:
            break;
        }
        break;
      case ERR_SYSTEM_FILAMENT_ALREADY_LOADED:
        switch (buttonSelectedOperation) {
          case ButtonOperations::Unload: // "Unload"
            return Buttons::Left;
          case ButtonOperations::Continue: // "Proceed/Continue"
            return Buttons::Right;
          default:
            break;
        }
        break;

      case ERR_SYSTEM_INVALID_TOOL:
        switch (buttonSelectedOperation) {
          case ButtonOperations::StopPrint: // "Stop print"
            return Buttons::StopPrint;
          case ButtonOperations::ResetMMU: // "Reset MMU"
            return Buttons::ResetMMU;
          default:
            break;
        }
        break;

      default:
        break;
    }

    return Buttons::NoButton;
  }

  void SetButtonResponse(ButtonOperations rsp) {
    buttonSelectedOperation = rsp;
  }

  ButtonOperations GetButtonResponse() {
    return buttonSelectedOperation;
  }

} // MMU3

#endif // HAS_PRUSA_MMU3
