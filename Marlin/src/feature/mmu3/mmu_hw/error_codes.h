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
 * error_codes.h
 */

#include <stdint.h>

/**
 * A complete set of error codes which may be a result of a high-level command/operation.
 * This header file should be included in the printer's firmware as well as a reference,
 * therefore the error codes have been extracted to one place.
 *
 * Please note the errors are intentionally coded as "negative" values (highest bit set),
 * becase they are a complement to reporting the state of the high-level state machines -
 * positive values are considered as normal progress, negative values are errors.
 *
 * Please note, that multiple TMC errors can occur at once, thus they are defined as a bitmask of the higher byte.
 * Also, as there are 3 TMC drivers on the board, each error is added a bit for the corresponding TMC -
 * TMC_PULLEY_BIT, TMC_SELECTOR_BIT, TMC_IDLER_BIT,
 * The resulting error is a bitwise OR over 3 TMC drivers and their status, which should cover most of the situations correctly.
 */
enum class ErrorCode : uint_fast16_t {
  RUNNING                    = 0x0000, //!< the operation is still running - keep this value as ZERO as it is used for initialization of error codes as well
  OK                         = 0x0001, //!< the operation finished OK

  // TMC bit masks
  TMC_PULLEY_BIT             = 0x0040, //!< +64 TMC Pulley bit
  TMC_SELECTOR_BIT           = 0x0080, //!< +128 TMC Pulley bit
  TMC_IDLER_BIT              = 0x0100, //!< +256 TMC Pulley bit

  // Unload Filament related error codes
  FINDA_DIDNT_SWITCH_ON      = 0x8001, //!< E32769 FINDA didn't switch on while loading filament - either there is something blocking the metal ball or a cable is broken/disconnected
  FINDA_DIDNT_SWITCH_OFF     = 0x8002, //!< E32770 FINDA didn't switch off while unloading filament

  FSENSOR_DIDNT_SWITCH_ON    = 0x8003, //!< E32771 Filament sensor didn't switch on while performing LoadFilament
  FSENSOR_DIDNT_SWITCH_OFF   = 0x8004, //!< E32772 Filament sensor didn't switch off while performing UnloadFilament

  FILAMENT_ALREADY_LOADED    = 0x8005, //!< E32773 cannot perform operation LoadFilament or move the selector as the filament is already loaded

  INVALID_TOOL               = 0x8006, //!< E32774 tool/slot index out of range (typically issuing T5 into an MMU with just 5 slots - valid range 0-4)

  HOMING_FAILED              = 0x8007, //!< generic homing failed error - always reported with the corresponding axis bit set (Idler or Selector) as follows:
  HOMING_SELECTOR_FAILED     = HOMING_FAILED | TMC_SELECTOR_BIT, //!< E32903 the Selector was unable to home properly - that means something is blocking its movement
  HOMING_IDLER_FAILED        = HOMING_FAILED | TMC_IDLER_BIT, //!< E33031 the Idler was unable to home properly - that means something is blocking its movement
  STALLED_PULLEY             = HOMING_FAILED | TMC_PULLEY_BIT, //!< E32839 for the Pulley "homing" means just StallGuard detected during Pulley's operation (Pulley doesn't home)

  FINDA_VS_EEPROM_DISREPANCY = 0x8008, //!< E32776 FINDA is pressed but we have no such record in EEPROM - this can only happen at the start of the MMU and can be resolved by issuing an Unload command

  FSENSOR_TOO_EARLY          = 0x8009, //!< E32777 FSensor triggered while doing FastFeedToBondtech - that means either:
  //!< - the PTFE is too short
  //!< - a piece of filament was left inside - pushed in front of the loaded filament causing the fsensor trigger too early
  //!< - fsensor is faulty producing bogus triggers

  FINDA_FLICKERS             = 0x800A, //!< FINDA flickers - seems to be badly calibrated and happens to be pressed at spots where it used to be not pressed before.
  //!< The user is obliged to inspect FINDA and tune its switching

  MOVE_FAILED                = 0x800B, //!< generic move failed error - always reported with the corresponding axis bit set (Idler or Selector) as follows:
  MOVE_SELECTOR_FAILED       = MOVE_FAILED | TMC_SELECTOR_BIT, //!< E32905 the Selector was unable to move to desired position properly - that means something is blocking its movement, e.g. a piece of filament got out of pulley body
  MOVE_IDLER_FAILED          = MOVE_FAILED | TMC_IDLER_BIT, //!< E33033 the Idler was unable to move - unused at the time of creation, but added for completeness
  MOVE_PULLEY_FAILED         = MOVE_FAILED | TMC_PULLEY_BIT, //!< E32841 the Pulley was unable to move - unused at the time of creation, but added for completeness

  FILAMENT_EJECTED           = 0x800C, //!< Filament was ejected, waiting for user input - technically, this is not an error

  MCU_UNDERVOLTAGE_VCC       = 0x800D, //!< MCU VCC rail undervoltage.

  FILAMENT_CHANGE            = 0x8029, //!< E32809 internal error of the printer - try-load-unload sequence detected missing filament -> failed load into the nozzle
  LOAD_TO_EXTRUDER_FAILED    = 0x802A, //!< E32810 internal error of the printer - try-load-unload sequence detected missing filament -> failed load into the nozzle
  QUEUE_FULL                 = 0x802B, //!< E32811 internal logic error - attempt to move with a full queue
  VERSION_MISMATCH           = 0x802C, //!< E32812 internal error of the printer - incompatible version of the MMU FW
  PROTOCOL_ERROR             = 0x802D, //!< E32813 internal error of the printer - communication with the MMU got garbled - protocol decoder couldn't decode the incoming messages
  MMU_NOT_RESPONDING         = 0x802E, //!< E32814 internal error of the printer - communication with the MMU is not working
  INTERNAL                   = 0x802F, //!< E32815 internal runtime error (software)

  // TMC driver init error - TMC dead or bad communication
  // - E33344 Pulley TMC driver
  // - E33408 Selector TMC driver
  // - E33536 Idler TMC driver
  // - E33728 All 3 TMC driver
  TMC_IOIN_MISMATCH = 0x8200,

  // TMC driver reset - recoverable, we just need to rehome the axis
  // Idler: can be rehomed any time
  // Selector: if there is a filament, remove it and rehome, if there is no filament, just rehome
  // Pulley: do nothing - for the loading sequence - just restart and move slowly, for the unload sequence just restart
  // - E33856 Pulley TMC driver
  // - E33920 Selector TMC driver
  // - E34048 Idler TMC driver
  // - E34240 All 3 TMC driver
  TMC_RESET = 0x8400,

  // not enough current for the TMC, NOT RECOVERABLE
  // - E34880 Pulley TMC driver
  // - E34944 Selector TMC driver
  // - E35072 Idler TMC driver
  // - E35264 All 3 TMC driver
  TMC_UNDERVOLTAGE_ON_CHARGE_PUMP = 0x8800,

  // TMC driver serious error - short to ground on coil A or coil B - dangerous to recover
  // - E36928 Pulley TMC driver
  // - E36992 Selector TMC driver
  // - E37120 Idler TMC driver
  // - E37312 All 3 TMC driver
  TMC_SHORT_TO_GROUND = 0x9000,

  // TMC driver over temperature warning - can be recovered by restarting the driver.
  // If this error happens, we should probably go into the error state as soon as the current command is finished.
  // The driver technically still works at this point.
  // - E41024 Pulley TMC driver
  // - E41088 Selector TMC driver
  // - E41216 Idler TMC driver
  // - E41408 All 3 TMC driver
  TMC_OVER_TEMPERATURE_WARN = 0xA000,

  // TMC driver over temperature error - we really shouldn't ever reach this error.
  // It can still be recovered if the driver cools down below 120C.
  // The driver needs to be disabled and enabled again for operation to resume after this error is cleared.
  // - E49216 Pulley TMC driver
  // - E49280 Selector TMC driver
  // - E49408 Idler TMC driver
  // - E49600 All 3 TMC driver
  TMC_OVER_TEMPERATURE_ERROR = 0xC000,

  // TMC driver - IO pins are unreliable. While in theory it's recoverable, in practice it most likely
  // means your hardware is borked (we can't command the drivers reliably via STEP/EN/DIR due to electrical
  // issues or hardware fault. Possible "fixable" cause is undervoltage on the 5v logic line.
  // Unfixable possible cause: bad or cracked solder joints on the PCB, failed shift register, failed driver.
  MMU_SOLDERING_NEEDS_ATTENTION = 0xC200,

};
