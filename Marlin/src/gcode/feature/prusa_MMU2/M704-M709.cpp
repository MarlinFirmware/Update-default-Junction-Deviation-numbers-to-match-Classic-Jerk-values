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

#include "../../../inc/MarlinConfigPre.h"

#if HAS_PRUSA_MMU3

#include "../../gcode.h"
#include "../../../module/settings.h"
#include "../../../feature/mmu3/mmu3.h"
#include "../../../feature/mmu3/mmu3_reporting.h"
#include "../../../feature/mmu3/SpoolJoin.h"

// Shared by the G-codes below to save flash memory.
static void gcodes_M704_M705_M706(uint16_t gcode) {
  const int8_t mmuSlotIndex = parser.intval('P', -1);

  if (mmu3.enabled() && WITHIN(mmuSlotIndex, 0, EXTRUDERS - 1)) {
    switch (gcode) {
      case 704: mmu3.load_to_feeder(mmuSlotIndex); break;
      case 705: mmu3.eject_filament(mmuSlotIndex, false); break;
      case 706:
        #if ENABLED(MMU3_HAS_CUTTER)
          if (mmu3.cutter_mode > 0) mmu3.cut_filament(mmuSlotIndex);
        #endif
        break;
      default: break;
    }
  }
}

/**
 * ### M704 - Preload to MMU
 * #### Usage
 *
 *   M704 [ P ]
 *
 * #### Parameters
 * - `P` - n index of slot (zero based, so 0-4 like T0 and T4)
 */
void GcodeSuite::M704() {
  gcodes_M704_M705_M706(704);
}

/**
 * ### M705 - Eject filament
 * #### Usage
 *
 *   M705 [ P ]
 *
 * #### Parameters
 * - `P` - n index of slot (zero based, so 0-4 like T0 and T4)
 */
void GcodeSuite::M705() {
  gcodes_M704_M705_M706(705);
}

/*!
 * ### M706 - Cut filament
 * #### Usage
 *
 *   M706 [ P ]
 *
 * #### Parameters
 * - `P` - n index of slot (zero based, so 0-4 like T0 and T4)
 */
void GcodeSuite::M706() {
  gcodes_M704_M705_M706(706);
}

/**
 * ### M707 - Read from MMU register
 * #### Usage
 *
 *   M707 [ A ]
 *
 * #### Parameters
 * - `A` - Address of register in hexidecimal.
 *
 * #### Example
 *
 * M707 A0x1b - Read a 8bit integer from register 0x1b and prints the result onto the serial line.
 *
 * Does nothing if the A parameter is not present or if MMU is not enabled.
 *
 */
void GcodeSuite::M707() {
  if (mmu3.enabled() && parser.seenval('A')) {
    char *address = parser.value_string();
    mmu3.readRegister(uint8_t(strtol(address, NULL, 16)));
  }
}

/**
 * ### M708 - Write to MMU register
 * #### Usage
 *
 *   M708 [ A | X ]
 *
 * #### Parameters
 * - `A` - Address of register in hexidecimal.
 * - `X` - Data to write (16-bit integer). Default value 0.
 *
 * #### Example
 * M708 A0x1b X05 - Write to register 0x1b the value 05.
 *
 * Does nothing if A parameter is missing or if MMU is not enabled.
 */
void GcodeSuite::M708() {
  if (mmu3.enabled() && parser.seenval('A')) {
    char *address = parser.value_string();
    const uint8_t addr = uint8_t(strtol(address, NULL, 16));
    if (addr) {
      const uint16_t data = parser.ushortval('X', 0);
      mmu3.writeRegister(addr, data);
    }
  }
}

/**
 * ### M709 - MMU power & reset
 * The MK3S cannot not power off the MMU, but we can en- and disable the MMU.
 *
 * The new state of the MMU is stored in printer's EEPROM.
 * i.e., If you disable the MMU via M709, it will not be activated after the printer resets.
 * Usage
 *
 *   M709 [ S | X ]
 *
 * Parameters
 * - `X` - Reset MMU (0:soft reset | 1:hardware reset | 42: erase MMU eeprom)
 * - `S` - En-/disable the MMU (0:off | 1:on)
 *
 * Examples
 *
 *  M709 X0   ; issue an X0 command via communication into the MMU (soft reset)
 *  M709 X1   ; toggle the MMU's reset pin (hardware reset)
 *  M709 X42  ; erase MMU EEPROM
 *  M709 S1   ; enable MMU
 *  M709 S0   ; disable MMU
 *  M709      ; Serial message if en- or disabled
 */
void GcodeSuite::M709() {
  if (parser.seenval('S')) {
    if (parser.value_bool())
      mmu3.start();
    else
      mmu3.stop();
  }

  if (mmu3.enabled() && parser.seenval('X')) {
    switch (parser.value_byte()) {
      case  0: mmu3.reset(MMU3::MMU3::Software); break;
      case  1: mmu3.reset(MMU3::MMU3::ResetPin); break;
      case 42: mmu3.reset(MMU3::MMU3::EraseEEPROM); break;
      default: break;
    }
  }
  mmu3.status();
}

/**
 * Report for M503.
 * TODO: Report MMU3 G-code settings here, status via a different G-code.
 */
void GcodeSuite::MMU3_report(const bool forReplay/*=true*/) {
  using namespace MMU3;
  report_heading(forReplay, F("MMU3 Operational Stats"));
  SERIAL_ECHOPGM("  MMU                "); serialprintln_onoff(mmu3.mmu_hw_enabled);
  SERIAL_ECHOPGM("  Stealth Mode       "); serialprintln_onoff(mmu3.stealth_mode);
  #if ENABLED(MMU3_HAS_CUTTER)
    SERIAL_ECHOPGM("  Cutter             ");
    serialprintln_onoff(mmu3.cutter_mode != 0);
  #endif
  SERIAL_ECHOPGM("  SpoolJoin          "); serialprintln_onoff(spooljoin.enabled);
  SERIAL_ECHOLNPGM("  Tool Changes       ", operation_statistics.tool_change_counter);
  SERIAL_ECHOLNPGM("  Total Tool Changes ", operation_statistics.tool_change_total_counter);
  SERIAL_ECHOLNPGM("  Fails              ", operation_statistics.fail_num);
  SERIAL_ECHOLNPGM("  Total Fails        ", operation_statistics.fail_total_num);
  SERIAL_ECHOLNPGM("  Load Fails         ", operation_statistics.load_fail_num);
  SERIAL_ECHOLNPGM("  Total Load Fails   ", operation_statistics.load_fail_total_num);
  SERIAL_ECHOLNPGM("  Power Fails        ", mmu3.tmcFailures());
}

#endif // HAS_PRUSA_MMU3
