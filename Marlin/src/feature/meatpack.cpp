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

/**
 * feature/meatpack.cpp
 * MeatPack G-code Compression
 *
 * Algorithm & Implementation: Scott Mudge - mail@scottmudge.com
 * Date: Dec. 2020
 *
 * Character Frequencies from ~30 MB of comment-stripped G-code:
 *  '1' -> 4451136    '4' -> 1353273   '\n' -> 1087683    '-' ->   90242
 *  '0' -> 4253577    '9' -> 1352147    'G' -> 1075806    'Z' ->   34109
 *  ' ' -> 3053297    '3' -> 1262929    'X' ->  975742    'M' ->   11879
 *  '.' -> 3035310    '5' -> 1189871    'E' ->  965275    'S' ->    9910
 *  '2' -> 1523296    '6' -> 1127900    'Y' ->  965274
 *  '8' -> 1366812    '7' -> 1112908    'F' ->   99416
 *
 * When space is omitted the letter 'E' is used in its place
 */

#include "../inc/MarlinConfig.h"

#if HAS_MEATPACK

#include "meatpack.h"

#define MeatPack_ProtocolVersion "PV01"
//#define MP_DEBUG

#define DEBUG_OUT ENABLED(MP_DEBUG)
#include "../core/debug_out.h"

// The 15 most-common characters used in G-code, ~90-95% of all G-code uses these characters
// Stored in SRAM for performance.
uint8_t meatPackLookupTable[16] = {
  '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
  '.', ' ', '\n', 'G', 'X',
  '\0' // Unused. 0b1111 indicates a literal character
};

#if ENABLED(MP_DEBUG)
  uint8_t chars_decoded = 0;  // Log the first 64 bytes after each reset
#endif

void MeatPack::reset_state() {
  state = 0;
  cmd_is_next = false;
  second_char = 0;
  cmd_count = full_char_count = char_out_count = 0;
  TERN_(MP_DEBUG, chars_decoded = 0);
}

/**
 * Unpack one or two characters from a packed byte into a buffer.
 * Return flags indicating whether any literal bytes follow.
 */
uint8_t MeatPack::unpack_chars(const uint8_t pk, uint8_t* __restrict const chars_out) {
  uint8_t out = 0;

  // If lower nybble is 1111, the higher nybble is unused, and next char is full.
  if ((pk & kFirstNotPacked) == kFirstNotPacked)
    out = kFirstCharIsLiteral;
  else {
    const uint8_t chr = pk & 0x0F;
    chars_out[0] = meatPackLookupTable[chr]; // Set the first char
  }

  // Check if upper nybble is 1111... if so, we don't need the second char.
  if ((pk & kSecondNotPacked) == kSecondNotPacked)
    out |= kSecondCharIsLiteral;
  else {
    const uint8_t chr = (pk >> 4) & 0x0F;
    chars_out[1] = meatPackLookupTable[chr]; // Set the second char
  }

  return out;
}

/**
 * Interpret a single (non-command) character
 * according to the current MeatPack state.
 */
void MeatPack::handle_rx_char_inner(const uint8_t c) {
  if (TEST(state, MPConfig_Bit_Active)) {                   // Is MeatPack active?
    if (!full_char_count) {                                 // No literal characters to fetch?
      uint8_t buf[2] = { 0, 0 };
      const uint8_t res = unpack_chars(c, buf);             // Decode the byte into one or two characters.
      if (res & kFirstCharIsLiteral) {                      // The 1st character couldn't be packed.
        ++full_char_count;                                  // So the next stream byte is a full character.
        if (res & kSecondCharIsLiteral) ++full_char_count;  // The 2nd character couldn't be packed. Another stream byte is a full character.
        else second_char = buf[1];                          // Retain the unpacked second character.
      }
      else {
        handle_output_char(buf[0]);                           // Send the unpacked first character out.
        if (buf[0] != '\n') {                                 // After a newline the next char won't be set
          if (res & kSecondCharIsLiteral) ++full_char_count;  // The 2nd character couldn't be packed. The next stream byte is a full character.
          else handle_output_char(buf[1]);                    // Send the unpacked second character out.
        }
      }
    }
    else {
      handle_output_char(c);                                // Pass through the character that couldn't be packed...
      if (second_char) {
        handle_output_char(second_char);                    // ...and send an unpacked 2nd character, if set.
        second_char = 0;
      }
      --full_char_count;                                    // One literal character was consumed
    }
  }
  else // Packing not enabled, just copy character to output
    handle_output_char(c);
}

/**
 * Buffer a single output character which will be picked up in
 * GCodeQueue::get_serial_commands via calls to get_result_char
 */
void MeatPack::handle_output_char(const uint8_t c) {
  char_out_buf[char_out_count++] = c;

  #if ENABLED(MP_DEBUG)
    if (chars_decoded < 1024) {
      ++chars_decoded;
      DEBUG_ECHOLNPGM("RB: ", C(c));
    }
  #endif
}

/**
 * Process a MeatPack command byte to update the state.
 * Report the new state to serial.
 */
void MeatPack::handle_command(const MeatPack_Command c) {
  switch (c) {
    case MPCommand_QueryConfig:     break;
    case MPCommand_EnablePacking:   SBI(state, MPConfig_Bit_Active);   DEBUG_ECHOLNPGM("[MPDBG] ENA REC");   break;
    case MPCommand_DisablePacking:  CBI(state, MPConfig_Bit_Active);   DEBUG_ECHOLNPGM("[MPDBG] DIS REC");   break;
    case MPCommand_ResetAll:        reset_state();                     DEBUG_ECHOLNPGM("[MPDBG] RESET REC"); break;
    case MPCommand_EnableNoSpaces:
      SBI(state, MPConfig_Bit_NoSpaces);
      meatPackLookupTable[kSpaceCharIdx] = kSpaceCharReplace;          DEBUG_ECHOLNPGM("[MPDBG] ENA NSP");   break;
    case MPCommand_DisableNoSpaces:
      CBI(state, MPConfig_Bit_NoSpaces);
      meatPackLookupTable[kSpaceCharIdx] = ' ';                        DEBUG_ECHOLNPGM("[MPDBG] DIS NSP");   break;
    default:                                                           DEBUG_ECHOLNPGM("[MPDBG] UNK CMD REC");
  }
  report_state();
}

void MeatPack::report_state() {
  // NOTE: if any configuration vars are added below, the outgoing sync text for host plugin
  // should not contain the "PV' substring, as this is used to indicate protocol version
  SERIAL_ECHOPGM("[MP] " MeatPack_ProtocolVersion " ");
  serialprint_onoff(TEST(state, MPConfig_Bit_Active));
  SERIAL_ECHO(TEST(state, MPConfig_Bit_NoSpaces) ? F(" NSP\n") : F(" ESP\n"));
}

/**
 * Interpret a single character received from serial
 * according to the current meatpack state.
 */
void MeatPack::handle_rx_char(const uint8_t c, const serial_index_t serial_ind) {
  if (c == kCommandByte) {                // A command (0xFF) byte?
    if (cmd_count) {                      // In fact, two in a row?
      cmd_is_next = true;                 // Then a MeatPack command follows
      cmd_count = 0;
    }
    else
      ++cmd_count; // cmd_count = 1       // One command byte received so far...
    return;
  }

  if (cmd_is_next) {                      // Were two command bytes received?
    PORT_REDIRECT(SERIAL_PORTMASK(serial_ind));
    handle_command((MeatPack_Command)c);  // Then the byte is a MeatPack command
    cmd_is_next = false;
    return;
  }

  if (cmd_count) {                        // Only a single 0xFF was received
    handle_rx_char_inner(kCommandByte);   // A single 0xFF is passed on literally so it can be interpreted as kFirstNotPacked|kSecondNotPacked
    cmd_count = 0;
  }

  handle_rx_char_inner(c);                // Other characters are passed on for MeatPack decoding
}

uint8_t MeatPack::get_result_char(char * const __restrict out) {
  uint8_t res = 0;
  if (char_out_count) {
    res = char_out_count;
    char_out_count = 0;
    for (uint8_t i = 0; i < res; ++i)
      out[i] = (char)char_out_buf[i];
  }
  return res;
}

#endif // HAS_MEATPACK
