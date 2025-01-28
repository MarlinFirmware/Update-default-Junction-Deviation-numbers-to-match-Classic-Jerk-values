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

#pragma once

/**
 * feature/meatpack.h
 * MeatPack G-code Compression
 *
 * Algorithm & Implementation: Scott Mudge - mail@scottmudge.com
 * Date: Dec. 2020
 *
 * Specifically optimized for 3D printing G-Code, this is a zero-cost data compression method
 * which packs ~180-190% more data into the same amount of bytes going to the CNC controller.
 * As a majority of G-Code can be represented by a restricted alphabet, I performed histogram
 * analysis on a wide variety of 3D printing G-code samples, and found ~93% of all G-code could
 * be represented by the same 15-character alphabet.
 *
 * This allowed me to design a system of packing 2 8-bit characters into a single byte, assuming
 * they fall within this limited 15-character alphabet. Using a 4-bit lookup table, these 8-bit
 * characters can be represented by a 4-bit index.
 *
 * Combined with some logic to allow commingling of full-width characters outside of this 15-
 * character alphabet (at the cost of an extra 8-bits per full-width character), and by stripping
 * out unnecessary comments, the end result is G-code which is roughly half the original size.
 *
 * Why did I do this? I noticed micro-stuttering and other data-bottleneck issues while printing
 * objects with high curvature, especially at high speeds. There is also the issue of the limited
 * baud rate provided by Prusa's Atmega2560-based boards, over the USB serial connection. So soft-
 * ware like OctoPrint would also suffer this same micro-stuttering and poor print quality issue.
 *
 */

#include <stdint.h>
#include "../core/serial_hook.h"

/**
 * Commands sent to MeatPack to control its behavior.
 * They are sent by first sending 2x MeatPack_CommandByte (0xFF) in sequence,
 *      followed by one of the command bytes below.
 * Provided that 0xFF is an exceedingly rare character that is virtually never
 * present in G-code naturally, it is safe to assume 2 in sequence should never
 * happen naturally, and so it is used as a signal here.
 *
 * 0xFF *IS* used in "packed" G-code (used to denote that the next 2 characters are
 * full-width), however 2 in a row will never occur, as the next 2 bytes will always
 * some non-0xFF character.
 */
enum MeatPack_Command : uint8_t {
  MPCommand_None            = 0,
  MPCommand_EnablePacking   = 0xFB,
  MPCommand_DisablePacking  = 0xFA,
  MPCommand_ResetAll        = 0xF9,
  MPCommand_QueryConfig     = 0xF8,
  MPCommand_EnableNoSpaces  = 0xF7,
  MPCommand_DisableNoSpaces = 0xF6
};

enum MeatPack_ConfigStateBits : uint8_t {
  MPConfig_Bit_Active   = 0,
  MPConfig_Bit_NoSpaces = 1
};

class MeatPack {

  // Utility definitions
  static const uint8_t kCommandByte         = 0b11111111,
                       kFirstNotPacked      = 0b00001111,
                       kSecondNotPacked     = 0b11110000,
                       kFirstCharIsLiteral  = 0b00000001,
                       kSecondCharIsLiteral = 0b00000010;

  static const uint8_t kSpaceCharIdx = 11;
  static const char kSpaceCharReplace = 'E';

  bool cmd_is_next;        // A command is pending
  uint8_t state;           // Configuration state
  uint8_t second_char;     // Buffers a character if dealing with out-of-sequence pairs
  uint8_t cmd_count,       // Counter of command bytes received (need 2)
          full_char_count, // Counter for full-width characters to be received
          char_out_count;  // Stores number of characters to be read out.
  uint8_t char_out_buf[2]; // Output buffer for caching up to 2 characters

public:
  // Pass in a character rx'd by SD card or serial. Automatically parses command/ctrl sequences,
  // and will control state internally.
  void handle_rx_char(const uint8_t c, const serial_index_t serial_ind);

  /**
   * After passing in rx'd char using above method, call this to get characters out.
   * Can return from 0 to 2 characters at once.
   * @param out [in] Output pointer for unpacked/processed data.
   * @return Number of characters returned. Range from 0 to 2.
   */
  uint8_t get_result_char(char * const __restrict out);

  void reset_state();
  void report_state();
  uint8_t unpack_chars(const uint8_t pk, uint8_t* __restrict const chars_out);
  void handle_command(const MeatPack_Command c);
  void handle_output_char(const uint8_t c);
  void handle_rx_char_inner(const uint8_t c);

  MeatPack() : cmd_is_next(false), state(0), second_char(0), cmd_count(0), full_char_count(0), char_out_count(0) {}
};

// Implement the MeatPack serial class so it's transparent to rest of the code
template <typename SerialT>
struct MeatpackSerial : public SerialBase <MeatpackSerial < SerialT >> {
  typedef SerialBase< MeatpackSerial<SerialT> > BaseClassT;

  SerialT & out;
  MeatPack meatpack;

  char serialBuffer[2];
  uint8_t charCount;
  uint8_t readIndex;

  NO_INLINE void write(uint8_t c)     { out.write(c); }
  void flush()                        { out.flush();  }
  void begin(long br)                 { out.begin(br); readIndex = 0; }
  void end()                          { out.end(); }

  void msgDone()                      { out.msgDone(); }
  // Existing instances implement Arduino's operator bool, so use that if it's available
  bool connected()                    { return Private::HasMember_connected<SerialT>::value ? CALL_IF_EXISTS(bool, &out, connected) : (bool)out; }
  void flushTX()                      { CALL_IF_EXISTS(void, &out, flushTX); }
  SerialFeature features(serial_index_t index) const  { return SerialFeature::MeatPack | CALL_IF_EXISTS(SerialFeature, &out, features, index);  }

  int available(serial_index_t index) {
    if (charCount) return charCount;          // The buffer still has data
    if (out.available(index) <= 0) return 0;  // No data to read

    // Don't read in read method, instead do it here, so we can make progress in the read method
    const int r = out.read(index);
    if (r == -1) return 0;  // This is an error from the underlying serial code
    meatpack.handle_rx_char((uint8_t)r, index);
    charCount = meatpack.get_result_char(serialBuffer);
    readIndex = 0;

    return charCount;
  }

  int readImpl(const serial_index_t index) {
    // Not enough char to make progress?
    if (charCount == 0 && available(index) == 0) return -1;

    charCount--;
    return serialBuffer[readIndex++];
  }

  int read(serial_index_t index)  { return readImpl(index); }
  int available()                 { return available(0); }
  int read()                      { return readImpl(0); }

  MeatpackSerial(const bool e, SerialT & out) : BaseClassT(e), out(out) {}
};
