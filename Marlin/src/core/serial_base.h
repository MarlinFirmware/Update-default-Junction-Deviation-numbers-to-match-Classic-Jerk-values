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

#include "../inc/MarlinConfigPre.h"

#include <stddef.h> // for size_t

#if ENABLED(EMERGENCY_PARSER)
  #include "../feature/e_parser.h"
#endif

// Used in multiple places
// You can build it but not manipulate it.
// There are only few places where it's required to access the underlying member: GCodeQueue, SerialMask and MultiSerial
struct serial_index_t {
  // A signed index, where -1 is a special case meaning no action (neither output or input)
  int8_t  index;

  // Check if the index is within the range [a ... b]
  constexpr inline bool within(const int8_t a, const int8_t b) const { return WITHIN(index, a, b); }
  constexpr inline bool valid() const { return WITHIN(index, 0, 7); } // At most, 8 bits

  // Construction is either from an index
  constexpr serial_index_t(const int8_t index) : index(index) {}

  // Default to "no index"
  constexpr serial_index_t() : index(-1) {}
};

// In order to catch usage errors in code, we make the base to encode number explicit
// If given a number (and not this enum), the compiler will reject the overload, falling back to the (double, digit) version
// We don't want hidden conversion of the first parameter to double, so it has to be as hard to do for the compiler as creating this enum
enum class PrintBase {
  Dec = 10,
  Hex = 16,
  Oct = 8,
  Bin = 2
};

// A simple feature list enumeration
enum class SerialFeature {
  None                = 0x00,
  MeatPack            = 0x01,   //!< Enabled when Meatpack is present
  BinaryFileTransfer  = 0x02,   //!< Enabled for BinaryFile transfer support (in the future)
  Virtual             = 0x04,   //!< Enabled for virtual serial port (like Telnet / Websocket / ...)
  Hookable            = 0x08,   //!< Enabled if the serial class supports a setHook method
};
ENUM_FLAGS(SerialFeature);

// flushTX is not implemented in all HAL, so use SFINAE to call the method where it is.
CALL_IF_EXISTS_IMPL(void, flushTX);
CALL_IF_EXISTS_IMPL(bool, connected, true);
CALL_IF_EXISTS_IMPL(SerialFeature, features, SerialFeature::None);

// A simple forward struct to prevent the compiler from selecting print(double, int) as a default overload
// for any type other than double/float. For double/float, a conversion exists so the call will be invisible.
struct EnsureDouble {
  double a;
  operator double() { return a; }
  // If the compiler breaks on ambiguity here, it's likely because print(X, base) is called with X not a double/float, and
  // a base that's not a PrintBase value. This code is made to detect the error. You MUST set a base explicitly like this:
  //SERIAL_PRINT(v, PrintBase::Hex)
  EnsureDouble(double a) : a(a) {}
  EnsureDouble(float a) : a(a) {}
};

// Using Curiously-Recurring Template Pattern here to avoid virtual table cost when compiling.
// Since the real serial class is known at compile time, this results in the compiler writing
// a completely efficient code.
template <class Child>
struct SerialBase {
  #if ENABLED(EMERGENCY_PARSER)
    const bool ep_enabled;
    EmergencyParser::State emergency_state;
    inline bool emergency_parser_enabled() { return ep_enabled; }
    SerialBase(bool ep_capable) : ep_enabled(ep_capable), emergency_state(EmergencyParser::State::EP_RESET) {}
  #else
    SerialBase(const bool) {}
  #endif

  #define SerialChild static_cast<Child*>(this)

  // Static dispatch methods below:
  // The most important method here is where it all ends to:
  void write(uint8_t c)             { SerialChild->write(c); }

  // Called when the parser finished processing an instruction, usually build to nothing
  void msgDone() const              { SerialChild->msgDone(); }

  // Called on initialization
  void begin(const long baudRate)   { SerialChild->begin(baudRate); }

  // Called on destruction
  void end()                        { SerialChild->end(); }

  /** Check for available data from the port
      @param index  The port index, usually 0 */
  int available(serial_index_t index=0) const { return SerialChild->available(index); }

  /** Read a value from the port
      @param index  The port index, usually 0 */
  int read(serial_index_t index=0)        { return SerialChild->read(index); }

  /** Combine the features of this serial instance and return it
      @param index  The port index, usually 0 */
  SerialFeature features(serial_index_t index=0) const { return static_cast<const Child*>(this)->features(index);  }

  // Check if the serial port has a feature
  bool has_feature(serial_index_t index, SerialFeature flag) const { return (features(index) & flag) != SerialFeature::None; }

  // Check if the serial port is connected (usually bypassed)
  bool connected() const            { return SerialChild->connected(); }

  // Redirect flush
  void flush()                      { SerialChild->flush(); }

  // Not all implementation have a flushTX, so let's call them only if the child has the implementation
  void flushTX()                    { CALL_IF_EXISTS(void, SerialChild, flushTX); }

  // Glue code here
  void write(const char *str)                    { while (*str) write(*str++); }
  void write(const uint8_t *buffer, size_t size) { while (size--) write(*buffer++); }
  void print(char *str)                          { write(str); }
  void print(const char *str)                    { write(str); }
  // No default argument to avoid ambiguity

  // Define print for every fundamental integer type, to ensure that all redirect properly
  // to the correct underlying implementation.

  // Prints are performed with a single size, to avoid needing multiple print functions.
  // The fixed integer size used for prints will be the larger of long or a pointer.
  #if __LONG_WIDTH__ >= __INTPTR_WIDTH__
    typedef long int_fixed_print_t;
    typedef unsigned long uint_fixed_print_t;
  #else
    typedef intptr_t int_fixed_print_t;
    typedef uintptr_t uint_fixed_print_t;

    FORCE_INLINE void print(intptr_t c, PrintBase base)         { printNumber_signed(c, base); }
    FORCE_INLINE void print(uintptr_t c, PrintBase base)        { printNumber_unsigned(c, base); }
  #endif

  FORCE_INLINE void print(char c, PrintBase base)               { printNumber_signed(c, base); }
  FORCE_INLINE void print(short c, PrintBase base)              { printNumber_signed(c, base); }
  FORCE_INLINE void print(int c, PrintBase base)                { printNumber_signed(c, base); }
  FORCE_INLINE void print(long c, PrintBase base)               { printNumber_signed(c, base); }
  FORCE_INLINE void print(unsigned char c, PrintBase base)      { printNumber_unsigned(c, base); }
  FORCE_INLINE void print(unsigned short c, PrintBase base)     { printNumber_unsigned(c, base); }
  FORCE_INLINE void print(unsigned int c, PrintBase base)       { printNumber_unsigned(c, base); }
  FORCE_INLINE void print(unsigned long c, PrintBase base)      { printNumber_unsigned(c, base); }

  void print(EnsureDouble c, int digits)           { printFloat(c, digits); }

  // Forward the call to the former's method

  // Default implementation for anything without a specialization
  // This handles integers since they are the most common
  template <typename T>
  void print(T c)    { print(c, PrintBase::Dec); }

  void print(float c)    { print(c, 2); }
  void print(double c)   { print(c, 2); }

  void println(char *s)               { print(s); println(); }
  void println(const char *s)         { print(s); println(); }
  void println(float c, int digits)   { print(c, digits); println(); }
  void println(double c, int digits)  { print(c, digits); println(); }
  void println()                      { write('\r'); write('\n'); }

  // Default implementations for types without a specialization. Handles integers.
  template <typename T>
  void println(T c, PrintBase base)   { print(c, base); println(); }

  template <typename T>
  void println(T c)                   { println(c, PrintBase::Dec); }

  // Forward the call to the former's method
  void println(float c)               { println(c, 2); }
  void println(double c)              { println(c, 2); }

  // Print a number with the given base
  NO_INLINE void printNumber_unsigned(uint_fixed_print_t n, PrintBase base) {
    if (n) {
      unsigned char buf[8 * sizeof(long)]; // Enough space for base 2
      int8_t i = 0;
      while (n) {
        buf[i++] = n % (uint_fixed_print_t)base;
        n /= (uint_fixed_print_t)base;
      }
      while (i--) write((char)(buf[i] + (buf[i] < 10 ? '0' : 'A' - 10)));
    }
    else write('0');
  }

  NO_INLINE void printNumber_signed(int_fixed_print_t n, PrintBase base) {
    if (base == PrintBase::Dec && n < 0) {
      n = -n; // This works because all platforms Marlin's builds on are using 2-complement encoding for negative number
              // On such CPU, changing the sign of a number is done by inverting the bits and adding one, so if n = 0x80000000 = -2147483648 then
              // -n = 0x7FFFFFFF + 1 => 0x80000000 = 2147483648 (if interpreted as unsigned) or -2147483648 if interpreted as signed.
              // On non 2-complement CPU, there would be no possible representation for 2147483648.
      write('-');
    }
    printNumber_unsigned((uint_fixed_print_t)n , base);
  }

  // Print a decimal number
  NO_INLINE void printFloat(double number, uint8_t digits) {
    // Handle negative numbers
    if (number < 0.0) {
      write('-');
      number = -number;
    }

    // Round correctly so that print(1.999, 2) prints as "2.00"
    double rounding = 0.5;
    for (uint8_t i = 0; i < digits; ++i) rounding *= 0.1;
    number += rounding;

    // Extract the integer part of the number and print it
    unsigned long int_part = (unsigned long)number;
    double remainder = number - (double)int_part;
    printNumber_unsigned(int_part, PrintBase::Dec);

    // Print the decimal point, but only if there are digits beyond
    if (digits) {
      write('.');
      // Extract digits from the remainder one at a time
      while (digits--) {
        remainder *= 10.0;
        unsigned long toPrint = (unsigned long)remainder;
        printNumber_unsigned(toPrint, PrintBase::Dec);
        remainder -= toPrint;
      }
    }
  }
};

// All serial instances will be built by chaining the features required
// for the function in the form of a template type definition.
