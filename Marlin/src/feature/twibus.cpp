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

#include "../inc/MarlinConfig.h"

#if ENABLED(EXPERIMENTAL_I2CBUS)

#include "twibus.h"

#include <Wire.h>

#include "../libs/hex_print.h"

TWIBus i2c;

TWIBus::TWIBus() {
  #if I2C_SLAVE_ADDRESS == 0

    #if PINS_EXIST(I2C_SCL, I2C_SDA) && DISABLED(SOFT_I2C_EEPROM)
      Wire.setSDA(pin_t(I2C_SDA_PIN));
      Wire.setSCL(pin_t(I2C_SCL_PIN));
    #endif

    Wire.begin();                   // No address joins the BUS as the master

  #else

    Wire.begin(I2C_SLAVE_ADDRESS);  // Join the bus as a slave

  #endif
  reset();
}

void TWIBus::reset() {
  buffer_s = 0;
  buffer[0] = 0x00;
}

void TWIBus::address(const uint8_t adr) {
  if (!WITHIN(adr, 8, 127))
    SERIAL_ECHO_MSG("Bad I2C address (8-127)");

  addr = adr;

  debug(F("address"), adr);
}

void TWIBus::addbyte(const char c) {
  if (buffer_s >= COUNT(buffer)) return;
  buffer[buffer_s++] = c;
  debug(F("addbyte"), c);
}

void TWIBus::addbytes(char src[], uint8_t bytes) {
  debug(F("addbytes"), bytes);
  while (bytes--) addbyte(*src++);
}

void TWIBus::addstring(char str[]) {
  debug(F("addstring"), str);
  while (char c = *str++) addbyte(c);
}

void TWIBus::send() {
  debug(F("send"), addr);

  Wire.beginTransmission(I2C_ADDRESS(addr));
  Wire.write(buffer, buffer_s);
  Wire.endTransmission();

  reset();
}

// static
void TWIBus::echoprefix(uint8_t bytes, FSTR_P const pref, uint8_t adr) {
  SERIAL_ECHO_START();
  SERIAL_ECHO(pref, F(": from:"), adr, F(" bytes:"), bytes, F(" data:"));
}

// static
void TWIBus::echodata(uint8_t bytes, FSTR_P const pref, uint8_t adr, const uint8_t style/*=0*/) {
  union TwoBytesToInt16 { uint8_t bytes[2]; int16_t integervalue; };
  TwoBytesToInt16 ConversionUnion;

  echoprefix(bytes, pref, adr);

  while (bytes-- && Wire.available()) {
    int value = Wire.read();
    switch (style) {

      // Style 1, HEX DUMP
      case 1:
        SERIAL_CHAR(hex_nybble((value & 0xF0) >> 4));
        SERIAL_CHAR(hex_nybble(value & 0x0F));
        if (bytes) SERIAL_CHAR(' ');
        break;

      // Style 2, signed two byte integer (int16)
      case 2:
        if (bytes == 1)
          ConversionUnion.bytes[1] = (uint8_t)value;
        else if (bytes == 0) {
          ConversionUnion.bytes[0] = (uint8_t)value;
          // Output value in base 10 (standard decimal)
          SERIAL_ECHO(ConversionUnion.integervalue);
        }
        break;

      // Style 3, unsigned byte, base 10 (uint8)
      case 3:
        SERIAL_ECHO(value);
        if (bytes) SERIAL_CHAR(' ');
        break;

      // Default style (zero), raw serial output
      default:
        // This can cause issues with some serial consoles, Pronterface is an example where things go wrong
        SERIAL_CHAR(value);
        break;
    }
  }

  SERIAL_EOL();
}

void TWIBus::echobuffer(FSTR_P const prefix, uint8_t adr) {
  echoprefix(buffer_s, prefix, adr);
  for (uint_fast8_t i = 0; i < buffer_s; ++i) SERIAL_CHAR(buffer[i]);
  SERIAL_EOL();
}

bool TWIBus::request(const uint8_t bytes) {
  if (!addr) return false;

  debug(F("request"), bytes);

  // requestFrom() is a blocking function
  if (Wire.requestFrom(I2C_ADDRESS(addr), bytes) == 0) {
    debug(F("request fail"), I2C_ADDRESS(addr));
    return false;
  }

  return true;
}

void TWIBus::relay(const uint8_t bytes, const uint8_t style/*=0*/) {
  debug(F("relay"), bytes);

  if (request(bytes))
    echodata(bytes, F("i2c-reply"), addr, style);
}

uint8_t TWIBus::capture(char *dst, const uint8_t bytes) {
  reset();
  uint8_t count = 0;
  while (count < bytes && Wire.available())
    dst[count++] = Wire.read();

  debug(F("capture"), count);

  return count;
}

// static
void TWIBus::flush() {
  while (Wire.available()) Wire.read();
}

#if I2C_SLAVE_ADDRESS > 0

  void TWIBus::receive(uint8_t bytes) {
    debug(F("receive"), bytes);
    echodata(bytes, F("i2c-receive"), 0);
  }

  void TWIBus::reply(char str[]/*=nullptr*/) {
    debug(F("reply"), str);

    if (str) {
      reset();
      addstring(str);
    }

    Wire.write(buffer, buffer_s);

    reset();
  }

  void i2c_on_receive(int bytes) { // just echo all bytes received to serial
    i2c.receive(bytes);
  }

  void i2c_on_request() {          // just send dummy data for now
    i2c.reply("Hello World!\n");
  }

#endif

#if ENABLED(DEBUG_TWIBUS)

  // static
  void TWIBus::prefix(FSTR_P const func) {
    SERIAL_ECHOPGM("TWIBus::", func, ": ");
  }
  void TWIBus::debug(FSTR_P const func, uint32_t adr) {
    if (DEBUGGING(INFO)) { prefix(func); SERIAL_ECHOLN(adr); }
  }
  void TWIBus::debug(FSTR_P const func, char c) {
    if (DEBUGGING(INFO)) { prefix(func); SERIAL_ECHOLN(c); }
  }
  void TWIBus::debug(FSTR_P const func, char str[]) {
    if (DEBUGGING(INFO)) { prefix(func); SERIAL_ECHOLN(str); }
  }

#endif

#endif // EXPERIMENTAL_I2CBUS
