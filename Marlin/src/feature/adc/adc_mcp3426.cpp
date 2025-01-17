/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * adc_mcp3426.cpp -  library for MicroChip MCP3426 I2C A/D converter
 *
 * For implementation details, please take a look at the datasheet:
 * https://www.microchip.com/en-us/product/MCP3426
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(HAS_MCP3426_ADC)

#include "adc_mcp3426.h"

// Read the ADC value from MCP342X on a specific channel
int16_t MCP3426::ReadValue(uint8_t channel, uint8_t gain, uint8_t address) {
  Error = false;

  #if PINS_EXIST(I2C_SCL, I2C_SDA) && DISABLED(SOFT_I2C_EEPROM)
    Wire.setSDA(pin_t(I2C_SDA_PIN));
    Wire.setSCL(pin_t(I2C_SCL_PIN));
  #endif

  Wire.begin(); // No address joins the BUS as the master

  Wire.beginTransmission(I2C_ADDRESS(address));

  // Continuous Conversion Mode, 16 bit, Channel 1, Gain x4
  // 26 = 0b00011000
  //        RXXCSSGG
  //  R = Ready Bit
  // XX = Channel (00=1, 01=2, 10=3 (MCP3428), 11=4 (MCP3428))
  //  C = Conversion Mode Bit (1=  Continuous Conversion Mode (Default))
  // SS = Sample rate, 10=15 samples per second @ 16 bits
  // GG = Gain 00 =x1
  uint8_t controlRegister = 0b00011000;

  if (channel == 2) controlRegister |= 0b00100000; // Select channel 2

  if (gain == 2)
    controlRegister |= 0b00000001;
  else if (gain == 4)
    controlRegister |= 0b00000010;
  else if (gain == 8)
    controlRegister |= 0b00000011;

  Wire.write(controlRegister);
  if (Wire.endTransmission() != 0) {
    Error = true;
    return 0;
  }

  const uint8_t len = 3;
  uint8_t buffer[len] = {};

  do {
    Wire.requestFrom(I2C_ADDRESS(address), len);
    if (Wire.available() != len) {
      Error = true;
      return 0;
    }

    for (uint_fast8_t i = 0; i < len; ++i)
      buffer[i] = Wire.read();

    // Is conversion ready, if not loop around again
  } while ((buffer[2] & 0x80) != 0);

  union TwoBytesToInt16 {
    uint8_t bytes[2];
    int16_t integervalue;
  };
  TwoBytesToInt16 ConversionUnion;

  ConversionUnion.bytes[1] = buffer[0];
  ConversionUnion.bytes[0] = buffer[1];

  return ConversionUnion.integervalue;
}

MCP3426 mcp3426;

#endif // HAS_MCP3426_ADC
