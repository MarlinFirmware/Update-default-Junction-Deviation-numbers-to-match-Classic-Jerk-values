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

#include "../../inc/MarlinConfig.h"

#if ENABLED(DIGIPOT_MCP4451)

#include "digipot.h"

#include <Stream.h>
#include <Wire.h>

#if MB(MKS_SBASE)
  #include "digipot_mcp4451_I2C_routines.h"
#endif

// Settings for the I2C based DIGIPOT (MCP4451) on Azteeg X3 Pro
#if MB(5DPRINT)
  #define DIGIPOT_I2C_FACTOR     117.96f
  #define DIGIPOT_I2C_MAX_CURRENT 1.736f
#elif MB(AZTEEG_X5_MINI, AZTEEG_X5_MINI_WIFI)
  #define DIGIPOT_I2C_FACTOR      113.5f
  #define DIGIPOT_I2C_MAX_CURRENT   2.0f
#elif MB(AZTEEG_X5_GT)
  #define DIGIPOT_I2C_FACTOR       51.0f
  #define DIGIPOT_I2C_MAX_CURRENT   3.0f
#else
  #define DIGIPOT_I2C_FACTOR      106.7f
  #define DIGIPOT_I2C_MAX_CURRENT   2.5f
#endif

static byte current_to_wiper(const float current) {
  return byte(TERN(DIGIPOT_USE_RAW_VALUES, current, CEIL(DIGIPOT_I2C_FACTOR * current)));
}

static void digipot_i2c_send(const byte addr, const byte a, const byte b) {
  #if MB(MKS_SBASE)
    digipot_mcp4451_start(addr);
    digipot_mcp4451_send_byte(a);
    digipot_mcp4451_send_byte(b);
  #else
    Wire.beginTransmission(I2C_ADDRESS(addr));
    Wire.write(a);
    Wire.write(b);
    Wire.endTransmission();
  #endif
}

// This is for the MCP4451 I2C based digipot
void DigipotI2C::set_current(const uint8_t channel, const float current) {
  // These addresses are specific to Azteeg X3 Pro, can be set to others.
  // In this case first digipot is at address A0=0, A1=0, second one is at A0=0, A1=1
  const byte addr = channel < 4 ? DIGIPOT_I2C_ADDRESS_A : DIGIPOT_I2C_ADDRESS_B; // channel 0-3 vs 4-7

  // Initial setup
  digipot_i2c_send(addr, 0x40, 0xFF);
  digipot_i2c_send(addr, 0xA0, 0xFF);

  // Set actual wiper value
  byte addresses[4] = { 0x00, 0x10, 0x60, 0x70 };
  digipot_i2c_send(addr, addresses[channel & 0x3], current_to_wiper(_MIN(float(_MAX(current, 0)), DIGIPOT_I2C_MAX_CURRENT)));
}

void DigipotI2C::init() {
  #if MB(MKS_SBASE)
    configure_i2c(16); // Set clock_option to 16 ensure I2C is initialized at 400kHz
  #else
    Wire.begin();
  #endif
  // Set up initial currents as defined in Configuration_adv.h
  static const float digipot_motor_current[] PROGMEM =
    #if ENABLED(DIGIPOT_USE_RAW_VALUES)
      DIGIPOT_MOTOR_CURRENT
    #else
      DIGIPOT_I2C_MOTOR_CURRENTS
    #endif
  ;
  for (uint_fast8_t i = 0; i < COUNT(digipot_motor_current); ++i)
    set_current(i, pgm_read_float(&digipot_motor_current[i]));
}

DigipotI2C digipot_i2c;

#endif // DIGIPOT_MCP4451
