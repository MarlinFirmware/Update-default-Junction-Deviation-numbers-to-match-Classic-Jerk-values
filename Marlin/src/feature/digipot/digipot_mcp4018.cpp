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

#if ENABLED(DIGIPOT_MCP4018)

#include "digipot.h"

#include <Stream.h>
#include <SlowSoftI2CMaster.h>  // https://github.com/felias-fogg/SlowSoftI2CMaster

// Settings for the I2C based DIGIPOT (MCP4018) based on WT150

#ifndef DIGIPOT_A4988_Rsx
  #define DIGIPOT_A4988_Rsx             0.250
#endif
#ifndef DIGIPOT_A4988_Vrefmax
  #define DIGIPOT_A4988_Vrefmax         1.666
#endif
#define DIGIPOT_MCP4018_MAX_VALUE     127

#define DIGIPOT_A4988_Itripmax(Vref)    ((Vref) / (8.0 * DIGIPOT_A4988_Rsx))

#define DIGIPOT_A4988_FACTOR            ((DIGIPOT_MCP4018_MAX_VALUE) / DIGIPOT_A4988_Itripmax(DIGIPOT_A4988_Vrefmax))
#define DIGIPOT_A4988_MAX_CURRENT       2.0

static byte current_to_wiper(const float current) {
  const int16_t value = TERN(DIGIPOT_USE_RAW_VALUES, current, CEIL(current * DIGIPOT_A4988_FACTOR));
  return byte(constrain(value, 0, DIGIPOT_MCP4018_MAX_VALUE));
}

static SlowSoftI2CMaster pots[DIGIPOT_I2C_NUM_CHANNELS] = {
  SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_X, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
  #if DIGIPOT_I2C_NUM_CHANNELS > 1
    , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_Y, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
    #if DIGIPOT_I2C_NUM_CHANNELS > 2
      , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_Z, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
      #if DIGIPOT_I2C_NUM_CHANNELS > 3
        , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_E0, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
        #if DIGIPOT_I2C_NUM_CHANNELS > 4
          , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_E1, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
          #if DIGIPOT_I2C_NUM_CHANNELS > 5
            , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_E2, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
            #if DIGIPOT_I2C_NUM_CHANNELS > 6
              , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_E3, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
              #if DIGIPOT_I2C_NUM_CHANNELS > 7
                , SlowSoftI2CMaster(DIGIPOTS_I2C_SDA_E4, DIGIPOTS_I2C_SCL, ENABLED(DIGIPOT_ENABLE_I2C_PULLUPS))
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
};

static void digipot_i2c_send(const uint8_t channel, const byte v) {
  if (WITHIN(channel, 0, DIGIPOT_I2C_NUM_CHANNELS - 1)) {
    pots[channel].i2c_start(((DIGIPOT_I2C_ADDRESS_A) << 1) | I2C_WRITE);
    pots[channel].i2c_write(v);
    pots[channel].i2c_stop();
  }
}

// This is for the MCP4018 I2C based digipot
void DigipotI2C::set_current(const uint8_t channel, const float current) {
  const float ival = _MIN(_MAX(current, 0), float(DIGIPOT_MCP4018_MAX_VALUE));
  digipot_i2c_send(channel, current_to_wiper(ival));
}

void DigipotI2C::init() {
  for (uint_fast8_t i = 0; i < DIGIPOT_I2C_NUM_CHANNELS; ++i) pots[i].i2c_init();

  // Init currents according to Configuration_adv.h
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

#endif // DIGIPOT_MCP4018
