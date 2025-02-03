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
 * feature/ammeter.cpp
 */

#include "../inc/MarlinConfig.h"

#if ENABLED(I2C_AMMETER)

#include "ammeter.h"

#ifndef I2C_AMMETER_IMAX
  #define I2C_AMMETER_IMAX 0.500 // Calibration range 500 Milliamps
#endif

INA226 ina;

Ammeter ammeter;

float Ammeter::scale;
float Ammeter::current;

void Ammeter::init() {
  ina.begin();
  ina.configure(INA226_AVERAGES_16, INA226_BUS_CONV_TIME_1100US, INA226_SHUNT_CONV_TIME_1100US, INA226_MODE_SHUNT_BUS_CONT);
  ina.calibrate(I2C_AMMETER_SHUNT_RESISTOR, I2C_AMMETER_IMAX);
}

float Ammeter::read() {
  scale = 1;
  current = ina.readShuntCurrent();
  if (current <= 0.0001f) current = 0; // Clean-up least-significant-bit amplification errors
  if (current < 0.1f) scale = 1000;
  return current * scale;
}

#endif // I2C_AMMETER
