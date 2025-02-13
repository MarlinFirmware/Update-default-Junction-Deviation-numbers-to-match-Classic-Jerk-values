/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * M102.cpp - Configure Bed Distance Sensor
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(BD_SENSOR)

#include "../gcode.h"
#include "../../feature/bedlevel/bdl/bdl.h"
#include "../../MarlinCore.h" // for printingIsActive

/**
 * M102: Configure the Bed Distance Sensor
 *
 *   M102 S<10ths> : Set adjustable Z height in 10ths of a mm (e.g., 'M102 S4' enables adjusting for Z <= 0.4mm.)
 *   M102 S0       : Disable adjustable Z height.
 *
 * Negative S values are commands:
 *   M102 S-1       : Read BDsensor version
 *   M102 S-2       : Read BDsensor distance value
 *   M102 S-5       : Read raw Calibration data
 *   M102 S-6       : Start Calibration
 */
void GcodeSuite::M102() {
  if (bdl.config_state < BDS_IDLE) {
    SERIAL_ECHOLNPGM("BDsensor is busy:", bdl.config_state);
    return;
  }
  if (parser.seenval('S')) {
    const int8_t command = parser.value_int();
    if (command == BDS_READ_MM)
      SERIAL_ECHOLNPGM("Bed Distance:", bdl.bd_read(), "mm");
    else if ((command < BDS_IDLE) && printingIsActive())
      return;
    else
      bdl.config_state = command;
  }
}

#endif // BD_SENSOR
