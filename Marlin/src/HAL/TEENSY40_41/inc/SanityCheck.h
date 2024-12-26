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
 * Test TEENSY41 specific configuration values for errors at compile-time.
 */

#if HAS_SPI_TFT || HAS_FSMC_TFT
  #error "Sorry! TFT displays are not available for Teensy 4.0/4.1."
#endif

#if ENABLED(EMERGENCY_PARSER)
  #error "EMERGENCY_PARSER is not yet implemented for Teensy 4.0/4.1. Disable EMERGENCY_PARSER to continue."
#endif

#if HAS_TMC_SW_SERIAL
  #error "TMC220x Software Serial is not supported for Teensy 4.0/4.1."
#endif

#if ENABLED(POSTMORTEM_DEBUGGING)
  #error "POSTMORTEM_DEBUGGING is not yet supported for Teensy 4.0/4.1."
#endif

#if ENABLED(SERIAL_STATS_MAX_RX_QUEUED) || ENABLED(SERIAL_STATS_DROPPED_RX) || ENABLED(SERIAL_STATS_RX_FRAMING_ERRORS) || ENABLED(SERIAL_STATS_RX_BUFFER_OVERRUNS)
  #error "SERIAL_STATS_* features not supported on Teensy 4.0/4.1."
#endif

#if ENABLED(BAUD_RATE_GCODE) && SERIAL_PORT_2 == -2
  #error "BAUD_RATE_GCODE cannot be enabled when using the Ethernet serial port."
#endif
