/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../../../inc/MarlinConfig.h"

#if ENABLED(ACCELEROMETER_ADXL345)

#include "../../gcode.h"
#include "../../../feature/accelerometer/acc_adxl345.h"

// Arduino FFT helpfully redefines these macros:
#undef MIN
#undef MAX
#undef constrain
#include <arduinoFFT.h>

// For development. Relocate to accelerometer instances.
static struct {
  float hz = 3200.0f;
  uint8_t orient = 0x20;
  uint8_t sample_size = 12;
} acc_settings[1];

/**
 * M955: Configure an accerometer connected to SPI or CAN bus (TODO). <RepRapFirmware>
 *
 * Parameters:
 *  P<id>           - The Accelerometer to use. Index, CAN bus ID, etc. Default: SPI or first CAN ID
 *  I<orientation>  - The ZX orientation of the accelerometer. (Default: 0x20 = aligned to +ZX)
 *                     0: +X, 1: +Y, 2: +Z
 *                     4: -X, 5: -Y, 6: -Z
 *  R<bits>         - Sample resolution bits, typically 8, 10, or 12.
 *  S<rate>         - Sample rate in Hz.
 */
void GcodeSuite::M955() {
  uint8_t pid = parser.intval('P');
  if (pid >= COUNT(acc_settings)) return;

  // ADXL345 Data Rate : 0 = 0.1Hz ; 15=3200Hz
  if (parser.seenval('S')) {
    const float hz = parser.value_float();
    const uint8_t rid = NEAR(hz, 0.1f) ? 0 : NEAR(hz, 3200.0f) ? 15 : int(hz);
    adxl345.setDataRate(rid);
    acc_settings[pid].hz = hz;
  }
  // Possible Orientations: 01 02 05 06 / 10 12 14 16 / 20 21 24 25 / 41 42 45 46 / 50 52 54 56 / 60 61 64 65
  if (parser.seenval('I')) {
    const uint8_t ori = parser.value_int(), orz = ori / 10, orx = ori % 10;
    if (orz < 7 && orx < 7 && orz != 3 && orx != 3 && (orz & 3) != (orx & 3))
      acc_settings[pid].orient = orz << 4 | orz;
  }
}

void GcodeSuite::M955_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F(STR_ACCELEROMETER));

  for (uint8_t i = 0; i < COUNT(acc_settings); ++i) {
    SERIAL_ECHOLNPGM_P(
        PSTR("  M955 P"), i
      , PSTR(" I"), acc_settings[i].orient
      , PSTR(" R"), acc_settings[i].sample_size
      , PSTR(" S"), acc_settings[i].hz
    );
  }
}

/**
 * M956: Collect accerometer data and save to the SD Card in CSV format. <RepRapFirmware>
 *
 * Parameters:
 *  P<id>           - The Accelerometer to use. Index, CAN bus ID, etc. Default: SPI or first CAN ID
 *  S<samples>      - Number of samples to collect
 *  XYZ             - Specify one or more axes to collect data for. (Default: XYZ)
 *  A<activation>   - Activate at a certain time:
 *                     0 = Immediately
 *                     1 = Just before the start of the next move
 *                     2 = Just before the start of the deceleration segment of the next move
 *  K<skip>         - Number of moves to skip before activation. Use with A1 or A2.
 */
void GcodeSuite::M956() {
}

/**
 * M958: Excite Harmonic Vibration. (No Firmwares Tagged.)
 *
 * XYZ              - X or Y or Z Machine axis
 * F<frequency>     - Frequency (Hz)
 * A<acceleration>  - Acceleration (mm/s2)
 * S<seconds>       - Duration (s)
 */
void GcodeSuite::M958() {
}

/**
 * M974: Measure Print Head Resonance - Report Raw Samples - <Prusa Firmware>
 *
 * Parameters:
 *  P<id>           - The Accelerometer to use. Index, CAN bus ID, etc. Default: SPI or first CAN ID
 *  F<speed>        - Revolutions-per-second feedrate. Based on extra configuration info TBD.
 *  XY              - Specify X or Y motor to move. (Default: X)   TODO: Single motor on CoreXY?
 */
void GcodeSuite::M974() {
}

/**
 * M975: Measure Accelerometer Sampling Frequency - <Prusa Firmware>
 *       Report "sample freq: <freq>"
 */
void GcodeSuite::M975() {
}

/**
 * M976: Measure Print Head Resonance - Frequency Response - <Prusa Firmware>
 *
 * Parameters:
 *  P<id>           - The Accelerometer to use. Index, CAN bus ID, etc. Default: SPI or first CAN ID
 *  F<speed>        - Revolutions-per-second feedrate. Based on extra configuration info TBD.
 *  XY              - Specify X or Y motor to move. (Default: X)   TODO: Single motor on CoreXY?
 */
void GcodeSuite::M976() {
}

/**
 * M977: Calibrate Stepper Motor Frequency Compensation (Input Shaping) - <Prusa Firmware>
 *
 * Parameters:
 *  XY              - Specify X or Y motor to calibrate. (Default: X)   TODO: Single motor on CoreXY?
 */
void GcodeSuite::M977() {
  // - Check print status. Don't allow calibration during printing.

  // - Wait for current motion to complete

  // - Prepare accelerometer

  // - Move axis, measure, analyze, repeat as necessary.

  // - Apply measurements to shaping settings.

  // - Re-run motion testing, adjust until resonance is minimized.

  //
  // Simplest approach is to start with a wide bracket for
  // testing and narrow down to specific frequencies that minimize
  // resonances for typical print speed ranges.
  // Apply testing at different heights to get values for dynamic adjustment.
  //

}

#endif // ACCELEROMETER_ADXL345
