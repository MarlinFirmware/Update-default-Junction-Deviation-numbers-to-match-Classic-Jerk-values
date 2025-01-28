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
 * feature/fancheck.cpp - fan tachometer check
 */

#include "../inc/MarlinConfig.h"

#if HAS_FANCHECK

#include "fancheck.h"
#include "../module/temperature.h"

#if HAS_AUTO_FAN && EXTRUDER_AUTO_FAN_SPEED != 255 && DISABLED(FOURWIRES_FANS)
  bool FanCheck::measuring = false;
#endif
Flags<TACHO_COUNT> FanCheck::tacho_state;
uint16_t FanCheck::edge_counter[TACHO_COUNT];
uint8_t FanCheck::rps[TACHO_COUNT];
FanCheck::TachoError FanCheck::error = FanCheck::TachoError::NONE;
bool FanCheck::enabled;

void FanCheck::init() {
  #define _TACHINIT(N) TERN(E##N##_FAN_TACHO_PULLUP, SET_INPUT_PULLUP, TERN(E##N##_FAN_TACHO_PULLDOWN, SET_INPUT_PULLDOWN, SET_INPUT))(E##N##_FAN_TACHO_PIN)
  #if HAS_E0_FAN_TACHO
    _TACHINIT(0);
  #endif
  #if HAS_E1_FAN_TACHO
    _TACHINIT(1);
  #endif
  #if HAS_E2_FAN_TACHO
    _TACHINIT(2);
  #endif
  #if HAS_E3_FAN_TACHO
    _TACHINIT(3);
  #endif
  #if HAS_E4_FAN_TACHO
    _TACHINIT(4);
  #endif
  #if HAS_E5_FAN_TACHO
    _TACHINIT(5);
  #endif
  #if HAS_E6_FAN_TACHO
    _TACHINIT(6);
  #endif
  #if HAS_E7_FAN_TACHO
    _TACHINIT(7);
  #endif
}

void FanCheck::update_tachometers() {
  bool status;

  #define _TACHO_CASE(N) case N: status = READ(E##N##_FAN_TACHO_PIN); break;
  for (uint8_t f = 0; f < TACHO_COUNT; ++f) {
    switch (f) {
      #if HAS_E0_FAN_TACHO
        _TACHO_CASE(0)
      #endif
      #if HAS_E1_FAN_TACHO
        _TACHO_CASE(1)
      #endif
      #if HAS_E2_FAN_TACHO
        _TACHO_CASE(2)
      #endif
      #if HAS_E3_FAN_TACHO
        _TACHO_CASE(3)
      #endif
      #if HAS_E4_FAN_TACHO
        _TACHO_CASE(4)
      #endif
      #if HAS_E5_FAN_TACHO
        _TACHO_CASE(5)
      #endif
      #if HAS_E6_FAN_TACHO
        _TACHO_CASE(6)
      #endif
      #if HAS_E7_FAN_TACHO
        _TACHO_CASE(7)
      #endif
      default: continue;
    }

    if (status != tacho_state[f]) {
      if (measuring) ++edge_counter[f];
      tacho_state.set(f, status);
    }
  }
}

void FanCheck::compute_speed(uint16_t elapsedTime) {
  static uint8_t errors_count[TACHO_COUNT];
  static uint8_t fan_reported_errors_msk = 0;

  uint8_t fan_error_msk = 0;
  for (uint8_t f = 0; f < TACHO_COUNT; ++f) {
    switch (f) {
      TERN_(HAS_E0_FAN_TACHO, case 0:)
      TERN_(HAS_E1_FAN_TACHO, case 1:)
      TERN_(HAS_E2_FAN_TACHO, case 2:)
      TERN_(HAS_E3_FAN_TACHO, case 3:)
      TERN_(HAS_E4_FAN_TACHO, case 4:)
      TERN_(HAS_E5_FAN_TACHO, case 5:)
      TERN_(HAS_E6_FAN_TACHO, case 6:)
      TERN_(HAS_E7_FAN_TACHO, case 7:)
        // Compute fan speed
        rps[f] = edge_counter[f] * float(250) / elapsedTime;
        edge_counter[f] = 0;

        // Check fan speed
        constexpr int8_t max_extruder_fan_errors = TERN(HAS_PWMFANCHECK, 10000, 5000) / Temperature::fan_update_interval_ms;

        if (rps[f] >= 20 || TERN0(HAS_AUTO_FAN, thermalManager.autofan_speed[f] == 0))
          errors_count[f] = 0;
        else if (errors_count[f] < max_extruder_fan_errors)
          ++errors_count[f];
        else if (enabled)
          SBI(fan_error_msk, f);
        break;
      }
    }

  // Drop the error when all fans are ok
  if (!fan_error_msk && error == TachoError::REPORTED) error = TachoError::FIXED;

  if (error == TachoError::FIXED && !printJobOngoing() && !printingIsPaused()) {
    error = TachoError::NONE; // if the issue has been fixed while the printer is idle, reenable immediately
    ui.reset_alert_level();
  }

  if (fan_error_msk & ~fan_reported_errors_msk) {
    // Handle new faults only
    for (uint8_t f = 0; f < TACHO_COUNT; ++f) if (TEST(fan_error_msk, f)) report_speed_error(f);
  }
  fan_reported_errors_msk = fan_error_msk;
}

void FanCheck::report_speed_error(uint8_t fan) {
  if (printJobOngoing()) {
    if (error == TachoError::NONE) {
      if (thermalManager.degTargetHotend(fan) != 0) {
        kill(GET_TEXT_F(MSG_FAN_SPEED_FAULT));
        error = TachoError::REPORTED;
      }
      else
        error = TachoError::DETECTED;   // Plans error for next processed command
    }
  }
  else if (!printingIsPaused()) {
    thermalManager.setTargetHotend(0, fan); // Always disable heating
    if (error == TachoError::NONE) error = TachoError::REPORTED;
  }

  SERIAL_ERROR_MSG(STR_ERR_FANSPEED, fan);
  LCD_ALERTMESSAGE(MSG_FAN_SPEED_FAULT);
}

void FanCheck::print_fan_states() {
  for (uint8_t s = 0; s < 2; ++s) {
    for (uint8_t f = 0; f < TACHO_COUNT; ++f) {
      switch (f) {
        TERN_(HAS_E0_FAN_TACHO, case 0:)
        TERN_(HAS_E1_FAN_TACHO, case 1:)
        TERN_(HAS_E2_FAN_TACHO, case 2:)
        TERN_(HAS_E3_FAN_TACHO, case 3:)
        TERN_(HAS_E4_FAN_TACHO, case 4:)
        TERN_(HAS_E5_FAN_TACHO, case 5:)
        TERN_(HAS_E6_FAN_TACHO, case 6:)
        TERN_(HAS_E7_FAN_TACHO, case 7:)
          SERIAL_ECHOPGM("E", f);
          if (s == 0)
            SERIAL_ECHOPGM(":", 60 * rps[f], " RPM ");
          else
            SERIAL_ECHOPGM("@:", TERN(HAS_AUTO_FAN, thermalManager.autofan_speed[f], 255), " ");
          break;
      }
    }
  }
  SERIAL_EOL();
}

#if ENABLED(AUTO_REPORT_FANS)
  AutoReporter<FanCheck::AutoReportFan> FanCheck::auto_reporter;
  void FanCheck::AutoReportFan::report() { print_fan_states(); }
#endif

#endif // HAS_FANCHECK
