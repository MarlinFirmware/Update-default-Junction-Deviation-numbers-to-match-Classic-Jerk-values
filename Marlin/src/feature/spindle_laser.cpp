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

/**
 * feature/spindle_laser.cpp
 */

#include "../inc/MarlinConfig.h"

#if HAS_CUTTER

#include "spindle_laser.h"

#if ENABLED(SPINDLE_SERVO)
  #include "../module/servo.h"
#endif

#if ENABLED(I2C_AMMETER)
  #include "ammeter.h"
#endif

SpindleLaser cutter;
bool SpindleLaser::enable_state;                                      // Virtual enable state, controls enable pin if present and or apply power if > 0
uint8_t SpindleLaser::power,                                          // Actual power output 0-255 ocr or "0 = off" > 0 = "on"
        SpindleLaser::last_power_applied; // = 0                      // Basic power state tracking

#if ENABLED(LASER_FEATURE)
  cutter_test_pulse_t SpindleLaser::testPulse = 50;                   // (ms) Test fire pulse default duration
  uint8_t SpindleLaser::last_block_power; // = 0                      // Track power changes for dynamic inline power
  feedRate_t SpindleLaser::feedrate_mm_m = 1500,
             SpindleLaser::last_feedrate_mm_m; // = 0                 // (mm/min) Track feedrate changes for dynamic power
#endif

bool SpindleLaser::isReadyForUI = false;                              // Ready to apply power setting from the UI to OCR
CutterMode SpindleLaser::cutter_mode = CUTTER_MODE_STANDARD;          // Default is standard mode

constexpr cutter_cpower_t SpindleLaser::power_floor;
cutter_power_t SpindleLaser::menuPower = 0,                           // Power value via LCD menu in PWM, PERCENT, or RPM based on configured format set by CUTTER_POWER_UNIT.
               SpindleLaser::unitPower = 0;                           // Unit power is in PWM, PERCENT, or RPM based on CUTTER_POWER_UNIT.

cutter_frequency_t SpindleLaser::frequency;                           // PWM frequency setting; range: 2K - 50K

#define SPINDLE_LASER_PWM_OFF TERN(SPINDLE_LASER_PWM_INVERT, 255, 0)

/**
 * Init the cutter to a safe OFF state
 */
void SpindleLaser::init() {
  #if ENABLED(SPINDLE_SERVO)
    servo[SPINDLE_SERVO_NR].move(SPINDLE_SERVO_MIN);
  #elif PIN_EXISTS(SPINDLE_LASER_ENA)
    OUT_WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE);    // Init spindle to off
  #endif
  #if ENABLED(SPINDLE_CHANGE_DIR)
    OUT_WRITE(SPINDLE_DIR_PIN, SPINDLE_INVERT_DIR);                   // Init rotation to clockwise (M3)
  #endif
  #if ENABLED(HAL_CAN_SET_PWM_FREQ) && SPINDLE_LASER_FREQUENCY
    frequency = SPINDLE_LASER_FREQUENCY;
    hal.set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_FREQUENCY);
  #endif
  #if ENABLED(SPINDLE_LASER_USE_PWM)
    SET_PWM(SPINDLE_LASER_PWM_PIN);
    hal.set_pwm_duty(pin_t(SPINDLE_LASER_PWM_PIN), SPINDLE_LASER_PWM_OFF); // Set to lowest speed
  #endif
  #if ENABLED(AIR_EVACUATION)
    OUT_WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE);            // Init Vacuum/Blower OFF
  #endif
  #if ENABLED(AIR_ASSIST)
    OUT_WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_ACTIVE);                    // Init Air Assist OFF
  #endif
  TERN_(I2C_AMMETER, ammeter.init());                                 // Init I2C Ammeter
}

#if ENABLED(SPINDLE_LASER_USE_PWM)
  /**
   * Set the cutter PWM directly to the given ocr value
   *
   * @param ocr Power value
   */
  void SpindleLaser::_set_ocr(const uint8_t ocr) {
    #if ENABLED(HAL_CAN_SET_PWM_FREQ) && SPINDLE_LASER_FREQUENCY
      hal.set_pwm_frequency(pin_t(SPINDLE_LASER_PWM_PIN), frequency);
    #endif
    hal.set_pwm_duty(pin_t(SPINDLE_LASER_PWM_PIN), ocr ^ SPINDLE_LASER_PWM_OFF);
  }

  void SpindleLaser::set_ocr(const uint8_t ocr) {
    #if PIN_EXISTS(SPINDLE_LASER_ENA)
      WRITE(SPINDLE_LASER_ENA_PIN,  SPINDLE_LASER_ACTIVE_STATE); // Cutter ON
    #endif
    _set_ocr(ocr);
  }

  void SpindleLaser::ocr_off() {
    #if PIN_EXISTS(SPINDLE_LASER_ENA)
      WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE); // Cutter OFF
    #endif
    _set_ocr(0);
  }
#endif // SPINDLE_LASER_USE_PWM

/**
 * Apply power for Laser or Spindle
 *
 * Apply cutter power value for PWM, Servo, and on/off pin.
 *
 * @param opwr Power value. Range 0 to MAX.
 */
void SpindleLaser::apply_power(const uint8_t opwr) {
  if (enabled() || opwr == 0) {                                   // 0 check allows us to disable where no ENA pin exists
    // Test and set the last power used to improve performance
    if (opwr == last_power_applied) return;
    last_power_applied = opwr;
    // Handle PWM driven or just simple on/off
    #if ENABLED(SPINDLE_LASER_USE_PWM)
      if (CUTTER_UNIT_IS(RPM) && unitPower == 0)
        ocr_off();
      else if (ENABLED(CUTTER_POWER_RELATIVE) || enabled() || opwr == 0) {
        set_ocr(opwr);
        isReadyForUI = true;
      }
      else
        ocr_off();
    #elif ENABLED(SPINDLE_SERVO)
      servo[SPINDLE_SERVO_NR].move(opwr);
    #else
      WRITE(SPINDLE_LASER_ENA_PIN, enabled() ? SPINDLE_LASER_ACTIVE_STATE : !SPINDLE_LASER_ACTIVE_STATE);
      isReadyForUI = true;
    #endif
  }
  else {
    #if PIN_EXISTS(SPINDLE_LASER_ENA)
      WRITE(SPINDLE_LASER_ENA_PIN, !SPINDLE_LASER_ACTIVE_STATE);
    #endif
    isReadyForUI = false; // Only used for UI display updates.
    TERN_(SPINDLE_LASER_USE_PWM, ocr_off());
  }
}

#if ENABLED(SPINDLE_CHANGE_DIR)
  /**
   * Set the spindle direction and apply immediately
   * Stop on direction change if SPINDLE_STOP_ON_DIR_CHANGE is enabled
   */
  void SpindleLaser::set_reverse(const bool reverse) {
    const bool dir_state = (reverse == SPINDLE_INVERT_DIR); // Forward (M3) HIGH when not inverted
    if (TERN0(SPINDLE_STOP_ON_DIR_CHANGE, enabled()) && READ(SPINDLE_DIR_PIN) != dir_state) disable();
    WRITE(SPINDLE_DIR_PIN, dir_state);
  }
#endif

#if ENABLED(AIR_EVACUATION)
  // Enable / disable Cutter Vacuum or Laser Blower motor
  void SpindleLaser::air_evac_enable()  { WRITE(AIR_EVACUATION_PIN,  AIR_EVACUATION_ACTIVE); } // Turn ON
  void SpindleLaser::air_evac_disable() { WRITE(AIR_EVACUATION_PIN, !AIR_EVACUATION_ACTIVE); } // Turn OFF
  void SpindleLaser::air_evac_toggle()  { TOGGLE(AIR_EVACUATION_PIN); } // Toggle state
#endif

#if ENABLED(AIR_ASSIST)
  // Enable / disable air assist
  void SpindleLaser::air_assist_enable()  { WRITE(AIR_ASSIST_PIN,  AIR_ASSIST_ACTIVE); } // Turn ON
  void SpindleLaser::air_assist_disable() { WRITE(AIR_ASSIST_PIN, !AIR_ASSIST_ACTIVE); } // Turn OFF
  void SpindleLaser::air_assist_toggle()  { TOGGLE(AIR_ASSIST_PIN); } // Toggle state
#endif

#endif // HAS_CUTTER
