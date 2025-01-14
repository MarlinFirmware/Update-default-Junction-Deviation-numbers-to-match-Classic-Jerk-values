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

#if ENABLED(USE_CONTROLLER_FAN)

#include "controllerfan.h"
#include "../module/stepper.h"
#include "../module/temperature.h"

ControllerFan controllerFan;

uint8_t ControllerFan::speed;

#if ENABLED(CONTROLLER_FAN_EDITABLE)
  controllerFan_settings_t ControllerFan::settings; // {0}
 #else
   const controllerFan_settings_t &ControllerFan::settings = controllerFan_defaults;
#endif

#if ENABLED(FAN_SOFT_PWM)
  uint8_t ControllerFan::soft_pwm_speed;
#endif

void ControllerFan::setup() {
  SET_OUTPUT(CONTROLLER_FAN_PIN);
  #ifdef CONTROLLER_FAN2_PIN
    SET_OUTPUT(CONTROLLER_FAN2_PIN);
  #endif
  init();
}

void ControllerFan::set_fan_speed(const uint8_t s) {
  speed = s < (CONTROLLERFAN_SPEED_MIN) ? 0 : s; // Fan OFF below minimum
}

void ControllerFan::update() {
  static millis_t lastComponentOn = 0,  // Last time a stepper, heater, etc. was turned on
                  nextFanCheck = 0;     // Last time the state was checked
  const millis_t ms = millis();
  if (ELAPSED(ms, nextFanCheck)) {
    nextFanCheck = ms + 2500UL; // Not a time critical function, so only check every 2.5s

    /**
     * If any triggers for the controller fan are true...
     *   - At least one stepper driver is enabled
     *   - The heated bed (MOSFET) is enabled
     *   - TEMP_SENSOR_BOARD is reporting >= CONTROLLER_FAN_MIN_BOARD_TEMP
     *   - TEMP_SENSOR_SOC is reporting >= CONTROLLER_FAN_MIN_SOC_TEMP
     */
    const ena_mask_t axis_mask = TERN(CONTROLLER_FAN_USE_Z_ONLY, _BV(Z_AXIS), (ena_mask_t)~TERN0(CONTROLLER_FAN_IGNORE_Z, _BV(Z_AXIS)));
    if ( (stepper.axis_enabled.bits & axis_mask)
      #if ALL(HAS_HEATED_BED, CONTROLLER_FAN_BED_HEATING)
        || thermalManager.temp_bed.soft_pwm_amount > 0
      #endif
      #ifdef CONTROLLER_FAN_MIN_BOARD_TEMP
        || thermalManager.wholeDegBoard() >= CONTROLLER_FAN_MIN_BOARD_TEMP
      #endif
      #ifdef CONTROLLER_FAN_MIN_SOC_TEMP
        || thermalManager.wholeDegSoc() >= CONTROLLER_FAN_MIN_SOC_TEMP
      #endif
    ) lastComponentOn = ms; //... set time to NOW so the fan will turn on

    /**
     * Fan Settings. Set fan > 0:
     *  - If AutoMode is on and hot components have been powered for CONTROLLERFAN_IDLE_TIME seconds.
     *  - If System is on idle and idle fan speed settings is activated.
     */
    set_fan_speed(
      settings.auto_mode && lastComponentOn && PENDING(ms, lastComponentOn + SEC_TO_MS(settings.duration))
      ? settings.active_speed : settings.idle_speed
    );

    speed = CALC_FAN_SPEED(speed);

    #if FAN_KICKSTART_TIME
      static millis_t fan_kick_end = 0;
      if (speed > FAN_OFF_PWM) {
        if (!fan_kick_end) {
          fan_kick_end = ms + FAN_KICKSTART_TIME; // May be longer based on slow update interval for controller fn check. Sets minimum
          speed = FAN_KICKSTART_POWER;
        }
        else if (PENDING(ms, fan_kick_end))
          speed = FAN_KICKSTART_POWER;
      }
      else
        fan_kick_end = 0;
    #endif

    #if ENABLED(FAN_SOFT_PWM)
      soft_pwm_speed = speed;
    #else
      if (PWM_PIN(CONTROLLER_FAN_PIN))
        hal.set_pwm_duty(pin_t(CONTROLLER_FAN_PIN), speed);
      else
        WRITE(CONTROLLER_FAN_PIN, speed > 0);

      #ifdef CONTROLLER_FAN2_PIN
        if (PWM_PIN(CONTROLLER_FAN2_PIN))
          hal.set_pwm_duty(pin_t(CONTROLLER_FAN2_PIN), speed);
        else
          WRITE(CONTROLLER_FAN2_PIN, speed > 0);
      #endif
    #endif
  }
}

#endif // USE_CONTROLLER_FAN
