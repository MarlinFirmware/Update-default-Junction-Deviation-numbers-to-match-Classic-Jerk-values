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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */

/**
 * Test SAMD51 specific configuration values for errors at compile-time.
 */

#if HAS_SPI_TFT || HAS_FSMC_TFT
  #error "Sorry! TFT displays are not available for HAL/SAMD51."
#endif

#if ENABLED(FLASH_EEPROM_EMULATION)
  #warning "Did you activate the SmartEEPROM? See https://github.com/GMagician/SAMD51-SmartEEprom-Manager/releases"
#endif

#if defined(ADAFRUIT_GRAND_CENTRAL_M4) && ANY_VOLUME_IS(CUSTOM)
  #error "No custom SD pins defined for this board."
#endif

#if (defined(TEMP_0_SCK_PIN) && defined(TEMP_0_MISO_PIN) && (TEMP_0_SCK_PIN == SCK1 || TEMP_0_MISO_PIN == MISO1)) || \
    (defined(TEMP_1_SCK_PIN) && defined(TEMP_1_MISO_PIN) && (TEMP_1_SCK_PIN == SCK1 || TEMP_1_MISO_PIN == MISO1))
  #error "OnBoard SPI BUS can't be shared with other devices."
#endif

#if SERVO_TC == MF_TIMER_RTC
  #error "Servos can't use RTC timer"
#endif

#if ENABLED(EMERGENCY_PARSER)
  #error "EMERGENCY_PARSER is not yet implemented for SAMD51. Disable EMERGENCY_PARSER to continue."
#endif

#if ENABLED(ONBOARD_SDIO)
  #error "ONBOARD_SDIO is not supported on SAMD51."
#endif

#if ENABLED(FAST_PWM_FAN) || SPINDLE_LASER_FREQUENCY
  #error "Features requiring Hardware PWM (FAST_PWM_FAN, SPINDLE_LASER_FREQUENCY) are not yet supported for HAL/SAMD51."
#endif

#if ENABLED(POSTMORTEM_DEBUGGING)
  #error "POSTMORTEM_DEBUGGING is not yet supported on AGCM4."
#endif
