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
 * feature/power_monitor.h
 */

#include "../inc/MarlinConfig.h"

#define PM_SAMPLE_RANGE HAL_ADC_RANGE
#define PM_K_VALUE      6
#define PM_K_SCALE      6

template <const float & SCALE, int K_VALUE, int K_SCALE>
struct pm_lpf_t {
  uint32_t filter_buf;
  float value;
  void add_sample(const uint16_t sample) {
    filter_buf += (uint32_t(sample) << K_SCALE) - (filter_buf >> K_VALUE);
  }
  void capture() {
    value = filter_buf * (SCALE * (1.0f / (1UL << (PM_K_VALUE + PM_K_SCALE))));
  }
  void reset(uint16_t reset_value = 0) {
    filter_buf = uint32_t(reset_value) << (K_VALUE + K_SCALE);
    capture();
  }
};

class PowerMonitor {
private:
  #if ENABLED(POWER_MONITOR_CURRENT)
    static constexpr float amps_adc_scale = (float(ADC_VREF_MV) / 1000.0f) / (POWER_MONITOR_VOLTS_PER_AMP * PM_SAMPLE_RANGE);
    static pm_lpf_t<amps_adc_scale, PM_K_VALUE, PM_K_SCALE> amps;
  #endif
  #if ENABLED(POWER_MONITOR_VOLTAGE)
    static constexpr float volts_adc_scale = (float(ADC_VREF_MV) / 1000.0f) / (POWER_MONITOR_VOLTS_PER_VOLT * PM_SAMPLE_RANGE);
    static pm_lpf_t<volts_adc_scale, PM_K_VALUE, PM_K_SCALE> volts;
  #endif

public:
  static uint8_t flags;  // M430 flags to display current

  static millis_t display_item_ms;
  static uint8_t display_item;

  PowerMonitor() { reset(); }

  enum PM_Display_Bit : uint8_t {
    PM_DISP_BIT_I, // Current display enable bit
    PM_DISP_BIT_V, // Voltage display enable bit
    PM_DISP_BIT_P  // Power display enable bit
  };

  #if ENABLED(POWER_MONITOR_CURRENT)
    FORCE_INLINE static float getAmps() { return amps.value + (POWER_MONITOR_CURRENT_OFFSET); }
    void add_current_sample(const uint16_t value) { amps.add_sample(value); }
  #endif

  #if ENABLED(POWER_MONITOR_VOLTAGE)
    FORCE_INLINE static float getVolts() { return volts.value + (POWER_MONITOR_VOLTAGE_OFFSET); }
    void add_voltage_sample(const uint16_t value) { volts.add_sample(value); }
  #else
    FORCE_INLINE static float getVolts() { return POWER_MONITOR_FIXED_VOLTAGE; }
  #endif

  #if HAS_POWER_MONITOR_WATTS
    FORCE_INLINE static float getPower() { return getAmps() * getVolts(); }
  #endif

  #if HAS_WIRED_LCD
    #if HAS_MARLINUI_U8GLIB && DISABLED(LIGHTWEIGHT_UI)
      FORCE_INLINE static bool display_enabled() { return flags != 0x00; }
    #endif
    #if ENABLED(POWER_MONITOR_CURRENT)
      static void draw_current();
      FORCE_INLINE static bool current_display_enabled() { return TEST(flags, PM_DISP_BIT_I); }
      FORCE_INLINE static void set_current_display(const bool b) { SET_BIT_TO(flags, PM_DISP_BIT_I, b); }
      FORCE_INLINE static void toggle_current_display() { TBI(flags, PM_DISP_BIT_I); }
    #endif
    #if ENABLED(POWER_MONITOR_VOLTAGE)
      static void draw_voltage();
      FORCE_INLINE static bool voltage_display_enabled() { return TEST(flags, PM_DISP_BIT_V); }
      FORCE_INLINE static void set_voltage_display(const bool b) { SET_BIT_TO(flags, PM_DISP_BIT_V, b); }
      FORCE_INLINE static void toggle_voltage_display() { TBI(flags, PM_DISP_BIT_V); }
    #endif
    #if HAS_POWER_MONITOR_WATTS
      static void draw_power();
      FORCE_INLINE static bool power_display_enabled() { return TEST(flags, PM_DISP_BIT_P); }
      FORCE_INLINE static void set_power_display(const bool b) { SET_BIT_TO(flags, PM_DISP_BIT_P, b); }
      FORCE_INLINE static void toggle_power_display() { TBI(flags, PM_DISP_BIT_P); }
    #endif
  #endif

  static void reset() {
    flags = 0x00;

    #if ENABLED(POWER_MONITOR_CURRENT)
      amps.reset();
    #endif

    #if ENABLED(POWER_MONITOR_VOLTAGE)
      volts.reset();
    #endif

    #if HAS_MEDIA
      display_item_ms = 0;
      display_item = 0;
    #endif
  }

  static void capture_values() {
    #if ENABLED(POWER_MONITOR_CURRENT)
     amps.capture();
    #endif
    #if ENABLED(POWER_MONITOR_VOLTAGE)
      volts.capture();
    #endif
  }
};

extern PowerMonitor power_monitor;
