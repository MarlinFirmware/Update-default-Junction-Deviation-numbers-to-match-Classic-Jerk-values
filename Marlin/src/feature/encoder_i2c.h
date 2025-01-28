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
 * feature/encoder_i2c.h
 */

#include "../inc/MarlinConfig.h"

#include "../module/planner.h"

#include <Wire.h>

//=========== Advanced / Less-Common Encoder Configuration Settings ==========

#define I2CPE_EC_THRESH_PROPORTIONAL       // If enabled, adjusts the error correction threshold
                                           // proportional to the current speed of the axis. Allows
                                           // for very small error margin at low speeds without
                                           // stuttering due to reading latency at high speeds

#define I2CPE_DEBUG                        // Enable encoder-related debug serial echos

#define I2CPE_REBOOT_TIME             5000 // Time we wait for an encoder module to reboot
                                           // after changing address.

#define I2CPE_MAG_SIG_GOOD               0
#define I2CPE_MAG_SIG_MID                1
#define I2CPE_MAG_SIG_BAD                2
#define I2CPE_MAG_SIG_NF               255

#define I2CPE_REQ_REPORT                 0
#define I2CPE_RESET_COUNT                1
#define I2CPE_SET_ADDR                   2
#define I2CPE_SET_REPORT_MODE            3
#define I2CPE_CLEAR_EEPROM               4

#define I2CPE_LED_PAR_MODE              10
#define I2CPE_LED_PAR_BRT               11
#define I2CPE_LED_PAR_RATE              14

#define I2CPE_REPORT_DISTANCE            0
#define I2CPE_REPORT_STRENGTH            1
#define I2CPE_REPORT_VERSION             2

// Default I2C addresses
#define I2CPE_PRESET_ADDR_X             30
#define I2CPE_PRESET_ADDR_Y             31
#define I2CPE_PRESET_ADDR_Z             32
#define I2CPE_PRESET_ADDR_E             33

#define I2CPE_DEF_AXIS              X_AXIS
#define I2CPE_DEF_ADDR I2CPE_PRESET_ADDR_X

// Error event counter; tracks how many times there is an error exceeding a certain threshold
#define I2CPE_ERR_CNT_THRESH             3.00
#define I2CPE_ERR_CNT_DEBOUNCE_MS     2000

#if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
  #define I2CPE_ERR_ARRAY_SIZE          32
  #define I2CPE_ERR_PRST_ARRAY_SIZE     10
#endif

// Error Correction Methods
#define I2CPE_ECM_NONE                   0
#define I2CPE_ECM_MICROSTEP              1
#define I2CPE_ECM_PLANNER                2
#define I2CPE_ECM_STALLDETECT            3

// Encoder types
#define I2CPE_ENC_TYPE_ROTARY            0
#define I2CPE_ENC_TYPE_LINEAR            1

// Parser
#define I2CPE_PARSE_ERR                  1
#define I2CPE_PARSE_OK                   0

#define LOOP_PE(VAR) for (uint8_t VAR = 0; VAR < I2CPE_ENCODER_CNT; ++VAR)
#define CHECK_IDX() do{ if (!WITHIN(idx, 0, I2CPE_ENCODER_CNT - 1)) return; }while(0)

typedef union {
  volatile int32_t val = 0;
  uint8_t bval[4];
} i2cLong;

class I2CPositionEncoder {
  private:
    AxisEnum  encoderAxis         = I2CPE_DEF_AXIS;

    uint8_t   i2cAddress          = I2CPE_DEF_ADDR,
              ecMethod            = I2CPE_DEF_EC_METHOD,
              type                = I2CPE_DEF_TYPE,
              H                   = I2CPE_MAG_SIG_NF; // Magnetic field strength

    int       encoderTicksPerUnit = I2CPE_DEF_ENC_TICKS_UNIT,
              stepperTicks        = I2CPE_DEF_TICKS_REV,
              errorCount          = 0,
              errorPrev           = 0;

    float     ecThreshold         = I2CPE_DEF_EC_THRESH;

    bool      homed               = false,
              trusted             = false,
              initialized         = false,
              active              = false,
              invert              = false,
              ec                  = true;

    int32_t   zeroOffset          = 0,
              lastPosition        = 0,
              position;

    millis_t  lastPositionTime    = 0,
              nextErrorCountTime  = 0,
              lastErrorTime;

    #if ENABLED(I2CPE_ERR_ROLLING_AVERAGE)
      uint8_t errIdx = 0, errPrstIdx = 0;
      int err[I2CPE_ERR_ARRAY_SIZE] = { 0 },
          errPrst[I2CPE_ERR_PRST_ARRAY_SIZE] = { 0 };
    #endif

  public:
    void init(const uint8_t address, const AxisEnum axis);
    void reset();

    void update();

    void set_homed();
    void set_unhomed();

    int32_t get_raw_count();

    FORCE_INLINE float mm_from_count(const int32_t count) {
      switch (type) {
        default: return -1;
        case I2CPE_ENC_TYPE_LINEAR:
          return count / encoderTicksPerUnit;
        case I2CPE_ENC_TYPE_ROTARY:
          return (count * stepperTicks) / (encoderTicksPerUnit * planner.settings.axis_steps_per_mm[encoderAxis]);
      }
    }

    FORCE_INLINE float get_position_mm() { return mm_from_count(get_position()); }
    FORCE_INLINE int32_t get_position() { return get_raw_count() - zeroOffset; }

    int32_t get_axis_error_steps(const bool report);
    float get_axis_error_mm(const bool report);

    void calibrate_steps_mm(const uint8_t iter);

    bool passes_test(const bool report);

    bool test_axis();

    FORCE_INLINE int get_error_count() { return errorCount; }
    FORCE_INLINE void set_error_count(const int newCount) { errorCount = newCount; }

    FORCE_INLINE uint8_t get_address() { return i2cAddress; }
    FORCE_INLINE void set_address(const uint8_t addr) { i2cAddress = addr; }

    FORCE_INLINE bool get_active() { return active; }
    FORCE_INLINE void set_active(const bool a) { active = a; }

    FORCE_INLINE void set_inverted(const bool i) { invert = i; }

    FORCE_INLINE AxisEnum get_axis() { return encoderAxis; }

    FORCE_INLINE bool get_ec_enabled() { return ec; }
    FORCE_INLINE void set_ec_enabled(const bool enabled) { ec = enabled; }

    FORCE_INLINE uint8_t get_ec_method() { return ecMethod; }
    FORCE_INLINE void set_ec_method(const byte method) { ecMethod = method; }

    FORCE_INLINE float get_ec_threshold() { return ecThreshold; }
    FORCE_INLINE void set_ec_threshold(const_float_t newThreshold) { ecThreshold = newThreshold; }

    FORCE_INLINE int get_encoder_ticks_mm() {
      switch (type) {
        default: return 0;
        case I2CPE_ENC_TYPE_LINEAR:
          return encoderTicksPerUnit;
        case I2CPE_ENC_TYPE_ROTARY:
          return (int)((encoderTicksPerUnit / stepperTicks) * planner.settings.axis_steps_per_mm[encoderAxis]);
      }
    }

    FORCE_INLINE int get_ticks_unit() { return encoderTicksPerUnit; }
    FORCE_INLINE void set_ticks_unit(const int ticks) { encoderTicksPerUnit = ticks; }

    FORCE_INLINE uint8_t get_type() { return type; }
    FORCE_INLINE void set_type(const byte newType) { type = newType; }

    FORCE_INLINE int get_stepper_ticks() { return stepperTicks; }
    FORCE_INLINE void set_stepper_ticks(const int ticks) { stepperTicks = ticks; }
};

class I2CPositionEncodersMgr {
  private:
    static bool I2CPE_anyaxis;
    static uint8_t I2CPE_addr, I2CPE_idx;

  public:

    static void init();

    // Consider only updating one endoder per call / tick if encoders become too time intensive
    static void update() { LOOP_PE(i) encoders[i].update(); }

    static void homed(const AxisEnum axis) {
      LOOP_PE(i)
        if (encoders[i].get_axis() == axis) encoders[i].set_homed();
    }

    static void unhomed(const AxisEnum axis) {
      LOOP_PE(i)
        if (encoders[i].get_axis() == axis) encoders[i].set_unhomed();
    }

    static void report_position(const int8_t idx, const bool units, const bool noOffset);

    static void report_status(const int8_t idx) {
      CHECK_IDX();
      SERIAL_ECHOLNPGM("Encoder ", idx, ": ");
      encoders[idx].get_raw_count();
      encoders[idx].passes_test(true);
    }

    static void report_error(const int8_t idx) {
      CHECK_IDX();
      encoders[idx].get_axis_error_steps(true);
    }

    static void test_axis(const int8_t idx) {
      CHECK_IDX();
      encoders[idx].test_axis();
    }

    static void calibrate_steps_mm(const int8_t idx, const int iterations) {
      CHECK_IDX();
      encoders[idx].calibrate_steps_mm(iterations);
    }

    static void change_module_address(const uint8_t oldaddr, const uint8_t newaddr);
    static void report_module_firmware(const uint8_t address);

    static void report_error_count(const int8_t idx, const AxisEnum axis) {
      CHECK_IDX();
      SERIAL_ECHOLNPGM("Error count on ", C(AXIS_CHAR(axis)), " axis is ", encoders[idx].get_error_count());
    }

    static void reset_error_count(const int8_t idx, const AxisEnum axis) {
      CHECK_IDX();
      encoders[idx].set_error_count(0);
      SERIAL_ECHOLNPGM("Error count on ", C(AXIS_CHAR(axis)), " axis has been reset.");
    }

    static void enable_ec(const int8_t idx, const bool enabled, const AxisEnum axis) {
      CHECK_IDX();
      encoders[idx].set_ec_enabled(enabled);
      SERIAL_ECHOPGM("Error correction on ", C(AXIS_CHAR(axis)));
      SERIAL_ECHO_TERNARY(encoders[idx].get_ec_enabled(), " axis is ", "en", "dis", "abled.\n");
    }

    static void set_ec_threshold(const int8_t idx, const float newThreshold, const AxisEnum axis) {
      CHECK_IDX();
      encoders[idx].set_ec_threshold(newThreshold);
      SERIAL_ECHOLNPGM("Error correct threshold for ", C(AXIS_CHAR(axis)), " axis set to ", newThreshold, "mm.");
    }

    static void get_ec_threshold(const int8_t idx, const AxisEnum axis) {
      CHECK_IDX();
      const float threshold = encoders[idx].get_ec_threshold();
      SERIAL_ECHOLNPGM("Error correct threshold for ", C(AXIS_CHAR(axis)), " axis is ", threshold, "mm.");
    }

    static int8_t idx_from_axis(const AxisEnum axis) {
      LOOP_PE(i)
        if (encoders[i].get_axis() == axis) return i;
      return -1;
    }

    static int8_t idx_from_addr(const uint8_t addr) {
      LOOP_PE(i)
        if (encoders[i].get_address() == addr) return i;
      return -1;
    }

    static int8_t parse();

    static void M860();
    static void M861();
    static void M862();
    static void M863();
    static void M864();
    static void M865();
    static void M866();
    static void M867();
    static void M868();
    static void M869();

    static I2CPositionEncoder encoders[I2CPE_ENCODER_CNT];
};

extern I2CPositionEncodersMgr I2CPEM;
