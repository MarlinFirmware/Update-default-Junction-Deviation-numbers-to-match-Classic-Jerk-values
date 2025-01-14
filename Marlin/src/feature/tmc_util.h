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

#include "../inc/MarlinConfig.h"
#include "../lcd/marlinui.h"

#if HAS_TRINAMIC_CONFIG

#include <TMCStepper.h>
#include "../module/planner.h"

#define CHOPPER_DEFAULT_12V  { 3, -1, 1 }   // { toff, hend, hstrt }
#define CHOPPER_DEFAULT_19V  { 4,  1, 1 }
#define CHOPPER_DEFAULT_24V  { 4,  2, 1 }
#define CHOPPER_DEFAULT_36V  { 5,  2, 4 }
#define CHOPPER_PRUSAMK3_24V { 3, -2, 6 }
#define CHOPPER_MARLIN_119   { 5,  2, 3 }
#define CHOPPER_09STEP_24V   { 3, -1, 5 }

#if ENABLED(MONITOR_DRIVER_STATUS) && !defined(MONITOR_DRIVER_STATUS_INTERVAL_MS)
  #define MONITOR_DRIVER_STATUS_INTERVAL_MS 500U
#endif

constexpr uint16_t _tmc_thrs(const uint16_t msteps, const uint32_t thrs, const uint32_t spmm) {
  return 12650000UL * msteps / (256 * thrs * spmm);
}

typedef struct {
  uint8_t toff;
  int8_t hend;
  uint8_t hstrt;
} chopper_timing_t;

template<char AXIS_LETTER, char DRIVER_ID>
class TMCStorage {
  protected:
    // Only a child class has access to constructor => Don't create on its own! "Poor man's abstract class"
    TMCStorage() {}

  public:
    uint16_t val_mA = 0;

    #if ENABLED(MONITOR_DRIVER_STATUS)
      uint8_t otpw_count = 0,
              error_count = 0;
      bool flag_otpw = false;
      bool getOTPW() { return flag_otpw; }
      void clear_otpw() { flag_otpw = 0; }
    #endif

    uint16_t getMilliamps() { return val_mA; }

    void printLabel() {
      SERIAL_CHAR(AXIS_LETTER);
      if (DRIVER_ID > '0') SERIAL_CHAR(DRIVER_ID);
    }

    struct {
      OPTCODE(HAS_STEALTHCHOP,  bool stealthChop_enabled = false)
      OPTCODE(HYBRID_THRESHOLD, uint16_t hybrid_thrs = 0)
      OPTCODE(USE_SENSORLESS,   int16_t  homing_thrs = 0)
    } stored;
};

template<class TMC, char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
class TMCMarlin : public TMC, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(const uint16_t cs_pin, const float RS) :
      TMC(cs_pin, RS)
      {}
    TMCMarlin(const uint16_t cs_pin, const float RS, const uint8_t axis_chain_index) :
      TMC(cs_pin, RS, axis_chain_index)
      {}
    TMCMarlin(const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK) :
      TMC(CS, RS, pinMOSI, pinMISO, pinSCK)
      {}
    TMCMarlin(const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK, const uint8_t axis_chain_index) :
      TMC(CS, RS, pinMOSI, pinMISO, pinSCK,  axis_chain_index)
      {}
    uint16_t rms_current() { return TMC::rms_current(); }
    void rms_current(uint16_t mA) {
      this->val_mA = mA;
      TMC::rms_current(mA);
    }
    void rms_current(const uint16_t mA, const float mult) {
      this->val_mA = mA;
      TMC::rms_current(mA, mult);
    }
    uint16_t get_microstep_counter() { return TMC::MSCNT(); }

    #if HAS_STEALTHCHOP
      bool get_stealthChop()                { return this->en_pwm_mode(); }
      bool get_stored_stealthChop()         { return this->stored.stealthChop_enabled; }
      void refresh_stepping_mode()          { this->en_pwm_mode(this->stored.stealthChop_enabled); }
      void set_stealthChop(const bool stch) { this->stored.stealthChop_enabled = stch; refresh_stepping_mode(); }
      bool toggle_stepping_mode()           { set_stealthChop(!this->stored.stealthChop_enabled); return get_stealthChop(); }
    #endif

    void set_chopper_times(const chopper_timing_t &ct) {
      TMC::toff(ct.toff);
      TMC::hysteresis_end(ct.hend);
      TMC::hysteresis_start(ct.hstrt);
    }

    #if ENABLED(HYBRID_THRESHOLD)
      uint32_t get_pwm_thrs() {
        return _tmc_thrs(this->microsteps(), this->TPWMTHRS(), planner.settings.axis_steps_per_mm[AXIS_ID]);
      }
      void set_pwm_thrs(const uint32_t thrs) {
        TMC::TPWMTHRS(_tmc_thrs(this->microsteps(), thrs, planner.settings.axis_steps_per_mm[AXIS_ID]));
        TERN_(HAS_MARLINUI_MENU, this->stored.hybrid_thrs = thrs);
      }
    #endif

    #if USE_SENSORLESS
      int16_t homing_threshold() { return TMC::sgt(); }
      void homing_threshold(int16_t sgt_val) {
        sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
        TMC::sgt(sgt_val);
        TERN_(HAS_MARLINUI_MENU, this->stored.homing_thrs = sgt_val);
      }
      #if ENABLED(SPI_ENDSTOPS)
        bool test_stall_status();
      #endif
    #endif

    void refresh_stepper_current() { rms_current(this->val_mA); }

    #if ENABLED(HYBRID_THRESHOLD)
      void refresh_hybrid_thrs() { set_pwm_thrs(this->stored.hybrid_thrs); }
    #endif
    #if USE_SENSORLESS
      void refresh_homing_thrs() { homing_threshold(this->stored.homing_thrs); }
    #endif

    static constexpr int8_t sgt_min = -64,
                            sgt_max =  63;
};

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
class TMCMarlin<TMC2208Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> : public TMC2208Stepper, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(Stream * SerialPort, const float RS, const uint8_t) :
      TMC2208Stepper(SerialPort, RS)
      {}
    TMCMarlin(Stream * SerialPort, const float RS, uint8_t addr, const uint16_t mul_pin1, const uint16_t mul_pin2) :
      TMC2208Stepper(SerialPort, RS, addr, mul_pin1, mul_pin2)
     {}
    TMCMarlin(const uint16_t RX, const uint16_t TX, const float RS, const uint8_t) :
      TMC2208Stepper(RX, TX, RS)
      {}

    uint16_t rms_current() { return TMC2208Stepper::rms_current(); }
    void rms_current(const uint16_t mA) {
      this->val_mA = mA;
      TMC2208Stepper::rms_current(mA);
    }
    void rms_current(const uint16_t mA, const float mult) {
      this->val_mA = mA;
      TMC2208Stepper::rms_current(mA, mult);
    }
    uint16_t get_microstep_counter() { return TMC2208Stepper::MSCNT(); }

    #if HAS_STEALTHCHOP
      bool get_stealthChop()                { return !this->en_spreadCycle(); }
      bool get_stored_stealthChop()         { return this->stored.stealthChop_enabled; }
      void refresh_stepping_mode()          { this->en_spreadCycle(!this->stored.stealthChop_enabled); }
      void set_stealthChop(const bool stch) { this->stored.stealthChop_enabled = stch; refresh_stepping_mode(); }
      bool toggle_stepping_mode()           { set_stealthChop(!this->stored.stealthChop_enabled); return get_stealthChop(); }
    #endif

    void set_chopper_times(const chopper_timing_t &ct) {
      TMC2208Stepper::toff(ct.toff);
      TMC2208Stepper::hysteresis_end(ct.hend);
      TMC2208Stepper::hysteresis_start(ct.hstrt);
    }

    #if ENABLED(HYBRID_THRESHOLD)
      uint32_t get_pwm_thrs() {
        return _tmc_thrs(this->microsteps(), this->TPWMTHRS(), planner.settings.axis_steps_per_mm[AXIS_ID]);
      }
      void set_pwm_thrs(const uint32_t thrs) {
        TMC2208Stepper::TPWMTHRS(_tmc_thrs(this->microsteps(), thrs, planner.settings.axis_steps_per_mm[AXIS_ID]));
        TERN_(HAS_MARLINUI_MENU, this->stored.hybrid_thrs = thrs);
      }
    #endif

    void refresh_stepper_current() { rms_current(this->val_mA); }

    #if ENABLED(HYBRID_THRESHOLD)
      void refresh_hybrid_thrs() { set_pwm_thrs(this->stored.hybrid_thrs); }
    #endif
};

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
class TMCMarlin<TMC2209Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> : public TMC2209Stepper, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(Stream * SerialPort, const float RS, const uint8_t addr) :
      TMC2209Stepper(SerialPort, RS, addr)
      {}
    TMCMarlin(const uint16_t RX, const uint16_t TX, const float RS, const uint8_t addr) :
      TMC2209Stepper(RX, TX, RS, addr)
      {}
    uint8_t get_address() { return slave_address; }
    uint16_t rms_current() { return TMC2209Stepper::rms_current(); }
    void rms_current(const uint16_t mA) {
      this->val_mA = mA;
      TMC2209Stepper::rms_current(mA);
    }
    void rms_current(const uint16_t mA, const float mult) {
      this->val_mA = mA;
      TMC2209Stepper::rms_current(mA, mult);
    }
    uint16_t get_microstep_counter() { return TMC2209Stepper::MSCNT(); }

    #if HAS_STEALTHCHOP
      bool get_stealthChop()                { return !this->en_spreadCycle(); }
      bool get_stored_stealthChop()         { return this->stored.stealthChop_enabled; }
      void refresh_stepping_mode()          { this->en_spreadCycle(!this->stored.stealthChop_enabled); }
      void set_stealthChop(const bool stch) { this->stored.stealthChop_enabled = stch; refresh_stepping_mode(); }
      bool toggle_stepping_mode()           { set_stealthChop(!this->stored.stealthChop_enabled); return get_stealthChop(); }
    #endif

    void set_chopper_times(const chopper_timing_t &ct) {
      TMC2209Stepper::toff(ct.toff);
      TMC2209Stepper::hysteresis_end(ct.hend);
      TMC2209Stepper::hysteresis_start(ct.hstrt);
    }

    #if ENABLED(HYBRID_THRESHOLD)
      uint32_t get_pwm_thrs() {
        return _tmc_thrs(this->microsteps(), this->TPWMTHRS(), planner.settings.axis_steps_per_mm[AXIS_ID]);
      }
      void set_pwm_thrs(const uint32_t thrs) {
        TMC2209Stepper::TPWMTHRS(_tmc_thrs(this->microsteps(), thrs, planner.settings.axis_steps_per_mm[AXIS_ID]));
        TERN_(HAS_MARLINUI_MENU, this->stored.hybrid_thrs = thrs);
      }
    #endif
    #if USE_SENSORLESS
      int16_t homing_threshold() { return TMC2209Stepper::SGTHRS(); }
      void homing_threshold(int16_t sgt_val) {
        sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
        TMC2209Stepper::SGTHRS(sgt_val);
        TERN_(HAS_MARLINUI_MENU, this->stored.homing_thrs = sgt_val);
      }
    #endif

    void refresh_stepper_current() { rms_current(this->val_mA); }

    #if ENABLED(HYBRID_THRESHOLD)
      void refresh_hybrid_thrs() { set_pwm_thrs(this->stored.hybrid_thrs); }
    #endif
    #if USE_SENSORLESS
      void refresh_homing_thrs() { homing_threshold(this->stored.homing_thrs); }
    #endif

    static constexpr uint8_t sgt_min = 0,
                             sgt_max = 255;
};

template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
class TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> : public TMC2660Stepper, public TMCStorage<AXIS_LETTER, DRIVER_ID> {
  public:
    TMCMarlin(const uint16_t cs_pin, const float RS, const uint8_t) :
      TMC2660Stepper(cs_pin, RS)
      {}
    TMCMarlin(const uint16_t CS, const float RS, const uint16_t pinMOSI, const uint16_t pinMISO, const uint16_t pinSCK, const uint8_t) :
      TMC2660Stepper(CS, RS, pinMOSI, pinMISO, pinSCK)
      {}
    uint16_t rms_current() { return TMC2660Stepper::rms_current(); }
    void rms_current(const uint16_t mA) {
      this->val_mA = mA;
      TMC2660Stepper::rms_current(mA);
    }
    uint16_t get_microstep_counter() { return TMC2660Stepper::mstep(); }

    void set_chopper_times(const chopper_timing_t &ct) {
      TMC2660Stepper::toff(ct.toff);
      TMC2660Stepper::hysteresis_end(ct.hend);
      TMC2660Stepper::hysteresis_start(ct.hstrt);
    }

    #if USE_SENSORLESS
      int16_t homing_threshold() { return TMC2660Stepper::sgt(); }
      void homing_threshold(int16_t sgt_val) {
        sgt_val = (int16_t)constrain(sgt_val, sgt_min, sgt_max);
        TMC2660Stepper::sgt(sgt_val);
        TERN_(HAS_MARLINUI_MENU, this->stored.homing_thrs = sgt_val);
      }
    #endif

    void refresh_stepper_current() { rms_current(this->val_mA); }

    #if USE_SENSORLESS
      void refresh_homing_thrs() { homing_threshold(this->stored.homing_thrs); }
    #endif

    static constexpr int8_t sgt_min = -64,
                            sgt_max =  63;
};

void monitor_tmc_drivers();
void test_tmc_connection(LOGICAL_AXIS_DECL_LC(const bool, true));

#if ENABLED(TMC_DEBUG)
  #if ENABLED(MONITOR_DRIVER_STATUS)
    void tmc_set_report_interval(const uint16_t update_interval);
  #endif
  void tmc_report_all(LOGICAL_AXIS_DECL_LC(const bool, true));
  void tmc_get_registers(LOGICAL_AXIS_ARGS_LC(const bool));
#endif

/**
 * TMC2130-specific sensorless homing using stallGuard2.
 * stallGuard2 only works when in spreadCycle mode.
 * spreadCycle and stealthChop are mutually-exclusive.
 *
 * Defined here because of limitations with templates and headers.
 */
#if USE_SENSORLESS

  // Track enabled status of stealthChop and only re-enable where applicable
  struct sensorless_t { bool NUM_AXIS_ARGS_() x2, y2, z2, z3, z4; };

  #if ENABLED(IMPROVE_HOMING_RELIABILITY)
    extern millis_t sg_guard_period;
    constexpr uint16_t default_sg_guard_duration = 400;
  #endif

  bool tmc_enable_stallguard(TMC2130Stepper &st);
  void tmc_disable_stallguard(TMC2130Stepper &st, const bool restore_stealth);

  bool tmc_enable_stallguard(TMC2209Stepper &st);
  void tmc_disable_stallguard(TMC2209Stepper &st, const bool restore_stealth);

  bool tmc_enable_stallguard(TMC2660Stepper);
  void tmc_disable_stallguard(TMC2660Stepper, const bool);

  #if ENABLED(SPI_ENDSTOPS)

    template<class TMC, char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
    bool TMCMarlin<TMC, AXIS_LETTER, DRIVER_ID, AXIS_ID>::test_stall_status() {
      this->switchCSpin(LOW);

      // read stallGuard flag from TMC library, will handle HW and SW SPI
      TMC2130_n::DRV_STATUS_t drv_status{0};
      drv_status.sr = this->DRV_STATUS();

      this->switchCSpin(HIGH);

      return drv_status.stallGuard;
    }

  #endif // SPI_ENDSTOPS

#endif // USE_SENSORLESS

#endif // HAS_TRINAMIC_CONFIG

#if HAS_TMC_SPI
  void tmc_init_cs_pins();
#endif
