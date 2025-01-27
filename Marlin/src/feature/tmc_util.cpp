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

#if HAS_TRINAMIC_CONFIG

#include "tmc_util.h"
#include "../MarlinCore.h"

#include "../module/stepper/indirection.h"
#include "../module/printcounter.h"
#include "../libs/duration_t.h"
#include "../gcode/gcode.h"

#if ENABLED(SOVOL_SV06_RTS)
  #include "../lcd/sovol_rts/sovol_rts.h"
#endif

#if ENABLED(TMC_DEBUG)
  #include "../libs/hex_print.h"
  #if ENABLED(MONITOR_DRIVER_STATUS)
    static uint16_t report_tmc_status_interval; // = 0
  #endif
#endif

/**
 * Check for over temperature or short to ground error flags.
 * Report and log warning of overtemperature condition.
 * Reduce driver current in a persistent otpw condition.
 * Keep track of otpw counter so we don't reduce current on a single instance,
 * and so we don't repeatedly report warning before the condition is cleared.
 */
#if ENABLED(MONITOR_DRIVER_STATUS)

  struct TMC_driver_data {
    uint32_t drv_status;
    bool is_otpw:1,
         is_ot:1,
         is_s2g:1,
         is_error:1
         #if ENABLED(TMC_DEBUG)
           , is_stall:1
           , is_stealth:1
           , is_standstill:1
           #if HAS_STALLGUARD
             , sg_result_reasonable:1
           #endif
         #endif
      ;
    #if ENABLED(TMC_DEBUG)
      #if HAS_TMCX1X0 || HAS_TMC220x
        uint8_t cs_actual;
      #endif
      #if HAS_STALLGUARD
        uint16_t sg_result;
      #endif
    #endif
  };

  #if HAS_TMCX1X0

    #if ENABLED(TMC_DEBUG)
      static uint32_t get_pwm_scale(TMC2130Stepper &st) { return st.PWM_SCALE(); }
    #endif

    static TMC_driver_data get_driver_data(TMC2130Stepper &st) {
      constexpr uint8_t OT_bp = 25, OTPW_bp = 26;
      constexpr uint32_t S2G_bm = 0x18000000;
      #if ENABLED(TMC_DEBUG)
        constexpr uint16_t SG_RESULT_bm = 0x3FF; // 0:9
        constexpr uint8_t STEALTH_bp = 14;
        constexpr uint32_t CS_ACTUAL_bm = 0x1F0000; // 16:20
        constexpr uint8_t STALL_GUARD_bp = 24;
        constexpr uint8_t STST_bp = 31;
      #endif
      TMC_driver_data data;
      const auto ds = data.drv_status = st.DRV_STATUS();
      #ifdef __AVR__

        // 8-bit optimization saves up to 70 bytes of PROGMEM per axis
        uint8_t spart;
        #if ENABLED(TMC_DEBUG)
          data.sg_result = ds & SG_RESULT_bm;
          spart = ds >> 8;
          data.is_stealth = TEST(spart, STEALTH_bp - 8);
          spart = ds >> 16;
          data.cs_actual = spart & (CS_ACTUAL_bm >> 16);
        #endif
        spart = ds >> 24;
        data.is_ot = TEST(spart, OT_bp - 24);
        data.is_otpw = TEST(spart, OTPW_bp - 24);
        data.is_s2g = !!(spart & (S2G_bm >> 24));
        #if ENABLED(TMC_DEBUG)
          data.is_stall = TEST(spart, STALL_GUARD_bp - 24);
          data.is_standstill = TEST(spart, STST_bp - 24);
          data.sg_result_reasonable = !data.is_standstill; // sg_result has no reasonable meaning while standstill
        #endif

      #else // !__AVR__

        data.is_ot = TEST(ds, OT_bp);
        data.is_otpw = TEST(ds, OTPW_bp);
        data.is_s2g = !!(ds & S2G_bm);
        #if ENABLED(TMC_DEBUG)
          constexpr uint8_t CS_ACTUAL_sb = 16;
          data.sg_result = ds & SG_RESULT_bm;
          data.is_stealth = TEST(ds, STEALTH_bp);
          data.cs_actual = (ds & CS_ACTUAL_bm) >> CS_ACTUAL_sb;
          data.is_stall = TEST(ds, STALL_GUARD_bp);
          data.is_standstill = TEST(ds, STST_bp);
          data.sg_result_reasonable = !data.is_standstill; // sg_result has no reasonable meaning while standstill
        #endif

      #endif // !__AVR__

      return data;
    }

  #endif // HAS_TMCX1X0

  #if HAS_TMC220x

    #if ENABLED(TMC_DEBUG)
      static uint32_t get_pwm_scale(TMC2208Stepper &st) { return st.pwm_scale_sum(); }
    #endif

    static TMC_driver_data get_driver_data(TMC2208Stepper &st) {
      constexpr uint8_t OTPW_bp = 0, OT_bp = 1;
      constexpr uint8_t S2G_bm = 0b111100; // 2..5
      TMC_driver_data data;
      const auto ds = data.drv_status = st.DRV_STATUS();
      data.is_otpw = TEST(ds, OTPW_bp);
      data.is_ot = TEST(ds, OT_bp);
      data.is_s2g = !!(ds & S2G_bm);
      #if ENABLED(TMC_DEBUG)
        constexpr uint32_t CS_ACTUAL_bm = 0x1F0000; // 16:20
        constexpr uint8_t STEALTH_bp = 30, STST_bp = 31;
        #ifdef __AVR__
          // 8-bit optimization saves up to 12 bytes of PROGMEM per axis
          uint8_t spart = ds >> 16;
          data.cs_actual = spart & (CS_ACTUAL_bm >> 16);
          spart = ds >> 24;
          data.is_stealth = TEST(spart, STEALTH_bp - 24);
          data.is_standstill = TEST(spart, STST_bp - 24);
        #else
          constexpr uint8_t CS_ACTUAL_sb = 16;
          data.cs_actual = (ds & CS_ACTUAL_bm) >> CS_ACTUAL_sb;
          data.is_stealth = TEST(ds, STEALTH_bp);
          data.is_standstill = TEST(ds, STST_bp);
        #endif
        TERN_(HAS_STALLGUARD, data.sg_result_reasonable = false);
      #endif
      return data;
    }

  #endif // TMC2208 || TMC2209

  #if HAS_DRIVER(TMC2660)

    #if ENABLED(TMC_DEBUG)
      static uint32_t get_pwm_scale(TMC2660Stepper) { return 0; }
    #endif

    static TMC_driver_data get_driver_data(TMC2660Stepper &st) {
      constexpr uint8_t OT_bp = 1, OTPW_bp = 2;
      constexpr uint8_t S2G_bm = 0b11000;
      TMC_driver_data data;
      const auto ds = data.drv_status = st.DRVSTATUS();
      uint8_t spart = ds & 0xFF;
      data.is_otpw = TEST(spart, OTPW_bp);
      data.is_ot = TEST(spart, OT_bp);
      data.is_s2g = !!(ds & S2G_bm);
      #if ENABLED(TMC_DEBUG)
        constexpr uint8_t STALL_GUARD_bp = 0;
        constexpr uint8_t STST_bp = 7, SG_RESULT_sp = 10;
        constexpr uint32_t SG_RESULT_bm = 0xFFC00; // 10:19
        data.is_stall = TEST(spart, STALL_GUARD_bp);
        data.is_standstill = TEST(spart, STST_bp);
        data.sg_result = (ds & SG_RESULT_bm) >> SG_RESULT_sp;
        data.sg_result_reasonable = true;
      #endif
      return data;
    }

  #endif // TMC2660

  #if ENABLED(STOP_ON_ERROR)
    void report_driver_error(const TMC_driver_data &data) {
      SERIAL_ECHOPGM(" driver error detected: 0x");
      SERIAL_PRINTLN(data.drv_status, PrintBase::Hex);
      if (data.is_ot) SERIAL_ECHOLNPGM("overtemperature");
      if (data.is_s2g) SERIAL_ECHOLNPGM("coil short circuit");
      TERN_(TMC_DEBUG, tmc_report_all());
      TERN_(SOVOL_SV06_RTS, rts.gotoPage(ID_DriverError_L, ID_DriverError_D));
      kill(F("Driver error"));
    }
  #endif

  template<typename TMC>
  void report_driver_otpw(TMC &st) {
    MString<13> timestamp;
    duration_t elapsed = print_job_timer.duration();
    const bool has_days = (elapsed.value > 60*60*24L);
    (void)elapsed.toDigital(&timestamp, has_days);
    TSS('\n', timestamp, F(": ")).echo();
    st.printLabel();
    SString<50>(F(" driver overtemperature warning! ("), st.getMilliamps(), F("mA)")).echoln();
  }

  template<typename TMC>
  void report_polled_driver_data(TMC &st, const TMC_driver_data &data) {
    const uint32_t pwm_scale = get_pwm_scale(st);
    st.printLabel();
    SString<60> report(':', pwm_scale);
    #if ENABLED(TMC_DEBUG)
      #if HAS_TMCX1X0 || HAS_TMC220x
        report.append('/', data.cs_actual);
      #endif
      #if HAS_STALLGUARD
        report += '/';
        if (data.sg_result_reasonable)
          report += data.sg_result;
        else
          report += '-';
      #endif
    #endif
    report += '|';
    if (st.error_count)       report += 'E'; // Error
    if (data.is_ot)           report += 'O'; // Over-temperature
    if (data.is_otpw)         report += 'W'; // over-temperature pre-Warning
    #if ENABLED(TMC_DEBUG)
      if (data.is_stall)      report += 'G'; // stallGuard
      if (data.is_stealth)    report += 'T'; // stealthChop
      if (data.is_standstill) report += 'I'; // standstIll
    #endif
    if (st.flag_otpw)         report += 'F'; // otpw Flag
    report += '|';
    if (st.otpw_count > 0)    report += st.otpw_count;
    report += '\t';
    report.echo();
  }

  #if CURRENT_STEP_DOWN > 0

    template<typename TMC>
    void step_current_down(TMC &st) {
      if (st.isEnabled()) {
        const uint16_t I_rms = st.getMilliamps() - (CURRENT_STEP_DOWN);
        if (I_rms > 50) {
          st.rms_current(I_rms);
          #if ENABLED(REPORT_CURRENT_CHANGE)
            st.printLabel();
            SERIAL_ECHOLNPGM(" current decreased to ", I_rms);
          #endif
        }
      }
    }

  #else

    #define step_current_down(...)

  #endif

  template<typename TMC>
  bool monitor_tmc_driver(TMC &st, const bool need_update_error_counters, const bool need_debug_reporting) {
    TMC_driver_data data = get_driver_data(st);
    if (data.drv_status == 0xFFFFFFFF || data.drv_status == 0x0) return false;

    bool should_step_down = false;

    if (need_update_error_counters) {
      if (data.is_ot | data.is_s2g) st.error_count++;
      else if (st.error_count > 0) st.error_count--;

      #if ENABLED(STOP_ON_ERROR)
        if (st.error_count >= 10) {
          SERIAL_EOL();
          st.printLabel();
          report_driver_error(data);
        }
      #endif

      // Report if a warning was triggered
      if (data.is_otpw && st.otpw_count == 0)
        report_driver_otpw(st);

      #if CURRENT_STEP_DOWN > 0
        // Decrease current if is_otpw is true and driver is enabled and there's been more than 4 warnings
        if (data.is_otpw && st.otpw_count > 4 && st.isEnabled())
          should_step_down = true;
      #endif

      if (data.is_otpw) {
        st.otpw_count++;
        st.flag_otpw = true;
      }
      else if (st.otpw_count > 0) st.otpw_count = 0;
    }

    #if ENABLED(TMC_DEBUG)
      if (need_debug_reporting) report_polled_driver_data(st, data);
    #endif

    return should_step_down;
  }

  void monitor_tmc_drivers() {
    const millis_t ms = millis();

    // Poll TMC drivers at the configured interval
    static millis_t next_poll = 0;
    const bool need_update_error_counters = ELAPSED(ms, next_poll);
    if (need_update_error_counters) next_poll = ms + MONITOR_DRIVER_STATUS_INTERVAL_MS;

    // Also poll at intervals for debugging
    #if ENABLED(TMC_DEBUG)
      static millis_t next_debug_reporting = 0;
      const bool need_debug_reporting = report_tmc_status_interval && ELAPSED(ms, next_debug_reporting);
      if (need_debug_reporting) next_debug_reporting = ms + report_tmc_status_interval;
    #else
      constexpr bool need_debug_reporting = false;
    #endif

    if (need_update_error_counters || need_debug_reporting) {

      #if AXIS_IS_TMC(X) || AXIS_IS_TMC(X2)
      {
        bool result = false;
        #if AXIS_IS_TMC(X)
          if (monitor_tmc_driver(stepperX, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        #if AXIS_IS_TMC(X2)
          if (monitor_tmc_driver(stepperX2, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        if (result) {
          #if AXIS_IS_TMC(X)
            step_current_down(stepperX);
          #endif
          #if AXIS_IS_TMC(X2)
            step_current_down(stepperX2);
          #endif
        }
      }
      #endif

      #if AXIS_IS_TMC(Y) || AXIS_IS_TMC(Y2)
      {
        bool result = false;
        #if AXIS_IS_TMC(Y)
          if (monitor_tmc_driver(stepperY, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        #if AXIS_IS_TMC(Y2)
          if (monitor_tmc_driver(stepperY2, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        if (result) {
          #if AXIS_IS_TMC(Y)
            step_current_down(stepperY);
          #endif
          #if AXIS_IS_TMC(Y2)
            step_current_down(stepperY2);
          #endif
        }
      }
      #endif

      #if AXIS_IS_TMC(Z) || AXIS_IS_TMC(Z2) || AXIS_IS_TMC(Z3) || AXIS_IS_TMC(Z4)
      {
        bool result = false;
        #if AXIS_IS_TMC(Z)
          if (monitor_tmc_driver(stepperZ, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        #if AXIS_IS_TMC(Z2)
          if (monitor_tmc_driver(stepperZ2, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        #if AXIS_IS_TMC(Z3)
          if (monitor_tmc_driver(stepperZ3, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        #if AXIS_IS_TMC(Z4)
          if (monitor_tmc_driver(stepperZ4, need_update_error_counters, need_debug_reporting)) result = true;
        #endif
        if (result) {
          #if AXIS_IS_TMC(Z)
            step_current_down(stepperZ);
          #endif
          #if AXIS_IS_TMC(Z2)
            step_current_down(stepperZ2);
          #endif
          #if AXIS_IS_TMC(Z3)
            step_current_down(stepperZ3);
          #endif
          #if AXIS_IS_TMC(Z4)
            step_current_down(stepperZ4);
          #endif
        }
      }
      #endif

      #if AXIS_IS_TMC(I)
        if (monitor_tmc_driver(stepperI, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperI);
      #endif
      #if AXIS_IS_TMC(J)
        if (monitor_tmc_driver(stepperJ, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperJ);
      #endif
      #if AXIS_IS_TMC(K)
        if (monitor_tmc_driver(stepperK, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperK);
      #endif
      #if AXIS_IS_TMC(U)
        if (monitor_tmc_driver(stepperU, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperU);
      #endif
      #if AXIS_IS_TMC(V)
        if (monitor_tmc_driver(stepperV, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperV);
      #endif
      #if AXIS_IS_TMC(W)
        if (monitor_tmc_driver(stepperW, need_update_error_counters, need_debug_reporting))
          step_current_down(stepperW);
      #endif

      #if AXIS_IS_TMC(E0)
        (void)monitor_tmc_driver(stepperE0, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E1)
        (void)monitor_tmc_driver(stepperE1, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E2)
        (void)monitor_tmc_driver(stepperE2, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E3)
        (void)monitor_tmc_driver(stepperE3, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E4)
        (void)monitor_tmc_driver(stepperE4, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E5)
        (void)monitor_tmc_driver(stepperE5, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E6)
        (void)monitor_tmc_driver(stepperE6, need_update_error_counters, need_debug_reporting);
      #endif
      #if AXIS_IS_TMC(E7)
        (void)monitor_tmc_driver(stepperE7, need_update_error_counters, need_debug_reporting);
      #endif

      if (TERN0(TMC_DEBUG, need_debug_reporting)) SERIAL_EOL();
    }
  }

#endif // MONITOR_DRIVER_STATUS

#if ENABLED(TMC_DEBUG)

  /**
   * M122 [S<0|1>] [Pnnn] Enable periodic status reports
   */
  #if ENABLED(MONITOR_DRIVER_STATUS)
    void tmc_set_report_interval(const uint16_t update_interval) {
      if ((report_tmc_status_interval = update_interval))
        SERIAL_ECHOLNPGM("axis:pwm_scale"
          TERN_(HAS_STEALTHCHOP, "/curr_scale")
          TERN_(HAS_STALLGUARD, "/mech_load")
          "|flags|warncount"
        );
    }
  #endif

  enum TMC_debug_enum : char {
    TMC_CODES,
    TMC_UART_ADDR,
    TMC_ENABLED,
    TMC_CURRENT,
    TMC_RMS_CURRENT,
    TMC_MAX_CURRENT,
    TMC_IRUN,
    TMC_IHOLD,
    TMC_GLOBAL_SCALER,
    TMC_CS_ACTUAL,
    TMC_PWM_SCALE,
    TMC_PWM_SCALE_SUM,
    TMC_PWM_SCALE_AUTO,
    TMC_PWM_OFS_AUTO,
    TMC_PWM_GRAD_AUTO,
    TMC_VSENSE,
    TMC_STEALTHCHOP,
    TMC_MICROSTEPS,
    TMC_TSTEP,
    TMC_TPWMTHRS,
    TMC_TPWMTHRS_MMS,
    TMC_OTPW,
    TMC_OTPW_TRIGGERED,
    TMC_TOFF,
    TMC_TBL,
    TMC_HEND,
    TMC_HSTRT,
    TMC_SGT,
    TMC_MSCNT,
    TMC_INTERPOLATE
  };
  enum TMC_drv_status_enum : char {
    TMC_DRV_CODES,
    TMC_STST,
    TMC_OLB,
    TMC_OLA,
    TMC_S2GB,
    TMC_S2GA,
    TMC_DRV_OTPW,
    TMC_OT,
    TMC_STALLGUARD,
    TMC_DRV_CS_ACTUAL,
    TMC_FSACTIVE,
    TMC_SG_RESULT,
    TMC_DRV_STATUS_HEX,
    TMC_T157,
    TMC_T150,
    TMC_T143,
    TMC_T120,
    TMC_STEALTH,
    TMC_S2VSB,
    TMC_S2VSA
  };
  enum TMC_get_registers_enum : char {
    TMC_AXIS_CODES,
    TMC_GET_GCONF,
    TMC_GET_IHOLD_IRUN,
    TMC_GET_GSTAT,
    TMC_GET_IOIN,
    TMC_GET_TPOWERDOWN,
    TMC_GET_TSTEP,
    TMC_GET_TPWMTHRS,
    TMC_GET_TCOOLTHRS,
    TMC_GET_THIGH,
    TMC_GET_CHOPCONF,
    TMC_GET_COOLCONF,
    TMC_GET_PWMCONF,
    TMC_GET_PWM_SCALE,
    TMC_GET_DRV_STATUS,
    TMC_GET_DRVCONF,
    TMC_GET_DRVCTRL,
    TMC_GET_DRVSTATUS,
    TMC_GET_SGCSCONF,
    TMC_GET_SMARTEN
  };

  template<class TMC>
  static void print_vsense(TMC &st) { SERIAL_ECHO(st.vsense() ? F("1=.18") : F("0=.325")); }

  #if HAS_DRIVER(TMC2130) || HAS_DRIVER(TMC5130)
    static void _tmc_status(TMC2130Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_ECHO(st.PWM_SCALE()); break;
        case TMC_SGT: SERIAL_ECHO(st.sgt()); break;
        case TMC_STEALTHCHOP: serialprint_truefalse(st.en_pwm_mode()); break;
        case TMC_INTERPOLATE: serialprint_truefalse(st.intpol()); break;
        default: break;
      }
    }
  #endif
  #if HAS_TMCX1X0
    static void _tmc_parse_drv_status(TMC2130Stepper &st, const TMC_drv_status_enum i) {
      switch (i) {
        case TMC_STALLGUARD: if (st.stallguard()) SERIAL_CHAR('*'); break;
        case TMC_SG_RESULT:  SERIAL_ECHO(st.sg_result()); break;
        case TMC_FSACTIVE:   if (st.fsactive())   SERIAL_CHAR('*'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_ECHO(st.cs_actual()); break;
        default: break;
      }
    }
  #endif

  #if HAS_DRIVER(TMC2160) || HAS_DRIVER(TMC5160)
    template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
    void print_vsense(TMCMarlin<TMC2160Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &) { }

    template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
    void print_vsense(TMCMarlin<TMC5160Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &) { }

    static void _tmc_status(TMC2160Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_PWM_SCALE: SERIAL_ECHO(st.PWM_SCALE()); break;
        case TMC_SGT: SERIAL_ECHO(st.sgt()); break;
        case TMC_STEALTHCHOP: serialprint_truefalse(st.en_pwm_mode()); break;
        case TMC_GLOBAL_SCALER:
          {
            const uint16_t value = st.GLOBAL_SCALER();
            SERIAL_ECHO(value ?: 256);
            SERIAL_ECHOPGM("/256");
          }
          break;
        case TMC_INTERPOLATE: serialprint_truefalse(st.intpol()); break;
        default: break;
      }
    }
  #endif // TMC2160 || TMC5160

  #if HAS_TMC220x

    static void _tmc_status(TMC2208Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_PWM_SCALE_SUM: SERIAL_ECHO(st.pwm_scale_sum()); break;
        case TMC_PWM_SCALE_AUTO: SERIAL_ECHO(st.pwm_scale_auto()); break;
        case TMC_PWM_OFS_AUTO: SERIAL_ECHO(st.pwm_ofs_auto()); break;
        case TMC_PWM_GRAD_AUTO: SERIAL_ECHO(st.pwm_grad_auto()); break;
        case TMC_STEALTHCHOP: serialprint_truefalse(st.stealth()); break;
        case TMC_INTERPOLATE: serialprint_truefalse(st.intpol()); break;
        default: break;
      }
    }

    #if HAS_DRIVER(TMC2209)
      template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
      static void _tmc_status(TMCMarlin<TMC2209Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const TMC_debug_enum i) {
        switch (i) {
          case TMC_SGT:       SERIAL_ECHO(st.SGTHRS()); break;
          case TMC_UART_ADDR: SERIAL_ECHO(st.get_address()); break;
          default:
            TMC2208Stepper *parent = &st;
            _tmc_status(*parent, i);
            break;
        }
      }
    #endif

    static void _tmc_parse_drv_status(TMC2208Stepper &st, const TMC_drv_status_enum i) {
      switch (i) {
        case TMC_T157: if (st.t157()) SERIAL_CHAR('*'); break;
        case TMC_T150: if (st.t150()) SERIAL_CHAR('*'); break;
        case TMC_T143: if (st.t143()) SERIAL_CHAR('*'); break;
        case TMC_T120: if (st.t120()) SERIAL_CHAR('*'); break;
        case TMC_S2VSA: if (st.s2vsa()) SERIAL_CHAR('*'); break;
        case TMC_S2VSB: if (st.s2vsb()) SERIAL_CHAR('*'); break;
        case TMC_DRV_CS_ACTUAL: SERIAL_ECHO(st.cs_actual()); break;
        default: break;
      }
    }

    #if HAS_DRIVER(TMC2209)
      static void _tmc_parse_drv_status(TMC2209Stepper &st, const TMC_drv_status_enum i) {
        switch (i) {
          case TMC_SG_RESULT: SERIAL_ECHO(st.SG_RESULT()); break;
          default:            _tmc_parse_drv_status(static_cast<TMC2208Stepper &>(st), i); break;
        }
      }
    #endif

  #endif // HAS_TMC220x

  #if HAS_DRIVER(TMC2660)
    static void _tmc_parse_drv_status(TMC2660Stepper, const TMC_drv_status_enum) { }
    static void _tmc_status(TMC2660Stepper &st, const TMC_debug_enum i) {
      switch (i) {
        case TMC_INTERPOLATE: serialprint_truefalse(st.intpol()); break;
        default: break;
      }
    }
  #endif

  template <typename TMC>
  static void tmc_status(TMC &st, const TMC_debug_enum i) {
    SERIAL_CHAR('\t');
    switch (i) {
      case TMC_CODES: st.printLabel(); break;
      case TMC_ENABLED: serialprint_truefalse(st.isEnabled()); break;
      case TMC_CURRENT: SERIAL_ECHO(st.getMilliamps()); break;
      case TMC_RMS_CURRENT: SERIAL_ECHO(st.rms_current()); break;
      case TMC_MAX_CURRENT: SERIAL_ECHO(p_float_t(st.rms_current() * 1.41, 0)); break;
      case TMC_IRUN:
        SERIAL_ECHO(st.irun());
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_IHOLD:
        SERIAL_ECHO(st.ihold());
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_CS_ACTUAL:
        SERIAL_ECHO(st.cs_actual());
        SERIAL_ECHOPGM("/31");
        break;
      case TMC_VSENSE: print_vsense(st); break;
      case TMC_MICROSTEPS: SERIAL_ECHO(st.microsteps()); break;
      case TMC_TSTEP: {
        const uint32_t tstep_value = st.TSTEP();
        if (tstep_value != 0xFFFFF) SERIAL_ECHO(tstep_value); else SERIAL_ECHOPGM("max");
      } break;
      #if ENABLED(HYBRID_THRESHOLD)
        case TMC_TPWMTHRS: SERIAL_ECHO(uint32_t(st.TPWMTHRS())); break;
        case TMC_TPWMTHRS_MMS: {
          const uint32_t tpwmthrs_val = st.get_pwm_thrs();
          if (tpwmthrs_val) SERIAL_ECHO(tpwmthrs_val); else SERIAL_CHAR('-');
        } break;
      #endif
      case TMC_OTPW: serialprint_truefalse(st.otpw()); break;
      #if ENABLED(MONITOR_DRIVER_STATUS)
        case TMC_OTPW_TRIGGERED: serialprint_truefalse(st.getOTPW()); break;
      #endif
      case TMC_TOFF: SERIAL_ECHO(st.toff()); break;
      case TMC_TBL: SERIAL_ECHO(st.blank_time()); break;
      case TMC_HEND: SERIAL_ECHO(st.hysteresis_end()); break;
      case TMC_HSTRT: SERIAL_ECHO(st.hysteresis_start()); break;
      case TMC_MSCNT: SERIAL_ECHO(st.get_microstep_counter()); break;
      default: _tmc_status(st, i); break;
    }
  }

  #if HAS_DRIVER(TMC2660)
    template<char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
    void tmc_status(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const TMC_debug_enum i) {
      SERIAL_CHAR('\t');
      switch (i) {
        case TMC_CODES: st.printLabel(); break;
        case TMC_ENABLED: serialprint_truefalse(st.isEnabled()); break;
        case TMC_CURRENT: SERIAL_ECHO(st.getMilliamps()); break;
        case TMC_RMS_CURRENT: SERIAL_ECHO(st.rms_current()); break;
        case TMC_MAX_CURRENT: SERIAL_ECHO(p_float_t(st.rms_current() * 1.41, 0)); break;
        case TMC_IRUN:
          SERIAL_ECHO(st.cs());
          SERIAL_ECHOPGM("/31");
          break;
        case TMC_VSENSE: SERIAL_ECHO(st.vsense() ? F("1=.165") : F("0=.310")); break;
        case TMC_MICROSTEPS: SERIAL_ECHO(st.microsteps()); break;
        //case TMC_OTPW: serialprint_truefalse(st.otpw()); break;
        //case TMC_OTPW_TRIGGERED: serialprint_truefalse(st.getOTPW()); break;
        case TMC_SGT: SERIAL_ECHO(st.sgt()); break;
        case TMC_TOFF: SERIAL_ECHO(st.toff()); break;
        case TMC_TBL: SERIAL_ECHO(st.blank_time()); break;
        case TMC_HEND: SERIAL_ECHO(st.hysteresis_end()); break;
        case TMC_HSTRT: SERIAL_ECHO(st.hysteresis_start()); break;
        default: break;
      }
    }
  #endif

  template <typename TMC>
  static void tmc_parse_drv_status(TMC &st, const TMC_drv_status_enum i) {
    SERIAL_CHAR('\t');
    switch (i) {
      case TMC_DRV_CODES:     st.printLabel();  break;
      case TMC_STST:          if (!st.stst())   SERIAL_CHAR('*'); break;
      case TMC_OLB:           if (st.olb())     SERIAL_CHAR('*'); break;
      case TMC_OLA:           if (st.ola())     SERIAL_CHAR('*'); break;
      case TMC_S2GB:          if (st.s2gb())    SERIAL_CHAR('*'); break;
      case TMC_S2GA:          if (st.s2ga())    SERIAL_CHAR('*'); break;
      case TMC_DRV_OTPW:      if (st.otpw())    SERIAL_CHAR('*'); break;
      case TMC_OT:            if (st.ot())      SERIAL_CHAR('*'); break;
      case TMC_DRV_STATUS_HEX: {
        const uint32_t drv_status = st.DRV_STATUS();
        SERIAL_CHAR('\t');
        st.printLabel();
        SERIAL_CHAR('\t');
        print_hex_long(drv_status, ':', true);
        if (drv_status == 0xFFFFFFFF || drv_status == 0) SERIAL_ECHOPGM("\t Bad response!");
        SERIAL_EOL();
        break;
      }
      default: _tmc_parse_drv_status(st, i); break;
    }
  }

  static void tmc_debug_loop(const TMC_debug_enum n OPTARGS_LOGICAL(const bool)) {
    if (TERN0(HAS_X_AXIS, x)) {
      #if AXIS_IS_TMC(X)
        tmc_status(stepperX, n);
      #endif
      #if AXIS_IS_TMC(X2)
        tmc_status(stepperX2, n);
      #endif
    }

    if (TERN0(HAS_Y_AXIS, y)) {
      #if AXIS_IS_TMC(Y)
        tmc_status(stepperY, n);
      #endif
      #if AXIS_IS_TMC(Y2)
        tmc_status(stepperY2, n);
      #endif
    }

    if (TERN0(HAS_Z_AXIS, z)) {
      #if AXIS_IS_TMC(Z)
        tmc_status(stepperZ, n);
      #endif
      #if AXIS_IS_TMC(Z2)
        tmc_status(stepperZ2, n);
      #endif
      #if AXIS_IS_TMC(Z3)
        tmc_status(stepperZ3, n);
      #endif
      #if AXIS_IS_TMC(Z4)
        tmc_status(stepperZ4, n);
      #endif
    }

    #if AXIS_IS_TMC(I)
      if (i) tmc_status(stepperI, n);
    #endif
    #if AXIS_IS_TMC(J)
      if (j) tmc_status(stepperJ, n);
    #endif
    #if AXIS_IS_TMC(K)
      if (k) tmc_status(stepperK, n);
    #endif
    #if AXIS_IS_TMC(U)
      if (u) tmc_status(stepperU, n);
    #endif
    #if AXIS_IS_TMC(V)
      if (v) tmc_status(stepperV, n);
    #endif
    #if AXIS_IS_TMC(W)
      if (w) tmc_status(stepperW, n);
    #endif

    if (TERN0(HAS_EXTRUDERS, e)) {
      #if AXIS_IS_TMC(E0)
        tmc_status(stepperE0, n);
      #endif
      #if AXIS_IS_TMC(E1)
        tmc_status(stepperE1, n);
      #endif
      #if AXIS_IS_TMC(E2)
        tmc_status(stepperE2, n);
      #endif
      #if AXIS_IS_TMC(E3)
        tmc_status(stepperE3, n);
      #endif
      #if AXIS_IS_TMC(E4)
        tmc_status(stepperE4, n);
      #endif
      #if AXIS_IS_TMC(E5)
        tmc_status(stepperE5, n);
      #endif
      #if AXIS_IS_TMC(E6)
        tmc_status(stepperE6, n);
      #endif
      #if AXIS_IS_TMC(E7)
        tmc_status(stepperE7, n);
      #endif
    }

    SERIAL_EOL();
  }

  static void drv_status_loop(const TMC_drv_status_enum n OPTARGS_LOGICAL(const bool)) {
    if (TERN0(HAS_X_AXIS, x)) {
      #if AXIS_IS_TMC(X)
        tmc_parse_drv_status(stepperX, n);
      #endif
      #if AXIS_IS_TMC(X2)
        tmc_parse_drv_status(stepperX2, n);
      #endif
    }

    if (TERN0(HAS_Y_AXIS, y)) {
      #if AXIS_IS_TMC(Y)
        tmc_parse_drv_status(stepperY, n);
      #endif
      #if AXIS_IS_TMC(Y2)
        tmc_parse_drv_status(stepperY2, n);
      #endif
    }

    if (TERN0(HAS_Z_AXIS, z)) {
      #if AXIS_IS_TMC(Z)
        tmc_parse_drv_status(stepperZ, n);
      #endif
      #if AXIS_IS_TMC(Z2)
        tmc_parse_drv_status(stepperZ2, n);
      #endif
      #if AXIS_IS_TMC(Z3)
        tmc_parse_drv_status(stepperZ3, n);
      #endif
      #if AXIS_IS_TMC(Z4)
        tmc_parse_drv_status(stepperZ4, n);
      #endif
    }

    #if AXIS_IS_TMC(I)
      if (i) tmc_parse_drv_status(stepperI, n);
    #endif
    #if AXIS_IS_TMC(J)
      if (j) tmc_parse_drv_status(stepperJ, n);
    #endif
    #if AXIS_IS_TMC(K)
      if (k) tmc_parse_drv_status(stepperK, n);
    #endif
    #if AXIS_IS_TMC(U)
      if (u) tmc_parse_drv_status(stepperU, n);
    #endif
    #if AXIS_IS_TMC(V)
      if (v) tmc_parse_drv_status(stepperV, n);
    #endif
    #if AXIS_IS_TMC(W)
      if (w) tmc_parse_drv_status(stepperW, n);
    #endif

    if (TERN0(HAS_EXTRUDERS, e)) {
      #if AXIS_IS_TMC(E0)
        tmc_parse_drv_status(stepperE0, n);
      #endif
      #if AXIS_IS_TMC(E1)
        tmc_parse_drv_status(stepperE1, n);
      #endif
      #if AXIS_IS_TMC(E2)
        tmc_parse_drv_status(stepperE2, n);
      #endif
      #if AXIS_IS_TMC(E3)
        tmc_parse_drv_status(stepperE3, n);
      #endif
      #if AXIS_IS_TMC(E4)
        tmc_parse_drv_status(stepperE4, n);
      #endif
      #if AXIS_IS_TMC(E5)
        tmc_parse_drv_status(stepperE5, n);
      #endif
      #if AXIS_IS_TMC(E6)
        tmc_parse_drv_status(stepperE6, n);
      #endif
      #if AXIS_IS_TMC(E7)
        tmc_parse_drv_status(stepperE7, n);
      #endif
    }

    SERIAL_EOL();
  }

  /**
   * M122 report functions
   */

  void tmc_report_all(LOGICAL_AXIS_ARGS_LC(const bool)) {
    #define TMC_REPORT(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL); tmc_debug_loop(ITEM OPTARGS_LOGICAL()); }while(0)
    #define DRV_REPORT(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL); drv_status_loop(ITEM OPTARGS_LOGICAL()); }while(0)

    TMC_REPORT("\t",                 TMC_CODES);
    #if HAS_DRIVER(TMC2209)
      TMC_REPORT("Address\t",        TMC_UART_ADDR);
    #endif
    TMC_REPORT("Enabled\t",          TMC_ENABLED);
    TMC_REPORT("Set current",        TMC_CURRENT);
    TMC_REPORT("RMS current",        TMC_RMS_CURRENT);
    TMC_REPORT("MAX current",        TMC_MAX_CURRENT);
    TMC_REPORT("Run current",        TMC_IRUN);
    TMC_REPORT("Hold current",       TMC_IHOLD);
    #if HAS_DRIVER(TMC2160) || HAS_DRIVER(TMC5160)
      TMC_REPORT("Global scaler",    TMC_GLOBAL_SCALER);
    #endif
    TMC_REPORT("CS actual",          TMC_CS_ACTUAL);
    TMC_REPORT("PWM scale",          TMC_PWM_SCALE);
    #if HAS_DRIVER(TMC2130) || HAS_DRIVER(TMC2224) || HAS_DRIVER(TMC2660) || HAS_TMC220x
      TMC_REPORT("vsense\t",         TMC_VSENSE);
    #endif
    TMC_REPORT("stealthChop",        TMC_STEALTHCHOP);
    TMC_REPORT("msteps\t",           TMC_MICROSTEPS);
    TMC_REPORT("interp\t",           TMC_INTERPOLATE);
    TMC_REPORT("tstep\t",            TMC_TSTEP);
    TMC_REPORT("PWM thresh.",        TMC_TPWMTHRS);
    TMC_REPORT("[mm/s]\t",           TMC_TPWMTHRS_MMS);
    TMC_REPORT("OT prewarn",         TMC_OTPW);
    #if ENABLED(MONITOR_DRIVER_STATUS)
      TMC_REPORT("triggered\n OTP\t", TMC_OTPW_TRIGGERED);
    #endif

    #if HAS_TMC220x
      TMC_REPORT("pwm scale sum",     TMC_PWM_SCALE_SUM);
      TMC_REPORT("pwm scale auto",    TMC_PWM_SCALE_AUTO);
      TMC_REPORT("pwm offset auto",   TMC_PWM_OFS_AUTO);
      TMC_REPORT("pwm grad auto",     TMC_PWM_GRAD_AUTO);
    #endif

    TMC_REPORT("off time",           TMC_TOFF);
    TMC_REPORT("blank time",         TMC_TBL);
    TMC_REPORT("hysteresis\n -end\t", TMC_HEND);
    TMC_REPORT(" -start\t",          TMC_HSTRT);
    TMC_REPORT("Stallguard thrs",    TMC_SGT);
    TMC_REPORT("uStep count",        TMC_MSCNT);
    DRV_REPORT("DRVSTATUS",          TMC_DRV_CODES);
    #if HAS_TMCX1X0 || HAS_TMC220x
      DRV_REPORT("sg_result",        TMC_SG_RESULT);
    #endif
    #if HAS_TMCX1X0
      DRV_REPORT("stallguard",       TMC_STALLGUARD);
      DRV_REPORT("fsactive",         TMC_FSACTIVE);
    #endif
    DRV_REPORT("stst\t",             TMC_STST);
    DRV_REPORT("olb\t",              TMC_OLB);
    DRV_REPORT("ola\t",              TMC_OLA);
    DRV_REPORT("s2gb\t",             TMC_S2GB);
    DRV_REPORT("s2ga\t",             TMC_S2GA);
    DRV_REPORT("otpw\t",             TMC_DRV_OTPW);
    DRV_REPORT("ot\t",               TMC_OT);
    #if HAS_TMC220x
      DRV_REPORT("157C\t",           TMC_T157);
      DRV_REPORT("150C\t",           TMC_T150);
      DRV_REPORT("143C\t",           TMC_T143);
      DRV_REPORT("120C\t",           TMC_T120);
      DRV_REPORT("s2vsa\t",          TMC_S2VSA);
      DRV_REPORT("s2vsb\t",          TMC_S2VSB);
    #endif
    DRV_REPORT("Driver registers:\n",TMC_DRV_STATUS_HEX);
    SERIAL_EOL();
  }

  #define PRINT_TMC_REGISTER(REG_CASE) case TMC_GET_##REG_CASE: print_hex_long(st.REG_CASE(), ':'); break

  #if HAS_TMCX1X0
    static void tmc_get_ic_registers(TMC2130Stepper &st, const TMC_get_registers_enum i) {
      switch (i) {
        PRINT_TMC_REGISTER(TCOOLTHRS);
        PRINT_TMC_REGISTER(THIGH);
        PRINT_TMC_REGISTER(COOLCONF);
        default: SERIAL_CHAR('\t'); break;
      }
    }
  #endif
  #if HAS_TMC220x
    static void tmc_get_ic_registers(TMC2208Stepper, const TMC_get_registers_enum) { SERIAL_CHAR('\t'); }
  #endif

  #if HAS_TRINAMIC_CONFIG
    template<class TMC>
    static void tmc_get_registers(TMC &st, const TMC_get_registers_enum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHAR('\t'); st.printLabel(); break;
        PRINT_TMC_REGISTER(GCONF);
        PRINT_TMC_REGISTER(IHOLD_IRUN);
        PRINT_TMC_REGISTER(GSTAT);
        PRINT_TMC_REGISTER(IOIN);
        PRINT_TMC_REGISTER(TPOWERDOWN);
        PRINT_TMC_REGISTER(TSTEP);
        PRINT_TMC_REGISTER(TPWMTHRS);
        PRINT_TMC_REGISTER(CHOPCONF);
        PRINT_TMC_REGISTER(PWMCONF);
        PRINT_TMC_REGISTER(PWM_SCALE);
        PRINT_TMC_REGISTER(DRV_STATUS);
        default: tmc_get_ic_registers(st, i); break;
      }
      SERIAL_CHAR('\t');
    }
  #endif
  #if HAS_DRIVER(TMC2660)
    template <char AXIS_LETTER, char DRIVER_ID, AxisEnum AXIS_ID>
    static void tmc_get_registers(TMCMarlin<TMC2660Stepper, AXIS_LETTER, DRIVER_ID, AXIS_ID> &st, const TMC_get_registers_enum i) {
      switch (i) {
        case TMC_AXIS_CODES: SERIAL_CHAR('\t'); st.printLabel(); break;
        PRINT_TMC_REGISTER(DRVCONF);
        PRINT_TMC_REGISTER(DRVCTRL);
        PRINT_TMC_REGISTER(CHOPCONF);
        PRINT_TMC_REGISTER(DRVSTATUS);
        PRINT_TMC_REGISTER(SGCSCONF);
        PRINT_TMC_REGISTER(SMARTEN);
        default: SERIAL_CHAR('\t'); break;
      }
      SERIAL_CHAR('\t');
    }
  #endif

  static void tmc_get_registers(TMC_get_registers_enum n OPTARGS_LOGICAL(const bool)) {
    if (TERN0(HAS_X_AXIS, x)) {
      #if AXIS_IS_TMC(X)
        tmc_get_registers(stepperX, n);
      #endif
      #if AXIS_IS_TMC(X2)
        tmc_get_registers(stepperX2, n);
      #endif
    }

    if (TERN0(HAS_Y_AXIS, y)) {
      #if AXIS_IS_TMC(Y)
        tmc_get_registers(stepperY, n);
      #endif
      #if AXIS_IS_TMC(Y2)
        tmc_get_registers(stepperY2, n);
      #endif
    }

    if (TERN0(HAS_Z_AXIS, z)) {
      #if AXIS_IS_TMC(Z)
        tmc_get_registers(stepperZ, n);
      #endif
      #if AXIS_IS_TMC(Z2)
        tmc_get_registers(stepperZ2, n);
      #endif
      #if AXIS_IS_TMC(Z3)
        tmc_get_registers(stepperZ3, n);
      #endif
      #if AXIS_IS_TMC(Z4)
        tmc_get_registers(stepperZ4, n);
      #endif
    }

    #if AXIS_IS_TMC(I)
      if (i) tmc_get_registers(stepperI, n);
    #endif
    #if AXIS_IS_TMC(J)
      if (j) tmc_get_registers(stepperJ, n);
    #endif
    #if AXIS_IS_TMC(K)
      if (k) tmc_get_registers(stepperK, n);
    #endif
    #if AXIS_IS_TMC(U)
      if (u) tmc_get_registers(stepperU, n);
    #endif
    #if AXIS_IS_TMC(V)
      if (v) tmc_get_registers(stepperV, n);
    #endif
    #if AXIS_IS_TMC(W)
      if (w) tmc_get_registers(stepperW, n);
    #endif

    if (TERN0(HAS_EXTRUDERS, e)) {
      #if AXIS_IS_TMC(E0)
        tmc_get_registers(stepperE0, n);
      #endif
      #if AXIS_IS_TMC(E1)
        tmc_get_registers(stepperE1, n);
      #endif
      #if AXIS_IS_TMC(E2)
        tmc_get_registers(stepperE2, n);
      #endif
      #if AXIS_IS_TMC(E3)
        tmc_get_registers(stepperE3, n);
      #endif
      #if AXIS_IS_TMC(E4)
        tmc_get_registers(stepperE4, n);
      #endif
      #if AXIS_IS_TMC(E5)
        tmc_get_registers(stepperE5, n);
      #endif
      #if AXIS_IS_TMC(E6)
        tmc_get_registers(stepperE6, n);
      #endif
      #if AXIS_IS_TMC(E7)
        tmc_get_registers(stepperE7, n);
      #endif
    }

    SERIAL_EOL();
  }

  void tmc_get_registers(LOGICAL_AXIS_ARGS_LC(bool)) {
    #define _TMC_GET_REG(LABEL, ITEM) do{ SERIAL_ECHOPGM(LABEL); tmc_get_registers(ITEM OPTARGS_LOGICAL()); }while(0)
    #define TMC_GET_REG(NAME, TABS) _TMC_GET_REG(STRINGIFY(NAME) TABS, TMC_GET_##NAME)
    _TMC_GET_REG("\t", TMC_AXIS_CODES);
    TMC_GET_REG(GCONF, "\t\t");
    TMC_GET_REG(IHOLD_IRUN, "\t");
    TMC_GET_REG(GSTAT, "\t\t");
    TMC_GET_REG(IOIN, "\t\t");
    TMC_GET_REG(TPOWERDOWN, "\t");
    TMC_GET_REG(TSTEP, "\t\t");
    TMC_GET_REG(TPWMTHRS, "\t");
    TMC_GET_REG(TCOOLTHRS, "\t");
    TMC_GET_REG(THIGH, "\t\t");
    TMC_GET_REG(CHOPCONF, "\t");
    TMC_GET_REG(COOLCONF, "\t");
    TMC_GET_REG(PWMCONF, "\t");
    TMC_GET_REG(PWM_SCALE, "\t");
    TMC_GET_REG(DRV_STATUS, "\t");
  }

#endif // TMC_DEBUG

#if USE_SENSORLESS

  bool tmc_enable_stallguard(TMC2130Stepper &st) {
    const bool stealthchop_was_enabled = st.en_pwm_mode();

    st.TCOOLTHRS(0xFFFFF);
    st.en_pwm_mode(false);
    st.diag1_stall(true);

    return stealthchop_was_enabled;
  }
  void tmc_disable_stallguard(TMC2130Stepper &st, const bool restore_stealth) {
    st.TCOOLTHRS(0);
    st.en_pwm_mode(restore_stealth);
    st.diag1_stall(false);
  }

  bool tmc_enable_stallguard(TMC2209Stepper &st) {
    const bool stealthchop_was_enabled = !st.en_spreadCycle();

    st.TCOOLTHRS(0xFFFFF);
    st.en_spreadCycle(false);
    return stealthchop_was_enabled;
  }
  void tmc_disable_stallguard(TMC2209Stepper &st, const bool restore_stealth) {
    st.en_spreadCycle(!restore_stealth);
    st.TCOOLTHRS(0);
  }

  bool tmc_enable_stallguard(TMC2660Stepper) {
    // TODO
    return false;
  }
  void tmc_disable_stallguard(TMC2660Stepper, const bool) {};

#endif // USE_SENSORLESS

template<typename TMC>
static bool test_connection(TMC &st) {
  SERIAL_ECHOPGM("Testing ");
  st.printLabel();
  SERIAL_ECHOPGM(" connection... ");
  const uint8_t test_result = st.test_connection();

  if (test_result > 0) SERIAL_ECHOPGM("Error: All ");

  FSTR_P stat;
  switch (test_result) {
    default:
    case 0: stat = F("OK"); break;
    case 1: stat = F("HIGH"); break;
    case 2: stat = F("LOW"); break;
  }
  SERIAL_ECHOLN(stat);

  return test_result;
}

void test_tmc_connection(LOGICAL_AXIS_ARGS_LC(const bool)) {
  uint8_t axis_connection = 0;

  if (TERN0(HAS_X_AXIS, x)) {
    #if AXIS_IS_TMC(X)
      axis_connection += test_connection(stepperX);
    #endif
    #if AXIS_IS_TMC(X2)
      axis_connection += test_connection(stepperX2);
    #endif
  }

  if (TERN0(HAS_Y_AXIS, y)) {
    #if AXIS_IS_TMC(Y)
      axis_connection += test_connection(stepperY);
    #endif
    #if AXIS_IS_TMC(Y2)
      axis_connection += test_connection(stepperY2);
    #endif
  }

  if (TERN0(HAS_Z_AXIS, z)) {
    #if AXIS_IS_TMC(Z)
      axis_connection += test_connection(stepperZ);
    #endif
    #if AXIS_IS_TMC(Z2)
      axis_connection += test_connection(stepperZ2);
    #endif
    #if AXIS_IS_TMC(Z3)
      axis_connection += test_connection(stepperZ3);
    #endif
    #if AXIS_IS_TMC(Z4)
      axis_connection += test_connection(stepperZ4);
    #endif
  }

  #if AXIS_IS_TMC(I)
    if (i) axis_connection += test_connection(stepperI);
  #endif
  #if AXIS_IS_TMC(J)
    if (j) axis_connection += test_connection(stepperJ);
  #endif
  #if AXIS_IS_TMC(K)
    if (k) axis_connection += test_connection(stepperK);
  #endif
  #if AXIS_IS_TMC(U)
    if (u) axis_connection += test_connection(stepperU);
  #endif
  #if AXIS_IS_TMC(V)
    if (v) axis_connection += test_connection(stepperV);
  #endif
  #if AXIS_IS_TMC(W)
    if (w) axis_connection += test_connection(stepperW);
  #endif

  if (TERN0(HAS_EXTRUDERS, e)) {
    #if AXIS_IS_TMC(E0)
      axis_connection += test_connection(stepperE0);
    #endif
    #if AXIS_IS_TMC(E1)
      axis_connection += test_connection(stepperE1);
    #endif
    #if AXIS_IS_TMC(E2)
      axis_connection += test_connection(stepperE2);
    #endif
    #if AXIS_IS_TMC(E3)
      axis_connection += test_connection(stepperE3);
    #endif
    #if AXIS_IS_TMC(E4)
      axis_connection += test_connection(stepperE4);
    #endif
    #if AXIS_IS_TMC(E5)
      axis_connection += test_connection(stepperE5);
    #endif
    #if AXIS_IS_TMC(E6)
      axis_connection += test_connection(stepperE6);
    #endif
    #if AXIS_IS_TMC(E7)
      axis_connection += test_connection(stepperE7);
    #endif
  }

  if (axis_connection) LCD_MESSAGE(MSG_ERROR_TMC);
}

#endif // HAS_TRINAMIC_CONFIG

#if HAS_TMC_SPI
  #define SET_CS_PIN(st) OUT_WRITE(st##_CS_PIN, HIGH)
  void tmc_init_cs_pins() {
    #if AXIS_HAS_SPI(X)
      SET_CS_PIN(X);
    #endif
    #if AXIS_HAS_SPI(Y)
      SET_CS_PIN(Y);
    #endif
    #if AXIS_HAS_SPI(Z)
      SET_CS_PIN(Z);
    #endif
    #if AXIS_HAS_SPI(X2)
      SET_CS_PIN(X2);
    #endif
    #if AXIS_HAS_SPI(Y2)
      SET_CS_PIN(Y2);
    #endif
    #if AXIS_HAS_SPI(Z2)
      SET_CS_PIN(Z2);
    #endif
    #if AXIS_HAS_SPI(Z3)
      SET_CS_PIN(Z3);
    #endif
    #if AXIS_HAS_SPI(Z4)
      SET_CS_PIN(Z4);
    #endif
    #if AXIS_HAS_SPI(I)
      SET_CS_PIN(I);
    #endif
    #if AXIS_HAS_SPI(J)
      SET_CS_PIN(J);
    #endif
    #if AXIS_HAS_SPI(K)
      SET_CS_PIN(K);
    #endif
    #if AXIS_HAS_SPI(U)
      SET_CS_PIN(U);
    #endif
    #if AXIS_HAS_SPI(V)
      SET_CS_PIN(V);
    #endif
    #if AXIS_HAS_SPI(W)
      SET_CS_PIN(W);
    #endif
    #if AXIS_HAS_SPI(E0)
      SET_CS_PIN(E0);
    #endif
    #if AXIS_HAS_SPI(E1)
      SET_CS_PIN(E1);
    #endif
    #if AXIS_HAS_SPI(E2)
      SET_CS_PIN(E2);
    #endif
    #if AXIS_HAS_SPI(E3)
      SET_CS_PIN(E3);
    #endif
    #if AXIS_HAS_SPI(E4)
      SET_CS_PIN(E4);
    #endif
    #if AXIS_HAS_SPI(E5)
      SET_CS_PIN(E5);
    #endif
    #if AXIS_HAS_SPI(E6)
      SET_CS_PIN(E6);
    #endif
    #if AXIS_HAS_SPI(E7)
      SET_CS_PIN(E7);
    #endif
  }
#endif // HAS_TMC_SPI
