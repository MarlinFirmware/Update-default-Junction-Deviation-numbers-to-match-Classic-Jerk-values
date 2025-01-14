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
 * temperature.h - temperature controller
 */

#include "thermistor/thermistors.h"

#include "../inc/MarlinConfig.h"

#if ENABLED(AUTO_POWER_CONTROL)
  #include "../feature/power.h"
#endif

#if ENABLED(AUTO_REPORT_TEMPERATURES)
  #include "../libs/autoreport.h"
#endif

#if HAS_FANCHECK
  #include "../feature/fancheck.h"
#endif

//#define ERR_INCLUDE_TEMP

#define HOTEND_INDEX TERN0(HAS_MULTI_HOTEND, e)
#define E_NAME TERN_(HAS_MULTI_HOTEND, e)

#if HAS_FAN
  #if NUM_REDUNDANT_FANS
    #define FAN_IS_REDUNDANT(Q) WITHIN(Q, REDUNDANT_PART_COOLING_FAN, REDUNDANT_PART_COOLING_FAN + NUM_REDUNDANT_FANS - 1)
  #else
    #define FAN_IS_REDUNDANT(Q) false
  #endif
  #define FAN_IS_M106ABLE(Q) (HAS_FAN##Q && !FAN_IS_REDUNDANT(Q))
#else
  #define FAN_IS_M106ABLE(Q) false
#endif

typedef int_fast8_t heater_id_t;

/**
 * States for ADC reading in the ISR
 */
enum ADCSensorState : char {
  StartSampling,
  #if HAS_TEMP_ADC_0
    PrepareTemp_0, MeasureTemp_0,
  #endif
  #if HAS_TEMP_ADC_BED
    PrepareTemp_BED, MeasureTemp_BED,
  #endif
  #if HAS_TEMP_ADC_CHAMBER
    PrepareTemp_CHAMBER, MeasureTemp_CHAMBER,
  #endif
  #if HAS_TEMP_ADC_COOLER
    PrepareTemp_COOLER, MeasureTemp_COOLER,
  #endif
  #if HAS_TEMP_ADC_PROBE
    PrepareTemp_PROBE, MeasureTemp_PROBE,
  #endif
  #if HAS_TEMP_ADC_BOARD
    PrepareTemp_BOARD, MeasureTemp_BOARD,
  #endif
  #if HAS_TEMP_ADC_SOC
    PrepareTemp_SOC, MeasureTemp_SOC,
  #endif
  #if HAS_TEMP_ADC_REDUNDANT
    PrepareTemp_REDUNDANT, MeasureTemp_REDUNDANT,
  #endif
  #if HAS_TEMP_ADC_1
    PrepareTemp_1, MeasureTemp_1,
  #endif
  #if HAS_TEMP_ADC_2
    PrepareTemp_2, MeasureTemp_2,
  #endif
  #if HAS_TEMP_ADC_3
    PrepareTemp_3, MeasureTemp_3,
  #endif
  #if HAS_TEMP_ADC_4
    PrepareTemp_4, MeasureTemp_4,
  #endif
  #if HAS_TEMP_ADC_5
    PrepareTemp_5, MeasureTemp_5,
  #endif
  #if HAS_TEMP_ADC_6
    PrepareTemp_6, MeasureTemp_6,
  #endif
  #if HAS_TEMP_ADC_7
    PrepareTemp_7, MeasureTemp_7,
  #endif
  #if HAS_JOY_ADC_X
    PrepareJoy_X, MeasureJoy_X,
  #endif
  #if HAS_JOY_ADC_Y
    PrepareJoy_Y, MeasureJoy_Y,
  #endif
  #if HAS_JOY_ADC_Z
    PrepareJoy_Z, MeasureJoy_Z,
  #endif
  #if ENABLED(FILAMENT_WIDTH_SENSOR)
    Prepare_FILWIDTH, Measure_FILWIDTH,
  #endif
  #if ENABLED(POWER_MONITOR_CURRENT)
    Prepare_POWER_MONITOR_CURRENT,
    Measure_POWER_MONITOR_CURRENT,
  #endif
  #if ENABLED(POWER_MONITOR_VOLTAGE)
    Prepare_POWER_MONITOR_VOLTAGE,
    Measure_POWER_MONITOR_VOLTAGE,
  #endif
  #if HAS_ADC_BUTTONS
    Prepare_ADC_KEY, Measure_ADC_KEY,
  #endif
  SensorsReady, // Temperatures ready. Delay the next round of readings to let ADC pins settle.
  StartupDelay  // Startup, delay initial temp reading a tiny bit so the hardware can settle
};

// Minimum number of Temperature::ISR loops between sensor readings.
// Multiplied by 16 (OVERSAMPLENR) to obtain the total time to
// get all oversampled sensor readings
#define MIN_ADC_ISR_LOOPS 10

#define ACTUAL_ADC_SAMPLES _MAX(int(MIN_ADC_ISR_LOOPS), int(SensorsReady))

//
// PID
//

typedef struct { float p, i, d; } raw_pid_t;
typedef struct { float p, i, d, c, f; } raw_pidcf_t;

#if HAS_PID_HEATING

  #define PID_K2 (1.0f - float(PID_K1))
  #define PID_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (TEMP_TIMER_FREQUENCY))

  // Apply the scale factors to the PID values
  #define scalePID_i(i)   ( float(i) * PID_dT )
  #define unscalePID_i(i) ( float(i) / PID_dT )
  #define scalePID_d(d)   ( float(d) / PID_dT )
  #define unscalePID_d(d) ( float(d) * PID_dT )

  /**
   * @brief The default PID class, only has Kp, Ki, Kd, other classes extend this one
   * @tparam MIN_POW output when current is above target by functional_range
   * @tparam MAX_POW output when current is below target by functional_range
   * @details This class has methods for Kc and Kf terms, but returns constant default values.
   *          PID classes that implement these features are expected to override these methods.
   *          Since the eventual PID class is typedef-d, there is no need to use virtual functions.
   */
  template<int MIN_POW, int MAX_POW>
  struct PID_t {
    protected:
      bool pid_reset = true;
      float temp_iState = 0.0f, temp_dState = 0.0f;
      float work_p = 0, work_i = 0, work_d = 0;

    public:
      float Kp = 0, Ki = 0, Kd = 0;
      float p() const { return Kp; }
      float i() const { return unscalePID_i(Ki); }
      float d() const { return unscalePID_d(Kd); }
      float c() const { return 1; }
      float f() const { return 0; }
      float pTerm() const { return work_p; }
      float iTerm() const { return work_i; }
      float dTerm() const { return work_d; }
      float cTerm() const { return 0; }
      float fTerm() const { return 0; }
      void set_Kp(float p) { Kp = p; }
      void set_Ki(float i) { Ki = scalePID_i(i); }
      void set_Kd(float d) { Kd = scalePID_d(d); }
      void set_Kc(float) {}
      void set_Kf(float) {}
      int low() const { return MIN_POW; }
      int high() const { return MAX_POW; }
      void reset() { pid_reset = true; }
      void set(float p, float i, float d, float c=1, float f=0) { set_Kp(p); set_Ki(i); set_Kd(d); set_Kc(c); set_Kf(f); }
      void set(const raw_pid_t &raw) { set(raw.p, raw.i, raw.d); }
      void set(const raw_pidcf_t &raw) { set(raw.p, raw.i, raw.d, raw.c, raw.f); }

      float get_fan_scale_output(const uint8_t) { return 0; }

      float get_extrusion_scale_output(const bool, const int32_t, const float, const int16_t) { return 0; }

      float get_pid_output(const float target, const float current) {
        const float pid_error = target - current;
        float output_pow;
        if (!target || pid_error < -(PID_FUNCTIONAL_RANGE)) {
          pid_reset = true;
          output_pow = 0;
        }
        else if (pid_error > PID_FUNCTIONAL_RANGE) {
          pid_reset = true;
          output_pow = MAX_POW;
        }
        else {
          if (pid_reset) {
            pid_reset = false;
            temp_iState = 0.0;
            work_d = 0.0;
          }

          const float max_power_over_i_gain = float(MAX_POW) / Ki - float(MIN_POW);
          temp_iState = constrain(temp_iState + pid_error, 0, max_power_over_i_gain);

          work_p = Kp * pid_error;
          work_i = Ki * temp_iState;
          work_d = work_d + PID_K2 * (Kd * (temp_dState - current) - work_d);

          output_pow = constrain(work_p + work_i + work_d + float(MIN_POW), 0, MAX_POW);
        }

        temp_dState = current;

        return output_pow;
      }

  };

#endif // HAS_PID_HEATING

#if ENABLED(PIDTEMP)

  // @brief Extrusion scaled PID class
  template<int MIN_POW, int MAX_POW, int LPQ_ARR_SZ>
  struct PIDC_t : public PID_t<MIN_POW, MAX_POW> {
    private:
      using base = PID_t<MIN_POW, MAX_POW>;
      float work_c = 0;
      float prev_e_pos = 0;
      int32_t lpq[LPQ_ARR_SZ] = {};
      int16_t lpq_ptr = 0;
    public:
      float Kc = 0;
      float c() const { return Kc; }
      void set_Kc(float c) { Kc = c; }
      float cTerm() const { return work_c; }
      void set(float p, float i, float d, float c=1, float f=0) {
        base::set_Kp(p);
        base::set_Ki(i);
        base::set_Kd(d);
        set_Kc(c);
        base::set_Kf(f);
      }
      void set(const raw_pid_t &raw) { set(raw.p, raw.i, raw.d); }
      void set(const raw_pidcf_t &raw) { set(raw.p, raw.i, raw.d, raw.c, raw.f); }
      void reset() {
        base::reset();
        prev_e_pos = 0;
        lpq_ptr = 0;
        for (uint8_t i = 0; i < LPQ_ARR_SZ; ++i) lpq[i] = 0;
      }

      float get_extrusion_scale_output(const bool is_active, const int32_t e_position, const float e_mm_per_step, const int16_t lpq_len) {
        work_c = 0;
        if (!is_active) return work_c;

        if (e_position > prev_e_pos) {
          lpq[lpq_ptr] = e_position - prev_e_pos;
          prev_e_pos = e_position;
        }
        else
          lpq[lpq_ptr] = 0;

        ++lpq_ptr;

        if (lpq_ptr >= LPQ_ARR_SZ || lpq_ptr >= lpq_len)
          lpq_ptr = 0;

        work_c = (lpq[lpq_ptr] * e_mm_per_step) * Kc;

        return work_c;
      }
  };

  // @brief Fan scaled PID, this class implements the get_fan_scale_output() method
  // @tparam MIN_POW @see PID_t
  // @tparam MAX_POW @see PID_t
  // @tparam SCALE_MIN_SPEED parameter from Configuration_adv.h
  // @tparam SCALE_LIN_FACTOR parameter from Configuration_adv.h
  template<int MIN_POW, int MAX_POW, int SCALE_MIN_SPEED, int SCALE_LIN_FACTOR>
  struct PIDF_t : public PID_t<MIN_POW, MAX_POW> {
    private:
      using base = PID_t<MIN_POW, MAX_POW>;
      float work_f = 0;
    public:
      float Kf = 0;
      float f() const { return Kf; }
      void set_Kf(float f) { Kf = f; }
      float fTerm() const { return work_f; }
      void set(float p, float i, float d, float c=1, float f=0) {
        base::set_Kp(p);
        base::set_Ki(i);
        base::set_Kd(d);
        base::set_Kc(c);
        set_Kf(f);
      }
      void set(const raw_pid_t &raw) { set(raw.p, raw.i, raw.d); }
      void set(const raw_pidcf_t &raw) { set(raw.p, raw.i, raw.d, raw.c, raw.f); }

      float get_fan_scale_output(const uint8_t fan_speed) {
        work_f = 0;
        if (fan_speed > SCALE_MIN_SPEED)
          work_f = Kf + (SCALE_LIN_FACTOR) * fan_speed;

        return work_f;
      }
  };

  // @brief Inherits PID and PIDC - can't use proper diamond inheritance w/o virtual
  template<int MIN_POW, int MAX_POW, int LPQ_ARR_SZ, int SCALE_MIN_SPEED, int SCALE_LIN_FACTOR>
  struct PIDCF_t : public PIDC_t<MIN_POW, MAX_POW, LPQ_ARR_SZ> {
    private:
      using base = PID_t<MIN_POW, MAX_POW>;
      using cPID = PIDC_t<MIN_POW, MAX_POW, LPQ_ARR_SZ>;
      float work_f = 0;
    public:
      float Kf = 0;
      float c() const { return cPID::c(); }
      float f() const { return Kf; }
      void set_Kc(float c) { cPID::set_Kc(c); }
      void set_Kf(float f) { Kf = f; }
      float cTerm() const { return cPID::cTerm(); }
      float fTerm() const { return work_f; }
      void set(float p, float i, float d, float c=1, float f=0) {
        base::set_Kp(p);
        base::set_Ki(i);
        base::set_Kd(d);
        cPID::set_Kc(c);
        set_Kf(f);
      }
      void set(const raw_pid_t &raw) { set(raw.p, raw.i, raw.d); }
      void set(const raw_pidcf_t &raw) { set(raw.p, raw.i, raw.d, raw.c, raw.f); }

      void reset() { cPID::reset(); }

      float get_fan_scale_output(const uint8_t fan_speed) {
        work_f = fan_speed > (SCALE_MIN_SPEED) ? Kf + (SCALE_LIN_FACTOR) * fan_speed : 0;
        return work_f;
      }
      float get_extrusion_scale_output(const bool is_active, const int32_t e_position, const float e_mm_per_step, const int16_t lpq_len) {
        return cPID::get_extrusion_scale_output(is_active, e_position, e_mm_per_step, lpq_len);
      }
  };

  typedef
    #if ALL(PID_EXTRUSION_SCALING, PID_FAN_SCALING)
      PIDCF_t<0, PID_MAX, LPQ_MAX_LEN, PID_FAN_SCALING_MIN_SPEED, PID_FAN_SCALING_LIN_FACTOR>
    #elif ENABLED(PID_EXTRUSION_SCALING)
      PIDC_t<0, PID_MAX, LPQ_MAX_LEN>
    #elif ENABLED(PID_FAN_SCALING)
      PIDF_t<0, PID_MAX, PID_FAN_SCALING_MIN_SPEED, PID_FAN_SCALING_LIN_FACTOR>
    #else
      PID_t<0, PID_MAX>
    #endif
  hotend_pid_t;

  #if ENABLED(PID_PARAMS_PER_HOTEND)
    #define SET_HOTEND_PID(F,H,V) thermalManager.temp_hotend[H].pid.set_##F(V)
  #else
    #define SET_HOTEND_PID(F,_,V) do{ HOTEND_LOOP() thermalManager.temp_hotend[e].pid.set_##F(V); }while(0)
  #endif

#elif ENABLED(MPCTEMP)

  typedef struct MPC {
    static bool e_paused;               // Pause E filament permm tracking
    static int32_t e_position;          // For E tracking
    float heater_power;                 // M306 P
    float block_heat_capacity;          // M306 C
    float sensor_responsiveness;        // M306 R
    float ambient_xfer_coeff_fan0;      // M306 A
    float filament_heat_capacity_permm; // M306 H
    #if ENABLED(MPC_INCLUDE_FAN)
      float fan255_adjustment;          // M306 F
      void applyFanAdjustment(const_float_t cf) { fan255_adjustment = cf - ambient_xfer_coeff_fan0; }
    #else
      void applyFanAdjustment(const_float_t) {}
    #endif
    float fanCoefficient() { return SUM_TERN(MPC_INCLUDE_FAN, ambient_xfer_coeff_fan0, fan255_adjustment); }
  } MPC_t;

  #define MPC_dT ((OVERSAMPLENR * float(ACTUAL_ADC_SAMPLES)) / (TEMP_TIMER_FREQUENCY))

#endif

#if ENABLED(G26_MESH_VALIDATION) && ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI)
  #define G26_CLICK_CAN_CANCEL 1
#endif

// A temperature sensor
typedef struct TempInfo {
  private:
    raw_adc_t acc;
    raw_adc_t raw;
  public:
    celsius_float_t celsius;
    inline void reset() { acc = 0; }
    inline void sample(const raw_adc_t s) { acc += s; }
    inline void update() { raw = acc; }
    void setraw(const raw_adc_t r) { raw = r; }
    raw_adc_t getraw() const { return raw; }
} temp_info_t;

#if HAS_TEMP_REDUNDANT
  // A redundant temperature sensor
  typedef struct RedundantTempInfo : public TempInfo {
    temp_info_t* target;
  } redundant_info_t;
#endif

// A PWM heater with temperature sensor
typedef struct HeaterInfo : public TempInfo {
  celsius_t target;
  uint8_t soft_pwm_amount;
  bool is_below_target(const celsius_t offs=0) const { return (target - celsius > offs); } // celsius < target - offs
  bool is_above_target(const celsius_t offs=0) const { return (celsius - target > offs); } // celsius > target + offs
  #if ENABLED(PELTIER_BED)
    bool peltier_dir_heating; // = false
  #endif
} heater_info_t;

// A heater with PID stabilization
template<typename T>
struct PIDHeaterInfo : public HeaterInfo {
  T pid;  // Initialized by settings.load
};

#if ENABLED(MPCTEMP)
  struct MPCHeaterInfo : public HeaterInfo {
    MPC_t mpc;
    float modeled_ambient_temp,
          modeled_block_temp,
          modeled_sensor_temp;
    float fanCoefficient() { return mpc.fanCoefficient(); }
    void applyFanAdjustment(const_float_t cf) { mpc.applyFanAdjustment(cf); }
  };
#endif

#if ENABLED(PIDTEMP)
  typedef struct PIDHeaterInfo<hotend_pid_t> hotend_info_t;
#elif ENABLED(MPCTEMP)
  typedef struct MPCHeaterInfo hotend_info_t;
#else
  typedef heater_info_t hotend_info_t;
#endif
#if HAS_HEATED_BED
  #if ENABLED(PIDTEMPBED)
    typedef struct PIDHeaterInfo<PID_t<MIN_BED_POWER, MAX_BED_POWER>> bed_info_t;
  #else
    typedef heater_info_t bed_info_t;
  #endif
#endif
#if HAS_HEATED_CHAMBER
  #if ENABLED(PIDTEMPCHAMBER)
    typedef struct PIDHeaterInfo<PID_t<MIN_CHAMBER_POWER, MAX_CHAMBER_POWER>> chamber_info_t;
  #else
    typedef heater_info_t chamber_info_t;
  #endif
#elif HAS_TEMP_CHAMBER
  typedef temp_info_t chamber_info_t;
#endif
#if HAS_TEMP_PROBE
  typedef temp_info_t probe_info_t;
#endif
#if ANY(HAS_COOLER, HAS_TEMP_COOLER)
  typedef heater_info_t cooler_info_t;
#endif
#if HAS_TEMP_BOARD
  typedef temp_info_t board_info_t;
#endif
#if HAS_TEMP_SOC
  typedef temp_info_t soc_info_t;
#endif

// Heater watch handling
template <int INCREASE, int HYSTERESIS, millis_t PERIOD>
struct HeaterWatch {
  celsius_t target;
  millis_t next_ms;
  inline bool elapsed(const millis_t &ms) { return next_ms && ELAPSED(ms, next_ms); }
  inline bool elapsed() { return elapsed(millis()); }

  inline bool check(const celsius_t curr) { return curr >= target; }

  inline void restart(const celsius_t curr, const celsius_t tgt) {
    if (tgt) {
      const celsius_t newtarget = curr + INCREASE;
      if (newtarget < tgt - HYSTERESIS - 1) {
        target = newtarget;
        next_ms = millis() + SEC_TO_MS(PERIOD);
        return;
      }
    }
    next_ms = 0;
  }
};

#if WATCH_HOTENDS
  typedef struct HeaterWatch<WATCH_TEMP_INCREASE, TEMP_HYSTERESIS, WATCH_TEMP_PERIOD> hotend_watch_t;
#endif
#if WATCH_BED
  typedef struct HeaterWatch<WATCH_BED_TEMP_INCREASE, TEMP_BED_HYSTERESIS, WATCH_BED_TEMP_PERIOD> bed_watch_t;
#endif
#if WATCH_CHAMBER
  typedef struct HeaterWatch<WATCH_CHAMBER_TEMP_INCREASE, TEMP_CHAMBER_HYSTERESIS, WATCH_CHAMBER_TEMP_PERIOD> chamber_watch_t;
#endif
#if WATCH_COOLER
  typedef struct HeaterWatch<WATCH_COOLER_TEMP_INCREASE, TEMP_COOLER_HYSTERESIS, WATCH_COOLER_TEMP_PERIOD> cooler_watch_t;
#endif

// Just raw temperature sensor ranges
typedef struct { raw_adc_t raw_min, raw_max; } temp_raw_range_t;

// Temperature sensor read value ranges
typedef struct { raw_adc_t raw_min, raw_max; celsius_t mintemp, maxtemp; } temp_range_t;

#define THERMISTOR_ABS_ZERO_C           -273.15f  // bbbbrrrrr cold !
#define THERMISTOR_RESISTANCE_NOMINAL_C 25.0f     // mmmmm comfortable

#if HAS_USER_THERMISTORS

  enum CustomThermistorIndex : uint8_t {
    #if TEMP_SENSOR_0_IS_CUSTOM
      CTI_HOTEND_0,
    #endif
    #if TEMP_SENSOR_1_IS_CUSTOM
      CTI_HOTEND_1,
    #endif
    #if TEMP_SENSOR_2_IS_CUSTOM
      CTI_HOTEND_2,
    #endif
    #if TEMP_SENSOR_3_IS_CUSTOM
      CTI_HOTEND_3,
    #endif
    #if TEMP_SENSOR_4_IS_CUSTOM
      CTI_HOTEND_4,
    #endif
    #if TEMP_SENSOR_5_IS_CUSTOM
      CTI_HOTEND_5,
    #endif
    #if TEMP_SENSOR_BED_IS_CUSTOM
      CTI_BED,
    #endif
    #if TEMP_SENSOR_CHAMBER_IS_CUSTOM
      CTI_CHAMBER,
    #endif
    #if TEMP_SENSOR_PROBE_IS_CUSTOM
      CTI_PROBE,
    #endif
    #if TEMP_SENSOR_COOLER_IS_CUSTOM
      CTI_COOLER,
    #endif
    #if TEMP_SENSOR_BOARD_IS_CUSTOM
      CTI_BOARD,
    #endif
    #if TEMP_SENSOR_REDUNDANT_IS_CUSTOM
      CTI_REDUNDANT,
    #endif
    USER_THERMISTORS
  };

  // User-defined thermistor
  typedef struct {
    bool pre_calc;     // true if pre-calculations update needed
    float sh_c_coeff,  // Steinhart-Hart C coefficient .. defaults to '0.0'
          sh_alpha,
          series_res,
          res_25, res_25_recip,
          res_25_log,
          beta, beta_recip;
  } user_thermistor_t;

#endif

#if HAS_AUTO_FAN || HAS_FANCHECK
  #define HAS_FAN_LOGIC 1
#endif

class Temperature {

  public:

    #if HAS_HOTEND
      static hotend_info_t temp_hotend[HOTENDS];
      static constexpr celsius_t hotend_maxtemp[HOTENDS] = ARRAY_BY_HOTENDS(HEATER_0_MAXTEMP, HEATER_1_MAXTEMP, HEATER_2_MAXTEMP, HEATER_3_MAXTEMP, HEATER_4_MAXTEMP, HEATER_5_MAXTEMP, HEATER_6_MAXTEMP, HEATER_7_MAXTEMP);
      static constexpr celsius_t hotend_max_target(const uint8_t e) { return hotend_maxtemp[e] - (HOTEND_OVERSHOOT); }
    #endif

    #if HAS_HEATED_BED
      static bed_info_t temp_bed;
    #endif
    #if HAS_TEMP_PROBE
      static probe_info_t temp_probe;
    #endif
    #if HAS_TEMP_CHAMBER
      static chamber_info_t temp_chamber;
    #endif
    #if HAS_TEMP_COOLER
      static cooler_info_t temp_cooler;
    #endif
    #if HAS_TEMP_BOARD
      static board_info_t temp_board;
    #endif
    #if HAS_TEMP_SOC
      static soc_info_t temp_soc;
    #endif
    #if HAS_TEMP_REDUNDANT
      static redundant_info_t temp_redundant;
    #endif

    #if ANY(AUTO_POWER_E_FANS, HAS_FANCHECK)
      static uint8_t autofan_speed[HOTENDS];
    #endif
    #if ENABLED(AUTO_POWER_CHAMBER_FAN)
      static uint8_t chamberfan_speed;
    #endif
    #if ENABLED(AUTO_POWER_COOLER_FAN)
      static uint8_t coolerfan_speed;
    #endif

    #if ENABLED(FAN_SOFT_PWM)
      static uint8_t soft_pwm_amount_fan[FAN_COUNT],
                     soft_pwm_count_fan[FAN_COUNT];
    #endif

    #if ALL(FAN_SOFT_PWM, USE_CONTROLLER_FAN)
      static uint8_t soft_pwm_controller_speed;
    #endif

    #if ALL(HAS_MARLINUI_MENU, PREVENT_COLD_EXTRUSION) && E_MANUAL > 0
      static bool allow_cold_extrude_override;
      static void set_menu_cold_override(const bool allow) { allow_cold_extrude_override = allow; }
    #else
      static constexpr bool allow_cold_extrude_override = false;
      static void set_menu_cold_override(const bool) {}
    #endif

    #if ENABLED(PREVENT_COLD_EXTRUSION)
      static bool allow_cold_extrude;
      static celsius_t extrude_min_temp;
      static bool tooCold(const celsius_t temp) { return !allow_cold_extrude && !allow_cold_extrude_override && temp < extrude_min_temp - (TEMP_WINDOW); }
      static bool tooColdToExtrude(const uint8_t E_NAME)       { return tooCold(wholeDegHotend(HOTEND_INDEX)); }
      static bool targetTooColdToExtrude(const uint8_t E_NAME) { return tooCold(degTargetHotend(HOTEND_INDEX)); }
    #else
      static constexpr bool allow_cold_extrude = true;
      static constexpr celsius_t extrude_min_temp = 0;
      static bool tooColdToExtrude(const uint8_t) { return false; }
      static bool targetTooColdToExtrude(const uint8_t) { return false; }
    #endif

    static bool hotEnoughToExtrude(const uint8_t e) { return !tooColdToExtrude(e); }
    static bool targetHotEnoughToExtrude(const uint8_t e) { return !targetTooColdToExtrude(e); }

    #if ANY(SINGLENOZZLE_STANDBY_TEMP, SINGLENOZZLE_STANDBY_FAN)
      #if ENABLED(SINGLENOZZLE_STANDBY_TEMP)
        static celsius_t singlenozzle_temp[EXTRUDERS];
      #endif
      #if ENABLED(SINGLENOZZLE_STANDBY_FAN)
        static uint8_t singlenozzle_fan_speed[EXTRUDERS];
      #endif
      static void singlenozzle_change(const uint8_t old_tool, const uint8_t new_tool);
    #endif

    #if HEATER_IDLE_HANDLER

      // Heater idle handling. Marlin creates one per hotend and one for the heated bed.
      typedef struct {
        millis_t timeout_ms;
        bool timed_out;
        inline void update(const millis_t &ms) { if (!timed_out && timeout_ms && ELAPSED(ms, timeout_ms)) timed_out = true; }
        inline void start(const millis_t &ms) { timeout_ms = millis() + ms; timed_out = false; }
        inline void reset() { timeout_ms = 0; timed_out = false; }
        inline void expire() { start(0); }
      } heater_idle_t;

      // Indices and size for the heater_idle array
      enum IdleIndex : int8_t {
        _II = -1

        #define _IDLE_INDEX_E(N) ,IDLE_INDEX_E##N
        REPEAT(HOTENDS, _IDLE_INDEX_E)
        #undef _IDLE_INDEX_E

        OPTARG(HAS_HEATED_BED, IDLE_INDEX_BED)

        , NR_HEATER_IDLE
      };

      // Convert the given heater_id_t to idle array index
      static IdleIndex idle_index_for_id(const int8_t heater_id) {
        OPTCODE(HAS_HEATED_BED, if (heater_id == H_BED) return IDLE_INDEX_BED)
        return (IdleIndex)_MAX(heater_id, 0);
      }

      static heater_idle_t heater_idle[NR_HEATER_IDLE];

    #endif // HEATER_IDLE_TIMER

    #if HAS_ADC_BUTTONS
      static uint32_t current_ADCKey_raw;
      static uint16_t ADCKey_count;
    #endif

    #if ENABLED(PID_EXTRUSION_SCALING)
      static int16_t lpq_len;
    #endif

    #if HAS_FAN_LOGIC
      static constexpr millis_t fan_update_interval_ms = TERN(HAS_PWMFANCHECK, 5000, TERN(HAS_FANCHECK, 1000, 2500));
    #endif

  private:

    #if ENABLED(WATCH_HOTENDS)
      static hotend_watch_t watch_hotend[HOTENDS];
    #endif

    #if HAS_HOTEND
      // Sensor ranges, not user-configured
      static temp_range_t temp_range[HOTENDS];
    #endif

    #if HAS_HEATED_BED
      #if WATCH_BED
        static bed_watch_t watch_bed;
      #endif
      #if DISABLED(PIDTEMPBED)
        static millis_t next_bed_check_ms;
      #endif
      static temp_raw_range_t temp_sensor_range_bed;
    #endif

    #if HAS_HEATED_CHAMBER
      #if WATCH_CHAMBER
        static chamber_watch_t watch_chamber;
      #endif
      #if DISABLED(PIDTEMPCHAMBER)
        static millis_t next_chamber_check_ms;
      #endif
      static temp_raw_range_t temp_sensor_range_chamber;
    #endif

    #if HAS_COOLER
      #if WATCH_COOLER
        static cooler_watch_t watch_cooler;
      #endif
      static millis_t next_cooler_check_ms, cooler_fan_flush_ms;
      static temp_raw_range_t temp_sensor_range_cooler;
    #endif

    #if ALL(HAS_TEMP_BOARD, THERMAL_PROTECTION_BOARD)
      static temp_raw_range_t temp_sensor_range_board;
    #endif

    #if ALL(HAS_TEMP_SOC, THERMAL_PROTECTION_SOC)
      static raw_adc_t maxtemp_raw_SOC;
    #endif

    #if MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED > 1
      static uint8_t consecutive_low_temperature_error[HOTENDS];
    #endif

    #if HAS_FAN_LOGIC
      static millis_t fan_update_ms;

      static void manage_extruder_fans(millis_t ms) {
        if (ELAPSED(ms, fan_update_ms)) { // only need to check fan state very infrequently
          const millis_t next_ms = ms + fan_update_interval_ms;
          #if HAS_PWMFANCHECK
            #define FAN_CHECK_DURATION 100
            if (fan_check.is_measuring()) {
              fan_check.compute_speed(ms + FAN_CHECK_DURATION - fan_update_ms);
              fan_update_ms = next_ms;
            }
            else
              fan_update_ms = ms + FAN_CHECK_DURATION;
            fan_check.toggle_measuring();
          #else
            TERN_(HAS_FANCHECK, fan_check.compute_speed(next_ms - fan_update_ms));
            fan_update_ms = next_ms;
          #endif
          TERN_(HAS_AUTO_FAN, update_autofans()); // Needed as last when HAS_PWMFANCHECK to properly force fan speed
        }
      }
    #endif

    #if ENABLED(PROBING_HEATERS_OFF)
      static bool paused_for_probing;
    #endif

  public:
    /**
     * Instance Methods
     */

    void init();

    /**
     * Static (class) methods
     */

    #if HAS_USER_THERMISTORS
      static user_thermistor_t user_thermistor[USER_THERMISTORS];
      static void M305_report(const uint8_t t_index, const bool forReplay=true);
      static void reset_user_thermistors();
      static celsius_float_t user_thermistor_to_deg_c(const uint8_t t_index, const raw_adc_t raw);
      static bool set_pull_up_res(int8_t t_index, float value) {
        //if (!WITHIN(t_index, 0, USER_THERMISTORS - 1)) return false;
        if (!WITHIN(value, 1, 1000000)) return false;
        user_thermistor[t_index].series_res = value;
        return true;
      }
      static bool set_res25(int8_t t_index, float value) {
        if (!WITHIN(value, 1, 10000000)) return false;
        user_thermistor[t_index].res_25 = value;
        user_thermistor[t_index].pre_calc = true;
        return true;
      }
      static bool set_beta(int8_t t_index, float value) {
        if (!WITHIN(value, 1, 1000000)) return false;
        user_thermistor[t_index].beta = value;
        user_thermistor[t_index].pre_calc = true;
        return true;
      }
      static bool set_sh_coeff(int8_t t_index, float value) {
        if (!WITHIN(value, -0.01f, 0.01f)) return false;
        user_thermistor[t_index].sh_c_coeff = value;
        user_thermistor[t_index].pre_calc = true;
        return true;
      }
    #endif

    #if HAS_HOTEND
      static celsius_float_t analog_to_celsius_hotend(const raw_adc_t raw, const uint8_t e);
    #endif
    #if HAS_HEATED_BED
      static celsius_float_t analog_to_celsius_bed(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_CHAMBER
      static celsius_float_t analog_to_celsius_chamber(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_PROBE
      static celsius_float_t analog_to_celsius_probe(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_COOLER
      static celsius_float_t analog_to_celsius_cooler(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_BOARD
      static celsius_float_t analog_to_celsius_board(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_SOC
      static celsius_float_t analog_to_celsius_soc(const raw_adc_t raw);
    #endif
    #if HAS_TEMP_REDUNDANT
      static celsius_float_t analog_to_celsius_redundant(const raw_adc_t raw);
    #endif

    #if HAS_FAN

      static uint8_t fan_speed[FAN_COUNT];
      #define FANS_LOOP(I) for (uint8_t I = 0; I < FAN_COUNT; ++I)

      static void set_fan_speed(const uint8_t fan, const uint16_t speed);

      #if ENABLED(REPORT_FAN_CHANGE)
        static void report_fan_speed(const uint8_t fan);
      #endif

      #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
        static bool fans_paused;
        static uint8_t saved_fan_speed[FAN_COUNT];
      #endif

      #if ENABLED(ADAPTIVE_FAN_SLOWING)
        static uint8_t fan_speed_scaler[FAN_COUNT];
      #endif

      static uint8_t scaledFanSpeed(const uint8_t fan, const uint8_t fs) {
        UNUSED(fan); // Potentially unused!
        return (fs * uint16_t(TERN(ADAPTIVE_FAN_SLOWING, fan_speed_scaler[fan], 128))) >> 7;
      }

      static uint8_t scaledFanSpeed(const uint8_t fan) {
        return scaledFanSpeed(fan, fan_speed[fan]);
      }

      static constexpr inline uint8_t pwmToPercent(const uint8_t speed) { return ui8_to_percent(speed); }
      static uint8_t fanSpeedPercent(const uint8_t fan)          { return ui8_to_percent(fan_speed[fan]); }
      static uint8_t scaledFanSpeedPercent(const uint8_t fan)    { return ui8_to_percent(scaledFanSpeed(fan)); }

      #if ENABLED(EXTRA_FAN_SPEED)
        typedef struct { uint8_t saved, speed; } extra_fan_t;
        static extra_fan_t extra_fan_speed[FAN_COUNT];
        static void set_temp_fan_speed(const uint8_t fan, const uint16_t command_or_speed);
      #endif

      #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
        void set_fans_paused(const bool p);
      #endif

    #endif // HAS_FAN

    static void zero_fan_speeds() {
      #if HAS_FAN
        FANS_LOOP(i) set_fan_speed(i, 0);
      #endif
    }

    /**
     * Called from the Temperature ISR
     */
    static void isr();
    static void readings_ready();

    /**
     * Call periodically to manage heaters and keep the watchdog fed
     */
    static void task();

    /**
     * Preheating hotends & bed
     */
    #if PREHEAT_TIME_HOTEND_MS > 0
      static millis_t preheat_end_ms_hotend[HOTENDS];
      static bool is_hotend_preheating(const uint8_t E_NAME) {
        return preheat_end_ms_hotend[HOTEND_INDEX] && PENDING(millis(), preheat_end_ms_hotend[HOTEND_INDEX]);
      }
      static void start_hotend_preheat_time(const uint8_t E_NAME) {
        preheat_end_ms_hotend[HOTEND_INDEX] = millis() + PREHEAT_TIME_HOTEND_MS;
      }
      static void reset_hotend_preheat_time(const uint8_t E_NAME) {
        preheat_end_ms_hotend[HOTEND_INDEX] = 0;
      }
    #else
      static bool is_hotend_preheating(const uint8_t) { return false; }
    #endif

    #if HAS_HEATED_BED
      #if PREHEAT_TIME_BED_MS > 0
        static millis_t preheat_end_ms_bed;
        static bool is_bed_preheating() {
          return preheat_end_ms_bed && PENDING(millis(), preheat_end_ms_bed);
        }
        static void start_bed_preheat_time() {
          preheat_end_ms_bed = millis() + PREHEAT_TIME_BED_MS;
        }
        static void reset_bed_preheat_time() {
          preheat_end_ms_bed = 0;
        }
      #else
        static bool is_bed_preheating() { return false; }
      #endif
    #endif

    //high level conversion routines, for use outside of temperature.cpp
    //inline so that there is no performance decrease.
    //deg=degreeCelsius

    static celsius_float_t degHotend(const uint8_t E_NAME) {
      return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].celsius);
    }

    static celsius_t wholeDegHotend(const uint8_t E_NAME) {
      return TERN0(HAS_HOTEND, static_cast<celsius_t>(temp_hotend[HOTEND_INDEX].celsius + 0.5f));
    }

    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      static raw_adc_t rawHotendTemp(const uint8_t E_NAME) {
        return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].getraw());
      }
    #endif

    static celsius_t degTargetHotend(const uint8_t E_NAME) {
      return TERN0(HAS_HOTEND, temp_hotend[HOTEND_INDEX].target);
    }

    #if HAS_HOTEND

      static void setTargetHotend(const celsius_t celsius, const uint8_t E_NAME) {
        const uint8_t ee = HOTEND_INDEX;
        #if PREHEAT_TIME_HOTEND_MS > 0
          if (celsius == 0)
            reset_hotend_preheat_time(ee);
          else if (temp_hotend[ee].target == 0)
            start_hotend_preheat_time(ee);
        #endif
        TERN_(AUTO_POWER_CONTROL, if (celsius) powerManager.power_on());
        temp_hotend[ee].target = _MIN(celsius, hotend_max_target(ee));
        start_watching_hotend(ee);
      }

      static bool isHeatingHotend(const uint8_t E_NAME) {
        return temp_hotend[HOTEND_INDEX].target > temp_hotend[HOTEND_INDEX].celsius;
      }

      static bool isCoolingHotend(const uint8_t E_NAME) {
        return temp_hotend[HOTEND_INDEX].target < temp_hotend[HOTEND_INDEX].celsius;
      }

      #if HAS_TEMP_HOTEND
        static bool wait_for_hotend(const uint8_t target_extruder, const bool no_wait_for_cooling=true
          OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel=false)
        );

        #if ENABLED(WAIT_FOR_HOTEND)
          static void wait_for_hotend_heating(const uint8_t target_extruder);
        #endif
      #endif

      static bool degHotendNear(const uint8_t e, const celsius_t temp) {
        return ABS(wholeDegHotend(e) - temp) < (TEMP_HYSTERESIS);
      }

      static bool still_heating(const uint8_t e) {
        return degTargetHotend(e) > TEMP_HYSTERESIS && !degHotendNear(e, degTargetHotend(e));
      }

      // Start watching a Hotend to make sure it's really heating up
      static void start_watching_hotend(const uint8_t E_NAME) {
        UNUSED(HOTEND_INDEX);
        #if WATCH_HOTENDS
          watch_hotend[HOTEND_INDEX].restart(degHotend(HOTEND_INDEX), degTargetHotend(HOTEND_INDEX));
        #endif
      }

      static void manage_hotends(const millis_t &ms);

    #endif // HAS_HOTEND

    #if HAS_HEATED_BED

      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawBedTemp()  { return temp_bed.getraw(); }
      #endif
      static celsius_float_t degBed()  { return temp_bed.celsius; }
      static celsius_t wholeDegBed()   { return static_cast<celsius_t>(degBed() + 0.5f); }
      static celsius_t degTargetBed()  { return temp_bed.target; }
      static bool isHeatingBed()       { return temp_bed.target > temp_bed.celsius; }
      static bool isCoolingBed()       { return temp_bed.target < temp_bed.celsius; }
      static bool degBedNear(const celsius_t temp) {
        return ABS(wholeDegBed() - temp) < (TEMP_BED_HYSTERESIS);
      }

      // Start watching the Bed to make sure it's really heating up
      static void start_watching_bed() { OPTCODE(WATCH_BED, watch_bed.restart(degBed(), degTargetBed())) }

      static void setTargetBed(const celsius_t celsius) {
        #if PREHEAT_TIME_BED_MS > 0
          if (celsius == 0)
            reset_bed_preheat_time();
          else if (temp_bed.target == 0)
            start_bed_preheat_time();
        #endif
        TERN_(AUTO_POWER_CONTROL, if (celsius) powerManager.power_on());
        temp_bed.target = _MIN(celsius, BED_MAX_TARGET);
        start_watching_bed();
      }

      static bool wait_for_bed(const bool no_wait_for_cooling=true
        OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel=false)
      );

      static void wait_for_bed_heating();

      static void manage_heated_bed(const millis_t &ms);

    #endif // HAS_HEATED_BED

    #if HAS_TEMP_PROBE
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawProbeTemp()  { return temp_probe.getraw(); }
      #endif
      static celsius_float_t degProbe()  { return temp_probe.celsius; }
      static celsius_t wholeDegProbe()   { return static_cast<celsius_t>(degProbe() + 0.5f); }
      static bool isProbeBelowTemp(const celsius_t target_temp) { return wholeDegProbe() < target_temp; }
      static bool isProbeAboveTemp(const celsius_t target_temp) { return wholeDegProbe() > target_temp; }
      static bool wait_for_probe(const celsius_t target_temp, bool no_wait_for_cooling=true);
    #endif

    #if HAS_TEMP_CHAMBER
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawChamberTemp()    { return temp_chamber.getraw(); }
      #endif
      static celsius_float_t degChamber()    { return temp_chamber.celsius; }
      static celsius_t wholeDegChamber()     { return static_cast<celsius_t>(degChamber() + 0.5f); }
      #if HAS_HEATED_CHAMBER
        static celsius_t degTargetChamber()  { return temp_chamber.target; }
        static bool isHeatingChamber()       { return temp_chamber.target > temp_chamber.celsius; }
        static bool isCoolingChamber()       { return temp_chamber.target < temp_chamber.celsius; }
        static bool wait_for_chamber(const bool no_wait_for_cooling=true);
        static void manage_heated_chamber(const millis_t &ms);
      #endif
    #endif

    #if HAS_HEATED_CHAMBER
      static void setTargetChamber(const celsius_t celsius) {
        temp_chamber.target = _MIN(celsius, CHAMBER_MAX_TARGET);
        start_watching_chamber();
      }
      // Start watching the Chamber to make sure it's really heating up
      static void start_watching_chamber() { OPTCODE(WATCH_CHAMBER, watch_chamber.restart(degChamber(), degTargetChamber())) }
    #endif

    #if HAS_TEMP_COOLER
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawCoolerTemp()   { return temp_cooler.getraw(); }
      #endif
      static celsius_float_t degCooler()   { return temp_cooler.celsius; }
      static celsius_t wholeDegCooler()    { return static_cast<celsius_t>(temp_cooler.celsius + 0.5f); }
      #if HAS_COOLER
        static celsius_t degTargetCooler() { return temp_cooler.target; }
        static bool isLaserHeating()       { return temp_cooler.target > temp_cooler.celsius; }
        static bool isLaserCooling()       { return temp_cooler.target < temp_cooler.celsius; }
        static bool wait_for_cooler(const bool no_wait_for_cooling=true);
        static void manage_cooler(const millis_t &ms);
      #endif
    #endif

    #if HAS_TEMP_BOARD
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawBoardTemp()  { return temp_board.getraw(); }
      #endif
      static celsius_float_t degBoard()  { return temp_board.celsius; }
      static celsius_t wholeDegBoard()   { return static_cast<celsius_t>(temp_board.celsius + 0.5f); }
    #endif

    #if HAS_TEMP_SOC
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawSocTemp()    { return temp_soc.getraw(); }
      #endif
      static celsius_float_t degSoc()    { return temp_soc.celsius; }
      static celsius_t wholeDegSoc()     { return static_cast<celsius_t>(temp_soc.celsius + 0.5f); }
    #endif

    #if HAS_TEMP_REDUNDANT
      #if ENABLED(SHOW_TEMP_ADC_VALUES)
        static raw_adc_t rawRedundantTemp()       { return temp_redundant.getraw(); }
      #endif
      static celsius_float_t degRedundant()       { return temp_redundant.celsius; }
      static celsius_float_t degRedundantTarget() { return (*temp_redundant.target).celsius; }
      static celsius_t wholeDegRedundant()        { return static_cast<celsius_t>(temp_redundant.celsius + 0.5f); }
      static celsius_t wholeDegRedundantTarget()  { return static_cast<celsius_t>((*temp_redundant.target).celsius + 0.5f); }
    #endif

    #if HAS_COOLER
      static void setTargetCooler(const celsius_t celsius) {
        temp_cooler.target = constrain(celsius, COOLER_MIN_TARGET, COOLER_MAX_TARGET);
        start_watching_cooler();
      }
      // Start watching the Cooler to make sure it's really cooling down
      static void start_watching_cooler() { OPTCODE(WATCH_COOLER, watch_cooler.restart(degCooler(), degTargetCooler())) }
    #endif

    /**
     * The software PWM power for a heater
     */
    static int16_t getHeaterPower(const heater_id_t heater_id);

    /**
     * Switch off all heaters, set all target temperatures to 0
     */
    static void disable_all_heaters();

    /**
     * Cooldown, as from the LCD. Disables all heaters and fans.
     */
    static void cooldown() {
      zero_fan_speeds();
      disable_all_heaters();
    }

    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * Methods to check if heaters are enabled, indicating an active job
       */
      static bool auto_job_over_threshold();
      static void auto_job_check_timer(const bool can_start, const bool can_stop);
    #endif

    #if ENABLED(TEMP_TUNING_MAINTAIN_FAN)
      static bool adaptive_fan_slowing;
    #elif ENABLED(ADAPTIVE_FAN_SLOWING)
      static constexpr bool adaptive_fan_slowing = true;
    #endif

    static bool tuning_idle(const millis_t &ms);

    /**
     * M303 PID auto-tuning for hotends or bed
     */
    #if HAS_PID_HEATING

      #if HAS_PID_DEBUG
        static bool pid_debug_flag;
      #endif

      static void PID_autotune(const celsius_t target, const heater_id_t heater_id, const int8_t ncycles, const bool set_result=false);

      // Update the temp manager when PID values change
      #if ENABLED(PIDTEMP)
        static void updatePID() { HOTEND_LOOP() temp_hotend[e].pid.reset(); }
        static void setPID(const uint8_t hotend, const_float_t p, const_float_t i, const_float_t d) {
          #if ENABLED(PID_PARAMS_PER_HOTEND)
            temp_hotend[hotend].pid.set(p, i, d);
          #else
            HOTEND_LOOP() temp_hotend[e].pid.set(p, i, d);
          #endif
          updatePID();
        }
      #endif

    #endif // HAS_PID_HEATING

    /**
     * M306 MPC auto-tuning for hotends
     */
    #if ENABLED(MPC_AUTOTUNE)

      // Utility class to perform MPCTEMP auto tuning measurements
      class MPC_autotuner {
        public:
          enum MeasurementState { CANCELLED, FAILED, SUCCESS };
          MPC_autotuner(const uint8_t extruderIdx);
          ~MPC_autotuner();
          MeasurementState measure_ambient_temp();
          MeasurementState measure_heatup();
          MeasurementState measure_transfer();

          celsius_float_t get_ambient_temp() { return ambient_temp; }
          celsius_float_t get_last_measured_temp() { return current_temp; }

          float get_elapsed_heating_time() { return elapsed_heating_time; }
          float get_sample_1_time() { return t1_time; }
          static float get_sample_1_temp() { return temp_samples[0]; }
          static float get_sample_2_temp() { return temp_samples[(sample_count - 1) >> 1]; }
          static float get_sample_3_temp() { return temp_samples[sample_count - 1]; }
          static float get_sample_interval() { return sample_distance * (sample_count >> 1); }

          static celsius_float_t get_temp_fastest() { return temp_fastest; }
          float get_time_fastest() { return time_fastest; }
          float get_rate_fastest() { return rate_fastest; }

          float get_power_fan0() { return power_fan0; }
          #if HAS_FAN
            static float get_power_fan255() { return power_fan255; }
          #endif

        protected:
          static void init_timers() { curr_time_ms = next_report_ms = millis(); }
          MeasurementState housekeeping();

          uint8_t e;

          float elapsed_heating_time;
          celsius_float_t ambient_temp, current_temp;
          float t1_time;

          static millis_t curr_time_ms, next_report_ms;
          static celsius_float_t temp_samples[16];
          static uint8_t sample_count;
          static uint16_t sample_distance;

          // Parameters from differential analysis
          static celsius_float_t temp_fastest;
          float time_fastest, rate_fastest;

          float power_fan0;
          #if HAS_FAN
            static float power_fan255;
          #endif
      };

      enum MPCTuningType { AUTO, FORCE_ASYMPTOTIC, FORCE_DIFFERENTIAL };
      static void MPC_autotune(const uint8_t e, MPCTuningType tuning_type);

    #endif // MPC_AUTOTUNE

    #if ENABLED(PROBING_HEATERS_OFF)
      static void pause_heaters(const bool p);
    #endif

    #if HEATER_IDLE_HANDLER

      static void reset_hotend_idle_timer(const uint8_t E_NAME) {
        heater_idle[HOTEND_INDEX].reset();
        start_watching_hotend(HOTEND_INDEX);
      }

      #if HAS_HEATED_BED
        static void reset_bed_idle_timer() {
          heater_idle[IDLE_INDEX_BED].reset();
          start_watching_bed();
        }
      #endif

    #endif // HEATER_IDLE_HANDLER

    #if HAS_TEMP_SENSOR
      static void print_heater_states(const int8_t target_extruder
        OPTARG(HAS_TEMP_REDUNDANT, const bool include_r=false)
      );
      #if ENABLED(AUTO_REPORT_TEMPERATURES)
        struct AutoReportTemp { static void report(); };
        static AutoReporter<AutoReportTemp> auto_reporter;
      #endif
    #endif

    #if HAS_HOTEND && HAS_STATUS_MESSAGE
      static void set_heating_message(const uint8_t e, const bool isM104=false);
    #else
      static void set_heating_message(const uint8_t, const bool=false) {}
    #endif

    #if HAS_MARLINUI_MENU && HAS_TEMPERATURE && HAS_PREHEAT
      static void lcd_preheat(const uint8_t e, const int8_t indh, const int8_t indb);
    #endif

  private:

    // Reading raw temperatures and converting to Celsius when ready
    static volatile bool raw_temps_ready;
    static void update_raw_temperatures();
    static void updateTemperaturesFromRawValues();
    static bool updateTemperaturesIfReady() {
      if (!raw_temps_ready) return false;
      updateTemperaturesFromRawValues();
      raw_temps_ready = false;
      return true;
    }

    // MAX Thermocouples
    #if HAS_MAX_TC
      #define MAX_TC_COUNT TEMP_SENSOR_IS_MAX_TC(0) + TEMP_SENSOR_IS_MAX_TC(1) + TEMP_SENSOR_IS_MAX_TC(2) + TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
      #if MAX_TC_COUNT > 1
        #define HAS_MULTI_MAX_TC 1
      #endif
      #define READ_MAX_TC(N) read_max_tc(TERN_(HAS_MULTI_MAX_TC, N))
      static raw_adc_t read_max_tc(TERN_(HAS_MULTI_MAX_TC, const uint8_t hindex=0));
    #endif
    #if TEMP_SENSOR_IS_MAX_TC(BED)
      static raw_adc_t read_max_tc_bed();
    #endif

    #if HAS_AUTO_FAN
      #if ENABLED(POWER_OFF_WAIT_FOR_COOLDOWN)
        static bool autofans_on;
      #endif
      static void update_autofans();
    #endif

    #if HAS_HOTEND
      static float get_pid_output_hotend(const uint8_t e);
    #endif
    #if ENABLED(PIDTEMPBED)
      static float get_pid_output_bed();
    #endif
    #if ENABLED(PIDTEMPCHAMBER)
      static float get_pid_output_chamber();
    #endif

    static void _temp_error(const heater_id_t e, FSTR_P const serial_msg, FSTR_P const lcd_msg OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg));
    static void mintemp_error(const heater_id_t e OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg));
    static void maxtemp_error(const heater_id_t e OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg));

    #define _TEMP_ERROR(e, m, l, d) _temp_error(heater_id_t(e), m, GET_TEXT_F(l) OPTARG(ERR_INCLUDE_TEMP, d))
    #define MINTEMP_ERROR(e, d) mintemp_error(heater_id_t(e) OPTARG(ERR_INCLUDE_TEMP, d))
    #define MAXTEMP_ERROR(e, d) maxtemp_error(heater_id_t(e) OPTARG(ERR_INCLUDE_TEMP, d))

    #define HAS_THERMAL_PROTECTION ANY(THERMAL_PROTECTION_HOTENDS, THERMAL_PROTECTION_CHAMBER, THERMAL_PROTECTION_BED, THERMAL_PROTECTION_COOLER)

    #if HAS_THERMAL_PROTECTION

      // Indices and size for the tr_state_machine array. One for each protected heater.
      enum RunawayIndex : int8_t {
        _RI = -1
        #if ENABLED(THERMAL_PROTECTION_HOTENDS)
          #define _RUNAWAY_IND_E(N) ,RUNAWAY_IND_E##N
          REPEAT(HOTENDS, _RUNAWAY_IND_E)
          #undef _RUNAWAY_IND_E
        #endif
        OPTARG(THERMAL_PROTECTION_BED, RUNAWAY_IND_BED)
        OPTARG(THERMAL_PROTECTION_CHAMBER, RUNAWAY_IND_CHAMBER)
        OPTARG(THERMAL_PROTECTION_COOLER, RUNAWAY_IND_COOLER)
        , NR_HEATER_RUNAWAY
      };

      // Convert the given heater_id_t to runaway state array index
      static RunawayIndex runaway_index_for_id(const int8_t heater_id) {
        OPTCODE(THERMAL_PROTECTION_CHAMBER, if (heater_id == H_CHAMBER) return RUNAWAY_IND_CHAMBER)
        OPTCODE(THERMAL_PROTECTION_COOLER,  if (heater_id == H_COOLER)  return RUNAWAY_IND_COOLER)
        OPTCODE(THERMAL_PROTECTION_BED,     if (heater_id == H_BED)     return RUNAWAY_IND_BED)
        return (RunawayIndex)_MAX(heater_id, 0);
      }

      enum TRState : char { TRInactive, TRFirstHeating, TRStable, TRRunaway
        OPTARG(THERMAL_PROTECTION_VARIANCE_MONITOR, TRMalfunction)
      };

      typedef struct {
        millis_t timer = 0;
        TRState state = TRInactive;
        celsius_float_t running_temp;
        #if ENABLED(THERMAL_PROTECTION_VARIANCE_MONITOR)
          millis_t variance_timer = 0;
          celsius_float_t last_temp = 0.0, variance = 0.0;
        #endif
        void run(const_celsius_float_t current, const_celsius_float_t target, const heater_id_t heater_id, const uint16_t period_seconds, const celsius_float_t hysteresis_degc);
      } tr_state_machine_t;

      static tr_state_machine_t tr_state_machine[NR_HEATER_RUNAWAY];

    #endif // HAS_THERMAL_PROTECTION
};

extern Temperature thermalManager;
