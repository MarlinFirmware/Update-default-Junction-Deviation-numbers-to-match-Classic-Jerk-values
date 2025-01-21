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
 * temperature.cpp - temperature control
 */

// Useful when debugging thermocouples
//#define IGNORE_THERMOCOUPLE_ERRORS

#include "../MarlinCore.h"
#include "../HAL/shared/Delay.h"
#include "../lcd/marlinui.h"
#include "../gcode/gcode.h"

#include "temperature.h"
#include "endstops.h"
#include "planner.h"
#include "printcounter.h"

#if ANY(HAS_COOLER, LASER_COOLANT_FLOW_METER)
  #include "../feature/cooler.h"
  #include "../feature/spindle_laser.h"
#endif

#if ENABLED(USE_CONTROLLER_FAN)
  #include "../feature/controllerfan.h"
#endif

#if ENABLED(EMERGENCY_PARSER)
  #include "motion.h"
#endif

#if ENABLED(DWIN_CREALITY_LCD)
  #include "../lcd/e3v2/creality/dwin.h"
#elif ENABLED(SOVOL_SV06_RTS)
  #include "../lcd/sovol_rts/sovol_rts.h"
#endif

#if ENABLED(EXTENSIBLE_UI)
  #include "../lcd/extui/ui_api.h"
#endif

#if ENABLED(HOST_PROMPT_SUPPORT)
  #include "../feature/host_actions.h"
#endif

#if ENABLED(NOZZLE_PARK_FEATURE)
  #include "../libs/nozzle.h"
#endif

#if LASER_SAFETY_TIMEOUT_MS > 0
  #include "../feature/spindle_laser.h"
#endif

#ifndef TEMP_SENSOR_0
  #define TEMP_SENSOR_0 0
#endif
#ifndef TEMP_SENSOR_1
  #define TEMP_SENSOR_1 0
#endif
#ifndef TEMP_SENSOR_2
  #define TEMP_SENSOR_2 0
#endif
#ifndef TEMP_SENSOR_3
  #define TEMP_SENSOR_3 0
#endif
#ifndef TEMP_SENSOR_4
  #define TEMP_SENSOR_4 0
#endif
#ifndef TEMP_SENSOR_5
  #define TEMP_SENSOR_5 0
#endif
#ifndef TEMP_SENSOR_6
  #define TEMP_SENSOR_6 0
#endif
#ifndef TEMP_SENSOR_7
  #define TEMP_SENSOR_7 0
#endif
#ifndef TEMP_SENSOR_BED
  #define TEMP_SENSOR_BED 0
#endif
#ifndef TEMP_SENSOR_BOARD
  #define TEMP_SENSOR_BOARD 0
#endif
#ifndef TEMP_SENSOR_CHAMBER
  #define TEMP_SENSOR_CHAMBER 0
#endif
#ifndef TEMP_SENSOR_COOLER
  #define TEMP_SENSOR_COOLER 0
#endif
#ifndef TEMP_SENSOR_PROBE
  #define TEMP_SENSOR_PROBE 0
#endif
#ifndef TEMP_SENSOR_REDUNDANT
  #define TEMP_SENSOR_REDUNDANT 0
#endif
#ifndef TEMP_SENSOR_SOC
  #define TEMP_SENSOR_SOC 0
#endif

// MAX TC related macros
#define TEMP_SENSOR_IS_MAX(n, M) (ENABLED(TEMP_SENSOR_##n##_IS_MAX##M) || (ENABLED(TEMP_SENSOR_REDUNDANT_IS_MAX##M) && REDUNDANT_TEMP_MATCH(SOURCE, E##n)))

// LIB_MAX6675 can be added to the build_flags in platformio.ini to use a user-defined library
// If LIB_MAX6675 is not on the build_flags then raw SPI reads will be used.
#if HAS_MAX6675 && USE_LIB_MAX6675
  #include <max6675.h>
  #define HAS_MAX6675_LIBRARY 1
#endif

// LIB_MAX31855 can be added to the build_flags in platformio.ini to use a user-defined library.
// If LIB_MAX31855 is not on the build_flags then raw SPI reads will be used.
#if HAS_MAX31855 && USE_ADAFRUIT_MAX31855
  #include <Adafruit_MAX31855.h>
  #define HAS_MAX31855_LIBRARY 1
  typedef Adafruit_MAX31855 MAX31855;
#endif

#if HAS_MAX31865
  #if USE_ADAFRUIT_MAX31865
    #include <Adafruit_MAX31865.h>
    typedef Adafruit_MAX31865 MAX31865;
  #else
    #include "../libs/MAX31865.h"
  #endif
#endif

#if HAS_MAX6675_LIBRARY || HAS_MAX31855_LIBRARY || HAS_MAX31865
  #define HAS_MAXTC_LIBRARIES 1
#endif

// If we have a MAX TC with SCK and MISO pins defined, it's either on a separate/dedicated Hardware
// SPI bus, or some pins for Software SPI. Alternate Hardware SPI buses are not supported yet, so
// your SPI options are:
//
//  1. Only CS pin(s) defined:                  Hardware SPI on the default bus (usually the SD card SPI).
//  2. CS, MISO, and SCK pins defined:          Software SPI on a separate bus, as defined by MISO, SCK.
//  3. CS, MISO, and SCK pins w/ FORCE_HW_SPI:  Hardware SPI on the default bus, ignoring MISO, SCK.
//
#if TEMP_SENSOR_IS_ANY_MAX_TC(0) && TEMP_SENSOR_0_HAS_SPI_PINS && DISABLED(TEMP_SENSOR_FORCE_HW_SPI)
  #define TEMP_SENSOR_0_USES_SW_SPI 1
#endif
#if TEMP_SENSOR_IS_ANY_MAX_TC(1) && TEMP_SENSOR_1_HAS_SPI_PINS && DISABLED(TEMP_SENSOR_FORCE_HW_SPI)
  #define TEMP_SENSOR_1_USES_SW_SPI 1
#endif
#if TEMP_SENSOR_IS_ANY_MAX_TC(2) && TEMP_SENSOR_2_HAS_SPI_PINS && DISABLED(TEMP_SENSOR_FORCE_HW_SPI)
  #define TEMP_SENSOR_2_USES_SW_SPI 1
#endif
#if TEMP_SENSOR_IS_ANY_MAX_TC(BED) && TEMP_SENSOR_0_HAS_SPI_PINS && DISABLED(TEMP_SENSOR_FORCE_HW_SPI)
  #define TEMP_SENSOR_BED_USES_SW_SPI 1
#endif

#if (TEMP_SENSOR_0_USES_SW_SPI || TEMP_SENSOR_1_USES_SW_SPI || TEMP_SENSOR_2_USES_SW_SPI) && !HAS_MAXTC_LIBRARIES
  #include "../libs/private_spi.h"
  #define HAS_MAXTC_SW_SPI 1

  // Define pins for SPI-based sensors
  #if TEMP_SENSOR_0_USES_SW_SPI
    #define SW_SPI_SCK_PIN    TEMP_0_SCK_PIN
    #define SW_SPI_MISO_PIN   TEMP_0_MISO_PIN
    #if PIN_EXISTS(TEMP_0_MOSI)
      #define SW_SPI_MOSI_PIN TEMP_0_MOSI_PIN
    #endif
  #elif TEMP_SENSOR_1_USES_SW_SPI
    #define SW_SPI_SCK_PIN    TEMP_1_SCK_PIN
    #define SW_SPI_MISO_PIN   TEMP_1_MISO_PIN
    #if PIN_EXISTS(TEMP_1_MOSI)
      #define SW_SPI_MOSI_PIN TEMP_1_MOSI_PIN
    #endif
  #elif TEMP_SENSOR_2_USES_SW_SPI
    #define SW_SPI_SCK_PIN    TEMP_2_SCK_PIN
    #define SW_SPI_MISO_PIN   TEMP_2_MISO_PIN
    #if PIN_EXISTS(TEMP_2_MOSI)
      #define SW_SPI_MOSI_PIN TEMP_2_MOSI_PIN
    #endif
  #endif
  #ifndef SW_SPI_MOSI_PIN
    #define SW_SPI_MOSI_PIN   SD_MOSI_PIN
  #endif
#endif

#if ENABLED(MPCTEMP)
  #include <math.h>
  #include "probe.h"
#endif

#if ANY(MPCTEMP, PID_EXTRUSION_SCALING)
  #include "stepper.h"
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #include "../feature/filwidth.h"
#endif

#if HAS_POWER_MONITOR
  #include "../feature/power_monitor.h"
#endif

#if ENABLED(EMERGENCY_PARSER)
  #include "../feature/e_parser.h"
#endif

#if ENABLED(PRINTER_EVENT_LEDS)
  #include "../feature/leds/printer_event_leds.h"
#endif

#if ENABLED(JOYSTICK)
  #include "../feature/joystick.h"
#endif

#if HAS_BEEPER
  #include "../libs/buzzer.h"
#endif

#if HAS_SERVOS
  #include "servo.h"
#endif

#if ANY(TEMP_SENSOR_0_IS_THERMISTOR, TEMP_SENSOR_1_IS_THERMISTOR, TEMP_SENSOR_2_IS_THERMISTOR, TEMP_SENSOR_3_IS_THERMISTOR, \
        TEMP_SENSOR_4_IS_THERMISTOR, TEMP_SENSOR_5_IS_THERMISTOR, TEMP_SENSOR_6_IS_THERMISTOR, TEMP_SENSOR_7_IS_THERMISTOR )
  #define HAS_HOTEND_THERMISTOR 1
#endif

#if HAS_HOTEND_THERMISTOR
  #define NEXT_TEMPTABLE(N) ,TEMPTABLE_##N
  #define NEXT_TEMPTABLE_LEN(N) ,TEMPTABLE_##N##_LEN
  static const temp_entry_t* heater_ttbl_map[HOTENDS] = ARRAY_BY_HOTENDS(TEMPTABLE_0 REPEAT_S(1, HOTENDS, NEXT_TEMPTABLE));
  static constexpr uint8_t heater_ttbllen_map[HOTENDS] = ARRAY_BY_HOTENDS(TEMPTABLE_0_LEN REPEAT_S(1, HOTENDS, NEXT_TEMPTABLE_LEN));
#endif

Temperature thermalManager;

PGMSTR(str_t_thermal_runaway, STR_T_THERMAL_RUNAWAY);
PGMSTR(str_t_heating_failed, STR_T_HEATING_FAILED);

//
// Initialize MAX TC objects/SPI
//
#if HAS_MAX_TC

  #if HAS_MAXTC_SW_SPI
    // Initialize SoftSPI for non-lib Software SPI; Libraries take care of it themselves.
    template<uint8_t MisoPin, uint8_t MosiPin, uint8_t SckPin>
      SoftSPI<MisoPin, MosiPin, SckPin> SPIclass<MisoPin, MosiPin, SckPin>::softSPI;
    SPIclass<SW_SPI_MISO_PIN, SW_SPI_MOSI_PIN, SW_SPI_SCK_PIN> max_tc_spi;

  #endif

  #define MAXTC_INIT(n, M) \
      MAX##M max##M##_##n = MAX##M( \
        TEMP_##n##_CS_PIN \
        OPTARG(_MAX31865_##n##_SW, TEMP_##n##_MOSI_PIN) \
        OPTARG(TEMP_SENSOR_##n##_USES_SW_SPI, TEMP_##n##_MISO_PIN, TEMP_##n##_SCK_PIN) \
        OPTARG(LARGE_PINMAP, HIGH) \
      )

  #if HAS_MAX6675_LIBRARY
    #if TEMP_SENSOR_IS_MAX(0, 6675)
      MAXTC_INIT(0, 6675);
    #endif
    #if TEMP_SENSOR_IS_MAX(1, 6675)
      MAXTC_INIT(1, 6675);
    #endif
    #if TEMP_SENSOR_IS_MAX(2, 6675)
      MAXTC_INIT(2, 6675);
    #endif
  #endif

  #if HAS_MAX31855_LIBRARY
    #if TEMP_SENSOR_IS_MAX(0, 31855)
      MAXTC_INIT(0, 31855);
    #endif
    #if TEMP_SENSOR_IS_MAX(1, 31855)
      MAXTC_INIT(1, 31855);
    #endif
    #if TEMP_SENSOR_IS_MAX(2, 31855)
      MAXTC_INIT(2, 31855);
    #endif
  #endif

  // MAX31865 always uses a library, unlike '55 & 6675
  #if HAS_MAX31865
    #define _MAX31865_0_SW TEMP_SENSOR_0_USES_SW_SPI
    #define _MAX31865_1_SW TEMP_SENSOR_1_USES_SW_SPI
    #define _MAX31865_2_SW TEMP_SENSOR_2_USES_SW_SPI
    #define _MAX31865_BED_SW TEMP_SENSOR_BED_USES_SW_SPI

    #if TEMP_SENSOR_IS_MAX(0, 31865)
      MAXTC_INIT(0, 31865);
    #endif
    #if TEMP_SENSOR_IS_MAX(1, 31865)
      MAXTC_INIT(1, 31865);
    #endif
    #if TEMP_SENSOR_IS_MAX(2, 31865)
      MAXTC_INIT(2, 31865);
    #endif
    #if TEMP_SENSOR_IS_MAX(BED, 31865)
      MAXTC_INIT(BED, 31865);
    #endif

    #undef _MAX31865_0_SW
    #undef _MAX31865_1_SW
    #undef _MAX31865_2_SW
    #undef _MAX31865_BED_SW
  #endif

  #undef MAXTC_INIT

#endif

/**
 * public:
 */

#if ENABLED(TEMP_TUNING_MAINTAIN_FAN)
  bool Temperature::adaptive_fan_slowing = true;
#endif

#if HAS_HOTEND
  hotend_info_t Temperature::temp_hotend[HOTENDS];
  constexpr celsius_t Temperature::hotend_maxtemp[HOTENDS];

  #if ENABLED(MPCTEMP)
    bool MPC::e_paused; // = false
    int32_t MPC::e_position; // = 0
  #endif

  // Sanity-check max readable temperatures
  #define CHECK_MAXTEMP_(N,M,S) static_assert( \
    S >= 998 || M <= _MAX(TT_NAME(S)[0].celsius, TT_NAME(S)[COUNT(TT_NAME(S)) - 1].celsius) - (HOTEND_OVERSHOOT), \
    "HEATER_" STRINGIFY(N) "_MAXTEMP (" STRINGIFY(M) ") is too high for thermistor_" STRINGIFY(S) ".h with HOTEND_OVERSHOOT=" STRINGIFY(HOTEND_OVERSHOOT) ".");
  #define CHECK_MAXTEMP(N) TERN(TEMP_SENSOR_##N##_IS_THERMISTOR, CHECK_MAXTEMP_, CODE_0)(N, HEATER_##N##_MAXTEMP, TEMP_SENSOR_##N)
  REPEAT(HOTENDS, CHECK_MAXTEMP)

  #if HAS_PREHEAT
    #define CHECK_PREHEAT__(N,P,T,M) static_assert(T <= (M) - (HOTEND_OVERSHOOT), "PREHEAT_" STRINGIFY(P) "_TEMP_HOTEND (" STRINGIFY(T) ") must be less than HEATER_" STRINGIFY(N) "_MAXTEMP (" STRINGIFY(M) ") - " STRINGIFY(HOTEND_OVERSHOOT) ".");
    #define CHECK_PREHEAT_(N,P) CHECK_PREHEAT__(N, P, PREHEAT_##P##_TEMP_HOTEND, HEATER_##N##_MAXTEMP)
    #define CHECK_PREHEAT(P) REPEAT2(HOTENDS, CHECK_PREHEAT_, P)
    #if PREHEAT_COUNT >= 1
      CHECK_PREHEAT(1)
    #endif
    #if PREHEAT_COUNT >= 2
      CHECK_PREHEAT(2)
    #endif
    #if PREHEAT_COUNT >= 3
      CHECK_PREHEAT(3)
    #endif
    #if PREHEAT_COUNT >= 4
      CHECK_PREHEAT(4)
    #endif
    #if PREHEAT_COUNT >= 5
      CHECK_PREHEAT(5)
    #endif
    #if PREHEAT_COUNT >= 6
      CHECK_PREHEAT(6)
    #endif
    #if PREHEAT_COUNT >= 7
      CHECK_PREHEAT(7)
    #endif
    #if PREHEAT_COUNT >= 8
      CHECK_PREHEAT(8)
    #endif
    #if PREHEAT_COUNT >= 9
      CHECK_PREHEAT(9)
    #endif
    #if PREHEAT_COUNT >= 10
      CHECK_PREHEAT(10)
    #endif
  #endif // HAS_PREHEAT

#endif // HAS_HOTEND

#if HAS_TEMP_REDUNDANT
  redundant_info_t Temperature::temp_redundant;
#endif

#if ANY(AUTO_POWER_E_FANS, HAS_FANCHECK)
  uint8_t Temperature::autofan_speed[HOTENDS] = ARRAY_N_1(HOTENDS, FAN_OFF_PWM);
#endif

#if ENABLED(AUTO_POWER_CHAMBER_FAN)
  uint8_t Temperature::chamberfan_speed = FAN_OFF_PWM;
#endif

#if ENABLED(AUTO_POWER_COOLER_FAN)
  uint8_t Temperature::coolerfan_speed = FAN_OFF_PWM;
#endif

#if ALL(FAN_SOFT_PWM, USE_CONTROLLER_FAN)
  uint8_t Temperature::soft_pwm_controller_speed = FAN_OFF_PWM;
#endif

// Init fans according to whether they're native PWM or Software PWM
#ifdef BOARD_OPENDRAIN_MOSFETS
  #define _INIT_SOFT_FAN(P) OUT_WRITE_OD(P, ENABLED(FAN_INVERTING) ? LOW : HIGH)
#else
  #define _INIT_SOFT_FAN(P) OUT_WRITE(P, ENABLED(FAN_INVERTING) ? LOW : HIGH)
#endif
#if ENABLED(FAN_SOFT_PWM)
  #define _INIT_FAN_PIN(P) _INIT_SOFT_FAN(P)
#else
  #define _INIT_FAN_PIN(P) do{ if (PWM_PIN(P)) SET_PWM(P); else _INIT_SOFT_FAN(P); }while(0)
#endif
#if ENABLED(FAST_PWM_FAN)
  #define SET_FAST_PWM_FREQ(P) hal.set_pwm_frequency(pin_t(P), FAST_PWM_FAN_FREQUENCY)
#else
  #define SET_FAST_PWM_FREQ(P) NOOP
#endif
#define INIT_FAN_PIN(P) do{ _INIT_FAN_PIN(P); SET_FAST_PWM_FREQ(P); }while(0)

// HAS_FAN does not include CONTROLLER_FAN
#if HAS_FAN

  uint8_t Temperature::fan_speed[FAN_COUNT] = ARRAY_N_1(FAN_COUNT, FAN_OFF_PWM);

  #if ENABLED(EXTRA_FAN_SPEED)

    Temperature::extra_fan_t Temperature::extra_fan_speed[FAN_COUNT] = ARRAY_N_1(FAN_COUNT, FAN_OFF_PWM);

    /**
     * Handle the M106 P<fan> T<speed> command:
     *  T1       = Restore fan speed saved on the last T2
     *  T2       = Save the fan speed, then set to the last T<3-255> value
     *  T<3-255> = Set the "extra fan speed"
     */
    void Temperature::set_temp_fan_speed(const uint8_t fan, const uint16_t command_or_speed) {
      switch (command_or_speed) {
        case 1:
          set_fan_speed(fan, extra_fan_speed[fan].saved);
          break;
        case 2:
          extra_fan_speed[fan].saved = fan_speed[fan];
          set_fan_speed(fan, extra_fan_speed[fan].speed);
          break;
        default:
          extra_fan_speed[fan].speed = _MIN(command_or_speed, 255U);
          break;
      }
    }

  #endif

  #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)
    bool Temperature::fans_paused; // = false
    uint8_t Temperature::saved_fan_speed[FAN_COUNT] = ARRAY_N_1(FAN_COUNT, FAN_OFF_PWM);
  #endif

  #if ENABLED(ADAPTIVE_FAN_SLOWING)
    uint8_t Temperature::fan_speed_scaler[FAN_COUNT] = ARRAY_N_1(FAN_COUNT, 128);
  #endif

  /**
   * Set the print fan speed for a target extruder
   */
  void Temperature::set_fan_speed(uint8_t fan, uint16_t speed) {

    NOMORE(speed, 255U);

    #if ENABLED(SINGLENOZZLE_STANDBY_FAN)
      if (fan != active_extruder) {
        if (fan < EXTRUDERS) singlenozzle_fan_speed[fan] = speed;
        return;
      }
    #endif

    TERN_(SINGLENOZZLE, if (fan < EXTRUDERS) fan = 0); // Always fan 0 for SINGLENOZZLE E fan

    if (fan >= FAN_COUNT) return;

    fan_speed[fan] = speed;

    #if NUM_REDUNDANT_FANS
      if (fan == 0) {
        for (uint8_t f = REDUNDANT_PART_COOLING_FAN; f < REDUNDANT_PART_COOLING_FAN + NUM_REDUNDANT_FANS; ++f)
          thermalManager.set_fan_speed(f, speed);
      }
    #endif

    TERN_(REPORT_FAN_CHANGE, report_fan_speed(fan));
  }

  #if ENABLED(REPORT_FAN_CHANGE)
    /**
     * Report print fan speed for a target extruder
     */
    void Temperature::report_fan_speed(const uint8_t fan) {
      if (fan >= FAN_COUNT) return;
      PORT_REDIRECT(SerialMask::All);
      SERIAL_ECHOLNPGM("M106 P", fan, " S", fan_speed[fan]);
    }
  #endif

  #if ANY(PROBING_FANS_OFF, ADVANCED_PAUSE_FANS_PAUSE)

    void Temperature::set_fans_paused(const bool p) {
      if (p != fans_paused) {
        fans_paused = p;
        if (p)
          FANS_LOOP(i) { saved_fan_speed[i] = fan_speed[i]; fan_speed[i] = 0; }
        else
          FANS_LOOP(i) fan_speed[i] = saved_fan_speed[i];
      }
    }

  #endif

#endif // HAS_FAN

#if WATCH_HOTENDS
  hotend_watch_t Temperature::watch_hotend[HOTENDS]; // = { { 0 } }
#endif
#if HEATER_IDLE_HANDLER
  Temperature::heater_idle_t Temperature::heater_idle[NR_HEATER_IDLE]; // = { { 0 } }
#endif

#if HAS_HEATED_BED
  bed_info_t Temperature::temp_bed; // = { 0 }
  // Init min and max temp with extreme values to prevent false errors during startup
  temp_raw_range_t Temperature::temp_sensor_range_bed = { TEMP_SENSOR_BED_RAW_LO_TEMP, TEMP_SENSOR_BED_RAW_HI_TEMP };
  #if WATCH_BED
    bed_watch_t Temperature::watch_bed; // = { 0 }
  #endif
  #if DISABLED(PIDTEMPBED)
    millis_t Temperature::next_bed_check_ms;
  #endif
#endif

#if HAS_TEMP_CHAMBER
  chamber_info_t Temperature::temp_chamber; // = { 0 }
  #if HAS_HEATED_CHAMBER
    millis_t next_cool_check_ms = 0;
    celsius_float_t old_temp = 9999;
    temp_raw_range_t Temperature::temp_sensor_range_chamber = { TEMP_SENSOR_CHAMBER_RAW_LO_TEMP, TEMP_SENSOR_CHAMBER_RAW_HI_TEMP };
    #if WATCH_CHAMBER
      chamber_watch_t Temperature::watch_chamber; // = { 0 }
    #endif
    #if DISABLED(PIDTEMPCHAMBER)
      millis_t Temperature::next_chamber_check_ms;
    #endif
  #endif
#endif

#if HAS_TEMP_COOLER
  cooler_info_t Temperature::temp_cooler; // = { 0 }
  #if HAS_COOLER
    bool flag_cooler_state;
    //bool flag_cooler_excess = false;
    celsius_float_t previous_temp = 9999;
    temp_raw_range_t Temperature::temp_sensor_range_cooler = { TEMP_SENSOR_COOLER_RAW_LO_TEMP, TEMP_SENSOR_COOLER_RAW_HI_TEMP };
    #if WATCH_COOLER
      cooler_watch_t Temperature::watch_cooler; // = { 0 }
    #endif
    millis_t Temperature::next_cooler_check_ms, Temperature::cooler_fan_flush_ms;
  #endif
#endif

#if HAS_TEMP_PROBE
  probe_info_t Temperature::temp_probe; // = { 0 }
#endif

#if HAS_TEMP_BOARD
  board_info_t Temperature::temp_board; // = { 0 }
  #if ENABLED(THERMAL_PROTECTION_BOARD)
    temp_raw_range_t Temperature::temp_sensor_range_board = { TEMP_SENSOR_BOARD_RAW_LO_TEMP, TEMP_SENSOR_BOARD_RAW_HI_TEMP };
  #endif
#endif

#if HAS_TEMP_SOC
  soc_info_t Temperature::temp_soc; // = { 0 }
  raw_adc_t Temperature::maxtemp_raw_SOC = TEMP_SENSOR_SOC_RAW_HI_TEMP;
#endif

#if ALL(HAS_MARLINUI_MENU, PREVENT_COLD_EXTRUSION) && E_MANUAL > 0
  bool Temperature::allow_cold_extrude_override = false;
#else
  constexpr bool Temperature::allow_cold_extrude_override;
#endif

#if ENABLED(PREVENT_COLD_EXTRUSION)
  bool Temperature::allow_cold_extrude = false;
  celsius_t Temperature::extrude_min_temp = EXTRUDE_MINTEMP;
#else
  constexpr bool Temperature::allow_cold_extrude;
  constexpr celsius_t Temperature::extrude_min_temp;
#endif

#if HAS_ADC_BUTTONS
  uint32_t Temperature::current_ADCKey_raw = HAL_ADC_RANGE;
  uint16_t Temperature::ADCKey_count = 0;
#endif

#if ENABLED(PID_EXTRUSION_SCALING)
  int16_t Temperature::lpq_len; // Initialized in settings.cpp
#endif

/**
 * private:
 */

volatile bool Temperature::raw_temps_ready = false;

#define TEMPDIR(N) ((TEMP_SENSOR_##N##_RAW_LO_TEMP) < (TEMP_SENSOR_##N##_RAW_HI_TEMP) ? 1 : -1)
#define TP_CMP(S,A,B) (TEMPDIR(S) < 0 ? ((A)<(B)) : ((A)>(B)))

#if HAS_HOTEND
  // Init mintemp and maxtemp with extreme values to prevent false errors during startup
  constexpr temp_range_t sensor_heater_0 { TEMP_SENSOR_0_RAW_LO_TEMP, TEMP_SENSOR_0_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_1 { TEMP_SENSOR_1_RAW_LO_TEMP, TEMP_SENSOR_1_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_2 { TEMP_SENSOR_2_RAW_LO_TEMP, TEMP_SENSOR_2_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_3 { TEMP_SENSOR_3_RAW_LO_TEMP, TEMP_SENSOR_3_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_4 { TEMP_SENSOR_4_RAW_LO_TEMP, TEMP_SENSOR_4_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_5 { TEMP_SENSOR_5_RAW_LO_TEMP, TEMP_SENSOR_5_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_6 { TEMP_SENSOR_6_RAW_LO_TEMP, TEMP_SENSOR_6_RAW_HI_TEMP, 0, 16383 },
                         sensor_heater_7 { TEMP_SENSOR_7_RAW_LO_TEMP, TEMP_SENSOR_7_RAW_HI_TEMP, 0, 16383 };

  temp_range_t Temperature::temp_range[HOTENDS] = ARRAY_BY_HOTENDS(sensor_heater_0, sensor_heater_1, sensor_heater_2, sensor_heater_3, sensor_heater_4, sensor_heater_5, sensor_heater_6, sensor_heater_7);
#endif

#if MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED > 1
  #define MULTI_MAX_CONSECUTIVE_LOW_TEMP_ERR 1
  uint8_t Temperature::consecutive_low_temperature_error[HOTENDS]; // = { 0 }
#endif

#if PREHEAT_TIME_HOTEND_MS > 0
  millis_t Temperature::preheat_end_ms_hotend[HOTENDS]; // = { 0 };
#endif
#if HAS_HEATED_BED && PREHEAT_TIME_BED_MS > 0
  millis_t Temperature::preheat_end_ms_bed = 0;
#endif

#if HAS_FAN_LOGIC
  constexpr millis_t Temperature::fan_update_interval_ms;
  millis_t Temperature::fan_update_ms = 0;
#endif

#if ENABLED(FAN_SOFT_PWM)
  uint8_t Temperature::soft_pwm_amount_fan[FAN_COUNT],
          Temperature::soft_pwm_count_fan[FAN_COUNT];
#endif

#if ENABLED(SINGLENOZZLE_STANDBY_TEMP)
  celsius_t Temperature::singlenozzle_temp[EXTRUDERS];
#endif
#if ENABLED(SINGLENOZZLE_STANDBY_FAN)
  uint8_t Temperature::singlenozzle_fan_speed[EXTRUDERS];
#endif

#if ENABLED(PROBING_HEATERS_OFF)
  bool Temperature::paused_for_probing;
#endif

/**
 * public:
 * Class and Instance Methods
 */

#if ANY(HAS_PID_HEATING, MPC_AUTOTUNE)

  /**
   * Run the minimal required activities during a tuning loop.
   * TODO: Allow tuning routines to call idle() for more complete keepalive.
   */
  bool Temperature::tuning_idle(const millis_t &ms) {

    // Run HAL idle tasks
    hal.idletask();

    const bool temp_ready = updateTemperaturesIfReady();

    #if HAS_FAN_LOGIC
      if (temp_ready) manage_extruder_fans(ms);
    #else
      UNUSED(ms);
    #endif

    // Run Controller Fan check (normally handled by manage_inactivity)
    TERN_(USE_CONTROLLER_FAN, controllerFan.update());

    // Run UI update
    #if ENABLED(SOVOL_SV06_RTS)
      RTS_Update();
    #else
      ui.update();
    #endif

    // Update beeper queue
    TERN_(HAS_BEEPER, buzzer.tick());

    return temp_ready;
  }

#endif

#if HAS_PID_HEATING

  inline void say_default_() { SERIAL_ECHOPGM("#define DEFAULT_"); }

  /**
   * PID Autotuning (M303)
   *
   * Alternately heat and cool the nozzle, observing its behavior to
   * determine the best PID values to achieve a stable temperature.
   * Needs sufficient heater power to make some overshoot at target
   * temperature to succeed.
   */
  void Temperature::PID_autotune(const celsius_t target, const heater_id_t heater_id, const int8_t ncycles, const bool set_result/*=false*/) {
    celsius_float_t current_temp = 0.0;
    int cycles = 0;
    bool heating = true;

    millis_t next_temp_ms = millis(), t1 = next_temp_ms, t2 = next_temp_ms;
    long t_high = 0, t_low = 0;

    raw_pid_t tune_pid = { 0, 0, 0 };
    celsius_float_t maxT = 0, minT = 10000;

    const bool isbed = (heater_id == H_BED),
           ischamber = (heater_id == H_CHAMBER);

    #if ENABLED(PIDTEMPCHAMBER)
      #define C_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define C_TERN(T,A,B) (B)
    #endif
    #if ENABLED(PIDTEMPBED)
      #define B_TERN(T,A,B) ((T) ? (A) : (B))
    #else
      #define B_TERN(T,A,B) (B)
    #endif
    #define GHV(C,B,H) C_TERN(ischamber, C, B_TERN(isbed, B, H))
    #define SHV(V) C_TERN(ischamber, temp_chamber.soft_pwm_amount = V, B_TERN(isbed, temp_bed.soft_pwm_amount = V, temp_hotend[heater_id].soft_pwm_amount = V))
    #define ONHEATINGSTART() C_TERN(ischamber, printerEventLEDs.onChamberHeatingStart(), B_TERN(isbed, printerEventLEDs.onBedHeatingStart(), printerEventLEDs.onHotendHeatingStart()))
    #define ONHEATING(S,C,T) C_TERN(ischamber, printerEventLEDs.onChamberHeating(S,C,T), B_TERN(isbed, printerEventLEDs.onBedHeating(S,C,T), printerEventLEDs.onHotendHeating(S,C,T)))

    #define WATCH_PID DISABLED(NO_WATCH_PID_TUNING) && (ALL(WATCH_CHAMBER, PIDTEMPCHAMBER) || ALL(WATCH_BED, PIDTEMPBED) || ALL(WATCH_HOTENDS, PIDTEMP))

    #if WATCH_PID
      #if ALL(THERMAL_PROTECTION_CHAMBER, PIDTEMPCHAMBER)
        #define C_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define C_GTV(T,A,B) (B)
      #endif
      #if ALL(THERMAL_PROTECTION_BED, PIDTEMPBED)
        #define B_GTV(T,A,B) ((T) ? (A) : (B))
      #else
        #define B_GTV(T,A,B) (B)
      #endif
      #define GTV(C,B,H) C_GTV(ischamber, C, B_GTV(isbed, B, H))
      const uint16_t watch_temp_period = GTV(WATCH_CHAMBER_TEMP_PERIOD, WATCH_BED_TEMP_PERIOD, WATCH_TEMP_PERIOD);
      const uint8_t watch_temp_increase = GTV(WATCH_CHAMBER_TEMP_INCREASE, WATCH_BED_TEMP_INCREASE, WATCH_TEMP_INCREASE);
      const celsius_float_t watch_temp_target = celsius_float_t(target - (watch_temp_increase + GTV(TEMP_CHAMBER_HYSTERESIS, TEMP_BED_HYSTERESIS, TEMP_HYSTERESIS) + 1));
      millis_t temp_change_ms = next_temp_ms + SEC_TO_MS(watch_temp_period);
      celsius_float_t next_watch_temp = 0.0;
      bool heated = false;
    #endif

    TERN_(HAS_FAN_LOGIC, fan_update_ms = next_temp_ms + fan_update_interval_ms);

    TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ischamber ? ExtUI::pidresult_t::PID_CHAMBER_STARTED : isbed ? ExtUI::pidresult_t::PID_BED_STARTED : ExtUI::pidresult_t::PID_STARTED));

    if (target > GHV(CHAMBER_MAX_TARGET, BED_MAX_TARGET, hotend_max_target(heater_id))) {
      SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
      TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ExtUI::pidresult_t::PID_TEMP_TOO_HIGH));
      TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
      return;
    }

    SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_START);

    disable_all_heaters();
    TERN_(AUTO_POWER_CONTROL, powerManager.power_on());

    long bias = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX) >> 1, d = bias;
    SHV(bias);

    #if ENABLED(PRINTER_EVENT_LEDS)
      const celsius_float_t start_temp = GHV(degChamber(), degBed(), degHotend(heater_id));
      const LEDColor oldcolor = ONHEATINGSTART();
    #endif

    TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = false);

    LCD_MESSAGE(MSG_HEATING);

    // PID Tuning loop
    wait_for_heatup = true;
    while (wait_for_heatup) { // Can be interrupted with M108

      const millis_t ms = millis();

      // Run minimal necessary machine tasks
      const bool temp_ready = tuning_idle(ms);

      // If a new sample has arrived process things
      if (temp_ready) {

        // Get the current temperature and constrain it
        current_temp = GHV(degChamber(), degBed(), degHotend(heater_id));
        NOLESS(maxT, current_temp);
        NOMORE(minT, current_temp);

        #if ENABLED(PRINTER_EVENT_LEDS)
          ONHEATING(start_temp, current_temp, target);
        #endif

        if (heating && current_temp > target && ELAPSED(ms, t2 + 5000UL)) {
          heating = false;
          SHV((bias - d) >> 1);
          t1 = ms;
          t_high = t1 - t2;
          maxT = target;
        }

        if (!heating && current_temp < target && ELAPSED(ms, t1 + 5000UL)) {
          heating = true;
          t2 = ms;
          t_low = t2 - t1;
          if (cycles > 0) {
            const long max_pow = GHV(MAX_CHAMBER_POWER, MAX_BED_POWER, PID_MAX);
            bias += (d * (t_high - t_low)) / (t_low + t_high);
            LIMIT(bias, 20, max_pow - 20);
            d = (bias > max_pow >> 1) ? max_pow - 1 - bias : bias;

            SERIAL_ECHOPGM(STR_BIAS, bias, STR_D_COLON, d, STR_T_MIN, minT, STR_T_MAX, maxT);
            if (cycles > 2) {
              const float Ku = (4.0f * d) / (float(M_PI) * (maxT - minT) * 0.5f),
                          Tu = float(t_low + t_high) * 0.001f,
                          pf = (ischamber || isbed) ? 0.2f : 0.6f,
                          df = (ischamber || isbed) ? 1.0f / 3.0f : 1.0f / 8.0f;

              tune_pid.p = Ku * pf;
              tune_pid.i = tune_pid.p * 2.0f / Tu;
              tune_pid.d = tune_pid.p * Tu * df;

              SERIAL_ECHOLNPGM(STR_KU, Ku, STR_TU, Tu);
              if (ischamber || isbed)
                SERIAL_ECHOLNPGM(" No overshoot");
              else
                SERIAL_ECHOLNPGM(STR_CLASSIC_PID);
              SERIAL_ECHOLNPGM(STR_KP, tune_pid.p, STR_KI, tune_pid.i, STR_KD, tune_pid.d);
            }
          }
          SHV((bias + d) >> 1);
          TERN_(HAS_STATUS_MESSAGE, ui.status_printf(0, F(S_FMT " %i/%i"), GET_TEXT_F(MSG_PID_CYCLE), cycles, ncycles));
          cycles++;
          minT = target;
        }
      }

      // Did the temperature overshoot very far?
      #ifndef MAX_OVERSHOOT_PID_AUTOTUNE
        #define MAX_OVERSHOOT_PID_AUTOTUNE 30
      #endif
      if (current_temp > target + MAX_OVERSHOOT_PID_AUTOTUNE) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TEMP_TOO_HIGH);
        TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ExtUI::pidresult_t::PID_TEMP_TOO_HIGH));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TEMP_TOO_HIGH)));
        break;
      }

      // Report heater states every 2 seconds
      if (ELAPSED(ms, next_temp_ms)) {
        #if HAS_TEMP_SENSOR
          print_heater_states(heater_id < 0 ? active_extruder : (int8_t)heater_id);
          SERIAL_EOL();
        #endif
        next_temp_ms = ms + 2000UL;

        // Make sure heating is actually working
        #if WATCH_PID
          if (ALL(WATCH_BED, WATCH_HOTENDS) || isbed == DISABLED(WATCH_HOTENDS) || ischamber == DISABLED(WATCH_HOTENDS)) {
            if (!heated) {                                            // If not yet reached target...
              if (current_temp > next_watch_temp) {                   // Over the watch temp?
                next_watch_temp = current_temp + watch_temp_increase; // - set the next temp to watch for
                temp_change_ms = ms + SEC_TO_MS(watch_temp_period);   // - move the expiration timer up
                if (current_temp > watch_temp_target) heated = true;  // - Flag if target temperature reached
              }
              else if (ELAPSED(ms, temp_change_ms)) {                 // Watch timer expired
                TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillHeat_L, ID_KillHeat_D));
                _TEMP_ERROR(heater_id, FPSTR(str_t_heating_failed), MSG_ERR_HEATING_FAILED, current_temp);
              }
            }
            else if (current_temp < target - (MAX_OVERSHOOT_PID_AUTOTUNE)) { // Heated, then temperature fell too far?
              TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillRunaway_L, ID_KillRunaway_D));
              _TEMP_ERROR(heater_id, FPSTR(str_t_thermal_runaway), MSG_ERR_THERMAL_RUNAWAY, current_temp);
            }
          }
        #endif
      } // every 2 seconds

      // Timeout after PID_AUTOTUNE_MAX_CYCLE_MINS minutes since the last undershoot/overshoot cycle
      #ifndef PID_AUTOTUNE_MAX_CYCLE_MINS
        #define PID_AUTOTUNE_MAX_CYCLE_MINS 20L
      #endif
      if ((ms - _MIN(t1, t2)) > MIN_TO_MS(PID_AUTOTUNE_MAX_CYCLE_MINS)) {
        TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
        TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ExtUI::pidresult_t::PID_TUNING_TIMEOUT));
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_TIMEOUT)));
        TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillHeat_L, ID_KillHeat_D));
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_TIMEOUT);
        break;
      }

      if (cycles > ncycles && cycles > 2) {
        SERIAL_ECHOPGM(STR_PID_AUTOTUNE); SERIAL_ECHOLNPGM(STR_PID_AUTOTUNE_FINISHED);
        TERN_(HOST_PROMPT_SUPPORT, hostui.notify(GET_TEXT_F(MSG_PID_AUTOTUNE_DONE)));

        #if ANY(PIDTEMPBED, PIDTEMPCHAMBER)
          FSTR_P const estring = GHV(F("chamber"), F("bed"), FPSTR(NUL_STR));
          say_default_(); SERIAL_ECHOLN(estring, F("Kp "), tune_pid.p);
          say_default_(); SERIAL_ECHOLN(estring, F("Ki "), tune_pid.i);
          say_default_(); SERIAL_ECHOLN(estring, F("Kd "), tune_pid.d);
        #else
          say_default_(); SERIAL_ECHOLNPGM("Kp ", tune_pid.p);
          say_default_(); SERIAL_ECHOLNPGM("Ki ", tune_pid.i);
          say_default_(); SERIAL_ECHOLNPGM("Kd ", tune_pid.d);
        #endif

        auto _set_hotend_pid = [](const uint8_t tool, const raw_pid_t &in_pid) {
          #if ENABLED(PIDTEMP)
            #if ENABLED(PID_PARAMS_PER_HOTEND)
              thermalManager.temp_hotend[tool].pid.set(in_pid);
            #else
              HOTEND_LOOP() thermalManager.temp_hotend[e].pid.set(in_pid);
            #endif
            updatePID();
          #endif
          UNUSED(tool); UNUSED(in_pid);
        };

        #if ENABLED(PIDTEMPBED)
          auto _set_bed_pid = [](const raw_pid_t &in_pid) {
            temp_bed.pid.set(in_pid);
          };
        #endif

        #if ENABLED(PIDTEMPCHAMBER)
          auto _set_chamber_pid = [](const raw_pid_t &in_pid) {
            temp_chamber.pid.set(in_pid);
          };
        #endif

        // Use the result? (As with "M303 U1")
        if (set_result)
          GHV(_set_chamber_pid(tune_pid), _set_bed_pid(tune_pid), _set_hotend_pid(heater_id, tune_pid));

        goto EXIT_M303;
      }
    }
    wait_for_heatup = false;

    disable_all_heaters();

    EXIT_M303:
      TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onPIDTuningDone(oldcolor));
      TERN_(EXTENSIBLE_UI, ExtUI::onPIDTuning(ExtUI::pidresult_t::PID_DONE));
      TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = true);
      return;
  }

#endif // HAS_PID_HEATING

#if ENABLED(MPC_AUTOTUNE)

  #if ANY(MPC_FAN_0_ALL_HOTENDS, MPC_FAN_0_ACTIVE_HOTEND)
    #define SINGLEFAN 1
  #endif

  millis_t Temperature::MPC_autotuner::curr_time_ms, Temperature::MPC_autotuner::next_report_ms;

  celsius_float_t Temperature::MPC_autotuner::temp_samples[16];
  uint8_t Temperature::MPC_autotuner::sample_count;
  uint16_t Temperature::MPC_autotuner::sample_distance;

  // Parameters from differential analysis
  celsius_float_t Temperature::MPC_autotuner::temp_fastest;

  #if HAS_FAN
    float Temperature::MPC_autotuner::power_fan255;
  #endif

  Temperature::MPC_autotuner::MPC_autotuner(const uint8_t extruderIdx) : e(extruderIdx) {
    TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = false);
  }

  Temperature::MPC_autotuner::~MPC_autotuner() {
    wait_for_heatup = false;

    ui.reset_status();

    temp_hotend[e].target = 0.0f;
    temp_hotend[e].soft_pwm_amount = 0;
    #if HAS_FAN
      set_fan_speed(TERN(SINGLEFAN, 0, e), 0);
      planner.sync_fan_speeds(fan_speed);
    #endif

    do_z_clearance(MPC_TUNING_END_Z, false);

    TERN_(TEMP_TUNING_MAINTAIN_FAN, adaptive_fan_slowing = true);
  }

  Temperature::MPC_autotuner::MeasurementState Temperature::MPC_autotuner::measure_ambient_temp() {
    init_timers();
    const millis_t test_interval_ms = 10000UL;
    millis_t next_test_ms = curr_time_ms + test_interval_ms;
    ambient_temp = current_temp = degHotend(e);
    wait_for_heatup = true;

    for (;;) { // Can be interrupted with M108
      if (housekeeping() == CANCELLED) return CANCELLED;

      if (ELAPSED(curr_time_ms, next_test_ms)) {
        if (current_temp >= ambient_temp) {
          ambient_temp = (ambient_temp + current_temp) / 2.0f;
          break;
        }
        ambient_temp = current_temp;
        next_test_ms += test_interval_ms;
      }
    }
    wait_for_heatup = false;

    #if ENABLED(MPC_AUTOTUNE_DEBUG)
      SERIAL_ECHOLNPGM("MPC_autotuner::measure_ambient_temp() Completed");
      SERIAL_ECHOLNPGM("=====");
      SERIAL_ECHOLNPGM("ambient_temp ", get_ambient_temp());
    #endif

    return SUCCESS;
  }

  Temperature::MPC_autotuner::MeasurementState Temperature::MPC_autotuner::measure_heatup() {
    init_timers();
    constexpr millis_t test_interval_ms = 1000UL;
    millis_t next_test_time_ms = curr_time_ms + test_interval_ms;
    MPCHeaterInfo &hotend = temp_hotend[e];

    current_temp = degHotend(e);
    millis_t heat_start_time_ms = curr_time_ms;
    sample_count = 0;
    sample_distance = 1;
    t1_time = 0;

    hotend.target = 200.0f;   // So M105 looks nice
    hotend.soft_pwm_amount = (MPC_MAX) >> 1;

    // Initialise rate of change to to steady state at current time
    temp_samples[0] = temp_samples[1] = temp_samples[2] = current_temp;
    time_fastest = rate_fastest = 0;

    wait_for_heatup = true;
    for (;;) { // Can be interrupted with M108
      if (housekeeping() == CANCELLED) return CANCELLED;

      if (ELAPSED(curr_time_ms, next_test_time_ms)) {
        if (current_temp < 100.0f) {
          // Initial regime (below 100deg): Measure rate of change of heating for differential tuning

          // Update the buffer of previous readings
          temp_samples[0] = temp_samples[1];
          temp_samples[1] = temp_samples[2];
          temp_samples[2] = current_temp;

          // Measure the rate of change of temperature, https://en.wikipedia.org/wiki/Symmetric_derivative
          const float h = MS_TO_SEC_PRECISE(test_interval_ms),
              curr_rate = (temp_samples[2] - temp_samples[0]) / 2 / h;
          if (curr_rate > rate_fastest) {
            // Update fastest values
            rate_fastest = curr_rate;
            temp_fastest = temp_samples[1];
            time_fastest = MS_TO_SEC_PRECISE(curr_time_ms - heat_start_time_ms - h);
          }

          next_test_time_ms += test_interval_ms;
        }
        else if (current_temp < 200.0f) {
          // Second regime (after 100deg) measure 3 points to determine asymptotic temperature

          // If there are too many samples, space them more widely
          if (sample_count == COUNT(temp_samples)) {
            for (uint8_t i = 0; i < COUNT(temp_samples) / 2; i++)
              temp_samples[i] = temp_samples[i * 2];
            sample_count /= 2;
            sample_distance *= 2;
          }

          if (sample_count == 0) t1_time = MS_TO_SEC_PRECISE(curr_time_ms - heat_start_time_ms);
          temp_samples[sample_count++] = current_temp;

          next_test_time_ms += test_interval_ms * sample_distance;
        }
        else {
          // Third regime (after 200deg) finished gathering data so finish
          break;
        }
      }
    }
    wait_for_heatup = false;

    hotend.soft_pwm_amount = 0;

    elapsed_heating_time = MS_TO_SEC_PRECISE(curr_time_ms - heat_start_time_ms);

    // Ensure sample count is odd so that we have 3 equally spaced samples
    if (sample_count == 0) return FAILED;
    if (sample_count % 2 == 0) sample_count--;

    #if ENABLED(MPC_AUTOTUNE_DEBUG)
      SERIAL_ECHOLNPGM("MPC_autotuner::measure_heatup() Completed");
      SERIAL_ECHOLNPGM("=====");
      SERIAL_ECHOLNPGM("t1_time ", t1_time);
      SERIAL_ECHOLNPGM("sample_count ", sample_count);
      SERIAL_ECHOLNPGM("sample_distance ", sample_distance);
      for (uint8_t i = 0; i < sample_count; i++)
        SERIAL_ECHOLNPGM("sample ", i, " : ", temp_samples[i]);
      SERIAL_ECHOLNPGM("t1 ", get_sample_1_temp(), " t2 ", get_sample_2_temp(), " t3 ", get_sample_3_temp());
    #endif

    return SUCCESS;
  }

  Temperature::MPC_autotuner::MeasurementState Temperature::MPC_autotuner::measure_transfer() {
    init_timers();
    const millis_t test_interval_ms = SEC_TO_MS(MPC_dT);
    millis_t next_test_ms = curr_time_ms + test_interval_ms;
    MPCHeaterInfo &hotend = temp_hotend[e];
    MPC_t &mpc = hotend.mpc;

    constexpr millis_t settle_time = 20000UL, test_duration = 20000UL;
    millis_t settle_end_ms = curr_time_ms + settle_time,
             test_end_ms = settle_end_ms + test_duration;
    float total_energy_fan0 = 0.0f;
    #if HAS_FAN
      bool fan0_done = false;
      float total_energy_fan255 = 0.0f;
    #endif
    float last_temp = current_temp;

    wait_for_heatup = true;
    for (;;) { // Can be interrupted with M108
      if (housekeeping() == CANCELLED) return CANCELLED;

      if (ELAPSED(curr_time_ms, next_test_ms)) {
        hotend.soft_pwm_amount = (int)get_pid_output_hotend(e) >> 1;

        if (ELAPSED(curr_time_ms, settle_end_ms) && PENDING(curr_time_ms, test_end_ms) && TERN1(HAS_FAN, !fan0_done))
          total_energy_fan0 += mpc.heater_power * hotend.soft_pwm_amount / 127 * MPC_dT + (last_temp - current_temp) * mpc.block_heat_capacity;
        #if HAS_FAN
          else if (ELAPSED(curr_time_ms, test_end_ms) && !fan0_done) {
            set_fan_speed(TERN(SINGLEFAN, 0, e), 255);
            planner.sync_fan_speeds(fan_speed);
            settle_end_ms = curr_time_ms + settle_time;
            test_end_ms = settle_end_ms + test_duration;
            fan0_done = true;
          }
          else if (ELAPSED(curr_time_ms, settle_end_ms) && PENDING(curr_time_ms, test_end_ms))
            total_energy_fan255 += mpc.heater_power * hotend.soft_pwm_amount / 127 * MPC_dT + (last_temp - current_temp) * mpc.block_heat_capacity;
        #endif
        else if (ELAPSED(curr_time_ms, test_end_ms)) break;

        last_temp = current_temp;
        next_test_ms += test_interval_ms;
      }

      // Ensure we don't drift too far from the window between the last sampled temp and the target temperature
      if (!WITHIN(current_temp, get_sample_3_temp() - 15.0f, hotend.target + 15.0f)) {
        SERIAL_ECHOLNPGM(STR_MPC_TEMPERATURE_ERROR);
        TERN_(EXTENSIBLE_UI, ExtUI::onMPCTuning(ExtUI::mpcresult_t::MPC_TEMP_ERROR));
        wait_for_heatup = false;
        return FAILED;
      }
    }
    wait_for_heatup = false;

    power_fan0 = total_energy_fan0 / MS_TO_SEC_PRECISE(test_duration);
    TERN_(HAS_FAN, power_fan255 = (total_energy_fan255 * 1000) / test_duration);

    #if ENABLED(MPC_AUTOTUNE_DEBUG)
      SERIAL_ECHOLNPGM("MPC_autotuner::measure_transfer() Completed");
      SERIAL_ECHOLNPGM("=====");
      SERIAL_ECHOLNPGM("power_fan0 ", power_fan0);
      TERN_(HAS_FAN, SERIAL_ECHOLNPGM("power_fan255 ", power_fan255));
    #endif

    return SUCCESS;
  }

  Temperature::MPC_autotuner::MeasurementState Temperature::MPC_autotuner::housekeeping() {
    constexpr millis_t report_interval_ms = 1000UL;
    curr_time_ms = millis();

    // Run minimal necessary machine tasks
    const bool temp_ready = tuning_idle(curr_time_ms);

    // Set MPC temp if a new sample is ready
    if (temp_ready) current_temp = degHotend(e);

    if (ELAPSED(curr_time_ms, next_report_ms)) {
      next_report_ms += report_interval_ms;
      print_heater_states(e);
      SERIAL_EOL();
    }

    if (!wait_for_heatup) {
      SERIAL_ECHOLNPGM(STR_MPC_AUTOTUNE_INTERRUPTED);
      TERN_(EXTENSIBLE_UI, ExtUI::onMPCTuning(ExtUI::mpcresult_t::MPC_INTERRUPTED));
      return MeasurementState::CANCELLED;
    }

    return MeasurementState::SUCCESS;
  }

  void Temperature::MPC_autotune(const uint8_t e, MPCTuningType tuning_type=AUTO) {
    SERIAL_ECHOLNPGM(STR_MPC_AUTOTUNE_START, e);

    MPC_autotuner tuner(e);

    MPCHeaterInfo &hotend = temp_hotend[e];
    MPC_t &mpc = hotend.mpc;

    // Move to center of bed, just above bed height and cool with max fan
    gcode.home_all_axes(true);
    disable_all_heaters();
    #if HAS_FAN
      zero_fan_speeds();
      set_fan_speed(TERN(SINGLEFAN, 0, e), 255);
      planner.sync_fan_speeds(fan_speed);
    #endif
    do_blocking_move_to(xyz_pos_t(MPC_TUNING_POS));

    // Determine ambient temperature.
    SERIAL_ECHOLNPGM(STR_MPC_COOLING_TO_AMBIENT);
    TERN_(EXTENSIBLE_UI, ExtUI::onMPCTuning(ExtUI::mpcresult_t::MPC_STARTED));
    TERN(DWIN_LCD_PROUI, LCD_ALERTMESSAGE(MSG_MPC_COOLING_TO_AMBIENT), LCD_MESSAGE(MSG_COOLING));

    if (tuner.measure_ambient_temp() != MPC_autotuner::MeasurementState::SUCCESS) return;
    hotend.modeled_ambient_temp = tuner.get_ambient_temp();

    #if HAS_FAN
      set_fan_speed(TERN(SINGLEFAN, 0, e), 0);
      planner.sync_fan_speeds(fan_speed);
    #endif

    // Heat to 200 degrees
    SERIAL_ECHOLNPGM(STR_MPC_HEATING_PAST_200);
    LCD_ALERTMESSAGE(MSG_MPC_HEATING_PAST_200);

    if (tuner.measure_heatup() != MPC_autotuner::MeasurementState::SUCCESS) return;

    // Calculate physical constants from three equally-spaced samples
    const float t1 = tuner.get_sample_1_temp(),
                t2 = tuner.get_sample_2_temp(),
                t3 = tuner.get_sample_3_temp();
    float asymp_temp = (t2 * t2 - t1 * t3) / (2 * t2 - t1 - t3),
          block_responsiveness = -log((t2 - asymp_temp) / (t1 - asymp_temp)) / tuner.get_sample_interval();

    #if ENABLED(MPC_AUTOTUNE_DEBUG)
      SERIAL_ECHOLNPGM("asymp_temp ", asymp_temp);
      SERIAL_ECHOLNPGM("block_responsiveness ", p_float_t(block_responsiveness, 4));
    #endif

    // Make initial guess at transfer coefficients
    mpc.ambient_xfer_coeff_fan0 = mpc.heater_power * (MPC_MAX) / 255 / (asymp_temp - tuner.get_ambient_temp());
    TERN_(MPC_INCLUDE_FAN, mpc.fan255_adjustment = 0.0f);

    if (tuning_type == AUTO || tuning_type == FORCE_ASYMPTOTIC) {
      // Analytic tuning
      mpc.block_heat_capacity = mpc.ambient_xfer_coeff_fan0 / block_responsiveness;
      mpc.sensor_responsiveness = block_responsiveness / (1.0f - (tuner.get_ambient_temp() - asymp_temp) * exp(-block_responsiveness * tuner.get_sample_1_time()) / (t1 - asymp_temp));
    }

    // If analytic tuning fails, fall back to differential tuning
    if (tuning_type == AUTO && (mpc.sensor_responsiveness <= 0 || mpc.block_heat_capacity <= 0))
        tuning_type = FORCE_DIFFERENTIAL;

    if (tuning_type == FORCE_DIFFERENTIAL) {
      #if ENABLED(MPC_AUTOTUNE_DEBUG)
        SERIAL_ECHOLNPGM("rate_fastest ", tuner.get_rate_fastest());
        SERIAL_ECHOLNPGM("time_fastest ", tuner.get_time_fastest());
        SERIAL_ECHOLNPGM("temp_fastest ", tuner.get_temp_fastest());
      #endif
      // Differential tuning
      mpc.block_heat_capacity = mpc.heater_power / tuner.get_rate_fastest();
      mpc.sensor_responsiveness = tuner.get_rate_fastest() / (tuner.get_rate_fastest() * tuner.get_time_fastest() + tuner.get_ambient_temp() - tuner.get_temp_fastest());
    }

    hotend.modeled_block_temp = asymp_temp + (tuner.get_ambient_temp() - asymp_temp) * exp(-block_responsiveness * tuner.get_elapsed_heating_time());
    hotend.modeled_sensor_temp = tuner.get_last_measured_temp();

    // Allow the system to stabilize under MPC, then get a better measure of ambient loss with and without fan
    SERIAL_ECHOLNPGM(STR_MPC_MEASURING_AMBIENT, hotend.modeled_block_temp);
    TERN(DWIN_LCD_PROUI, LCD_ALERTMESSAGE(MSG_MPC_MEASURING_AMBIENT), LCD_MESSAGE(MSG_MPC_MEASURING_AMBIENT));

    // Use the estimated overshoot of the temperature as the target to achieve.
    hotend.target = hotend.modeled_block_temp;
    if (tuner.measure_transfer() != MPC_autotuner::MeasurementState::SUCCESS) return;

    // Update the transfer coefficients
    mpc.ambient_xfer_coeff_fan0 = tuner.get_power_fan0() / (hotend.target - tuner.get_ambient_temp());
    #if HAS_FAN
      const float ambient_xfer_coeff_fan255 = tuner.get_power_fan255() / (hotend.target - tuner.get_ambient_temp());
      mpc.applyFanAdjustment(ambient_xfer_coeff_fan255);
    #endif

    if (tuning_type == AUTO || tuning_type == FORCE_ASYMPTOTIC) {
      // Calculate a new and better asymptotic temperature and re-evaluate the other constants
      asymp_temp = tuner.get_ambient_temp() + mpc.heater_power * (MPC_MAX) / 255 / mpc.ambient_xfer_coeff_fan0;
      block_responsiveness = -log((t2 - asymp_temp) / (t1 - asymp_temp)) / tuner.get_sample_interval();

      #if ENABLED(MPC_AUTOTUNE_DEBUG)
        SERIAL_ECHOLNPGM("Refining estimates for:");
        SERIAL_ECHOLNPGM("asymp_temp ", asymp_temp);
        SERIAL_ECHOLNPGM("block_responsiveness ", p_float_t(block_responsiveness, 4));
      #endif

      // Update analytic tuning values based on the above
      mpc.block_heat_capacity = mpc.ambient_xfer_coeff_fan0 / block_responsiveness;
      mpc.sensor_responsiveness = block_responsiveness / (1.0f - (tuner.get_ambient_temp() - asymp_temp) * exp(-block_responsiveness * tuner.get_sample_1_time()) / (t1 - asymp_temp));

      // Check again: If analytic tuning fails, fall back to differential tuning
      if (tuning_type == AUTO && (mpc.sensor_responsiveness <= 0 || mpc.block_heat_capacity <= 0)) {
        #if ENABLED(MPC_AUTOTUNE_DEBUG)
          SERIAL_ECHOLNPGM("rate_fastest ", tuner.get_rate_fastest());
          SERIAL_ECHOLNPGM("time_fastest ", tuner.get_time_fastest());
          SERIAL_ECHOLNPGM("temp_fastest ", tuner.get_temp_fastest());
        #endif
        tuning_type = FORCE_DIFFERENTIAL;
        mpc.block_heat_capacity = mpc.heater_power / tuner.get_rate_fastest();
        mpc.sensor_responsiveness = tuner.get_rate_fastest() / (tuner.get_rate_fastest() * tuner.get_time_fastest() + tuner.get_ambient_temp() - tuner.get_temp_fastest());
      }
    }

    SERIAL_ECHOLNPGM(STR_MPC_AUTOTUNE_FINISHED);
    TERN_(EXTENSIBLE_UI, ExtUI::onMPCTuning(ExtUI::mpcresult_t::MPC_DONE));

    SERIAL_ECHOLNPGM("MPC_BLOCK_HEAT_CAPACITY ", mpc.block_heat_capacity);
    SERIAL_ECHOLNPGM("MPC_SENSOR_RESPONSIVENESS ", p_float_t(mpc.sensor_responsiveness, 4));
    SERIAL_ECHOLNPGM("MPC_AMBIENT_XFER_COEFF ", p_float_t(mpc.ambient_xfer_coeff_fan0, 4));
    TERN_(HAS_FAN, SERIAL_ECHOLNPGM("MPC_AMBIENT_XFER_COEFF_FAN255 ", p_float_t(ambient_xfer_coeff_fan255, 4)));
  }

#endif // MPC_AUTOTUNE

int16_t Temperature::getHeaterPower(const heater_id_t heater_id) {
  switch (heater_id) {
    #if HAS_HEATED_BED
      case H_BED: return temp_bed.soft_pwm_amount;
    #endif
    #if HAS_HEATED_CHAMBER
      case H_CHAMBER: return temp_chamber.soft_pwm_amount;
    #endif
    #if HAS_COOLER
      case H_COOLER: return temp_cooler.soft_pwm_amount;
    #endif
    #if HAS_HOTEND
      case 0 ... HOTENDS - 1: return temp_hotend[heater_id].soft_pwm_amount;
    #endif
    default: return 0;
  }
}

#if HAS_AUTO_FAN

  #define _EFANOVERLAP(I,N) ((I != N) && _FANOVERLAP(I,E##N))

  #if EXTRUDER_AUTO_FAN_SPEED != 255
    #define INIT_E_AUTO_FAN_PIN(P) do{ if (PWM_PIN(P)) { SET_PWM(P); SET_FAST_PWM_FREQ(P); } else SET_OUTPUT(P); }while(0)
  #else
    #define INIT_E_AUTO_FAN_PIN(P) SET_OUTPUT(P)
  #endif
  #if CHAMBER_AUTO_FAN_SPEED != 255
    #define INIT_CHAMBER_AUTO_FAN_PIN(P) do{ if (PWM_PIN(P)) { SET_PWM(P); SET_FAST_PWM_FREQ(P); } else SET_OUTPUT(P); }while(0)
  #else
    #define INIT_CHAMBER_AUTO_FAN_PIN(P) SET_OUTPUT(P)
  #endif
  #if COOLER_AUTO_FAN_SPEED != 255
    #define INIT_COOLER_AUTO_FAN_PIN(P) do{ if (PWM_PIN(P)) { SET_PWM(P); SET_FAST_PWM_FREQ(P); } else SET_OUTPUT(P); }while(0)
  #else
    #define INIT_COOLER_AUTO_FAN_PIN(P) SET_OUTPUT(P)
  #endif

  #ifndef CHAMBER_FAN_INDEX
    #define CHAMBER_FAN_INDEX HOTENDS
  #endif

  void Temperature::update_autofans() {
    #define _EFAN(I,N) _EFANOVERLAP(I,N) ? I :
    static const uint8_t fanBit[] PROGMEM = {
      0
      #if HAS_MULTI_HOTEND
        #define _NEXT_FAN(N) , REPEAT2(N,_EFAN,N) N
        RREPEAT_S(1, HOTENDS, _NEXT_FAN)
      #endif
      #define _NFAN HOTENDS
      #if HAS_AUTO_CHAMBER_FAN
        #define _CHFAN(I) _FANOVERLAP(I,CHAMBER) ? I :
        , (REPEAT(HOTENDS,_CHFAN) (_NFAN))
        #undef _NFAN
        #define _NFAN INCREMENT(HOTENDS)
      #endif
      #if HAS_AUTO_COOLER_FAN
        #define _COFAN(I) _FANOVERLAP(I,COOLER) ? I :
        , (REPEAT(HOTENDS,_COFAN) (_NFAN))
      #endif
    };

    uint8_t fanState = 0;
    HOTEND_LOOP() {
      if (temp_hotend[e].celsius >= EXTRUDER_AUTO_FAN_TEMPERATURE)
        SBI(fanState, pgm_read_byte(&fanBit[e]));
    }

    #if HAS_AUTO_CHAMBER_FAN
      if (temp_chamber.celsius >= CHAMBER_AUTO_FAN_TEMPERATURE)
        SBI(fanState, pgm_read_byte(&fanBit[CHAMBER_FAN_INDEX]));
    #endif

    #if HAS_AUTO_COOLER_FAN
      if (temp_cooler.celsius >= COOLER_AUTO_FAN_TEMPERATURE)
        SBI(fanState, pgm_read_byte(&fanBit[COOLER_FAN_INDEX]));
    #endif

    #define _UPDATE_AUTO_FAN(P,D,A) do{                   \
      if (PWM_PIN(P##_AUTO_FAN_PIN) && A < 255)           \
        hal.set_pwm_duty(pin_t(P##_AUTO_FAN_PIN), D ? A : 0); \
      else                                                \
        WRITE(P##_AUTO_FAN_PIN, D);                       \
    }while(0)

    uint8_t fanDone = 0;
    for (uint8_t f = 0; f < COUNT(fanBit); ++f) {
      const uint8_t realFan = pgm_read_byte(&fanBit[f]);
      if (TEST(fanDone, realFan)) continue;
      const bool fan_on = TEST(fanState, realFan);
      switch (f) {
        #if ENABLED(AUTO_POWER_CHAMBER_FAN)
          case CHAMBER_FAN_INDEX:
            chamberfan_speed = fan_on ? CHAMBER_AUTO_FAN_SPEED : 0;
            break;
        #endif
        #if ENABLED(AUTO_POWER_COOLER_FAN)
          case COOLER_FAN_INDEX:
            coolerfan_speed = fan_on ? COOLER_AUTO_FAN_SPEED : 0;
            break;
        #endif
        default:
          #if ANY(AUTO_POWER_E_FANS, HAS_FANCHECK)
            autofan_speed[realFan] = fan_on ? EXTRUDER_AUTO_FAN_SPEED : 0;
          #endif
          break;
      }

      #if ALL(HAS_FANCHECK, HAS_PWMFANCHECK)
        #define _AUTOFAN_SPEED() fan_check.is_measuring() ? 255 : EXTRUDER_AUTO_FAN_SPEED
      #else
        #define _AUTOFAN_SPEED() EXTRUDER_AUTO_FAN_SPEED
      #endif
      #define _AUTOFAN_CASE(N) case N: _UPDATE_AUTO_FAN(E##N, fan_on, _AUTOFAN_SPEED()); break;
      #define _AUTOFAN_NOT(N)
      #define AUTOFAN_CASE(N) TERN(HAS_AUTO_FAN_##N, _AUTOFAN_CASE, _AUTOFAN_NOT)(N)

      switch (f) {
        REPEAT(HOTENDS, AUTOFAN_CASE)
        #if HAS_AUTO_CHAMBER_FAN && !AUTO_CHAMBER_IS_E
          case CHAMBER_FAN_INDEX: _UPDATE_AUTO_FAN(CHAMBER, fan_on, CHAMBER_AUTO_FAN_SPEED); break;
        #endif
        #if HAS_AUTO_COOLER_FAN && !AUTO_COOLER_IS_E
          case COOLER_FAN_INDEX: _UPDATE_AUTO_FAN(COOLER, fan_on, COOLER_AUTO_FAN_SPEED); break;
        #endif
      }
      SBI(fanDone, realFan);
    }
  }

#endif // HAS_AUTO_FAN

//
// Temperature Error Handlers
//

/**
 * Loud Kill
 * @brief  Produce a loud alarm, park the head, "kill" the machine
 *         in response to a temperature error, e.g., a thermal runaway.
 * @param  lcd_msg:  The message to display on the LCD
 * @param  heater_id:  The heater that caused the error
 */
inline void loud_kill(FSTR_P const lcd_msg, const heater_id_t heater_id) {
  marlin_state = MarlinState::MF_KILLED;
  thermalManager.disable_all_heaters();
  #if HAS_BEEPER
    for (uint8_t i = 20; i--;) {
      hal.watchdog_refresh();
      buzzer.click(25);
      delay(80);
      hal.watchdog_refresh();
    }
    buzzer.on();
  #endif
  #if ENABLED(NOZZLE_PARK_FEATURE)
    if (!homing_needed_error()) {
      nozzle.park(0);
      planner.synchronize();
    }
  #endif

  #define _FSTR_BED(h)     TERN(HAS_HEATED_BED,     (h) == H_BED      ? GET_TEXT_F(MSG_BED) :,)
  #define _FSTR_CHAMBER(h) TERN(HAS_HEATED_CHAMBER, (h) == H_CHAMBER  ? GET_TEXT_F(MSG_CHAMBER) :,)
  #define _FSTR_COOLER(h)  TERN(HAS_COOLER,         (h) == H_COOLER   ? GET_TEXT_F(MSG_COOLER) :,)
  #define _FSTR_E(h,N)     TERN(HAS_HOTEND,        ((h) == N && (HOTENDS) > N) ? F(STR_E##N) :,)
  #define HEATER_FSTR(h) _FSTR_BED(h) _FSTR_CHAMBER(h) _FSTR_COOLER(h) \
                         _FSTR_E(h,1) _FSTR_E(h,2) _FSTR_E(h,3) _FSTR_E(h,4) \
                         _FSTR_E(h,5) _FSTR_E(h,6) _FSTR_E(h,7) F(STR_E0)

  kill(lcd_msg, HEATER_FSTR(heater_id));
}

/**
 * Temperature Error
 * @brief  Handle a temperature error, e.g., a thermal runaway.
 * @param  heater_id:  The heater that caused the error
 * @param  serial_msg:  The message to display on the serial console
 * @param  lcd_msg:  The message to display on the LCD
 * @param  deg:  The detected temperature (if ERR_INCLUDE_TEMP)
 */
void Temperature::_temp_error(
  const heater_id_t heater_id, FSTR_P const serial_msg, FSTR_P const lcd_msg
  OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg)
) {
  #if BOGUS_TEMPERATURE_GRACE_PERIOD
    #define HAS_BOGUS_TEMPERATURE_GRACE_PERIOD 1
  #endif
  static uint8_t killed = 0;

  if (IsRunning() && killed == TERN(HAS_BOGUS_TEMPERATURE_GRACE_PERIOD, 2, 0)) {
    SERIAL_ERROR_START();
    SERIAL_ECHO(serial_msg);
    SERIAL_ECHOPGM(STR_STOPPED_HEATER);

    heater_id_t real_heater_id = heater_id;

    #if HAS_TEMP_REDUNDANT
      if (heater_id == H_REDUNDANT) {
        SERIAL_ECHOPGM(STR_REDUNDANT); // print redundant and cascade to print target, too.
        real_heater_id = (heater_id_t)HEATER_ID(TEMP_SENSOR_REDUNDANT_TARGET);
      }
    #endif

    switch (real_heater_id) {
      OPTCODE(HAS_TEMP_COOLER,  case H_COOLER:  SERIAL_ECHOPGM(STR_COOLER);         break)
      OPTCODE(HAS_TEMP_PROBE,   case H_PROBE:   SERIAL_ECHOPGM(STR_PROBE);          break)
      OPTCODE(HAS_TEMP_BOARD,   case H_BOARD:   SERIAL_ECHOPGM(STR_MOTHERBOARD);    break)
      OPTCODE(HAS_TEMP_SOC,     case H_SOC:     SERIAL_ECHOPGM(STR_SOC);            break)
      OPTCODE(HAS_TEMP_CHAMBER, case H_CHAMBER: SERIAL_ECHOPGM(STR_HEATER_CHAMBER); break)
      OPTCODE(HAS_TEMP_BED,     case H_BED:     SERIAL_ECHOPGM(STR_HEATER_BED);     break)
      default:
        if (real_heater_id >= 0) SERIAL_ECHO(C('E'), real_heater_id);
    }
    #if ENABLED(ERR_INCLUDE_TEMP)
      SERIAL_ECHOLNPGM(STR_DETECTED_TEMP_B, deg, STR_DETECTED_TEMP_E);
    #else
      SERIAL_EOL();
    #endif
  }

  disable_all_heaters(); // always disable (even for bogus temp)
  hal.watchdog_refresh();

  #if HAS_BOGUS_TEMPERATURE_GRACE_PERIOD

    // During boot the temperature may be unreliable, so when killed ==
    //  0: Set the expire time
    //  1: Check the expiration time has elapsed
    //  2: Kill the machine
    // >2: Do nothing
    const millis_t ms = millis();
    static millis_t expire_ms;
    switch (killed) {
      case 0:
        expire_ms = ms + BOGUS_TEMPERATURE_GRACE_PERIOD;
        ++killed;
        break;
      case 1:
        if (ELAPSED(ms, expire_ms)) ++killed;
        break;
      case 2:
        loud_kill(lcd_msg, heater_id);
        ++killed;
        break;
    }

  #else

    if (!killed) { killed = 1; loud_kill(lcd_msg, heater_id); }

  #endif
}

/**
 * Max Temp Error
 * @brief - The temperature reading is out of range, e.g., too high.
 *          May be caused by a disconnected thermistor which has infinite resistance.
 * @param heater_id:  The heater that caused the error
 * @param deg:  The detected temperature (if ERR_INCLUDE_TEMP)
 */
void Temperature::maxtemp_error(const heater_id_t heater_id OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg)) {
  #if HAS_HOTEND || HAS_HEATED_BED
    TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillBadTemp_L, ID_KillBadTemp_D));
    TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(1));
    TERN_(EXTENSIBLE_UI, ExtUI::onMaxTempError(heater_id));
  #endif
  _TEMP_ERROR(heater_id, F(STR_T_MAXTEMP), MSG_ERR_MAXTEMP, deg);
}

/**
 * Min Temp Error
 * @brief - The temperature reading is out of range, e.g., too high.
 *          May be caused by a shorted thermistor which has near zero resistance.
 * @param heater_id:  The heater that caused the error
 * @param deg:  The detected temperature (if ERR_INCLUDE_TEMP)
 */
void Temperature::mintemp_error(const heater_id_t heater_id OPTARG(ERR_INCLUDE_TEMP, const celsius_float_t deg)) {
  #if HAS_HOTEND || HAS_HEATED_BED
    TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillBadTemp_L, ID_KillBadTemp_D));
    TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
    TERN_(EXTENSIBLE_UI, ExtUI::onMinTempError(heater_id));
  #endif
  _TEMP_ERROR(heater_id, F(STR_T_MINTEMP), MSG_ERR_MINTEMP, deg);
}

#if HAS_PID_DEBUG
  bool Temperature::pid_debug_flag; // = false
#endif

#if HAS_PID_HEATING

  template<typename TT>
  class PIDRunner {
  public:
    TT &tempinfo;

    PIDRunner(TT &t) : tempinfo(t) { }

    float get_pid_output(const uint8_t extr=0) {
      #if ENABLED(PID_OPENLOOP)

        return constrain(tempinfo.target, 0, MAX_POW);

      #else // !PID_OPENLOOP

        float out = tempinfo.pid.get_pid_output(tempinfo.target, tempinfo.celsius);

        #if ENABLED(PID_FAN_SCALING)
          out += tempinfo.pid.get_fan_scale_output(thermalManager.fan_speed[extr]);
        #endif

        #if ENABLED(PID_EXTRUSION_SCALING)
          out += tempinfo.pid.get_extrusion_scale_output(
            extr == active_extruder, stepper.position(E_AXIS), planner.mm_per_step[E_AXIS], thermalManager.lpq_len
          );
        #endif

        return constrain(out, tempinfo.pid.low(), tempinfo.pid.high());

      #endif // !PID_OPENLOOP
    }

    FORCE_INLINE void debug(const_celsius_float_t c, const_float_t pid_out, FSTR_P const name=nullptr, const int8_t index=-1) {
      if (TERN0(HAS_PID_DEBUG, thermalManager.pid_debug_flag)) {
        SERIAL_ECHO_START();
        if (name) SERIAL_ECHO(name);
        if (index >= 0) SERIAL_ECHO(index);
        SERIAL_ECHOLNPGM(
          STR_PID_DEBUG_INPUT, c,
          STR_PID_DEBUG_OUTPUT, pid_out
          #if DISABLED(PID_OPENLOOP)
            , " pTerm ", tempinfo.pid.pTerm(), " iTerm ", tempinfo.pid.iTerm(), " dTerm ", tempinfo.pid.dTerm()
            , " cTerm ", tempinfo.pid.cTerm(), " fTerm ", tempinfo.pid.fTerm()
          #endif
        );
      }
    }
  };

#endif // HAS_PID_HEATING

#if HAS_HOTEND

  /**
   * PID Output Hotend
   * @brief Calculate the power output for the hotend (using PID or MPC)
   *        that is required to get closer to the target temperature.
   * @param E_NAME:  The extruder index
   * @return The power output for the hotend
   */
  float Temperature::get_pid_output_hotend(const uint8_t E_NAME) {
    const uint8_t ee = HOTEND_INDEX;

    const bool is_idling = TERN0(HEATER_IDLE_HANDLER, heater_idle[ee].timed_out);

    #if ENABLED(PIDTEMP)

      typedef PIDRunner<hotend_info_t> PIDRunnerHotend;

      static PIDRunnerHotend hotend_pid[HOTENDS] = {
        #define _HOTENDPID(E) temp_hotend[E],
        REPEAT(HOTENDS, _HOTENDPID)
      };

      const float pid_output = is_idling ? 0 : hotend_pid[ee].get_pid_output(ee);

      #if ENABLED(PID_DEBUG)
        if (ee == active_extruder)
          hotend_pid[ee].debug(temp_hotend[ee].celsius, pid_output, F("E"), ee);
      #endif

    #elif ENABLED(MPCTEMP)

      MPCHeaterInfo &hotend = temp_hotend[ee];
      MPC_t &mpc = hotend.mpc;

      // At startup, initialize modeled temperatures
      if (isnan(hotend.modeled_block_temp)) {
        hotend.modeled_ambient_temp = _MIN(30.0f, hotend.celsius);   // Cap initial value at reasonable max room temperature of 30C
        hotend.modeled_block_temp = hotend.modeled_sensor_temp = hotend.celsius;
      }

      #if HOTENDS == 1
        constexpr bool this_hotend = true;
      #else
        const bool this_hotend = (ee == active_extruder);
      #endif

      float ambient_xfer_coeff = mpc.ambient_xfer_coeff_fan0;
      #if ENABLED(MPC_INCLUDE_FAN)
        const uint8_t fan_index = TERN(SINGLEFAN, 0, ee);
        const float fan_fraction = TERN_(MPC_FAN_0_ACTIVE_HOTEND, !this_hotend ? 0.0f : ) fan_speed[fan_index] * RECIPROCAL(255);
        ambient_xfer_coeff += fan_fraction * mpc.fan255_adjustment;
      #endif

      if (this_hotend) {
        const int32_t e_position = stepper.position(E_AXIS);
        const float e_speed = (e_position - MPC::e_position) * planner.mm_per_step[E_AXIS] / MPC_dT;

        // The position can appear to make big jumps when, e.g., homing
        if (fabs(e_speed) > planner.settings.max_feedrate_mm_s[E_AXIS])
          MPC::e_position = e_position;
        else if (e_speed > 0.0f) {  // Ignore retract/recover moves
          if (!MPC::e_paused) ambient_xfer_coeff += e_speed * mpc.filament_heat_capacity_permm;
          MPC::e_position = e_position;
        }
      }

      // Update the modeled temperatures
      float blocktempdelta = hotend.soft_pwm_amount * mpc.heater_power * (MPC_dT / 127) / mpc.block_heat_capacity;
      blocktempdelta += (hotend.modeled_ambient_temp - hotend.modeled_block_temp) * ambient_xfer_coeff * MPC_dT / mpc.block_heat_capacity;
      hotend.modeled_block_temp += blocktempdelta;

      const float sensortempdelta = (hotend.modeled_block_temp - hotend.modeled_sensor_temp) * (mpc.sensor_responsiveness * MPC_dT);
      hotend.modeled_sensor_temp += sensortempdelta;

      // Any delta between hotend.modeled_sensor_temp and hotend.celsius is either model
      // error diverging slowly or (fast) noise. Slowly correct towards this temperature and noise will average out.
      const float delta_to_apply = (hotend.celsius - hotend.modeled_sensor_temp) * (MPC_SMOOTHING_FACTOR);
      hotend.modeled_block_temp += delta_to_apply;
      hotend.modeled_sensor_temp += delta_to_apply;

      // Only correct ambient when close to steady state (output power is not clipped or asymptotic temperature is reached)
      if (WITHIN(hotend.soft_pwm_amount, 1, 126) || fabs(blocktempdelta + delta_to_apply) < (MPC_STEADYSTATE * MPC_dT))
        hotend.modeled_ambient_temp += delta_to_apply > 0.f ? _MAX(delta_to_apply, MPC_MIN_AMBIENT_CHANGE * MPC_dT) : _MIN(delta_to_apply, -MPC_MIN_AMBIENT_CHANGE * MPC_dT);

      float power = 0.0;
      if (hotend.target != 0 && !is_idling) {
        // Plan power level to get to target temperature in 2 seconds
        power = (hotend.target - hotend.modeled_block_temp) * mpc.block_heat_capacity / 2.0f;
        power -= (hotend.modeled_ambient_temp - hotend.modeled_block_temp) * ambient_xfer_coeff;
      }

      float pid_output = power * 254.0f / mpc.heater_power + 1.0f;        // Ensure correct quantization into a range of 0 to 127
      LIMIT(pid_output, 0, MPC_MAX);

      /* <-- add a slash to enable
        static uint32_t nexttime = millis() + 1000;
        if (ELAPSED(millis(), nexttime)) {
          nexttime += 1000;
          SERIAL_ECHOLNPGM("block temp ", hotend.modeled_block_temp,
                           ", celsius ", hotend.celsius,
                           ", blocktempdelta ", blocktempdelta,
                           ", delta_to_apply ", delta_to_apply,
                           ", ambient ", hotend.modeled_ambient_temp,
                           ", power ", power,
                           ", pid_output ", pid_output,
                           ", pwm ", (int)pid_output >> 1);
        }
      //*/

    #else // No PID or MPC enabled

      const float pid_output = (!is_idling && temp_hotend[ee].is_below_target()) ? BANG_MAX : 0;

    #endif

    return pid_output;
  }

#endif // HAS_HOTEND

#if ENABLED(PIDTEMPBED)

  /**
   * PID Output Bed
   * @brief Calculate the bed power output using PID that is required
   *        to get closer to the target temperature.
   * @return The power output for the bed
   */
  float Temperature::get_pid_output_bed() {
    static PIDRunner<bed_info_t> bed_pid(temp_bed);
    const float pid_output = bed_pid.get_pid_output();
    TERN_(PID_BED_DEBUG, bed_pid.debug(temp_bed.celsius, pid_output, F("(Bed)")));
    return pid_output;
  }

#endif // PIDTEMPBED

#if ENABLED(PIDTEMPCHAMBER)

  /**
   * PID Output Chamber
   * @brief Calculate the chamber power output using PID that is required
   *        to get closer to the target temperature.
   * @return The power output for the chamber
   */
  float Temperature::get_pid_output_chamber() {
    static PIDRunner<chamber_info_t> chamber_pid(temp_chamber);
    const float pid_output = chamber_pid.get_pid_output();
    TERN_(PID_CHAMBER_DEBUG, chamber_pid.debug(temp_chamber.celsius, pid_output, F("(Chamber)")));
    return pid_output;
  }

#endif // PIDTEMPCHAMBER

#if HAS_HOTEND

  /**
   * Manage Hotend Temperatures
   * @brief Task for managing hotends, called from Temperature::task()
   * @param ms Current Time
   */
  void Temperature::manage_hotends(const millis_t &ms) {
    HOTEND_LOOP() {
      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
      {
        const auto deg = degHotend(e);
        if (deg > temp_range[e].maxtemp) {
          TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillBadTemp_L, ID_KillBadTemp_D));
          MAXTEMP_ERROR(e, deg);
        }
      }
      #endif

      TERN_(HEATER_IDLE_HANDLER, heater_idle[e].update(ms));

      #if ENABLED(THERMAL_PROTECTION_HOTENDS)
        // Check for thermal runaway
        tr_state_machine[e].run(temp_hotend[e].celsius, temp_hotend[e].target, (heater_id_t)e, THERMAL_PROTECTION_PERIOD, THERMAL_PROTECTION_HYSTERESIS);
      #endif

      temp_hotend[e].soft_pwm_amount = (temp_hotend[e].celsius > temp_range[e].mintemp || is_hotend_preheating(e))
        && temp_hotend[e].celsius < temp_range[e].maxtemp ? (int)get_pid_output_hotend(e) >> 1 : 0;

      #if WATCH_HOTENDS
        // Make sure temperature is increasing
        if (watch_hotend[e].elapsed(ms)) {          // Enabled and time to check?
          auto temp = degHotend(e);
          if (watch_hotend[e].check(temp))          // Increased enough?
            start_watching_hotend(e);               // If temp reached, turn off elapsed check
          else {
            TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillHeat_L, ID_KillHeat_D));
            TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
            TERN_(EXTENSIBLE_UI, ExtUI::onHeatingError(e));
            _TEMP_ERROR(e, FPSTR(str_t_heating_failed), MSG_ERR_HEATING_FAILED, temp);
          }
        }
      #endif

    } // HOTEND_LOOP
  }

#endif // HAS_HOTEND

#if HAS_HEATED_BED

  /**
   * Manage Heated Bed Temperature
   * @brief Task for managing the heated bed, called from Temperature::task()
   * @param ms Current Time
   */
  void Temperature::manage_heated_bed(const millis_t &ms) {

    #if ENABLED(THERMAL_PROTECTION_BED)
    {
      const auto deg = degBed();
      if (deg > BED_MAXTEMP) {
        TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillBadTemp_L, ID_KillBadTemp_D));
        MAXTEMP_ERROR(H_BED, deg);
      }
    }
    #endif

    #if WATCH_BED
    {
      // Make sure temperature is increasing
      if (watch_bed.elapsed(ms)) {              // Time to check the bed?
        const auto deg = degBed();
        if (watch_bed.check(deg))               // Increased enough?
          start_watching_bed();                 // If temp reached, turn off elapsed check
        else {
          TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillHeat_L, ID_KillHeat_D));
          TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
          TERN_(EXTENSIBLE_UI, ExtUI::onHeatingError(H_BED));
          _TEMP_ERROR(H_BED, FPSTR(str_t_heating_failed), MSG_ERR_HEATING_FAILED, deg);
        }
      }
    }
    #endif // WATCH_BED

    #if ALL(PROBING_HEATERS_OFF, BED_LIMIT_SWITCHING)
      #define PAUSE_CHANGE_REQD 1
    #endif

    #if PAUSE_CHANGE_REQD
      static bool last_pause_state;
    #endif

    do { // 'break' out of this block

      #if DISABLED(PIDTEMPBED)
        if (PENDING(ms, next_bed_check_ms)
          && TERN1(PAUSE_CHANGE_REQD, paused_for_probing == last_pause_state)
        ) break;
        next_bed_check_ms = ms + BED_CHECK_INTERVAL;
        TERN_(PAUSE_CHANGE_REQD, last_pause_state = paused_for_probing);
      #endif

      TERN_(HEATER_IDLE_HANDLER, heater_idle[IDLE_INDEX_BED].update(ms));

      #if ENABLED(THERMAL_PROTECTION_BED)
        tr_state_machine[RUNAWAY_IND_BED].run(temp_bed.celsius, temp_bed.target, H_BED, THERMAL_PROTECTION_BED_PERIOD, THERMAL_PROTECTION_BED_HYSTERESIS);
      #endif

      #if HEATER_IDLE_HANDLER
        const bool bed_timed_out = heater_idle[IDLE_INDEX_BED].timed_out;
        if (bed_timed_out) {
          temp_bed.soft_pwm_amount = 0;
          if (DISABLED(PIDTEMPBED)) WRITE_HEATER_BED(LOW);
        }
      #else
        constexpr bool bed_timed_out = false;
      #endif

      if (bed_timed_out) break;

      if (is_bed_preheating()) {
        temp_bed.soft_pwm_amount = MAX_BED_POWER >> 1;
        break;
      }

      #if ENABLED(PIDTEMPBED)

        //
        // PID Bed Heating
        //
        temp_bed.soft_pwm_amount = WITHIN(temp_bed.celsius, BED_MINTEMP, BED_MAXTEMP) ? (int)get_pid_output_bed() >> 1 : 0;

      #else // !PIDTEMPBED

        //
        // Range-limited "bang-bang" bed heating
        //

        // Bed Off if the current bed temperature is outside the allowed range
        if (!WITHIN(temp_bed.celsius, BED_MINTEMP, BED_MAXTEMP)) {
          temp_bed.soft_pwm_amount = 0;
          WRITE_HEATER_BED(LOW);
          break;
        }

        #if ENABLED(PELTIER_BED)
          /**
           * Peltier bang-bang maintains max bed power but changes
           * current direction to switch between heating/cooling.
           */
          if (temp_bed.target && temp_bed.is_above_target(BED_HYSTERESIS)) {  // Fast Cooling
            temp_bed.soft_pwm_amount = MAX_BED_POWER;
            temp_bed.peltier_dir_heating = false;
          }
          else if (temp_bed.is_below_target(BED_HYSTERESIS)) {                // Heating
            temp_bed.soft_pwm_amount = MAX_BED_POWER;
            temp_bed.peltier_dir_heating = true;
          }
          else
            temp_bed.soft_pwm_amount = 0;                                     // Off (ambient cooling)

        #else // !PELTIER_BED

          #if ENABLED(BED_LIMIT_SWITCHING)
            if (temp_bed.is_above_target(BED_HYSTERESIS))       // Cooling (implicit off)
              temp_bed.soft_pwm_amount = 0;
            else if (temp_bed.is_below_target(BED_HYSTERESIS))  // Heating
              temp_bed.soft_pwm_amount = MAX_BED_POWER >> 1;
          #else                                                 // Not bed limit switching
            temp_bed.soft_pwm_amount = temp_bed.is_below_target() ? MAX_BED_POWER >> 1 : 0;
          #endif

        #endif // !PELTIER_BED

      #endif // !PIDTEMPBED

    } while (false);
  }

#endif // HAS_HEATED_BED

#if HAS_HEATED_CHAMBER

  /**
   * Manage Heated Chamber Temperature
   * @brief Task for managing the heated chamber, called from Temperature::task()
   * @param ms Current Time
   */
  void Temperature::manage_heated_chamber(const millis_t &ms) {

    #ifndef CHAMBER_CHECK_INTERVAL
      #define CHAMBER_CHECK_INTERVAL 1000UL
    #endif

    #if ENABLED(THERMAL_PROTECTION_CHAMBER)
    {
      const auto deg = degChamber();
      if (deg > CHAMBER_MAXTEMP) MAXTEMP_ERROR(H_CHAMBER, deg);
    }
    #endif

    #if WATCH_CHAMBER
    {
      // Make sure temperature is increasing
      if (watch_chamber.elapsed(ms)) {          // Time to check the chamber?
        const auto deg = degChamber();
        if (watch_chamber.check(deg))           // Increased enough? Error below.
          start_watching_chamber();             // If temp reached, turn off elapsed check.
        else
          _TEMP_ERROR(H_CHAMBER, FPSTR(str_t_heating_failed), MSG_ERR_HEATING_FAILED, deg);
      }
    }
    #endif

    #if ANY(CHAMBER_FAN, CHAMBER_VENT) || DISABLED(PIDTEMPCHAMBER)
      static bool flag_chamber_excess_heat; // = false
    #endif

    #if ANY(CHAMBER_FAN, CHAMBER_VENT)
      static bool flag_chamber_off; // = false

      if (temp_chamber.target > CHAMBER_MINTEMP) {
        flag_chamber_off = false;

        #if ENABLED(CHAMBER_FAN)
          int16_t fan_chamber_pwm;
          #if CHAMBER_FAN_MODE == 0
            fan_chamber_pwm = CHAMBER_FAN_BASE;
          #elif CHAMBER_FAN_MODE == 1
            fan_chamber_pwm = temp_chamber.is_above_target() ? (CHAMBER_FAN_BASE) + (CHAMBER_FAN_FACTOR) * (temp_chamber.celsius - temp_chamber.target) : 0;
          #elif CHAMBER_FAN_MODE == 2
            fan_chamber_pwm = (CHAMBER_FAN_BASE) + (CHAMBER_FAN_FACTOR) * ABS(temp_chamber.celsius - temp_chamber.target);
            if (temp_chamber.soft_pwm_amount) fan_chamber_pwm += (CHAMBER_FAN_FACTOR) * 2;
          #elif CHAMBER_FAN_MODE == 3
            fan_chamber_pwm = (CHAMBER_FAN_BASE) + _MAX((CHAMBER_FAN_FACTOR) * (temp_chamber.celsius - temp_chamber.target), 0);
          #endif
          NOMORE(fan_chamber_pwm, 255);
          set_fan_speed(CHAMBER_FAN_INDEX, fan_chamber_pwm);
        #endif

        #if ENABLED(CHAMBER_VENT)
          #ifndef MIN_COOLING_SLOPE_TIME_CHAMBER_VENT
            #define MIN_COOLING_SLOPE_TIME_CHAMBER_VENT 20
          #endif
          #ifndef MIN_COOLING_SLOPE_DEG_CHAMBER_VENT
            #define MIN_COOLING_SLOPE_DEG_CHAMBER_VENT 1.5
          #endif
          if (!flag_chamber_excess_heat && temp_chamber.is_above_target(HIGH_EXCESS_HEAT_LIMIT)) {
            // Open vent after MIN_COOLING_SLOPE_TIME_CHAMBER_VENT seconds if the
            // temperature didn't drop at least MIN_COOLING_SLOPE_DEG_CHAMBER_VENT
            if (next_cool_check_ms == 0 || ELAPSED(ms, next_cool_check_ms)) {
              if (temp_chamber.celsius - old_temp > MIN_COOLING_SLOPE_DEG_CHAMBER_VENT)
                flag_chamber_excess_heat = true; // the bed is heating the chamber too much
              next_cool_check_ms = ms + SEC_TO_MS(MIN_COOLING_SLOPE_TIME_CHAMBER_VENT);
              old_temp = temp_chamber.celsius;
            }
          }
          else {
            next_cool_check_ms = 0;
            old_temp = 9999;
          }
          if (flag_chamber_excess_heat && temp_chamber.is_above_target(LOW_EXCESS_HEAT_LIMIT))
            flag_chamber_excess_heat = false;
        #endif
      }
      else if (!flag_chamber_off) {
        #if ENABLED(CHAMBER_FAN)
          flag_chamber_off = true;
          set_fan_speed(CHAMBER_FAN_INDEX, 0);
        #endif
        #if ENABLED(CHAMBER_VENT)
          flag_chamber_excess_heat = false;
          servo[CHAMBER_VENT_SERVO_NR].move(90);
        #endif
      }
    #endif

    #if ENABLED(PIDTEMPCHAMBER)
      // PIDTEMPCHAMBER doesn't support a CHAMBER_VENT yet.
      temp_chamber.soft_pwm_amount = WITHIN(temp_chamber.celsius, CHAMBER_MINTEMP, CHAMBER_MAXTEMP) ? (int)get_pid_output_chamber() >> 1 : 0;
    #else
      if (ELAPSED(ms, next_chamber_check_ms)) {
        next_chamber_check_ms = ms + CHAMBER_CHECK_INTERVAL;

        if (WITHIN(temp_chamber.celsius, CHAMBER_MINTEMP, CHAMBER_MAXTEMP)) {
          if (flag_chamber_excess_heat) {
            temp_chamber.soft_pwm_amount = 0;
            #if ENABLED(CHAMBER_VENT)
              if (!flag_chamber_off) servo[CHAMBER_VENT_SERVO_NR].move(temp_chamber.is_below_target() ? 0 : 90);
            #endif
          }
          else {
            #if ENABLED(CHAMBER_LIMIT_SWITCHING)
              if (temp_chamber.is_above_target(TEMP_CHAMBER_HYSTERESIS))
                temp_chamber.soft_pwm_amount = 0;
              else if (temp_chamber.is_below_target(TEMP_CHAMBER_HYSTERESIS))
                temp_chamber.soft_pwm_amount = (MAX_CHAMBER_POWER) >> 1;
            #else
              temp_chamber.soft_pwm_amount = temp_chamber.is_below_target() ? (MAX_CHAMBER_POWER) >> 1 : 0;
            #endif
            #if ENABLED(CHAMBER_VENT)
              if (!flag_chamber_off) servo[CHAMBER_VENT_SERVO_NR].move(0);
            #endif
          }
        }
        else {
          temp_chamber.soft_pwm_amount = 0;
          WRITE_HEATER_CHAMBER(LOW);
        }
     }
     #if ENABLED(THERMAL_PROTECTION_CHAMBER)
       tr_state_machine[RUNAWAY_IND_CHAMBER].run(temp_chamber.celsius, temp_chamber.target, H_CHAMBER, THERMAL_PROTECTION_CHAMBER_PERIOD, THERMAL_PROTECTION_CHAMBER_HYSTERESIS);
     #endif
   #endif
  }

#endif // HAS_HEATED_CHAMBER

#if HAS_COOLER

  /**
   * Manage Heated Cooler Temperature
   * @brief Task for managing the cooler, called from Temperature::task()
   * @param ms Current Time
   */
  void Temperature::manage_cooler(const millis_t &ms) {

    #ifndef COOLER_CHECK_INTERVAL
      #define COOLER_CHECK_INTERVAL 2000UL
    #endif

    #if ENABLED(THERMAL_PROTECTION_COOLER)
    {
      const auto deg = degCooler();
      if (deg > COOLER_MAXTEMP) MAXTEMP_ERROR(H_COOLER, deg);
    }
    #endif

    #if WATCH_COOLER
      // Make sure temperature is decreasing
      if (watch_cooler.elapsed(ms)) {             // Time to check the cooler?
        const auto deg = degCooler();
        if (deg > watch_cooler.target)            // Failed to decrease enough?
          _TEMP_ERROR(H_COOLER, GET_TEXT_F(MSG_ERR_COOLING_FAILED), MSG_ERR_COOLING_FAILED, deg);
        else
          start_watching_cooler();                // Start again if the target is still far off
      }
    #endif

    static bool flag_cooler_state; // = false

    if (cooler.enabled) {
      flag_cooler_state = true; // used to allow M106 fan control when cooler is disabled
      if (temp_cooler.target == 0) temp_cooler.target = COOLER_MIN_TARGET;
      if (ELAPSED(ms, next_cooler_check_ms)) {
        next_cooler_check_ms = ms + COOLER_CHECK_INTERVAL;
        if (temp_cooler.is_above_target()) { // too warm?
          temp_cooler.soft_pwm_amount = MAX_COOLER_POWER;
          #if ENABLED(COOLER_FAN)
            const int16_t fan_cooler_pwm = (COOLER_FAN_BASE) + (COOLER_FAN_FACTOR) * ABS(temp_cooler.celsius - temp_cooler.target);
            set_fan_speed(COOLER_FAN_INDEX, _MIN(fan_cooler_pwm, 255)); // Set cooler fan pwm
            cooler_fan_flush_ms = ms + 5000;
          #endif
        }
        else {
          temp_cooler.soft_pwm_amount = 0;
          #if ENABLED(COOLER_FAN)
            set_fan_speed(COOLER_FAN_INDEX, temp_cooler.is_above_target(-2) ? COOLER_FAN_BASE : 0);
          #endif
          WRITE_HEATER_COOLER(LOW);
        }
      }
    }
    else {
      temp_cooler.soft_pwm_amount = 0;
      if (flag_cooler_state) {
        flag_cooler_state = false;
        thermalManager.set_fan_speed(COOLER_FAN_INDEX, 0);
      }
      WRITE_HEATER_COOLER(LOW);
    }

    #if ENABLED(THERMAL_PROTECTION_COOLER)
      tr_state_machine[RUNAWAY_IND_COOLER].run(temp_cooler.celsius, temp_cooler.target, H_COOLER, THERMAL_PROTECTION_COOLER_PERIOD, THERMAL_PROTECTION_COOLER_HYSTERESIS);
    #endif
  }

#endif // HAS_COOLER

/**
 * Manage heating activities for extruder hot-ends and a heated bed
 *  - Acquire updated temperature readings
 *    - Also resets the watchdog timer
 *  - Invoke thermal runaway protection
 *  - Manage extruder auto-fan
 *  - Apply filament width to the extrusion rate (may move)
 *  - Update the heated bed PID output value
 */
void Temperature::task() {
  if (marlin_state == MarlinState::MF_INITIALIZING) return hal.watchdog_refresh(); // If Marlin isn't started, at least reset the watchdog!

  static bool no_reentry = false;  // Prevent recursion
  if (no_reentry) return;
  REMEMBER(mh, no_reentry, true);

  #if ENABLED(EMERGENCY_PARSER)
    if (emergency_parser.killed_by_M112) kill(FPSTR(M112_KILL_STR), nullptr, true);

    if (emergency_parser.quickstop_by_M410) {
      emergency_parser.quickstop_by_M410 = false; // quickstop_stepper may call idle so clear this now!
      quickstop_stepper();
    }

    #if HAS_MEDIA
      if (emergency_parser.sd_abort_by_M524) { // abort SD print immediately
        emergency_parser.sd_abort_by_M524 = false;
        card.flag.abort_sd_printing = true;
        gcode.process_subcommands_now(F("M524"));
      }
    #endif
  #endif

  if (!updateTemperaturesIfReady()) return; // Will also reset the watchdog if temperatures are ready

  #if DISABLED(IGNORE_THERMOCOUPLE_ERRORS)
    #if TEMP_SENSOR_IS_MAX_TC(0)
    {
      const auto deg = degHotend(0);
      if (deg > _MIN(HEATER_0_MAXTEMP, TEMP_SENSOR_0_MAX_TC_TMAX - 1.00f)) MAXTEMP_ERROR(H_E0, deg);
      if (deg < _MAX(HEATER_0_MINTEMP, TEMP_SENSOR_0_MAX_TC_TMIN + 0.01f)) MINTEMP_ERROR(H_E0, deg);
    }
    #endif
    #if TEMP_SENSOR_IS_MAX_TC(1)
    {
      const auto deg = degHotend(1);
      if (deg > _MIN(HEATER_1_MAXTEMP, TEMP_SENSOR_1_MAX_TC_TMAX - 1.00f)) MAXTEMP_ERROR(H_E1, deg);
      if (deg < _MAX(HEATER_1_MINTEMP, TEMP_SENSOR_1_MAX_TC_TMIN + 0.01f)) MINTEMP_ERROR(H_E1, deg);
    }
    #endif
    #if TEMP_SENSOR_IS_MAX_TC(2)
    {
      const auto deg = degHotend(2);
      if (deg > _MIN(HEATER_2_MAXTEMP, TEMP_SENSOR_2_MAX_TC_TMAX - 1.00f)) MAXTEMP_ERROR(H_E2, deg);
      if (deg < _MAX(HEATER_2_MINTEMP, TEMP_SENSOR_2_MAX_TC_TMIN + 0.01f)) MINTEMP_ERROR(H_E2, deg);
    }
    #endif
    #if TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
    {
      const auto deg = degRedundant();
      if (deg > TEMP_SENSOR_REDUNDANT_MAX_TC_TMAX - 1.00f) MAXTEMP_ERROR(H_REDUNDANT, deg);
      if (deg < TEMP_SENSOR_REDUNDANT_MAX_TC_TMIN + 0.01f) MINTEMP_ERROR(H_REDUNDANT, deg);
    }
    #endif
    #if TEMP_SENSOR_IS_MAX_TC(BED)
    {
      const auto deg = degBed();
      if (deg > _MIN(BED_MAXTEMP, TEMP_SENSOR_BED_MAX_TC_TMAX - 1.00f)) MAXTEMP_ERROR(H_BED, deg);
      if (deg < _MAX(BED_MINTEMP, TEMP_SENSOR_BED_MAX_TC_TMIN + 0.01f)) MINTEMP_ERROR(H_BED, deg);
    }
    #endif
  #endif

  const millis_t ms = millis();

  // Handle Hotend Temp Errors, Heating Watch, etc.
  TERN_(HAS_HOTEND, manage_hotends(ms));

  #if HAS_TEMP_REDUNDANT
  {
    const auto deg = degRedundant();
    // Make sure measured temperatures are close together
    if (ABS(degRedundantTarget() - deg) > TEMP_SENSOR_REDUNDANT_MAX_DIFF)
      _TEMP_ERROR(HEATER_ID(TEMP_SENSOR_REDUNDANT_TARGET), F(STR_REDUNDANCY), MSG_ERR_REDUNDANT_TEMP, deg);
  }
  #endif

  // Manage extruder auto fans and/or read fan tachometers
  TERN_(HAS_FAN_LOGIC, manage_extruder_fans(ms));

  /**
   * Dynamically set the volumetric multiplier based
   * on the delayed Filament Width measurement.
   */
  TERN_(FILAMENT_WIDTH_SENSOR, filwidth.update_volumetric());

  // Handle Bed Temp Errors, Heating Watch, etc.
  TERN_(HAS_HEATED_BED, manage_heated_bed(ms));

  // Handle Heated Chamber Temp Errors, Heating Watch, etc.
  TERN_(HAS_HEATED_CHAMBER, manage_heated_chamber(ms));

  // Handle Cooler Temp Errors, Cooling Watch, etc.
  TERN_(HAS_COOLER, manage_cooler(ms));

  #if ENABLED(LASER_COOLANT_FLOW_METER)
    cooler.flowmeter_task(ms);
    #if ENABLED(FLOWMETER_SAFETY)
      if (cooler.check_flow_too_low()) {
        TERN_(HAS_DISPLAY, if (cutter.enabled()) ui.flow_fault());
        cutter.disable();
        cutter.cutter_mode = CUTTER_MODE_ERROR;   // Immediately kill stepper inline power output
      }
    #endif
  #endif

  UNUSED(ms);
}

// For a 5V input the AD595 returns a value scaled with 10mV per °C. (Minimum input voltage is 5V.)
#define TEMP_AD595(RAW)  ((RAW) * (ADC_VREF_MV / 10) / float(HAL_ADC_RANGE) / (OVERSAMPLENR) * (TEMP_SENSOR_AD595_GAIN) + TEMP_SENSOR_AD595_OFFSET)
// For a 5V input the AD8495 returns a value scaled with 5mV per °C. (Minimum input voltage is 2.7V.)
#define TEMP_AD8495(RAW) ((RAW) * (ADC_VREF_MV /  5) / float(HAL_ADC_RANGE) / (OVERSAMPLENR) * (TEMP_SENSOR_AD8495_GAIN) + TEMP_SENSOR_AD8495_OFFSET)

/**
 * Bisect search for the range of the 'raw' value, then interpolate
 * proportionally between the under and over values.
 */
#define SCAN_THERMISTOR_TABLE(TBL,LEN) do{                                \
  uint8_t l = 0, r = LEN, m;                                              \
  for (;;) {                                                              \
    m = (l + r) >> 1;                                                     \
    if (!m) return celsius_t(pgm_read_word(&TBL[0].celsius));             \
    if (m == l || m == r) return celsius_t(pgm_read_word(&TBL[LEN-1].celsius)); \
    raw_adc_t v00 = pgm_read_word(&TBL[m-1].value),                       \
              v10 = pgm_read_word(&TBL[m-0].value);                       \
         if (raw < v00) r = m;                                            \
    else if (raw > v10) l = m;                                            \
    else {                                                                \
      const celsius_t v01 = celsius_t(pgm_read_word(&TBL[m-1].celsius)),  \
                      v11 = celsius_t(pgm_read_word(&TBL[m-0].celsius));  \
      return v01 + (raw - v00) * float(v11 - v01) / float(v10 - v00);     \
    }                                                                     \
  }                                                                       \
}while(0)

#if HAS_USER_THERMISTORS

  user_thermistor_t Temperature::user_thermistor[USER_THERMISTORS]; // Initialized by settings.load

  void Temperature::reset_user_thermistors() {
    user_thermistor_t default_user_thermistor[USER_THERMISTORS] = {
      #if TEMP_SENSOR_0_IS_CUSTOM
        { true, HOTEND0_SH_C_COEFF, 0, HOTEND0_PULLUP_RESISTOR_OHMS, HOTEND0_RESISTANCE_25C_OHMS, 0, 0, HOTEND0_BETA, 0 },
      #endif
      #if TEMP_SENSOR_1_IS_CUSTOM
        { true, HOTEND1_SH_C_COEFF, 0, HOTEND1_PULLUP_RESISTOR_OHMS, HOTEND1_RESISTANCE_25C_OHMS, 0, 0, HOTEND1_BETA, 0 },
      #endif
      #if TEMP_SENSOR_2_IS_CUSTOM
        { true, HOTEND2_SH_C_COEFF, 0, HOTEND2_PULLUP_RESISTOR_OHMS, HOTEND2_RESISTANCE_25C_OHMS, 0, 0, HOTEND2_BETA, 0 },
      #endif
      #if TEMP_SENSOR_3_IS_CUSTOM
        { true, HOTEND3_SH_C_COEFF, 0, HOTEND3_PULLUP_RESISTOR_OHMS, HOTEND3_RESISTANCE_25C_OHMS, 0, 0, HOTEND3_BETA, 0 },
      #endif
      #if TEMP_SENSOR_4_IS_CUSTOM
        { true, HOTEND4_SH_C_COEFF, 0, HOTEND4_PULLUP_RESISTOR_OHMS, HOTEND4_RESISTANCE_25C_OHMS, 0, 0, HOTEND4_BETA, 0 },
      #endif
      #if TEMP_SENSOR_5_IS_CUSTOM
        { true, HOTEND5_SH_C_COEFF, 0, HOTEND5_PULLUP_RESISTOR_OHMS, HOTEND5_RESISTANCE_25C_OHMS, 0, 0, HOTEND5_BETA, 0 },
      #endif
      #if TEMP_SENSOR_6_IS_CUSTOM
        { true, HOTEND6_SH_C_COEFF, 0, HOTEND6_PULLUP_RESISTOR_OHMS, HOTEND6_RESISTANCE_25C_OHMS, 0, 0, HOTEND6_BETA, 0 },
      #endif
      #if TEMP_SENSOR_7_IS_CUSTOM
        { true, HOTEND7_SH_C_COEFF, 0, HOTEND7_PULLUP_RESISTOR_OHMS, HOTEND7_RESISTANCE_25C_OHMS, 0, 0, HOTEND7_BETA, 0 },
      #endif
      #if TEMP_SENSOR_BED_IS_CUSTOM
        { true, BED_SH_C_COEFF, 0, BED_PULLUP_RESISTOR_OHMS, BED_RESISTANCE_25C_OHMS, 0, 0, BED_BETA, 0 },
      #endif
      #if TEMP_SENSOR_CHAMBER_IS_CUSTOM
        { true, CHAMBER_SH_C_COEFF, 0, CHAMBER_PULLUP_RESISTOR_OHMS, CHAMBER_RESISTANCE_25C_OHMS, 0, 0, CHAMBER_BETA, 0 },
      #endif
      #if TEMP_SENSOR_COOLER_IS_CUSTOM
        { true, COOLER_SH_C_COEFF, 0, COOLER_PULLUP_RESISTOR_OHMS, COOLER_RESISTANCE_25C_OHMS, 0, 0, COOLER_BETA, 0 },
      #endif
      #if TEMP_SENSOR_PROBE_IS_CUSTOM
        { true, PROBE_SH_C_COEFF, 0, PROBE_PULLUP_RESISTOR_OHMS, PROBE_RESISTANCE_25C_OHMS, 0, 0, PROBE_BETA, 0 },
      #endif
      #if TEMP_SENSOR_BOARD_IS_CUSTOM
        { true, BOARD_SH_C_COEFF, 0, BOARD_PULLUP_RESISTOR_OHMS, BOARD_RESISTANCE_25C_OHMS, 0, 0, BOARD_BETA, 0 },
      #endif
      #if TEMP_SENSOR_REDUNDANT_IS_CUSTOM
        { true, REDUNDANT_SH_C_COEFF, 0, REDUNDANT_PULLUP_RESISTOR_OHMS, REDUNDANT_RESISTANCE_25C_OHMS, 0, 0, REDUNDANT_BETA, 0 },
      #endif
    };
    COPY(user_thermistor, default_user_thermistor);
  }

  void Temperature::M305_report(const uint8_t t_index, const bool forReplay/*=true*/) {
    TERN_(MARLIN_SMALL_BUILD, return);

    gcode.report_heading_etc(forReplay, F(STR_USER_THERMISTORS));
    SERIAL_ECHOPGM("  M305 P", AS_DIGIT(t_index));

    const user_thermistor_t &t = user_thermistor[t_index];

    SERIAL_ECHO(
      F(" R"), p_float_t(t.series_res, 1), FPSTR(SP_T_STR), p_float_t(t.res_25, 1),
      FPSTR(SP_B_STR), p_float_t(t.beta, 1), FPSTR(SP_C_STR), p_float_t(t.sh_c_coeff, 9),
      F(" ; ")
    );
    SERIAL_ECHOLN(
      TERN_(TEMP_SENSOR_0_IS_CUSTOM, t_index == CTI_HOTEND_0 ? F("HOTEND 0") :)
      TERN_(TEMP_SENSOR_1_IS_CUSTOM, t_index == CTI_HOTEND_1 ? F("HOTEND 1") :)
      TERN_(TEMP_SENSOR_2_IS_CUSTOM, t_index == CTI_HOTEND_2 ? F("HOTEND 2") :)
      TERN_(TEMP_SENSOR_3_IS_CUSTOM, t_index == CTI_HOTEND_3 ? F("HOTEND 3") :)
      TERN_(TEMP_SENSOR_4_IS_CUSTOM, t_index == CTI_HOTEND_4 ? F("HOTEND 4") :)
      TERN_(TEMP_SENSOR_5_IS_CUSTOM, t_index == CTI_HOTEND_5 ? F("HOTEND 5") :)
      TERN_(TEMP_SENSOR_6_IS_CUSTOM, t_index == CTI_HOTEND_6 ? F("HOTEND 6") :)
      TERN_(TEMP_SENSOR_7_IS_CUSTOM, t_index == CTI_HOTEND_7 ? F("HOTEND 7") :)
      TERN_(TEMP_SENSOR_BED_IS_CUSTOM, t_index == CTI_BED ? F("BED") :)
      TERN_(TEMP_SENSOR_CHAMBER_IS_CUSTOM, t_index == CTI_CHAMBER ? F("CHAMBER") :)
      TERN_(TEMP_SENSOR_COOLER_IS_CUSTOM, t_index == CTI_COOLER ? F("COOLER") :)
      TERN_(TEMP_SENSOR_PROBE_IS_CUSTOM, t_index == CTI_PROBE ? F("PROBE") :)
      TERN_(TEMP_SENSOR_BOARD_IS_CUSTOM, t_index == CTI_BOARD ? F("BOARD") :)
      TERN_(TEMP_SENSOR_REDUNDANT_IS_CUSTOM, t_index == CTI_REDUNDANT ? F("REDUNDANT") :)
      FSTR_P(nullptr)
    );
  }

  celsius_float_t Temperature::user_thermistor_to_deg_c(const uint8_t t_index, const raw_adc_t raw) {

    if (!WITHIN(t_index, 0, COUNT(user_thermistor) - 1)) return 25;

    user_thermistor_t &t = user_thermistor[t_index];
    if (t.pre_calc) { // pre-calculate some variables
      t.pre_calc     = false;
      t.res_25_recip = 1.0f / t.res_25;
      t.res_25_log   = logf(t.res_25);
      t.beta_recip   = 1.0f / t.beta;
      t.sh_alpha     = RECIPROCAL(THERMISTOR_RESISTANCE_NOMINAL_C - (THERMISTOR_ABS_ZERO_C))
                        - (t.beta_recip * t.res_25_log) - (t.sh_c_coeff * cu(t.res_25_log));
    }

    // Maximum ADC value .. take into account the over sampling
    constexpr raw_adc_t adc_max = MAX_RAW_THERMISTOR_VALUE;
    const raw_adc_t adc_raw = constrain(raw, 1, adc_max - 1); // constrain to prevent divide-by-zero

    const float adc_inverse = (adc_max - adc_raw) - 0.5f,
                resistance = t.series_res * (adc_raw + 0.5f) / adc_inverse,
                log_resistance = logf(resistance);

    float value = t.sh_alpha;
    value += log_resistance * t.beta_recip;
    if (t.sh_c_coeff != 0)
      value += t.sh_c_coeff * cu(log_resistance);
    value = 1.0f / value;

    // Return degrees C (up to 999, as the LCD only displays 3 digits)
    return _MIN(value + THERMISTOR_ABS_ZERO_C, 999);
  }
#endif

#if HAS_HOTEND
  // Derived from RepRap FiveD extruder::getTemperature()
  // For hot end temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_hotend(const raw_adc_t raw, const uint8_t e) {
    if (e >= HOTENDS) {
      SERIAL_ERROR_START();
      SERIAL_ECHO(e);
      SERIAL_ECHOLNPGM(STR_INVALID_EXTRUDER_NUM);
      kill();
      return 0;
    }

    switch (e) {
      case 0:
        #if TEMP_SENSOR_0_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_0, raw);
        #elif TEMP_SENSOR_IS_MAX_TC(0)
          #if TEMP_SENSOR_0_IS_MAX31865
            return TERN(LIB_INTERNAL_MAX31865,
              max31865_0.temperature(raw),
              max31865_0.temperature(MAX31865_SENSOR_OHMS_0, MAX31865_CALIBRATION_OHMS_0)
            );
          #else
            return (int16_t)raw * 0.25f;
          #endif
        #elif TEMP_SENSOR_0_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_0_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 1:
        #if TEMP_SENSOR_1_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_1, raw);
        #elif TEMP_SENSOR_IS_MAX_TC(1)
          #if TEMP_SENSOR_0_IS_MAX31865
            return TERN(LIB_INTERNAL_MAX31865,
              max31865_1.temperature(raw),
              max31865_1.temperature(MAX31865_SENSOR_OHMS_1, MAX31865_CALIBRATION_OHMS_1)
            );
          #else
            return (int16_t)raw * 0.25f;
          #endif
        #elif TEMP_SENSOR_1_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_1_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 2:
        #if TEMP_SENSOR_2_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_2, raw);
        #elif TEMP_SENSOR_IS_MAX_TC(2)
          #if TEMP_SENSOR_0_IS_MAX31865
            return TERN(LIB_INTERNAL_MAX31865,
              max31865_2.temperature(raw),
              max31865_2.temperature(MAX31865_SENSOR_OHMS_2, MAX31865_CALIBRATION_OHMS_2)
            );
          #else
            return (int16_t)raw * 0.25f;
          #endif
        #elif TEMP_SENSOR_2_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_2_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 3:
        #if TEMP_SENSOR_3_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_3, raw);
        #elif TEMP_SENSOR_3_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_3_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 4:
        #if TEMP_SENSOR_4_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_4, raw);
        #elif TEMP_SENSOR_4_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_4_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 5:
        #if TEMP_SENSOR_5_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_5, raw);
        #elif TEMP_SENSOR_5_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_5_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 6:
        #if TEMP_SENSOR_6_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_6, raw);
        #elif TEMP_SENSOR_6_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_6_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      case 7:
        #if TEMP_SENSOR_7_IS_CUSTOM
          return user_thermistor_to_deg_c(CTI_HOTEND_7, raw);
        #elif TEMP_SENSOR_7_IS_AD595
          return TEMP_AD595(raw);
        #elif TEMP_SENSOR_7_IS_AD8495
          return TEMP_AD8495(raw);
        #else
          break;
        #endif
      default: break;
    }

    #if HAS_HOTEND_THERMISTOR
      // Thermistor with conversion table?
      const temp_entry_t(*tt)[] = (temp_entry_t(*)[])(heater_ttbl_map[e]);
      SCAN_THERMISTOR_TABLE((*tt), heater_ttbllen_map[e]);
    #endif

    return 0;
  }
#endif // HAS_HOTEND

#if HAS_HEATED_BED
  // For bed temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_bed(const raw_adc_t raw) {
    #if TEMP_SENSOR_BED_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_BED, raw);
    #elif TEMP_SENSOR_IS_MAX_TC(BED)
      #if TEMP_SENSOR_BED_IS_MAX31865
        return TERN(LIB_INTERNAL_MAX31865,
          max31865_BED.temperature(raw),
          max31865_BED.temperature(MAX31865_SENSOR_OHMS_BED, MAX31865_CALIBRATION_OHMS_BED)
        );
      #else
        return (int16_t)raw * 0.25f;
      #endif
    #elif TEMP_SENSOR_BED_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_BED, TEMPTABLE_BED_LEN);
    #elif TEMP_SENSOR_BED_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_BED_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_HEATED_BED

#if HAS_TEMP_CHAMBER
  // For chamber temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_chamber(const raw_adc_t raw) {
    #if TEMP_SENSOR_CHAMBER_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_CHAMBER, raw);
    #elif TEMP_SENSOR_CHAMBER_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_CHAMBER, TEMPTABLE_CHAMBER_LEN);
    #elif TEMP_SENSOR_CHAMBER_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_CHAMBER_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_TEMP_CHAMBER

#if HAS_TEMP_COOLER
  // For cooler temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_cooler(const raw_adc_t raw) {
    #if TEMP_SENSOR_COOLER_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_COOLER, raw);
    #elif TEMP_SENSOR_COOLER_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_COOLER, TEMPTABLE_COOLER_LEN);
    #elif TEMP_SENSOR_COOLER_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_COOLER_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_TEMP_COOLER

#if HAS_TEMP_PROBE
  // For probe temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_probe(const raw_adc_t raw) {
    #if TEMP_SENSOR_PROBE_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_PROBE, raw);
    #elif TEMP_SENSOR_PROBE_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_PROBE, TEMPTABLE_PROBE_LEN);
    #elif TEMP_SENSOR_PROBE_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_PROBE_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_TEMP_PROBE

#if HAS_TEMP_BOARD
  // For motherboard temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_board(const raw_adc_t raw) {
    #if TEMP_SENSOR_BOARD_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_BOARD, raw);
    #elif TEMP_SENSOR_BOARD_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_BOARD, TEMPTABLE_BOARD_LEN);
    #elif TEMP_SENSOR_BOARD_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_BOARD_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_TEMP_BOARD

#if HAS_TEMP_SOC
  // For SoC temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_soc(const raw_adc_t raw) {
    return (
      #ifdef TEMP_SOC_SENSOR
        TEMP_SOC_SENSOR(raw)
      #else
        0
        #error "TEMP_SENSOR_SOC requires the TEMP_SOC_SENSOR(RAW) macro to be defined for your board."
      #endif
    );
  }
#endif

#if HAS_TEMP_REDUNDANT
  // For redundant temperature measurement.
  celsius_float_t Temperature::analog_to_celsius_redundant(const raw_adc_t raw) {
    #if TEMP_SENSOR_REDUNDANT_IS_CUSTOM
      return user_thermistor_to_deg_c(CTI_REDUNDANT, raw);
    #elif TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E0)
      return TERN(TEMP_SENSOR_REDUNDANT_IS_MAX31865, max31865_0.temperature(raw), (int16_t)raw * 0.25f);
    #elif TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E1)
      return TERN(TEMP_SENSOR_REDUNDANT_IS_MAX31865, max31865_1.temperature(raw), (int16_t)raw * 0.25f);
    #elif TEMP_SENSOR_IS_MAX_TC(REDUNDANT) && REDUNDANT_TEMP_MATCH(SOURCE, E2)
      return TERN(TEMP_SENSOR_REDUNDANT_IS_MAX31865, max31865_2.temperature(raw), (int16_t)raw * 0.25f);
    #elif TEMP_SENSOR_REDUNDANT_IS_THERMISTOR
      SCAN_THERMISTOR_TABLE(TEMPTABLE_REDUNDANT, TEMPTABLE_REDUNDANT_LEN);
    #elif TEMP_SENSOR_REDUNDANT_IS_AD595
      return TEMP_AD595(raw);
    #elif TEMP_SENSOR_REDUNDANT_IS_AD8495
      return TEMP_AD8495(raw);
    #else
      UNUSED(raw);
      return 0;
    #endif
  }
#endif // HAS_TEMP_REDUNDANT

/**
 * Convert the raw sensor readings into actual Celsius temperatures and
 * validate raw temperatures. Bad readings generate min/maxtemp errors.
 *
 * The raw values are generated entirely in interrupt context, and this
 * method is called from normal context once 'raw_temps_ready' has been
 * set by update_raw_temperatures().
 *
 * The watchdog is dependent on this method. If 'raw_temps_ready' stops
 * being set by the interrupt so that this method is not called for over
 * 4 seconds then something has gone afoul and the machine will be reset.
 */
void Temperature::updateTemperaturesFromRawValues() {

  hal.watchdog_refresh(); // Reset because raw_temps_ready was set by the interrupt

  #if TEMP_SENSOR_IS_MAX_TC(0)
    temp_hotend[0].setraw(READ_MAX_TC(0));
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(1)
    temp_hotend[1].setraw(READ_MAX_TC(1));
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(2)
    temp_hotend[2].setraw(READ_MAX_TC(2));
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
    temp_redundant.setraw(READ_MAX_TC(HEATER_ID(TEMP_SENSOR_REDUNDANT_SOURCE)));
  #endif
  #if TEMP_SENSOR_IS_MAX_TC(BED)
    temp_bed.setraw(read_max_tc_bed());
  #endif

  #if HAS_HOTEND
    HOTEND_LOOP() temp_hotend[e].celsius = analog_to_celsius_hotend(temp_hotend[e].getraw(), e);
  #endif

  TERN_(HAS_HEATED_BED,     temp_bed.celsius       = analog_to_celsius_bed(temp_bed.getraw()));
  TERN_(HAS_TEMP_CHAMBER,   temp_chamber.celsius   = analog_to_celsius_chamber(temp_chamber.getraw()));
  TERN_(HAS_TEMP_COOLER,    temp_cooler.celsius    = analog_to_celsius_cooler(temp_cooler.getraw()));
  TERN_(HAS_TEMP_PROBE,     temp_probe.celsius     = analog_to_celsius_probe(temp_probe.getraw()));
  TERN_(HAS_TEMP_BOARD,     temp_board.celsius     = analog_to_celsius_board(temp_board.getraw()));
  TERN_(HAS_TEMP_SOC,       temp_soc.celsius       = analog_to_celsius_soc(temp_soc.getraw()));
  TERN_(HAS_TEMP_REDUNDANT, temp_redundant.celsius = analog_to_celsius_redundant(temp_redundant.getraw()));

  TERN_(FILAMENT_WIDTH_SENSOR, filwidth.update_measured_mm());
  TERN_(HAS_POWER_MONITOR,     power_monitor.capture_values());

  #if HAS_HOTEND
    #define _TEMPDIR(N) TEMP_SENSOR_IS_ANY_MAX_TC(N) ? 0 : TEMPDIR(N),
    static constexpr int8_t temp_dir[HOTENDS] = { REPEAT(HOTENDS, _TEMPDIR) };

    HOTEND_LOOP() {
      const raw_adc_t r = temp_hotend[e].getraw();
      const bool neg = temp_dir[e] < 0, pos = temp_dir[e] > 0;
      if ((neg && r < temp_range[e].raw_max) || (pos && r > temp_range[e].raw_max))
        MAXTEMP_ERROR(e, temp_hotend[e].celsius);

      /**
      // DEBUG PREHEATING TIME
      SERIAL_ECHOLNPGM("\nExtruder = ", e, " Preheat On/Off = ", is_preheating(e));
      const float test_is_preheating = (preheat_end_ms_hotend[HOTEND_INDEX] - millis()) * 0.001f;
      if (test_is_preheating < 31) SERIAL_ECHOLNPGM("Extruder = ", e, " Preheat remaining time = ", test_is_preheating, "s", "\n");
      //*/

      const bool heater_on = temp_hotend[e].target > 0;
      if (heater_on && !is_hotend_preheating(e) && ((neg && r > temp_range[e].raw_min) || (pos && r < temp_range[e].raw_min))) {
        if (TERN1(MULTI_MAX_CONSECUTIVE_LOW_TEMP_ERR, ++consecutive_low_temperature_error[e] >= MAX_CONSECUTIVE_LOW_TEMPERATURE_ERROR_ALLOWED))
          MINTEMP_ERROR(e, temp_hotend[e].celsius);
      }
      else {
        TERN_(MULTI_MAX_CONSECUTIVE_LOW_TEMP_ERR, consecutive_low_temperature_error[e] = 0);
      }
    }

  #endif // HAS_HOTEND

  #if ENABLED(THERMAL_PROTECTION_BED)
    if (TP_CMP(BED, temp_bed.getraw(), temp_sensor_range_bed.raw_max))
      MAXTEMP_ERROR(H_BED, temp_bed.celsius);
    if (temp_bed.target > 0 && !is_bed_preheating() && TP_CMP(BED, temp_sensor_range_bed.raw_min, temp_bed.getraw()))
      MINTEMP_ERROR(H_BED, temp_bed.celsius);
  #endif

  #if ALL(HAS_HEATED_CHAMBER, THERMAL_PROTECTION_CHAMBER)
    if (TP_CMP(CHAMBER, temp_chamber.getraw(), temp_sensor_range_chamber.raw_max))
      MAXTEMP_ERROR(H_CHAMBER, temp_chamber.celsius);
    if (temp_chamber.target > 0 && TP_CMP(CHAMBER, temp_sensor_range_chamber.raw_min, temp_chamber.getraw()))
      MINTEMP_ERROR(H_CHAMBER, temp_chamber.celsius);
  #endif

  #if ALL(HAS_COOLER, THERMAL_PROTECTION_COOLER)
    if (cutter.unitPower > 0 && TP_CMP(COOLER, temp_cooler.getraw(), temp_sensor_range_cooler.raw_max))
      MAXTEMP_ERROR(H_COOLER, temp_cooler.celsius);
    if (TP_CMP(COOLER, temp_sensor_range_cooler.raw_min, temp_cooler.getraw()))
      MINTEMP_ERROR(H_COOLER, temp_cooler.celsius);
  #endif

  #if ALL(HAS_TEMP_BOARD, THERMAL_PROTECTION_BOARD)
    if (TP_CMP(BOARD, temp_board.getraw(), temp_sensor_range_board.raw_max))
      MAXTEMP_ERROR(H_BOARD, temp_board.celsius);
    if (TP_CMP(BOARD, temp_sensor_range_board.raw_min, temp_board.getraw()))
      MINTEMP_ERROR(H_BOARD, temp_board.celsius);
  #endif

  #if ALL(HAS_TEMP_SOC, THERMAL_PROTECTION_SOC)
    if (TP_CMP(SOC, temp_soc.getraw(), maxtemp_raw_SOC)) MAXTEMP_ERROR(H_SOC, temp_soc.celsius);
  #endif

} // Temperature::updateTemperaturesFromRawValues

/**
 * Initialize the temperature manager
 *
 * The manager is implemented by periodic calls to task()
 *
 *  - Init (and disable) SPI thermocouples like MAX6675 and MAX31865
 *  - Disable RUMBA JTAG to accommodate a thermocouple extension
 *  - Read-enable thermistors with a read-enable pin
 *  - Init HEATER and COOLER pins for OUTPUT in OFF state
 *  - Init the FAN pins as PWM or OUTPUT
 *  - Init the SPI interface for SPI thermocouples
 *  - Init ADC according to the HAL
 *  - Set thermistor pins to analog inputs according to the HAL
 *  - Start the Temperature ISR timer
 *  - Init the AUTO FAN pins as PWM or OUTPUT
 *  - Wait 250ms for temperatures to settle
 *  - Init temp_range[], used for catching min/maxtemp
 */
void Temperature::init() {

  TERN_(PROBING_HEATERS_OFF, paused_for_probing = false);

  // Init (and disable) SPI thermocouples
  #if TEMP_SENSOR_IS_ANY_MAX_TC(0) && PIN_EXISTS(TEMP_0_CS)
    OUT_WRITE(TEMP_0_CS_PIN, HIGH);
  #endif
  #if TEMP_SENSOR_IS_ANY_MAX_TC(1) && PIN_EXISTS(TEMP_1_CS)
    OUT_WRITE(TEMP_1_CS_PIN, HIGH);
  #endif
  #if TEMP_SENSOR_IS_ANY_MAX_TC(2) && PIN_EXISTS(TEMP_2_CS)
    OUT_WRITE(TEMP_2_CS_PIN, HIGH);
  #endif

  // Setup objects for library-based polling of MAX TCs
  #if HAS_MAXTC_LIBRARIES
    #define _MAX31865_WIRES(n) MAX31865_##n##WIRE
    #define MAX31865_WIRES(n) _MAX31865_WIRES(n)

    #if TEMP_SENSOR_IS_MAX(0, 6675) && HAS_MAX6675_LIBRARY
      max6675_0.begin();
    #elif TEMP_SENSOR_IS_MAX(0, 31855) && HAS_MAX31855_LIBRARY
      max31855_0.begin();
    #elif TEMP_SENSOR_IS_MAX(0, 31865)
      max31865_0.begin(
        MAX31865_WIRES(MAX31865_SENSOR_WIRES_0) // MAX31865_2WIRE, MAX31865_3WIRE, MAX31865_4WIRE
        OPTARG(LIB_INTERNAL_MAX31865, MAX31865_SENSOR_OHMS_0, MAX31865_CALIBRATION_OHMS_0, MAX31865_WIRE_OHMS_0)
      );
    #endif

    #if TEMP_SENSOR_IS_MAX(1, 6675) && HAS_MAX6675_LIBRARY
      max6675_1.begin();
    #elif TEMP_SENSOR_IS_MAX(1, 31855) && HAS_MAX31855_LIBRARY
      max31855_1.begin();
    #elif TEMP_SENSOR_IS_MAX(1, 31865)
      max31865_1.begin(
        MAX31865_WIRES(MAX31865_SENSOR_WIRES_1) // MAX31865_2WIRE, MAX31865_3WIRE, MAX31865_4WIRE
        OPTARG(LIB_INTERNAL_MAX31865, MAX31865_SENSOR_OHMS_1, MAX31865_CALIBRATION_OHMS_1, MAX31865_WIRE_OHMS_1)
      );
    #endif

    #if TEMP_SENSOR_IS_MAX(2, 6675) && HAS_MAX6675_LIBRARY
      max6675_2.begin();
    #elif TEMP_SENSOR_IS_MAX(2, 31855) && HAS_MAX31855_LIBRARY
      max31855_2.begin();
    #elif TEMP_SENSOR_IS_MAX(2, 31865)
      max31865_2.begin(
        MAX31865_WIRES(MAX31865_SENSOR_WIRES_2) // MAX31865_2WIRE, MAX31865_3WIRE, MAX31865_4WIRE
        OPTARG(LIB_INTERNAL_MAX31865, MAX31865_SENSOR_OHMS_2, MAX31865_CALIBRATION_OHMS_2, MAX31865_WIRE_OHMS_2)
      );
    #endif

    #if TEMP_SENSOR_IS_MAX(BED, 6675) && HAS_MAX6675_LIBRARY
      max6675_BED.begin();
    #elif TEMP_SENSOR_IS_MAX(BED, 31855) && HAS_MAX31855_LIBRARY
      max31855_BED.begin();
    #elif TEMP_SENSOR_IS_MAX(BED, 31865)
      max31865_BED.begin(
        MAX31865_WIRES(MAX31865_SENSOR_WIRES_BED) // MAX31865_BEDWIRE, MAX31865_3WIRE, MAX31865_4WIRE
        OPTARG(LIB_INTERNAL_MAX31865, MAX31865_SENSOR_OHMS_BED, MAX31865_CALIBRATION_OHMS_BED, MAX31865_WIRE_OHMS_BED)
      );
    #endif

    #undef MAX31865_WIRES
    #undef _MAX31865_WIRES
  #endif

  #if MB(RUMBA)
    // Disable RUMBA JTAG in case the thermocouple extension is plugged on top of JTAG connector
    #define _AD(N) (TEMP_SENSOR_##N##_IS_AD595 || TEMP_SENSOR_##N##_IS_AD8495)
    #if _AD(0) || _AD(1) || _AD(2) || _AD(BED) || _AD(CHAMBER) || _AD(REDUNDANT)
      MCUCR = _BV(JTD);
      MCUCR = _BV(JTD);
    #endif
  #endif

  // Thermistor activation by MCU pin
  #if PIN_EXISTS(TEMP_0_TR_ENABLE)
    OUT_WRITE(TEMP_0_TR_ENABLE_PIN, (
      #if TEMP_SENSOR_IS_ANY_MAX_TC(0)
        HIGH
      #else
        LOW
      #endif
    ));
  #endif
  #if PIN_EXISTS(TEMP_1_TR_ENABLE)
    OUT_WRITE(TEMP_1_TR_ENABLE_PIN, (
      #if TEMP_SENSOR_IS_ANY_MAX_TC(1)
        HIGH
      #else
        LOW
      #endif
    ));
  #endif
  #if PIN_EXISTS(TEMP_2_TR_ENABLE)
    OUT_WRITE(TEMP_2_TR_ENABLE_PIN, (
      #if TEMP_SENSOR_IS_ANY_MAX_TC(2)
        HIGH
      #else
        LOW
      #endif
    ));
  #endif

  #if ENABLED(MPCTEMP)
    HOTEND_LOOP() temp_hotend[e].modeled_block_temp = NAN;
  #endif

  #if HAS_HEATER_0
    #ifdef BOARD_OPENDRAIN_MOSFETS
      OUT_WRITE_OD(HEATER_0_PIN, ENABLED(HEATER_0_INVERTING));
    #else
      OUT_WRITE(HEATER_0_PIN, ENABLED(HEATER_0_INVERTING));
    #endif
  #endif
  #if HAS_HEATER_1
    OUT_WRITE(HEATER_1_PIN, ENABLED(HEATER_1_INVERTING));
  #endif
  #if HAS_HEATER_2
    OUT_WRITE(HEATER_2_PIN, ENABLED(HEATER_2_INVERTING));
  #endif
  #if HAS_HEATER_3
    OUT_WRITE(HEATER_3_PIN, ENABLED(HEATER_3_INVERTING));
  #endif
  #if HAS_HEATER_4
    OUT_WRITE(HEATER_4_PIN, ENABLED(HEATER_4_INVERTING));
  #endif
  #if HAS_HEATER_5
    OUT_WRITE(HEATER_5_PIN, ENABLED(HEATER_5_INVERTING));
  #endif
  #if HAS_HEATER_6
    OUT_WRITE(HEATER_6_PIN, ENABLED(HEATER_6_INVERTING));
  #endif
  #if HAS_HEATER_7
    OUT_WRITE(HEATER_7_PIN, ENABLED(HEATER_7_INVERTING));
  #endif

  #if HAS_HEATED_BED
    #if ENABLED(PELTIER_BED)
      SET_OUTPUT(PELTIER_DIR_PIN);
      OUT_WRITE(PELTIER_DIR_PIN, !PELTIER_DIR_HEAT_STATE);
    #endif
    #ifdef BOARD_OPENDRAIN_MOSFETS
      OUT_WRITE_OD(HEATER_BED_PIN, ENABLED(HEATER_BED_INVERTING));
    #else
      OUT_WRITE(HEATER_BED_PIN, ENABLED(HEATER_BED_INVERTING));
    #endif
  #endif

  #if HAS_HEATED_CHAMBER
    OUT_WRITE(HEATER_CHAMBER_PIN, ENABLED(HEATER_CHAMBER_INVERTING));
  #endif

  #if HAS_COOLER
    OUT_WRITE(COOLER_PIN, ENABLED(COOLER_INVERTING));
  #endif

  #if HAS_FAN0
    INIT_FAN_PIN(FAN0_PIN);
  #endif
  #if HAS_FAN1
    INIT_FAN_PIN(FAN1_PIN);
  #endif
  #if HAS_FAN2
    INIT_FAN_PIN(FAN2_PIN);
  #endif
  #if HAS_FAN3
    INIT_FAN_PIN(FAN3_PIN);
  #endif
  #if HAS_FAN4
    INIT_FAN_PIN(FAN4_PIN);
  #endif
  #if HAS_FAN5
    INIT_FAN_PIN(FAN5_PIN);
  #endif
  #if HAS_FAN6
    INIT_FAN_PIN(FAN6_PIN);
  #endif
  #if HAS_FAN7
    INIT_FAN_PIN(FAN7_PIN);
  #endif
  #if ENABLED(USE_CONTROLLER_FAN)
    INIT_FAN_PIN(CONTROLLER_FAN_PIN);
  #endif

  TERN_(HAS_MAXTC_SW_SPI, max_tc_spi.init());

  hal.adc_init();

  TERN_(HAS_TEMP_ADC_0,         hal.adc_enable(TEMP_0_PIN));
  TERN_(HAS_TEMP_ADC_1,         hal.adc_enable(TEMP_1_PIN));
  TERN_(HAS_TEMP_ADC_2,         hal.adc_enable(TEMP_2_PIN));
  TERN_(HAS_TEMP_ADC_3,         hal.adc_enable(TEMP_3_PIN));
  TERN_(HAS_TEMP_ADC_4,         hal.adc_enable(TEMP_4_PIN));
  TERN_(HAS_TEMP_ADC_5,         hal.adc_enable(TEMP_5_PIN));
  TERN_(HAS_TEMP_ADC_6,         hal.adc_enable(TEMP_6_PIN));
  TERN_(HAS_TEMP_ADC_7,         hal.adc_enable(TEMP_7_PIN));
  TERN_(HAS_JOY_ADC_X,          hal.adc_enable(JOY_X_PIN));
  TERN_(HAS_JOY_ADC_Y,          hal.adc_enable(JOY_Y_PIN));
  TERN_(HAS_JOY_ADC_Z,          hal.adc_enable(JOY_Z_PIN));
  TERN_(HAS_TEMP_ADC_BED,       hal.adc_enable(TEMP_BED_PIN));
  TERN_(HAS_TEMP_ADC_CHAMBER,   hal.adc_enable(TEMP_CHAMBER_PIN));
  TERN_(HAS_TEMP_ADC_PROBE,     hal.adc_enable(TEMP_PROBE_PIN));
  TERN_(HAS_TEMP_ADC_COOLER,    hal.adc_enable(TEMP_COOLER_PIN));
  TERN_(HAS_TEMP_ADC_BOARD,     hal.adc_enable(TEMP_BOARD_PIN));
  TERN_(HAS_TEMP_ADC_SOC,       hal.adc_enable(TEMP_SOC_PIN));
  TERN_(HAS_TEMP_ADC_REDUNDANT, hal.adc_enable(TEMP_REDUNDANT_PIN));
  TERN_(FILAMENT_WIDTH_SENSOR,  hal.adc_enable(FILWIDTH_PIN));
  TERN_(HAS_ADC_BUTTONS,        hal.adc_enable(ADC_KEYPAD_PIN));
  TERN_(POWER_MONITOR_CURRENT,  hal.adc_enable(POWER_MONITOR_CURRENT_PIN));
  TERN_(POWER_MONITOR_VOLTAGE,  hal.adc_enable(POWER_MONITOR_VOLTAGE_PIN));

  #if HAS_JOY_ADC_EN
    SET_INPUT_PULLUP(JOY_EN_PIN);
  #endif

  HAL_timer_start(MF_TIMER_TEMP, TEMP_TIMER_FREQUENCY);
  ENABLE_TEMPERATURE_INTERRUPT();

  #if HAS_AUTO_FAN
    #define _OREFAN(I,N) || _EFANOVERLAP(I,N)
    #if HAS_AUTO_FAN_0
      INIT_E_AUTO_FAN_PIN(E0_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_1 && !_EFANOVERLAP(0,1)
      INIT_E_AUTO_FAN_PIN(E1_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_2 && !(0 REPEAT2(2, _OREFAN, 2))
      INIT_E_AUTO_FAN_PIN(E2_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_3 && !(0 REPEAT2(3, _OREFAN, 3))
      INIT_E_AUTO_FAN_PIN(E3_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_4 && !(0 REPEAT2(4, _OREFAN, 4))
      INIT_E_AUTO_FAN_PIN(E4_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_5 && !(0 REPEAT2(5, _OREFAN, 5))
      INIT_E_AUTO_FAN_PIN(E5_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_6 && !(0 REPEAT2(6, _OREFAN, 6))
      INIT_E_AUTO_FAN_PIN(E6_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_FAN_7 && !(0 REPEAT2(7, _OREFAN, 7))
      INIT_E_AUTO_FAN_PIN(E7_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_CHAMBER_FAN && !AUTO_CHAMBER_IS_E
      INIT_CHAMBER_AUTO_FAN_PIN(CHAMBER_AUTO_FAN_PIN);
    #endif
    #if HAS_AUTO_COOLER_FAN && !AUTO_COOLER_IS_E
      INIT_COOLER_AUTO_FAN_PIN(COOLER_AUTO_FAN_PIN);
    #endif
  #endif // HAS_AUTO_FAN

  #if HAS_HOTEND
    #define _TEMP_MIN_E(NR) do{ \
      const celsius_t tmin_tmp = TERN(TEMP_SENSOR_##NR##_IS_CUSTOM, 0, int16_t(pgm_read_word(&TEMPTABLE_##NR [TEMP_SENSOR_##NR##_MINTEMP_IND].celsius))), \
                      tmin = _MAX(HEATER_##NR##_MINTEMP, tmin_tmp); \
      temp_range[NR].mintemp = tmin; \
      while (analog_to_celsius_hotend(temp_range[NR].raw_min, NR) < tmin) \
        temp_range[NR].raw_min += TEMPDIR(NR) * (OVERSAMPLENR); \
    }while(0)
    #define _TEMP_MAX_E(NR) do{ \
      const celsius_t tmax_tmp = TERN(TEMP_SENSOR_##NR##_IS_CUSTOM, 2000, int16_t(pgm_read_word(&TEMPTABLE_##NR [TEMP_SENSOR_##NR##_MAXTEMP_IND].celsius)) - 1), \
                      tmax = _MIN(HEATER_##NR##_MAXTEMP, tmax_tmp); \
      temp_range[NR].maxtemp = tmax; \
      while (analog_to_celsius_hotend(temp_range[NR].raw_max, NR) > tmax) \
        temp_range[NR].raw_max -= TEMPDIR(NR) * (OVERSAMPLENR); \
    }while(0)

    #define _MINMAX_TEST(N,M) (!TEMP_SENSOR_##N##_IS_DUMMY && HOTENDS > (N) && TEMP_SENSOR_##N##_IS_THERMISTOR && defined(HEATER_##N##_##M##TEMP))

    #if _MINMAX_TEST(0, MIN)
      _TEMP_MIN_E(0);
    #endif
    #if _MINMAX_TEST(0, MAX)
      _TEMP_MAX_E(0);
    #endif
    #if _MINMAX_TEST(1, MIN)
      _TEMP_MIN_E(1);
    #endif
    #if _MINMAX_TEST(1, MAX)
      _TEMP_MAX_E(1);
    #endif
    #if _MINMAX_TEST(2, MIN)
      _TEMP_MIN_E(2);
    #endif
    #if _MINMAX_TEST(2, MAX)
      _TEMP_MAX_E(2);
    #endif
    #if _MINMAX_TEST(3, MIN)
      _TEMP_MIN_E(3);
    #endif
    #if _MINMAX_TEST(3, MAX)
      _TEMP_MAX_E(3);
    #endif
    #if _MINMAX_TEST(4, MIN)
      _TEMP_MIN_E(4);
    #endif
    #if _MINMAX_TEST(4, MAX)
      _TEMP_MAX_E(4);
    #endif
    #if _MINMAX_TEST(5, MIN)
      _TEMP_MIN_E(5);
    #endif
    #if _MINMAX_TEST(5, MAX)
      _TEMP_MAX_E(5);
    #endif
    #if _MINMAX_TEST(6, MIN)
      _TEMP_MIN_E(6);
    #endif
    #if _MINMAX_TEST(6, MAX)
      _TEMP_MAX_E(6);
    #endif
    #if _MINMAX_TEST(7, MIN)
      _TEMP_MIN_E(7);
    #endif
    #if _MINMAX_TEST(7, MAX)
      _TEMP_MAX_E(7);
    #endif
  #endif // HAS_HOTEND

  // TODO: combine these into the macros above
  #if HAS_HEATED_BED
    while (analog_to_celsius_bed(temp_sensor_range_bed.raw_min) < BED_MINTEMP)
      temp_sensor_range_bed.raw_min += TEMPDIR(BED) * (OVERSAMPLENR);
    while (analog_to_celsius_bed(temp_sensor_range_bed.raw_max) > BED_MAXTEMP)
      temp_sensor_range_bed.raw_max -= TEMPDIR(BED) * (OVERSAMPLENR);
  #endif

  #if HAS_HEATED_CHAMBER
    while (analog_to_celsius_chamber(temp_sensor_range_chamber.raw_min) < CHAMBER_MINTEMP)
      temp_sensor_range_chamber.raw_min += TEMPDIR(CHAMBER) * (OVERSAMPLENR);
    while (analog_to_celsius_chamber(temp_sensor_range_chamber.raw_max) > CHAMBER_MAXTEMP)
      temp_sensor_range_chamber.raw_max -= TEMPDIR(CHAMBER) * (OVERSAMPLENR);
  #endif

  #if HAS_COOLER
    while (analog_to_celsius_cooler(temp_sensor_range_cooler.raw_min) > COOLER_MINTEMP)
      temp_sensor_range_cooler.raw_min += TEMPDIR(COOLER) * (OVERSAMPLENR);
    while (analog_to_celsius_cooler(temp_sensor_range_cooler.raw_max) < COOLER_MAXTEMP)
      temp_sensor_range_cooler.raw_max -= TEMPDIR(COOLER) * (OVERSAMPLENR);
  #endif

  #if ALL(HAS_TEMP_BOARD, THERMAL_PROTECTION_BOARD)
    while (analog_to_celsius_board(temp_sensor_range_board.raw_min) < BOARD_MINTEMP)
      temp_sensor_range_board.raw_min += TEMPDIR(BOARD) * (OVERSAMPLENR);
    while (analog_to_celsius_board(temp_sensor_range_board.raw_max) > BOARD_MAXTEMP)
      temp_sensor_range_board.raw_max -= TEMPDIR(BOARD) * (OVERSAMPLENR);
  #endif

  #if ALL(HAS_TEMP_SOC, THERMAL_PROTECTION_SOC)
    while (analog_to_celsius_soc(maxtemp_raw_SOC) > SOC_MAXTEMP) maxtemp_raw_SOC -= OVERSAMPLENR;
  #endif

  #if HAS_TEMP_REDUNDANT
    temp_redundant.target = &(
      #if REDUNDANT_TEMP_MATCH(TARGET, COOLER) && HAS_TEMP_COOLER
        temp_cooler
      #elif REDUNDANT_TEMP_MATCH(TARGET, PROBE) && HAS_TEMP_PROBE
        temp_probe
      #elif REDUNDANT_TEMP_MATCH(TARGET, BOARD) && HAS_TEMP_BOARD
        temp_board
      #elif REDUNDANT_TEMP_MATCH(TARGET, CHAMBER) && HAS_TEMP_CHAMBER
        temp_chamber
      #elif REDUNDANT_TEMP_MATCH(TARGET, BED) && HAS_TEMP_BED
        temp_bed
      #else
        temp_hotend[HEATER_ID(TEMP_SENSOR_REDUNDANT_TARGET)]
      #endif
    );
  #endif
}

#if HAS_THERMAL_PROTECTION

  #pragma GCC diagnostic push
  #if __has_cpp_attribute(fallthrough)
    #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
  #endif

  Temperature::tr_state_machine_t Temperature::tr_state_machine[NR_HEATER_RUNAWAY]; // = { { TRInactive, 0 } };

  /**
   * @brief Thermal Runaway state machine for a single heater
   * @param current          current measured temperature
   * @param target           current target temperature
   * @param heater_id        extruder index
   * @param period_seconds   missed temperature allowed time
   * @param hysteresis_degc  allowed distance from target
   *
   * TODO: Embed the last 3 parameters during init, if not less optimal
   */
  void Temperature::tr_state_machine_t::run(const_celsius_float_t current, const_celsius_float_t target, const heater_id_t heater_id, const uint16_t period_seconds, const celsius_float_t hysteresis_degc) {

    #if HEATER_IDLE_HANDLER
      // Convert the given heater_id_t to an idle array index
      const IdleIndex idle_index = idle_index_for_id(heater_id);
    #endif

    /**
      SERIAL_ECHO_START();
      SERIAL_ECHOPGM("Thermal Runaway Running. Heater ID: ");
      switch (heater_id) {
        case H_BED:     SERIAL_ECHOPGM("bed"); break;
        case H_CHAMBER: SERIAL_ECHOPGM("chamber"); break;
        default:        SERIAL_ECHO(heater_id);
      }
      SERIAL_ECHOLNPGM(
        " ; sizeof(running_temp):", sizeof(running_temp),
        " ;  State:", state, " ;  Timer:", timer, " ;  Temperature:", current, " ;  Target Temp:", target
        #if HEATER_IDLE_HANDLER
          , " ;  Idle Timeout:", heater_idle[idle_index].timed_out
        #endif
      );
    */

    #if ENABLED(THERMAL_PROTECTION_VARIANCE_MONITOR)

      #ifdef THERMAL_PROTECTION_VARIANCE_MONITOR_PERIOD
        #define VARIANCE_WINDOW THERMAL_PROTECTION_VARIANCE_MONITOR_PERIOD
      #else
        #define VARIANCE_WINDOW period_seconds
      #endif

      if (state == TRMalfunction) { // temperature invariance may continue, regardless of heater state
        variance += ABS(current - last_temp); // no need for detection window now, a single change in variance is enough
        last_temp = current;
        if (!NEAR_ZERO(variance)) {
          variance_timer = millis() + SEC_TO_MS(VARIANCE_WINDOW);
          variance = 0.0;
          state = TRStable; // resume from where we detected the problem
        }
      }
    #endif

    if (TERN1(THERMAL_PROTECTION_VARIANCE_MONITOR, state != TRMalfunction)) {
      // If the heater idle timeout expires, restart
      if (TERN0(HEATER_IDLE_HANDLER, heater_idle[idle_index].timed_out)) {
        state = TRInactive;
        running_temp = 0;
        TERN_(THERMAL_PROTECTION_VARIANCE_MONITOR, variance_timer = 0);
      }
      else if (running_temp != target) { // If the target temperature changes, restart
        running_temp = target;
        state = target > 0 ? TRFirstHeating : TRInactive;
        TERN_(THERMAL_PROTECTION_VARIANCE_MONITOR, variance_timer = 0);
      }
    }

    switch (state) {
      // Inactive state waits for a target temperature to be set
      case TRInactive: break;

      // When first heating, wait for the temperature to be reached then go to Stable state
      case TRFirstHeating:
        if (current < running_temp) break;
        state = TRStable;

      // While the temperature is stable watch for a bad temperature
      case TRStable: {

        const celsius_float_t rdiff = running_temp - current;

        #if ENABLED(ADAPTIVE_FAN_SLOWING)
          if (adaptive_fan_slowing && heater_id >= 0) {
            const int_fast8_t fan_index = _MIN(heater_id, FAN_COUNT - 1);
            uint8_t scale;
            if (fan_speed[fan_index] == 0 || rdiff <= hysteresis_degc * 0.25f)
              scale = 128;
            else if (rdiff <= hysteresis_degc * 0.3335f)
              scale = 96;
            else if (rdiff <= hysteresis_degc * 0.5f)
              scale = 64;
            else if (rdiff <= hysteresis_degc * 0.8f)
              scale = 32;
            else
              scale = 0;

            if (TERN0(REPORT_ADAPTIVE_FAN_SLOWING, DEBUGGING(INFO))) {
              const uint8_t fss7 = fan_speed_scaler[fan_index] & 0x80;
              if (fss7 ^ (scale & 0x80))
                serial_ternary(F("Adaptive Fan Slowing "), fss7, nullptr, F("de"), F("activated.\n"));
            }

            fan_speed_scaler[fan_index] = scale;
          }
        #endif // ADAPTIVE_FAN_SLOWING

        const millis_t now = millis();

        #if ENABLED(THERMAL_PROTECTION_VARIANCE_MONITOR)
          if (PENDING(now, variance_timer)) {
            variance += ABS(current - last_temp);
            last_temp = current;
          }
          else {
            if (NEAR_ZERO(variance) && variance_timer) { // valid variance monitoring window
              state = TRMalfunction;
              break;
            }
            variance_timer = now + SEC_TO_MS(VARIANCE_WINDOW);
            variance = 0.0;
            last_temp = current;
          }
        #endif

        if (rdiff <= hysteresis_degc) {
          timer = now + SEC_TO_MS(period_seconds);
          break;
        }
        else if (PENDING(now, timer)) break;
        state = TRRunaway;

      } // fall through

      case TRRunaway:
        TERN_(SOVOL_SV06_RTS, rts.gotoPageBeep(ID_KillRunaway_L, ID_KillRunaway_D));
        TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
        TERN_(EXTENSIBLE_UI, ExtUI::onHeatingError(heater_id));
        _TEMP_ERROR(heater_id, FPSTR(str_t_thermal_runaway), MSG_ERR_THERMAL_RUNAWAY, current);
        break;

      #if ENABLED(THERMAL_PROTECTION_VARIANCE_MONITOR)
        case TRMalfunction:
          TERN_(DWIN_CREALITY_LCD, dwinPopupTemperature(0));
          TERN_(EXTENSIBLE_UI, ExtUI::onHeatingError(heater_id));
          _TEMP_ERROR(heater_id, F(STR_T_THERMAL_MALFUNCTION), MSG_ERR_TEMP_MALFUNCTION, current);
          break;
      #endif
    }
  }

  #pragma GCC diagnostic pop

#endif // HAS_THERMAL_PROTECTION

void Temperature::disable_all_heaters() {

  // Disable autotemp, unpause and reset everything
  TERN_(AUTOTEMP, planner.autotemp.enabled = false);
  TERN_(PROBING_HEATERS_OFF, pause_heaters(false));

  #if HAS_HOTEND
    HOTEND_LOOP() {
      setTargetHotend(0, e);
      temp_hotend[e].soft_pwm_amount = 0;
    }
  #endif

  #if HAS_TEMP_HOTEND
    #define DISABLE_HEATER(N) WRITE_HEATER_##N(LOW);
    REPEAT(HOTENDS, DISABLE_HEATER);
  #endif

  #if HAS_HEATED_BED
    setTargetBed(0);
    temp_bed.soft_pwm_amount = 0;
    WRITE_HEATER_BED(LOW);
  #endif

  #if HAS_HEATED_CHAMBER
    setTargetChamber(0);
    temp_chamber.soft_pwm_amount = 0;
    WRITE_HEATER_CHAMBER(LOW);
  #endif

  #if HAS_COOLER
    setTargetCooler(0);
    temp_cooler.soft_pwm_amount = 0;
    WRITE_HEATER_COOLER(LOW);
  #endif
}

#if ENABLED(PRINTJOB_TIMER_AUTOSTART)

  bool Temperature::auto_job_over_threshold() {
    #if HAS_HOTEND
      HOTEND_LOOP() if (degTargetHotend(e) > (EXTRUDE_MINTEMP) / 2) return true;
    #endif
    return TERN0(HAS_HEATED_BED, degTargetBed() > BED_MINTEMP)
        || TERN0(HAS_HEATED_CHAMBER, degTargetChamber() > CHAMBER_MINTEMP);
  }

  void Temperature::auto_job_check_timer(const bool can_start, const bool can_stop) {
    if (auto_job_over_threshold()) {
      if (can_start) startOrResumeJob();
    }
    else if (can_stop) {
      print_job_timer.stop();
      ui.reset_status();
    }
  }

#endif // PRINTJOB_TIMER_AUTOSTART

#if ENABLED(PROBING_HEATERS_OFF)

  void Temperature::pause_heaters(const bool p) {
    if (p != paused_for_probing) {
      paused_for_probing = p;
      if (p) {
        HOTEND_LOOP() heater_idle[e].expire();    // Timeout immediately
        TERN_(HAS_HEATED_BED, heater_idle[IDLE_INDEX_BED].expire()); // Timeout immediately
      }
      else {
        HOTEND_LOOP() reset_hotend_idle_timer(e);
        TERN_(HAS_HEATED_BED, reset_bed_idle_timer());
      }
    }
  }

#endif // PROBING_HEATERS_OFF

#if ANY(SINGLENOZZLE_STANDBY_TEMP, SINGLENOZZLE_STANDBY_FAN)

  void Temperature::singlenozzle_change(const uint8_t old_tool, const uint8_t new_tool) {
    #if ENABLED(SINGLENOZZLE_STANDBY_FAN)
      singlenozzle_fan_speed[old_tool] = fan_speed[0];
      fan_speed[0] = singlenozzle_fan_speed[new_tool];
    #endif
    #if ENABLED(SINGLENOZZLE_STANDBY_TEMP)
      singlenozzle_temp[old_tool] = temp_hotend[0].target;
      if (singlenozzle_temp[new_tool] && singlenozzle_temp[new_tool] != singlenozzle_temp[old_tool]) {
        setTargetHotend(singlenozzle_temp[new_tool], 0);
        TERN_(AUTOTEMP, planner.autotemp_update());
        set_heating_message(0);
        (void)wait_for_hotend(0, false);  // Wait for heating or cooling
      }
    #endif
  }

#endif // SINGLENOZZLE_STANDBY_TEMP || SINGLENOZZLE_STANDBY_FAN

#if HAS_MAX_TC
  typedef TERN(HAS_MAX31855, uint32_t, uint16_t) max_tc_temp_t;

  #ifndef THERMOCOUPLE_MAX_ERRORS
    #define THERMOCOUPLE_MAX_ERRORS 15
  #endif
#endif

#if TEMP_SENSOR_IS_MAX_TC(0) || TEMP_SENSOR_IS_MAX_TC(1) || TEMP_SENSOR_IS_MAX_TC(2)
  /**
   * @brief Read MAX Thermocouple temperature.
   *
   * Reads the thermocouple board via HW or SW SPI, using a library (LIB_USR_x) or raw SPI reads.
   * Doesn't strictly return a temperature; returns an "ADC Value" (i.e. raw register content).
   *
   * @param  hindex  the hotend we're referencing (if MULTI_MAX_TC)
   * @return         integer representing the board's buffer, to be converted later if needed
   */
  raw_adc_t Temperature::read_max_tc(TERN_(HAS_MULTI_MAX_TC, const uint8_t hindex/*=0*/)) {
    #define MAXTC_HEAT_INTERVAL 250UL

    #if HAS_MAX31855
      #define MAX_TC_ERROR_MASK    7        // D2-0: SCV, SCG, OC
      #define MAX_TC_DISCARD_BITS 18        // Data D31-18; sign bit D31
      #define MAX_TC_SPEED_BITS    3        // ~1MHz
    #elif HAS_MAX31865
      #define MAX_TC_ERROR_MASK    1        // D0 Bit on fault only
      #define MAX_TC_DISCARD_BITS  1        // Data is in D15-D1
      #define MAX_TC_SPEED_BITS    3        // ~1MHz
    #else // MAX6675
      #define MAX_TC_ERROR_MASK    3        // D2 only; 1 = open circuit
      #define MAX_TC_DISCARD_BITS  3        // Data D15-D1
      #define MAX_TC_SPEED_BITS    2        // ~2MHz
    #endif

    #if HAS_MULTI_MAX_TC
      // Needed to return the correct temp when this is called between readings
      static raw_adc_t max_tc_temp_previous[MAX_TC_COUNT] = { 0 };
      #define THERMO_TEMP(I) max_tc_temp_previous[I]
      #if MAX_TC_COUNT > 2
        #define THERMO_SEL(A,B,C) (hindex > 1 ? (C) : hindex == 1 ? (B) : (A))
        #define MAXTC_CS_WRITE(V) do{ switch (hindex) { case 1: WRITE(TEMP_1_CS_PIN, V); break; case 2: WRITE(TEMP_2_CS_PIN, V); break; default: WRITE(TEMP_0_CS_PIN, V); } }while(0)
      #elif MAX_TC_COUNT > 1
        #define THERMO_SEL(A,B,C) ( hindex == 1 ? (B) : (A))
        #define MAXTC_CS_WRITE(V) do{ switch (hindex) { case 1: WRITE(TEMP_1_CS_PIN, V); break; default: WRITE(TEMP_0_CS_PIN, V); } }while(0)
      #endif
    #else
      // When we have only 1 max tc, THERMO_SEL will pick the appropriate sensor
      // variable, and MAXTC_*() macros will be hardcoded to the correct CS pin.
      constexpr uint8_t hindex = 0;
      #define THERMO_TEMP(I) max_tc_temp
      #if TEMP_SENSOR_IS_ANY_MAX_TC(0)
        #define THERMO_SEL(A,B,C) A
        #define MAXTC_CS_WRITE(V)  WRITE(TEMP_0_CS_PIN, V)
      #elif TEMP_SENSOR_IS_ANY_MAX_TC(1)
        #define THERMO_SEL(A,B,C) B
        #define MAXTC_CS_WRITE(V)  WRITE(TEMP_1_CS_PIN, V)
      #elif TEMP_SENSOR_IS_ANY_MAX_TC(2)
        #define THERMO_SEL(A,B,C) C
        #define MAXTC_CS_WRITE(V)  WRITE(TEMP_2_CS_PIN, V)
      #endif
    #endif

    static TERN(HAS_MAX31855, uint32_t, uint16_t) max_tc_temp = THERMO_SEL(
      TEMP_SENSOR_0_MAX_TC_TMAX,
      TEMP_SENSOR_1_MAX_TC_TMAX,
      TEMP_SENSOR_2_MAX_TC_TMAX
    );

    static uint8_t max_tc_errors[MAX_TC_COUNT] = { 0 };
    static millis_t next_max_tc_ms[MAX_TC_COUNT] = { 0 };

    // Return last-read value between readings
    const millis_t ms = millis();
    if (PENDING(ms, next_max_tc_ms[hindex]))
      return THERMO_TEMP(hindex);

    next_max_tc_ms[hindex] = ms + MAXTC_HEAT_INTERVAL;

    #if !HAS_MAXTC_LIBRARIES
      max_tc_temp = 0;

      #if !HAS_MAXTC_SW_SPI
        // Initialize SPI using the default Hardware SPI bus.
        // FIXME: spiBegin, spiRec and spiInit doesn't work when soft spi is used.
        spiBegin();
        spiInit(MAX_TC_SPEED_BITS);
      #endif

      MAXTC_CS_WRITE(LOW);  // Enable MAXTC
      DELAY_NS(100);        // Ensure 100ns delay

      // Read a big-endian temperature value without using a library
      for (uint8_t i = sizeof(max_tc_temp); i--;) {
        max_tc_temp |= TERN(HAS_MAXTC_SW_SPI, max_tc_spi.receive(), spiRec());
        if (i > 0) max_tc_temp <<= 8; // shift left if not the last byte
      }

      MAXTC_CS_WRITE(HIGH);  // Disable MAXTC
    #else
      #if HAS_MAX6675_LIBRARY
        MAX6675 &max6675ref = THERMO_SEL(max6675_0, max6675_1, max6675_2);
        max_tc_temp = max6675ref.readRaw16();
      #endif
      #if HAS_MAX31855_LIBRARY
        MAX31855 &max855ref = THERMO_SEL(max31855_0, max31855_1, max31855_2);
        max_tc_temp = max855ref.readRaw32();
      #endif
      #if HAS_MAX31865
        MAX31865 &max865ref = THERMO_SEL(max31865_0, max31865_1, max31865_2);
        max_tc_temp = TERN(LIB_INTERNAL_MAX31865, max865ref.readRaw(), max865ref.readRTD_with_Fault());
      #endif
    #endif

    // Handle an error. If there have been more than THERMOCOUPLE_MAX_ERRORS, send an error over serial.
    // Either way, return the TMAX for the thermocouple to trigger a maxtemp_error()
    if (max_tc_temp & MAX_TC_ERROR_MASK) {
      max_tc_errors[hindex]++;

      if (max_tc_errors[hindex] > THERMOCOUPLE_MAX_ERRORS) {
        SERIAL_ERROR_START();
        SERIAL_ECHOPGM("Temp measurement error! ");
        #if HAS_MAX31855
          SERIAL_ECHOPGM("MAX31855 Fault: (", max_tc_temp & 0x7, ") >> ");
               if (max_tc_temp & 0x1) SERIAL_ECHOLNPGM("Open Circuit");
          else if (max_tc_temp & 0x2) SERIAL_ECHOLNPGM("Short to GND");
          else if (max_tc_temp & 0x4) SERIAL_ECHOLNPGM("Short to VCC");
        #elif HAS_MAX31865
          const uint8_t fault_31865 = max865ref.readFault();
          max865ref.clearFault();
          if (fault_31865) {
            SERIAL_EOL();
            SERIAL_ECHOLNPGM("\nMAX31865 Fault: (", fault_31865, ")  >>");
            if (fault_31865 & MAX31865_FAULT_HIGHTHRESH) SERIAL_ECHOLNPGM("RTD High Threshold");
            if (fault_31865 & MAX31865_FAULT_LOWTHRESH)  SERIAL_ECHOLNPGM("RTD Low Threshold");
            if (fault_31865 & MAX31865_FAULT_REFINLOW)   SERIAL_ECHOLNPGM("REFIN- > 0.85 x V bias");
            if (fault_31865 & MAX31865_FAULT_REFINHIGH)  SERIAL_ECHOLNPGM("REFIN- < 0.85 x V bias (FORCE- open)");
            if (fault_31865 & MAX31865_FAULT_RTDINLOW)   SERIAL_ECHOLNPGM("REFIN- < 0.85 x V bias (FORCE- open)");
            if (fault_31865 & MAX31865_FAULT_OVUV)       SERIAL_ECHOLNPGM("Under/Over voltage");
          }
        #else // MAX6675
          SERIAL_ECHOLNPGM("MAX6675 Fault: Open Circuit");
        #endif

        // Set thermocouple above max temperature (TMAX)
        max_tc_temp = THERMO_SEL(TEMP_SENSOR_0_MAX_TC_TMAX, TEMP_SENSOR_1_MAX_TC_TMAX, TEMP_SENSOR_2_MAX_TC_TMAX) << (MAX_TC_DISCARD_BITS + 1);
      }
    }
    else {
      max_tc_errors[hindex] = 0; // No error bit, reset error count
    }

    max_tc_temp >>= MAX_TC_DISCARD_BITS;

    #if HAS_MAX31855
      // Support negative temperature for MAX38155
      if (max_tc_temp & 0x00002000) max_tc_temp |= 0xFFFFC000;
    #endif

    THERMO_TEMP(hindex) = max_tc_temp;

    return max_tc_temp;
  }

#endif // HAS_MAX_TC

#if TEMP_SENSOR_IS_MAX_TC(BED)
  /**
   * @brief Read MAX Thermocouple temperature.
   *
   * Reads the thermocouple board via HW or SW SPI, using a library (LIB_USR_x) or raw SPI reads.
   * Doesn't strictly return a temperature; returns an "ADC Value" (i.e. raw register content).
   *
   * @return         integer representing the board's buffer, to be converted later if needed
   */
  raw_adc_t Temperature::read_max_tc_bed() {
    #define MAXTC_HEAT_INTERVAL 250UL

    #if TEMP_SENSOR_BED_IS_MAX31855
      #define BED_MAX_TC_ERROR_MASK    7    // D2-0: SCV, SCG, OC
      #define BED_MAX_TC_DISCARD_BITS 18    // Data D31-18; sign bit D31
      #define BED_MAX_TC_SPEED_BITS    3    // ~1MHz
    #elif TEMP_SENSOR_BED_IS_MAX31865
      #define BED_MAX_TC_ERROR_MASK    1    // D0 Bit on fault only
      #define BED_MAX_TC_DISCARD_BITS  1    // Data is in D15-D1
      #define BED_MAX_TC_SPEED_BITS    3    // ~1MHz
    #else // MAX6675
      #define BED_MAX_TC_ERROR_MASK    3    // D2 only; 1 = open circuit
      #define BED_MAX_TC_DISCARD_BITS  3    // Data D15-D1
      #define BED_MAX_TC_SPEED_BITS    2    // ~2MHz
    #endif

    static max_tc_temp_t max_tc_temp = TEMP_SENSOR_BED_MAX_TC_TMAX;

    static uint8_t max_tc_errors = 0;
    static millis_t next_max_tc_ms = 0;

    // Return last-read value between readings
    const millis_t ms = millis();
    if (PENDING(ms, next_max_tc_ms)) return max_tc_temp;
    next_max_tc_ms = ms + MAXTC_HEAT_INTERVAL;

    #if !HAS_MAXTC_LIBRARIES
      max_tc_temp = 0;

      #if !HAS_MAXTC_SW_SPI
        // Initialize SPI using the default Hardware SPI bus.
        // FIXME: spiBegin, spiRec and spiInit doesn't work when soft spi is used.
        spiBegin();
        spiInit(BED_MAX_TC_SPEED_BITS);
      #endif

      WRITE(TEMP_BED_CS_PIN, LOW);  // Enable MAXTC
      DELAY_NS(100);        // Ensure 100ns delay

      // Read a big-endian temperature value without using a library
      for (uint8_t i = sizeof(max_tc_temp); i--;) {
        max_tc_temp |= TERN(HAS_MAXTC_SW_SPI, max_tc_spi.receive(), spiRec());
        if (i > 0) max_tc_temp <<= 8; // shift left if not the last byte
      }

      WRITE(TEMP_BED_CS_PIN, HIGH);  // Disable MAXTC

    #elif ALL(TEMP_SENSOR_BED_IS_MAX6675, HAS_MAX6675_LIBRARY)
      MAX6675 &max6675ref = max6675_BED;
      max_tc_temp = max6675ref.readRaw16();
    #elif ALL(TEMP_SENSOR_BED_IS_MAX31855, HAS_MAX31855_LIBRARY)
      MAX31855 &max855ref = max31855_BED;
      max_tc_temp = max855ref.readRaw32();
    #elif TEMP_SENSOR_BED_IS_MAX31865
      MAX31865 &max865ref = max31865_BED;
      max_tc_temp = TERN(LIB_INTERNAL_MAX31865, max865ref.readRaw(), max865ref.readRTD_with_Fault());
    #endif

    // Handle an error. If there have been more than THERMOCOUPLE_MAX_ERRORS, send an error over serial.
    // Either way, return the TMAX for the thermocouple to trigger a maxtemp_error()
    if (max_tc_temp & BED_MAX_TC_ERROR_MASK) {
      max_tc_errors++;

      if (max_tc_errors > THERMOCOUPLE_MAX_ERRORS) {
        SERIAL_ERROR_START();
        SERIAL_ECHOPGM("Bed temp measurement error! ");
        #if TEMP_SENSOR_BED_IS_MAX31855
          SERIAL_ECHOPGM("MAX31855 Fault: (", max_tc_temp & 0x7, ") >> ");
               if (max_tc_temp & 0x1) SERIAL_ECHOLNPGM("Open Circuit");
          else if (max_tc_temp & 0x2) SERIAL_ECHOLNPGM("Short to GND");
          else if (max_tc_temp & 0x4) SERIAL_ECHOLNPGM("Short to VCC");
        #elif TEMP_SENSOR_BED_IS_MAX31865
          const uint8_t fault_31865 = max865ref.readFault();
          max865ref.clearFault();
          if (fault_31865) {
            SERIAL_EOL();
            SERIAL_ECHOLNPGM("\nMAX31865 Fault: (", fault_31865, ")  >>");
            if (fault_31865 & MAX31865_FAULT_HIGHTHRESH) SERIAL_ECHOLNPGM("RTD High Threshold");
            if (fault_31865 & MAX31865_FAULT_LOWTHRESH)  SERIAL_ECHOLNPGM("RTD Low Threshold");
            if (fault_31865 & MAX31865_FAULT_REFINLOW)   SERIAL_ECHOLNPGM("REFIN- > 0.85 x V bias");
            if (fault_31865 & MAX31865_FAULT_REFINHIGH)  SERIAL_ECHOLNPGM("REFIN- < 0.85 x V bias (FORCE- open)");
            if (fault_31865 & MAX31865_FAULT_RTDINLOW)   SERIAL_ECHOLNPGM("REFIN- < 0.85 x V bias (FORCE- open)");
            if (fault_31865 & MAX31865_FAULT_OVUV)       SERIAL_ECHOLNPGM("Under/Over voltage");
          }
        #else // MAX6675
          SERIAL_ECHOLNPGM("MAX6675 Fault: Open Circuit");
        #endif

        // Set thermocouple above max temperature (TMAX)
        max_tc_temp = max_tc_temp_t(TEMP_SENSOR_BED_MAX_TC_TMAX) << (BED_MAX_TC_DISCARD_BITS + 1);
      }
    }
    else {
      max_tc_errors = 0; // No error bit, reset error count
    }

    max_tc_temp >>= BED_MAX_TC_DISCARD_BITS;

    #if TEMP_SENSOR_BED_IS_MAX31855
      // Support negative temperature for MAX38155
      if (max_tc_temp & 0x00002000) max_tc_temp |= 0xFFFFC000;
    #endif

    return max_tc_temp;
  }

#endif // TEMP_SENSOR_IS_MAX_TC(BED)

/**
 * Update raw temperatures
 *
 * Called by ISR => readings_ready when new temperatures have been set by updateTemperaturesFromRawValues.
 * Applies all the accumulators to the current raw temperatures.
 */
void Temperature::update_raw_temperatures() {

  // TODO: can this be collapsed into a HOTEND_LOOP()?
  #if HAS_TEMP_ADC_0 && !TEMP_SENSOR_IS_MAX_TC(0)
    temp_hotend[0].update();
  #endif

  #if HAS_TEMP_ADC_1 && !TEMP_SENSOR_IS_MAX_TC(1)
    temp_hotend[1].update();
  #endif

  #if HAS_TEMP_ADC_2 && !TEMP_SENSOR_IS_MAX_TC(2)
    temp_hotend[2].update();
  #endif

  #if HAS_TEMP_ADC_REDUNDANT && !TEMP_SENSOR_IS_MAX_TC(REDUNDANT)
    temp_redundant.update();
  #endif

  #if HAS_TEMP_ADC_BED && !TEMP_SENSOR_IS_MAX_TC(BED)
    temp_bed.update();
  #endif

  TERN_(HAS_TEMP_ADC_3,       temp_hotend[3].update());
  TERN_(HAS_TEMP_ADC_4,       temp_hotend[4].update());
  TERN_(HAS_TEMP_ADC_5,       temp_hotend[5].update());
  TERN_(HAS_TEMP_ADC_6,       temp_hotend[6].update());
  TERN_(HAS_TEMP_ADC_7,       temp_hotend[7].update());
  TERN_(HAS_TEMP_ADC_CHAMBER, temp_chamber.update());
  TERN_(HAS_TEMP_ADC_PROBE,   temp_probe.update());
  TERN_(HAS_TEMP_ADC_COOLER,  temp_cooler.update());
  TERN_(HAS_TEMP_ADC_BOARD,   temp_board.update());
  TERN_(HAS_TEMP_ADC_SOC,     temp_soc.update());

  TERN_(HAS_JOY_ADC_X, joystick.x.update());
  TERN_(HAS_JOY_ADC_Y, joystick.y.update());
  TERN_(HAS_JOY_ADC_Z, joystick.z.update());
}

/**
 * Called by the Temperature ISR when all the ADCs have been processed.
 * Reset all the ADC accumulators for another round of updates.
 */
void Temperature::readings_ready() {

  // Update raw values only if they're not already set.
  if (!raw_temps_ready) {
    update_raw_temperatures();
    raw_temps_ready = true;
  }

  // Filament Sensor - can be read any time since IIR filtering is used
  TERN_(FILAMENT_WIDTH_SENSOR, filwidth.reading_ready());

  #if HAS_HOTEND
    HOTEND_LOOP() temp_hotend[e].reset();
  #endif

  TERN_(HAS_HEATED_BED,     temp_bed.reset());
  TERN_(HAS_TEMP_CHAMBER,   temp_chamber.reset());
  TERN_(HAS_TEMP_PROBE,     temp_probe.reset());
  TERN_(HAS_TEMP_COOLER,    temp_cooler.reset());
  TERN_(HAS_TEMP_BOARD,     temp_board.reset());
  TERN_(HAS_TEMP_SOC,       temp_soc.reset());
  TERN_(HAS_TEMP_REDUNDANT, temp_redundant.reset());

  TERN_(HAS_JOY_ADC_X, joystick.x.reset());
  TERN_(HAS_JOY_ADC_Y, joystick.y.reset());
  TERN_(HAS_JOY_ADC_Z, joystick.z.reset());
}

/**
 * Timer 0 is shared with millies so don't change the prescaler.
 *
 * On AVR this ISR uses the compare method so it runs at the base
 * frequency (16 MHz / 64 / 256 = 976.5625 Hz), but at the TCNT0 set
 * in OCR0B above (128 or halfway between OVFs).
 *
 *  - Manage PWM to all the heaters and fan
 *  - Prepare or Measure one of the raw ADC sensor values
 *  - Check new temperature values for MIN/MAX errors (kill on error)
 *  - Step the babysteps value for each axis towards 0
 *  - For PINS_DEBUGGING, monitor and report endstop pins
 *  - For ENDSTOP_INTERRUPTS_FEATURE check endstops if flagged
 *  - Call planner.isr to count down its "ignore" time
 */
HAL_TEMP_TIMER_ISR() {
  HAL_timer_isr_prologue(MF_TIMER_TEMP);

  Temperature::isr();

  HAL_timer_isr_epilogue(MF_TIMER_TEMP);
}

#if ENABLED(SLOW_PWM_HEATERS) && !defined(MIN_STATE_TIME)
  #define MIN_STATE_TIME 16 // MIN_STATE_TIME * 65.5 = time in milliseconds
#endif

class SoftPWM {
public:
  uint8_t count;
  inline bool add(const uint8_t mask, const uint8_t amount) {
    count = (count & mask) + amount; return (count > mask);
  }
  #if ENABLED(SLOW_PWM_HEATERS)
    bool state_heater;
    uint8_t state_timer_heater;
    inline void dec() { if (state_timer_heater > 0) state_timer_heater--; }
    inline bool ready(const bool v) {
      const bool rdy = !state_timer_heater;
      if (rdy && state_heater != v) {
        state_heater = v;
        state_timer_heater = MIN_STATE_TIME;
      }
      return rdy;
    }
  #endif
};

/**
 * Handle various ~1kHz tasks associated with temperature
 *  - Check laser safety timeout
 *  - Heater PWM (~1kHz with scaler)
 *  - LCD Button polling (~500Hz)
 *  - Start / Read one ADC sensor
 *  - Advance Babysteps
 *  - Endstop polling
 *  - Planner clean buffer
 */
void Temperature::isr() {

  // Shut down the laser if steppers are inactive for > LASER_SAFETY_TIMEOUT_MS ms
  #if LASER_SAFETY_TIMEOUT_MS > 0
    if (cutter.last_power_applied && ELAPSED(millis(), gcode.previous_move_ms + (LASER_SAFETY_TIMEOUT_MS))) {
      cutter.power = 0;       // Prevent planner idle from re-enabling power
      cutter.apply_power(0);
    }
  #endif

  static int8_t temp_count = -1;
  static ADCSensorState adc_sensor_state = StartupDelay;

  #ifndef SOFT_PWM_SCALE
    #define SOFT_PWM_SCALE 0
  #endif
  static uint8_t pwm_count = _BV(SOFT_PWM_SCALE);

  // Avoid multiple loads of pwm_count
  uint8_t pwm_count_tmp = pwm_count;

  #if HAS_ADC_BUTTONS
    static raw_adc_t raw_ADCKey_value = 0;
    static bool ADCKey_pressed = false;
  #endif

  #if HAS_HOTEND
    static SoftPWM soft_pwm_hotend[HOTENDS];
  #endif

  #if HAS_HEATED_BED
    static SoftPWM soft_pwm_bed;
  #endif

  #if HAS_HEATED_CHAMBER
    static SoftPWM soft_pwm_chamber;
  #endif

  #if HAS_COOLER
    static SoftPWM soft_pwm_cooler;
  #endif

  #if ALL(FAN_SOFT_PWM, USE_CONTROLLER_FAN)
    static SoftPWM soft_pwm_controller;
  #endif

  #define WRITE_FAN(n, v) WRITE(FAN##n##_PIN, (v) ^ ENABLED(FAN_INVERTING))

  #if DISABLED(SLOW_PWM_HEATERS)

    #if ANY(HAS_HOTEND, HAS_HEATED_BED, HAS_HEATED_CHAMBER, HAS_COOLER, FAN_SOFT_PWM)
      constexpr uint8_t pwm_mask = TERN0(SOFT_PWM_DITHER, _BV(SOFT_PWM_SCALE) - 1);
      #define _PWM_MOD(N,S,T) do{                           \
        const bool on = S.add(pwm_mask, T.soft_pwm_amount); \
        WRITE_HEATER_##N(on);                               \
      }while(0)
    #endif

    /**
     * Standard heater PWM modulation
     */
    if (pwm_count_tmp >= 127) {
      pwm_count_tmp -= 127;

      #if HAS_HOTEND
        #define _PWM_MOD_E(N) _PWM_MOD(N,soft_pwm_hotend[N],temp_hotend[N]);
        REPEAT(HOTENDS, _PWM_MOD_E);
      #endif

      #if HAS_HEATED_BED
        _PWM_MOD(BED, soft_pwm_bed, temp_bed);
        #if ENABLED(PELTIER_BED)
          WRITE_PELTIER_DIR(temp_bed.peltier_dir_heating);
        #endif
      #endif

      #if HAS_HEATED_CHAMBER
        _PWM_MOD(CHAMBER, soft_pwm_chamber, temp_chamber);
      #endif

      #if HAS_COOLER
        _PWM_MOD(COOLER, soft_pwm_cooler, temp_cooler);
      #endif

      #if ENABLED(FAN_SOFT_PWM)

        #if ENABLED(USE_CONTROLLER_FAN)
          WRITE(CONTROLLER_FAN_PIN, soft_pwm_controller.add(pwm_mask, controllerFan.soft_pwm_speed));
        #endif

        #define _FAN_PWM(N) do{                                     \
          uint8_t &spcf = soft_pwm_count_fan[N];                    \
          spcf = (spcf & pwm_mask) + (soft_pwm_amount_fan[N] >> 1); \
          WRITE_FAN(N, spcf > pwm_mask ? HIGH : LOW);               \
        }while(0)

        #if HAS_FAN0
          _FAN_PWM(0);
        #endif
        #if HAS_FAN1
          _FAN_PWM(1);
        #endif
        #if HAS_FAN2
          _FAN_PWM(2);
        #endif
        #if HAS_FAN3
          _FAN_PWM(3);
        #endif
        #if HAS_FAN4
          _FAN_PWM(4);
        #endif
        #if HAS_FAN5
          _FAN_PWM(5);
        #endif
        #if HAS_FAN6
          _FAN_PWM(6);
        #endif
        #if HAS_FAN7
          _FAN_PWM(7);
        #endif
      #endif
    }
    else {
      #define _PWM_LOW(N,S) do{ if (S.count <= pwm_count_tmp) WRITE_HEATER_##N(LOW); }while(0)
      #if HAS_HOTEND
        #define _PWM_LOW_E(N) _PWM_LOW(N, soft_pwm_hotend[N]);
        REPEAT(HOTENDS, _PWM_LOW_E);
      #endif

      #if HAS_HEATED_BED
        _PWM_LOW(BED, soft_pwm_bed);
      #endif

      #if HAS_HEATED_CHAMBER
        _PWM_LOW(CHAMBER, soft_pwm_chamber);
      #endif

      #if HAS_COOLER
        _PWM_LOW(COOLER, soft_pwm_cooler);
      #endif

      #if ENABLED(FAN_SOFT_PWM)
        #if HAS_FAN0
          if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(0, LOW);
        #endif
        #if HAS_FAN1
          if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN(1, LOW);
        #endif
        #if HAS_FAN2
          if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN(2, LOW);
        #endif
        #if HAS_FAN3
          if (soft_pwm_count_fan[3] <= pwm_count_tmp) WRITE_FAN(3, LOW);
        #endif
        #if HAS_FAN4
          if (soft_pwm_count_fan[4] <= pwm_count_tmp) WRITE_FAN(4, LOW);
        #endif
        #if HAS_FAN5
          if (soft_pwm_count_fan[5] <= pwm_count_tmp) WRITE_FAN(5, LOW);
        #endif
        #if HAS_FAN6
          if (soft_pwm_count_fan[6] <= pwm_count_tmp) WRITE_FAN(6, LOW);
        #endif
        #if HAS_FAN7
          if (soft_pwm_count_fan[7] <= pwm_count_tmp) WRITE_FAN(7, LOW);
        #endif
        #if ENABLED(USE_CONTROLLER_FAN)
          if (soft_pwm_controller.count <= pwm_count_tmp) WRITE(CONTROLLER_FAN_PIN, LOW);
        #endif
      #endif
    }

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

  #else // SLOW_PWM_HEATERS

    /**
     * SLOW PWM HEATERS
     *
     * For relay-driven heaters
     */
    #define _SLOW_SET(NR,PWM,V) do{ if (PWM.ready(V)) WRITE_HEATER_##NR(V); }while(0)
    #define _SLOW_PWM(NR,PWM,SRC) do{ PWM.count = SRC.soft_pwm_amount; _SLOW_SET(NR,PWM,(PWM.count > 0)); }while(0)
    #define _PWM_OFF(NR,PWM) do{ if (PWM.count < slow_pwm_count) _SLOW_SET(NR,PWM,0); }while(0)

    static uint8_t slow_pwm_count = 0;

    if (slow_pwm_count == 0) {

      #if HAS_HOTEND
        #define _SLOW_PWM_E(N) _SLOW_PWM(N, soft_pwm_hotend[N], temp_hotend[N]);
        REPEAT(HOTENDS, _SLOW_PWM_E);
      #endif

      #if HAS_HEATED_BED
        _SLOW_PWM(BED, soft_pwm_bed, temp_bed);
      #endif

      #if HAS_HEATED_CHAMBER
        _SLOW_PWM(CHAMBER, soft_pwm_chamber, temp_chamber);
      #endif

      #if HAS_COOLER
        _SLOW_PWM(COOLER, soft_pwm_cooler, temp_cooler);
      #endif

    } // slow_pwm_count == 0

    #if HAS_HOTEND
      #define _PWM_OFF_E(N) _PWM_OFF(N, soft_pwm_hotend[N]);
      REPEAT(HOTENDS, _PWM_OFF_E);
    #endif

    #if HAS_HEATED_BED
      _PWM_OFF(BED, soft_pwm_bed);
    #endif

    #if HAS_HEATED_CHAMBER
      _PWM_OFF(CHAMBER, soft_pwm_chamber);
    #endif

    #if HAS_COOLER
      _PWM_OFF(COOLER, soft_pwm_cooler, temp_cooler);
    #endif

    #if ENABLED(FAN_SOFT_PWM)
      if (pwm_count_tmp >= 127) {
        pwm_count_tmp = 0;
        #define _PWM_FAN(N) do{                                 \
          soft_pwm_count_fan[N] = soft_pwm_amount_fan[N] >> 1;  \
          WRITE_FAN(N, soft_pwm_count_fan[N] > 0 ? HIGH : LOW); \
        }while(0)
        #if HAS_FAN0
          _PWM_FAN(0);
        #endif
        #if HAS_FAN1
          _PWM_FAN(1);
        #endif
        #if HAS_FAN2
          _PWM_FAN(2);
        #endif
        #if HAS_FAN3
          _FAN_PWM(3);
        #endif
        #if HAS_FAN4
          _FAN_PWM(4);
        #endif
        #if HAS_FAN5
          _FAN_PWM(5);
        #endif
        #if HAS_FAN6
          _FAN_PWM(6);
        #endif
        #if HAS_FAN7
          _FAN_PWM(7);
        #endif
      }
      #if HAS_FAN0
        if (soft_pwm_count_fan[0] <= pwm_count_tmp) WRITE_FAN(0, LOW);
      #endif
      #if HAS_FAN1
        if (soft_pwm_count_fan[1] <= pwm_count_tmp) WRITE_FAN(1, LOW);
      #endif
      #if HAS_FAN2
        if (soft_pwm_count_fan[2] <= pwm_count_tmp) WRITE_FAN(2, LOW);
      #endif
      #if HAS_FAN3
        if (soft_pwm_count_fan[3] <= pwm_count_tmp) WRITE_FAN(3, LOW);
      #endif
      #if HAS_FAN4
        if (soft_pwm_count_fan[4] <= pwm_count_tmp) WRITE_FAN(4, LOW);
      #endif
      #if HAS_FAN5
        if (soft_pwm_count_fan[5] <= pwm_count_tmp) WRITE_FAN(5, LOW);
      #endif
      #if HAS_FAN6
        if (soft_pwm_count_fan[6] <= pwm_count_tmp) WRITE_FAN(6, LOW);
      #endif
      #if HAS_FAN7
        if (soft_pwm_count_fan[7] <= pwm_count_tmp) WRITE_FAN(7, LOW);
      #endif
    #endif // FAN_SOFT_PWM

    // SOFT_PWM_SCALE to frequency:
    //
    // 0: 16000000/64/256/128 =   7.6294 Hz
    // 1:                / 64 =  15.2588 Hz
    // 2:                / 32 =  30.5176 Hz
    // 3:                / 16 =  61.0352 Hz
    // 4:                /  8 = 122.0703 Hz
    // 5:                /  4 = 244.1406 Hz
    pwm_count = pwm_count_tmp + _BV(SOFT_PWM_SCALE);

    // Increment slow_pwm_count only every 64th pwm_count,
    // i.e., yielding a PWM frequency of 16/128 Hz (8s).
    if (((pwm_count >> SOFT_PWM_SCALE) & 0x3F) == 0) {
      slow_pwm_count++;
      slow_pwm_count &= 0x7F;

      #if HAS_HOTEND
        HOTEND_LOOP() soft_pwm_hotend[e].dec();
      #endif
      TERN_(HAS_HEATED_BED, soft_pwm_bed.dec());
      TERN_(HAS_HEATED_CHAMBER, soft_pwm_chamber.dec());
      TERN_(HAS_COOLER, soft_pwm_cooler.dec());
    }

  #endif // SLOW_PWM_HEATERS

  //
  // Update lcd buttons 488 times per second
  //
  static bool do_buttons;
  if ((do_buttons ^= true)) ui.update_buttons();

  /**
   * One sensor is sampled on every other call of the ISR.
   * Each sensor is read 16 (OVERSAMPLENR) times, taking the average.
   *
   * On each Prepare pass, ADC is started for a sensor pin.
   * On the next pass, the ADC value is read and accumulated.
   *
   * This gives each ADC 0.9765ms to charge up.
   */
  #define ACCUMULATE_ADC(obj) do{ \
    if (!hal.adc_ready()) next_sensor_state = adc_sensor_state; \
    else obj.sample(hal.adc_value()); \
  }while(0)

  ADCSensorState next_sensor_state = adc_sensor_state < SensorsReady ? (ADCSensorState)(int(adc_sensor_state) + 1) : StartSampling;

  switch (adc_sensor_state) {

    #pragma GCC diagnostic push
    #if __has_cpp_attribute(fallthrough)
      #pragma GCC diagnostic ignored "-Wimplicit-fallthrough"
    #endif

    case SensorsReady: {
      // All sensors have been read. Stay in this state for a few
      // ISRs to save on calls to temp update/checking code below.
      constexpr int8_t extra_loops = MIN_ADC_ISR_LOOPS - (int8_t)SensorsReady;
      static uint8_t delay_count = 0;
      if (extra_loops > 0) {
        if (delay_count == 0) delay_count = extra_loops;  // Init this delay
        if (--delay_count)                                // While delaying...
          next_sensor_state = SensorsReady;               // retain this state (else, next state will be 0)
        break;
      }
      else {
        adc_sensor_state = StartSampling;                 // Fall-through to start sampling
        next_sensor_state = (ADCSensorState)(int(StartSampling) + 1);
      }
    }

    #pragma GCC diagnostic pop

    case StartSampling:                                   // Start of sampling loops. Do updates/checks.
      if (++temp_count >= OVERSAMPLENR) {                 // 10 * 16 * 1/(16000000/64/256)  = 164ms.
        temp_count = 0;
        readings_ready();
      }
      break;

    #if HAS_TEMP_ADC_0
      case PrepareTemp_0: hal.adc_start(TEMP_0_PIN); break;
      case MeasureTemp_0: ACCUMULATE_ADC(temp_hotend[0]); break;
    #endif

    #if HAS_TEMP_ADC_BED
      case PrepareTemp_BED: hal.adc_start(TEMP_BED_PIN); break;
      case MeasureTemp_BED: ACCUMULATE_ADC(temp_bed); break;
    #endif

    #if HAS_TEMP_ADC_CHAMBER
      case PrepareTemp_CHAMBER: hal.adc_start(TEMP_CHAMBER_PIN); break;
      case MeasureTemp_CHAMBER: ACCUMULATE_ADC(temp_chamber); break;
    #endif

    #if HAS_TEMP_ADC_COOLER
      case PrepareTemp_COOLER: hal.adc_start(TEMP_COOLER_PIN); break;
      case MeasureTemp_COOLER: ACCUMULATE_ADC(temp_cooler); break;
    #endif

    #if HAS_TEMP_ADC_PROBE
      case PrepareTemp_PROBE: hal.adc_start(TEMP_PROBE_PIN); break;
      case MeasureTemp_PROBE: ACCUMULATE_ADC(temp_probe); break;
    #endif

    #if HAS_TEMP_ADC_BOARD
      case PrepareTemp_BOARD: hal.adc_start(TEMP_BOARD_PIN); break;
      case MeasureTemp_BOARD: ACCUMULATE_ADC(temp_board); break;
    #endif

    #if HAS_TEMP_ADC_SOC
      case PrepareTemp_SOC: hal.adc_start(TEMP_SOC_PIN); break;
      case MeasureTemp_SOC: ACCUMULATE_ADC(temp_soc); break;
    #endif

    #if HAS_TEMP_ADC_REDUNDANT
      case PrepareTemp_REDUNDANT: hal.adc_start(TEMP_REDUNDANT_PIN); break;
      case MeasureTemp_REDUNDANT: ACCUMULATE_ADC(temp_redundant); break;
    #endif

    #if HAS_TEMP_ADC_1
      case PrepareTemp_1: hal.adc_start(TEMP_1_PIN); break;
      case MeasureTemp_1: ACCUMULATE_ADC(temp_hotend[1]); break;
    #endif

    #if HAS_TEMP_ADC_2
      case PrepareTemp_2: hal.adc_start(TEMP_2_PIN); break;
      case MeasureTemp_2: ACCUMULATE_ADC(temp_hotend[2]); break;
    #endif

    #if HAS_TEMP_ADC_3
      case PrepareTemp_3: hal.adc_start(TEMP_3_PIN); break;
      case MeasureTemp_3: ACCUMULATE_ADC(temp_hotend[3]); break;
    #endif

    #if HAS_TEMP_ADC_4
      case PrepareTemp_4: hal.adc_start(TEMP_4_PIN); break;
      case MeasureTemp_4: ACCUMULATE_ADC(temp_hotend[4]); break;
    #endif

    #if HAS_TEMP_ADC_5
      case PrepareTemp_5: hal.adc_start(TEMP_5_PIN); break;
      case MeasureTemp_5: ACCUMULATE_ADC(temp_hotend[5]); break;
    #endif

    #if HAS_TEMP_ADC_6
      case PrepareTemp_6: hal.adc_start(TEMP_6_PIN); break;
      case MeasureTemp_6: ACCUMULATE_ADC(temp_hotend[6]); break;
    #endif

    #if HAS_TEMP_ADC_7
      case PrepareTemp_7: hal.adc_start(TEMP_7_PIN); break;
      case MeasureTemp_7: ACCUMULATE_ADC(temp_hotend[7]); break;
    #endif

    #if ENABLED(FILAMENT_WIDTH_SENSOR)
      case Prepare_FILWIDTH: hal.adc_start(FILWIDTH_PIN); break;
      case Measure_FILWIDTH:
        if (!hal.adc_ready()) next_sensor_state = adc_sensor_state; // Redo this state
        else filwidth.accumulate(hal.adc_value());
      break;
    #endif

    #if ENABLED(POWER_MONITOR_CURRENT)
      case Prepare_POWER_MONITOR_CURRENT:
        hal.adc_start(POWER_MONITOR_CURRENT_PIN);
        break;
      case Measure_POWER_MONITOR_CURRENT:
        if (!hal.adc_ready()) next_sensor_state = adc_sensor_state; // Redo this state
        else power_monitor.add_current_sample(hal.adc_value());
        break;
    #endif

    #if ENABLED(POWER_MONITOR_VOLTAGE)
      case Prepare_POWER_MONITOR_VOLTAGE:
        hal.adc_start(POWER_MONITOR_VOLTAGE_PIN);
        break;
      case Measure_POWER_MONITOR_VOLTAGE:
        if (!hal.adc_ready()) next_sensor_state = adc_sensor_state; // Redo this state
        else power_monitor.add_voltage_sample(hal.adc_value());
        break;
    #endif

    #if HAS_JOY_ADC_X
      case PrepareJoy_X: hal.adc_start(JOY_X_PIN); break;
      case MeasureJoy_X: ACCUMULATE_ADC(joystick.x); break;
    #endif

    #if HAS_JOY_ADC_Y
      case PrepareJoy_Y: hal.adc_start(JOY_Y_PIN); break;
      case MeasureJoy_Y: ACCUMULATE_ADC(joystick.y); break;
    #endif

    #if HAS_JOY_ADC_Z
      case PrepareJoy_Z: hal.adc_start(JOY_Z_PIN); break;
      case MeasureJoy_Z: ACCUMULATE_ADC(joystick.z); break;
    #endif

    #if HAS_ADC_BUTTONS
      #ifndef ADC_BUTTON_DEBOUNCE_DELAY
        #define ADC_BUTTON_DEBOUNCE_DELAY 16
      #endif
      case Prepare_ADC_KEY: hal.adc_start(ADC_KEYPAD_PIN); break;
      case Measure_ADC_KEY:
        if (!hal.adc_ready())
          next_sensor_state = adc_sensor_state; // redo this state
        else if (ADCKey_count < ADC_BUTTON_DEBOUNCE_DELAY) {
          raw_ADCKey_value = hal.adc_value();
          if (raw_ADCKey_value <= 900UL * HAL_ADC_RANGE / 1024UL) {
            NOMORE(current_ADCKey_raw, raw_ADCKey_value);
            ADCKey_count++;
          }
          else { //ADC Key release
            if (ADCKey_count > 0) ADCKey_count++; else ADCKey_pressed = false;
            if (ADCKey_pressed) {
              ADCKey_count = 0;
              current_ADCKey_raw = HAL_ADC_RANGE;
            }
          }
        }
        if (ADCKey_count == ADC_BUTTON_DEBOUNCE_DELAY) ADCKey_pressed = true;
        break;
    #endif // HAS_ADC_BUTTONS

    case StartupDelay: break;

  } // switch(adc_sensor_state)

  // Go to the next state
  adc_sensor_state = next_sensor_state;

  //
  // Additional ~1kHz Tasks
  //

  // Check fan tachometers
  TERN_(HAS_FANCHECK, fan_check.update_tachometers());

  // Poll endstops state, if required
  endstops.poll();

  // Periodically call the planner timer service routine
  planner.isr();
}

#if HAS_TEMP_SENSOR
  /**
   * Print a single heater state in the form:
   *     Extruder: " T0:nnn.nn /nnn.nn"
   *          Bed: " B:nnn.nn /nnn.nn"
   *      Chamber: " C:nnn.nn /nnn.nn"
   *       Cooler: " L:nnn.nn /nnn.nn"
   *        Probe: " P:nnn.nn"
   *        Board: " M:nnn.nn"
   *          SoC: " S:nnn.nn"
   *    Redundant: " R:nnn.nn /nnn.nn"
   *     With ADC: " T0:nnn.nn /nnn.nn (nnn.nn)"
   */
  static void print_heater_state(const heater_id_t e, const_celsius_float_t c, const_celsius_float_t t
    OPTARG(SHOW_TEMP_ADC_VALUES, const float r)
  ) {
    char k;
    bool show_t = true;
    switch (e) {
      default:
        #if HAS_TEMP_HOTEND
          k = 'T'; break;
        #endif
      #if HAS_TEMP_BED
        case H_BED: k = 'B'; break;
      #endif
      #if HAS_TEMP_CHAMBER
        case H_CHAMBER: k = 'C'; break;
      #endif
      #if HAS_TEMP_COOLER
        case H_COOLER: k = 'L'; break;
      #endif
      #if HAS_TEMP_PROBE
        case H_PROBE: k = 'P'; show_t = false; break;
      #endif
      #if HAS_TEMP_BOARD
        case H_BOARD: k = 'M'; show_t = false; break;
      #endif
      #if HAS_TEMP_SOC
        case H_SOC: k = 'S'; show_t = false; break;
      #endif
      #if HAS_TEMP_REDUNDANT
        case H_REDUNDANT: k = 'R'; break;
      #endif
    }
    #ifndef HEATER_STATE_FLOAT_PRECISION
      #define HEATER_STATE_FLOAT_PRECISION _MIN(SERIAL_FLOAT_PRECISION, 2)
    #endif

    SString<50> s(' ', k);
    if (TERN0(HAS_MULTI_HOTEND, e >= 0)) s += char('0' + e);
    s += ':'; s += p_float_t(c, HEATER_STATE_FLOAT_PRECISION);
    if (show_t) { s += F(" /"); s += p_float_t(t, HEATER_STATE_FLOAT_PRECISION); }
    #if ENABLED(SHOW_TEMP_ADC_VALUES)
      // Temperature MAX SPI boards do not have an OVERSAMPLENR defined
      s.append(F(" ("), TERN(HAS_MAXTC_LIBRARIES, k == 'T', false) ? r : r * RECIPROCAL(OVERSAMPLENR), ')');
    #endif
    s.echo();
    delay(2);
  }

  /**
   * Print all heater states followed by power data on a single line.
   * See print_heater_state for heater output strings.
   * Power output strings are in the format:
   *     Extruder: " @:nnn"
   *          Bed: " B@:nnn"
   *      Peltier: " P@:H/C"
   *      Chamber: " C@:nnn"
   *       Cooler: " L@:nnn"
   *      Hotends: " @0:nnn @1:nnn ..."
   */
  void Temperature::print_heater_states(const int8_t target_extruder
    OPTARG(HAS_TEMP_REDUNDANT, const bool include_r/*=false*/)
  ) {
    #if HAS_TEMP_HOTEND
      print_heater_state(H_NONE, degHotend(target_extruder), degTargetHotend(target_extruder) OPTARG(SHOW_TEMP_ADC_VALUES, rawHotendTemp(target_extruder)));
    #endif
    #if HAS_HEATED_BED
      print_heater_state(H_BED, degBed(), degTargetBed() OPTARG(SHOW_TEMP_ADC_VALUES, rawBedTemp()));
    #endif
    #if HAS_TEMP_CHAMBER
      print_heater_state(H_CHAMBER, degChamber(), TERN0(HAS_HEATED_CHAMBER, degTargetChamber()) OPTARG(SHOW_TEMP_ADC_VALUES, rawChamberTemp()));
    #endif
    #if HAS_TEMP_COOLER
      print_heater_state(H_COOLER, degCooler(), TERN0(HAS_COOLER, degTargetCooler()) OPTARG(SHOW_TEMP_ADC_VALUES, rawCoolerTemp()));
    #endif
    #if HAS_TEMP_PROBE
      print_heater_state(H_PROBE, degProbe(), 0 OPTARG(SHOW_TEMP_ADC_VALUES, rawProbeTemp()));
    #endif
    #if HAS_TEMP_BOARD
      print_heater_state(H_BOARD, degBoard(), 0 OPTARG(SHOW_TEMP_ADC_VALUES, rawBoardTemp()));
    #endif
    #if HAS_TEMP_SOC
      print_heater_state(H_SOC, degSoc(), 0 OPTARG(SHOW_TEMP_ADC_VALUES, rawSocTemp()));
    #endif
    #if HAS_TEMP_REDUNDANT
      if (include_r) print_heater_state(H_REDUNDANT, degRedundant(), degRedundantTarget() OPTARG(SHOW_TEMP_ADC_VALUES, rawRedundantTemp()));
    #endif
    #if HAS_MULTI_HOTEND
      HOTEND_LOOP() print_heater_state((heater_id_t)e, degHotend(e), degTargetHotend(e) OPTARG(SHOW_TEMP_ADC_VALUES, rawHotendTemp(e)));
    #endif
    SString<100> s(F(" @:"), getHeaterPower((heater_id_t)target_extruder));
    TERN_(HAS_HEATED_BED,     s.append(F(" B@:"), getHeaterPower(H_BED)));
    TERN_(PELTIER_BED,        s.append(F(" P@:"), temp_bed.peltier_dir_heating ? 'H' : 'C'));
    TERN_(HAS_HEATED_CHAMBER, s.append(F(" C@:"), getHeaterPower(H_CHAMBER)));
    TERN_(HAS_COOLER,         s.append(F(" L@:"), getHeaterPower(H_COOLER)));
    #if HAS_MULTI_HOTEND
      HOTEND_LOOP() s.append(F(" @"), e, ':', getHeaterPower((heater_id_t)e));
    #endif
    s.echo();
  }

  #if ENABLED(AUTO_REPORT_TEMPERATURES)
    AutoReporter<Temperature::AutoReportTemp> Temperature::auto_reporter;
    void Temperature::AutoReportTemp::report() {
      if (wait_for_heatup) return;
      print_heater_states(active_extruder OPTARG(HAS_TEMP_REDUNDANT, ENABLED(AUTO_REPORT_REDUNDANT)));
      SERIAL_EOL();
    }
  #endif

  #if HAS_HOTEND && HAS_STATUS_MESSAGE
    void Temperature::set_heating_message(const uint8_t e, const bool isM104/*=false*/) {
      const bool heating = isHeatingHotend(e);
      ui.status_printf(0,
        #if HAS_MULTI_HOTEND
          F("E%c " S_FMT), '1' + e
        #else
          F("E1 " S_FMT)
        #endif
        , heating ? GET_TEXT_F(MSG_HEATING) : GET_TEXT_F(MSG_COOLING)
      );

      if (isM104) {
        static uint8_t wait_e; wait_e = e;
        ui.set_status_reset_fn([]{
          const celsius_t c = degTargetHotend(wait_e);
          return c < 30 || degHotendNear(wait_e, c);
        });
      }
    }
  #endif

  #if HAS_TEMP_HOTEND

    #ifndef MIN_COOLING_SLOPE_DEG
      #define MIN_COOLING_SLOPE_DEG 1.50
    #endif
    #ifndef MIN_COOLING_SLOPE_TIME
      #define MIN_COOLING_SLOPE_TIME 60
    #endif

    bool Temperature::wait_for_hotend(const uint8_t target_extruder, const bool no_wait_for_cooling/*=true*/
      OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel/*=false*/)
    ) {
      #if ENABLED(AUTOTEMP)
        REMEMBER(1, planner.autotemp.enabled, false);
      #endif

      #if TEMP_RESIDENCY_TIME > 0
        millis_t residency_start_ms = 0;
        bool first_loop = true;
        // Loop until the temperature has stabilized
        #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + SEC_TO_MS(TEMP_RESIDENCY_TIME)))
      #else
        // Loop until the temperature is very close target
        #define TEMP_CONDITIONS (wants_to_cool ? isCoolingHotend(target_extruder) : isHeatingHotend(target_extruder))
      #endif

      #if DISABLED(BUSY_WHILE_HEATING) && ENABLED(HOST_KEEPALIVE_FEATURE)
        KEEPALIVE_STATE(NOT_BUSY);
      #endif

      #if ENABLED(PRINTER_EVENT_LEDS)
        const celsius_float_t start_temp = degHotend(target_extruder);
        printerEventLEDs.onHotendHeatingStart();
      #endif

      bool wants_to_cool = false;
      celsius_float_t target_temp = -1.0, old_temp = 9999.0;
      millis_t now, next_temp_ms = 0, cool_check_ms = 0;
      wait_for_heatup = true;
      do {
        // Target temperature might be changed during the loop
        if (target_temp != degTargetHotend(target_extruder)) {
          wants_to_cool = isCoolingHotend(target_extruder);
          target_temp = degTargetHotend(target_extruder);

          // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
          if (no_wait_for_cooling && wants_to_cool) break;
        }

        now = millis();
        if (ELAPSED(now, next_temp_ms)) { // Print temp & remaining time every 1s while waiting
          next_temp_ms = now + 1000UL;
          print_heater_states(target_extruder);
          #if TEMP_RESIDENCY_TIME > 0
            SString<20> s(F(" W:"));
            if (residency_start_ms)
              s += long((SEC_TO_MS(TEMP_RESIDENCY_TIME) - (now - residency_start_ms)) / 1000UL);
            else
              s += '?';
            s.echo();
          #endif
          SERIAL_EOL();
        }

        idle();
        gcode.reset_stepper_timeout(); // Keep steppers powered

        const celsius_float_t temp = degHotend(target_extruder);

        #if ENABLED(PRINTER_EVENT_LEDS)
          // Gradually change LED strip from violet to red as nozzle heats up
          if (!wants_to_cool) printerEventLEDs.onHotendHeating(start_temp, temp, target_temp);
        #endif

        #if TEMP_RESIDENCY_TIME > 0

          const celsius_float_t temp_diff = ABS(target_temp - temp);

          if (!residency_start_ms) {
            // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
            if (temp_diff < TEMP_WINDOW)
              residency_start_ms = now + (first_loop ? SEC_TO_MS(TEMP_RESIDENCY_TIME) / 3 : 0);
          }
          else if (temp_diff > TEMP_HYSTERESIS) {
            // Restart the timer whenever the temperature falls outside the hysteresis.
            residency_start_ms = now;
          }

          first_loop = false;

        #endif

        // Prevent a wait-forever situation if R is misused i.e. M109 R0
        if (wants_to_cool) {
          // Break after MIN_COOLING_SLOPE_TIME seconds
          // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
          if (!cool_check_ms || ELAPSED(now, cool_check_ms)) {
            if (old_temp - temp < float(MIN_COOLING_SLOPE_DEG)) break;
            cool_check_ms = now + SEC_TO_MS(MIN_COOLING_SLOPE_TIME);
            old_temp = temp;
          }
        }

        #if G26_CLICK_CAN_CANCEL
          if (click_to_cancel && ui.use_click()) {
            wait_for_heatup = false;
            TERN_(HAS_MARLINUI_MENU, ui.quick_feedback());
          }
        #endif

      } while (wait_for_heatup && TEMP_CONDITIONS);

      // If wait_for_heatup is set, temperature was reached, no cancel
      if (wait_for_heatup) {
        wait_for_heatup = false;
        #if ENABLED(DWIN_CREALITY_LCD)
          hmiFlag.heat_flag = 0;
          duration_t elapsed = print_job_timer.duration();  // Print timer
          dwin_heat_time = elapsed.value;
        #elif ENABLED(SOVOL_SV06_RTS)
          update_time_value = RTS_UPDATE_VALUE;
          if (IS_SD_PRINTING()) rts.refreshTime();
          rts.start_print_flag = false;
        #else
          ui.reset_status();
        #endif
        TERN_(PRINTER_EVENT_LEDS, printerEventLEDs.onHeatingDone());
        return true;
      }

      return false;
    }

    #if ENABLED(WAIT_FOR_HOTEND)
      void Temperature::wait_for_hotend_heating(const uint8_t target_extruder) {
        if (isHeatingHotend(target_extruder)) {
          SERIAL_ECHOLNPGM("Wait for hotend heating...");
          LCD_MESSAGE(MSG_HEATING);
          wait_for_hotend(target_extruder);
          ui.reset_status();
        }
      }
    #endif

  #endif // HAS_TEMP_HOTEND

  #if HAS_HEATED_BED

    #ifndef MIN_COOLING_SLOPE_DEG_BED
      #define MIN_COOLING_SLOPE_DEG_BED 1.00
    #endif
    #ifndef MIN_COOLING_SLOPE_TIME_BED
      #define MIN_COOLING_SLOPE_TIME_BED 60
    #endif

    bool Temperature::wait_for_bed(const bool no_wait_for_cooling/*=true*/
      OPTARG(G26_CLICK_CAN_CANCEL, const bool click_to_cancel/*=false*/)
    ) {
      #if TEMP_BED_RESIDENCY_TIME > 0
        millis_t residency_start_ms = 0;
        bool first_loop = true;
        // Loop until the temperature has stabilized
        #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + SEC_TO_MS(TEMP_BED_RESIDENCY_TIME)))
      #else
        // Loop until the temperature is very close target
        #define TEMP_BED_CONDITIONS (wants_to_cool ? isCoolingBed() : isHeatingBed())
      #endif

      #if DISABLED(BUSY_WHILE_HEATING) && ENABLED(HOST_KEEPALIVE_FEATURE)
        KEEPALIVE_STATE(NOT_BUSY);
      #endif

      #if ENABLED(PRINTER_EVENT_LEDS)
        const celsius_float_t start_temp = degBed();
        printerEventLEDs.onBedHeatingStart();
      #endif

      bool wants_to_cool = false;
      celsius_float_t target_temp = -1, old_temp = 9999;
      millis_t now, next_temp_ms = 0, cool_check_ms = 0;
      wait_for_heatup = true;
      do {
        // Target temperature might be changed during the loop
        if (target_temp != degTargetBed()) {
          wants_to_cool = isCoolingBed();
          target_temp = degTargetBed();

          // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
          if (no_wait_for_cooling && wants_to_cool) break;
        }

        now = millis();
        if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
          next_temp_ms = now + 1000UL;
          print_heater_states(active_extruder);
          #if TEMP_BED_RESIDENCY_TIME > 0
            SString<20> s(F(" W:"));
            if (residency_start_ms)
              s += long((SEC_TO_MS(TEMP_BED_RESIDENCY_TIME) - (now - residency_start_ms)) / 1000UL);
            else
              s += '?';
            s.echo();
          #endif
          SERIAL_EOL();
        }

        idle();
        gcode.reset_stepper_timeout(); // Keep steppers powered

        const celsius_float_t temp = degBed();

        #if ENABLED(PRINTER_EVENT_LEDS)
          // Gradually change LED strip from blue to violet as bed heats up
          if (!wants_to_cool) printerEventLEDs.onBedHeating(start_temp, temp, target_temp);
        #endif

        #if TEMP_BED_RESIDENCY_TIME > 0

          const celsius_float_t temp_diff = ABS(target_temp - temp);

          if (!residency_start_ms) {
            // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
            if (temp_diff < TEMP_BED_WINDOW)
              residency_start_ms = now + (first_loop ? SEC_TO_MS(TEMP_BED_RESIDENCY_TIME) / 3 : 0);
          }
          else if (temp_diff > TEMP_BED_HYSTERESIS) {
            // Restart the timer whenever the temperature falls outside the hysteresis.
            residency_start_ms = now;
          }

        #endif // TEMP_BED_RESIDENCY_TIME > 0

        // Prevent a wait-forever situation if R is misused i.e. M190 R0
        if (wants_to_cool) {
          // Break after MIN_COOLING_SLOPE_TIME_BED seconds
          // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
          if (!cool_check_ms || ELAPSED(now, cool_check_ms)) {
            if (old_temp - temp < float(MIN_COOLING_SLOPE_DEG_BED)) break;
            cool_check_ms = now + SEC_TO_MS(MIN_COOLING_SLOPE_TIME_BED);
            old_temp = temp;
          }
        }

        #if G26_CLICK_CAN_CANCEL
          if (click_to_cancel && ui.use_click()) {
            wait_for_heatup = false;
            TERN_(HAS_MARLINUI_MENU, ui.quick_feedback());
          }
        #endif

        #if TEMP_BED_RESIDENCY_TIME > 0
          first_loop = false;
        #endif

      } while (wait_for_heatup && TEMP_BED_CONDITIONS);

      // If wait_for_heatup is set, temperature was reached, no cancel
      if (wait_for_heatup) {
        wait_for_heatup = false;
        ui.reset_status();
        return true;
      }

      return false;
    }

    void Temperature::wait_for_bed_heating() {
      if (isHeatingBed()) {
        SERIAL_ECHOLNPGM("Wait for bed heating...");
        LCD_MESSAGE(MSG_BED_HEATING);
        wait_for_bed();
        ui.reset_status();
      }
    }

  #endif // HAS_HEATED_BED

  #if HAS_TEMP_PROBE

    #ifndef MIN_DELTA_SLOPE_DEG_PROBE
      #define MIN_DELTA_SLOPE_DEG_PROBE 1.0
    #endif
    #ifndef MIN_DELTA_SLOPE_TIME_PROBE
      #define MIN_DELTA_SLOPE_TIME_PROBE 600
    #endif

    bool Temperature::wait_for_probe(const celsius_t target_temp, bool no_wait_for_cooling/*=true*/) {

      const bool wants_to_cool = isProbeAboveTemp(target_temp),
                 will_wait = !(wants_to_cool && no_wait_for_cooling);
      if (will_wait)
        SString<60>(F("Waiting for probe to "), wants_to_cool ? F("cool down") : F("heat up"), F(" to "), target_temp, F(" degrees.")).echoln();

      #if DISABLED(BUSY_WHILE_HEATING) && ENABLED(HOST_KEEPALIVE_FEATURE)
        KEEPALIVE_STATE(NOT_BUSY);
      #endif

      float old_temp = 9999;
      millis_t next_temp_ms = 0, next_delta_check_ms = 0;
      wait_for_heatup = true;
      while (will_wait && wait_for_heatup) {

        // Print Temp Reading every 10 seconds while heating up.
        millis_t now = millis();
        if (!next_temp_ms || ELAPSED(now, next_temp_ms)) {
          next_temp_ms = now + 10000UL;
          print_heater_states(active_extruder);
          SERIAL_EOL();
        }

        idle();
        gcode.reset_stepper_timeout(); // Keep steppers powered

        // Break after MIN_DELTA_SLOPE_TIME_PROBE seconds if the temperature
        // did not drop at least MIN_DELTA_SLOPE_DEG_PROBE. This avoids waiting
        // forever as the probe is not actively heated.
        if (!next_delta_check_ms || ELAPSED(now, next_delta_check_ms)) {
          const float temp = degProbe(),
                      delta_temp = old_temp > temp ? old_temp - temp : temp - old_temp;
          if (delta_temp < float(MIN_DELTA_SLOPE_DEG_PROBE)) {
            SERIAL_ECHOLNPGM("Timed out waiting for probe temperature.");
            break;
          }
          next_delta_check_ms = now + SEC_TO_MS(MIN_DELTA_SLOPE_TIME_PROBE);
          old_temp = temp;
        }

        // Loop until the temperature is very close target
        if (!(wants_to_cool ? isProbeAboveTemp(target_temp) : isProbeBelowTemp(target_temp))) {
          SString<60>(wants_to_cool ? F("Cooldown") : F("Heatup"), F(" complete, target probe temperature reached.")).echoln();
          break;
        }
      }

      // If wait_for_heatup is set, temperature was reached, no cancel
      if (wait_for_heatup) {
        wait_for_heatup = false;
        ui.reset_status();
        return true;
      }
      else if (will_wait)
        SERIAL_ECHOLNPGM("Canceled wait for probe temperature.");

      return false;
    }

  #endif // HAS_TEMP_PROBE

  #if HAS_HEATED_CHAMBER

    #ifndef MIN_COOLING_SLOPE_DEG_CHAMBER
      #define MIN_COOLING_SLOPE_DEG_CHAMBER 1.50
    #endif
    #ifndef MIN_COOLING_SLOPE_TIME_CHAMBER
      #define MIN_COOLING_SLOPE_TIME_CHAMBER 120
    #endif

    bool Temperature::wait_for_chamber(const bool no_wait_for_cooling/*=true*/) {
      #if TEMP_CHAMBER_RESIDENCY_TIME > 0
        millis_t residency_start_ms = 0;
        bool first_loop = true;
        // Loop until the temperature has stabilized
        #define TEMP_CHAMBER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + SEC_TO_MS(TEMP_CHAMBER_RESIDENCY_TIME)))
      #else
        // Loop until the temperature is very close target
        #define TEMP_CHAMBER_CONDITIONS (wants_to_cool ? isCoolingChamber() : isHeatingChamber())
      #endif

      #if DISABLED(BUSY_WHILE_HEATING) && ENABLED(HOST_KEEPALIVE_FEATURE)
        KEEPALIVE_STATE(NOT_BUSY);
      #endif

      bool wants_to_cool = false;
      float target_temp = -1, old_temp = 9999;
      millis_t now, next_temp_ms = 0, cool_check_ms = 0;
      wait_for_heatup = true;
      do {
        // Target temperature might be changed during the loop
        if (target_temp != degTargetChamber()) {
          wants_to_cool = isCoolingChamber();
          target_temp = degTargetChamber();

          // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
          if (no_wait_for_cooling && wants_to_cool) break;
        }

        now = millis();
        if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
          next_temp_ms = now + 1000UL;
          print_heater_states(active_extruder);
          #if TEMP_CHAMBER_RESIDENCY_TIME > 0
            SString<20> s(F(" W:"));
            if (residency_start_ms)
              s += long((SEC_TO_MS(TEMP_CHAMBER_RESIDENCY_TIME) - (now - residency_start_ms)) / 1000UL);
            else
              s += '?';
            s.echo();
          #endif
          SERIAL_EOL();
        }

        idle();
        gcode.reset_stepper_timeout(); // Keep steppers powered

        const float temp = degChamber();

        #if TEMP_CHAMBER_RESIDENCY_TIME > 0

          const float temp_diff = ABS(target_temp - temp);

          if (!residency_start_ms) {
            // Start the TEMP_CHAMBER_RESIDENCY_TIME timer when we reach target temp for the first time.
            if (temp_diff < TEMP_CHAMBER_WINDOW)
              residency_start_ms = now + (first_loop ? SEC_TO_MS(TEMP_CHAMBER_RESIDENCY_TIME) / 3 : 0);
          }
          else if (temp_diff > TEMP_CHAMBER_HYSTERESIS) {
            // Restart the timer whenever the temperature falls outside the hysteresis.
            residency_start_ms = now;
          }

          first_loop = false;
        #endif // TEMP_CHAMBER_RESIDENCY_TIME > 0

        // Prevent a wait-forever situation if R is misused i.e. M191 R0
        if (wants_to_cool) {
          // Break after MIN_COOLING_SLOPE_TIME_CHAMBER seconds
          // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_CHAMBER
          if (!cool_check_ms || ELAPSED(now, cool_check_ms)) {
            if (old_temp - temp < float(MIN_COOLING_SLOPE_DEG_CHAMBER)) break;
            cool_check_ms = now + SEC_TO_MS(MIN_COOLING_SLOPE_TIME_CHAMBER);
            old_temp = temp;
          }
        }
      } while (wait_for_heatup && TEMP_CHAMBER_CONDITIONS);

      // If wait_for_heatup is set, temperature was reached, no cancel
      if (wait_for_heatup) {
        wait_for_heatup = false;
        ui.reset_status();
        return true;
      }

      return false;
    }

  #endif // HAS_HEATED_CHAMBER

  #if HAS_COOLER

    #ifndef MIN_COOLING_SLOPE_DEG_COOLER
      #define MIN_COOLING_SLOPE_DEG_COOLER 1.50
    #endif
    #ifndef MIN_COOLING_SLOPE_TIME_COOLER
      #define MIN_COOLING_SLOPE_TIME_COOLER 120
    #endif

    bool Temperature::wait_for_cooler(const bool no_wait_for_cooling/*=true*/) {

      #if TEMP_COOLER_RESIDENCY_TIME > 0
        millis_t residency_start_ms = 0;
        bool first_loop = true;
        // Loop until the temperature has stabilized
        #define TEMP_COOLER_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + SEC_TO_MS(TEMP_COOLER_RESIDENCY_TIME)))
      #else
        // Loop until the temperature is very close target
        #define TEMP_COOLER_CONDITIONS (wants_to_cool ? isLaserHeating() : isLaserCooling())
      #endif

      #if DISABLED(BUSY_WHILE_HEATING) && ENABLED(HOST_KEEPALIVE_FEATURE)
        KEEPALIVE_STATE(NOT_BUSY);
      #endif

      bool wants_to_cool = false;
      float target_temp = -1, previous_temp = 9999;
      millis_t now, next_temp_ms = 0, next_cooling_check_ms = 0;
      wait_for_heatup = true;
      do {
        // Target temperature might be changed during the loop
        if (target_temp != degTargetCooler()) {
          wants_to_cool = isLaserHeating();
          target_temp = degTargetCooler();

          // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
          if (no_wait_for_cooling && wants_to_cool) break;
        }

        now = millis();
        if (ELAPSED(now, next_temp_ms)) { // Print Temp Reading every 1 second while heating up.
          next_temp_ms = now + 1000UL;
          print_heater_states(active_extruder);
          #if TEMP_COOLER_RESIDENCY_TIME > 0
            SString<20> s(F(" W:"));
            if (residency_start_ms)
              s += long((SEC_TO_MS(TEMP_COOLER_RESIDENCY_TIME) - (now - residency_start_ms)) / 1000UL);
            else
              s += '?';
            s.echo();
          #endif
          SERIAL_EOL();
        }

        idle();
        gcode.reset_stepper_timeout(); // Keep steppers powered

        const celsius_float_t current_temp = degCooler();

        #if TEMP_COOLER_RESIDENCY_TIME > 0

          const celsius_float_t temp_diff = ABS(target_temp - temp);

          if (!residency_start_ms) {
            // Start the TEMP_COOLER_RESIDENCY_TIME timer when we reach target temp for the first time.
            if (temp_diff < TEMP_COOLER_WINDOW)
              residency_start_ms = now + (first_loop ? SEC_TO_MS(TEMP_COOLER_RESIDENCY_TIME) / 3 : 0);
          }
          else if (temp_diff > TEMP_COOLER_HYSTERESIS) {
            // Restart the timer whenever the temperature falls outside the hysteresis.
            residency_start_ms = now;
          }

          first_loop = false;
        #endif // TEMP_COOLER_RESIDENCY_TIME > 0

        // Prevent a wait-forever situation if R is misused i.e. M191 R0
        if (wants_to_cool) {
          // Break after MIN_COOLING_SLOPE_TIME_CHAMBER seconds
          // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_CHAMBER
          if (!next_cooling_check_ms || ELAPSED(now, next_cooling_check_ms)) {
            if (previous_temp - current_temp < float(MIN_COOLING_SLOPE_DEG_COOLER)) break;
            next_cooling_check_ms = now + SEC_TO_MS(MIN_COOLING_SLOPE_TIME_COOLER);
            previous_temp = current_temp;
          }
        }

      } while (wait_for_heatup && TEMP_COOLER_CONDITIONS);

      // If wait_for_heatup is set, temperature was reached, no cancel
      if (wait_for_heatup) {
        wait_for_heatup = false;
        ui.reset_status();
        return true;
      }

      return false;
    }

  #endif // HAS_COOLER

#endif // HAS_TEMP_SENSOR
