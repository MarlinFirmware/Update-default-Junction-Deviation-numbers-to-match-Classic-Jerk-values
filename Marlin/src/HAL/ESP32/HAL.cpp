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
#ifdef ARDUINO_ARCH_ESP32

#include "../../inc/MarlinConfig.h"

#include <rom/rtc.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <HardwareSerial.h>

#if ENABLED(USE_ESP32_TASK_WDT)
  #include <esp_task_wdt.h>
#endif

#if ENABLED(WIFISUPPORT)
  #include <ESPAsyncWebServer.h>
  #include "wifi.h"
  #if ENABLED(OTASUPPORT)
    #include "ota.h"
  #endif
  #if ENABLED(WEBSUPPORT)
    #include "spiffs.h"
    #include "web.h"
  #endif
#endif

#if ENABLED(ESP3D_WIFISUPPORT)
  DefaultSerial1 MSerial0(false, Serial2Socket);
#endif

// ------------------------
// Externs
// ------------------------

portMUX_TYPE MarlinHAL::spinlock = portMUX_INITIALIZER_UNLOCKED;

// ------------------------
// Local defines
// ------------------------

#define V_REF 1100

// ------------------------
// Public Variables
// ------------------------

uint16_t MarlinHAL::adc_result;
pwm_pin_t MarlinHAL::pwm_pin_data[MAX_EXPANDER_BITS];

// ------------------------
// Private Variables
// ------------------------

esp_adc_cal_characteristics_t characteristics[ADC_ATTEN_MAX];
adc_atten_t attenuations[ADC1_CHANNEL_MAX] = {};
uint32_t thresholds[ADC_ATTEN_MAX];

volatile int numPWMUsed = 0;
volatile struct { pin_t pin; int value; } pwmState[MAX_PWM_PINS];

pin_t chan_pin[CHANNEL_MAX_NUM + 1] = { 0 }; // PWM capable IOpins - not 0 or >33 on ESP32

struct {
  uint32_t freq; // ledcReadFreq doesn't work if a duty hasn't been set yet!
  uint16_t res;
} pwmInfo[(CHANNEL_MAX_NUM + 1) / 2];

// ------------------------
// Public functions
// ------------------------

#if ENABLED(WIFI_CUSTOM_COMMAND)

  bool wifi_custom_command(char * const command_ptr) {
    #if ENABLED(ESP3D_WIFISUPPORT)
      return esp3dlib.parse(command_ptr);
    #else
      UNUSED(command_ptr);
      return false;
    #endif
  }

#endif

#if ENABLED(USE_ESP32_EXIO)

  HardwareSerial YSerial2(2);

  void Write_EXIO(uint8_t IO, uint8_t v) {
    if (hal.isr_state()) {
      hal.isr_off();
      YSerial2.write(0x80 | (((char)v) << 5) | (IO - 100));
      hal.isr_on();
    }
    else
      YSerial2.write(0x80 | (((char)v) << 5) | (IO - 100));
  }

#endif

void MarlinHAL::init_board() {
  #if ENABLED(USE_ESP32_TASK_WDT)
    esp_task_wdt_init(10, true);
  #endif
  #if ENABLED(ESP3D_WIFISUPPORT)
    esp3dlib.init();
  #elif ENABLED(WIFISUPPORT)
    wifi_init();
    TERN_(OTASUPPORT, OTA_init());
    #if ENABLED(WEBSUPPORT)
      spiffs_init();
      web_init();
    #endif
    server.begin();
  #endif

  // ESP32 uses a GPIO matrix that allows pins to be assigned to hardware serial ports.
  // The following code initializes hardware Serial1 and Serial2 to use user-defined pins
  // if they have been defined.
  #if defined(HARDWARE_SERIAL1_RX) && defined(HARDWARE_SERIAL1_TX)
    HardwareSerial Serial1(1);
    #ifdef TMC_BAUD_RATE  // use TMC_BAUD_RATE for Serial1 if defined
      Serial1.begin(TMC_BAUD_RATE, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);
    #else  // use default BAUDRATE if TMC_BAUD_RATE not defined
      Serial1.begin(BAUDRATE, SERIAL_8N1, HARDWARE_SERIAL1_RX, HARDWARE_SERIAL1_TX);
    #endif
  #endif
  #if defined(HARDWARE_SERIAL2_RX) && defined(HARDWARE_SERIAL2_TX)
    HardwareSerial Serial2(2);
    #ifdef TMC_BAUD_RATE  // use TMC_BAUD_RATE for Serial1 if defined
      Serial2.begin(TMC_BAUD_RATE, SERIAL_8N1, HARDWARE_SERIAL2_RX, HARDWARE_SERIAL2_TX);
    #else  // use default BAUDRATE if TMC_BAUD_RATE not defined
      Serial2.begin(BAUDRATE, SERIAL_8N1, HARDWARE_SERIAL2_RX, HARDWARE_SERIAL2_TX);
    #endif
  #endif

  // Initialize the i2s peripheral only if the I2S stepper stream is enabled.
  // The following initialization is performed after Serial1 and Serial2 are defined as
  // their native pins might conflict with the i2s stream even when they are remapped.
  #if ENABLED(USE_ESP32_EXIO)
    YSerial2.begin(460800 * 3, SERIAL_8N1, 16, 17);
  #elif ENABLED(I2S_STEPPER_STREAM)
    i2s_init();
  #endif
}

void MarlinHAL::idletask() {
  #if ALL(WIFISUPPORT, OTASUPPORT)
    OTA_handle();
  #endif
  TERN_(ESP3D_WIFISUPPORT, esp3dlib.idletask());
}

uint8_t MarlinHAL::get_reset_source() { return rtc_get_reset_reason(1); }

void MarlinHAL::reboot() { ESP.restart(); }

void _delay_ms(const int ms) { delay(ms); }

// return free memory between end of heap (or end bss) and whatever is current
int MarlinHAL::freeMemory() { return ESP.getFreeHeap(); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  extern "C" {
    esp_err_t esp_task_wdt_reset();
  }

  void watchdogSetup() {
    // do whatever. don't remove this function.
  }

  void MarlinHAL::watchdog_init() {
    // TODO
  }

  // Reset watchdog.
  void MarlinHAL::watchdog_refresh() { esp_task_wdt_reset(); }

#endif

// ------------------------
// ADC
// ------------------------

// https://docs.espressif.com/projects/esp-idf/en/release-v4.4/esp32/api-reference/peripherals/adc.html
adc1_channel_t get_channel(int pin) {
  switch (pin) {
    case 39: return ADC1_CHANNEL_3;
    case 36: return ADC1_CHANNEL_0;
    case 35: return ADC1_CHANNEL_7;
    case 34: return ADC1_CHANNEL_6;
    case 33: return ADC1_CHANNEL_5;
    case 32: return ADC1_CHANNEL_4;
    case 37: return ADC1_CHANNEL_1;
    case 38: return ADC1_CHANNEL_2;
  }
  return ADC1_CHANNEL_MAX;
}

void adc1_set_attenuation(adc1_channel_t chan, adc_atten_t atten) {
  if (attenuations[chan] != atten) {
    adc1_config_channel_atten(chan, atten);
    attenuations[chan] = atten;
  }
}

void MarlinHAL::adc_init() {
  // Configure ADC
  adc1_config_width(ADC_WIDTH_12Bit);

  // Configure channels only if used as (re-)configuring a pin for ADC that is used elsewhere might have adverse effects
  TERN_(HAS_TEMP_ADC_0,        adc1_set_attenuation(get_channel(TEMP_0_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_1,        adc1_set_attenuation(get_channel(TEMP_1_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_2,        adc1_set_attenuation(get_channel(TEMP_2_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_3,        adc1_set_attenuation(get_channel(TEMP_3_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_4,        adc1_set_attenuation(get_channel(TEMP_4_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_5,        adc1_set_attenuation(get_channel(TEMP_5_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_6,        adc2_set_attenuation(get_channel(TEMP_6_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_ADC_7,        adc3_set_attenuation(get_channel(TEMP_7_PIN), ADC_ATTEN_11db));
  TERN_(HAS_HEATED_BED,        adc1_set_attenuation(get_channel(TEMP_BED_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_CHAMBER,      adc1_set_attenuation(get_channel(TEMP_CHAMBER_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_PROBE,        adc1_set_attenuation(get_channel(TEMP_PROBE_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_COOLER,       adc1_set_attenuation(get_channel(TEMP_COOLER_PIN), ADC_ATTEN_11db));
  TERN_(HAS_TEMP_BOARD,        adc1_set_attenuation(get_channel(TEMP_BOARD_PIN), ADC_ATTEN_11db));
  TERN_(FILAMENT_WIDTH_SENSOR, adc1_set_attenuation(get_channel(FILWIDTH_PIN), ADC_ATTEN_11db));

  // Note that adc2 is shared with the WiFi module, which has higher priority, so the conversion may fail.
  // That's why we're not setting it up here.

  // Calculate ADC characteristics (i.e., gain and offset factors for each attenuation level)
  for (int i = 0; i < ADC_ATTEN_MAX; i++) {
    esp_adc_cal_characterize(ADC_UNIT_1, (adc_atten_t)i, ADC_WIDTH_BIT_12, V_REF, &characteristics[i]);

    // Change attenuation 100mV below the calibrated threshold
    thresholds[i] = esp_adc_cal_raw_to_voltage(4095, &characteristics[i]);
  }
}

#ifndef ADC_REFERENCE_VOLTAGE
  #define ADC_REFERENCE_VOLTAGE 3.3
#endif

void MarlinHAL::adc_start(const pin_t pin) {
  const adc1_channel_t chan = get_channel(pin);
  uint32_t mv;
  esp_adc_cal_get_voltage((adc_channel_t)chan, &characteristics[attenuations[chan]], &mv);

  adc_result = mv * isr_float_t(1023) / isr_float_t(ADC_REFERENCE_VOLTAGE) / isr_float_t(1000);

  // Change the attenuation level based on the new reading
  adc_atten_t atten;
  if (mv < thresholds[ADC_ATTEN_DB_0] - 100)
    atten = ADC_ATTEN_DB_0;
  else if (mv > thresholds[ADC_ATTEN_DB_0] - 50 && mv < thresholds[ADC_ATTEN_DB_2_5] - 100)
    atten = ADC_ATTEN_DB_2_5;
  else if (mv > thresholds[ADC_ATTEN_DB_2_5] - 50 && mv < thresholds[ADC_ATTEN_DB_6] - 100)
    atten = ADC_ATTEN_DB_6;
  else if (mv > thresholds[ADC_ATTEN_DB_6] - 50)
    atten = ADC_ATTEN_DB_11;
  else return;

  adc1_set_attenuation(chan, atten);
}

// ------------------------
// PWM
// ------------------------

int8_t channel_for_pin(const uint8_t pin) {
  for (int i = 0; i <= CHANNEL_MAX_NUM; i++)
    if (chan_pin[i] == pin) return i;
  return -1;
}

// get PWM channel for pin - if none then attach a new one
// return -1 if fail or invalid pin#, channel # (0-15) if success
int8_t get_pwm_channel(const pin_t pin, const uint32_t freq, const uint16_t res) {
  if (!WITHIN(pin, 1, MAX_PWM_IOPIN)) return -1; // Not a hardware PWM pin!
  int8_t cid = channel_for_pin(pin);
  if (cid >= 0) return cid;

  // Find an empty adjacent channel (same timer & freq/res)
  for (int i = 0; i <= CHANNEL_MAX_NUM; i++) {
    if (chan_pin[i] == 0) {
      if (chan_pin[i ^ 0x1] != 0) {
        if (pwmInfo[i / 2].freq == freq && pwmInfo[i / 2].res == res) {
          chan_pin[i] = pin; // Allocate PWM to this channel
          ledcAttachPin(pin, i);
          return i;
        }
      }
      else if (cid == -1)    // Pair of empty channels?
        cid = i & 0xFE;      // Save lower channel number
    }
  }
  // not attached, is an empty timer slot avail?
  if (cid >= 0) {
    chan_pin[cid] = pin;
    pwmInfo[cid / 2].freq = freq;
    pwmInfo[cid / 2].res = res;
    ledcSetup(cid, freq, res);
    ledcAttachPin(pin, cid);
  }
  return cid; // -1 if no channel avail
}

void MarlinHAL::set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size/*=_BV(PWM_RESOLUTION)-1*/, const bool invert/*=false*/) {
  #if ENABLED(I2S_STEPPER_STREAM)
    if (pin > 127) {
      const uint8_t pinlo = pin & 0x7F;
      pwm_pin_t &pindata = pwm_pin_data[pinlo];
      const uint32_t duty = map(invert ? v_size - v : v, 0, v_size, 0, pindata.pwm_cycle_ticks);
      if (duty == 0 || duty == pindata.pwm_cycle_ticks) { // max or min (i.e., on/off)
        pindata.pwm_duty_ticks = 0;  // turn off PWM for this pin
        duty ? SBI32(i2s_port_data, pinlo) : CBI32(i2s_port_data, pinlo); // set pin level
      }
      else
        pindata.pwm_duty_ticks = duty; // PWM duty count = # of 4µs ticks per full PWM cycle

      return;
    }
  #endif

  const int8_t cid = get_pwm_channel(pin, PWM_FREQUENCY, PWM_RESOLUTION);
  if (cid >= 0) {
    const uint32_t duty = map(invert ? v_size - v : v, 0, v_size, 0, _BV(PWM_RESOLUTION)-1);
    ledcWrite(cid, duty);
  }
}

int8_t MarlinHAL::set_pwm_frequency(const pin_t pin, const uint32_t f_desired) {
  #if ENABLED(I2S_STEPPER_STREAM)
    if (pin > 127) {
      pwm_pin_data[pin & 0x7F].pwm_cycle_ticks = 1000000UL / f_desired / 4; // # of 4µs ticks per full PWM cycle
      return 0;
    }
  #endif

  const int8_t cid = channel_for_pin(pin);
  if (cid >= 0) {
    if (f_desired == ledcReadFreq(cid)) return cid; // no freq change
    ledcDetachPin(chan_pin[cid]);
    chan_pin[cid] = 0;              // remove old freq channel
  }
  return get_pwm_channel(pin, f_desired, PWM_RESOLUTION); // try for new one
}

// use hardware PWM if avail, if not then ISR
void analogWrite(const pin_t pin, const uint16_t value, const uint32_t freq/*=PWM_FREQUENCY*/, const uint16_t res/*=8*/) { // always 8 bit resolution!
  // Use ledc hardware for internal pins
  const int8_t cid = get_pwm_channel(pin, freq, res);
  if (cid >= 0) {
    ledcWrite(cid, value); // set duty value
    return;
  }

  // not a hardware PWM pin OR no PWM channels available
  int idx = -1;

  // Search Pin
  for (int i = 0; i < numPWMUsed; ++i)
    if (pwmState[i].pin == pin) { idx = i; break; }

  // not found ?
  if (idx < 0) {
    // No slots remaining
    if (numPWMUsed >= MAX_PWM_PINS) return;

    // Take new slot for pin
    idx = numPWMUsed;
    pwmState[idx].pin = pin;
    // Start timer on first use
    if (idx == 0) HAL_timer_start(MF_TIMER_PWM, PWM_TIMER_FREQUENCY);

    ++numPWMUsed;
  }

  // Use 7bit internal value - add 1 to have 100% high at 255
  pwmState[idx].value = (value + 1) / 2;
}

// Handle PWM timer interrupt
HAL_PWM_TIMER_ISR() {
  HAL_timer_isr_prologue(MF_TIMER_PWM);

  static uint8_t count = 0;

  for (int i = 0; i < numPWMUsed; ++i) {
    if (count == 0)                   // Start of interval
      digitalWrite(pwmState[i].pin, pwmState[i].value ? HIGH : LOW);
    else if (pwmState[i].value == count)   // End of duration
      digitalWrite(pwmState[i].pin, LOW);
  }

  // 128 for 7 Bit resolution
  count = (count + 1) & 0x7F;

  HAL_timer_isr_epilogue(MF_TIMER_PWM);
}

#endif // ARDUINO_ARCH_ESP32
