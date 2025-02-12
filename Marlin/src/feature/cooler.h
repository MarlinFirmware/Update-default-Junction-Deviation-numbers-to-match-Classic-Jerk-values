/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2021 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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

#include "../inc/MarlinConfigPre.h"

#ifndef FLOWMETER_PPL
  #define FLOWMETER_PPL      5880 // Pulses per liter
#endif
#ifndef FLOWMETER_INTERVAL
  #define FLOWMETER_INTERVAL 1000 // milliseconds
#endif

// Cooling device

class Cooler {
public:
  static uint16_t capacity;   // Cooling capacity in watts
  static uint16_t load;       // Cooling load in watts

  static bool enabled;
  static void enable()  { enabled = true; }
  static void disable() { enabled = false; }
  static void toggle()  { FLIP(enabled); }

  static uint8_t mode;                  // 0 = CO2 Liquid cooling, 1 = Laser Diode TEC Heatsink Cooling
  static void set_mode(const uint8_t m) { mode = m; }

  #if ENABLED(LASER_COOLANT_FLOW_METER)
    static float flowrate;                // Flow meter reading in liters-per-minute.
    static bool flowmeter;                // Flag to monitor the flow
    static volatile uint16_t flowpulses;  // Flowmeter IRQ pulse count
    static millis_t flowmeter_next_ms;    // Next time at which to calculate flow

    static void set_flowmeter(const bool sflag) {
      if (flowmeter != sflag) {
        flowmeter = sflag;
        if (sflag) {
          flowpulses = 0;
          flowmeter_next_ms = millis() + FLOWMETER_INTERVAL;
        }
      }
    }

    // To calculate flow we only need to count pulses
    static void flowmeter_ISR() { flowpulses++; }

    // Enable / Disable the flow meter interrupt
    static void flowmeter_interrupt_enable() {
      attachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN), flowmeter_ISR, RISING);
    }
    static void flowmeter_interrupt_disable() {
      detachInterrupt(digitalPinToInterrupt(FLOWMETER_PIN));
    }

    // Enable / Disable the flow meter interrupt
    static void flowmeter_enable()  { set_flowmeter(true); flowpulses = 0; flowmeter_interrupt_enable(); }
    static void flowmeter_disable() { set_flowmeter(false); flowmeter_interrupt_disable(); flowpulses = 0; }

    // Get the total flow (in liters per minute) since the last reading
    static void calc_flowrate() {
      // flowrate = (litres) * (seconds) = litres per minute
      flowrate = (flowpulses / (float)FLOWMETER_PPL) * ((1000.0f / (float)FLOWMETER_INTERVAL) * 60.0f);
      flowpulses = 0;
    }

    // Userland task to update the flow meter
    static void flowmeter_task(const millis_t ms=millis()) {
      if (!flowmeter)       // !! The flow meter must always be on !!
        flowmeter_enable(); // Init and prime
      if (ELAPSED(ms, flowmeter_next_ms)) {
        calc_flowrate();
        flowmeter_next_ms = ms + FLOWMETER_INTERVAL;
      }
    }

    #if ENABLED(FLOWMETER_SAFETY)
      static bool flowfault;                // Flag that the cooler is in a fault state
      static bool flowsafety_enabled;       // Flag to disable the cutter if flow rate is too low
      static void flowsafety_toggle()   { FLIP(flowsafety_enabled); }
      static bool check_flow_too_low() {
        const bool too_low = flowsafety_enabled && flowrate < (FLOWMETER_MIN_LITERS_PER_MINUTE);
        flowfault =  too_low;
        return too_low;
      }
    #endif
  #endif
};

extern Cooler cooler;
