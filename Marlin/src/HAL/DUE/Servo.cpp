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

/*
  Copyright (c) 2013 Arduino LLC. All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/

#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"

#if HAS_SERVOS

#include "../shared/servo.h"
#include "../shared/servo_private.h"

static Flags<_Nbr_16timers> DisablePending; // ISR should disable the timer at the next timer reset

// ------------------------
// Interrupt handler for the TC0 channel 1.
// ------------------------
void Servo_Handler(const timer16_Sequence_t, Tc*, const uint8_t);

#ifdef _useTimer1
  void HANDLER_FOR_TIMER1() { Servo_Handler(_timer1, TC_FOR_TIMER1, CHANNEL_FOR_TIMER1); }
#endif
#ifdef _useTimer2
  void HANDLER_FOR_TIMER2() { Servo_Handler(_timer2, TC_FOR_TIMER2, CHANNEL_FOR_TIMER2); }
#endif
#ifdef _useTimer3
  void HANDLER_FOR_TIMER3() { Servo_Handler(_timer3, TC_FOR_TIMER3, CHANNEL_FOR_TIMER3); }
#endif
#ifdef _useTimer4
  void HANDLER_FOR_TIMER4() { Servo_Handler(_timer4, TC_FOR_TIMER4, CHANNEL_FOR_TIMER4); }
#endif
#ifdef _useTimer5
  void HANDLER_FOR_TIMER5() { Servo_Handler(_timer5, TC_FOR_TIMER5, CHANNEL_FOR_TIMER5); }
#endif

void Servo_Handler(const timer16_Sequence_t timer, Tc *tc, const uint8_t channel) {
  static int8_t Channel[_Nbr_16timers];                               // Servo counters to pulse (or -1 for refresh interval)
  int8_t cho = Channel[timer];                                        // Handle the prior Channel[timer] first
  if (cho < 0) {                                                      // Channel -1 indicates the refresh interval completed...
    tc->TC_CHANNEL[channel].TC_CCR |= TC_CCR_SWTRG;                   // ...so reset the timer
    if (DisablePending[timer]) {
      // Disabling only after the full servo period expires prevents
      // pulses being too close together if immediately re-enabled.
      DisablePending.clear(timer);
      TC_Stop(tc, channel);
      tc->TC_CHANNEL[channel].TC_SR;                                  // clear interrupt
      return;
    }
  }
  else if (SERVO_INDEX(timer, cho) < ServoCount)                      // prior channel handled?
    extDigitalWrite(SERVO(timer, cho).Pin.nbr, LOW);                  // pulse the prior channel LOW

  Channel[timer] = ++cho;                                             // go to the next channel (or 0)
  if (cho < SERVOS_PER_TIMER && SERVO_INDEX(timer, cho) < ServoCount) {
    tc->TC_CHANNEL[channel].TC_RA = tc->TC_CHANNEL[channel].TC_CV + SERVO(timer, cho).ticks;
    if (SERVO(timer, cho).Pin.isActive)                               // activated?
      extDigitalWrite(SERVO(timer, cho).Pin.nbr, HIGH);               // yes: pulse HIGH
  }
  else {
    // finished all channels so wait for the refresh period to expire before starting over
    const unsigned int cval = tc->TC_CHANNEL[channel].TC_CV + 128 / (SERVO_TIMER_PRESCALER), // allow 128 cycles to ensure the next CV not missed
                       ival = (unsigned int)usToTicks(REFRESH_INTERVAL); // at least REFRESH_INTERVAL has elapsed
    tc->TC_CHANNEL[channel].TC_RA = max(cval, ival);

    Channel[timer] = -1;                                              // reset the timer CCR on the next call
  }

  tc->TC_CHANNEL[channel].TC_SR;                                      // clear interrupt
}

static void _initISR(Tc *tc, uint32_t channel, uint32_t id, IRQn_Type irqn) {
  pmc_enable_periph_clk(id);
  TC_Configure(tc, channel,
      TC_CMR_WAVE                   // Waveform mode
    | TC_CMR_WAVSEL_UP_RC           // Counter running up and reset when equal to RC
    | (SERVO_TIMER_PRESCALER ==   2 ? TC_CMR_TCCLKS_TIMER_CLOCK1 : 0) // MCK/2
    | (SERVO_TIMER_PRESCALER ==   8 ? TC_CMR_TCCLKS_TIMER_CLOCK2 : 0) // MCK/8
    | (SERVO_TIMER_PRESCALER ==  32 ? TC_CMR_TCCLKS_TIMER_CLOCK3 : 0) // MCK/32
    | (SERVO_TIMER_PRESCALER == 128 ? TC_CMR_TCCLKS_TIMER_CLOCK4 : 0) // MCK/128
  );

  // Wait 1ms before the first ISR
  TC_SetRA(tc, channel, (F_CPU) / (SERVO_TIMER_PRESCALER) / 1000UL); // 1ms

  // Configure and enable interrupt
  NVIC_EnableIRQ(irqn);
  tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPAS; // TC_IER_CPAS: RA Compare

  // Enables the timer clock and performs a software reset to start the counting
  TC_Start(tc, channel);
}

void initISR(const timer16_Sequence_t timer_index) {
  CRITICAL_SECTION_START();
  const bool disable_soon = DisablePending[timer_index];
  DisablePending.clear(timer_index);
  CRITICAL_SECTION_END();

  if (!disable_soon) switch (timer_index) {
    default: break;
    #ifdef _useTimer1
      case _timer1: return _initISR(TC_FOR_TIMER1, CHANNEL_FOR_TIMER1, ID_TC_FOR_TIMER1, IRQn_FOR_TIMER1);
    #endif
    #ifdef _useTimer2
      case _timer2: return _initISR(TC_FOR_TIMER2, CHANNEL_FOR_TIMER2, ID_TC_FOR_TIMER2, IRQn_FOR_TIMER2);
    #endif
    #ifdef _useTimer3
      case _timer3: return _initISR(TC_FOR_TIMER3, CHANNEL_FOR_TIMER3, ID_TC_FOR_TIMER3, IRQn_FOR_TIMER3);
    #endif
    #ifdef _useTimer4
      case _timer4: return _initISR(TC_FOR_TIMER4, CHANNEL_FOR_TIMER4, ID_TC_FOR_TIMER4, IRQn_FOR_TIMER4);
    #endif
    #ifdef _useTimer5
      case _timer5: return _initISR(TC_FOR_TIMER5, CHANNEL_FOR_TIMER5, ID_TC_FOR_TIMER5, IRQn_FOR_TIMER5);
    #endif
  }
}

void finISR(const timer16_Sequence_t timer_index) {
  // Timer is disabled from the ISR, to ensure proper final pulse length.
  DisablePending.set(timer_index);
}

#endif // HAS_SERVOS

#endif // ARDUINO_ARCH_SAM
