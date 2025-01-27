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
 * HAL for Arduino Due and compatible (SAM3X8E)
 */

#ifdef ARDUINO_ARCH_SAM

#include "../../inc/MarlinConfig.h"
#include "../../MarlinCore.h"

#include <Wire.h>
#include "usb/usb_task.h"

// ------------------------
// Public Variables
// ------------------------

uint16_t MarlinHAL::adc_result;

// ------------------------
// Public functions
// ------------------------

#if ENABLED(POSTMORTEM_DEBUGGING)
  extern void install_min_serial();
#endif

void MarlinHAL::init() {
  #if HAS_MEDIA
    OUT_WRITE(SD_SS_PIN, HIGH);  // Try to set SDSS inactive before any other SPI users start up
  #endif
  usb_task_init();          // Initialize the USB stack
  TERN_(POSTMORTEM_DEBUGGING, install_min_serial()); // Install the min serial handler
}

void MarlinHAL::init_board() {
  #ifdef BOARD_INIT
    BOARD_INIT();
  #endif
}

void MarlinHAL::idletask() { usb_task_idle(); } // Perform USB stack housekeeping

uint8_t MarlinHAL::get_reset_source() {
  switch ((RSTC->RSTC_SR >> 8) & 0x07) {
    case 0: return RST_POWER_ON;
    case 1: return RST_BACKUP;
    case 2: return RST_WATCHDOG;
    case 3: return RST_SOFTWARE;
    case 4: return RST_EXTERNAL;
    default: return 0;
  }
}

void MarlinHAL::reboot() { rstc_start_software_reset(RSTC); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  // Initialize watchdog - On SAM3X, Watchdog was already configured
  //  and enabled or disabled at startup, so no need to reconfigure it
  //  here.
  void MarlinHAL::watchdog_init() { WDT_Restart(WDT); } // Reset watchdog to start clean

  // Reset watchdog. MUST be called at least every 4 seconds after the
  // first watchdog_init or AVR will go into emergency procedures.
  void MarlinHAL::watchdog_refresh() { watchdogReset(); }

#endif

// Override Arduino runtime to either config or disable the watchdog
//
// We need to configure the watchdog as soon as possible in the boot
// process, because watchdog initialization at hardware reset on SAM3X8E
// is unreliable, and there is risk of unintended resets if we delay
// that initialization to a later time.
void watchdogSetup() {

  #if ENABLED(USE_WATCHDOG)

    // 4 seconds timeout
    uint32_t timeout = TERN(WATCHDOG_DURATION_8S, 8000, 4000);

    // Calculate timeout value in WDT counter ticks: This assumes
    // the slow clock is running at 32.768 kHz watchdog
    // frequency is therefore 32768 / 128 = 256 Hz
    timeout = (timeout << 8) / 1000;
    if (timeout == 0)
      timeout = 1;
    else if (timeout > 0xFFF)
      timeout = 0xFFF;

    // We want to enable the watchdog with the specified timeout
    uint32_t value =
      WDT_MR_WDV(timeout) |               // With the specified timeout
      WDT_MR_WDD(timeout) |               // and no invalid write window
    #if !(SAMV70 || SAMV71 || SAME70 || SAMS70)
      WDT_MR_WDRPROC   |                  // WDT fault resets processor only - We want
                                          // to keep PIO controller state
    #endif
      WDT_MR_WDDBGHLT  |                  // WDT stops in debug state.
      WDT_MR_WDIDLEHLT;                   // WDT stops in idle state.

    #if ENABLED(WATCHDOG_RESET_MANUAL)
      // We enable the watchdog timer, but only for the interrupt.

      // Configure WDT to only trigger an interrupt
      value |= WDT_MR_WDFIEN;             // Enable WDT fault interrupt.

      // Disable WDT interrupt (just in case, to avoid triggering it!)
      NVIC_DisableIRQ(WDT_IRQn);

      // We NEED memory barriers to ensure Interrupts are actually disabled!
      // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
      __DSB();
      __ISB();

      // Initialize WDT with the given parameters
      WDT_Enable(WDT, value);

      // Configure and enable WDT interrupt.
      NVIC_ClearPendingIRQ(WDT_IRQn);
      NVIC_SetPriority(WDT_IRQn, 0); // Use highest priority, so we detect all kinds of lockups
      NVIC_EnableIRQ(WDT_IRQn);

    #else

      // a WDT fault triggers a reset
      value |= WDT_MR_WDRSTEN;

      // Initialize WDT with the given parameters
      WDT_Enable(WDT, value);

    #endif

    // Reset the watchdog
    WDT_Restart(WDT);

  #else

    // Make sure to completely disable the Watchdog
    WDT_Disable(WDT);

  #endif
}

// ------------------------
// Free Memory Accessor
// ------------------------

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// Return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory, heap_end = (int)_sbrk(0);
  return (int)&free_memory - (heap_end ?: (int)&_ebss);
}

// ------------------------
// Serial Ports
// ------------------------

// Forward the default serial ports
#if USING_HW_SERIAL0
  DefaultSerial1 MSerial0(false, Serial);
#endif
#if USING_HW_SERIAL1
  DefaultSerial2 MSerial1(false, Serial1);
#endif
#if USING_HW_SERIAL2
  DefaultSerial3 MSerial2(false, Serial2);
#endif
#if USING_HW_SERIAL3
  DefaultSerial4 MSerial3(false, Serial3);
#endif

#endif // ARDUINO_ARCH_SAM
