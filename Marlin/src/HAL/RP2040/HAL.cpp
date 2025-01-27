/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2024 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../platforms.h"

#ifdef __PLAT_RP2040__

#include "HAL.h"
//#include "usb_serial.h"

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

extern "C" {
  #include "pico/bootrom.h"
  #include "hardware/watchdog.h"
}

#if HAS_SD_HOST_DRIVE
  #include "msc_sd.h"
  #include "usbd_cdc_if.h"
#endif

// ------------------------
// Public Variables
// ------------------------

volatile uint16_t adc_result;

// ------------------------
// Public functions
// ------------------------

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

// HAL initialization task
void MarlinHAL::init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if HAS_MEDIA && DISABLED(SDIO_SUPPORT) && PIN_EXISTS(SD_SS)
    OUT_WRITE(SD_SS_PIN, HIGH); // Try to set SD_SS_PIN inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    // __HAL_RCC_PWR_CLK_ENABLE();
    // HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    // __HAL_RCC_BKPSRAM_CLK_ENABLE();
    // LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    // while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  HAL_timer_init();

  #if ENABLED(EMERGENCY_PARSER) && USBD_USE_CDC
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());    // Install the min serial handler

  TERN_(HAS_SD_HOST_DRIVE, MSC_SD_init());              // Enable USB SD card access

  #if PIN_EXISTS(USB_CONNECT)
    OUT_WRITE(USB_CONNECT_PIN, !USB_CONNECT_INVERTING); // USB clear connection
    delay_ms(1000);                                        // Give OS time to notice
    WRITE(USB_CONNECT_PIN, USB_CONNECT_INVERTING);
  #endif
}

uint8_t MarlinHAL::get_reset_source() {
  return watchdog_enable_caused_reboot() ? RST_WATCHDOG : 0;
}

void MarlinHAL::reboot() { watchdog_reboot(0, 0, 1); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  extern "C" {
    #include "hardware/watchdog.h"
  }

  void MarlinHAL::watchdog_init() {
    #if DISABLED(DISABLE_WATCHDOG_INIT)
      static_assert(WDT_TIMEOUT_US > 1000, "WDT Timout is too small, aborting");
      watchdog_enable(WDT_TIMEOUT_US/1000, true);
    #endif
  }

  void MarlinHAL::watchdog_refresh() {
    watchdog_update();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif
  }

#endif

// ------------------------
// ADC
// ------------------------

volatile bool MarlinHAL::adc_has_result = false;

void MarlinHAL::adc_init() {
  analogReadResolution(HAL_ADC_RESOLUTION);
  ::adc_init();
  adc_fifo_setup(true, false, 1, false, false);
  irq_set_exclusive_handler(ADC_IRQ_FIFO, adc_exclusive_handler);
  irq_set_enabled(ADC_IRQ_FIFO, true);
  adc_irq_set_enabled(true);
}

void MarlinHAL::adc_enable(const pin_t pin) {
  if (pin >= A0 && pin <= A3)
    adc_gpio_init(pin);
  else if (pin == HAL_ADC_MCU_TEMP_DUMMY_PIN)
    adc_set_temp_sensor_enabled(true);
}

void MarlinHAL::adc_start(const pin_t pin) {
  adc_has_result = false;
  // Select an ADC input. 0...3 are GPIOs 26...29 respectively.
  adc_select_input(pin == HAL_ADC_MCU_TEMP_DUMMY_PIN ? 4 : pin - A0);
  adc_run(true);
}

void MarlinHAL::adc_exclusive_handler() {
  adc_run(false); // Disable since we only want one result
  irq_clear(ADC_IRQ_FIFO); // Clear the IRQ

  if (adc_fifo_get_level() >= 1) {
    adc_result = adc_fifo_get(); // Pop the result
    adc_fifo_drain();
    adc_has_result = true; // Signal the end of the conversion
  }
}

uint16_t MarlinHAL::adc_value() { return adc_result; }

// Reset the system to initiate a firmware flash
void flashFirmware(const int16_t) { hal.reboot(); }

extern "C" {
  void * _sbrk(int incr);
  extern unsigned int __bss_end__; // end of bss section
}

// Return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory, heap_end = (int)_sbrk(0);
  return (int)&free_memory - (heap_end ?: (int)&__bss_end__);
}

#endif // __PLAT_RP2040__
