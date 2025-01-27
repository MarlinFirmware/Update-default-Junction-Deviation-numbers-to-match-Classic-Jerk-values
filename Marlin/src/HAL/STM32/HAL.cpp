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
#include "../platforms.h"

#ifdef HAL_STM32

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"

#include "usb_serial.h"

#ifdef USBCON
  DefaultSerial1 MSerialUSB(false, SerialUSB);
#endif

#if ENABLED(SRAM_EEPROM_EMULATION)
  #if STM32F7xx
    #include <stm32f7xx_ll_pwr.h>
  #elif STM32F4xx
    #include <stm32f4xx_ll_pwr.h>
  #else
    #error "SRAM_EEPROM_EMULATION is currently only supported for STM32F4xx and STM32F7xx"
  #endif
#endif

#if HAS_SD_HOST_DRIVE
  #include "msc_sd.h"
  #include "usbd_cdc_if.h"
#endif

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

// HAL initialization task
void MarlinHAL::init() {
  // Ensure F_CPU is a constant expression.
  // If the compiler breaks here, it means that delay code that should compute at compile time will not work.
  // So better safe than sorry here.
  constexpr int cpuFreq = F_CPU;
  UNUSED(cpuFreq);

  #if HAS_MEDIA && DISABLED(ONBOARD_SDIO) && PIN_EXISTS(SD_SS)
    OUT_WRITE(SD_SS_PIN, HIGH); // Try to set SDSS inactive before any other SPI users start up
  #endif

  #if PIN_EXISTS(LED)
    OUT_WRITE(LED_PIN, LOW);
  #endif

  #if ENABLED(SRAM_EEPROM_EMULATION)
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();           // Enable access to backup SRAM
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    LL_PWR_EnableBkUpRegulator();         // Enable backup regulator
    while (!LL_PWR_IsActiveFlag_BRR());   // Wait until backup regulator is initialized
  #endif

  SetTimerInterruptPriorities();

  #if ENABLED(EMERGENCY_PARSER) && (USBD_USE_CDC || USBD_USE_CDC_MSC)
    USB_Hook_init();
  #endif

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial());    // Install the min serial handler

  TERN_(HAS_SD_HOST_DRIVE, MSC_SD_init());              // Enable USB SD card access

  #if PIN_EXISTS(USB_CONNECT)
    OUT_WRITE(USB_CONNECT_PIN, !USB_CONNECT_INVERTING); // USB clear connection
    delay(1000);                                        // Give OS time to notice
    WRITE(USB_CONNECT_PIN, USB_CONNECT_INVERTING);
  #endif
}

// HAL idle task
void MarlinHAL::idletask() {
  #if HAS_SHARED_MEDIA
    // Stm32duino currently doesn't have a "loop/idle" method
    CDC_resume_receive();
    CDC_continue_transmit();
  #endif
}

void MarlinHAL::reboot() { NVIC_SystemReset(); }

uint8_t MarlinHAL::get_reset_source() {
  return
    #ifdef RCC_FLAG_IWDGRST // Some sources may not exist...
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)  ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG1RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG1RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_IWDG2RST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_IWDG2RST) ? RST_WATCHDOG :
    #endif
    #ifdef RCC_FLAG_SFTRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)   ? RST_SOFTWARE :
    #endif
    #ifdef RCC_FLAG_PINRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)   ? RST_EXTERNAL :
    #endif
    #ifdef RCC_FLAG_PORRST
      RESET != __HAL_RCC_GET_FLAG(RCC_FLAG_PORRST)   ? RST_POWER_ON :
    #endif
    0
  ;
}

void MarlinHAL::clear_reset_source() { __HAL_RCC_CLEAR_RESET_FLAGS(); }

// ------------------------
// Watchdog Timer
// ------------------------

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  #include <IWatchdog.h>

  void MarlinHAL::watchdog_init() {
    IF_DISABLED(DISABLE_WATCHDOG_INIT, IWatchdog.begin(WDT_TIMEOUT_US));
  }

  void MarlinHAL::watchdog_refresh() {
    IWatchdog.reload();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif
  }

#endif

extern "C" {
  extern unsigned int _ebss; // end of bss section
}

// Reset the system to initiate a firmware flash
WEAK void flashFirmware(const int16_t) { hal.reboot(); }

// Maple Compatibility
volatile uint32_t systick_uptime_millis = 0;
systickCallback_t systick_user_callback;
void systick_attach_callback(systickCallback_t cb) { systick_user_callback = cb; }
void HAL_SYSTICK_Callback() {
  systick_uptime_millis++;
  if (systick_user_callback) systick_user_callback();
}

#endif // HAL_STM32
