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
 * HAL for stm32duino.com based on Libmaple and compatible (STM32F1)
 */

#define CPU_32_BIT

#include "../../core/macros.h"
#include "../shared/Marduino.h"
#include "../shared/math_32bit.h"
#include "../shared/HAL_SPI.h"

#include "fastio.h"

#include <stdint.h>
#include <util/atomic.h>

#include "../../inc/MarlinConfigPre.h"

#if HAS_SD_HOST_DRIVE
  #include "msc_sd.h"
#endif

// ------------------------
// Defines
// ------------------------

//
// Default graphical display delays
//
#define CPU_ST7920_DELAY_1 300
#define CPU_ST7920_DELAY_2  40
#define CPU_ST7920_DELAY_3 340

#ifndef STM32_FLASH_SIZE
  #if ANY(MCU_STM32F103RE, MCU_STM32F103VE, MCU_STM32F103ZE)
    #define STM32_FLASH_SIZE 512
  #else
    #define STM32_FLASH_SIZE 256
  #endif
#endif

//
// Serial Ports
//

#include "MarlinSerial.h"

/**
 * TODO: review this to return 1 for pins that are not analog input
 */
#ifndef analogInputToDigitalPin
  #define analogInputToDigitalPin(p) pin_t(p)
#endif

#ifndef digitalPinHasPWM
  #define digitalPinHasPWM(P) !!PIN_MAP[P].timer_device
  #define NO_COMPILE_TIME_PWM
#endif

// Reset Reason
#define RST_POWER_ON   1
#define RST_EXTERNAL   2
#define RST_BROWN_OUT  4
#define RST_WATCHDOG   8
#define RST_JTAG       16
#define RST_SOFTWARE   32
#define RST_BACKUP     64

// ------------------------
// Types
// ------------------------

typedef int8_t pin_t;

// ------------------------
// Interrupts
// ------------------------

#define CRITICAL_SECTION_START()  const bool irqon = !__get_primask(); (void)__iCliRetVal()
#define CRITICAL_SECTION_END()    if (!irqon) (void)__iSeiRetVal()
#define cli() noInterrupts()
#define sei() interrupts()

// ------------------------
// ADC
// ------------------------

#ifdef ADC_RESOLUTION
  #define HAL_ADC_RESOLUTION ADC_RESOLUTION
#else
  #define HAL_ADC_RESOLUTION 12
#endif

#define HAL_ADC_VREF_MV   3300

uint16_t analogRead(const pin_t pin); // need hal.adc_enable() first
void analogWrite(const pin_t pin, int pwm_val8); // PWM only! mul by 257 in maple!?

//
// Pin Mapping for M42, M43, M226
//
#define GET_PIN_MAP_PIN(index) index
#define GET_PIN_MAP_INDEX(pin) pin
#define PARSED_PIN_INDEX(code, dval) parser.intval(code, dval)

#define JTAG_DISABLE()    afio_cfg_debug_ports(AFIO_DEBUG_SW_ONLY)
#define JTAGSWD_DISABLE() afio_cfg_debug_ports(AFIO_DEBUG_NONE)

#ifndef PLATFORM_M997_SUPPORT
  #define PLATFORM_M997_SUPPORT
#endif
void flashFirmware(const int16_t);

#define HAL_CAN_SET_PWM_FREQ      // This HAL supports PWM Frequency adjustment
#ifndef PWM_FREQUENCY
  #define PWM_FREQUENCY      1000 // Default PWM Frequency
#endif

// ------------------------
// Class Utilities
// ------------------------

// Memory related
#define __bss_end __bss_end__

extern "C" char* _sbrk(int incr);

void NVIC_SetPriorityGrouping(uint32_t PriorityGroup);

#pragma GCC diagnostic push
#if GCC_VERSION <= 50000
  #pragma GCC diagnostic ignored "-Wunused-function"
#endif

static inline int freeMemory() {
  volatile char top;
  return &top - _sbrk(0);
}

#pragma GCC diagnostic pop

// ------------------------
// MarlinHAL Class
// ------------------------

class MarlinHAL {
public:

  // Earliest possible init, before setup()
  MarlinHAL() {}

  // Watchdog
  static void watchdog_init()    IF_DISABLED(USE_WATCHDOG, {});
  static void watchdog_refresh() IF_DISABLED(USE_WATCHDOG, {});

  static void init();          // Called early in setup()
  static void init_board() {}  // Called less early in setup()
  static void reboot();        // Restart the firmware from 0x0

  // Interrupts
  static bool isr_state() { return !__get_primask(); }
  static void isr_on()  { ((void)__iSeiRetVal()); }
  static void isr_off() { ((void)__iCliRetVal()); }

  static void delay_ms(const int ms) { delay(ms); }

  // Tasks, called from idle()
  static void idletask();

  // Reset
  static uint8_t get_reset_source() { return RST_POWER_ON; }
  static void clear_reset_source() {}

  // Free SRAM
  static int freeMemory() { return ::freeMemory(); }

  //
  // ADC Methods
  //

  static uint16_t adc_result;

  // Called by Temperature::init once at startup
  static void adc_init();

  // Called by Temperature::init for each sensor at startup
  static void adc_enable(const pin_t pin) { pinMode(pin, INPUT_ANALOG); }

  // Begin ADC sampling on the given pin. Called from Temperature::isr!
  static void adc_start(const pin_t pin);

  // Is the ADC ready for reading?
  static bool adc_ready() { return true; }

  // The current value of the ADC register
  static uint16_t adc_value() { return adc_result; }

  /**
   * Set the PWM duty cycle for the pin to the given value.
   * Optionally invert the duty cycle [default = false]
   * Optionally change the maximum size of the provided value to enable finer PWM duty control [default = 255]
   * The timer must be pre-configured with set_pwm_frequency() if the default frequency is not desired.
   */
  static void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t=255, const bool=false);

  /**
   * Set the frequency of the timer for the given pin.
   * All Timer PWM pins run at the same frequency.
   */
  static void set_pwm_frequency(const pin_t pin, const uint16_t f_desired);

};

// ------------------------
// Types
// ------------------------

#define __I
#define __IO volatile
 typedef struct {
   __I  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
   __IO uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
   __IO uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
   __IO uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
   __IO uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
   __IO uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
   __IO uint8_t  SHP[12];                 /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
   __IO uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
   __IO uint32_t CFSR;                    /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register                    */
   __IO uint32_t HFSR;                    /*!< Offset: 0x02C (R/W)  HardFault Status Register                             */
   __IO uint32_t DFSR;                    /*!< Offset: 0x030 (R/W)  Debug Fault Status Register                           */
   __IO uint32_t MMFAR;                   /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register                      */
   __IO uint32_t BFAR;                    /*!< Offset: 0x038 (R/W)  BusFault Address Register                             */
   __IO uint32_t AFSR;                    /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register                       */
   __I  uint32_t PFR[2];                  /*!< Offset: 0x040 (R/ )  Processor Feature Register                            */
   __I  uint32_t DFR;                     /*!< Offset: 0x048 (R/ )  Debug Feature Register                                */
   __I  uint32_t ADR;                     /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register                            */
   __I  uint32_t MMFR[4];                 /*!< Offset: 0x050 (R/ )  Memory Model Feature Register                         */
   __I  uint32_t ISAR[5];                 /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register                   */
        uint32_t RESERVED0[5];
   __IO uint32_t CPACR;                   /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register                   */
 } SCB_Type;

// ------------------------
// System Control Space
// ------------------------

#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address  */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address  */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */

/* SCB Application Interrupt and Reset Control Register Definitions */
#define SCB_AIRCR_VECTKEY_Pos              16                                             /*!< SCB AIRCR: VECTKEY Position */
#define SCB_AIRCR_VECTKEY_Msk              (0xFFFFUL << SCB_AIRCR_VECTKEY_Pos)            /*!< SCB AIRCR: VECTKEY Mask */

#define SCB_AIRCR_PRIGROUP_Pos              8                                             /*!< SCB AIRCR: PRIGROUP Position */
#define SCB_AIRCR_PRIGROUP_Msk             (7UL << SCB_AIRCR_PRIGROUP_Pos)                /*!< SCB AIRCR: PRIGROUP Mask */
