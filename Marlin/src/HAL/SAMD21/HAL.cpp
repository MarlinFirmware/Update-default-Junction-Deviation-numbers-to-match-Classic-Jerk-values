/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
 * SAMD21 HAL developed by Bart Meijer (brupje)
 * Based on SAMD51 HAL by Giuliano Zaro (AKA GMagician)
 */
#ifdef __SAMD21__

#include "../../inc/MarlinConfig.h"

#include <wiring_private.h>

#if USING_HW_SERIALUSB
  DefaultSerial1 MSerialUSB(false, SerialUSB);
#endif
#if USING_HW_SERIAL0
  DefaultSerial2 MSerial1(false, Serial1);
#endif
#if USING_HW_SERIAL1
  DefaultSerial3 MSerial2(false, Serial2);
#endif

#define WDT_CONFIG_PER_7_Val 0x9u
#define WDT_CONFIG_PER_Pos 0
#define WDT_CONFIG_PER_7 (WDT_CONFIG_PER_7_Val << WDT_CONFIG_PER_Pos)

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_REG TERN(WATCHDOG_DURATION_8S, WDT_CONFIG_PER_CYC8192, WDT_CONFIG_PER_CYC4096) // 4 or 8 second timeout

  void MarlinHAL::watchdog_init() {
       // Set up the generic clock (GCLK2) used to clock the watchdog timer at 1.024kHz
    GCLK->GENDIV.reg = GCLK_GENDIV_DIV(4) |            // Divide the 32.768kHz clock source by divisor 32, where 2^(4 + 1): 32.768kHz/32=1.024kHz
                      GCLK_GENDIV_ID(2);              // Select Generic Clock (GCLK) 2
    while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

    REG_GCLK_GENCTRL =  GCLK_GENCTRL_DIVSEL |         // Set to divide by 2^(GCLK_GENDIV_DIV(4) + 1)
                        GCLK_GENCTRL_IDC |            // Set the duty cycle to 50/50 HIGH/LOW
                        GCLK_GENCTRL_GENEN |          // Enable GCLK2
                        GCLK_GENCTRL_SRC_OSCULP32K |  // Set the clock source to the ultra low power oscillator (OSCULP32K)
                        GCLK_GENCTRL_ID(2);           // Select GCLK2
    while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

    // Feed GCLK2 to WDT (Watchdog Timer)
    REG_GCLK_CLKCTRL =  GCLK_CLKCTRL_CLKEN |          // Enable GCLK2 to the WDT
                        GCLK_CLKCTRL_GEN_GCLK2 |      // Select GCLK2
                        GCLK_CLKCTRL_ID_WDT;          // Feed the GCLK2 to the WDT
    while (GCLK->STATUS.bit.SYNCBUSY);                // Wait for synchronization

    WDT->CONFIG.bit.PER = WDT_CONFIG_PER_7;                // Set the WDT reset timeout to 4 seconds
    while (WDT->STATUS.bit.SYNCBUSY);                 // Wait for synchronization
    REG_WDT_CTRL = WDT_CTRL_ENABLE;                   // Enable the WDT in normal mode
    while (WDT->STATUS.bit.SYNCBUSY);                 // Wait for synchronization
  }

  // Reset watchdog. MUST be called at least every 4 seconds after the
  // first watchdog_init or SAMD will go into emergency procedures.
  void MarlinHAL::watchdog_refresh() {
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
    while (WDT->STATUS.bit.SYNCBUSY);
  }

#endif

// ------------------------
// Types
// ------------------------

// ------------------------
// Private Variables
// ------------------------

// ------------------------
// Private functions
// ------------------------

void MarlinHAL::dma_init() {}

// ------------------------
// Public functions
// ------------------------

// HAL initialization task
void MarlinHAL::init() {
  TERN_(DMA_IS_REQUIRED, dma_init());
  #if HAS_MEDIA
    #if HAS_SD_DETECT && SD_CONNECTION_IS(ONBOARD)
      SET_INPUT_PULLUP(SD_DETECT_PIN);
    #endif
    OUT_WRITE(SD_SS_PIN, HIGH);  // Try to set SDSS inactive before any other SPI users start up
  #endif
}

#pragma push_macro("WDT")
#undef WDT    // Required to be able to use '.bit.WDT'. Compiler wrongly replace struct field with WDT define
uint8_t MarlinHAL::get_reset_source() {

  return 0;
}
#pragma pop_macro("WDT")

void MarlinHAL::reboot() { NVIC_SystemReset(); }

extern "C" {
  void * _sbrk(int incr);
  extern unsigned int __bss_end__; // end of bss section
}

// Return free memory between end of heap (or end bss) and whatever is current
int freeMemory() {
  int free_memory, heap_end = (int)_sbrk(0);
  return (int)&free_memory - (heap_end ?: (int)&__bss_end__);
}

// ------------------------
// ADC
// ------------------------

uint16_t MarlinHAL::adc_result;

void MarlinHAL::adc_init() {
  /* thanks to https://www.eevblog.com/forum/microcontrollers/samd21g18-adc-with-resrdy-interrupts-only-reads-once-or-twice/ */

  ADC->CTRLA.bit.ENABLE = false;
  while(ADC->STATUS.bit.SYNCBUSY);

  // load chip corrections
  uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
  uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
  linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {};

  ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);

  /* Wait for bus synchronization. */
  while (ADC->STATUS.bit.SYNCBUSY) {};

  ADC->CTRLA.bit.SWRST = true;
  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->REFCTRL.reg = ADC_REFCTRL_REFSEL_INTVCC1;
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_32| ADC_AVGCTRL_ADJRES(4);;

  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV128 |
                 ADC_CTRLB_RESSEL_16BIT |
                 ADC_CTRLB_FREERUN;
  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->SAMPCTRL.bit.SAMPLEN = 0x00;
  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->INPUTCTRL.reg =  ADC_INPUTCTRL_INPUTSCAN(HAL_ADC_AIN_LEN) // scan  (INPUTSCAN + NUM_EXTUDERS - 1) pins
  | ADC_INPUTCTRL_GAIN_DIV2  |ADC_INPUTCTRL_MUXNEG_GND| HAL_ADC_AIN_START ;  /* set to first AIN */

  while(ADC->STATUS.bit.SYNCBUSY);

  ADC->INTENSET.reg |= ADC_INTENSET_RESRDY; // enable Result Ready ADC interrupts
  while (ADC->STATUS.bit.SYNCBUSY);

  NVIC_EnableIRQ(ADC_IRQn); // enable ADC interrupts

  NVIC_SetPriority(ADC_IRQn, 3);

  ADC->CTRLA.bit.ENABLE = true;
}

volatile uint32_t adc_results[HAL_ADC_AIN_NUM_SENSORS];

void ADC_Handler() {
  while(ADC->STATUS.bit.SYNCBUSY == 1);
  int pos = ADC->INPUTCTRL.bit.INPUTOFFSET;

  adc_results[pos] = ADC->RESULT.reg; /* Read the value. */
  ADC->INTFLAG.reg = ADC_INTENSET_RESRDY; /* Clear the data ready flag. */
}

void MarlinHAL::adc_start(const pin_t pin) {
  /* due to the way INPUTOFFSET works, the last sensor is the first position in the array
    and we want the ADC_handler interrupt to be as simple possible, so we do the calculation here.
  */
  unsigned int pos = PIN_TO_INPUTCTRL(pin) - HAL_ADC_AIN_START + 1;
  if (pos == HAL_ADC_AIN_NUM_SENSORS) pos = 0;
  adc_result = adc_results[pos]; // 16-bit resolution
  //adc_result = 0xFFFF;
}

#endif // __SAMD21__
