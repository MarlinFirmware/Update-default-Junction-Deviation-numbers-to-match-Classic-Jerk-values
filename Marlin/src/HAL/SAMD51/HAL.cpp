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
 * SAMD51 HAL developed by Giuliano Zaro (AKA GMagician)
 */
#ifdef __SAMD51__

#include "../../inc/MarlinConfig.h"
#include <Adafruit_ZeroDMA.h>
#include <wiring_private.h>

#ifdef ADAFRUIT_GRAND_CENTRAL_M4
  #if USING_HW_SERIALUSB
    DefaultSerial1 MSerial0(false, Serial);
  #endif
  #if USING_HW_SERIAL0
    DefaultSerial2 MSerial1(false, Serial1);
  #endif
  #if USING_HW_SERIAL1
    DefaultSerial3 MSerial2(false, Serial2);
  #endif
  #if USING_HW_SERIAL2
    DefaultSerial4 MSerial3(false, Serial3);
  #endif
  #if USING_HW_SERIAL3
    DefaultSerial5 MSerial4(false, Serial4);
  #endif
#endif

#define GET_TEMP_0_ADC()            TERN(HAS_TEMP_ADC_0,        PIN_TO_ADC(TEMP_0_PIN),                 -1)
#define GET_TEMP_1_ADC()            TERN(HAS_TEMP_ADC_1,        PIN_TO_ADC(TEMP_1_PIN),                 -1)
#define GET_TEMP_2_ADC()            TERN(HAS_TEMP_ADC_2,        PIN_TO_ADC(TEMP_2_PIN),                 -1)
#define GET_TEMP_3_ADC()            TERN(HAS_TEMP_ADC_3,        PIN_TO_ADC(TEMP_3_PIN),                 -1)
#define GET_TEMP_4_ADC()            TERN(HAS_TEMP_ADC_4,        PIN_TO_ADC(TEMP_4_PIN),                 -1)
#define GET_TEMP_5_ADC()            TERN(HAS_TEMP_ADC_5,        PIN_TO_ADC(TEMP_5_PIN),                 -1)
#define GET_TEMP_6_ADC()            TERN(HAS_TEMP_ADC_6,        PIN_TO_ADC(TEMP_6_PIN),                 -1)
#define GET_TEMP_7_ADC()            TERN(HAS_TEMP_ADC_7,        PIN_TO_ADC(TEMP_7_PIN),                 -1)
#define GET_BED_ADC()               TERN(HAS_TEMP_ADC_BED,      PIN_TO_ADC(TEMP_BED_PIN),               -1)
#define GET_CHAMBER_ADC()           TERN(HAS_TEMP_ADC_CHAMBER,  PIN_TO_ADC(TEMP_CHAMBER_PIN),           -1)
#define GET_PROBE_ADC()             TERN(HAS_TEMP_ADC_PROBE,    PIN_TO_ADC(TEMP_PROBE_PIN),             -1)
#define GET_COOLER_ADC()            TERN(HAS_TEMP_ADC_COOLER,   PIN_TO_ADC(TEMP_COOLER_PIN),            -1)
#define GET_BOARD_ADC()             TERN(HAS_TEMP_ADC_BOARD,    PIN_TO_ADC(TEMP_BOARD_PIN),             -1)
#define GET_SOC_ADC()               TERN(HAS_TEMP_ADC_BOARD,    PIN_TO_ADC(TEMP_BOARD_PIN),             -1)
#define GET_FILAMENT_WIDTH_ADC()    TERN(FILAMENT_WIDTH_SENSOR, PIN_TO_ADC(FILWIDTH_PIN),               -1)
#define GET_BUTTONS_ADC()           TERN(HAS_ADC_BUTTONS,       PIN_TO_ADC(ADC_KEYPAD_PIN),             -1)
#define GET_JOY_ADC_X()             TERN(HAS_JOY_ADC_X,         PIN_TO_ADC(JOY_X_PIN),                  -1)
#define GET_JOY_ADC_Y()             TERN(HAS_JOY_ADC_Y,         PIN_TO_ADC(JOY_Y_PIN),                  -1)
#define GET_JOY_ADC_Z()             TERN(HAS_JOY_ADC_Z,         PIN_TO_ADC(JOY_Z_PIN),                  -1)
#define GET_POWERMON_ADC_CURRENT()  TERN(POWER_MONITOR_CURRENT, PIN_TO_ADC(POWER_MONITOR_CURRENT_PIN),  -1)
#define GET_POWERMON_ADC_VOLTS()    TERN(POWER_MONITOR_VOLTAGE, PIN_TO_ADC(POWER_MONITOR_VOLTAGE_PIN),  -1)

#define IS_ADC_REQUIRED(n) ( \
     GET_TEMP_0_ADC() == n || GET_TEMP_1_ADC() == n || GET_TEMP_2_ADC() == n || GET_TEMP_3_ADC() == n \
  || GET_TEMP_4_ADC() == n || GET_TEMP_5_ADC() == n || GET_TEMP_6_ADC() == n || GET_TEMP_7_ADC() == n \
  || GET_BED_ADC() == n \
  || GET_CHAMBER_ADC() == n \
  || GET_PROBE_ADC() == n \
  || GET_COOLER_ADC() == n \
  || GET_BOARD_ADC() == n || GET_SOC_ADC() == n \
  || GET_FILAMENT_WIDTH_ADC() == n \
  || GET_BUTTONS_ADC() == n \
  || GET_JOY_ADC_X() == n || GET_JOY_ADC_Y() == n || GET_JOY_ADC_Z() == n \
  || GET_POWERMON_ADC_CURRENT() == n || GET_POWERMON_ADC_VOLTS() == n \
)

#if IS_ADC_REQUIRED(0)
  #define ADC0_IS_REQUIRED 1
  #define FIRST_ADC     0
#else
  #define FIRST_ADC     1
#endif
#if IS_ADC_REQUIRED(1)
  #define ADC1_IS_REQUIRED 1
  #define LAST_ADC      1
#else
  #define LAST_ADC      0
#endif
#if ADC0_IS_REQUIRED || ADC1_IS_REQUIRED
  #define ADC_IS_REQUIRED 1
  #define DMA_IS_REQUIRED 1
#endif

enum ADCIndex {
  #if GET_TEMP_0_ADC() == 0
    TEMP_0,
  #endif
  #if GET_TEMP_1_ADC() == 0
    TEMP_1,
  #endif
  #if GET_TEMP_2_ADC() == 0
    TEMP_2,
  #endif
  #if GET_TEMP_3_ADC() == 0
    TEMP_3,
  #endif
  #if GET_TEMP_4_ADC() == 0
    TEMP_4,
  #endif
  #if GET_TEMP_5_ADC() == 0
    TEMP_5,
  #endif
  #if GET_TEMP_6_ADC() == 0
    TEMP_6,
  #endif
  #if GET_TEMP_7_ADC() == 0
    TEMP_7,
  #endif
  #if GET_BED_ADC() == 0
    TEMP_BED,
  #endif
  #if GET_CHAMBER_ADC() == 0
    TEMP_CHAMBER,
  #endif
  #if GET_PROBE_ADC() == 0
    TEMP_PROBE,
  #endif
  #if GET_COOLER_ADC() == 0
    TEMP_COOLER,
  #endif
  #if GET_BOARD_ADC() == 0
    TEMP_BOARD,
  #endif
  #if GET_SOC_ADC() == 0
    TEMP_SOC,
  #endif
  #if GET_FILAMENT_WIDTH_ADC() == 0
    FILWIDTH,
  #endif
  #if GET_BUTTONS_ADC() == 0
    ADC_KEY,
  #endif
  #if GET_JOY_ADC_X() == 0
    JOY_X,
  #endif
  #if GET_JOY_ADC_Y() == 0
    JOY_Y,
  #endif
  #if GET_JOY_ADC_Z() == 0
    JOY_Z,
  #endif
  #if GET_POWERMON_ADC_CURRENT() == 0
    POWERMON_CURRENT,
  #endif
  #if GET_POWERMON_ADC_VOLTS() == 0
    POWERMON_VOLTAGE,
  #endif

  // Indexes for ADC1 after those for ADC0

  #if GET_TEMP_0_ADC() == 1
    TEMP_0,
  #endif
  #if GET_TEMP_1_ADC() == 1
    TEMP_1,
  #endif
  #if GET_TEMP_2_ADC() == 1
    TEMP_2,
  #endif
  #if GET_TEMP_3_ADC() == 1
    TEMP_3,
  #endif
  #if GET_TEMP_4_ADC() == 1
    TEMP_4,
  #endif
  #if GET_TEMP_5_ADC() == 1
    TEMP_5,
  #endif
  #if GET_TEMP_6_ADC() == 1
    TEMP_6,
  #endif
  #if GET_TEMP_7_ADC() == 1
    TEMP_7,
  #endif
  #if GET_BED_ADC() == 1
    TEMP_BED,
  #endif
  #if GET_CHAMBER_ADC() == 1
    TEMP_CHAMBER,
  #endif
  #if GET_PROBE_ADC() == 1
    TEMP_PROBE,
  #endif
  #if GET_COOLER_ADC() == 1
    TEMP_COOLER,
  #endif
  #if GET_BOARD_ADC() == 1
    TEMP_BOARD,
  #endif
  #if GET_SOC_ADC() == 1
    TEMP_SOC,
  #endif
  #if GET_FILAMENT_WIDTH_ADC() == 1
    FILWIDTH,
  #endif
  #if GET_BUTTONS_ADC() == 1
    ADC_KEY,
  #endif
  #if GET_JOY_ADC_X() == 1
    JOY_X,
  #endif
  #if GET_JOY_ADC_Y() == 1
    JOY_Y,
  #endif
  #if GET_JOY_ADC_Z() == 1
    JOY_Z,
  #endif
  #if GET_POWERMON_ADC_CURRENT() == 1
    POWERMON_CURRENT,
  #endif
  #if GET_POWERMON_ADC_VOLTS() == 1
    POWERMON_VOLTAGE,
  #endif
  ADC_COUNT
};

#if ENABLED(USE_WATCHDOG)

  #define WDT_TIMEOUT_REG TERN(WATCHDOG_DURATION_8S, WDT_CONFIG_PER_CYC8192, WDT_CONFIG_PER_CYC4096) // 4 or 8 second timeout

  void MarlinHAL::watchdog_init() {
    // The low-power oscillator used by the WDT runs at 32,768 Hz with
    // a 1:32 prescale, thus 1024 Hz, though probably not super precise.

    // Setup WDT clocks
    MCLK->APBAMASK.bit.OSC32KCTRL_ = true;
    MCLK->APBAMASK.bit.WDT_ = true;
    OSC32KCTRL->OSCULP32K.bit.EN1K = true;  // Enable out 1K (this is what WDT uses)

    WDT->CTRLA.bit.ENABLE = false;          // Disable watchdog for config
    SYNC(WDT->SYNCBUSY.bit.ENABLE);

    WDT->INTENCLR.reg = WDT_INTENCLR_EW;    // Disable early warning interrupt
    WDT->CONFIG.reg = WDT_TIMEOUT_REG;      // Set a 4s or 8s period for chip reset

    hal.watchdog_refresh();

    WDT->CTRLA.reg = WDT_CTRLA_ENABLE;      // Start watchdog now in normal mode
    SYNC(WDT->SYNCBUSY.bit.ENABLE);
  }

  // Reset watchdog. MUST be called at least every 4 seconds after the
  // first watchdog_init or SAMD will go into emergency procedures.
  void MarlinHAL::watchdog_refresh() {
    SYNC(WDT->SYNCBUSY.bit.CLEAR);        // Test first if previous is 'ongoing' to save time waiting for command execution
    WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;
  }

#endif

// ------------------------
// Types
// ------------------------

#if DMA_IS_REQUIRED

  // Struct must be 32 bits aligned because of DMA accesses but fields needs to be 8 bits packed
  typedef struct  __attribute__((aligned(4), packed)) {
    ADC_INPUTCTRL_Type INPUTCTRL;
  } HAL_DMA_DAC_Registers;    // DMA transferred registers

#endif

// ------------------------
// Private Variables
// ------------------------

#if ADC_IS_REQUIRED

  // Pins used by ADC inputs. Order must be ADC0 inputs first then ADC1
  static constexpr uint8_t adc_pins[ADC_COUNT] = {
    // ADC0 pins
    #if GET_TEMP_0_ADC() == 0
      TEMP_0_PIN,
    #endif
    #if GET_TEMP_1_ADC() == 0
      TEMP_1_PIN,
    #endif
    #if GET_TEMP_2_ADC() == 0
      TEMP_2_PIN,
    #endif
    #if GET_TEMP_3_ADC() == 0
      TEMP_3_PIN,
    #endif
    #if GET_TEMP_4_ADC() == 0
      TEMP_4_PIN,
    #endif
    #if GET_TEMP_5_ADC() == 0
      TEMP_5_PIN,
    #endif
    #if GET_TEMP_6_ADC() == 0
      TEMP_6_PIN,
    #endif
    #if GET_TEMP_7_ADC() == 0
      TEMP_7_PIN,
    #endif
    #if GET_BED_ADC() == 0
      TEMP_BED_PIN,
    #endif
    #if GET_CHAMBER_ADC() == 0
      TEMP_CHAMBER_PIN,
    #endif
    #if GET_PROBE_ADC() == 0
      TEMP_PROBE_PIN,
    #endif
    #if GET_COOLER_ADC() == 0
      TEMP_COOLER_PIN,
    #endif
    #if GET_BOARD_ADC() == 0
      TEMP_BOARD_PIN,
    #endif
    #if GET_SOC_ADC() == 0
      TEMP_SOC_PIN,
    #endif
    #if GET_FILAMENT_WIDTH_ADC() == 0
      FILWIDTH_PIN,
    #endif
    #if GET_BUTTONS_ADC() == 0
      ADC_KEYPAD_PIN,
    #endif
    #if GET_JOY_ADC_X() == 0
      JOY_X_PIN,
    #endif
    #if GET_JOY_ADC_Y() == 0
      JOY_Y_PIN,
    #endif
    #if GET_JOY_ADC_Z() == 0
      JOY_Z_PIN,
    #endif
    #if GET_POWERMON_ADC_CURRENT() == 0
      POWER_MONITOR_CURRENT_PIN,
    #endif
    #if GET_POWERMON_ADC_VOLTS() == 0
      POWER_MONITOR_VOLTAGE_PIN,
    #endif

    // Pins for ADC1 after ADC0

    #if GET_TEMP_0_ADC() == 1
      TEMP_0_PIN,
    #endif
    #if GET_TEMP_1_ADC() == 1
      TEMP_1_PIN,
    #endif
    #if GET_TEMP_2_ADC() == 1
      TEMP_2_PIN,
    #endif
    #if GET_TEMP_3_ADC() == 1
      TEMP_3_PIN,
    #endif
    #if GET_TEMP_4_ADC() == 1
      TEMP_4_PIN,
    #endif
    #if GET_TEMP_5_ADC() == 1
      TEMP_5_PIN,
    #endif
    #if GET_TEMP_6_ADC() == 1
      TEMP_6_PIN,
    #endif
    #if GET_TEMP_7_ADC() == 1
      TEMP_7_PIN,
    #endif
    #if GET_BED_ADC() == 1
      TEMP_BED_PIN,
    #endif
    #if GET_CHAMBER_ADC() == 1
      TEMP_CHAMBER_PIN,
    #endif
    #if GET_PROBE_ADC() == 1
      TEMP_PROBE_PIN,
    #endif
    #if GET_COOLER_ADC() == 1
      TEMP_COOLER_PIN,
    #endif
    #if GET_BOARD_ADC() == 1
      TEMP_BOARD_PIN,
    #endif
    #if GET_SOC_ADC() == 1
      TEMP_SOC_PIN,
    #endif
    #if GET_FILAMENT_WIDTH_ADC() == 1
      FILWIDTH_PIN,
    #endif
    #if GET_BUTTONS_ADC() == 1
      ADC_KEYPAD_PIN,
    #endif
    #if GET_JOY_ADC_X() == 1
      JOY_X_PIN,
    #endif
    #if GET_JOY_ADC_Y() == 1
      JOY_Y_PIN,
    #endif
    #if GET_JOY_ADC_Z() == 1
      JOY_Z_PIN,
    #endif
    #if GET_POWERMON_ADC_CURRENT() == 1
      POWER_MONITOR_CURRENT_PIN,
    #endif
    #if GET_POWERMON_ADC_VOLTS() == 1
      POWER_MONITOR_VOLTAGE_PIN,
    #endif
  };

  static uint16_t adc_results[ADC_COUNT];

  #if ADC0_IS_REQUIRED
    Adafruit_ZeroDMA adc0DMAProgram, adc0DMARead;

    static constexpr HAL_DMA_DAC_Registers adc0_dma_regs_list[ADC_COUNT] = {
      #if GET_TEMP_0_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_0_PIN) },
      #endif
      #if GET_TEMP_1_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_1_PIN) },
      #endif
      #if GET_TEMP_2_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_2_PIN) },
      #endif
      #if GET_TEMP_3_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_3_PIN) },
      #endif
      #if GET_TEMP_4_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_4_PIN) },
      #endif
      #if GET_TEMP_5_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_5_PIN) },
      #endif
      #if GET_TEMP_6_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_6_PIN) },
      #endif
      #if GET_TEMP_7_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_7_PIN) },
      #endif
      #if GET_BED_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_BED_PIN) },
      #endif
      #if GET_CHAMBER_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_CHAMBER_PIN) },
      #endif
      #if GET_PROBE_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_PROBE_PIN) },
      #endif
      #if GET_COOLER_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_COOLER_PIN) },
      #endif
      #if GET_BOARD_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_BOARD_PIN) },
      #endif
      #if GET_SOC_ADC() == 0
        { PIN_TO_INPUTCTRL(TEMP_SOC_PIN) },
      #endif
      #if GET_FILAMENT_WIDTH_ADC() == 0
        { PIN_TO_INPUTCTRL(FILWIDTH_PIN) },
      #endif
      #if GET_BUTTONS_ADC() == 0
        { PIN_TO_INPUTCTRL(ADC_KEYPAD_PIN) },
      #endif
      #if GET_JOY_ADC_X() == 0
        { PIN_TO_INPUTCTRL(JOY_X_PIN) },
      #endif
      #if GET_JOY_ADC_Y() == 0
        { PIN_TO_INPUTCTRL(JOY_Y_PIN) },
      #endif
      #if GET_JOY_ADC_Z() == 0
        { PIN_TO_INPUTCTRL(JOY_Z_PIN) },
      #endif
      #if GET_POWERMON_ADC_CURRENT() == 0
        { PIN_TO_INPUTCTRL(POWER_MONITOR_CURRENT_PIN) },
      #endif
      #if GET_POWERMON_ADC_VOLTS() == 0
        { PIN_TO_INPUTCTRL(POWER_MONITOR_VOLTAGE_PIN) },
      #endif
    };

    #define ADC0_AINCOUNT   COUNT(adc0_dma_regs_list)
  #endif // ADC0_IS_REQUIRED

  #if ADC1_IS_REQUIRED
    Adafruit_ZeroDMA adc1DMAProgram, adc1DMARead;

    static constexpr HAL_DMA_DAC_Registers adc1_dma_regs_list[ADC_COUNT] = {
      #if GET_TEMP_0_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_0_PIN) },
      #endif
      #if GET_TEMP_1_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_1_PIN) },
      #endif
      #if GET_TEMP_2_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_2_PIN) },
      #endif
      #if GET_TEMP_3_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_3_PIN) },
      #endif
      #if GET_TEMP_4_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_4_PIN) },
      #endif
      #if GET_TEMP_5_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_5_PIN) },
      #endif
      #if GET_TEMP_6_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_6_PIN) },
      #endif
      #if GET_TEMP_7_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_7_PIN) },
      #endif
      #if GET_BED_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_BED_PIN) },
      #endif
      #if GET_CHAMBER_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_CHAMBER_PIN) },
      #endif
      #if GET_PROBE_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_PROBE_PIN) },
      #endif
      #if GET_COOLER_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_COOLER_PIN) },
      #endif
      #if GET_BOARD_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_BOARD_PIN) },
      #endif
      #if GET_SOC_ADC() == 1
        { PIN_TO_INPUTCTRL(TEMP_SOC_PIN) },
      #endif
      #if GET_FILAMENT_WIDTH_ADC() == 1
        { PIN_TO_INPUTCTRL(FILWIDTH_PIN) },
      #endif
      #if GET_BUTTONS_ADC() == 1
        { PIN_TO_INPUTCTRL(ADC_KEYPAD_PIN) },
      #endif
      #if GET_JOY_ADC_X() == 1
        { PIN_TO_INPUTCTRL(JOY_X_PIN) },
      #endif
      #if GET_JOY_ADC_Y() == 1
        { PIN_TO_INPUTCTRL(JOY_Y_PIN) },
      #endif
      #if GET_JOY_ADC_Z() == 1
        { PIN_TO_INPUTCTRL(JOY_Z_PIN) },
      #endif
      #if GET_POWERMON_ADC_CURRENT() == 1
        { PIN_TO_INPUTCTRL(POWER_MONITOR_CURRENT_PIN) },
      #endif
      #if GET_POWERMON_ADC_VOLTS() == 1
        { PIN_TO_INPUTCTRL(POWER_MONITOR_VOLTAGE_PIN) },
      #endif
    };

    #define ADC1_AINCOUNT   COUNT(adc1_dma_regs_list)
  #endif // ADC1_IS_REQUIRED

#endif // ADC_IS_REQUIRED

// ------------------------
// Private functions
// ------------------------

void MarlinHAL::dma_init() {

  #if DMA_IS_REQUIRED

    DmacDescriptor *descriptor;

    #if ADC0_IS_REQUIRED
      adc0DMAProgram.setTrigger(ADC0_DMAC_ID_SEQ);
      adc0DMAProgram.setAction(DMA_TRIGGER_ACTON_BEAT);
      adc0DMAProgram.loop(true);
      if (adc0DMAProgram.allocate() == DMA_STATUS_OK) {
        descriptor = adc0DMAProgram.addDescriptor(
          (void *)adc0_dma_regs_list,         // SRC
          (void *)&ADC0->DSEQDATA.reg,        // DEST
          sizeof(adc0_dma_regs_list) / 4,     // CNT
          DMA_BEAT_SIZE_WORD,
          true,                               // SRCINC
          false,                              // DSTINC
          DMA_ADDRESS_INCREMENT_STEP_SIZE_1,  // STEPSIZE
          DMA_STEPSEL_SRC                     // STEPSEL
        );
        if (descriptor)
          descriptor->BTCTRL.bit.EVOSEL = DMA_EVENT_OUTPUT_BEAT;
        adc0DMAProgram.startJob();
      }

      adc0DMARead.setTrigger(ADC0_DMAC_ID_RESRDY);
      adc0DMARead.setAction(DMA_TRIGGER_ACTON_BEAT);
      adc0DMARead.loop(true);
      if (adc0DMARead.allocate() == DMA_STATUS_OK) {
        adc0DMARead.addDescriptor(
          (void *)&ADC0->RESULT.reg,          // SRC
          &adc_results,                       // DEST
          ADC0_AINCOUNT,                      // CNT
          DMA_BEAT_SIZE_HWORD,
          false,                              // SRCINC
          true,                               // DSTINC
          DMA_ADDRESS_INCREMENT_STEP_SIZE_1,  // STEPSIZE
          DMA_STEPSEL_DST                     // STEPSEL
        );
        adc0DMARead.startJob();
      }
    #endif
    #if ADC1_IS_REQUIRED
      adc1DMAProgram.setTrigger(ADC1_DMAC_ID_SEQ);
      adc1DMAProgram.setAction(DMA_TRIGGER_ACTON_BEAT);
      adc1DMAProgram.loop(true);
      if (adc1DMAProgram.allocate() == DMA_STATUS_OK) {
        descriptor = adc1DMAProgram.addDescriptor(
          (void *)adc1_dma_regs_list,         // SRC
          (void *)&ADC1->DSEQDATA.reg,        // DEST
          sizeof(adc1_dma_regs_list) / 4,     // CNT
          DMA_BEAT_SIZE_WORD,
          true,                               // SRCINC
          false,                              // DSTINC
          DMA_ADDRESS_INCREMENT_STEP_SIZE_1,  // STEPSIZE
          DMA_STEPSEL_SRC                     // STEPSEL
        );
        if (descriptor)
          descriptor->BTCTRL.bit.EVOSEL = DMA_EVENT_OUTPUT_BEAT;
        adc1DMAProgram.startJob();
      }

      adc1DMARead.setTrigger(ADC1_DMAC_ID_RESRDY);
      adc1DMARead.setAction(DMA_TRIGGER_ACTON_BEAT);
      adc1DMARead.loop(true);
      if (adc1DMARead.allocate() == DMA_STATUS_OK) {
        adc1DMARead.addDescriptor(
          (void *)&ADC1->RESULT.reg,          // SRC
          &adc_results[ADC0_AINCOUNT],        // DEST
          ADC1_AINCOUNT,                      // CNT
          DMA_BEAT_SIZE_HWORD,
          false,                              // SRCINC
          true,                               // DSTINC
          DMA_ADDRESS_INCREMENT_STEP_SIZE_1,  // STEPSIZE
          DMA_STEPSEL_DST                     // STEPSEL
        );
        adc1DMARead.startJob();
      }
    #endif

    DMAC->PRICTRL0.bit.RRLVLEN0 = true;                         // Activate round robin for DMA channels required by ADCs

  #endif // DMA_IS_REQUIRED
}

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
  RSTC_RCAUSE_Type resetCause;

  resetCause.reg = REG_RSTC_RCAUSE;
  if (resetCause.bit.POR)                                   return RST_POWER_ON;
  else if (resetCause.bit.EXT)                              return RST_EXTERNAL;
  else if (resetCause.bit.BODCORE || resetCause.bit.BODVDD) return RST_BROWN_OUT;
  else if (resetCause.bit.WDT)                              return RST_WATCHDOG;
  else if (resetCause.bit.SYST || resetCause.bit.NVM)       return RST_SOFTWARE;
  else if (resetCause.bit.BACKUP)                           return RST_BACKUP;
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
  #if ADC_IS_REQUIRED
    memset(adc_results, 0xFF, sizeof(adc_results));                         // Fill result with invalid values

    for (uint8_t pi = 0; pi < COUNT(adc_pins); ++pi)
      pinPeripheral(adc_pins[pi], PIO_ANALOG);

    for (uint8_t ai = FIRST_ADC; ai <= LAST_ADC; ++ai) {
      Adc* adc = ((Adc*[])ADC_INSTS)[ai];

      // ADC clock setup
      GCLK->PCHCTRL[ADC0_GCLK_ID + ai].bit.CHEN = false;
      SYNC(GCLK->PCHCTRL[ADC0_GCLK_ID + ai].bit.CHEN);
      GCLK->PCHCTRL[ADC0_GCLK_ID + ai].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;   // 48MHz startup code programmed
      SYNC(!GCLK->PCHCTRL[ADC0_GCLK_ID + ai].bit.CHEN);
      adc->CTRLA.bit.PRESCALER = ADC_CTRLA_PRESCALER_DIV32_Val;             // 1.5MHZ adc clock

      // ADC setup
      // Preloaded data (fixed for all ADC instances hence not loaded by DMA)
      adc->REFCTRL.bit.REFSEL = ADC_REFCTRL_REFSEL_AREFA_Val;               // VRefA pin
      SYNC(adc->SYNCBUSY.bit.REFCTRL);
      adc->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;                   // ... ADC_CTRLB_RESSEL_16BIT_Val
      SYNC(adc->SYNCBUSY.bit.CTRLB);
      adc->SAMPCTRL.bit.SAMPLEN = (6 - 1);                                  // Sampling clocks
      //adc->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_16 | ADC_AVGCTRL_ADJRES(4);  // 16 Accumulated conversions and shift 4 to get oversampled 12 bits result
      //SYNC(adc->SYNCBUSY.bit.AVGCTRL);

      // Registers loaded by DMA
      adc->DSEQCTRL.bit.INPUTCTRL = true;
      adc->DSEQCTRL.bit.AUTOSTART = true;                                   // Start conversion after DMA sequence

      adc->CTRLA.bit.ENABLE = true;                                         // Enable ADC
      SYNC(adc->SYNCBUSY.bit.ENABLE);
    }
  #endif // ADC_IS_REQUIRED
}

void MarlinHAL::adc_start(const pin_t pin) {
  #if ADC_IS_REQUIRED
    for (uint8_t pi = 0; pi < COUNT(adc_pins); ++pi)
      if (pin == adc_pins[pi]) { adc_result = adc_results[pi]; return; }
  #endif

  adc_result = 0xFFFF;
}

#endif // __SAMD51__
