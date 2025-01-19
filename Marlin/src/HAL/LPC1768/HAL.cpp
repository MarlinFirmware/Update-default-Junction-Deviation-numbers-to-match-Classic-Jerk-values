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
#ifdef TARGET_LPC1768

#include "../../inc/MarlinConfig.h"
#include "../shared/Delay.h"
#include "../../core/millis_t.h"

#include <usb/usb.h>
#include <usb/usbcfg.h>
#include <usb/usbhw.h>
#include <usb/usbcore.h>
#include <usb/cdc.h>
#include <usb/cdcuser.h>
#include <usb/mscuser.h>
#include <CDCSerial.h>
#include <usb/mscuser.h>

uint32_t MarlinHAL::adc_result = 0;
pin_t MarlinHAL::adc_pin = 0;

// U8glib required functions
extern "C" {
  void u8g_xMicroDelay(uint16_t val) { DELAY_US(val); }
  void u8g_MicroDelay()              { u8g_xMicroDelay(1); }
  void u8g_10MicroDelay()            { u8g_xMicroDelay(10); }
  void u8g_Delay(uint16_t val)       { delay(val); }
}

// return free heap space
int freeMemory() {
  char stack_end;
  void *heap_start = malloc(sizeof(uint32_t));
  if (heap_start == 0) return 0;

  uint32_t result = (uint32_t)&stack_end - (uint32_t)heap_start;
  free(heap_start);
  return result;
}

extern "C" {
  #include <debug_frmwrk.h>
  int isLPC1769();
  void disk_timerproc();
}

extern uint32_t MSC_SD_Init(uint8_t pdrv);

void SysTick_Callback() { disk_timerproc(); }

TERN_(POSTMORTEM_DEBUGGING, extern void install_min_serial());

void MarlinHAL::init() {

  // Init LEDs
  #if PIN_EXISTS(LED)
    SET_DIR_OUTPUT(LED_PIN);
    WRITE_PIN_CLR(LED_PIN);
    #if PIN_EXISTS(LED2)
      SET_DIR_OUTPUT(LED2_PIN);
      WRITE_PIN_CLR(LED2_PIN);
      #if PIN_EXISTS(LED3)
        SET_DIR_OUTPUT(LED3_PIN);
        WRITE_PIN_CLR(LED3_PIN);
        #if PIN_EXISTS(LED4)
          SET_DIR_OUTPUT(LED4_PIN);
          WRITE_PIN_CLR(LED4_PIN);
        #endif
      #endif
    #endif

    // Flash status LED 3 times to indicate Marlin has started booting
    for (uint8_t i = 0; i < 6; ++i) {
      TOGGLE(LED_PIN);
      delay(100);
    }
  #endif

  // Init Servo Pins
  #define INIT_SERVO(N) OUT_WRITE(SERVO##N##_PIN, LOW)
  #if HAS_SERVO_0
    INIT_SERVO(0);
  #endif
  #if HAS_SERVO_1
    INIT_SERVO(1);
  #endif
  #if HAS_SERVO_2
    INIT_SERVO(2);
  #endif
  #if HAS_SERVO_3
    INIT_SERVO(3);
  #endif
  #if HAS_SERVO_4
    INIT_SERVO(4);
  #endif
  #if HAS_SERVO_5
    INIT_SERVO(5);
  #endif

  //debug_frmwrk_init();
  //_DBG("\n\nDebug running\n");
  // Initialize the SD card chip select pins as soon as possible
  #if PIN_EXISTS(SD_SS)
    OUT_WRITE(SD_SS_PIN, HIGH);
  #endif

  #if PIN_EXISTS(ONBOARD_SD_CS) && ONBOARD_SD_CS_PIN != SD_SS_PIN
    OUT_WRITE(ONBOARD_SD_CS_PIN, HIGH);
  #endif

  #ifdef LPC1768_ENABLE_CLKOUT_12M
   /**
    * CLKOUTCFG register
    * bit 8 (CLKOUT_EN) = enables CLKOUT signal. Disabled for now to prevent glitch when enabling GPIO.
    * bits 7:4 (CLKOUTDIV) = set to 0 for divider setting of /1
    * bits 3:0 (CLKOUTSEL) = set to 1 to select main crystal oscillator as CLKOUT source
    */
    LPC_SC->CLKOUTCFG = (0<<8)|(0<<4)|(1<<0);
    // set P1.27 pin to function 01 (CLKOUT)
    PINSEL_CFG_Type PinCfg;
    PinCfg.Portnum = 1;
    PinCfg.Pinnum = 27;
    PinCfg.Funcnum = 1;    // function 01 (CLKOUT)
    PinCfg.OpenDrain = 0;  // not open drain
    PinCfg.Pinmode = 2;    // no pull-up/pull-down
    PINSEL_ConfigPin(&PinCfg);
    // now set CLKOUT_EN bit
    SBI(LPC_SC->CLKOUTCFG, 8);
  #endif

  USB_Init();                               // USB Initialization
  USB_Connect(false);                       // USB clear connection
  delay(1000);                              // Give OS time to notice
  USB_Connect(true);

  TERN_(HAS_SD_HOST_DRIVE, MSC_SD_Init(0)); // Enable USB SD card access

  const millis_t usb_timeout = millis() + 2000;
  while (!USB_Configuration && PENDING(millis(), usb_timeout)) {
    delay(50);
    idletask();
    #if PIN_EXISTS(LED)
      TOGGLE(LED_PIN);     // Flash quickly during USB initialization
    #endif
  }

  HAL_timer_init();

  TERN_(POSTMORTEM_DEBUGGING, install_min_serial()); // Install the min serial handler
}

#include "../../sd/cardreader.h"

// HAL idle task
void MarlinHAL::idletask() {
  #if HAS_SHARED_MEDIA
    // If Marlin is using the SD card we need to lock it to prevent access from
    // a PC via USB.
    // Other HALs use IS_SD_PRINTING() and IS_SD_FILE_OPEN() to check for access but
    // this will not reliably detect delete operations. To be safe we will lock
    // the disk if Marlin has it mounted. Unfortunately there is currently no way
    // to unmount the disk from the LCD menu.
    // if (IS_SD_PRINTING() || IS_SD_FILE_OPEN())
    if (card.isMounted())
      MSC_Aquire_Lock();
    else
      MSC_Release_Lock();
  #endif
  // Perform USB stack housekeeping
  MSC_RunDeferredCommands();
}

void MarlinHAL::reboot() { NVIC_SystemReset(); }

uint8_t MarlinHAL::get_reset_source() {
  #if ENABLED(USE_WATCHDOG)
    if (watchdog_timed_out()) return RST_WATCHDOG;
  #endif
  return RST_POWER_ON;
}

void MarlinHAL::clear_reset_source() { watchdog_clear_timeout_flag(); }

void flashFirmware(const int16_t) {
  delay(500);          // Give OS time to disconnect
  USB_Connect(false);  // USB clear connection
  delay(1000);         // Give OS time to notice
  hal.reboot();
}

#if ENABLED(USE_WATCHDOG)

  #include <lpc17xx_wdt.h>

  #define WDT_TIMEOUT_US TERN(WATCHDOG_DURATION_8S, 8000000, 4000000) // 4 or 8 second timeout

  void MarlinHAL::watchdog_init() {
    #if ENABLED(WATCHDOG_RESET_MANUAL)
      // We enable the watchdog timer, but only for the interrupt.

      // Configure WDT to only trigger an interrupt
      // Disable WDT interrupt (just in case, to avoid triggering it!)
      NVIC_DisableIRQ(WDT_IRQn);

      // We NEED memory barriers to ensure Interrupts are actually disabled!
      // ( https://dzone.com/articles/nvic-disabling-interrupts-on-arm-cortex-m-and-the )
      __DSB();
      __ISB();

      // Configure WDT to only trigger an interrupt
      // Initialize WDT with the given parameters
      WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_INT_ONLY);

      // Configure and enable WDT interrupt.
      NVIC_ClearPendingIRQ(WDT_IRQn);
      NVIC_SetPriority(WDT_IRQn, 0); // Use highest priority, so we detect all kinds of lockups
      NVIC_EnableIRQ(WDT_IRQn);
    #else
      WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
    #endif
    WDT_Start(WDT_TIMEOUT_US);
  }

  void MarlinHAL::watchdog_refresh() {
    WDT_Feed();
    #if DISABLED(PINS_DEBUGGING) && PIN_EXISTS(LED)
      TOGGLE(LED_PIN);  // heartbeat indicator
    #endif
  }

  // Timeout state
  bool MarlinHAL::watchdog_timed_out() { return TEST(WDT_ReadTimeOutFlag(), 0); }
  void MarlinHAL::watchdog_clear_timeout_flag() { WDT_ClrTimeOutFlag(); }

#endif // USE_WATCHDOG

#include "../../../gcode/parser.h"

// For M42/M43, scan command line for pin code
//   return index into pin map array if found and the pin is valid.
//   return dval if not found or not a valid pin.
int16_t PARSED_PIN_INDEX(const char code, const int16_t dval) {
  const uint16_t val = (uint16_t)parser.intval(code, -1), port = val / 100, pin = val % 100;
  const  int16_t ind = (port < ((NUM_DIGITAL_PINS) >> 5) && pin < 32) ? ((port << 5) | pin) : -2;
  return ind > -1 ? ind : dval;
}

#endif // TARGET_LPC1768
