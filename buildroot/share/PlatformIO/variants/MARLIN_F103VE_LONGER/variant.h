#pragma once

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

// STM32F103VET6    | DIGITAL     | ANALOG        | USART      | TWI       | SPI                  | SPECIAL    |
//------------------|-------------|---------------|------------|-----------|----------------------|------------|
#define PA0  0   // |             | A0 Nozzle T°c |            |           |                      |            |
#define PA1  1   // |             | A1 Bed T°c    |            |           |                      |            |
#define PA2  2   // |             |               | USART2_TX  |           |                      |            |
#define PA3  3   // |             | DAC_OUT1**    | USART2_RX  |           |                      |            |
#define PA4  4   // |             | DAC_OUT2**    |            |           | SPI1_SS*(wired?)     |            |
#define PA5  5   // | O           |               |            |           | SPI1_SCK  EEPROM     |            |
#define PA6  6   // | I           |               |            |           | SPI1_MISO EEPROM     |            |
#define PA7  7   // | O           |               |            |           | SPI1_MOSI EEPROM     |            |
#define PA8  8   // | Od BED      |               |            |           |                      |            |
#define PA9  9   // |             |               | USART1_TX  |           |                      |            |
#define PA10 10  // |             |               | USART1_RX  |           |                      |            |
#define PA11 11  // | I           |               |            |           |                      | USB_DM     |
#define PA12 12  // | I           |               |            |           |                      | USB_DP     |
#define PA13 13  // | I           |               |            |           |                      | SWD_SWDIO  |
#define PA14 14  // | I           |               |            |           |                      | SWD_SWCLK  |
#define PA15 15  // | Od FAN      |               |            |           |                      |            |
//                  |-------------|---------------|------------|-----------|----------------------|------------|
#define PB0  16  // |             |               |            |           |                      |            |
#define PB1  17  // |             |               |            |           |                      |            |
#define PB2  18  // | I+          |               |            |           |                      | BOOT1      |
#define PB3  19  // | O  X_DIR    |               |            |           |                      |            |
#define PB4  20  // | O  X_STEP   |               |            |           |                      |            |
#define PB5  21  // | O  X_EN     |               |            |           |                      |            |
#define PB6  22  // | O  Y_DIR    |               |            |           |                      |            |
#define PB7  23  // | O  Y_STEP   |               |            |           |                      |            |
#define PB8  24  // | O  Y_EN     |               |            |           |                      |            |
#define PB9  25  // | O  Z_DIR    |               |            |           |                      |            |
#define PB10 26  // | I+          |               | USART3_TX* | TWI2_SCL* |                      |            |
#define PB11 27  // | I+          |               | USART3_RX* | TWI2_SDA* |                      |            |
#define PB12 28  // | O  TFT      |               |            |           | SPI2_SS              | TOUCH_CS   |
#define PB13 29  // | O  TFT      |               |            |           | SPI2_SCK             | TOUCH_SCK  |
#define PB14 30  // | O  TFT      |               |            |           | SPI2_MISO (bad>MOSI) | TOUCH_MOSI |
#define PB15 31  // | I  TFT      |               |            |           | SPI2_MOSI (bad>MISO) | TOUCH_MISO |
//                  |-------------|---------------|------------|-----------|----------------------|------------|
#define PC0  32  // | I  E_OUT    |               |            |           |                      |            |
#define PC1  33  // | I+ X_MIN    |               |            |           |                      |            |
#define PC2  34  // | O  LED      |               |            |           |                      |            |
#define PC3  35  // | I+          |               |            |           |                      |            |
#define PC4  36  // | O  TFT      |               |            |           |                      | TFT RESET  |
#define PC5  37  // | O  CS1      |               |            |           | for SPI1 EEPROM CS   |            |
#define PC6  38  // | I  TFT      |               |            |           |                      | TOUCH_INT  |
#define PC7  39  // |             |               |            |           |                      |            |
#define PC8  40  // | x  SDIO     |               |            |           |                      | SD_D0      |
#define PC9  41  // | x  SDIO     |               |            |           |                      | SD_D1      |
#define PC10 42  // | x  SDIO     |               |            |           |                      | SD_D2      |
#define PC11 43  // | x  SDIO     |               |            |           |                      | SD_D3      |
#define PC12 44  // | O  SDIO     |               |            |           |                      | SD_CLK     |
#define PC13 45  // | I           |               |            |           |                      |            |
#define PC14 46  // | I+ Y_MAX    |               |            |           |                      |            |
#define PC15 47  // | I+ Y_MIN    |               |            |           |                      |            |
//                  |-------------|---------------|------------|-----------|----------------------|------------|
#define PD0  48  // | O  TFT      |               |            |           |                      | OSC_IN  D2 |
#define PD1  49  // | O  TFT      |               |            |           |                      | OSC_OUT D3 |
#define PD2  50  // | O  SDIO     |               |            |           |                      | SD_CMD     |
#define PD3  51  // | Od NOZZLE   |               |            |           |                      |            |
#define PD4  52  // | O  TFT      |               |            |           |                      | FSMC_NOE   |
#define PD5  53  // | O  TFT      |               |            |           |                      | FSMC_NWE   |
#define PD6  54  // | I  wired?*  |               |            |           |                      | FSMC_NWAIT*|
#define PD7  55  // | O  TFT      |               |            |           |                      | FSMC_NE1/CS|
#define PD8  56  // | O  TFT      |               |            |           |                      | FSMC_D13   |
#define PD9  57  // | O  TFT      |               |            |           |                      | FSMC_D14   |
#define PD10 58  // | O  TFT      |               |            |           |                      | FSMC_D15   |
#define PD11 59  // | O  TFT      |               |            |           |                      | FSMC_A16   |
#define PD12 60  // | O  TFT      |               |            |           |                      | TFT BL     |
#define PD13 61  // | Od PWM*     |               |            |           |                      | SERVO0     |
#define PD14 62  // | O  TFT      |               |            |           |                      | FSMC_D00   |
#define PD15 63  // | O  TFT      |               |            |           |                      | FSMC_D01   |
//                  |-------------|---------------|------------|-----------|----------------------|------------|
#define PE0  64  // | O  Z_STEP   |               |            |           |                      |            |
#define PE1  65  // | O  Z_EN     |               |            |           |                      |            |
#define PE2  66  // | O  E0_DIR   |               |            |           |                      |            |
#define PE3  67  // | O  E0_STEP  |               |            |           |                      |            |
#define PE4  68  // | O  E0_EN    |               |            |           |                      |            |
#define PE5  69  // | I+ Z_MAX    |               |            |           |                      |            |
#define PE6  70  // | I+ Z_MIN    |               |            |           |                      |            |
#define PE7  71  // | O  TFT      |               |            |           |                      | FSMC_D04   |
#define PE8  72  // | O  TFT      |               |            |           |                      | FSMC_D05   |
#define PE9  73  // | O  TFT      |               |            |           |                      | FSMC_D06   |
#define PE10 74  // | O  TFT      |               |            |           |                      | FSMC_D07   |
#define PE11 75  // | O  TFT      |               |            |           |                      | FSMC_D08   |
#define PE12 76  // | O  TFT      |               |            |           |                      | FSMC_D09   |
#define PE13 77  // | O  TFT      |               |            |           |                      | FSMC_D10   |
#define PE14 78  // | O  TFT      |               |            |           |                      | FSMC_D11   |
#define PE15 79  // | O  TFT      |               |            |           |                      | FSMC_D12   |
//------------------|-------------|---------------|------------|-----------|----------------------|------------|

// This must be a literal
#define NUM_DIGITAL_PINS        80
#define NUM_ANALOG_INPUTS       16 // 2 first are used, but cant be reduced to 2...

// On-board LED pin number
#ifndef LED_BUILTIN
#define LED_BUILTIN             PC2
#endif

// On-board user button (not wired)
#ifndef USER_BTN
#define USER_BTN                PC13
#endif

// SPI Definition (SPI1 EEPROM)
#define PIN_SPI_SS              PC5
#define PIN_SPI_MOSI            PA7
#define PIN_SPI_MISO            PA6
#define PIN_SPI_SCK             PA5

// I2C Definition (Unused)
#define PIN_WIRE_SDA            PB11
#define PIN_WIRE_SCL            PB10

// Timer Definitions
// Leave TIMER 2 for optional Fan PWM
#define TEMP_TIMER              3
// Leave TIMER 4 for TFT backlight PWM or Servo freq...
#define STEP_TIMER              5
#define TIMER_TONE              TIM6  // TIMER_TONE must be defined in this file
#define TIMER_SERVO             TIM7  // TIMER_SERVO must be defined in this file
#define TIMER_SERIAL            TIM8  // TIMER_SERIAL must be defined in this file

// UART Definitions
// Define here Serial instance number to map on Serial generic name
#define SERIAL_UART_INSTANCE    1

// Default pin used for 'Serial' instance (linked to CH340 USB port)
#define PIN_SERIAL_RX           PA10
#define PIN_SERIAL_TX           PA9
#define PIN_SERIAL1_RX          PA10
#define PIN_SERIAL1_TX          PA9
// Default pin used for 'Serial2' instance (connector exists but unsoldered)
#define PIN_SERIAL2_RX          PA3
#define PIN_SERIAL2_TX          PA2

// Extra HAL modules
#ifdef STM32F103xE
#ifndef HAL_DAC_MODULE_ENABLED
  //#define HAL_DAC_MODULE_ENABLED (unused or maybe for the eeprom write?)
#endif
#ifndef HAL_SD_MODULE_ENABLED
  #define HAL_SD_MODULE_ENABLED
#endif
#ifndef HAL_SRAM_MODULE_ENABLED
  #define HAL_SRAM_MODULE_ENABLED
#endif
#endif

#ifdef __cplusplus
} // extern "C"
#endif

/*----------------------------------------------------------------------------
 *        Arduino objects - C++ only
 *----------------------------------------------------------------------------*/

#ifdef __cplusplus
  // These serial port names are intended to allow libraries and architecture-neutral
  // sketches to automatically default to the correct port name for a particular type
  // of use.  For example, a GPS module would normally connect to SERIAL_PORT_HARDWARE_OPEN,
  // the first hardware serial port whose RX/TX pins are not dedicated to another use.
  //
  // SERIAL_PORT_MONITOR        Port which normally prints to the Arduino Serial Monitor
  //
  // SERIAL_PORT_USBVIRTUAL     Port which is USB virtual serial
  //
  // SERIAL_PORT_LINUXBRIDGE    Port which connects to a Linux system via Bridge library
  //
  // SERIAL_PORT_HARDWARE       Hardware serial port, physical RX & TX pins.
  //
  // SERIAL_PORT_HARDWARE_OPEN  Hardware serial ports which are open for use.  Their RX & TX
  //                            pins are NOT connected to anything by default.
  #define SERIAL_PORT_MONITOR       Serial1
  #define SERIAL_PORT_HARDWARE      Serial1
  #define SERIAL_PORT_HARDWARE_OPEN Serial2
#endif

