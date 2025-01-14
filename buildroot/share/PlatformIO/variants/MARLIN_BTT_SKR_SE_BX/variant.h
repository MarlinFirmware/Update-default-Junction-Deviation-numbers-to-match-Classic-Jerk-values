#ifndef _VARIANT_ARDUINO_STM32_
#define _VARIANT_ARDUINO_STM32_
#ifdef __cplusplus

extern "C" {
#endif // __cplusplus
/*----------------------------------------------------------------------------
 *        Pins
 *----------------------------------------------------------------------------*/
#define PE2  0
#define PE3  1
#define PE4  2
#define PE5  3
#define PE6  4
#define PI8  5
#define PC13 6
#define PC14 7
#define PC15 8
#define PI9  9
#define PI10 10
#define PI11 11
#define PF0  12
#define PF1  13
#define PF2  14
#define PH0  15
#define PH1  16
#define PB2  17
#define PF15 18
#define PG0  19
#define PG1  20
#define PE7  21
#define PE8  22
#define PE9  23
#define PE10 24
#define PE11 25
#define PE12 26
#define PE13 27
#define PE14 28
#define PE15 29
#define PB10 30
#define PB11 31
#define PH6  32
#define PH7  33
#define PH8  34
#define PH9  35
#define PH10 36
#define PH11 37
#define PH12 38
#define PB12 39
#define PB13 40
#define PB14 41
#define PB15 42
#define PD8  43
#define PD9  44
#define PD10 45
#define PD11 46
#define PD12 47
#define PD13 48
#define PD14 49
#define PD15 50
#define PG2  51
#define PG3  52
#define PG4  53
#define PG5  54
#define PG6  55
#define PG7  56
#define PG8  57
#define PC6  58
#define PC7  59
#define PC8  60
#define PC9  61
#define PA8  62
#define PA9  63
#define PA10 64
#define PA11 65
#define PA12 66
#define PA13 67
#define PH13 68
#define PH14 69
#define PH15 70
#define PI0  71
#define PI1  72
#define PI2  73
#define PI3  74
#define PA14 75
#define PA15 76
#define PC10 77
#define PC11 78
#define PC12 79
#define PD0  80
#define PD1  81
#define PD2  82
#define PD3  83
#define PD4  84
#define PD5  85
#define PD6  86
#define PD7  87
#define PG9  88
#define PG10 89
#define PG11 90
#define PG12 91
#define PG13 92
#define PG14 93
#define PG15 94
#define PB3  95
#define PB4  96
#define PB5  97
#define PB6  98
#define PB7  99
#define PB8  100
#define PB9  101
#define PE0  102
#define PE1  103
#define PI4  104
#define PI5  105
#define PI6  106
#define PI7  107
#define PA0  108
#define PA1  109
#define PA2  110
#define PA3  111
#define PA4  112
#define PA5  113
#define PA6  114
#define PA7  115
#define PB0  116
#define PB1  117
#define PH2  118
#define PH3  119
#define PH4  120
#define PH5  121
#define PC0  122
#define PC1  123
#define PC2  124
#define PC3  125
#define PC4  126
#define PC5  127
#define PF3  128
#define PF4  129
#define PF5  130
#define PF6  131
#define PF7  132
#define PF8  133
#define PF9  134
#define PF10 135
#define PF11 136
#define PF12 137
#define PF13 138
#define PF14 139

// This must be a literal with the same value as PEND
#define NUM_DIGITAL_PINS        140

// This must be a literal with a value less than or equal to to MAX_ANALOG_INPUTS
#define NUM_ANALOG_INPUTS       24
#define NUM_ANALOG_FIRST        108

// Timer Definitions
// Do not use timer used by PWM pins when possible. See PinMap_PWM in PeripheralPins.c
#define TIMER_TONE              TIM2  // TIMER_TONE must be defined in this file
#define TIMER_SERVO             TIM5  // TIMER_SERVO must be defined in this file
#define TIMER_SERIAL            TIM7  // TIMER_SERIAL must be defined in this file

// UART1 for TFT port
#define ENABLE_HWSERIAL1
#define PIN_SERIAL1_RX          PA10
#define PIN_SERIAL1_TX          PA9

// UART4 for ESP-01 port
#define ENABLE_HWSERIAL4
#define PIN_SERIAL4_RX          PA1
#define PIN_SERIAL4_TX          PA0

// IIC1 for onboard 24C32 EEPROM
#define PIN_WIRE_SDA            PB9
#define PIN_WIRE_SCL            PB8

// SPI3 for onboard SD card
// #define PIN_SPI_MOSI            PC12
// #define PIN_SPI_MISO            PC11
// #define PIN_SPI_SCK             PC10

// HSE default value is 25MHz in HAL
// HSE_BYPASS is 25MHz
#ifndef HSE_BYPASS_NOT_USED
  #define HSE_VALUE 25000000
#endif

// #define USE_USB_FS
/* Extra HAL modules */
#ifndef HAL_HCD_MODULE_ENABLED
  //#define HAL_HCD_MODULE_ENABLED
#endif
#ifndef HAL_DAC_MODULE_ENABLED
  //#define HAL_DAC_MODULE_ENABLED
#endif
#ifndef HAL_ETH_MODULE_ENABLED
  //#define HAL_ETH_MODULE_ENABLED
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
#define SERIAL_PORT_MONITOR     Serial
#define SERIAL_PORT_HARDWARE    Serial
#endif

#endif /* _VARIANT_ARDUINO_STM32_ */
