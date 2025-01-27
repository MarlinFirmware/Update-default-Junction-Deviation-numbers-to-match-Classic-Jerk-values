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
#pragma once

//
// MKS Neptune X
//

#define HAS_OTG_USB_HOST_SUPPORT                  // USB Flash Drive support

// Avoid conflict with TIMER_TONE
#define STEP_TIMER 10

// Use one of these or SDCard-based Emulation will be used
//#define SRAM_EEPROM_EMULATION                   // Use BackSRAM-based EEPROM emulation
//#define FLASH_EEPROM_EMULATION                  // Use Flash-based EEPROM emulation
#if EITHER(NO_EEPROM_SELECTED, I2C_EEPROM)
  #define I2C_EEPROM
  #define MARLIN_EEPROM_SIZE             0x1000U  // 4K
  #define I2C_SCL_PIN                       PB6
  #define I2C_SDA_PIN                       PB7
#endif

//
// Release PB4 (Z_DIR_PIN) from JTAG NRST role
//
//#define DISABLE_DEBUG

//
// Servos
//
#define SERVO0_PIN                          PA8   // Enable BLTOUCH

//
// Limit Switches
//
#define X_DIAG_PIN                          PA15
#define Y_DIAG_PIN                          PC15
#define Z_DIAG_PIN                          PC14
#define E0_DIAG_PIN                         PC4
#define E1_DIAG_PIN                         PE7

#define X_STOP_PIN                    X_DIAG_PIN
#define Y_STOP_PIN                    Y_DIAG_PIN
#define Z_MIN_PIN                     Z_DIAG_PIN
#define Z_MAX_PIN                    E0_DIAG_PIN

//
// Steppers
//
#define X_ENABLE_PIN                        PE4
#define X_STEP_PIN                          PE3
#define X_DIR_PIN                           PE2

#define Y_ENABLE_PIN                        PE1
#define Y_STEP_PIN                          PE0
#define Y_DIR_PIN                           PB9

#define Z_ENABLE_PIN                        PB8
#define Z_STEP_PIN                          PB5
#define Z_DIR_PIN                           PB4

#define E0_ENABLE_PIN                       PB3
#define E0_STEP_PIN                         PD6
#define E0_DIR_PIN                          PD3

#define E1_ENABLE_PIN                       PA3
#define E1_STEP_PIN                         PA6
#define E1_DIR_PIN                          PA1

#if HAS_TMC_UART
  //
  // Software serial
  // No Hardware serial for steppers
  //
  #define X_SERIAL_TX_PIN                   PD5
  #define X_SERIAL_RX_PIN        X_SERIAL_TX_PIN

  #define Y_SERIAL_TX_PIN                   PD7
  #define Y_SERIAL_RX_PIN        Y_SERIAL_TX_PIN

  #define Z_SERIAL_TX_PIN                   PD4
  #define Z_SERIAL_RX_PIN        Z_SERIAL_TX_PIN

  #define E0_SERIAL_TX_PIN                  PD9
  #define E0_SERIAL_RX_PIN      E0_SERIAL_TX_PIN

  #define E1_SERIAL_TX_PIN                  PD8
  #define E1_SERIAL_RX_PIN      E1_SERIAL_TX_PIN

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    19200
#endif

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC1   // TH1
#define TEMP_1_PIN                          PA2   // TH2
#define TEMP_BED_PIN                        PC0   // TB1

#if HOTENDS == 1 && !REDUNDANT_TEMP_MATCH(SOURCE, E1)
  #if TEMP_SENSOR_PROBE
    #define TEMP_PROBE_PIN            TEMP_1_PIN
  #elif TEMP_SENSOR_CHAMBER
    #define TEMP_CHAMBER_PIN          TEMP_1_PIN
  #endif
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC3   // HEATER1
#define HEATER_1_PIN                        PB0   // HEATER2
#define HEATER_BED_PIN                      PA0   // HOT BED

#define FAN_PIN                             PC14  // FAN
#define FAN1_PIN                            PB1   // FAN1

//
// Thermocouples
//
//#define TEMP_0_CS_PIN             HEATER_0_PIN  // TC1 - CS1
//#define TEMP_0_CS_PIN             HEATER_1_PIN  // TC2 - CS2

//
// Misc. Functions
//
#if HAS_TFT_LVGL_UI
  #define MT_DET_1_PIN                      PA4   // MT_DET
  #define MT_DET_2_PIN                      PE6
  #define MT_DET_PIN_STATE                  LOW
#endif

#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                    PA4
#endif
#ifndef FIL_RUNOUT2_PIN
  #define FIL_RUNOUT2_PIN                   PE6
#endif

#ifndef POWER_LOSS_PIN
  #define POWER_LOSS_PIN                    PA2   // PW_DET
#endif

//#define SUICIDE_PIN                       PB2
//#define LED_PIN                           PB2
//#define KILL_PIN                          PA2
//#define KILL_PIN_STATE                    LOW

//
// Power Supply Control
//
#if ENABLED(MKS_PWC)
  #if ENABLED(TFT_LVGL_UI)
    #if ENABLED(PSU_CONTROL)
      #error "PSU_CONTROL is incompatible with MKS_PWC plus TFT_LVGL_UI."
    #endif
    #undef MKS_PWC
    #define SUICIDE_PIN                     PB2
    #define SUICIDE_PIN_STATE               LOW
  #else
    #define PS_ON_PIN                       PB2   // PW_OFF
  #endif
  #define KILL_PIN                          PA13  // PW_DET
  #define KILL_PIN_STATE                    HIGH
#endif

// Random Info
#define USB_SERIAL              -1  // USB Serial
#define WIFI_SERIAL              3  // USART3
#define MKS_WIFI_MODULE_SERIAL   1  // USART1
#define MKS_WIFI_MODULE_SPI      2  // SPI2

#ifndef SDCARD_CONNECTION
  #define SDCARD_CONNECTION              ONBOARD
#endif

// MKS WIFI MODULE
#if ENABLED(MKS_WIFI_MODULE)
  #define WIFI_IO0_PIN                      PC13
  #define WIFI_IO1_PIN                      PC7
  #define WIFI_RESET_PIN                    PE9
#endif

// MKS TEST
#if ENABLED(MKS_TEST)
  #define MKS_TEST_POWER_LOSS_PIN           PA13  // PW_DET
  #define MKS_TEST_PS_ON_PIN                PB2   // PW_OFF
#endif

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers.
   * If the screen stays white, disable 'TFT_RESET_PIN'
   * to let the bootloader init the screen.
   */
  #define TFT_RESET_PIN                     PC6   // FSMC_RST
  #define TFT_BACKLIGHT_PIN                 PD13

  #define DOGLCD_MOSI                       -1    // Prevent auto-define by Conditionals_post.h
  #define DOGLCD_SCK                        -1

  #define TOUCH_CS_PIN                      PA7   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI

  #define LCD_USE_DMA_FSMC                        // Use DMA transfers to send data to the TFT
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define FSMC_DMA_DEV                      DMA2
  #define FSMC_DMA_CHANNEL               DMA_CH5

  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          2

  #define TFT_BUFFER_SIZE                  14400
#endif

//
// Onboard SD card
//
// detect pin doesn't work when ONBOARD and NO_SD_HOST_DRIVE disabled
#if SD_CONNECTION_IS(ONBOARD)
  #define ENABLE_SPI3
  #define SD_SS_PIN                         -1
  #define SDSS                              PC9
  #define SD_SCK_PIN                        PC10
  #define SD_MISO_PIN                       PC11
  #define SD_MOSI_PIN                       PC12
  #define SD_DETECT_PIN                     PD12
#endif

#define SPI_FLASH
#if ENABLED(SPI_FLASH)
  #define HAS_SPI_FLASH                        1
  #define SPI_DEVICE                           2
  #define SPI_FLASH_SIZE               0x1000000
  #define SPI_FLASH_CS_PIN                  PB12
  #define SPI_FLASH_MOSI_PIN                PC3
  #define SPI_FLASH_MISO_PIN                PC2
  #define SPI_FLASH_SCK_PIN                 PB13
#endif

//
// LCD / Controller
//
#define BEEPER_PIN                          PC5

//
// TFT with FSMC interface
//
#if HAS_FSMC_TFT
  /**
   * Note: MKS Robin TFT screens use various TFT controllers.
   * If the screen stays white, disable 'TFT_RESET_PIN'
   * to let the bootloader init the screen.
   */
  #define TFT_RESET_PIN                     PC6   // FSMC_RST
  #define TFT_BACKLIGHT_PIN                 PD13

  #define DOGLCD_MOSI                       -1    // Prevent auto-define by Conditionals_post.h
  #define DOGLCD_SCK                        -1

  #define TOUCH_CS_PIN                      PA7   // SPI2_NSS
  #define TOUCH_SCK_PIN                     PB13  // SPI2_SCK
  #define TOUCH_MISO_PIN                    PB14  // SPI2_MISO
  #define TOUCH_MOSI_PIN                    PB15  // SPI2_MOSI

  #define LCD_USE_DMA_FSMC                        // Use DMA transfers to send data to the TFT
  #define FSMC_CS_PIN                       PD7
  #define FSMC_RS_PIN                       PD11
  #define FSMC_DMA_DEV                      DMA2
  #define FSMC_DMA_CHANNEL               DMA_CH5

  #define TFT_CS_PIN                 FSMC_CS_PIN
  #define TFT_RS_PIN                 FSMC_RS_PIN

  #define TOUCH_BUTTONS_HW_SPI
  #define TOUCH_BUTTONS_HW_SPI_DEVICE          2

  #define TFT_BUFFER_SIZE                  14400
#endif
