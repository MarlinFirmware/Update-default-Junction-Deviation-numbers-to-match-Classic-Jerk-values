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
#pragma once

/**
 * ReprapWorld Minitronics v2.0
 * https://reprap.org/wiki/Minitronics_20
 * 48MHz Atmel SAMD21J18 ARM Cortex-M0+
 */

#if NOT_TARGET(__SAMD21__)
  #error "Oops! Select 'Minitronics v2.0' in 'Tools > Board.'"
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Minitronics V2.0"
#endif

/**
 * NOTE: We need the Serial port on the -1 to make it work!!. Remember to change it on configuration.h #define SERIAL_PORT -1
 */

/**
 * EEPROM EMULATION: Works with some bugs already, but the board needs an I2C EEPROM memory soldered on.
 */
//#define FLASH_EEPROM_EMULATION
//#define I2C_EEPROM                              // EEPROM on I2C-0
#define MARLIN_EEPROM_SIZE                  500U  // 4000 bytes

//This its another option to emulate an EEPROM, but its more efficient to dont loose the data the first One.
//#define SDCARD_EEPROM_EMULATION

//
// BLTouch
//
#define SERVO0_PIN                            33  // BLTouch PWM

//
//  Limit Switches
//
#define X_STOP_PIN                            54
#define Y_STOP_PIN                            55
#define Z_STOP_PIN                             4

/**
 * NOTE: For extra TMC2209 used as independent axes a new digital PIN is needed.
 * We can configure on the board the extra pin of this stepper,
 * e.g., as a MIN_PIN/MAX_PIN. This pin is D14.
 */
//#define Z2_STOP_PIN                         14
//#define X2_STOP_PIN                         14
//#define Y2_STOP_PIN                         14

//
// Z Probe (when not Z_MIN_PIN)
//
#ifndef Z_MIN_PROBE_PIN
  #define Z_MIN_PROBE_PIN                     12
#endif

//
// Steppers
//
#define X_STEP_PIN                             1
#define X_DIR_PIN                              3
#define X_ENABLE_PIN                           0

#define Y_STEP_PIN                            29
#define Y_DIR_PIN                             28
#define Y_ENABLE_PIN                           0

#define Z_STEP_PIN                            16
#define Z_DIR_PIN                             17
#define Z_ENABLE_PIN                           0

#define E0_STEP_PIN                           14
#define E0_DIR_PIN                            15
#define E0_ENABLE_PIN                          0

#define E1_STEP_PIN                           20
#define E1_DIR_PIN                            13
#define E1_ENABLE_PIN                         21

// Filament runout. You may choose to use this pin for some other purpose. It's a normal GPIO that can be configured as I/O.
// For example, a switch to detect any kind of behavior, Power supply pin .... etc.
#ifndef FIL_RUNOUT_PIN
  #define FIL_RUNOUT_PIN                      44
#endif

// This board have the option to use an extra TMC2209 stepper, one of the use could be as a second extruder.
#if EXTRUDERS < 2
  // TODO: Correct here that when we have two extruders (or whatever), use the extra endstops. i.e., The max, there is no Z2_endstop.
  #if NUM_Z_STEPPERS > 1
    #define Z2_STOP_PIN                       14
  #endif
#else
  // If we want to configure the extra stepper as a Extruder, we should have undef all of the extra motors.
  #undef X2_DRIVER_TYPE
  #undef Y2_DRIVER_TYPE
  #undef Z2_DRIVER_TYPE
  #undef Z3_DRIVER_TYPE
  #undef Z4_DRIVER_TYPE

  // For more than one extruder define the new extruder and its pins.
  // Remember to #define TEMP_SENSOR_1, since this contains the E1 sensor type.

  #define FIL_RUNOUT2_PIN                     14

#endif

// Verify that drivers match the hardware
#if (HAS_X_AXIS && !AXIS_DRIVER_TYPE_X(DRV8825)) || (HAS_Y_AXIS && !AXIS_DRIVER_TYPE_Y(DRV8825)) || (HAS_Z_AXIS && !AXIS_DRIVER_TYPE_Z(DRV8825)) || (HAS_EXTRUDER && !AXIS_DRIVER_TYPE_E0(DRV8825))
  #error "Minitronics v2.0 has hard-wired DRV8825 drivers. Comment out this line to continue."
#endif

//
// Extruder / Bed
//

// Temperature Sensors
#define TEMP_0_PIN                             4  // T1

// You could use one of the ADC for a temp chamber if you don't use the second extruder, for example.
#if TEMP_SENSOR_CHAMBER > 0
  #define TEMP_CHAMBER_PIN                     3
#else
  #define TEMP_1_PIN                           2  // T3
#endif

#define TEMP_BED_PIN                           3  // T2

//
// Heaters / Fans
//
#define HEATER_0_PIN                          10
#define HEATER_1_PIN                          11
#define HEATER_BED_PIN                         6
#define SPINDLE_LASER_PWM_PIN                  6

// The board has 4 PWM fans, use and configure as desired
#define FAN0_PIN                              24

//
// LCD / Controller
//

#if ENABLED(CR10_STOCKDISPLAY)
  #define EXP3_01_PIN                EXP1_01_PIN
  #define EXP3_02_PIN                EXP1_02_PIN
  #define EXP3_03_PIN                EXP1_03_PIN
  #define EXP3_04_PIN                EXP1_04_PIN
  #define EXP3_05_PIN                EXP1_05_PIN
  #define EXP3_06_PIN                EXP1_06_PIN
  #define EXP3_07_PIN                EXP1_07_PIN
  #define EXP3_08_PIN                EXP1_08_PIN
#endif

/**
 * This section is based on pins_RAMPS_144.h. To use a new display, you
 * could use normal duponts and connect with the scheme shown before.
 * Tested:
 *   - Ender-3 Old display (Character LCD)
 *   - Ender-3 New Serial DWING Display
 *   - Reprap Display
 *   - Ender-5 New Serial Display
 *   - Any Reprap character display like
 */

#if HAS_WIRED_LCD

  //
  // LCD Display output pins
  //

  #if ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

    #define LCD_PINS_RS                       18  // CS chip select /SS chip slave select
    #define LCD_PINS_EN                     MOSI  // SID (MOSI)
    #define LCD_PINS_D4                      SCK  // SCK (CLK) clock

    #define BTN_ENC                           23
    #define BTN_EN1                           27
    #define BTN_EN2                           33

  #elif ALL(IS_NEWPANEL, PANEL_ONE)

    // TO TEST
    //#define LCD_PINS_RS            EXP1_02_PIN
    //#define LCD_PINS_EN            EXP2_05_PIN
    //#define LCD_PINS_D4                     57  // Mega/Due:65 - AGCM4:57
    //#define LCD_PINS_D5                     58  // Mega/Due:66 - AGCM4:58
    //#define LCD_PINS_D6            EXP2_07_PIN
    //#define LCD_PINS_D7                     56  // Mega/Due:64 - AGCM4:56

  #else

    #if ENABLED(CR10_STOCKDISPLAY)

      // TO TEST
      //#define LCD_PINS_RS          EXP3_04_PIN
      //#define LCD_PINS_EN          EXP3_03_PIN
      //#define LCD_PINS_D4          EXP3_05_PIN

      #if !IS_NEWPANEL
        // TO TEST
        //#define BEEPER_PIN         EXP3_05_PIN
      #endif

    #elif ENABLED(ZONESTAR_LCD)

      // TO TEST
      //#define LCD_PINS_RS                   56  // Mega/Due:64 - AGCM4:56
      //#define LCD_PINS_EN          EXP2_07_PIN
      //#define LCD_PINS_D4                   55  // Mega/Due:63 - AGCM4:55
      //#define LCD_PINS_D5          EXP1_02_PIN
      //#define LCD_PINS_D6          EXP2_05_PIN
      //#define LCD_PINS_D7                   57  // Mega/Due:65 - AGCM4:57

    #else

      #if ANY(MKS_12864OLED, MKS_12864OLED_SSD1306)
        // TO TEST
        //#define LCD_PINS_DC                 25  // Set as output on init
        //#define LCD_PINS_RS                 27  // Pull low for 1s to init
        // DOGM SPI LCD Support
        //#define DOGLCD_CS                   16
        //#define DOGLCD_MOSI                 17
        //#define DOGLCD_SCK                  23
        //#define DOGLCD_A0          LCD_PINS_DC

      #else
        // Definitions for any standard Display
        #define LCD_PINS_RS          EXP1_04_PIN
        #define LCD_PINS_EN          EXP1_03_PIN
        #define LCD_PINS_D4          EXP1_05_PIN
        #define LCD_PINS_D5          EXP1_06_PIN
        #define LCD_PINS_D6          EXP1_07_PIN
      #endif

      #define LCD_PINS_D7            EXP1_08_PIN

      #if !IS_NEWPANEL
        #define BEEPER_PIN           EXP1_01_PIN
      #endif

    #endif

    #if !IS_NEWPANEL
      // Buttons attached to a shift register
      // Not wired yet
      //#define SHIFT_CLK_PIN        EXP1_07_PIN
      //#define SHIFT_LD_PIN         EXP2_05_PIN
      //#define SHIFT_OUT_PIN        EXP1_02_PIN
      //#define SHIFT_EN_PIN                  17
    #endif

  #endif

  //
  // LCD Display input pins
  //
  #if IS_NEWPANEL

    #if IS_RRD_SC

      //#define BEEPER_PIN           EXP1_01_PIN

      #if ENABLED(CR10_STOCKDISPLAY)
        // TO TEST
        #define BTN_EN1              EXP1_03_PIN
        #define BTN_EN2              EXP1_05_PIN
      #else
        // Definitions fpr any standard Display
        #define BTN_EN1              EXP2_05_PIN
        #define BTN_EN2              EXP2_03_PIN
        #if ENABLED(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER)
          #define BTN_ENC_EN         LCD_PINS_D7  // Detect the presence of the encoder
        #endif
      #endif

      #define BTN_ENC                EXP1_02_PIN
      #ifndef SD_DETECT_PIN
        #define SD_DETECT_PIN        EXP2_07_PIN
      #endif
      //#define KILL_PIN             EXP2_10_PIN

      #if ENABLED(BQ_LCD_SMART_CONTROLLER)
        //#define LCD_BACKLIGHT_PIN  EXP1_08_PIN  // TO TEST
      #endif

    #elif ENABLED(LCD_I2C_PANELOLU2)

      // TO TEST
      //#define BTN_EN1                       47
      //#define BTN_EN2              EXP2_03_PIN
      //#define BTN_ENC                       32
      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define KILL_PIN             EXP1_01_PIN
      //#undef LCD_PINS_EN                        // not used, causes false pin conflict report

    #elif ENABLED(LCD_I2C_VIKI)

      // TO TEST
      //#define BTN_EN1              EXP1_02_PIN  // https://files.panucatt.com/datasheets/viki_wiring_diagram.pdf explains 40/42.
      //#define BTN_EN2              EXP2_05_PIN
      //#define BTN_ENC                       -1

      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN

    #elif ANY(VIKI2, miniVIKI)

      // TO TEST
      //#define DOGLCD_CS                     45
      //#define DOGLCD_A0            EXP2_07_PIN
      //#define LCD_SCREEN_ROT_180

      //#define BEEPER_PIN                    33
      //#define STAT_LED_RED_PIN              32
      //#define STAT_LED_BLUE_PIN    EXP1_03_PIN

      //#define BTN_EN1                       22
      //#define BTN_EN2                        7
      //#define BTN_ENC              EXP1_08_PIN

      //#define SD_DETECT_PIN                 -1  // Pin 49 for display SD interface, 72 for easy adapter board
      //#define KILL_PIN                      31

    #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)

      // TO TEST
      //#define DOGLCD_CS                     29
      //#define DOGLCD_A0                     27

      //#define BEEPER_PIN                    23
      //#define LCD_BACKLIGHT_PIN             33

      //#define BTN_EN1              EXP1_03_PIN
      //#define BTN_EN2              EXP1_06_PIN
      //#define BTN_ENC                       31

      //#define LCD_SDSS_PIN           SD_SS_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN
      //#define KILL_PIN             EXP1_01_PIN

    #elif ANY(MKS_MINI_12864, FYSETC_MINI_12864)

      // TO TEST
      //#define BEEPER_PIN           EXP1_06_PIN
      //#define BTN_ENC              EXP1_03_PIN
      //#define SD_DETECT_PIN        EXP2_10_PIN

      //#ifndef KILL_PIN
      //  #define KILL_PIN           EXP1_01_PIN
      //#endif

      #if ENABLED(MKS_MINI_12864)

        // TO TEST
        //#define DOGLCD_A0                   27
        //#define DOGLCD_CS                   25

        // GLCD features
        // Uncomment screen orientation
        //#define LCD_SCREEN_ROT_90
        //#define LCD_SCREEN_ROT_180
        //#define LCD_SCREEN_ROT_270

        // not connected to a pin
        //#define LCD_BACKLIGHT_PIN           57  // backlight LED on A11/D? (Mega/Due:65 - AGCM4:57)

        //#define BTN_EN1                     31
        //#define BTN_EN2                     33

      #elif ENABLED(FYSETC_MINI_12864)

        // From https://wiki.fysetc.com/Mini12864_Panel/

        // TO TEST
        //#define DOGLCD_A0                   16
        //#define DOGLCD_CS                   17

        //#define BTN_EN1                     33
        //#define BTN_EN2                     31

        //#define FORCE_SOFT_SPI                  // Use this if default of hardware SPI causes display problems
                                                  //   results in LCD soft SPI mode 3, SD soft SPI mode 0

        //#define LCD_RESET_PIN               23  // Must be high or open for LCD to operate normally.

        #if ANY(FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0)
          #ifndef RGB_LED_R_PIN
            // TO TEST
            //#define RGB_LED_R_PIN           25
          #endif
          #ifndef RGB_LED_G_PIN
            // TO TEST
            //#define RGB_LED_G_PIN           27
          #endif
          #ifndef RGB_LED_B_PIN
            // TO TEST
            //#define RGB_LED_B_PIN           29
          #endif
        #elif ENABLED(FYSETC_MINI_12864_2_1)
          // TO TEST
          //#define NEOPIXEL_PIN              25
        #endif

      #endif

    #elif ENABLED(MINIPANEL)

      // TO TEST
      //#define BEEPER_PIN           EXP2_05_PIN
      // not connected to a pin
      //#define LCD_BACKLIGHT_PIN             57  // Backlight LED on A11/D? (Mega/Due:65 - AGCM4:57)

      //#define DOGLCD_A0            EXP2_07_PIN
      //#define DOGLCD_CS                     58  // Mega/Due:66 - AGCM4:58

      // GLCD features
      // Uncomment screen orientation
      //#define LCD_SCREEN_ROT_90
      //#define LCD_SCREEN_ROT_180
      //#define LCD_SCREEN_ROT_270

      //#define BTN_EN1              EXP1_02_PIN
      //#define BTN_EN2                       55  // Mega/Due:63 - AGCM4:55
      //#define BTN_ENC                       72  // Mega/Due:59 - AGCM4:72

      //#define SD_DETECT_PIN        EXP2_10_PIN
      //#define KILL_PIN                      56  // Mega/Due:64 - AGCM4:56

    #elif ENABLED(ZONESTAR_LCD)

      // TO TEST
      //#define ADC_KEYPAD_PIN                12

    #elif ENABLED(AZSMZ_12864)

      // TO TEST

    #else

      // Beeper on AUX-4
      //#define BEEPER_PIN                    33

      // Buttons are directly attached to AUX-2
      #if IS_RRW_KEYPAD
        // TO TEST
        //#define SHIFT_OUT_PIN      EXP1_02_PIN
        //#define SHIFT_CLK_PIN      EXP2_07_PIN
        //#define SHIFT_LD_PIN       EXP2_05_PIN
        //#define BTN_EN1                     56  // Mega/Due:64 - AGCM4:56
        //#define BTN_EN2                     72  // Mega/Due:59 - AGCM4:72
        //#define BTN_ENC                     55  // Mega/Due:63 - AGCM4:55
      #elif ENABLED(PANEL_ONE)
        // TO TEST
        //#define BTN_EN1                     72  // AUX2 PIN 3 (Mega/Due:59 - AGCM4:72)
        //#define BTN_EN2                     55  // AUX2 PIN 4 (Mega/Due:63 - AGCM4:55)
        //#define BTN_ENC            EXP2_10_PIN  // AUX3 PIN 7
      #else
        // TO TEST
        //#define BTN_EN1            EXP1_06_PIN
        //#define BTN_EN2            EXP1_03_PIN
        //#define BTN_ENC                     31
      #endif

      #if ENABLED(G3D_PANEL)
        // TO TEST
        //#define SD_DETECT_PIN      EXP2_10_PIN
        //#define KILL_PIN           EXP1_01_PIN
      #endif

    #endif
  #endif // IS_NEWPANEL

#endif // HAS_WIRED_LCD

//
// SD Support
//

#define SD_SS_PIN                              2
#undef SD_DETECT_PIN
#define SD_DETECT_PIN                         22

#if HAS_TMC_UART
  /**
   * TMC2208/TMC2209 stepper drivers
   * Seems to work fine with Software Serial. If you want to test, use SAMD51 Serial1 and Serial2. Be careful with the Sercom configurations.
   * Steppers 1,2,3,4 (X,Y,Z,E0) are on Serial1, Sercom (RX=0, TX=1), extra stepper 5 (E1 or any axis you want) is on Serial2, Sercom (RX=17, TX=16)
   */

  //#define X_HARDWARE_SERIAL  Serial1
  //#define Y_HARDWARE_SERIAL  Serial1
  //#define Z_HARDWARE_SERIAL  Serial1
  //#define E0_HARDWARE_SERIAL Serial1
  //#define E1_HARDWARE_SERIAL Serial2

  // Default TMC slave addresses
  #ifndef X_SLAVE_ADDRESS
    #define X_SLAVE_ADDRESS                  0
  #endif
  #ifndef Y_SLAVE_ADDRESS
    #define Y_SLAVE_ADDRESS                  1
  #endif
  #ifndef Z_SLAVE_ADDRESS
    #define Z_SLAVE_ADDRESS                  2
  #endif
  #ifndef E0_SLAVE_ADDRESS
    #define E0_SLAVE_ADDRESS                 3
  #endif
  #ifndef E1_SLAVE_ADDRESS
    #define E1_SLAVE_ADDRESS                 0
  #endif
  static_assert(X_SLAVE_ADDRESS == 0, "X_SLAVE_ADDRESS must be 0 for BOARD_MINITRONICS20.");
  static_assert(Y_SLAVE_ADDRESS == 1, "Y_SLAVE_ADDRESS must be 1 for BOARD_MINITRONICS20.");
  static_assert(Z_SLAVE_ADDRESS == 2, "Z_SLAVE_ADDRESS must be 2 for BOARD_MINITRONICS20.");
  static_assert(E0_SLAVE_ADDRESS == 3, "E0_SLAVE_ADDRESS must be 3 for BOARD_MINITRONICS20.");
  static_assert(E1_SLAVE_ADDRESS == 0, "E1_SLAVE_ADDRESS must be 0 for BOARD_MINITRONICS20.");

  // Reduce baud rate to improve software serial reliability
  #ifndef TMC_BAUD_RATE
    #define TMC_BAUD_RATE                  19200 // 250000
  #endif

  //
  // Software serial
  //
  #define X_SERIAL_TX_PIN                      0
  #define X_SERIAL_RX_PIN                      1

  #define Y_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Y_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define Z_SERIAL_TX_PIN        X_SERIAL_TX_PIN
  #define Z_SERIAL_RX_PIN        X_SERIAL_RX_PIN

  #define E0_SERIAL_TX_PIN       X_SERIAL_TX_PIN
  #define E0_SERIAL_RX_PIN       X_SERIAL_RX_PIN

  #define E1_SERIAL_TX_PIN                    17
  #define E1_SERIAL_RX_PIN                    16

#endif
