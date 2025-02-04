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
 * Conditionals_LCD.h
 * Conditionals that need to be set before Configuration_adv.h or pins.h
 */

/**
 * Extruders have some combination of stepper motors and hotends
 * so we separate these concepts into the defines:
 *
 *  EXTRUDERS    - Number of Selectable Tools
 *  HOTENDS      - Number of hotends, whether connected or separate
 *  E_STEPPERS   - Number of actual E stepper motors
 *  E_MANUAL     - Number of E steppers for LCD move options
 *
 * These defines must be simple constants for use in REPEAT, etc.
 */
#if EXTRUDERS
  #define HAS_EXTRUDERS 1
  #if EXTRUDERS > 1
    #define HAS_MULTI_EXTRUDER 1
  #endif
  #define E_AXIS_N(E) AxisEnum(E_AXIS + E_INDEX_N(E))
#else
  #undef EXTRUDERS
  #define EXTRUDERS 0
  #undef TEMP_SENSOR_0
  #undef TEMP_SENSOR_1
  #undef TEMP_SENSOR_2
  #undef TEMP_SENSOR_3
  #undef TEMP_SENSOR_4
  #undef TEMP_SENSOR_5
  #undef TEMP_SENSOR_6
  #undef TEMP_SENSOR_7
  #undef SINGLENOZZLE
  #undef SWITCHING_EXTRUDER
  #undef MECHANICAL_SWITCHING_EXTRUDER
  #undef SWITCHING_NOZZLE
  #undef MECHANICAL_SWITCHING_NOZZLE
  #undef MIXING_EXTRUDER
  #undef HOTEND_IDLE_TIMEOUT
  #undef DISABLE_E
  #undef PREVENT_LENGTHY_EXTRUDE
  #undef FILAMENT_RUNOUT_SENSOR
  #undef FILAMENT_RUNOUT_DISTANCE_MM
  #undef DISABLE_OTHER_EXTRUDERS
#endif

#define E_OPTARG(N) OPTARG(HAS_MULTI_EXTRUDER, N)
#define E_TERN_(N)  TERN_(HAS_MULTI_EXTRUDER, N)
#define E_TERN0(N)  TERN0(HAS_MULTI_EXTRUDER, N)

#if ANY(SWITCHING_EXTRUDER, MECHANICAL_SWITCHING_EXTRUDER)
  #define HAS_SWITCHING_EXTRUDER 1
#endif
#if ANY(SWITCHING_NOZZLE, MECHANICAL_SWITCHING_NOZZLE)
  #define HAS_SWITCHING_NOZZLE 1
#endif

/**
 *  Multi-Material-Unit supported models
 */
#ifdef MMU_MODEL
  #define HAS_MMU 1
  #define SINGLENOZZLE

  #define _PRUSA_MMU1             1
  #define _PRUSA_MMU2             2
  #define _PRUSA_MMU2S            3
  #define _EXTENDABLE_EMU_MMU2   12
  #define _EXTENDABLE_EMU_MMU2S  13
  #define _MMU CAT(_,MMU_MODEL)

  #if _MMU == _PRUSA_MMU1
    #define HAS_PRUSA_MMU1 1
  #elif _MMU % 10 == _PRUSA_MMU2
    #define HAS_PRUSA_MMU2 1
  #elif _MMU % 10 == _PRUSA_MMU2S
    #define HAS_PRUSA_MMU2 1
    #define HAS_PRUSA_MMU2S 1
  #endif
  #if _MMU == _EXTENDABLE_EMU_MMU2 || _MMU == _EXTENDABLE_EMU_MMU2S
    #define HAS_EXTENDABLE_MMU 1
  #endif

  #undef _MMU
  #undef _PRUSA_MMU1
  #undef _PRUSA_MMU2
  #undef _PRUSA_MMU2S
  #undef _EXTENDABLE_EMU_MMU2
  #undef _EXTENDABLE_EMU_MMU2S
#endif

#if ENABLED(E_DUAL_STEPPER_DRIVERS) // E0/E1 steppers act in tandem as E0

  #define E_STEPPERS      2
  #define E_MANUAL        1

#elif HAS_SWITCHING_EXTRUDER        // One stepper for every two EXTRUDERS

  #if EXTRUDERS > 4
    #define E_STEPPERS    3
  #elif EXTRUDERS > 2
    #define E_STEPPERS    2
  #else
    #define E_STEPPERS    1
  #endif

#elif ENABLED(MIXING_EXTRUDER)      // Multiple feeds are mixed proportionally

  #define E_STEPPERS      MIXING_STEPPERS
  #define E_MANUAL        1
  #if MIXING_STEPPERS == 2
    #define HAS_DUAL_MIXING 1
  #endif
  #ifndef MIXING_VIRTUAL_TOOLS
    #define MIXING_VIRTUAL_TOOLS 1
  #endif

#elif ENABLED(SWITCHING_TOOLHEAD)   // Toolchanger

  #define E_STEPPERS      EXTRUDERS
  #define E_MANUAL        EXTRUDERS

#elif HAS_PRUSA_MMU2                // Průša Multi-Material Unit v2

  #define E_STEPPERS      1
  #define E_MANUAL        1

#endif

// Default E steppers / manual motion is one per extruder
#ifndef E_STEPPERS
  #define E_STEPPERS EXTRUDERS
#endif
#ifndef E_MANUAL
  #define E_MANUAL EXTRUDERS
#endif

// Number of hotends...
#if ANY(SINGLENOZZLE, MIXING_EXTRUDER)                // Only one for singlenozzle or mixing extruder
  #define HOTENDS 1
#elif HAS_SWITCHING_EXTRUDER && !HAS_SWITCHING_NOZZLE // One for each pair of abstract "extruders"
  #define HOTENDS E_STEPPERS
#elif TEMP_SENSOR_0
  #define HOTENDS EXTRUDERS                           // One per extruder if at least one heater exists
#else
  #define HOTENDS 0                                   // A machine with no hotends at all can still extrude
#endif

// At least one hotend...
#if HOTENDS
  #define HAS_HOTEND 1
  #ifndef HOTEND_OVERSHOOT
    #define HOTEND_OVERSHOOT 15
  #endif
#endif

// More than one hotend...
#if HOTENDS > 1
  #define HAS_MULTI_HOTEND 1
  #define HAS_HOTEND_OFFSET 1
  #ifndef HOTEND_OFFSET_X
    #define HOTEND_OFFSET_X { 0 } // X offsets for each extruder
  #endif
  #ifndef HOTEND_OFFSET_Y
    #define HOTEND_OFFSET_Y { 0 } // Y offsets for each extruder
  #endif
  #ifndef HOTEND_OFFSET_Z
    #define HOTEND_OFFSET_Z { 0 } // Z offsets for each extruder
  #endif
#else
  #undef HOTEND_OFFSET_X
  #undef HOTEND_OFFSET_Y
  #undef HOTEND_OFFSET_Z
#endif

// Clean up E-stepper-based settings...
#if E_STEPPERS <= 7
  #undef INVERT_E7_DIR
  #undef E7_DRIVER_TYPE
  #if E_STEPPERS <= 6
    #undef INVERT_E6_DIR
    #undef E6_DRIVER_TYPE
    #if E_STEPPERS <= 5
      #undef INVERT_E5_DIR
      #undef E5_DRIVER_TYPE
      #if E_STEPPERS <= 4
        #undef INVERT_E4_DIR
        #undef E4_DRIVER_TYPE
        #if E_STEPPERS <= 3
          #undef INVERT_E3_DIR
          #undef E3_DRIVER_TYPE
          #if E_STEPPERS <= 2
            #undef INVERT_E2_DIR
            #undef E2_DRIVER_TYPE
            #if E_STEPPERS <= 1
              #undef INVERT_E1_DIR
              #undef E1_DRIVER_TYPE
              #if E_STEPPERS == 0
                #undef INVERT_E0_DIR
                #undef E0_DRIVER_TYPE
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

/**
 * Number of Linear Axes (e.g., XYZIJKUVW)
 * All the logical axes except for the tool (E) axis
 */
#ifdef NUM_AXES
  #undef NUM_AXES
  #define NUM_AXES_WARNING 1
#endif

#ifdef W_DRIVER_TYPE
  #define NUM_AXES 9
#elif defined(V_DRIVER_TYPE)
  #define NUM_AXES 8
#elif defined(U_DRIVER_TYPE)
  #define NUM_AXES 7
#elif defined(K_DRIVER_TYPE)
  #define NUM_AXES 6
#elif defined(J_DRIVER_TYPE)
  #define NUM_AXES 5
#elif defined(I_DRIVER_TYPE)
  #define NUM_AXES 4
#elif defined(Z_DRIVER_TYPE)
  #define NUM_AXES 3
#elif defined(Y_DRIVER_TYPE)
  #define NUM_AXES 2
#elif defined(X_DRIVER_TYPE)
  #define NUM_AXES 1
#else
  #define NUM_AXES 0
#endif
#if NUM_AXES >= 1
  #define HAS_X_AXIS 1
  #define HAS_A_AXIS 1
  #if NUM_AXES >= XY
    #define HAS_Y_AXIS 1
    #define HAS_B_AXIS 1
    #if NUM_AXES >= XYZ
      #define HAS_Z_AXIS 1
      #define HAS_C_AXIS 1
      #if NUM_AXES >= 4
        #define HAS_I_AXIS 1
        #if NUM_AXES >= 5
          #define HAS_J_AXIS 1
          #if NUM_AXES >= 6
            #define HAS_K_AXIS 1
            #if NUM_AXES >= 7
              #define HAS_U_AXIS 1
              #if NUM_AXES >= 8
                #define HAS_V_AXIS 1
                #if NUM_AXES >= 9
                  #define HAS_W_AXIS 1
                #endif
              #endif
            #endif
          #endif
        #endif
      #endif
    #endif
  #endif
#endif

#if !HAS_X_AXIS
  #undef AVOID_OBSTACLES
  #undef ENDSTOPPULLUP_XMIN
  #undef ENDSTOPPULLUP_XMAX
  #undef X_MIN_ENDSTOP_HIT_STATE
  #undef X_MAX_ENDSTOP_HIT_STATE
  #undef X2_DRIVER_TYPE
  #undef X_ENABLE_ON
  #undef DISABLE_X
  #undef INVERT_X_DIR
  #undef X_HOME_DIR
  #undef X_MIN_POS
  #undef X_MAX_POS
  #undef MANUAL_X_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOPS
  #undef MAX_SOFTWARE_ENDSTOPS
#endif

#if !HAS_Y_AXIS
  #undef AVOID_OBSTACLES
  #undef ENDSTOPPULLUP_YMIN
  #undef ENDSTOPPULLUP_YMAX
  #undef Y_MIN_ENDSTOP_HIT_STATE
  #undef Y_MAX_ENDSTOP_HIT_STATE
  #undef Y2_DRIVER_TYPE
  #undef Y_ENABLE_ON
  #undef DISABLE_Y
  #undef INVERT_Y_DIR
  #undef Y_HOME_DIR
  #undef Y_MIN_POS
  #undef Y_MAX_POS
  #undef MANUAL_Y_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_Y
  #undef MAX_SOFTWARE_ENDSTOP_Y
#endif

#if HAS_Z_AXIS
  #ifdef Z4_DRIVER_TYPE
    #define NUM_Z_STEPPERS 4
  #elif defined(Z3_DRIVER_TYPE)
    #define NUM_Z_STEPPERS 3
  #elif defined(Z2_DRIVER_TYPE)
    #define NUM_Z_STEPPERS 2
  #else
    #define NUM_Z_STEPPERS 1
  #endif
#else
  #undef ENDSTOPPULLUP_ZMIN
  #undef ENDSTOPPULLUP_ZMAX
  #undef Z_MIN_ENDSTOP_HIT_STATE
  #undef Z_MAX_ENDSTOP_HIT_STATE
  #undef Z2_DRIVER_TYPE
  #undef Z3_DRIVER_TYPE
  #undef Z4_DRIVER_TYPE
  #undef Z_ENABLE_ON
  #undef DISABLE_Z
  #undef INVERT_Z_DIR
  #undef Z_HOME_DIR
  #undef Z_MIN_POS
  #undef Z_MAX_POS
  #undef MANUAL_Z_HOME_POS
  #undef Z_SAFE_HOMING
  #undef MIN_SOFTWARE_ENDSTOP_Z
  #undef MAX_SOFTWARE_ENDSTOP_Z
#endif

#if !HAS_I_AXIS
  #undef ENDSTOPPULLUP_IMIN
  #undef ENDSTOPPULLUP_IMAX
  #undef I_MIN_ENDSTOP_HIT_STATE
  #undef I_MAX_ENDSTOP_HIT_STATE
  #undef I_ENABLE_ON
  #undef DISABLE_I
  #undef INVERT_I_DIR
  #undef I_HOME_DIR
  #undef I_MIN_POS
  #undef I_MAX_POS
  #undef MANUAL_I_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_I
  #undef MAX_SOFTWARE_ENDSTOP_I
#endif

#if !HAS_J_AXIS
  #undef ENDSTOPPULLUP_JMIN
  #undef ENDSTOPPULLUP_JMAX
  #undef J_MIN_ENDSTOP_HIT_STATE
  #undef J_MAX_ENDSTOP_HIT_STATE
  #undef J_ENABLE_ON
  #undef DISABLE_J
  #undef INVERT_J_DIR
  #undef J_HOME_DIR
  #undef J_MIN_POS
  #undef J_MAX_POS
  #undef MANUAL_J_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_J
  #undef MAX_SOFTWARE_ENDSTOP_J
#endif

#if !HAS_K_AXIS
  #undef ENDSTOPPULLUP_KMIN
  #undef ENDSTOPPULLUP_KMAX
  #undef K_MIN_ENDSTOP_HIT_STATE
  #undef K_MAX_ENDSTOP_HIT_STATE
  #undef K_ENABLE_ON
  #undef DISABLE_K
  #undef INVERT_K_DIR
  #undef K_HOME_DIR
  #undef K_MIN_POS
  #undef K_MAX_POS
  #undef MANUAL_K_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_K
  #undef MAX_SOFTWARE_ENDSTOP_K
#endif

#if !HAS_U_AXIS
  #undef ENDSTOPPULLUP_UMIN
  #undef ENDSTOPPULLUP_UMAX
  #undef U_MIN_ENDSTOP_HIT_STATE
  #undef U_MAX_ENDSTOP_HIT_STATE
  #undef U_ENABLE_ON
  #undef DISABLE_U
  #undef INVERT_U_DIR
  #undef U_HOME_DIR
  #undef U_MIN_POS
  #undef U_MAX_POS
  #undef MANUAL_U_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_U
  #undef MAX_SOFTWARE_ENDSTOP_U
#endif

#if !HAS_V_AXIS
  #undef ENDSTOPPULLUP_VMIN
  #undef ENDSTOPPULLUP_VMAX
  #undef V_MIN_ENDSTOP_HIT_STATE
  #undef V_MAX_ENDSTOP_HIT_STATE
  #undef V_ENABLE_ON
  #undef DISABLE_V
  #undef INVERT_V_DIR
  #undef V_HOME_DIR
  #undef V_MIN_POS
  #undef V_MAX_POS
  #undef MANUAL_V_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_V
  #undef MAX_SOFTWARE_ENDSTOP_V
#endif

#if !HAS_W_AXIS
  #undef ENDSTOPPULLUP_WMIN
  #undef ENDSTOPPULLUP_WMAX
  #undef W_MIN_ENDSTOP_HIT_STATE
  #undef W_MAX_ENDSTOP_HIT_STATE
  #undef W_ENABLE_ON
  #undef DISABLE_W
  #undef INVERT_W_DIR
  #undef W_HOME_DIR
  #undef W_MIN_POS
  #undef W_MAX_POS
  #undef MANUAL_W_HOME_POS
  #undef MIN_SOFTWARE_ENDSTOP_W
  #undef MAX_SOFTWARE_ENDSTOP_W
#endif

#define _OR_HAS_DA(A) ENABLED(DISABLE_##A) ||
#if MAP(_OR_HAS_DA, X, Y, Z, I, J, K, U, V, W) 0
  #define HAS_DISABLE_MAIN_AXES 1
#endif
#if HAS_DISABLE_MAIN_AXES || ENABLED(DISABLE_E)
  #define HAS_DISABLE_AXES 1
#endif
#undef _OR_HAS_DA

#ifdef X2_DRIVER_TYPE
  #define HAS_X2_STEPPER 1
#endif
#ifdef Y2_DRIVER_TYPE
  #define HAS_Y2_STEPPER 1
#endif

/**
 * Number of Primary Linear Axes (e.g., XYZ)
 * X, XY, or XYZ axes. Excluding duplicate axes (X2, Y2, Z2, Z3, Z4)
 */
#if NUM_AXES >= 3
  #define PRIMARY_LINEAR_AXES 3
#else
  #define PRIMARY_LINEAR_AXES NUM_AXES
#endif

/**
 * Number of Secondary Axes (e.g., IJKUVW)
 * All linear/rotational axes between XYZ and E.
 */
#define SECONDARY_AXES SUB3(NUM_AXES)

/**
 * Number of Rotational Axes (e.g., IJK)
 * All axes for which AXIS*_ROTATES is defined.
 * For these axes, positions are specified in angular degrees.
 */
#if ENABLED(AXIS9_ROTATES)
  #define ROTATIONAL_AXES 6
#elif ENABLED(AXIS8_ROTATES)
  #define ROTATIONAL_AXES 5
#elif ENABLED(AXIS7_ROTATES)
  #define ROTATIONAL_AXES 4
#elif ENABLED(AXIS6_ROTATES)
  #define ROTATIONAL_AXES 3
#elif ENABLED(AXIS5_ROTATES)
  #define ROTATIONAL_AXES 2
#elif ENABLED(AXIS4_ROTATES)
  #define ROTATIONAL_AXES 1
#else
  #define ROTATIONAL_AXES 0
#endif

/**
 * Number of Secondary Linear Axes (e.g., UVW)
 * All secondary axes for which AXIS*_ROTATES is not defined.
 * Excluding primary axes and excluding duplicate axes (X2, Y2, Z2, Z3, Z4)
 */
#define SECONDARY_LINEAR_AXES (NUM_AXES - PRIMARY_LINEAR_AXES - ROTATIONAL_AXES)

/**
 * Number of Logical Axes (e.g., XYZIJKUVWE)
 * All logical axes that can be commanded directly by G-code.
 * Delta maps stepper-specific values to ABC steppers.
 */
#if HAS_EXTRUDERS
  #define LOGICAL_AXES INCREMENT(NUM_AXES)
#else
  #define LOGICAL_AXES NUM_AXES
#endif

/**
 * DISTINCT_E_FACTORS is set to give extruders (some) individual settings.
 *
 * DISTINCT_AXES is the number of distinct addressable axes (not steppers).
 *  Includes all linear axes plus all distinguished extruders.
 *  The default behavior is to treat all extruders as a single E axis
 *  with shared motion and temperature settings.
 *
 * DISTINCT_E is the number of distinguished extruders. By default this
 *  will be 1 which indicates all extruders share the same settings.
 *
 * E_INDEX_N(E) should be used to get the E index of any item that might be
 *  distinguished.
 */
#if ENABLED(DISTINCT_E_FACTORS) && E_STEPPERS > 1
  #define DISTINCT_AXES (NUM_AXES + E_STEPPERS)
  #define DISTINCT_E E_STEPPERS
  #define E_INDEX_N(E) (E)
#else
  #undef DISTINCT_E_FACTORS
  #define DISTINCT_AXES LOGICAL_AXES
  #define DISTINCT_E 1
  #define E_INDEX_N(E) 0
#endif

// Helper macros for extruder and hotend arrays
#define _EXTRUDER_LOOP(E) for (int8_t E = 0; E < EXTRUDERS; E++)
#define EXTRUDER_LOOP() _EXTRUDER_LOOP(e)
#define _HOTEND_LOOP(H) for (int8_t H = 0; H < HOTENDS; H++)
#define HOTEND_LOOP() _HOTEND_LOOP(e)

#define ARRAY_BY_EXTRUDERS(V...) ARRAY_N(EXTRUDERS, V)
#define ARRAY_BY_EXTRUDERS1(v1) ARRAY_N_1(EXTRUDERS, v1)
#define ARRAY_BY_HOTENDS(V...) ARRAY_N(HOTENDS, V)
#define ARRAY_BY_HOTENDS1(v1) ARRAY_N_1(HOTENDS, v1)

// Support for SD Card and other file storage
#if ENABLED(SDSUPPORT)
  #define HAS_MEDIA 1
#endif

/**
 * Conditionals for the configured LCD / Controller
 */

// MKS_LCD12864A/B is a variant of MKS_MINI_12864
#if ANY(MKS_LCD12864A, MKS_LCD12864B)
  #define MKS_MINI_12864
#endif

// MKS_MINI_12864_V3 , BTT_MINI_12864 and BEEZ_MINI_12864 have identical pinouts to FYSETC_MINI_12864_2_1
#if ANY(MKS_MINI_12864_V3, BTT_MINI_12864, BEEZ_MINI_12864)
  #define FYSETC_MINI_12864_2_1
#endif

// Old settings are now conditional on DGUS_LCD_UI
#if DGUS_UI_IS(ORIGIN)
  #define DGUS_LCD_UI_ORIGIN 1
#elif DGUS_UI_IS(FYSETC)
  #define DGUS_LCD_UI_FYSETC 1
#elif DGUS_UI_IS(HIPRECY)
  #define DGUS_LCD_UI_HIPRECY 1
#elif DGUS_UI_IS(MKS)
  #define DGUS_LCD_UI_MKS 1
#elif DGUS_UI_IS(RELOADED)
  #define DGUS_LCD_UI_RELOADED 1
#elif DGUS_UI_IS(IA_CREALITY)
  #define DGUS_LCD_UI_IA_CREALITY 1
#elif DGUS_UI_IS(E3S1PRO)
  #define DGUS_LCD_UI_E3S1PRO 1
#endif

/**
 * General Flags that may be set below by specific LCDs
 *
 *  DOGLCD                  : Run a Graphical LCD through U8GLib (with MarlinUI)
 *  IS_ULTIPANEL            : Define LCD_PINS_D5/6/7 for direct-connected "Ultipanel" LCDs
 *  HAS_WIRED_LCD           : Ultra LCD, not necessarily Ultipanel.
 *  IS_RRD_SC               : Common RRD Smart Controller digital interface pins
 *  IS_RRD_FG_SC            : Common RRD Full Graphical Smart Controller digital interface pins
 *  IS_U8GLIB_ST7920        : Most common DOGM display SPI interface, supporting a "lightweight" display mode.
 *  U8GLIB_SH1106           : SH1106 OLED with I2C interface via U8GLib
 *  IS_U8GLIB_SSD1306       : SSD1306 OLED with I2C interface via U8GLib (U8GLIB_SSD1306)
 *  U8GLIB_SSD1309          : SSD1309 OLED with I2C interface via U8GLib (HAS_U8GLIB_I2C_OLED, HAS_WIRED_LCD, DOGLCD)
 *  IS_U8GLIB_ST7565_64128N : ST7565 128x64 LCD with SPI interface via U8GLib
 *  IS_U8GLIB_LM6059_AF     : LM6059 with Hardware SPI via U8GLib
 */
#if ANY(MKS_MINI_12864, ENDER2_STOCKDISPLAY)

  #define MINIPANEL

#elif ENABLED(YHCB2004)

  #define IS_ULTIPANEL 1

#elif ENABLED(CARTESIO_UI)

  #define DOGLCD
  #define IS_ULTIPANEL 1

#elif ANY(DWIN_MARLINUI_PORTRAIT, DWIN_MARLINUI_LANDSCAPE)

  #define IS_DWIN_MARLINUI 1
  #define IS_ULTIPANEL 1

#elif ENABLED(ZONESTAR_LCD)

  #define HAS_ADC_BUTTONS 1
  #define REPRAPWORLD_KEYPAD_MOVE_STEP 10.0
  #define ADC_KEY_NUM 8
  #define IS_ULTIPANEL 1

  // This helps to implement HAS_ADC_BUTTONS menus
  #define REVERSE_MENU_DIRECTION
  #define STD_ENCODER_PULSES_PER_STEP 1
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #define ENCODER_FEEDRATE_DEADZONE 2

#elif ENABLED(ZONESTAR_12864LCD)
  #define DOGLCD
  #define IS_RRD_SC 1
  #define IS_U8GLIB_ST7920 1

#elif ENABLED(ZONESTAR_12864OLED)
  #define IS_RRD_SC 1
  #define U8GLIB_SH1106

#elif ENABLED(ZONESTAR_12864OLED_SSD1306)
  #define IS_RRD_SC 1
  #define IS_U8GLIB_SSD1306

#elif ENABLED(RADDS_DISPLAY)
  #define IS_ULTIPANEL 1
  #define STD_ENCODER_PULSES_PER_STEP 2

#elif ANY(miniVIKI, VIKI2, WYH_L12864, ELB_FULL_GRAPHIC_CONTROLLER, AZSMZ_12864, EMOTION_TECH_LCD)

  #define DOGLCD
  #define IS_DOGM_12864 1
  #define IS_ULTIPANEL 1

  #if ENABLED(miniVIKI)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(VIKI2)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(WYH_L12864)
    #define IS_U8GLIB_ST7565_64128N 1
    #define ST7565_XOFFSET 0x04
  #elif ENABLED(ELB_FULL_GRAPHIC_CONTROLLER)
    #define IS_U8GLIB_LM6059_AF 1
  #elif ENABLED(AZSMZ_12864)
    #define IS_U8GLIB_ST7565_64128N 1
  #elif ENABLED(EMOTION_TECH_LCD)
    #define IS_U8GLIB_ST7565_64128N 1
    #define ST7565_VOLTAGE_DIVIDER_VALUE 0x07
  #endif

#elif ENABLED(OLED_PANEL_TINYBOY2)

  #define IS_U8GLIB_SSD1306
  #define IS_ULTIPANEL 1

#elif ENABLED(RA_CONTROL_PANEL)

  #define LCD_I2C_TYPE_PCA8574
  #define LCD_I2C_ADDRESS 0x27   // I2C Address of the port expander
  #define IS_ULTIPANEL 1

#elif ENABLED(REPRAPWORLD_GRAPHICAL_LCD)

  #define DOGLCD
  #define IS_U8GLIB_ST7920 1
  #define IS_ULTIPANEL 1
  #define ENCODER_PULSES_PER_STEP 2

#elif ENABLED(MKS_12864OLED)

  #define IS_RRD_SC 1
  #define U8GLIB_SH1106

#elif ENABLED(MKS_12864OLED_SSD1306)

  #define IS_RRD_SC 1
  #define IS_U8GLIB_SSD1306

#elif ENABLED(SAV_3DGLCD)

  #ifdef U8GLIB_SSD1306
    #define IS_U8GLIB_SSD1306 // Allow for U8GLIB_SSD1306 + SAV_3DGLCD
  #endif
  #define IS_NEWPANEL 1

#elif ENABLED(FYSETC_242_OLED_12864)

  #define IS_RRD_SC 1
  #define U8GLIB_SH1106

  #ifndef NEOPIXEL_BRIGHTNESS
    #define NEOPIXEL_BRIGHTNESS 127
  #endif

#elif ANY(FYSETC_MINI_12864_X_X, FYSETC_MINI_12864_1_2, FYSETC_MINI_12864_2_0, FYSETC_MINI_12864_2_1, FYSETC_GENERIC_12864_1_1)

  #define FYSETC_MINI_12864
  #define DOGLCD
  #define IS_ULTIPANEL 1
  #define LED_COLORS_REDUCE_GREEN

  // Require LED backlighting enabled
  #if ENABLED(FYSETC_MINI_12864_2_1)
    #ifndef NEOPIXEL_BRIGHTNESS
      #define NEOPIXEL_BRIGHTNESS 127
    #endif
    //#define NEOPIXEL_STARTUP_TEST
  #endif

#elif ENABLED(ULTI_CONTROLLER)

  #define IS_ULTIPANEL 1
  #define U8GLIB_SSD1309
  #define LCD_RESET_PIN LCD_PINS_D6 //  This controller need a reset pin
  #define STD_ENCODER_PULSES_PER_STEP 4
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #ifndef PCA9632
    #define PCA9632
  #endif

#elif ENABLED(MAKEBOARD_MINI_2_LINE_DISPLAY_1602)

  #define IS_RRD_SC 1
  #define LCD_WIDTH 16
  #define LCD_HEIGHT 2

#elif ANY(TFTGLCD_PANEL_SPI, TFTGLCD_PANEL_I2C)

  #define IS_TFTGLCD_PANEL 1
  #define IS_ULTIPANEL 1                    // Note that IS_ULTIPANEL leads to HAS_WIRED_LCD

  #if HAS_MEDIA && DISABLED(LCD_PROGRESS_BAR)
    #define LCD_PROGRESS_BAR
  #endif
  #if ENABLED(TFTGLCD_PANEL_I2C)
    #define LCD_I2C_ADDRESS           0x33  // Must be 0x33 for STM32 main boards and equal to panel's I2C slave address
  #endif
  #define LCD_USE_I2C_BUZZER                // Enable buzzer on LCD, used for both I2C and SPI buses (LiquidTWI2 not required)
  #define STD_ENCODER_PULSES_PER_STEP 2
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
  #define LCD_WIDTH                   20    // 20 or 24 chars in line
  #define LCD_HEIGHT                  10    // Character lines
  #define LCD_CONTRAST_MIN            127
  #define LCD_CONTRAST_MAX            255
  #define LCD_CONTRAST_DEFAULT        250
  #define CONVERT_TO_EXT_ASCII        // Use extended 128-255 symbols from ASCII table.
                                      // At this time present conversion only for cyrillic - bg, ru and uk languages.
                                      // First 7 ASCII symbols in panel font must be replaced with Marlin's special symbols.

#elif ENABLED(CR10_STOCKDISPLAY)

  #define IS_RRD_FG_SC 1
  #define NO_LCD_SDCARD
  #define LCD_ST7920_DELAY_1           125
  #define LCD_ST7920_DELAY_2           125
  #define LCD_ST7920_DELAY_3           125

#elif ANY(ANET_FULL_GRAPHICS_LCD, CTC_A10S_A13)

  #define IS_RRD_FG_SC 1
  #define LCD_ST7920_DELAY_1           150
  #define LCD_ST7920_DELAY_2           150
  #define LCD_ST7920_DELAY_3           150

#elif ANY(REPRAP_DISCOUNT_FULL_GRAPHIC_SMART_CONTROLLER, BQ_LCD_SMART_CONTROLLER, K3D_FULL_GRAPHIC_SMART_CONTROLLER)

  #define IS_RRD_FG_SC 1

#elif ENABLED(REPRAP_DISCOUNT_SMART_CONTROLLER)

  #define IS_RRD_SC 1   // RepRapDiscount LCD or Graphical LCD with rotary click encoder

#elif ENABLED(K3D_242_OLED_CONTROLLER)

  #define IS_RRD_SC 1
  #define U8GLIB_SSD1309

#endif

// ST7920-based graphical displays
#if ANY(IS_RRD_FG_SC, LCD_FOR_MELZI, SILVER_GATE_GLCD_CONTROLLER)
  #define DOGLCD
  #define IS_U8GLIB_ST7920 1
  #define IS_RRD_SC 1
#endif

// ST7565 / 64128N graphical displays
#if ANY(MAKRPANEL, MINIPANEL)
  #define IS_ULTIPANEL 1
  #define DOGLCD
  #if ENABLED(MAKRPANEL)
    #define IS_U8GLIB_ST7565_64128N 1
  #endif
#endif

#if ENABLED(IS_U8GLIB_SSD1306)
  #define U8GLIB_SSD1306
#endif

#if ENABLED(OVERLORD_OLED)
  #define IS_ULTIPANEL 1
  #define U8GLIB_SH1106
  /**
   * PCA9632 for buzzer and LEDs via i2c
   * No auto-inc, red and green leds switched, buzzer
   */
  #define PCA9632
  #define PCA9632_NO_AUTO_INC
  #define PCA9632_GRN         0x00
  #define PCA9632_RED         0x02
  #define PCA9632_BUZZER
  #define PCA9632_BUZZER_DATA { 0x09, 0x02 }

  #define STD_ENCODER_PULSES_PER_STEP     1 // Overlord uses buttons
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif

// 128x64 I2C OLED LCDs - SSD1306/SSD1309/SH1106
#if ANY(U8GLIB_SSD1306, U8GLIB_SSD1309, U8GLIB_SH1106)
  #define HAS_U8GLIB_I2C_OLED 1
  #define HAS_WIRED_LCD 1
  #define DOGLCD
#endif

/**
 * SPI Ultipanels
 */

// Basic Ultipanel-like displays
#if ANY(ULTIMAKERCONTROLLER, IS_RRD_SC, G3D_PANEL, RIGIDBOT_PANEL, PANEL_ONE, U8GLIB_SH1106)
  #define IS_ULTIPANEL 1
#endif

// Einstart OLED has Cardinal nav via pins defined in pins_EINSTART-S.h
#if ENABLED(U8GLIB_SH1106_EINSTART)
  #define DOGLCD
  #define IS_ULTIPANEL 1
#endif

// TFT Legacy options masquerade as TFT_GENERIC
#if ANY(FSMC_GRAPHICAL_TFT, SPI_GRAPHICAL_TFT, TFT_320x240, TFT_480x320, TFT_320x240_SPI, TFT_480x320_SPI, TFT_LVGL_UI_FSMC, TFT_LVGL_UI_SPI)
  #define IS_LEGACY_TFT 1
  #define TFT_GENERIC
  #if ANY(FSMC_GRAPHICAL_TFT, TFT_320x240, TFT_480x320, TFT_LVGL_UI_FSMC)
    #define TFT_INTERFACE_FSMC
  #elif ANY(SPI_GRAPHICAL_TFT, TFT_320x240_SPI, TFT_480x320_SPI, TFT_LVGL_UI_SPI)
    #define TFT_INTERFACE_SPI
  #endif
  #if ANY(FSMC_GRAPHICAL_TFT, SPI_GRAPHICAL_TFT)
    #define TFT_CLASSIC_UI
  #elif ANY(TFT_320x240, TFT_480x320, TFT_320x240_SPI, TFT_480x320_SPI)
    #define TFT_COLOR_UI
  #elif ANY(TFT_LVGL_UI_FSMC, TFT_LVGL_UI_SPI)
    #define TFT_LVGL_UI
  #endif
#endif

// FSMC/SPI TFT Panels (LVGL)
#if ENABLED(TFT_LVGL_UI)
  #define HAS_TFT_LVGL_UI 1
  #define SERIAL_RUNTIME_HOOK 1
#endif

// FSMC/SPI TFT Panels
#if ENABLED(TFT_CLASSIC_UI)
  #define TFT_SCALED_DOGLCD 1
#endif

#if TFT_SCALED_DOGLCD
  #define DOGLCD
  #define IS_ULTIPANEL 1
  #define DELAYED_BACKLIGHT_INIT
#elif HAS_TFT_LVGL_UI
  #define DELAYED_BACKLIGHT_INIT
#endif

// Color UI
#if ENABLED(TFT_COLOR_UI)
  #define HAS_GRAPHICAL_TFT 1
  #define IS_ULTIPANEL 1
#endif

/**
 * I2C Panels
 */

#if ANY(IS_RRD_SC, IS_DOGM_12864, OLED_PANEL_TINYBOY2, LCD_I2C_PANELOLU2)

  #define STD_ENCODER_PULSES_PER_STEP 4
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

  #if ENABLED(LCD_I2C_PANELOLU2)  // PANELOLU2 LCD with status LEDs, separate encoder and click inputs
    #define LCD_I2C_TYPE_MCP23017 // I2C Character-based 12864 display
    #define LCD_I2C_ADDRESS 0x20  // I2C Address of the port expander
    #define LCD_USE_I2C_BUZZER    // Enable buzzer on LCD (optional)
    #define IS_ULTIPANEL 1
  #endif

#elif ANY(LCD_SAINSMART_I2C_1602, LCD_SAINSMART_I2C_2004)

  #define LCD_I2C_TYPE_PCF8575    // I2C Character-based 12864 display
  #define LCD_I2C_ADDRESS 0x27    // I2C Address of the port expander
  #define IS_ULTIPANEL 1

  #if ENABLED(LCD_SAINSMART_I2C_2004)
    #define LCD_WIDTH 20
    #define LCD_HEIGHT 4
  #endif

#elif ENABLED(LCD_I2C_VIKI)

  /**
   * Panucatt VIKI LCD with status LEDs, integrated click & L/R/U/P buttons, separate encoder inputs
   *
   * This uses the LiquidTWI2 library v1.2.3 or later ( https://github.com/lincomatic/LiquidTWI2 )
   * Make sure the LiquidTWI2 directory is placed in the Arduino or Sketchbook libraries subdirectory.
   * Note: The pause/stop/resume LCD button pin should be connected to the Arduino
   *       BTN_ENC pin (or set BTN_ENC to -1 if not used)
   */
  #define LCD_I2C_TYPE_MCP23017
  #define LCD_I2C_ADDRESS 0x20 // I2C Address of the port expander
  #define LCD_USE_I2C_BUZZER   // Enable buzzer on LCD (requires LiquidTWI2 v1.2.3 or later)
  #define IS_ULTIPANEL 1

  #define ENCODER_FEEDRATE_DEADZONE 4

  #define STD_ENCODER_PULSES_PER_STEP 1
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 2

#elif ENABLED(G3D_PANEL)

  #define STD_ENCODER_PULSES_PER_STEP 2
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1

#endif

#if ANY(LCD_I2C_TYPE_MCP23017, LCD_I2C_TYPE_MCP23008) && DISABLED(NO_LCD_DETECT)
  #define DETECT_I2C_LCD_DEVICE 1
#endif

// Encoder behavior
#ifndef STD_ENCODER_PULSES_PER_STEP
  #if ENABLED(TOUCH_SCREEN)
    #define STD_ENCODER_PULSES_PER_STEP 2
  #else
    #define STD_ENCODER_PULSES_PER_STEP 5
  #endif
#endif
#ifndef STD_ENCODER_STEPS_PER_MENU_ITEM
  #define STD_ENCODER_STEPS_PER_MENU_ITEM 1
#endif
#ifndef ENCODER_PULSES_PER_STEP
  #define ENCODER_PULSES_PER_STEP STD_ENCODER_PULSES_PER_STEP
#endif
#ifndef ENCODER_STEPS_PER_MENU_ITEM
  #define ENCODER_STEPS_PER_MENU_ITEM STD_ENCODER_STEPS_PER_MENU_ITEM
#endif
#ifndef ENCODER_FEEDRATE_DEADZONE
  #define ENCODER_FEEDRATE_DEADZONE 6
#endif

// Shift register panels
// ---------------------
// 2 wire Non-latching LCD SR from:
// https://github.com/fmalpartida/New-LiquidCrystal/wiki/schematics#user-content-ShiftRegister_connection
#if ENABLED(FF_INTERFACEBOARD)
  #define SR_LCD_3W_NL    // Non latching 3 wire shift register
  #define IS_ULTIPANEL 1
#elif ENABLED(SAV_3DLCD)
  #define SR_LCD_2W_NL    // Non latching 2 wire shift register
  #define IS_ULTIPANEL 1
#elif ENABLED(ULTIPANEL)
  #define IS_ULTIPANEL 1
#endif

#if ANY(IS_ULTIPANEL, ULTRA_LCD)
  #define HAS_WIRED_LCD 1
#endif

#if ANY(IS_ULTIPANEL, REPRAPWORLD_KEYPAD)
  #define IS_NEWPANEL 1
#endif

#if ANY(ZONESTAR_LCD, REPRAPWORLD_KEYPAD)
  #define IS_RRW_KEYPAD 1
  #ifndef REPRAPWORLD_KEYPAD_MOVE_STEP
    #define REPRAPWORLD_KEYPAD_MOVE_STEP 1.0
  #endif
#endif

// Aliases for LCD features
#if !DGUS_UI_IS(NONE) || ENABLED(ANYCUBIC_LCD_VYPER)
  #define HAS_DGUS_LCD 1
  #if DGUS_UI_IS(ORIGIN, FYSETC, HIPRECY, MKS)
    #define HAS_DGUS_LCD_CLASSIC 1
  #endif
#endif

// Extensible UI serial touch screens. (See src/lcd/extui)
#if ANY(HAS_DGUS_LCD, MALYAN_LCD, ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON, NEXTION_TFT, TOUCH_UI_FTDI_EVE, EXTUI_LVGL_TEST)
  #define IS_EXTUI 1 // Just for sanity check.
  #define EXTENSIBLE_UI
#endif

// Aliases for LCD features
#if ANY(DWIN_CREALITY_LCD, DWIN_LCD_PROUI)
  #define HAS_DWIN_E3V2_BASIC 1
#endif
#if ANY(HAS_DWIN_E3V2_BASIC, DWIN_CREALITY_LCD_JYERSUI)
  #define HAS_DWIN_E3V2 1
#endif

// E3V2 extras
#if HAS_DWIN_E3V2 || IS_DWIN_MARLINUI
  #define SERIAL_CATCHALL 0
  #define HAS_LCD_BRIGHTNESS 1
  #define LCD_BRIGHTNESS_MAX 250
#endif

#if ENABLED(DWIN_LCD_PROUI)
  #define DO_LIST_BIN_FILES 1
  #define LCD_BRIGHTNESS_DEFAULT 127
#endif

// Serial Controllers require LCD_SERIAL_PORT
#if ANY(IS_DWIN_MARLINUI, HAS_DWIN_E3V2, HAS_DGUS_LCD, MALYAN_LCD, ANYCUBIC_LCD_I3MEGA, ANYCUBIC_LCD_CHIRON, NEXTION_TFT)
  #define LCD_IS_SERIAL_HOST 1
#endif

#if HAS_WIRED_LCD
  #if ENABLED(DOGLCD)
    #define HAS_MARLINUI_U8GLIB 1
  #elif IS_TFTGLCD_PANEL
    // Neither DOGM nor HD44780. Fully customized interface.
  #elif IS_DWIN_MARLINUI
    // Since HAS_MARLINUI_U8GLIB refers to U8G displays
    // the DWIN display can define its own flags
  #elif !HAS_GRAPHICAL_TFT
    #define HAS_MARLINUI_HD44780 1
  #endif
#endif

#if ANY(HAS_WIRED_LCD, EXTENSIBLE_UI, DWIN_LCD_PROUI, DWIN_CREALITY_LCD_JYERSUI)
  /**
   * HAS_DISPLAY indicates the display uses these MarlinUI methods...
   *  - update
   *  - abort_print
   *  - pause_print
   *  - resume_print
   *  - poweroff        (for PSU_CONTROL and HAS_MARLINUI_MENU)
   *
   *  ...and implements these MarlinUI methods:
   *  - zoffset_overlay (if BABYSTEP_GFX_OVERLAY or MESH_EDIT_GFX_OVERLAY are supported)
   *  - draw_kill_screen
   *  - kill_screen
   *  - draw_status_message
   */
  #define HAS_DISPLAY 1
#endif

#if ANY(HAS_DISPLAY, DWIN_CREALITY_LCD)
  #define HAS_UI_UPDATE 1
#endif

#if HAS_WIRED_LCD && !HAS_GRAPHICAL_TFT && !IS_DWIN_MARLINUI
  #define HAS_LCDPRINT 1
#endif

#if HAS_DISPLAY || HAS_LCDPRINT
  #define HAS_UTF8_UTILS 1
#endif

#if ANY(HAS_DISPLAY, HAS_DWIN_E3V2)
  #define HAS_STATUS_MESSAGE 1
#endif

#if IS_ULTIPANEL && DISABLED(NO_LCD_MENUS)
  #define HAS_MARLINUI_MENU 1
#endif

#if ANY(HAS_MARLINUI_MENU, EXTENSIBLE_UI, HAS_DWIN_E3V2)
  #define HAS_MANUAL_MOVE_MENU 1
#endif

#if HAS_MARLINUI_U8GLIB
  #ifndef LCD_PIXEL_WIDTH
    #define LCD_PIXEL_WIDTH 128
  #endif
  #ifndef LCD_PIXEL_HEIGHT
    #define LCD_PIXEL_HEIGHT 64
  #endif
#endif

/**
 * Disable unused SINGLENOZZLE sub-options
 */
#if DISABLED(SINGLENOZZLE)
  #undef SINGLENOZZLE_STANDBY_TEMP
#endif
#if !ALL(HAS_FAN, SINGLENOZZLE)
  #undef SINGLENOZZLE_STANDBY_FAN
#endif

// No inactive extruders with SWITCHING_NOZZLE or Průša MMU1 or just 1 E stepper exists
#if HAS_SWITCHING_NOZZLE || HAS_PRUSA_MMU1 || E_STEPPERS < 2
  #undef DISABLE_OTHER_EXTRUDERS
#endif

// Switching extruder has its own servo?
#if ENABLED(SWITCHING_EXTRUDER) && (!HAS_SWITCHING_NOZZLE || SWITCHING_EXTRUDER_SERVO_NR != SWITCHING_NOZZLE_SERVO_NR)
  #define DO_SWITCH_EXTRUDER 1
#endif

/**
 * The BLTouch Probe emulates a servo probe
 * and uses "special" angles for its state.
 */
#if ENABLED(BLTOUCH) && !defined(Z_PROBE_SERVO_NR)
  #define Z_PROBE_SERVO_NR 0
#endif

/**
 * Set a flag for a servo probe (or BLTouch)
 */
#ifdef Z_PROBE_SERVO_NR
  #define HAS_Z_SERVO_PROBE 1
#endif
#if ANY(HAS_Z_SERVO_PROBE, SWITCHING_EXTRUDER, SWITCHING_NOZZLE)
  #define HAS_SERVO_ANGLES 1
#endif
#if !HAS_SERVO_ANGLES
  #undef EDITABLE_SERVO_ANGLES
#endif

/**
 * Set flags for any form of bed probe
 */
#if ANY(TOUCH_MI_PROBE, Z_PROBE_ALLEN_KEY, HAS_Z_SERVO_PROBE, SOLENOID_PROBE, Z_PROBE_SLED, RACK_AND_PINION_PROBE, SENSORLESS_PROBING, MAGLEV4, MAG_MOUNTED_PROBE, BIQU_MICROPROBE_V1, BIQU_MICROPROBE_V2)
  #define HAS_STOWABLE_PROBE 1
#endif
#if ANY(HAS_STOWABLE_PROBE, FIX_MOUNTED_PROBE, BD_SENSOR, NOZZLE_AS_PROBE)
  #define HAS_BED_PROBE 1
#endif

// Probing tool change
#if !HAS_MULTI_EXTRUDER
  #undef PROBING_TOOL
#endif
#if HAS_BED_PROBE && defined(PROBING_TOOL)
  #define DO_TOOLCHANGE_FOR_PROBING 1
#endif

/**
 * Fill in undefined Filament Sensor options
 */
#if ENABLED(FILAMENT_RUNOUT_SENSOR)
  #if NUM_RUNOUT_SENSORS >= 1
    #ifndef FIL_RUNOUT1_STATE
      #define FIL_RUNOUT1_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT1_PULLUP
      #define FIL_RUNOUT1_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT1_PULLDOWN
      #define FIL_RUNOUT1_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 2
    #ifndef FIL_RUNOUT2_STATE
      #define FIL_RUNOUT2_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT2_PULLUP
      #define FIL_RUNOUT2_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT2_PULLDOWN
      #define FIL_RUNOUT2_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 3
    #ifndef FIL_RUNOUT3_STATE
      #define FIL_RUNOUT3_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT3_PULLUP
      #define FIL_RUNOUT3_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT3_PULLDOWN
      #define FIL_RUNOUT3_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 4
    #ifndef FIL_RUNOUT4_STATE
      #define FIL_RUNOUT4_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT4_PULLUP
      #define FIL_RUNOUT4_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT4_PULLDOWN
      #define FIL_RUNOUT4_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 5
    #ifndef FIL_RUNOUT5_STATE
      #define FIL_RUNOUT5_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT5_PULLUP
      #define FIL_RUNOUT5_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT5_PULLDOWN
      #define FIL_RUNOUT5_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 6
    #ifndef FIL_RUNOUT6_STATE
      #define FIL_RUNOUT6_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT6_PULLUP
      #define FIL_RUNOUT6_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT6_PULLDOWN
      #define FIL_RUNOUT6_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 7
    #ifndef FIL_RUNOUT7_STATE
      #define FIL_RUNOUT7_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT7_PULLUP
      #define FIL_RUNOUT7_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT7_PULLDOWN
      #define FIL_RUNOUT7_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_RUNOUT_SENSORS >= 8
    #ifndef FIL_RUNOUT8_STATE
      #define FIL_RUNOUT8_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_RUNOUT8_PULLUP
      #define FIL_RUNOUT8_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_RUNOUT8_PULLDOWN
      #define FIL_RUNOUT8_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
#endif // FILAMENT_RUNOUT_SENSOR

#if ENABLED(FILAMENT_SWITCH_AND_MOTION)
  #if NUM_MOTION_SENSORS >= 1
    #ifndef FIL_MOTION1_STATE
      #define FIL_MOTION1_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION1_PULLUP
      #define FIL_MOTION1_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION1_PULLDOWN
      #define FIL_MOTION1_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 2
    #ifndef FIL_MOTION2_STATE
      #define FIL_MOTION2_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION2_PULLUP
      #define FIL_MOTION2_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION2_PULLDOWN
      #define FIL_MOTION2_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 3
    #ifndef FIL_MOTION3_STATE
      #define FIL_MOTION3_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION3_PULLUP
      #define FIL_MOTION3_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION3_PULLDOWN
      #define FIL_MOTION3_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 4
    #ifndef FIL_MOTION4_STATE
      #define FIL_MOTION4_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION4_PULLUP
      #define FIL_MOTION4_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION4_PULLDOWN
      #define FIL_MOTION4_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 5
    #ifndef FIL_MOTION5_STATE
      #define FIL_MOTION5_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION5_PULLUP
      #define FIL_MOTION5_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION5_PULLDOWN
      #define FIL_MOTION5_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 6
    #ifndef FIL_MOTION6_STATE
      #define FIL_MOTION6_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION6_PULLUP
      #define FIL_MOTION6_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION6_PULLDOWN
      #define FIL_MOTION6_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 7
    #ifndef FIL_MOTION7_STATE
      #define FIL_MOTION7_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION7_PULLUP
      #define FIL_MOTION7_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION7_PULLDOWN
      #define FIL_MOTION7_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
  #if NUM_MOTION_SENSORS >= 8
    #ifndef FIL_MOTION8_STATE
      #define FIL_MOTION8_STATE FIL_RUNOUT_STATE
    #endif
    #ifndef FIL_MOTION8_PULLUP
      #define FIL_MOTION8_PULLUP FIL_RUNOUT_PULLUP
    #endif
    #ifndef FIL_MOTION8_PULLDOWN
      #define FILMOTION8_PULLDOWN FIL_RUNOUT_PULLDOWN
    #endif
  #endif
#endif // FILAMENT_SWITCH_AND_MOTION

// Homing to Min or Max
#if HAS_X_AXIS
  #if X_HOME_DIR > 0
    #define X_HOME_TO_MAX 1
  #elif X_HOME_DIR < 0
    #define X_HOME_TO_MIN 1
  #endif
#endif
#if HAS_Y_AXIS
  #if Y_HOME_DIR > 0
    #define Y_HOME_TO_MAX 1
  #elif Y_HOME_DIR < 0
    #define Y_HOME_TO_MIN 1
  #endif
#endif
#if HAS_Z_AXIS
  #if Z_HOME_DIR > 0
    #define Z_HOME_TO_MAX 1
  #elif Z_HOME_DIR < 0
    #define Z_HOME_TO_MIN 1
  #endif
#endif
#if HAS_I_AXIS
  #if I_HOME_DIR > 0
    #define I_HOME_TO_MAX 1
  #elif I_HOME_DIR < 0
    #define I_HOME_TO_MIN 1
  #endif
#endif
#if HAS_J_AXIS
  #if J_HOME_DIR > 0
    #define J_HOME_TO_MAX 1
  #elif J_HOME_DIR < 0
    #define J_HOME_TO_MIN 1
  #endif
#endif
#if HAS_K_AXIS
  #if K_HOME_DIR > 0
    #define K_HOME_TO_MAX 1
  #elif K_HOME_DIR < 0
    #define K_HOME_TO_MIN 1
  #endif
#endif
#if HAS_U_AXIS
  #if U_HOME_DIR > 0
    #define U_HOME_TO_MAX 1
  #elif U_HOME_DIR < 0
    #define U_HOME_TO_MIN 1
  #endif
#endif
#if HAS_V_AXIS
  #if V_HOME_DIR > 0
    #define V_HOME_TO_MAX 1
  #elif V_HOME_DIR < 0
    #define V_HOME_TO_MIN 1
  #endif
#endif
#if HAS_W_AXIS
  #if W_HOME_DIR > 0
    #define W_HOME_TO_MAX 1
  #elif W_HOME_DIR < 0
    #define W_HOME_TO_MIN 1
  #endif
#endif

/**
 * Conditionals based on the type of Bed Probe
 */
#if HAS_BED_PROBE
  #if ALL(DELTA, SENSORLESS_PROBING)
    #define HAS_DELTA_SENSORLESS_PROBING 1
  #else
    #define HAS_REAL_BED_PROBE 1
  #endif
  #if HAS_REAL_BED_PROBE && NONE(Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN, Z_SPI_SENSORLESS)
    #define NEED_Z_MIN_PROBE_PIN 1
  #endif
  #if Z_HOME_TO_MIN && (!NEED_Z_MIN_PROBE_PIN || ENABLED(USE_PROBE_FOR_Z_HOMING))
    #define HOMING_Z_WITH_PROBE 1
  #endif
  #if DISABLED(NOZZLE_AS_PROBE)
    #define HAS_PROBE_XY_OFFSET 1
  #endif
  #if ANY(Z_PROBE_ALLEN_KEY, MAG_MOUNTED_PROBE)
    #define PROBE_TRIGGERED_WHEN_STOWED_TEST 1 // Extra test for Allen Key Probe
  #endif
  #ifndef Z_CLEARANCE_BETWEEN_PROBES
    #define Z_CLEARANCE_BETWEEN_PROBES 5
  #endif
  #ifndef Z_CLEARANCE_MULTI_PROBE
    #define Z_CLEARANCE_MULTI_PROBE 5
  #endif
  #ifndef Z_PROBE_ERROR_TOLERANCE
    #define Z_PROBE_ERROR_TOLERANCE Z_CLEARANCE_MULTI_PROBE
  #endif
  #if MULTIPLE_PROBING > 1
    #if EXTRA_PROBING > 0
      #define TOTAL_PROBING (MULTIPLE_PROBING + EXTRA_PROBING)
    #else
      #define TOTAL_PROBING MULTIPLE_PROBING
    #endif
  #endif
#else
  // Clear probe pin settings when no probe is selected
  #undef Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN
  #undef USE_PROBE_FOR_Z_HOMING
  #undef Z_MIN_PROBE_REPEATABILITY_TEST
  #undef HOMING_Z_WITH_PROBE
  #undef Z_CLEARANCE_MULTI_PROBE
  #undef Z_PROBE_ERROR_TOLERANCE
  #undef MULTIPLE_PROBING
  #undef EXTRA_PROBING
  #undef PROBE_OFFSET_ZMIN
  #undef PROBE_OFFSET_ZMAX
  #undef PAUSE_BEFORE_DEPLOY_STOW
  #undef PAUSE_PROBE_DEPLOY_WHEN_TRIGGERED
  #undef PROBING_HEATERS_OFF
  #undef WAIT_FOR_BED_HEATER
  #undef WAIT_FOR_HOTEND
  #undef PROBING_STEPPERS_OFF
  #undef DELAY_BEFORE_PROBING
  #undef PREHEAT_BEFORE_PROBING
  #undef PROBING_NOZZLE_TEMP
  #undef PROBING_BED_TEMP
  #undef NOZZLE_TO_PROBE_OFFSET
#endif

#ifndef Z_CLEARANCE_DEPLOY_PROBE
  #define Z_CLEARANCE_DEPLOY_PROBE 10
#endif
#ifndef Z_PROBE_LOW_POINT
  #define Z_PROBE_LOW_POINT -5
#endif

#if ENABLED(BELTPRINTER) && !defined(HOME_Y_BEFORE_X)
  #define HOME_Y_BEFORE_X
#endif

#if Z_HOME_TO_MAX && DISABLED(Z_SAFE_HOMING)
  #define HOME_Z_FIRST // If homing away from BED do Z first
#endif

/**
 * Conditionals based on the type of Bed Leveling
 */
#if ENABLED(AUTO_BED_LEVELING_UBL)
  #undef LCD_BED_LEVELING
  #if ANY(DELTA, SEGMENT_LEVELED_MOVES)
    #define UBL_SEGMENTED 1
  #endif
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_3POINT)
  #define ABL_PLANAR 1
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR)
  #define ABL_USES_GRID 1
#endif
#if ANY(AUTO_BED_LEVELING_LINEAR, AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_3POINT)
  #define HAS_ABL_NOT_UBL 1
#endif
#if ANY(AUTO_BED_LEVELING_BILINEAR, AUTO_BED_LEVELING_UBL, MESH_BED_LEVELING)
  #define HAS_MESH 1
#endif
#if ANY(AUTO_BED_LEVELING_UBL, AUTO_BED_LEVELING_3POINT)
  #define NEEDS_THREE_PROBE_POINTS 1
#endif
#if ANY(HAS_ABL_NOT_UBL, AUTO_BED_LEVELING_UBL)
  #define HAS_ABL_OR_UBL 1
  #if DISABLED(PROBE_MANUALLY)
    #define HAS_AUTOLEVEL 1
  #endif
#endif
#if ANY(HAS_ABL_OR_UBL, MESH_BED_LEVELING)
  #define HAS_LEVELING 1
  #if DISABLED(AUTO_BED_LEVELING_UBL)
    #define PLANNER_LEVELING 1
  #endif
#endif
#if !HAS_LEVELING
  #undef RESTORE_LEVELING_AFTER_G28
  #undef ENABLE_LEVELING_AFTER_G28
  #undef G29_RETRY_AND_RECOVER
#endif
#if !HAS_LEVELING || ANY(MESH_BED_LEVELING, AUTO_BED_LEVELING_UBL)
  #undef PROBE_MANUALLY
#endif
#if ANY(HAS_BED_PROBE, PROBE_MANUALLY, MESH_BED_LEVELING)
  #define PROBE_SELECTED 1
#endif

#ifdef GRID_MAX_POINTS_X
  #define GRID_MAX_POINTS ((GRID_MAX_POINTS_X) * (GRID_MAX_POINTS_Y))
  #define GRID_LOOP(A,B) for (uint8_t A = 0; A < GRID_MAX_POINTS_X; ++A) for (uint8_t B = 0; B < GRID_MAX_POINTS_Y; ++B)
#endif

// Slim menu optimizations
#if ENABLED(SLIM_LCD_MENUS)
  #define BOOT_MARLIN_LOGO_SMALL
#endif

// Flow and feedrate editing
#if HAS_EXTRUDERS && ANY(HAS_MARLINUI_MENU, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN)
  #define HAS_FLOW_EDIT 1
#endif
#if ANY(HAS_MARLINUI_MENU, ULTIPANEL_FEEDMULTIPLY, DWIN_CREALITY_LCD, DWIN_LCD_PROUI, MALYAN_LCD, TOUCH_SCREEN)
  #define HAS_FEEDRATE_EDIT 1
#endif

/**
 * CoreXY, CoreXZ, and CoreYZ - and their reverse
 */
#if ANY(COREXY, COREYX)
  #define CORE_IS_XY 1
#endif
#if ANY(COREXZ, COREZX)
  #define CORE_IS_XZ 1
#endif
#if ANY(COREYZ, COREZY)
  #define CORE_IS_YZ 1
#endif
#if CORE_IS_XY || CORE_IS_XZ || CORE_IS_YZ
  #define IS_CORE 1
#endif
#if IS_CORE
  #if CORE_IS_XY
    #define CORE_AXIS_1 A_AXIS
    #define CORE_AXIS_2 B_AXIS
    #define NORMAL_AXIS Z_AXIS
  #elif CORE_IS_XZ
    #define CORE_AXIS_1 A_AXIS
    #define NORMAL_AXIS Y_AXIS
    #define CORE_AXIS_2 C_AXIS
  #elif CORE_IS_YZ
    #define NORMAL_AXIS X_AXIS
    #define CORE_AXIS_1 B_AXIS
    #define CORE_AXIS_2 C_AXIS
  #endif
  #define CORESIGN(n) (ANY(COREYX, COREZX, COREZY) ? (-(n)) : (n))
#elif ANY(MARKFORGED_XY, MARKFORGED_YX)
  // Markforged kinematics
  #define CORE_AXIS_1 A_AXIS
  #define CORE_AXIS_2 B_AXIS
  #define NORMAL_AXIS Z_AXIS
#endif

#if ANY(MORGAN_SCARA, MP_SCARA, AXEL_TPARA)
  #define IS_SCARA 1
  #define IS_KINEMATIC 1
#elif ANY(DELTA, POLARGRAPH, POLAR)
  #define IS_KINEMATIC 1
#else
  #define IS_CARTESIAN 1
  #if !IS_CORE
    #define IS_FULL_CARTESIAN 1
  #endif
#endif

#if DISABLED(DELTA)
  #undef DELTA_HOME_TO_SAFE_ZONE
#endif

//
// Serial Port Info
//
#ifdef SERIAL_PORT_2
  #define HAS_MULTI_SERIAL 1
  #ifdef SERIAL_PORT_3
    #define NUM_SERIAL 3
  #else
    #define NUM_SERIAL 2
  #endif
#elif defined(SERIAL_PORT)
  #define NUM_SERIAL 1
#else
  #define NUM_SERIAL 0
  #undef BAUD_RATE_GCODE
#endif
#if SERIAL_PORT == -1 || SERIAL_PORT_2 == -1 || SERIAL_PORT_3 == -1
  #define HAS_USB_SERIAL 1
#endif
#if SERIAL_PORT_2 == -2
  #define HAS_ETHERNET 1
#endif

/**
 * This setting is also used by M109 when trying to calculate
 * a ballpark safe margin to prevent wait-forever situation.
 */
#ifndef EXTRUDE_MINTEMP
  #define EXTRUDE_MINTEMP 170
#endif

#if ANY(PID_DEBUG, PID_BED_DEBUG, PID_CHAMBER_DEBUG)
  #define HAS_PID_DEBUG 1
#endif

#if DISABLED(MPC_AUTOTUNE)
  #undef MPC_AUTOTUNE_MENU
#endif

/**
 * TFT Displays
 *
 * Configure parameters for TFT displays:
 *  - TFT_DEFAULT_ORIENTATION
 *  - TFT_DRIVER
 *  - TFT_WIDTH
 *  - TFT_HEIGHT
 *  - TFT_INTERFACE_(SPI|FSMC)
 *  - TFT_COLOR
 *  - GRAPHICAL_TFT_UPSCALE
 */
#if ANY(MKS_TS35_V2_0, BTT_TFT35_SPI_V1_0)                                    // ST7796
  #define TFT_DEFAULT_DRIVER ST7796
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_480x320
  #define TFT_INTERFACE_SPI
  #define NO_LCD_SDCARD
#elif ANY(LERDGE_TFT35, ANET_ET5_TFT35)                                       // ST7796
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_480x320
  #define TFT_INTERFACE_FSMC
#elif ANY(ANET_ET4_TFT28, MKS_ROBIN_TFT24, MKS_ROBIN_TFT28, MKS_ROBIN_TFT32)  // ST7789
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_Y)
  #define TFT_RES_320x240
  #define TFT_INTERFACE_FSMC
#elif ANY(MKS_ROBIN_TFT35, TFT_TRONXY_X5SA, ANYCUBIC_TFT35)                   // ILI9488
  #define TFT_DRIVER ILI9488
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #define TFT_RES_480x320
  #define TFT_INTERFACE_FSMC
#elif ENABLED(MKS_ROBIN_TFT43)
  #define TFT_DRIVER SSD1963
  #define TFT_DEFAULT_ORIENTATION 0
  #define TFT_RES_480x272
  #define TFT_INTERFACE_FSMC
#elif ANY(MKS_ROBIN_TFT_V1_1R, LONGER_LK_TFT28)                               // ILI9328 or R61505
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #define TFT_RES_320x240
  #define TFT_INTERFACE_FSMC
#elif ENABLED(BIQU_BX_TFT70)                                                  // RGB
  #define TFT_DEFAULT_ORIENTATION TFT_EXCHANGE_XY
  #define TFT_RES_1024x600
  #define TFT_INTERFACE_LTDC
  #if ENABLED(TOUCH_SCREEN)
    #define TFT_TOUCH_DEVICE_GT911
  #endif
#elif ENABLED(TFT_GENERIC)
  #define TFT_DEFAULT_ORIENTATION (TFT_EXCHANGE_XY | TFT_INVERT_X | TFT_INVERT_Y)
  #if NONE(TFT_RES_320x240, TFT_RES_480x272, TFT_RES_480x320, TFT_RES_1024x600)
    #define TFT_RES_320x240
  #endif
  #if NONE(TFT_INTERFACE_FSMC, TFT_INTERFACE_SPI)
    #define TFT_INTERFACE_SPI
  #endif
#endif

// FSMC/SPI TFT Panels using standard HAL/tft/tft_(fsmc|spi|ltdc).h
#if ENABLED(TFT_INTERFACE_FSMC)
  #define HAS_FSMC_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_FSMC_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_FSMC 1
  #endif
#elif ENABLED(TFT_INTERFACE_SPI)
  #define HAS_SPI_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_SPI_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_SPI 1
  #endif
#elif ENABLED(TFT_INTERFACE_LTDC)
  #define HAS_LTDC_TFT 1
  #if TFT_SCALED_DOGLCD
    #define HAS_LTDC_GRAPHICAL_TFT 1
  #elif HAS_TFT_LVGL_UI
    #define HAS_TFT_LVGL_UI_LTDC 1
  #endif
#endif

#if ENABLED(EXTUI_LVGL_TEST) && ANY(HAS_FSMC_TFT, HAS_SPI_TFT, SDL_CONTROLLER)
  #define HAS_LVGL 1
#endif

#if ANY(HAS_SPI_TFT, HAS_FSMC_TFT, HAS_LTDC_TFT)
  #include "../lcd/tft_io/tft_orientation.h" // for TFT_COLOR_UI_PORTRAIT
#endif

#if ENABLED(TFT_RES_320x240)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define TFT_WIDTH  240
    #define TFT_HEIGHT 320
  #else
    #define TFT_WIDTH  320
    #define TFT_HEIGHT 240
  #endif
  #define GRAPHICAL_TFT_UPSCALE 2
#elif ENABLED(TFT_RES_480x272)
  #define TFT_WIDTH  480
  #define TFT_HEIGHT 272
  #define GRAPHICAL_TFT_UPSCALE 2
#elif ENABLED(TFT_RES_480x320)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define TFT_WIDTH  320
    #define TFT_HEIGHT 480
  #else
    #define TFT_WIDTH  480
    #define TFT_HEIGHT 320
  #endif
  #define GRAPHICAL_TFT_UPSCALE 3
#elif ENABLED(TFT_RES_1024x600)
  #define TFT_WIDTH  1024
  #define TFT_HEIGHT 600
  #if ENABLED(TOUCH_SCREEN)
    #define GRAPHICAL_TFT_UPSCALE 6
    #define TFT_PIXEL_OFFSET_X 120
  #else
    #define GRAPHICAL_TFT_UPSCALE 8
    #define TFT_PIXEL_OFFSET_X 0
  #endif
#endif

#if ENABLED(TFT_COLOR_UI)
  #if (TFT_WIDTH == 320 && TFT_HEIGHT == 240) || (TFT_WIDTH == 240 && TFT_HEIGHT == 320)
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_320x240_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_320x240
    #endif
  #elif TFT_HEIGHT == 320 || (TFT_HEIGHT == 480 && ENABLED(TFT_COLOR_UI_PORTRAIT))
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_480x320_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_480x320
    #endif
  #elif TFT_HEIGHT == 272
    #if ENABLED(TFT_INTERFACE_SPI)
      #define TFT_480x272_SPI
    #elif ENABLED(TFT_INTERFACE_FSMC)
      #define TFT_480x272
    #endif
  #elif TFT_HEIGHT == 600
    #if ENABLED(TFT_INTERFACE_LTDC)
      #define TFT_1024x600_LTDC
    #else
      #define TFT_1024x600_SIM  // "Simulation" - for testing purposes only
    #endif
  #endif
#endif

#if ANY(TFT_320x240, TFT_320x240_SPI)
  #define HAS_UI_320x240 1
#elif ANY(TFT_480x320, TFT_480x320_SPI)
  #define HAS_UI_480x320 1
#elif ANY(TFT_480x272, TFT_480x272_SPI)
  #define HAS_UI_480x272 1
#elif ANY(TFT_1024x600_LTDC, TFT_1024x600_SIM)
  #define HAS_UI_1024x600 1
#endif

// Number of text lines the screen can display (may depend on font used)
// Touch screens leave space for extra buttons at the bottom
#if ANY(HAS_UI_320x240, HAS_UI_480x272)
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 8, 9)
  #else
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 6, 7)
  #endif
#elif HAS_UI_480x320
  #if ENABLED(TFT_COLOR_UI_PORTRAIT)
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 9, 10)
  #else
    #define LCD_HEIGHT TERN(TOUCH_SCREEN, 6, 7)
  #endif
#elif HAS_UI_1024x600
  #define LCD_HEIGHT TERN(TOUCH_SCREEN, 12, 13)
#endif

// This emulated DOGM has 'touch/xpt2046', not 'tft/xpt2046'
#if ENABLED(TOUCH_SCREEN)
  #if TOUCH_IDLE_SLEEP_MINS
    #define HAS_TOUCH_SLEEP 1
  #endif
  #if NONE(TFT_TOUCH_DEVICE_GT911, TFT_TOUCH_DEVICE_XPT2046)
    #define TFT_TOUCH_DEVICE_XPT2046          // ADS7843/XPT2046 ADC Touchscreen such as ILI9341 2.8
  #endif
  #if ENABLED(TFT_TOUCH_DEVICE_GT911)         // GT911 Capacitive touch screen such as BIQU_BX_TFT70
    #undef TOUCH_SCREEN_CALIBRATION
    #undef TOUCH_CALIBRATION_AUTO_SAVE
  #endif
  #if !HAS_GRAPHICAL_TFT
    #undef TOUCH_SCREEN
    #if ENABLED(TFT_CLASSIC_UI)
      #define HAS_TOUCH_BUTTONS 1
      #if ENABLED(TFT_TOUCH_DEVICE_GT911)
        #define HAS_CAP_TOUCH_BUTTONS 1
      #else
        #define HAS_RES_TOUCH_BUTTONS 1
      #endif
    #endif
  #endif
#endif

#if  (HAS_X_AXIS && X_HOME_DIR) || (HAS_Y_AXIS && Y_HOME_DIR) || (HAS_Z_AXIS && Z_HOME_DIR) \
  || (HAS_I_AXIS && I_HOME_DIR) || (HAS_J_AXIS && J_HOME_DIR) || (HAS_K_AXIS && K_HOME_DIR) \
  || (HAS_U_AXIS && U_HOME_DIR) || (HAS_V_AXIS && V_HOME_DIR) || (HAS_W_AXIS && W_HOME_DIR)
  #define HAS_ENDSTOPS 1
  #define COORDINATE_OKAY(N,L,H) WITHIN(N,L,H)
#else
  #define COORDINATE_OKAY(N,L,H) true
#endif

/**
 * LED Backlight INDEX END
 */
#if defined(NEOPIXEL_BKGD_INDEX_FIRST) && !defined(NEOPIXEL_BKGD_INDEX_LAST)
  #define NEOPIXEL_BKGD_INDEX_LAST NEOPIXEL_BKGD_INDEX_FIRST
#endif

#if ALL(SPI_FLASH, HAS_MEDIA, MARLIN_DEV_MODE)
  #define SPI_FLASH_BACKUP 1
#endif
