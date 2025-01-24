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
 * File: pins/pins.h
 *
 * Include pins definitions
 *
 * Pins numbering schemes:
 *
 *  - Digital I/O pin number if used by READ/WRITE macros. (e.g., X_STEP_DIR)
 *    The FastIO headers map digital pins to their ports and functions.
 *
 *  - Analog Input number if used by analogRead or DAC. (e.g., TEMP_n_PIN)
 *    These numbers are the same in any pin mapping.
 */

#define MAX_E_STEPPERS 8

#if NONE(FET_ORDER_EEF, FET_ORDER_EEB, FET_ORDER_EFF, FET_ORDER_EFB, FET_ORDER_SF)
  #if   MB(RAMPS_13_EFB, RAMPS_14_EFB, RAMPS_PLUS_EFB, RAMPS_14_RE_ARM_EFB, RAMPS_SMART_EFB, RAMPS_DUO_EFB, RAMPS4DUE_EFB, RAMPS_BTT_16_PLUS_EFB)
    #define FET_ORDER_EFB
  #elif MB(RAMPS_13_EEB, RAMPS_14_EEB, RAMPS_PLUS_EEB, RAMPS_14_RE_ARM_EEB, RAMPS_SMART_EEB, RAMPS_DUO_EEB, RAMPS4DUE_EEB, RAMPS_BTT_16_PLUS_EEB)
    #define FET_ORDER_EEB
  #elif MB(RAMPS_13_EFF, RAMPS_14_EFF, RAMPS_PLUS_EFF, RAMPS_14_RE_ARM_EFF, RAMPS_SMART_EFF, RAMPS_DUO_EFF, RAMPS4DUE_EFF, RAMPS_BTT_16_PLUS_EFF)
    #define FET_ORDER_EFF
  #elif MB(RAMPS_13_EEF, RAMPS_14_EEF, RAMPS_PLUS_EEF, RAMPS_14_RE_ARM_EEF, RAMPS_SMART_EEF, RAMPS_DUO_EEF, RAMPS4DUE_EEF, RAMPS_BTT_16_PLUS_EEF)
    #define FET_ORDER_EEF
  #elif MB(RAMPS_13_SF,  RAMPS_14_SF,  RAMPS_PLUS_SF,  RAMPS_14_RE_ARM_SF,  RAMPS_SMART_SF,  RAMPS_DUO_SF,  RAMPS4DUE_SF,  RAMPS_BTT_16_PLUS_SF)
    #define FET_ORDER_SF
  #elif ANY(HAS_MULTI_HOTEND, HEATERS_PARALLEL) || ALL(HAS_EXTRUDERS, HAS_CUTTER)
    #if TEMP_SENSOR_BED
      #define FET_ORDER_EEB
    #else
      #define FET_ORDER_EEF
    #endif
  #elif TEMP_SENSOR_BED
    #define FET_ORDER_EFB
  #else
    #define FET_ORDER_EFF
  #endif
#endif

#if !(ALL(HAS_WIRED_LCD, IS_NEWPANEL) && ANY(PANEL_ONE, VIKI2, miniVIKI, WYH_L12864, MINIPANEL, REPRAPWORLD_KEYPAD))
  #define HAS_FREE_AUX2_PINS 1
#endif

//
// Check for additional used endstop pins
//
#ifndef X_MIN_PIN
  #define X_MIN_PIN 1001
#endif
#ifndef Y_MIN_PIN
  #define Y_MIN_PIN 1002
#endif
#ifndef Z_MIN_PIN
  #define Z_MIN_PIN 1003
#endif
#ifndef X_MAX_PIN
  #define X_MAX_PIN 1004
#endif
#ifndef Y_MAX_PIN
  #define Y_MAX_PIN 1005
#endif
#ifndef Z_MAX_PIN
  #define Z_MAX_PIN 1006
#endif
#define _ENDSTOP_IS_ANY(P) (HAS_EXTRA_ENDSTOPS && (X2_STOP_PIN == P || Y2_STOP_PIN == P || Z2_STOP_PIN == P || Z3_STOP_PIN == P || Z4_STOP_PIN == P))
#if ENABLED(DUAL_X_CARRIAGE) || _ENDSTOP_IS_ANY(X_MIN_PIN) || _ENDSTOP_IS_ANY(X_MAX_PIN)
  #define NEEDS_X_MINMAX 1
#endif
#if _ENDSTOP_IS_ANY(Y_MIN_PIN) || _ENDSTOP_IS_ANY(Y_MAX_PIN)
  #define NEEDS_Y_MINMAX 1
#endif
#if _ENDSTOP_IS_ANY(Z_MIN_PIN) || _ENDSTOP_IS_ANY(Z_MAX_PIN) || ALL(Z_HOME_TO_MAX, Z_MIN_PROBE_USES_Z_MIN_ENDSTOP_PIN)
  #define NEEDS_Z_MINMAX 1
#endif
#undef _ENDSTOP_IS_ANY
#if X_MIN_PIN > 1000
  #undef X_MIN_PIN
#endif
#if Y_MIN_PIN > 1000
  #undef Y_MIN_PIN
#endif
#if Z_MIN_PIN > 1000
  #undef Z_MIN_PIN
#endif
#if X_MAX_PIN > 1000
  #undef X_MAX_PIN
#endif
#if Y_MAX_PIN > 1000
  #undef Y_MAX_PIN
#endif
#if Z_MAX_PIN > 1000
  #undef Z_MAX_PIN
#endif

// Test the target within the included pins file
#ifdef __MARLIN_DEPS__
  #define NOT_TARGET(V...) 0
#else
  #define NOT_TARGET NONE
#endif

#ifndef NO_CONTROLLER_CUSTOM_WIRING_WARNING
  #define CONTROLLER_WARNING(PF,CN,V...) static_assert(false, "\n\nWARNING! " CN " requires wiring modification! See pins_" PF ".h for details." V "\n (Define NO_CONTROLLER_CUSTOM_WIRING_WARNING to suppress this warning.)\n\n");
#else
  #define CONTROLLER_WARNING(...)
#endif

//
// RAMPS 1.3 / 1.4 / 1.6+ - ATmega1280, ATmega2560
//

#if MB(RAMPS_OLD)
  #include "ramps/pins_RAMPS_OLD.h"                 // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RAMPS_13_EFB, RAMPS_13_EEB, RAMPS_13_EFF, RAMPS_13_EEF, RAMPS_13_SF)
  #include "ramps/pins_RAMPS_13.h"                  // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RAMPS_14_EFB, RAMPS_14_EEB, RAMPS_14_EFF, RAMPS_14_EEF, RAMPS_14_SF)
  #include "ramps/pins_RAMPS.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RAMPS_PLUS_EFB, RAMPS_PLUS_EEB, RAMPS_PLUS_EFF, RAMPS_PLUS_EEF, RAMPS_PLUS_SF)
  #include "ramps/pins_RAMPS_PLUS.h"                // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RAMPS_BTT_16_PLUS_EFB, RAMPS_BTT_16_PLUS_EEB, RAMPS_BTT_16_PLUS_EFF, RAMPS_BTT_16_PLUS_EEF, RAMPS_BTT_16_PLUS_SF)
  #include "ramps/pins_RAMPS_BTT_16_PLUS.h"         // ATmega2560, ATmega1280               env:mega2560 env:mega1280

//
// RAMPS Derivatives - ATmega1280, ATmega2560
//

#elif MB(3DRAG)
  #include "ramps/pins_3DRAG.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(K8200)
  #include "ramps/pins_K8200.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(K8400)
  #include "ramps/pins_K8400.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(K8600)
  #include "ramps/pins_K8600.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(K8800)
  #include "ramps/pins_K8800.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(BAM_DICE)
  #include "ramps/pins_RAMPS.h"                     // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(BAM_DICE_DUE)
  #include "ramps/pins_BAM_DICE_DUE.h"              // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(MKS_BASE)
  #include "ramps/pins_MKS_BASE_10.h"               // ATmega2560                           env:mega2560
#elif MB(MKS_BASE_14)
  #include "ramps/pins_MKS_BASE_14.h"               // ATmega2560                           env:mega2560
#elif MB(MKS_BASE_15)
  #include "ramps/pins_MKS_BASE_15.h"               // ATmega2560                           env:mega2560
#elif MB(MKS_BASE_16)
  #include "ramps/pins_MKS_BASE_16.h"               // ATmega2560                           env:mega2560
#elif MB(MKS_BASE_HEROIC)
  #include "ramps/pins_MKS_BASE_HEROIC.h"           // ATmega2560                           env:mega2560
#elif MB(MKS_GEN_13)
  #include "ramps/pins_MKS_GEN_13.h"                // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(MKS_GEN_L)
  #include "ramps/pins_MKS_GEN_L.h"                 // ATmega2560                           env:mega2560
#elif MB(KFB_2)
  #include "ramps/pins_BIQU_KFB_2.h"                // ATmega2560                           env:mega2560
#elif MB(ZRIB_V20)
  #include "ramps/pins_ZRIB_V20.h"                  // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(ZRIB_V52)
  #include "ramps/pins_ZRIB_V52.h"                  // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(ZRIB_V53)
  #include "ramps/pins_ZRIB_V53.h"                  // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(FELIX2)
  #include "ramps/pins_FELIX2.h"                    // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RIGIDBOARD)
  #include "ramps/pins_RIGIDBOARD.h"                // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(RIGIDBOARD_V2)
  #include "ramps/pins_RIGIDBOARD_V2.h"             // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(SAINSMART_2IN1)
  #include "ramps/pins_SAINSMART_2IN1.h"            // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(ULTIMAKER)
  #include "ramps/pins_ULTIMAKER.h"                 // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(ULTIMAKER_OLD)
  #include "ramps/pins_ULTIMAKER_OLD.h"             // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(AZTEEG_X3)
  #include "ramps/pins_AZTEEG_X3.h"                 // ATmega2560                           env:mega2560
#elif MB(AZTEEG_X3_PRO)
  #include "ramps/pins_AZTEEG_X3_PRO.h"             // ATmega2560                           env:mega2560
#elif MB(ULTIMAIN_2)
  #include "ramps/pins_ULTIMAIN_2.h"                // ATmega2560                           env:mega2560ext
#elif MB(RUMBA)
  #include "ramps/pins_RUMBA.h"                     // ATmega2560                           env:mega2560
#elif MB(RUMBA_RAISE3D)
  #include "ramps/pins_RUMBA_RAISE3D.h"             // ATmega2560                           env:mega2560
#elif MB(RL200)
  #include "ramps/pins_RL200.h"                     // ATmega2560                           env:mega2560
#elif MB(FORMBOT_TREX2PLUS)
  #include "ramps/pins_FORMBOT_TREX2PLUS.h"         // ATmega2560                           env:mega2560
#elif MB(FORMBOT_TREX3)
  #include "ramps/pins_FORMBOT_TREX3.h"             // ATmega2560                           env:mega2560
#elif MB(FORMBOT_RAPTOR)
  #include "ramps/pins_FORMBOT_RAPTOR.h"            // ATmega2560                           env:mega2560
#elif MB(FORMBOT_RAPTOR2)
  #include "ramps/pins_FORMBOT_RAPTOR2.h"           // ATmega2560                           env:mega2560
#elif MB(BQ_ZUM_MEGA_3D)
  #include "ramps/pins_BQ_ZUM_MEGA_3D.h"            // ATmega2560                           env:mega2560ext
#elif MB(MAKEBOARD_MINI)
  #include "ramps/pins_MAKEBOARD_MINI.h"            // ATmega2560                           env:mega2560
#elif MB(TRIGORILLA_13)
  #include "ramps/pins_TRIGORILLA_13.h"             // ATmega2560                           env:mega2560
#elif MB(TRIGORILLA_14, TRIGORILLA_14_11)
  #include "ramps/pins_TRIGORILLA_14.h"             // ATmega2560                           env:mega2560
#elif MB(RAMPS_ENDER_4)
  #include "ramps/pins_RAMPS_ENDER_4.h"             // ATmega2560                           env:mega2560
#elif MB(RAMPS_CREALITY)
  #include "ramps/pins_RAMPS_CREALITY.h"            // ATmega2560                           env:mega2560
#elif MB(DAGOMA_F5)
  #include "ramps/pins_DAGOMA_F5.h"                 // ATmega2560                           env:mega2560
#elif MB(DAGOMA_D6)
  #include "ramps/pins_DAGOMA_D6.h"                 // ATmega2560                           env:mega2560ext
#elif MB(FYSETC_F6_13)
  #include "ramps/pins_FYSETC_F6_13.h"              // ATmega2560                           env:FYSETC_F6
#elif MB(FYSETC_F6_14)
  #include "ramps/pins_FYSETC_F6_14.h"              // ATmega2560                           env:FYSETC_F6
#elif MB(DUPLICATOR_I3_PLUS)
  #include "ramps/pins_DUPLICATOR_I3_PLUS.h"        // ATmega2560                           env:mega2560
#elif MB(VORON)
  #include "ramps/pins_VORON.h"                     // ATmega2560                           env:mega2560
#elif MB(TRONXY_V3_1_0)
  #include "ramps/pins_TRONXY_V3_1_0.h"             // ATmega2560                           env:mega2560
#elif MB(Z_BOLT_X_SERIES)
  #include "ramps/pins_Z_BOLT_X_SERIES.h"           // ATmega2560                           env:mega2560
#elif MB(TT_OSCAR)
  #include "ramps/pins_TT_OSCAR.h"                  // ATmega2560                           env:mega2560
#elif MB(TANGO)
  #include "ramps/pins_TANGO.h"                     // ATmega2560                           env:mega2560
#elif MB(MKS_GEN_L_V2)
  #include "ramps/pins_MKS_GEN_L_V2.h"              // ATmega2560                           env:mega2560
#elif MB(MKS_GEN_L_V21)
  #include "ramps/pins_MKS_GEN_L_V21.h"             // ATmega2560                           env:mega2560
#elif MB(COPYMASTER_3D)
  #include "ramps/pins_COPYMASTER_3D.h"             // ATmega2560                           env:mega2560
#elif MB(ORTUR_4)
  #include "ramps/pins_ORTUR_4.h"                   // ATmega2560                           env:mega2560
#elif MB(TENLOG_D3_HERO)
  #include "ramps/pins_TENLOG_D3_HERO.h"            // ATmega2560                           env:mega2560
#elif MB(TENLOG_MB1_V23)
  #include "ramps/pins_TENLOG_MB1_V23.h"            // ATmega2560                           env:mega2560
#elif MB(RAMPS_S_12_EEFB, RAMPS_S_12_EEEB, RAMPS_S_12_EFFB)
  #include "ramps/pins_RAMPS_S_12.h"                // ATmega2560                           env:mega2560
#elif MB(LONGER3D_LK1_PRO, LONGER3D_LKx_PRO)
  #include "ramps/pins_LONGER3D_LKx_PRO.h"          // ATmega2560                           env:mega2560
#elif MB(PXMALION_CORE_I3)
  #include "ramps/pins_PXMALION_CORE_I3.h"          // ATmega2560                           env:mega2560
#elif MB(PANOWIN_CUTLASS)
  #include "ramps/pins_PANOWIN_CUTLASS.h"           // ATmega2560                           env:mega2560ext
#elif MB(KODAMA_BARDO)
  #include "ramps/pins_KODAMA_BARDO.h"              // ATmega2560                           env:mega2560ext
#elif MB(XTLW_MFF_V1)
  #include "ramps/pins_XTLW_MFF_V1.h"               // ATmega2560                           env:mega2560
#elif MB(XTLW_MFF_V2)
  #include "ramps/pins_XTLW_MFF_V2.h"               // ATmega2560                           env:mega2560

//
// RAMBo and derivatives
//

#elif MB(RAMBO)
  #include "rambo/pins_RAMBO.h"                     // ATmega2560                           env:rambo
#elif MB(MINIRAMBO, MINIRAMBO_10A)
  #include "rambo/pins_MINIRAMBO.h"                 // ATmega2560                           env:rambo
#elif MB(EINSY_RAMBO)
  #include "rambo/pins_EINSY_RAMBO.h"               // ATmega2560                           env:rambo
#elif MB(EINSY_RETRO)
  #include "rambo/pins_EINSY_RETRO.h"               // ATmega2560                           env:rambo
#elif MB(SCOOVO_X9H)
  #include "rambo/pins_SCOOVO_X9H.h"                // ATmega2560                           env:rambo
#elif MB(RAMBO_THINKERV2)
  #include "rambo/pins_RAMBO_THINKERV2.h"           // ATmega2560                           env:rambo

//
// Other ATmega1280, ATmega2560
//

#elif MB(CNCONTROLS_11)
  #include "mega/pins_CNCONTROLS_11.h"              // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(CNCONTROLS_12)
  #include "mega/pins_CNCONTROLS_12.h"              // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(CNCONTROLS_15)
  #include "mega/pins_CNCONTROLS_15.h"              // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(CHEAPTRONIC)
  #include "mega/pins_CHEAPTRONIC.h"                // ATmega2560                           env:mega2560
#elif MB(CHEAPTRONIC_V2)
  #include "mega/pins_CHEAPTRONICv2.h"              // ATmega2560                           env:mega2560
#elif MB(MIGHTYBOARD_REVE)
  #include "mega/pins_MIGHTYBOARD_REVE.h"           // ATmega2560, ATmega1280               env:mega2560ext env:MightyBoard1280 env:MightyBoard2560
#elif MB(MEGATRONICS)
  #include "mega/pins_MEGATRONICS.h"                // ATmega2560                           env:mega2560
#elif MB(MEGATRONICS_2)
  #include "mega/pins_MEGATRONICS_2.h"              // ATmega2560                           env:mega2560
#elif MB(MEGATRONICS_3, MEGATRONICS_31, MEGATRONICS_32)
  #include "mega/pins_MEGATRONICS_3.h"              // ATmega2560                           env:mega2560
#elif MB(ELEFU_3)
  #include "mega/pins_ELEFU_3.h"                    // ATmega2560                           env:mega2560
#elif MB(LEAPFROG)
  #include "mega/pins_LEAPFROG.h"                   // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(MEGACONTROLLER)
  #include "mega/pins_MEGACONTROLLER.h"             // ATmega2560                           env:mega2560
#elif MB(GT2560_REV_A)
  #include "mega/pins_GT2560_REV_A.h"               // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(GT2560_REV_A_PLUS)
  #include "mega/pins_GT2560_REV_A_PLUS.h"          // ATmega2560, ATmega1280               env:mega2560 env:mega1280
#elif MB(GT2560_REV_B)
  #include "mega/pins_GT2560_REV_B.h"               // ATmega2560                           env:mega2560
#elif MB(GT2560_V3)
  #include "mega/pins_GT2560_V3.h"                  // ATmega2560                           env:mega2560
#elif MB(GT2560_V3_MC2)
  #include "mega/pins_GT2560_V3_MC2.h"              // ATmega2560                           env:mega2560
#elif MB(GT2560_V3_A20)
  #include "mega/pins_GT2560_V3_A20.h"              // ATmega2560                           env:mega2560
#elif MB(GT2560_V4)
  #include "mega/pins_GT2560_V4.h"                  // ATmega2560                           env:mega2560
#elif MB(GT2560_V4_A20)
  #include "mega/pins_GT2560_V4_A20.h"              // ATmega2560                           env:mega2560
#elif MB(GT2560_V41B)
  #include "mega/pins_GT2560_V41b.h"                // ATmega2560                           env:mega2560ext
#elif MB(EINSTART_S)
  #include "mega/pins_EINSTART-S.h"                 // ATmega2560, ATmega1280               env:mega2560ext env:mega1280
#elif MB(WANHAO_ONEPLUS)
  #include "mega/pins_WANHAO_ONEPLUS.h"             // ATmega2560                           env:mega2560
#elif MB(OVERLORD)
  #include "mega/pins_OVERLORD.h"                   // ATmega2560                           env:mega2560
#elif MB(HJC2560C_REV1)
  #include "mega/pins_HJC2560C_REV2.h"              // ATmega2560                           env:mega2560
#elif MB(HJC2560C_REV2)
  #include "mega/pins_HJC2560C_REV2.h"              // ATmega2560                           env:mega2560
#elif MB(LEAPFROG_XEED2015)
  #include "mega/pins_LEAPFROG_XEED2015.h"          // ATmega2560                           env:mega2560
#elif MB(PICA_REVB)
  #include "mega/pins_PICAOLD.h"                    // ATmega2560                           env:mega2560
#elif MB(PICA)
  #include "mega/pins_PICA.h"                       // ATmega2560                           env:mega2560
#elif MB(INTAMSYS40)
  #include "mega/pins_INTAMSYS40.h"                 // ATmega2560                           env:mega2560
#elif MB(MALYAN_M180)
  #include "mega/pins_MALYAN_M180.h"                // ATmega2560                           env:mega2560
#elif MB(PROTONEER_CNC_SHIELD_V3)
  #include "mega/pins_PROTONEER_CNC_SHIELD_V3.h"    // ATmega2560                           env:mega2560
#elif MB(WEEDO_62A)
  #include "mega/pins_WEEDO_62A.h"                  // ATmega2560                           env:mega2560

//
// ATmega1281, ATmega2561
//

#elif MB(MINITRONICS)
  #include "mega/pins_MINITRONICS.h"                // ATmega1281                           env:mega1281
#elif MB(SILVER_GATE)
  #include "mega/pins_SILVER_GATE.h"                // ATmega2561                           env:mega2560

//
// Sanguinololu and Derivatives - ATmega644P, ATmega1284P
//

#elif MB(SANGUINOLOLU_11)
  #include "sanguino/pins_SANGUINOLOLU_11.h"        // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(SANGUINOLOLU_12)
  #include "sanguino/pins_SANGUINOLOLU_12.h"        // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(MELZI)
  #include "sanguino/pins_MELZI.h"                  // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(MELZI_V2)
  #include "sanguino/pins_MELZI_V2.h"               // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(MELZI_MAKR3D)
  #include "sanguino/pins_MELZI_MAKR3D.h"           // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(MELZI_CREALITY)
  #include "sanguino/pins_MELZI_CREALITY.h"         // ATmega1284P                          env:melzi_optiboot_optimized env:melzi_optiboot env:melzi_optimized env:melzi
#elif MB(MELZI_CREALITY_ENDER2)
  #include "sanguino/pins_MELZI_CREALITY_E2.h"      // ATmega1284P                          env:melzi_optiboot_optimized env:melzi_optiboot env:melzi_optimized env:melzi
#elif MB(MELZI_MALYAN)
  #include "sanguino/pins_MELZI_MALYAN.h"           // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(MELZI_TRONXY)
  #include "sanguino/pins_MELZI_TRONXY.h"           // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(STB_11)
  #include "sanguino/pins_STB_11.h"                 // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(AZTEEG_X1)
  #include "sanguino/pins_AZTEEG_X1.h"              // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(ANET_10)
  #include "sanguino/pins_ANET_10.h"                // ATmega1284P                          env:sanguino1284p env:sanguino1284p_optimized env:melzi_optiboot
#elif MB(ZMIB_V2)
  #include "sanguino/pins_ZMIB_V2.h"                // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p

//
// Other ATmega644P, ATmega644, ATmega1284P
//

#elif MB(GEN3_MONOLITHIC)
  #include "sanguino/pins_GEN3_MONOLITHIC.h"        // ATmega644P                           env:sanguino644p
#elif MB(GEN3_PLUS)
  #include "sanguino/pins_GEN3_PLUS.h"              // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN6)
  #include "sanguino/pins_GEN6.h"                   // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN6_DELUXE)
  #include "sanguino/pins_GEN6_DELUXE.h"            // ATmega644P, ATmega1284P              env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN7_CUSTOM)
  #include "sanguino/pins_GEN7_CUSTOM.h"            // ATmega644P, ATmega644, ATmega1284P   env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN7_12)
  #include "sanguino/pins_GEN7_12.h"                // ATmega644P, ATmega644, ATmega1284P   env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN7_13)
  #include "sanguino/pins_GEN7_13.h"                // ATmega644P, ATmega644, ATmega1284P   env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(GEN7_14)
  #include "sanguino/pins_GEN7_14.h"                // ATmega644P, ATmega644, ATmega1284P   env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p
#elif MB(OMCA_A)
  #include "sanguino/pins_OMCA_A.h"                 // ATmega644                            env:sanguino644p
#elif MB(OMCA)
  #include "sanguino/pins_OMCA.h"                   // ATmega644P, ATmega644                env:sanguino644p
#elif MB(SETHI)
  #include "sanguino/pins_SETHI.h"                  // ATmega644P, ATmega644, ATmega1284P   env:sanguino1284p_optimized env:sanguino1284p env:sanguino644p

//
// Teensyduino - AT90USB1286, AT90USB1286P
//

#elif MB(TEENSYLU)
  #include "teensy2/pins_TEENSYLU.h"                // AT90USB1286, AT90USB1286P            env:at90usb1286_cdc
#elif MB(PRINTRBOARD)
  #include "teensy2/pins_PRINTRBOARD.h"             // AT90USB1286                          env:at90usb1286_dfu
#elif MB(PRINTRBOARD_REVF)
  #include "teensy2/pins_PRINTRBOARD_REVF.h"        // AT90USB1286                          env:at90usb1286_dfu
#elif MB(BRAINWAVE)
  #include "teensy2/pins_BRAINWAVE.h"               // AT90USB646                           env:at90usb1286_cdc
#elif MB(BRAINWAVE_PRO)
  #include "teensy2/pins_BRAINWAVE_PRO.h"           // AT90USB1286                          env:at90usb1286_cdc
#elif MB(SAV_MKI)
  #include "teensy2/pins_SAV_MKI.h"                 // AT90USB1286                          env:at90usb1286_cdc
#elif MB(TEENSY2)
  #include "teensy2/pins_TEENSY2.h"                 // AT90USB1286                          env:teensy20
#elif MB(5DPRINT)
  #include "teensy2/pins_5DPRINT.h"                 // AT90USB1286                          env:at90usb1286_dfu

//
// LPC1768 ARM Cortex-M3
//

#elif MB(RAMPS_14_RE_ARM_EFB, RAMPS_14_RE_ARM_EEB, RAMPS_14_RE_ARM_EFF, RAMPS_14_RE_ARM_EEF, RAMPS_14_RE_ARM_SF)
  #include "lpc1768/pins_RAMPS_RE_ARM.h"            // LPC1768                              env:LPC1768
#elif MB(MKS_SBASE)
  #include "lpc1768/pins_MKS_SBASE.h"               // LPC1768/9                            env:LPC1768 env:LPC1769
#elif MB(AZSMZ_MINI)
  #include "lpc1768/pins_AZSMZ_MINI.h"              // LPC1768                              env:LPC1768
#elif MB(BIQU_BQ111_A4)
  #include "lpc1768/pins_BIQU_BQ111_A4.h"           // LPC1768                              env:LPC1768
#elif MB(SELENA_COMPACT)
  #include "lpc1768/pins_SELENA_COMPACT.h"          // LPC1768                              env:LPC1768
#elif MB(BIQU_B300_V1_0)
  #include "lpc1768/pins_BIQU_B300_V1.0.h"          // LPC1768                              env:LPC1768
#elif MB(MKS_SGEN_L)
  #include "lpc1768/pins_MKS_SGEN_L.h"              // LPC1768                              env:LPC1768
#elif MB(GMARSH_X6_REV1)
  #include "lpc1768/pins_GMARSH_X6_REV1.h"          // LPC1768                              env:LPC1768
#elif MB(BTT_SKR_V1_1)
  #include "lpc1768/pins_BTT_SKR_V1_1.h"            // LPC1768                              env:LPC1768
#elif MB(BTT_SKR_V1_3)
  #include "lpc1768/pins_BTT_SKR_V1_3.h"            // LPC1768                              env:LPC1768
#elif MB(BTT_SKR_V1_4)
  #include "lpc1768/pins_BTT_SKR_V1_4.h"            // LPC1768                              env:LPC1768
#elif MB(EMOTRONIC)
  #include "lpc1768/pins_EMOTRONIC.h"               // LPC1768                              env:LPC1768

//
// LPC1769 ARM Cortex-M3
//

#elif MB(MKS_SGEN)
  #include "lpc1769/pins_MKS_SGEN.h"                // LPC1769                              env:LPC1769
#elif MB(AZTEEG_X5_GT)
  #include "lpc1769/pins_AZTEEG_X5_GT.h"            // LPC1769                              env:LPC1769
#elif MB(AZTEEG_X5_MINI)
  #include "lpc1769/pins_AZTEEG_X5_MINI.h"          // LPC1769                              env:LPC1769
#elif MB(AZTEEG_X5_MINI_WIFI)
  #include "lpc1769/pins_AZTEEG_X5_MINI_WIFI.h"     // LPC1769                              env:LPC1769
#elif MB(COHESION3D_REMIX)
  #include "lpc1769/pins_COHESION3D_REMIX.h"        // LPC1769                              env:LPC1769
#elif MB(COHESION3D_MINI)
  #include "lpc1769/pins_COHESION3D_MINI.h"         // LPC1769                              env:LPC1769
#elif MB(SMOOTHIEBOARD)
  #include "lpc1769/pins_SMOOTHIEBOARD.h"           // LPC1768/9                            env:LPC1768 env:LPC1769
#elif MB(TH3D_EZBOARD)
  #include "lpc1769/pins_TH3D_EZBOARD.h"            // LPC1769                              env:LPC1769
#elif MB(BTT_SKR_V1_4_TURBO)
  #include "lpc1769/pins_BTT_SKR_V1_4_TURBO.h"      // LPC1769                              env:LPC1769
#elif MB(MKS_SGEN_L_V2)
  #include "lpc1769/pins_MKS_SGEN_L_V2.h"           // LPC1769                              env:LPC1769
#elif MB(BTT_SKR_E3_TURBO)
  #include "lpc1769/pins_BTT_SKR_E3_TURBO.h"        // LPC1769                              env:LPC1769
#elif MB(FLY_CDY)
  #include "lpc1769/pins_FLY_CDY.h"                 // LPC1769                              env:LPC1769
#elif MB(XTLW_CLIMBER_8TH_LPC)
  #include "lpc1769/pins_XTLW_CLIMBER_8TH_LPC.h"    // LPC1769                              env:LPC1769

//
// SAM3X8E ARM Cortex-M3
//

#elif MB(DUE3DOM)
  #include "sam/pins_DUE3DOM.h"                     // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(DUE3DOM_MINI)
  #include "sam/pins_DUE3DOM_MINI.h"                // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RADDS)
  #include "sam/pins_RADDS.h"                       // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RAMPS_FD_V1)
  #include "sam/pins_RAMPS_FD_V1.h"                 // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RAMPS_FD_V2)
  #include "sam/pins_RAMPS_FD_V2.h"                 // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RAMPS_SMART_EFB, RAMPS_SMART_EEB, RAMPS_SMART_EFF, RAMPS_SMART_EEF, RAMPS_SMART_SF)
  #include "sam/pins_RAMPS_SMART.h"                 // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RAMPS_DUO_EFB, RAMPS_DUO_EEB, RAMPS_DUO_EFF, RAMPS_DUO_EEF, RAMPS_DUO_SF)
  #include "sam/pins_RAMPS_DUO.h"                   // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RAMPS4DUE_EFB, RAMPS4DUE_EEB, RAMPS4DUE_EFF, RAMPS4DUE_EEF, RAMPS4DUE_SF)
  #include "sam/pins_RAMPS4DUE.h"                   // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RURAMPS4D_11)
  #include "sam/pins_RURAMPS4D_11.h"                // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(RURAMPS4D_13)
  #include "sam/pins_RURAMPS4D_13.h"                // SAM3X8E                              env:DUE env:DUE_USB env:DUE_debug
#elif MB(ULTRATRONICS_PRO)
  #include "sam/pins_ULTRATRONICS_PRO.h"            // SAM3X8E                              env:DUE env:DUE_debug
#elif MB(ARCHIM1)
  #include "sam/pins_ARCHIM1.h"                     // SAM3X8E                              env:DUE_archim env:DUE_archim_debug
#elif MB(ARCHIM2)
  #include "sam/pins_ARCHIM2.h"                     // SAM3X8E                              env:DUE_archim env:DUE_archim_debug
#elif MB(ALLIGATOR)
  #include "sam/pins_ALLIGATOR_R2.h"                // SAM3X8E                              env:DUE env:DUE_debug
#elif MB(CNCONTROLS_15D)
  #include "sam/pins_CNCONTROLS_15D.h"              // SAM3X8E                              env:DUE env:DUE_USB
#elif MB(KRATOS32)
  #include "sam/pins_KRATOS32.h"                    // SAM3X8E                              env:DUE env:DUE_USB

//
// SAM3X8C ARM Cortex-M3
//

#elif MB(PRINTRBOARD_G2)
  #include "sam/pins_PRINTRBOARD_G2.h"              // SAM3X8C                              env:DUE_USB
#elif MB(ADSK)
  #include "sam/pins_ADSK.h"                        // SAM3X8C                              env:DUE env:DUE_debug

//
// STM32 ARM Cortex-M0+
//

#elif MB(BTT_EBB42_V1_1)
  #include "stm32g0/pins_BTT_EBB42_V1_1.h"          // STM32G0                              env:BTT_EBB42_V1_1_filament_extruder
#elif MB(BTT_SKR_MINI_E3_V3_0)
  #include "stm32g0/pins_BTT_SKR_MINI_E3_V3_0.h"    // STM32G0                              env:STM32G0B1RE_btt env:STM32G0B1RE_btt_xfer
#elif MB(BTT_MANTA_E3_EZ_V1_0)
  #include "stm32g0/pins_BTT_MANTA_E3_EZ_V1_0.h"    // STM32G0                              env:STM32G0B1RE_manta_btt
#elif MB(BTT_MANTA_M4P_V2_1)
  #include "stm32g0/pins_BTT_MANTA_M4P_V2_1.h"      // STM32G0                              env:STM32G0B1RE_manta_btt
#elif MB(BTT_MANTA_M5P_V1_0)
  #include "stm32g0/pins_BTT_MANTA_M5P_V1_0.h"      // STM32G0                              env:STM32G0B1RE_manta_btt
#elif MB(BTT_MANTA_M8P_V1_0)
  #include "stm32g0/pins_BTT_MANTA_M8P_V1_0.h"      // STM32G0                              env:STM32G0B1VE_btt
#elif MB(BTT_MANTA_M8P_V1_1)
  #include "stm32g0/pins_BTT_MANTA_M8P_V1_1.h"      // STM32G0                              env:STM32G0B1VE_btt
#elif MB(BTT_SKRAT_V1_0)
  #include "stm32g0/pins_BTT_SKRAT_V1_0.h"          // STM32G0                              env:STM32G0B1VE_btt env:STM32G0B1VE_btt_xfer

//
// STM32 ARM Cortex-M0
//

#elif MB(MALYAN_M200_V2)
  #include "stm32f0/pins_MALYAN_M200_V2.h"          // STM32F0                              env:STM32F070RB_malyan env:STM32F070CB_malyan
#elif MB(MALYAN_M300)
  #include "stm32f0/pins_MALYAN_M300.h"             // STM32F0                              env:malyan_M300

//
// STM32 ARM Cortex-M3
//

#elif MB(STM32F103RE)
  #include "stm32f1/pins_STM32F1R.h"                // STM32F1                              env:STM32F103RE env:STM32F103RE_maple
#elif MB(MALYAN_M200)
  #include "stm32f1/pins_MALYAN_M200.h"             // STM32F1                              env:STM32F103CB_malyan env:STM32F103CB_malyan_maple
#elif MB(STM3R_MINI)
  #include "stm32f1/pins_STM3R_MINI.h"              // STM32F1                              env:STM32F103VE env:STM32F103RE_maple
#elif MB(GTM32_PRO_VB)
  #include "stm32f1/pins_GTM32_PRO_VB.h"            // STM32F1                              env:STM32F103VE env:STM32F103VE_GTM32_maple
#elif MB(GTM32_PRO_VD)
  #include "stm32f1/pins_GTM32_PRO_VD.h"            // STM32F1                              env:STM32F103VE env:STM32F103VE_GTM32_maple
#elif MB(GTM32_MINI)
  #include "stm32f1/pins_GTM32_MINI.h"              // STM32F1                              env:STM32F103VE env:STM32F103VE_GTM32_maple
#elif MB(GTM32_MINI_A30)
  #include "stm32f1/pins_GTM32_MINI_A30.h"          // STM32F1                              env:STM32F103VE env:STM32F103VE_GTM32_maple
#elif MB(GTM32_REV_B)
  #include "stm32f1/pins_GTM32_REV_B.h"             // STM32F1                              env:STM32F103VE env:STM32F103VE_GTM32_maple
#elif MB(MORPHEUS)
  #include "stm32f1/pins_MORPHEUS.h"                // STM32F1                              env:STM32F103RE env:STM32F103RE_maple
#elif MB(CHITU3D)
  #include "stm32f1/pins_CHITU3D.h"                 // STM32F1                              env:STM32F103ZE env:STM32F103RE_maple
#elif MB(MKS_ROBIN)
  #include "stm32f1/pins_MKS_ROBIN.h"               // STM32F1                              env:mks_robin env:mks_robin_maple
#elif MB(MKS_ROBIN_MINI)
  #include "stm32f1/pins_MKS_ROBIN_MINI.h"          // STM32F1                              env:mks_robin_mini env:mks_robin_mini_maple
#elif MB(MKS_ROBIN_NANO)
  #include "stm32f1/pins_MKS_ROBIN_NANO.h"          // STM32F1                              env:mks_robin_nano_v1v2 env:mks_robin_nano_v1v2_maple env:mks_robin_nano_v1v2_usbmod
#elif MB(MKS_ROBIN_NANO_V2)
  #include "stm32f1/pins_MKS_ROBIN_NANO_V2.h"       // STM32F1                              env:mks_robin_nano_v1v2 env:mks_robin_nano_v1v2_maple
#elif MB(MKS_ROBIN_LITE)
  #include "stm32f1/pins_MKS_ROBIN_LITE.h"          // STM32F1                              env:mks_robin_lite env:mks_robin_lite_maple
#elif MB(MKS_ROBIN_LITE3)
  #include "stm32f1/pins_MKS_ROBIN_LITE3.h"         // STM32F1                              env:mks_robin_lite3 env:mks_robin_lite3_maple
#elif MB(MKS_ROBIN_PRO)
  #include "stm32f1/pins_MKS_ROBIN_PRO.h"           // STM32F1                              env:mks_robin_pro env:mks_robin_pro_maple
#elif MB(MKS_ROBIN_E3)
  #include "stm32f1/pins_MKS_ROBIN_E3.h"            // STM32F1                              env:mks_robin_e3 env:mks_robin_e3_maple
#elif MB(MKS_ROBIN_E3_V1_1)
  #include "stm32f1/pins_MKS_ROBIN_E3_V1_1.h"       // STM32F1                              env:mks_robin_e3
#elif MB(MKS_ROBIN_E3D)
  #include "stm32f1/pins_MKS_ROBIN_E3D.h"           // STM32F1                              env:mks_robin_e3
#elif MB(MKS_ROBIN_E3D_V1_1)
  #include "stm32f1/pins_MKS_ROBIN_E3D_V1_1.h"      // STM32F1                              env:mks_robin_e3 env:mks_robin_e3_maple
#elif MB(MKS_ROBIN_E3P)
  #include "stm32f1/pins_MKS_ROBIN_E3P.h"           // STM32F1                              env:mks_robin_e3p env:mks_robin_e3p_maple
#elif MB(BTT_SKR_MINI_V1_1)
  #include "stm32f1/pins_BTT_SKR_MINI_V1_1.h"       // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple
#elif MB(BTT_SKR_MINI_E3_V1_0)
  #include "stm32f1/pins_BTT_SKR_MINI_E3_V1_0.h"    // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple
#elif MB(BTT_SKR_MINI_E3_V1_2)
  #include "stm32f1/pins_BTT_SKR_MINI_E3_V1_2.h"    // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple
#elif MB(BTT_SKR_MINI_E3_V2_0)
  #include "stm32f1/pins_BTT_SKR_MINI_E3_V2_0.h"    // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RE_btt env:STM32F103RE_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple env:STM32F103RE_btt_maple env:STM32F103RE_btt_USB_maple
#elif MB(BTT_SKR_MINI_MZ_V1_0)
  #include "stm32f1/pins_BTT_SKR_MINI_MZ_V1_0.h"    // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple
#elif MB(BTT_SKR_E3_DIP)
  #include "stm32f1/pins_BTT_SKR_E3_DIP.h"          // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RE_btt env:STM32F103RE_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple env:STM32F103RE_btt_maple env:STM32F103RE_btt_USB_maple
#elif MB(BTT_SKR_CR6)
  #include "stm32f1/pins_BTT_SKR_CR6.h"             // STM32F1                              env:STM32F103RE_btt env:STM32F103RE_btt_USB env:STM32F103RE_btt_maple env:STM32F103RE_btt_USB_maple
#elif MB(JGAURORA_A5S_A1)
  #include "stm32f1/pins_JGAURORA_A5S_A1.h"         // STM32F1                              env:jgaurora_a5s_a1 env:jgaurora_a5s_a1_maple
#elif MB(FYSETC_AIO_II)
  #include "stm32f1/pins_FYSETC_AIO_II.h"           // STM32F1                              env:STM32F103RC_fysetc env:STM32F103RC_fysetc_maple
#elif MB(FYSETC_CHEETAH)
  #include "stm32f1/pins_FYSETC_CHEETAH.h"          // STM32F1                              env:STM32F103RC_fysetc env:STM32F103RC_fysetc_maple
#elif MB(FYSETC_CHEETAH_V12)
  #include "stm32f1/pins_FYSETC_CHEETAH_V12.h"      // STM32F1                              env:STM32F103RC_fysetc env:STM32F103RC_fysetc_maple
#elif MB(LONGER3D_LK)
  #include "stm32f1/pins_LONGER3D_LK.h"             // STM32F1                              env:STM32F103VE_longer env:STM32F103VE_longer_maple
#elif MB(CCROBOT_MEEB_3DP)
  #include "stm32f1/pins_CCROBOT_MEEB_3DP.h"        // STM32F1                              env:STM32F103RC_meeb_maple
#elif MB(CHITU3D_V5)
  #include "stm32f1/pins_CHITU3D_V5.h"              // STM32F1                              env:chitu_f103 env:chitu_f103_maple env:chitu_v5_gpio_init env:chitu_v5_gpio_init_maple
#elif MB(CHITU3D_V6)
  #include "stm32f1/pins_CHITU3D_V6.h"              // STM32F1                              env:chitu_f103 env:chitu_f103_maple
#elif MB(CHITU3D_V9)
  #include "stm32f1/pins_CHITU3D_V9.h"              // STM32F1                              env:chitu_f103 env:chitu_f103_maple
#elif MB(CREALITY_V4)
  #include "stm32f1/pins_CREALITY_V4.h"             // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V422)
  #include "stm32f1/pins_CREALITY_V422.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V423)
  #include "stm32f1/pins_CREALITY_V423.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer
#elif MB(CREALITY_V425)
  #include "stm32f1/pins_CREALITY_V425.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V427)
  #include "stm32f1/pins_CREALITY_V427.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V4210)
  #include "stm32f1/pins_CREALITY_V4210.h"          // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V431, CREALITY_V431_A, CREALITY_V431_B, CREALITY_V431_C, CREALITY_V431_D)
  #include "stm32f1/pins_CREALITY_V431.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V452)
  #include "stm32f1/pins_CREALITY_V452.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V453)
  #include "stm32f1/pins_CREALITY_V453.h"           // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V521)
  #include "stm32f1/pins_CREALITY_V521.h"           // STM32F1                              env:STM32F103VE_creality
#elif MB(CREALITY_V24S1)
  #include "stm32f1/pins_CREALITY_V24S1.h"          // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V24S1_301)
  #include "stm32f1/pins_CREALITY_V24S1_301.h"      // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_xfer env:STM32F103RC_creality env:STM32F103RC_creality_xfer env:STM32F103RE_creality_maple
#elif MB(CREALITY_V25S1)
  #include "stm32f1/pins_CREALITY_V25S1.h"          // STM32F1                              env:STM32F103RE_creality_smartPro env:STM32F103RE_creality_smartPro_maple
#elif MB(TRIGORILLA_PRO)
  #include "stm32f1/pins_TRIGORILLA_PRO.h"          // STM32F1                              env:trigorilla_pro env:trigorilla_pro_maple env:trigorilla_pro_disk
#elif MB(FLY_MINI)
  #include "stm32f1/pins_FLY_MINI.h"                // STM32F1                              env:FLY_MINI env:FLY_MINI_maple
#elif MB(FLSUN_HISPEED)
  #include "stm32f1/pins_FLSUN_HISPEED.h"           // STM32F1                              env:flsun_hispeedv1
#elif MB(BEAST)
  #include "stm32f1/pins_BEAST.h"                   // STM32F1                              env:STM32F103VE env:STM32F103RE_maple
#elif MB(MINGDA_MPX_ARM_MINI)
  #include "stm32f1/pins_MINGDA_MPX_ARM_MINI.h"     // STM32F1                              env:mingda_mpx_arm_mini
#elif MB(ZONESTAR_ZM3E2)
  #include "stm32f1/pins_ZM3E2_V1_0.h"              // STM32F1                              env:STM32F103RC_ZM3E2_USB env:STM32F103RC_ZM3E2_USB_maple
#elif MB(ZONESTAR_ZM3E4)
  #include "stm32f1/pins_ZM3E4_V1_0.h"              // STM32F1                              env:STM32F103VC_ZM3E4_USB env:STM32F103VC_ZM3E4_USB_maple
#elif MB(ZONESTAR_ZM3E4V2)
  #include "stm32f1/pins_ZM3E4_V2_0.h"              // STM32F1                              env:STM32F103VE_ZM3E4V2_USB env:STM32F103VE_ZM3E4V2_USB_maple
#elif MB(ERYONE_ERY32_MINI)
  #include "stm32f1/pins_ERYONE_ERY32_MINI.h"       // STM32F1                              env:ERYONE_ERY32_MINI_maple
#elif MB(PANDA_PI_V29)
  #include "stm32f1/pins_PANDA_PI_V29.h"            // STM32F1                              env:PANDA_PI_V29
#elif MB(SOVOL_V131)
  #include "gd32f1/pins_SOVOL_V131.h"               // GD32F1                               env:GD32F103RET6_sovol_maple
#elif MB(TRIGORILLA_V006)
  #include "gd32f1/pins_TRIGORILLA_V006.h"          // GD32F1                               env:trigorilla_v006
#elif MB(KEDI_CONTROLLER_V1_2)
  #include "stm32f1/pins_KEDI_CONTROLLER_V1_2.h"    // STM32F1                              env:STM32F103RC_btt env:STM32F103RC_btt_USB env:STM32F103RC_btt_maple env:STM32F103RC_btt_USB_maple
#elif MB(MD_D301)
  #include "stm32f1/pins_MD_D301.h"                 // STM32F1                              env:mingda_d301 env:mingda_d301_maple
#elif MB(VOXELAB_AQUILA)
  #include "gd32f1/pins_VOXELAB_AQUILA.h"           // GD32F1, N32G4, STM32F1               env:GD32F103RC_voxelab_maple env:N32G455RE_voxelab_maple env:STM32F103RE_creality_maple env:STM32F103RE_creality
#elif MB(SPRINGER_CONTROLLER)
  #include "stm32f1/pins_ORCA_3D_SPRINGER.h"        // STM32F1                              env:STM32F103VC_orca3d

//
// ARM Cortex-M4F
//

#elif MB(TEENSY31_32)
  #include "teensy3/pins_TEENSY31_32.h"             // TEENSY31_32                          env:teensy31
#elif MB(TEENSY35_36)
  #include "teensy3/pins_TEENSY35_36.h"             // TEENSY35_36                          env:teensy35 env:teensy36

//
// STM32 ARM Cortex-M4F
//

#elif MB(ARMED)
  #include "stm32f4/pins_ARMED.h"                   // STM32F4                              env:ARMED
#elif MB(RUMBA32_V1_0, RUMBA32_V1_1)
  #include "stm32f4/pins_RUMBA32_AUS3D.h"           // STM32F4                              env:rumba32
#elif MB(RUMBA32_MKS)
  #include "stm32f4/pins_RUMBA32_MKS.h"             // STM32F4                              env:rumba32
#elif MB(RUMBA32_BTT)
  #include "stm32f4/pins_RUMBA32_BTT.h"             // STM32F4                              env:rumba32
#elif MB(BLACK_STM32F407VE)
  #include "stm32f4/pins_BLACK_STM32F407VE.h"       // STM32F4                              env:STM32F407VE_black
#elif MB(BLACK_STM32F407ZE)
  #error "BLACK_STM32F407ZE is not yet supported."
#elif MB(BTT_SKR_MINI_E3_V3_0_1)
  #include "stm32f4/pins_BTT_SKR_MINI_E3_V3_0_1.h"  // STM32F4                              env:STM32F401RC_btt env:STM32F401RC_btt_xfer
#elif MB(BTT_SKR_PRO_V1_1)
  #include "stm32f4/pins_BTT_SKR_PRO_V1_1.h"        // STM32F4                              env:BTT_SKR_PRO env:BTT_SKR_PRO_usb_flash_drive
#elif MB(BTT_SKR_PRO_V1_2)
  #include "stm32f4/pins_BTT_SKR_PRO_V1_2.h"        // STM32F4                              env:BTT_SKR_PRO env:BTT_SKR_PRO_usb_flash_drive
#elif MB(BTT_BTT002_V1_0)
  #include "stm32f4/pins_BTT_BTT002_V1_0.h"         // STM32F4                              env:BTT_BTT002 env:BTT_BTT002_VET6
#elif MB(BTT_E3_RRF)
  #include "stm32f4/pins_BTT_E3_RRF.h"              // STM32F4                              env:BTT_E3_RRF
#elif MB(BTT_SKR_V2_0_REV_A)
  #include "stm32f4/pins_BTT_SKR_V2_0_REV_A.h"      // STM32F4                              env:STM32F407VG_btt env:STM32F407VG_btt_USB env:STM32F407VG_btt_USB_debug
#elif MB(BTT_SKR_V2_0_REV_B)
  #include "stm32f4/pins_BTT_SKR_V2_0_REV_B.h"      // STM32F4                              env:STM32F407VG_btt env:STM32F407VG_btt_USB env:STM32F407VG_btt_USB_debug env:STM32F429VG_btt env:STM32F429VG_btt_USB env:STM32F429VG_btt_USB_debug
#elif MB(BTT_GTR_V1_0)
  #include "stm32f4/pins_BTT_GTR_V1_0.h"            // STM32F4                              env:BTT_GTR_V1_0 env:BTT_GTR_V1_0_usb_flash_drive
#elif MB(BTT_OCTOPUS_V1_0)
  #include "stm32f4/pins_BTT_OCTOPUS_V1_0.h"        // STM32F4                              env:STM32F446ZE_btt env:STM32F446ZE_btt_usb_flash_drive
#elif MB(BTT_OCTOPUS_V1_1)
  #include "stm32f4/pins_BTT_OCTOPUS_V1_1.h"        // STM32F4                              env:STM32F446ZE_btt env:STM32F446ZE_btt_usb_flash_drive env:STM32F429ZG_btt env:STM32F429ZG_btt_usb_flash_drive env:STM32F407ZE_btt env:STM32F407ZE_btt_usb_flash_drive
#elif MB(BTT_OCTOPUS_PRO_V1_0)
  #include "stm32f4/pins_BTT_OCTOPUS_PRO_V1_0.h"    // STM32F4                              env:STM32F446ZE_btt env:STM32F446ZE_btt_usb_flash_drive env:STM32F429ZG_btt env:STM32F429ZG_btt_usb_flash_drive env:STM32H723ZE_btt
#elif MB(LERDGE_K)
  #include "stm32f4/pins_LERDGE_K.h"                // STM32F4                              env:LERDGEK env:LERDGEK_usb_flash_drive
#elif MB(LERDGE_S)
  #include "stm32f4/pins_LERDGE_S.h"                // STM32F4                              env:LERDGES env:LERDGES_usb_flash_drive
#elif MB(LERDGE_X)
  #include "stm32f4/pins_LERDGE_X.h"                // STM32F4                              env:LERDGEX env:LERDGEX_usb_flash_drive
#elif MB(FYSETC_S6)
  #include "stm32f4/pins_FYSETC_S6.h"               // STM32F4                              env:FYSETC_S6 env:FYSETC_S6_8000
#elif MB(FYSETC_S6_V2_0)
  #include "stm32f4/pins_FYSETC_S6_V2_0.h"          // STM32F4                              env:FYSETC_S6 env:FYSETC_S6_8000
#elif MB(FYSETC_SPIDER)
  #include "stm32f4/pins_FYSETC_SPIDER.h"           // STM32F4                              env:FYSETC_S6 env:FYSETC_S6_8000
#elif MB(FYSETC_SPIDER_V2_2)
  #include "stm32f4/pins_FYSETC_SPIDER_V2_2.h"      // STM32F4                              env:FYSETC_S6 env:FYSETC_S6_8000
#elif MB(FLYF407ZG)
  #include "stm32f4/pins_FLYF407ZG.h"               // STM32F4                              env:FLYF407ZG
#elif MB(MKS_ROBIN2)
  #include "stm32f4/pins_MKS_ROBIN2.h"              // STM32F4                              env:mks_robin2
#elif MB(MKS_ROBIN_PRO_V2)
  #include "stm32f4/pins_MKS_ROBIN_PRO_V2.h"        // STM32F4                              env:mks_robin_pro2
#elif MB(MKS_ROBIN_NANO_V3)
  #include "stm32f4/pins_MKS_ROBIN_NANO_V3.h"       // STM32F4                              env:mks_robin_nano_v3 env:mks_robin_nano_v3_usb_flash_drive env:mks_robin_nano_v3_usb_flash_drive_msc
#elif MB(MKS_ROBIN_NANO_V3_1)
  #include "stm32f4/pins_MKS_ROBIN_NANO_V3.h"       // STM32F4                              env:mks_robin_nano_v3_1 env:mks_robin_nano_v3_1_usb_flash_drive env:mks_robin_nano_v3_1_usb_flash_drive_msc
#elif MB(MKS_MONSTER8_V1)
  #include "stm32f4/pins_MKS_MONSTER8_V1.h"         // STM32F4                              env:mks_monster8 env:mks_monster8_usb_flash_drive env:mks_monster8_usb_flash_drive_msc
#elif MB(MKS_MONSTER8_V2)
  #include "stm32f4/pins_MKS_MONSTER8_V2.h"         // STM32F4                              env:mks_monster8 env:mks_monster8_usb_flash_drive env:mks_monster8_usb_flash_drive_msc
#elif MB(ANET_ET4)
  #include "stm32f4/pins_ANET_ET4.h"                // STM32F4                              env:Anet_ET4_no_bootloader env:Anet_ET4_OpenBLT
#elif MB(ANET_ET4P)
  #include "stm32f4/pins_ANET_ET4P.h"               // STM32F4                              env:Anet_ET4_no_bootloader env:Anet_ET4_OpenBLT
#elif MB(FYSETC_CHEETAH_V20)
  #include "stm32f4/pins_FYSETC_CHEETAH_V20.h"      // STM32F4                              env:FYSETC_CHEETAH_V20
#elif MB(FYSETC_CHEETAH_V30)
  #include "stm32f4/pins_FYSETC_CHEETAH_V30.h"      // STM32F4                              env:FYSETC_CHEETAH_V30
#elif MB(TH3D_EZBOARD_V2)
  #include "stm32f4/pins_TH3D_EZBOARD_V2.h"         // STM32F4                              env:TH3D_EZBoard_V2_no_bootloader env:TH3D_EZBoard_V2_OpenBLT
#elif MB(OPULO_LUMEN_REV3)
  #include "stm32f4/pins_OPULO_LUMEN_REV3.h"        // STM32F4                              env:Opulo_Lumen_REV3
#elif MB(OPULO_LUMEN_REV4)
  #include "stm32f4/pins_OPULO_LUMEN_REV4.h"        // STM32F4                              env:Opulo_Lumen_REV4
#elif MB(MKS_ROBIN_NANO_V1_3_F4)
  #include "stm32f4/pins_MKS_ROBIN_NANO_V1_3_F4.h"  // STM32F4                              env:mks_robin_nano_v1_3_f4 env:mks_robin_nano_v1_3_f4_usbmod
#elif MB(MKS_EAGLE)
  #include "stm32f4/pins_MKS_EAGLE.h"               // STM32F4                              env:mks_eagle env:mks_eagle_usb_flash_drive env:mks_eagle_usb_flash_drive_msc
#elif MB(ARTILLERY_RUBY)
  #include "stm32f4/pins_ARTILLERY_RUBY.h"          // STM32F4                              env:Artillery_Ruby
#elif MB(CREALITY_V24S1_301F4)
  #include "stm32f4/pins_CREALITY_V24S1_301F4.h"    // STM32F4                              env:STM32F401RC_creality env:STM32F401RC_creality_nobootloader env:STM32F401RC_creality_jlink env:STM32F401RC_creality_stlink
#elif MB(CREALITY_CR4NTXXC10)
  #include "stm32f4/pins_CREALITY_CR4NTXXC10.h"     // STM32F4                              env:STM32F401RE_freeruns env:STM32F401RE_freeruns_jlink env:STM32F401RE_freeruns_stlink
#elif MB(FYSETC_SPIDER_KING407)
  #include "stm32f4/pins_FYSETC_SPIDER_KING407.h"   // STM32F4                              env:FYSETC_SPIDER_KING407
#elif MB(MKS_SKIPR_V1)
  #include "stm32f4/pins_MKS_SKIPR_V1_0.h"          // STM32F4                              env:mks_skipr_v1 env:mks_skipr_v1_nobootloader
#elif MB(TRONXY_CXY_446_V10)
  #include "stm32f4/pins_TRONXY_CXY_446_V10.h"      // STM32F4                              env:TRONXY_CXY_446_V10 env:TRONXY_CXY_446_V10_usb_flash_drive
#elif MB(CREALITY_F401RE)
  #include "stm32f4/pins_CREALITY_F401.h"           // STM32F4                              env:STM32F401RE_creality
#elif MB(BLACKPILL_CUSTOM)
  #include "stm32f4/pins_BLACKPILL_CUSTOM.h"        // STM32F4                              env:STM32F401CD_blackpill_stlink
#elif MB(I3DBEEZ9_V1)
  #include "stm32f4/pins_I3DBEEZ9.h"                // STM32F4                              env:I3DBEEZ9_V1
#elif MB(MELLOW_FLY_E3_V2)
  #include "stm32f4/pins_MELLOW_FLY_E3_V2.h"        // STM32F4                              env:FLY_E3_V2
#elif MB(BLACKBEEZMINI_V1)
  #include "stm32f4/pins_BLACKBEEZMINI.h"           // STM32F4                              env:BLACKBEEZMINI_V1
#elif MB(XTLW_CLIMBER_8TH)
  #include "stm32f4/pins_XTLW_CLIMBER_8TH.h"        // STM32F4                              env:XTLW_CLIMBER_8TH
#elif MB(FLY_RRF_E3_V1)
  #include "stm32f4/pins_FLY_RRF_E3_V1.h"           // STM32F4                              env:FLY_RRF_E3_V1
#elif MB(ZNP_ROBIN_NANO)
  #include "stm32f1/pins_MKS_ROBIN_NANO.h"          // STM32F1                              env:znp_robin_nano35
#elif MB(ZNP_ROBIN_NANO_V1_3)
  #include "stm32f4/pins_MKS_ROBIN_NANO_V1_3_F4.h"  // STM32F4                              env:znp_robin_nano_v1_3
#elif MB(MKS_NEPTUNE_X)
  #include "stm32f4/pins_MKS_NEPTUNE_X.h"           // STM32F4                              env:mks_neptune_x
#elif MB(MKS_NEPTUNE_3)
  #include "stm32f4/pins_MKS_NEPTUNE_3.h"           // STM32F4                              env:mks_neptune_3

//
// Other ARM Cortex-M4
//

#elif MB(CREALITY_CR4NS)
  #include "stm32f1/pins_CREALITY_CR4NS.h"          // STM32F1                              env:STM32F103RE_creality env:STM32F103RE_creality_maple

//
// ARM Cortex-M7
//

#elif MB(REMRAM_V1)
  #include "stm32f7/pins_REMRAM_V1.h"               // STM32F7                              env:REMRAM_V1
#elif MB(NUCLEO_F767ZI)
  #include "stm32f7/pins_NUCLEO_F767ZI.h"           // STM32F7                              env:NUCLEO_F767ZI
#elif MB(BTT_SKR_SE_BX_V2)
  #include "stm32h7/pins_BTT_SKR_SE_BX_V2.h"        // STM32H7                              env:BTT_SKR_SE_BX
#elif MB(BTT_SKR_SE_BX_V3)
  #include "stm32h7/pins_BTT_SKR_SE_BX_V3.h"        // STM32H7                              env:BTT_SKR_SE_BX
#elif MB(BTT_SKR_V3_0)
  #include "stm32h7/pins_BTT_SKR_V3_0.h"            // STM32H7                              env:STM32H743VI_btt env:STM32H723VG_btt
#elif MB(BTT_SKR_V3_0_EZ)
  #include "stm32h7/pins_BTT_SKR_V3_0_EZ.h"         // STM32H7                              env:STM32H743VI_btt env:STM32H723VG_btt
#elif MB(BTT_OCTOPUS_MAX_EZ_V1_0)
  #include "stm32h7/pins_BTT_OCTOPUS_MAX_EZ.h"      // STM32H7                              env:STM32H723ZE_btt
#elif MB(BTT_OCTOPUS_PRO_V1_0_1)
  #include "stm32h7/pins_BTT_OCTOPUS_PRO_V1_0_1.h"  // STM32H7                              env:STM32H723ZE_btt
#elif MB(BTT_OCTOPUS_PRO_V1_1)
  #include "stm32h7/pins_BTT_OCTOPUS_PRO_V1_1.h"    // STM32H7                              env:STM32H723ZE_btt
#elif MB(BTT_MANTA_M8P_V2_0)
  #include "stm32h7/pins_BTT_MANTA_M8P_V2_0.h"      // STM32H7                              env:STM32H723ZE_btt
#elif MB(BTT_KRAKEN_V1_0)
  #include "stm32h7/pins_BTT_KRAKEN_V1_0.h"         // STM32H7                              env:STM32H723ZG_btt
#elif MB(TEENSY41)
  #include "teensy4/pins_TEENSY41.h"                // Teensy-4.x                           env:teensy41
#elif MB(T41U5XBB)
  #include "teensy4/pins_T41U5XBB.h"                // Teensy-4.x                           env:teensy41

//
// Espressif ESP32
//

#elif MB(ESPRESSIF_ESP32)
  #include "esp32/pins_ESP32.h"                     // ESP32                                env:esp32
#elif MB(MRR_ESPA)
  #include "esp32/pins_MRR_ESPA.h"                  // ESP32                                env:esp32
#elif MB(MRR_ESPE)
  #include "esp32/pins_MRR_ESPE.h"                  // ESP32                                env:esp32
#elif MB(E4D_BOX)
  #include "esp32/pins_E4D.h"                       // ESP32                                env:esp32
#elif MB(RESP32_CUSTOM)
  #include "esp32/pins_RESP32_CUSTOM.h"             // ESP32                                env:esp32
#elif MB(FYSETC_E4)
  #include "esp32/pins_FYSETC_E4.h"                 // ESP32                                env:FYSETC_E4
#elif MB(PANDA_ZHU)
  #include "esp32/pins_PANDA_ZHU.h"                 // ESP32                                env:PANDA
#elif MB(PANDA_M4)
  #include "esp32/pins_PANDA_M4.h"                  // ESP32                                env:PANDA
#elif MB(MKS_TINYBEE)
  #include "esp32/pins_MKS_TINYBEE.h"               // ESP32                                env:mks_tinybee
#elif MB(ENWI_ESPNP)
  #include "esp32/pins_ENWI_ESPNP.h"                // ESP32                                env:esp32
#elif MB(GODI_CONTROLLER_V1_0)
  #include "esp32/pins_GODI_CONTROLLER_V1_0.h"      // ESP32                                env:godi_esp32
#elif MB(MM_JOKER)
  #include "esp32/pins_MM_JOKER.h"                  // ESP32                                env:esp32

//
// Adafruit Grand Central M4 (SAMD51 ARM Cortex-M4)
//

#elif MB(AGCM4_RAMPS_144)
  #include "samd/pins_RAMPS_144.h"                  // SAMD51                               env:SAMD51_grandcentral_m4
#elif MB(BRICOLEMON_V1_0)
  #include "samd/pins_BRICOLEMON_V1_0.h"            // SAMD51                               env:SAMD51_grandcentral_m4
#elif MB(BRICOLEMON_LITE_V1_0)
  #include "samd/pins_BRICOLEMON_LITE_V1_0.h"       // SAMD51                               env:SAMD51_grandcentral_m4

//
// ReprapWorld Minitronics (SAMD21)
//

#elif MB(MINITRONICS20)
  #include "samd/pins_MINITRONICS20.h"              // SAMD21                               env:SAMD21_minitronics20

//
// HC32 ARM Cortex-M4
//

#elif MB(AQUILA_V101)
  #include "hc32f4/pins_AQUILA_101.h"               // HC32F460                             env:HC32F460C_aquila_101
#elif MB(CREALITY_ENDER2P_V24S4)
  #include "hc32f4/pins_CREALITY_ENDER2P_V24S4.h"   // HC32F460                             env:HC32F460C_e2p24s4

//
// Raspberry Pi RP2040
//

#elif MB(RP2040)
  #include "rp2040/pins_RP2040.h"                   // RP2040                               env:RP2040
#elif MB(BTT_SKR_PICO)
  #include "rp2040/pins_BTT_SKR_Pico.h"             // RP2040                               env:SKR_Pico env:SKR_Pico_UART

//
// Custom board (with custom PIO env)
//

#elif MB(CUSTOM)
  #include "pins_custom.h"                          //                                      env:custom

//
// Linux Native Debug board
//

#elif MB(SIMULATED)
  #include "native/pins_RAMPS_NATIVE.h"             // Native or Simulation                 lin:linux_native mac:simulator_macos_debug mac:simulator_macos_release win:simulator_windows lin:simulator_linux_debug lin:simulator_linux_release

#else

  //
  // Obsolete or unknown board
  //

  #define BOARD_MKS_13                  99900
  #define BOARD_TRIGORILLA              99901
  #define BOARD_RURAMPS4D               99902
  #define BOARD_FORMBOT_TREX2           99903
  #define BOARD_BIQU_SKR_V1_1           99904
  #define BOARD_STM32F1R                99905
  #define BOARD_STM32F103R              99906
  #define BOARD_ESP32                   99907
  #define BOARD_STEVAL                  99908
  #define BOARD_STEVAL_3DP001V1         99908
  #define BOARD_BIGTREE_SKR_V1_1        99909
  #define BOARD_BIGTREE_SKR_V1_3        99910
  #define BOARD_BIGTREE_SKR_V1_4        99911
  #define BOARD_BIGTREE_SKR_V1_4_TURBO  99912
  #define BOARD_BIGTREE_BTT002_V1_0     99913
  #define BOARD_BIGTREE_SKR_PRO_V1_1    99914
  #define BOARD_BIGTREE_SKR_MINI_V1_1   99915
  #define BOARD_BIGTREE_SKR_MINI_E3     99916
  #define BOARD_BIGTREE_SKR_E3_DIP      99917
  #define BOARD_RUMBA32                 99918
  #define BOARD_RUMBA32_AUS3D           99919
  #define BOARD_RAMPS_DAGOMA            99920
  #define BOARD_RAMPS_LONGER3D_LK4PRO   99921
  #define BOARD_BTT_SKR_V2_0            99922
  #define BOARD_TH3D_EZBOARD_LITE_V2    99923
  #define BOARD_BTT_SKR_SE_BX           99924
  #define BOARD_MKS_MONSTER8            99925
  #define BOARD_LINUX_RAMPS             99926
  #define BOARD_BTT_MANTA_M4P_V1_0      99927
  #define BOARD_VAKE403D                99928
  #define BOARD_TRONXY_V10              99929

  #if MB(MKS_13)
    #error "BOARD_MKS_13 is now BOARD_MKS_GEN_13. Please update your configuration."
  #elif MB(TRIGORILLA)
    #error "BOARD_TRIGORILLA is now BOARD_TRIGORILLA_13. Please update your configuration."
  #elif MB(RURAMPS4D)
    #error "BOARD_RURAMPS4D is now BOARD_RURAMPS4D_11. Please update your configuration."
  #elif MB(FORMBOT_TREX2)
    #error "FORMBOT_TREX2 is now BOARD_FORMBOT_TREX2PLUS. Please update your configuration."
  #elif MB(BIQU_SKR_V1_1)
    #error "BOARD_BIQU_SKR_V1_1 is now BOARD_BTT_SKR_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_1)
    #error "BOARD_BIGTREE_SKR_V1_1 is now BOARD_BTT_SKR_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_3)
    #error "BOARD_BIGTREE_SKR_V1_3 is now BOARD_BTT_SKR_V1_3. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_4)
    #error "BOARD_BIGTREE_SKR_V1_4 is now BOARD_BTT_SKR_V1_4. Please update your configuration."
  #elif MB(BIGTREE_SKR_V1_4_TURBO)
    #error "BOARD_BIGTREE_SKR_V1_4_TURBO is now BOARD_BTT_SKR_V1_4_TURBO. Please update your configuration."
  #elif MB(BIGTREE_BTT002_V1_0)
    #error "BOARD_BIGTREE_BTT002_V1_0 is now BOARD_BTT_BTT002_V1_0. Please update your configuration."
  #elif MB(BIGTREE_SKR_PRO_V1_1)
    #error "BOARD_BIGTREE_SKR_PRO_V1_1 is now BOARD_BTT_SKR_PRO_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_MINI_V1_1)
    #error "BOARD_BIGTREE_SKR_MINI_V1_1 is now BOARD_BTT_SKR_MINI_V1_1. Please update your configuration."
  #elif MB(BIGTREE_SKR_MINI_E3)
    #error "BOARD_BIGTREE_SKR_MINI_E3 is now BOARD_BTT_SKR_MINI_E3_V1_0. Please update your configuration."
  #elif MB(BIGTREE_SKR_E3_DIP)
    #error "BOARD_BIGTREE_SKR_E3_DIP is now BOARD_BTT_SKR_E3_DIP. Please update your configuration."
  #elif MB(STM32F1R)
    #error "BOARD_STM32F1R is now BOARD_STM32F103RE. Please update your configuration."
  #elif MB(STM32F103R)
    #error "BOARD_STM32F103R is now BOARD_STM32F103RE. Please update your configuration."
  #elif MOTHERBOARD == BOARD_ESP32
    #error "BOARD_ESP32 is now BOARD_ESPRESSIF_ESP32. Please update your configuration."
  #elif MB(STEVAL)
    #error "BOARD_STEVAL_3DP001V1 (BOARD_STEVAL) is no longer supported in Marlin."
  #elif MB(RUMBA32)
    #error "BOARD_RUMBA32 is now BOARD_RUMBA32_MKS or BOARD_RUMBA32_V1_0. Please update your configuration."
  #elif MB(RUMBA32_AUS3D)
    #error "BOARD_RUMBA32_AUS3D is now BOARD_RUMBA32_V1_0. Please update your configuration."
  #elif MB(RAMPS_DAGOMA)
    #error "BOARD_RAMPS_DAGOMA is now BOARD_DAGOMA_F5. Please update your configuration."
  #elif MB(RAMPS_LONGER3D_LK4PRO)
    #error "BOARD_RAMPS_LONGER3D_LK4PRO is now BOARD_LONGER3D_LKx_PRO. Please update your configuration."
  #elif MB(BTT_SKR_V2_0)
    #error "BOARD_BTT_SKR_V2_0 is now BOARD_BTT_SKR_V2_0_REV_A or BOARD_BTT_SKR_V2_0_REV_B. See https://bit.ly/3t5d9JQ for more information. Please update your configuration."
  #elif MB(TH3D_EZBOARD_LITE_V2)
    #error "BOARD_TH3D_EZBOARD_LITE_V2 is now BOARD_TH3D_EZBOARD_V2. Please update your configuration."
  #elif MB(BTT_SKR_SE_BX)
    #error "BOARD_BTT_SKR_SE_BX is now BOARD_BTT_SKR_SE_BX_V2 or BOARD_BTT_SKR_SE_BX_V3. Please update your configuration."
  #elif MB(MKS_MONSTER8)
    #error "BOARD_MKS_MONSTER8 is now BOARD_MKS_MONSTER8_V1 or BOARD_MKS_MONSTER8_V2. Please update your configuration."
  #elif MB(LINUX_RAMPS)
    #error "BOARD_LINUX_RAMPS is now BOARD_SIMULATED. Please update your configuration."
  #elif MB(BTT_MANTA_M4P_V1_0)
    #error "BOARD_BTT_MANTA_M4P_V1_0 is now BOARD_BTT_MANTA_M4P_V2_1. Please update your configuration."
  #elif MB(TRONXY_V10)
    #error "BOARD_TRONXY_V10 is now BOARD_TRONXY_CXY_446_V10. Please update your configuration."
  #elif MB(VAKE403D)
    #error "BOARD_VAKE403D is no longer supported in Marlin."
  #elif defined(MOTHERBOARD)
    #error "Unknown MOTHERBOARD value set in Configuration.h."
  #else
    #error "MOTHERBOARD not defined! Use '#define MOTHERBOARD BOARD_...' in Configuration.h."
  #endif

  #undef BOARD_MKS_13
  #undef BOARD_TRIGORILLA
  #undef BOARD_RURAMPS4D
  #undef BOARD_FORMBOT_TREX2
  #undef BOARD_BIQU_SKR_V1_1
  #undef BOARD_STM32F1R
  #undef BOARD_STM32F103R
  #undef BOARD_ESP32
  #undef BOARD_STEVAL
  #undef BOARD_STEVAL_3DP001V1
  #undef BOARD_BIGTREE_SKR_V1_1
  #undef BOARD_BIGTREE_SKR_V1_3
  #undef BOARD_BIGTREE_SKR_V1_4
  #undef BOARD_BIGTREE_SKR_V1_4_TURBO
  #undef BOARD_BIGTREE_BTT002_V1_0
  #undef BOARD_BIGTREE_SKR_PRO_V1_1
  #undef BOARD_BIGTREE_SKR_MINI_V1_1
  #undef BOARD_BIGTREE_SKR_MINI_E3
  #undef BOARD_BIGTREE_SKR_E3_DIP
  #undef BOARD_RUMBA32
  #undef BOARD_RUMBA32_AUS3D
  #undef BOARD_RAMPS_DAGOMA
  #undef BOARD_RAMPS_LONGER3D_LK4PRO
  #undef BOARD_BTT_SKR_V2_0
  #undef BOARD_TH3D_EZBOARD_LITE_V2
  #undef BOARD_BTT_SKR_SE_BX
  #undef BOARD_MKS_MONSTER8
  #undef BOARD_LINUX_RAMPS
  #undef BOARD_BTT_MANTA_M4P_V1_0
  #undef BOARD_VAKE403D
  #undef BOARD_TRONXY_V10

#endif

//
// LCD / Controller Pins based on board expansion headers with adapters
//
#include "pins_lcd.h"

//
// Post-process pins according to configured settings
//
#include "pins_postprocess.h"

// Cleanup
#undef CONTROLLER_WARNING
