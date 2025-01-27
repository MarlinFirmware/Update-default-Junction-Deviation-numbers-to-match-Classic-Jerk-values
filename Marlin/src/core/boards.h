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
 * Whenever changes are made to this file, please update Marlin/Makefile
 * and _data/boards.yml in the MarlinDocumentation repo.
 */

#include "macros.h"

#define BOARD_UNKNOWN -1

//
// RAMPS 1.3 / 1.4 - ATmega1280, ATmega2560
//

#define BOARD_RAMPS_OLD               1000  // MEGA/RAMPS up to 1.2

#define BOARD_RAMPS_13_EFB            1010  // RAMPS 1.3 (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_13_EEB            1011  // RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_13_EFF            1012  // RAMPS 1.3 (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_13_EEF            1013  // RAMPS 1.3 (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_13_SF             1014  // RAMPS 1.3 (Power outputs: Spindle, Controller Fan)

#define BOARD_RAMPS_14_EFB            1020  // RAMPS 1.4 (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_14_EEB            1021  // RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_14_EFF            1022  // RAMPS 1.4 (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_14_EEF            1023  // RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_14_SF             1024  // RAMPS 1.4 (Power outputs: Spindle, Controller Fan)

#define BOARD_RAMPS_PLUS_EFB          1030  // RAMPS Plus 3DYMY (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_PLUS_EEB          1031  // RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_PLUS_EFF          1032  // RAMPS Plus 3DYMY (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_PLUS_EEF          1033  // RAMPS Plus 3DYMY (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_PLUS_SF           1034  // RAMPS Plus 3DYMY (Power outputs: Spindle, Controller Fan)

#define BOARD_RAMPS_BTT_16_PLUS_EFB   1035  // RAMPS 1.6+ (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_BTT_16_PLUS_EEB   1036  // RAMPS 1.6+ (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_BTT_16_PLUS_EFF   1037  // RAMPS 1.6+ (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_BTT_16_PLUS_EEF   1038  // RAMPS 1.6+ (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_BTT_16_PLUS_SF    1039  // RAMPS 1.6+ (Power outputs: Spindle, Controller Fan)

//
// RAMPS Derivatives - ATmega1280, ATmega2560
//

#define BOARD_3DRAG                   1100  // 3Drag Controller
#define BOARD_K8200                   1101  // Velleman K8200 Controller (derived from 3Drag Controller)
#define BOARD_K8400                   1102  // Velleman K8400 Controller (derived from 3Drag Controller)
#define BOARD_K8600                   1103  // Velleman K8600 Controller (Vertex Nano)
#define BOARD_K8800                   1104  // Velleman K8800 Controller (Vertex Delta)
#define BOARD_BAM_DICE                1105  // 2PrintBeta BAM&DICE with STK drivers
#define BOARD_BAM_DICE_DUE            1106  // 2PrintBeta BAM&DICE Due with STK drivers
#define BOARD_MKS_BASE                1107  // MKS BASE v1.0
#define BOARD_MKS_BASE_14             1108  // MKS BASE v1.4 with Allegro A4982 stepper drivers
#define BOARD_MKS_BASE_15             1109  // MKS BASE v1.5 with Allegro A4982 stepper drivers
#define BOARD_MKS_BASE_16             1110  // MKS BASE v1.6 with Allegro A4982 stepper drivers
#define BOARD_MKS_BASE_HEROIC         1111  // MKS BASE 1.0 with Heroic HR4982 stepper drivers
#define BOARD_MKS_GEN_13              1112  // MKS GEN v1.3 or 1.4
#define BOARD_MKS_GEN_L               1113  // MKS GEN L
#define BOARD_KFB_2                   1114  // BigTreeTech or BIQU KFB2.0
#define BOARD_ZRIB_V20                1115  // Zonestar zrib V2.0 (Chinese RAMPS replica)
#define BOARD_ZRIB_V52                1116  // Zonestar zrib V5.2 (Chinese RAMPS replica)
#define BOARD_ZRIB_V53                1117  // Zonestar zrib V5.3 (Chinese RAMPS replica)
#define BOARD_FELIX2                  1118  // Felix 2.0+ Electronics Board (RAMPS like)
#define BOARD_RIGIDBOARD              1119  // Invent-A-Part RigidBoard
#define BOARD_RIGIDBOARD_V2           1120  // Invent-A-Part RigidBoard V2
#define BOARD_SAINSMART_2IN1          1121  // Sainsmart 2-in-1 board
#define BOARD_ULTIMAKER               1122  // Ultimaker
#define BOARD_ULTIMAKER_OLD           1123  // Ultimaker (Older electronics. Pre 1.5.4. This is rare)
#define BOARD_AZTEEG_X3               1124  // Azteeg X3
#define BOARD_AZTEEG_X3_PRO           1125  // Azteeg X3 Pro
#define BOARD_ULTIMAIN_2              1126  // Ultimainboard 2.x (Uses TEMP_SENSOR 20)
#define BOARD_RUMBA                   1127  // Rumba
#define BOARD_RUMBA_RAISE3D           1128  // Raise3D N series Rumba derivative
#define BOARD_RL200                   1129  // Rapide Lite 200 (v1, low-cost RUMBA clone with drv)
#define BOARD_FORMBOT_TREX2PLUS       1130  // Formbot T-Rex 2 Plus
#define BOARD_FORMBOT_TREX3           1131  // Formbot T-Rex 3
#define BOARD_FORMBOT_RAPTOR          1132  // Formbot Raptor
#define BOARD_FORMBOT_RAPTOR2         1133  // Formbot Raptor 2
#define BOARD_BQ_ZUM_MEGA_3D          1134  // bq ZUM Mega 3D
#define BOARD_MAKEBOARD_MINI          1135  // MakeBoard Mini v2.1.2 by MicroMake
#define BOARD_TRIGORILLA_13           1136  // TriGorilla Anycubic version 1.3-based on RAMPS EFB
#define BOARD_TRIGORILLA_14           1137  //   ... Ver 1.4
#define BOARD_TRIGORILLA_14_11        1138  //   ... Rev 1.1 (new servo pin order)
#define BOARD_RAMPS_ENDER_4           1139  // Creality: Ender-4, CR-8
#define BOARD_RAMPS_CREALITY          1140  // Creality: CR10S, CR20, CR-X
#define BOARD_DAGOMA_F5               1141  // Dagoma F5
#define BOARD_DAGOMA_D6               1142  // Dagoma D6 (as found in the Dagoma DiscoUltimate V2 TMC)
#define BOARD_FYSETC_F6_13            1143  // FYSETC F6 1.3
#define BOARD_FYSETC_F6_14            1144  // FYSETC F6 1.4
#define BOARD_DUPLICATOR_I3_PLUS      1145  // Wanhao Duplicator i3 Plus
#define BOARD_VORON                   1146  // VORON Design
#define BOARD_TRONXY_V3_1_0           1147  // Tronxy TRONXY-V3-1.0
#define BOARD_Z_BOLT_X_SERIES         1148  // Z-Bolt X Series
#define BOARD_TT_OSCAR                1149  // TT OSCAR
#define BOARD_TANGO                   1150  // BIQU Tango V1
#define BOARD_MKS_GEN_L_V2            1151  // MKS GEN L V2
#define BOARD_MKS_GEN_L_V21           1152  // MKS GEN L V2.1
#define BOARD_COPYMASTER_3D           1153  // Copymaster 3D
#define BOARD_ORTUR_4                 1154  // Ortur 4
#define BOARD_TENLOG_D3_HERO          1155  // Tenlog D3 Hero IDEX printer
#define BOARD_TENLOG_MB1_V23          1156  // Tenlog D3, D5, D6 IDEX Printer
#define BOARD_RAMPS_S_12_EEFB         1157  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Fan, Bed)
#define BOARD_RAMPS_S_12_EEEB         1158  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend0, Hotend1, Hotend2, Bed)
#define BOARD_RAMPS_S_12_EFFB         1159  // Ramps S 1.2 by Sakul.cz (Power outputs: Hotend, Fan0, Fan1, Bed)
#define BOARD_LONGER3D_LK1_PRO        1160  // Longer LK1 PRO / Alfawise U20 Pro (PRO version)
#define BOARD_LONGER3D_LKx_PRO        1161  // Longer LKx PRO / Alfawise Uxx Pro (PRO version)
#define BOARD_PXMALION_CORE_I3        1162  // Pxmalion Core I3
#define BOARD_PANOWIN_CUTLASS         1163  // Panowin Cutlass (as found in the Panowin F1)
#define BOARD_KODAMA_BARDO            1164  // Kodama Bardo V1.x (as found in the Kodama Trinus)
#define BOARD_XTLW_MFF_V1             1165  // XTLW MFF V1.0
#define BOARD_XTLW_MFF_V2             1166  // XTLW MFF V2.0

//
// RAMBo and derivatives
//

#define BOARD_RAMBO                   1200  // Rambo
#define BOARD_MINIRAMBO               1201  // Mini-Rambo
#define BOARD_MINIRAMBO_10A           1202  // Mini-Rambo 1.0a
#define BOARD_EINSY_RAMBO             1203  // Einsy Rambo
#define BOARD_EINSY_RETRO             1204  // Einsy Retro
#define BOARD_SCOOVO_X9H              1205  // abee Scoovo X9H
#define BOARD_RAMBO_THINKERV2         1206  // ThinkerV2

//
// Other ATmega1280, ATmega2560
//

#define BOARD_CNCONTROLS_11           1300  // Cartesio CN Controls V11
#define BOARD_CNCONTROLS_12           1301  // Cartesio CN Controls V12
#define BOARD_CNCONTROLS_15           1302  // Cartesio CN Controls V15
#define BOARD_CHEAPTRONIC             1303  // Cheaptronic v1.0
#define BOARD_CHEAPTRONIC_V2          1304  // Cheaptronic v2.0
#define BOARD_MIGHTYBOARD_REVE        1305  // Makerbot Mightyboard Revision E
#define BOARD_MEGATRONICS             1306  // Megatronics
#define BOARD_MEGATRONICS_2           1307  // Megatronics v2.0
#define BOARD_MEGATRONICS_3           1308  // Megatronics v3.0
#define BOARD_MEGATRONICS_31          1309  // Megatronics v3.1
#define BOARD_MEGATRONICS_32          1310  // Megatronics v3.2
#define BOARD_ELEFU_3                 1311  // Elefu Ra Board (v3)
#define BOARD_LEAPFROG                1312  // Leapfrog
#define BOARD_MEGACONTROLLER          1313  // Mega controller
#define BOARD_GT2560_REV_A            1314  // Geeetech GT2560 Rev A
#define BOARD_GT2560_REV_A_PLUS       1315  // Geeetech GT2560 Rev A+ (with auto level probe)
#define BOARD_GT2560_REV_B            1316  // Geeetech GT2560 Rev B
#define BOARD_GT2560_V3               1317  // Geeetech GT2560 Rev B for A10(M/T/D)
#define BOARD_GT2560_V3_MC2           1318  // Geeetech GT2560 Rev B for Mecreator2
#define BOARD_GT2560_V3_A20           1319  // Geeetech GT2560 Rev B for A20(M/T/D)
#define BOARD_GT2560_V4               1320  // Geeetech GT2560 Rev B for A10(M/T/D)
#define BOARD_GT2560_V4_A20           1321  // Geeetech GT2560 Rev B for A20(M/T/D)
#define BOARD_GT2560_V41B             1322  // Geeetech GT2560 V4.1B for A10(M/T/D)
#define BOARD_EINSTART_S              1323  // Einstart retrofit
#define BOARD_WANHAO_ONEPLUS          1324  // Wanhao 0ne+ i3 Mini
#define BOARD_OVERLORD                1325  // Overlord/Overlord Pro
#define BOARD_HJC2560C_REV1           1326  // ADIMLab Gantry v1
#define BOARD_HJC2560C_REV2           1327  // ADIMLab Gantry v2
#define BOARD_LEAPFROG_XEED2015       1328  // Leapfrog Xeed 2015
#define BOARD_PICA_REVB               1329  // PICA Shield (original version)
#define BOARD_PICA                    1330  // PICA Shield (rev C or later)
#define BOARD_INTAMSYS40              1331  // Intamsys 4.0 (Funmat HT)
#define BOARD_MALYAN_M180             1332  // Malyan M180 Mainboard Version 2 (no display function, direct G-code only)
#define BOARD_PROTONEER_CNC_SHIELD_V3 1333  // Mega controller & Protoneer CNC Shield V3.00
#define BOARD_WEEDO_62A               1334  // WEEDO 62A board (TINA2, Monoprice Cadet, etc.)

//
// ATmega1281, ATmega2561
//

#define BOARD_MINITRONICS             1400  // Minitronics v1.0/1.1
#define BOARD_SILVER_GATE             1401  // Silvergate v1.0

//
// Sanguinololu and Derivatives - ATmega644P, ATmega1284P
//

#define BOARD_SANGUINOLOLU_11         1500  // Sanguinololu < 1.2
#define BOARD_SANGUINOLOLU_12         1501  // Sanguinololu 1.2 and above
#define BOARD_MELZI                   1502  // Melzi
#define BOARD_MELZI_V2                1503  // Melzi V2
#define BOARD_MELZI_MAKR3D            1504  // Melzi with ATmega1284 (MaKr3d version)
#define BOARD_MELZI_CREALITY          1505  // Melzi Creality3D (for CR-10 etc)
#define BOARD_MELZI_CREALITY_ENDER2   1506  // Melzi Creality3D (for Ender-2)
#define BOARD_MELZI_MALYAN            1507  // Melzi Malyan M150
#define BOARD_MELZI_TRONXY            1508  // Tronxy X5S
#define BOARD_STB_11                  1509  // STB V1.1
#define BOARD_AZTEEG_X1               1510  // Azteeg X1
#define BOARD_ANET_10                 1511  // Anet 1.0 (Melzi clone)
#define BOARD_ZMIB_V2                 1512  // ZoneStar ZMIB V2

//
// Other ATmega644P, ATmega644, ATmega1284P
//

#define BOARD_GEN3_MONOLITHIC         1600  // Gen3 Monolithic Electronics
#define BOARD_GEN3_PLUS               1601  // Gen3+
#define BOARD_GEN6                    1602  // Gen6
#define BOARD_GEN6_DELUXE             1603  // Gen6 deluxe
#define BOARD_GEN7_CUSTOM             1604  // Gen7 custom (Alfons3 Version) https://github.com/Alfons3/Generation_7_Electronics
#define BOARD_GEN7_12                 1605  // Gen7 v1.1, v1.2
#define BOARD_GEN7_13                 1606  // Gen7 v1.3
#define BOARD_GEN7_14                 1607  // Gen7 v1.4
#define BOARD_OMCA_A                  1608  // Alpha OMCA
#define BOARD_OMCA                    1609  // Final OMCA
#define BOARD_SETHI                   1610  // Sethi 3D_1

//
// Teensyduino - AT90USB1286, AT90USB1286P
//

#define BOARD_TEENSYLU                1700  // Teensylu
#define BOARD_PRINTRBOARD             1701  // Printrboard (AT90USB1286)
#define BOARD_PRINTRBOARD_REVF        1702  // Printrboard Revision F (AT90USB1286)
#define BOARD_BRAINWAVE               1703  // Brainwave (AT90USB646)
#define BOARD_BRAINWAVE_PRO           1704  // Brainwave Pro (AT90USB1286)
#define BOARD_SAV_MKI                 1705  // SAV Mk-I (AT90USB1286)
#define BOARD_TEENSY2                 1706  // Teensy++2.0 (AT90USB1286)
#define BOARD_5DPRINT                 1707  // 5DPrint D8 Driver Board

//
// LPC1768 ARM Cortex-M3
//

#define BOARD_RAMPS_14_RE_ARM_EFB     2000  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_14_RE_ARM_EEB     2001  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_14_RE_ARM_EFF     2002  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_14_RE_ARM_EEF     2003  // Re-ARM with RAMPS 1.4 (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_14_RE_ARM_SF      2004  // Re-ARM with RAMPS 1.4 (Power outputs: Spindle, Controller Fan)
#define BOARD_MKS_SBASE               2005  // MKS-Sbase
#define BOARD_AZSMZ_MINI              2006  // AZSMZ Mini
#define BOARD_BIQU_BQ111_A4           2007  // BIQU BQ111-A4
#define BOARD_SELENA_COMPACT          2008  // Selena Compact
#define BOARD_BIQU_B300_V1_0          2009  // BIQU B300_V1.0
#define BOARD_MKS_SGEN_L              2010  // MKS-SGen-L
#define BOARD_GMARSH_X6_REV1          2011  // GMARSH X6, revision 1 prototype
#define BOARD_BTT_SKR_V1_1            2012  // BigTreeTech SKR v1.1
#define BOARD_BTT_SKR_V1_3            2013  // BigTreeTech SKR v1.3
#define BOARD_BTT_SKR_V1_4            2014  // BigTreeTech SKR v1.4
#define BOARD_EMOTRONIC               2015  // eMotion-Tech eMotronic

//
// LPC1769 ARM Cortex-M3
//

#define BOARD_MKS_SGEN                2500  // MKS-SGen
#define BOARD_AZTEEG_X5_GT            2501  // Azteeg X5 GT
#define BOARD_AZTEEG_X5_MINI          2502  // Azteeg X5 Mini
#define BOARD_AZTEEG_X5_MINI_WIFI     2503  // Azteeg X5 Mini Wifi
#define BOARD_COHESION3D_REMIX        2504  // Cohesion3D ReMix
#define BOARD_COHESION3D_MINI         2505  // Cohesion3D Mini
#define BOARD_SMOOTHIEBOARD           2506  // Smoothieboard
#define BOARD_TH3D_EZBOARD            2507  // TH3D EZBoard v1.0
#define BOARD_BTT_SKR_V1_4_TURBO      2508  // BigTreeTech SKR v1.4 TURBO
#define BOARD_MKS_SGEN_L_V2           2509  // MKS SGEN_L V2
#define BOARD_BTT_SKR_E3_TURBO        2510  // BigTreeTech SKR E3 Turbo
#define BOARD_FLY_CDY                 2511  // FLYmaker FLY CDY
#define BOARD_XTLW_CLIMBER_8TH_LPC    2512  // XTLW_CLIMBER_8TH_LPC

//
// SAM3X8E ARM Cortex-M3
//

#define BOARD_DUE3DOM                 3000  // DUE3DOM for Arduino DUE
#define BOARD_DUE3DOM_MINI            3001  // DUE3DOM MINI for Arduino DUE
#define BOARD_RADDS                   3002  // RADDS
#define BOARD_RAMPS_FD_V1             3003  // RAMPS-FD v1
#define BOARD_RAMPS_FD_V2             3004  // RAMPS-FD v2
#define BOARD_RAMPS_SMART_EFB         3005  // RAMPS-SMART (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_SMART_EEB         3006  // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_SMART_EFF         3007  // RAMPS-SMART (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_SMART_EEF         3008  // RAMPS-SMART (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_SMART_SF          3009  // RAMPS-SMART (Power outputs: Spindle, Controller Fan)
#define BOARD_RAMPS_DUO_EFB           3010  // RAMPS Duo (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS_DUO_EEB           3011  // RAMPS Duo (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS_DUO_EFF           3012  // RAMPS Duo (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS_DUO_EEF           3013  // RAMPS Duo (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS_DUO_SF            3014  // RAMPS Duo (Power outputs: Spindle, Controller Fan)
#define BOARD_RAMPS4DUE_EFB           3015  // RAMPS4DUE (Power outputs: Hotend, Fan, Bed)
#define BOARD_RAMPS4DUE_EEB           3016  // RAMPS4DUE (Power outputs: Hotend0, Hotend1, Bed)
#define BOARD_RAMPS4DUE_EFF           3017  // RAMPS4DUE (Power outputs: Hotend, Fan0, Fan1)
#define BOARD_RAMPS4DUE_EEF           3018  // RAMPS4DUE (Power outputs: Hotend0, Hotend1, Fan)
#define BOARD_RAMPS4DUE_SF            3019  // RAMPS4DUE (Power outputs: Spindle, Controller Fan)
#define BOARD_RURAMPS4D_11            3020  // RuRAMPS4Duo v1.1
#define BOARD_RURAMPS4D_13            3021  // RuRAMPS4Duo v1.3
#define BOARD_ULTRATRONICS_PRO        3022  // ReprapWorld Ultratronics Pro V1.0
#define BOARD_ARCHIM1                 3023  // UltiMachine Archim1 (with DRV8825 drivers)
#define BOARD_ARCHIM2                 3024  // UltiMachine Archim2 (with TMC2130 drivers)
#define BOARD_ALLIGATOR               3025  // Alligator Board R2
#define BOARD_CNCONTROLS_15D          3026  // Cartesio CN Controls V15 on DUE
#define BOARD_KRATOS32                3027  // K.3D Kratos32 (Arduino Due Shield)

//
// SAM3X8C ARM Cortex-M3
//

#define BOARD_PRINTRBOARD_G2          3100  // Printrboard G2
#define BOARD_ADSK                    3101  // Arduino DUE Shield Kit (ADSK)

//
// STM32 ARM Cortex-M0+
//

#define BOARD_BTT_EBB42_V1_1          4000  // BigTreeTech EBB42 V1.1 (STM32G0B1CB)
#define BOARD_BTT_SKR_MINI_E3_V3_0    4001  // BigTreeTech SKR Mini E3 V3.0 (STM32G0B0RE / STM32G0B1RE)
#define BOARD_BTT_MANTA_E3_EZ_V1_0    4002  // BigTreeTech Manta E3 EZ V1.0 (STM32G0B1RE)
#define BOARD_BTT_MANTA_M4P_V2_1      4003  // BigTreeTech Manta M4P V2.1 (STM32G0B0RE)
#define BOARD_BTT_MANTA_M5P_V1_0      4004  // BigTreeTech Manta M5P V1.0 (STM32G0B1RE)
#define BOARD_BTT_MANTA_M8P_V1_0      4005  // BigTreeTech Manta M8P V1.0 (STM32G0B1VE)
#define BOARD_BTT_MANTA_M8P_V1_1      4006  // BigTreeTech Manta M8P V1.1 (STM32G0B1VE)
#define BOARD_BTT_SKRAT_V1_0          4007  // BigTreeTech SKRat V1.0 (STM32G0B1VE)

//
// STM32 ARM Cortex-M3
//

#define BOARD_MALYAN_M200_V2          5000  // STM32F070CB controller
#define BOARD_MALYAN_M300             5001  // STM32F070-based delta
#define BOARD_STM32F103RE             5002  // STM32F103RE Libmaple-based STM32F1 controller
#define BOARD_MALYAN_M200             5003  // STM32C8 Libmaple-based STM32F1 controller
#define BOARD_STM3R_MINI              5004  // STM32F103RE Libmaple-based STM32F1 controller
#define BOARD_GTM32_PRO_VB            5005  // STM32F103VE controller
#define BOARD_GTM32_PRO_VD            5006  // STM32F103VE controller
#define BOARD_GTM32_MINI              5007  // STM32F103VE controller
#define BOARD_GTM32_MINI_A30          5008  // STM32F103VE controller
#define BOARD_GTM32_REV_B             5009  // STM32F103VE controller
#define BOARD_MORPHEUS                5010  // STM32F103C8 / STM32F103CB  Libmaple-based STM32F1 controller
#define BOARD_CHITU3D                 5011  // Chitu3D (STM32F103RE)
#define BOARD_MKS_ROBIN               5012  // MKS Robin (STM32F103ZE)
#define BOARD_MKS_ROBIN_MINI          5013  // MKS Robin Mini (STM32F103VE)
#define BOARD_MKS_ROBIN_NANO          5014  // MKS Robin Nano (STM32F103VE)
#define BOARD_MKS_ROBIN_NANO_V2       5015  // MKS Robin Nano V2 (STM32F103VE)
#define BOARD_MKS_ROBIN_LITE          5016  // MKS Robin Lite/Lite2 (STM32F103RC)
#define BOARD_MKS_ROBIN_LITE3         5017  // MKS Robin Lite3 (STM32F103RC)
#define BOARD_MKS_ROBIN_PRO           5018  // MKS Robin Pro (STM32F103ZE)
#define BOARD_MKS_ROBIN_E3            5019  // MKS Robin E3 (STM32F103RC)
#define BOARD_MKS_ROBIN_E3_V1_1       5020  // MKS Robin E3 V1.1 (STM32F103RC)
#define BOARD_MKS_ROBIN_E3D           5021  // MKS Robin E3D (STM32F103RC)
#define BOARD_MKS_ROBIN_E3D_V1_1      5022  // MKS Robin E3D V1.1 (STM32F103RC)
#define BOARD_MKS_ROBIN_E3P           5023  // MKS Robin E3P (STM32F103VE)
#define BOARD_BTT_SKR_MINI_V1_1       5024  // BigTreeTech SKR Mini v1.1 (STM32F103RC)
#define BOARD_BTT_SKR_MINI_E3_V1_0    5025  // BigTreeTech SKR Mini E3 (STM32F103RC)
#define BOARD_BTT_SKR_MINI_E3_V1_2    5026  // BigTreeTech SKR Mini E3 V1.2 (STM32F103RC)
#define BOARD_BTT_SKR_MINI_E3_V2_0    5027  // BigTreeTech SKR Mini E3 V2.0 (STM32F103RC / STM32F103RE)
#define BOARD_BTT_SKR_MINI_MZ_V1_0    5028  // BigTreeTech SKR Mini MZ V1.0 (STM32F103RC)
#define BOARD_BTT_SKR_E3_DIP          5029  // BigTreeTech SKR E3 DIP V1.0 (STM32F103RC / STM32F103RE)
#define BOARD_BTT_SKR_CR6             5030  // BigTreeTech SKR CR6 v1.0 (STM32F103RE)
#define BOARD_JGAURORA_A5S_A1         5031  // JGAurora A5S A1 (STM32F103ZE)
#define BOARD_FYSETC_AIO_II           5032  // FYSETC AIO_II (STM32F103RC)
#define BOARD_FYSETC_CHEETAH          5033  // FYSETC Cheetah (STM32F103RC)
#define BOARD_FYSETC_CHEETAH_V12      5034  // FYSETC Cheetah V1.2 (STM32F103RC)
#define BOARD_LONGER3D_LK             5035  // Longer3D LK1/2 - Alfawise U20/U20+/U30 (STM32F103VE)
#define BOARD_CCROBOT_MEEB_3DP        5036  // ccrobot-online.com MEEB_3DP (STM32F103RC)
#define BOARD_CHITU3D_V5              5037  // Chitu3D TronXY X5SA V5 Board (STM32F103ZE)
#define BOARD_CHITU3D_V6              5038  // Chitu3D TronXY X5SA V6 Board (STM32F103ZE)
#define BOARD_CHITU3D_V9              5039  // Chitu3D TronXY X5SA V9 Board (STM32F103ZE)
#define BOARD_CREALITY_V4             5040  // Creality v4.x (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V422           5041  // Creality v4.2.2 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V423           5042  // Creality v4.2.3 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V425           5043  // Creality v4.2.5 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V427           5044  // Creality v4.2.7 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V4210          5045  // Creality v4.2.10 (STM32F103RC / STM32F103RE) as found in the CR-30
#define BOARD_CREALITY_V431           5046  // Creality v4.3.1 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V431_A         5047  // Creality v4.3.1a (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V431_B         5048  // Creality v4.3.1b (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V431_C         5049  // Creality v4.3.1c (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V431_D         5050  // Creality v4.3.1d (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V452           5051  // Creality v4.5.2 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V453           5052  // Creality v4.5.3 (STM32F103RC / STM32F103RE)
#define BOARD_CREALITY_V521           5053  // Creality v5.2.1 (STM32F103VE) as found in the SV04
#define BOARD_CREALITY_V24S1          5054  // Creality v2.4.S1 (STM32F103RC / STM32F103RE) CR-FDM-v2.4.S1_v101 as found in the Ender-7
#define BOARD_CREALITY_V24S1_301      5055  // Creality v2.4.S1_301 (STM32F103RC / STM32F103RE) CR-FDM-v24S1_301 as found in the Ender-3 S1
#define BOARD_CREALITY_V25S1          5056  // Creality v2.5.S1 (STM32F103RE) CR-FDM-v2.5.S1_100 as found in the CR-10 Smart Pro
#define BOARD_TRIGORILLA_PRO          5057  // Trigorilla Pro (STM32F103ZE)
#define BOARD_FLY_MINI                5058  // FLYmaker FLY MINI (STM32F103RC)
#define BOARD_FLSUN_HISPEED           5059  // FLSUN HiSpeedV1 (STM32F103VE)
#define BOARD_BEAST                   5060  // STM32F103RE Libmaple-based controller
#define BOARD_MINGDA_MPX_ARM_MINI     5061  // STM32F103ZE Mingda MD-16
#define BOARD_ZONESTAR_ZM3E2          5062  // Zonestar ZM3E2    (STM32F103RC)
#define BOARD_ZONESTAR_ZM3E4          5063  // Zonestar ZM3E4 V1 (STM32F103VC)
#define BOARD_ZONESTAR_ZM3E4V2        5064  // Zonestar ZM3E4 V2 (STM32F103VC)
#define BOARD_ERYONE_ERY32_MINI       5065  // Eryone Ery32 mini (STM32F103VE)
#define BOARD_PANDA_PI_V29            5066  // Panda Pi V2.9 - Standalone (STM32F103RC)
#define BOARD_SOVOL_V131              5067  // Sovol V1.3.1 (GD32F103RE)
#define BOARD_TRIGORILLA_V006         5068  // Trigorilla V0.0.6 (GD32F103RE)
#define BOARD_KEDI_CONTROLLER_V1_2    5069  // EDUTRONICS Kedi Controller V1.2 (STM32F103RC)
#define BOARD_MD_D301                 5070  // Mingda D2 DZ301 V1.0 (STM32F103ZE)
#define BOARD_VOXELAB_AQUILA          5071  // Voxelab Aquila V1.0.0/V1.0.1 (GD32F103RC / N32G455RE / STM32F103RE)
#define BOARD_SPRINGER_CONTROLLER     5072  // ORCA 3D SPRINGER Modular Controller (STM32F103VC)

//
// ARM Cortex-M4F
//

#define BOARD_TEENSY31_32             5100  // Teensy3.1 and Teensy3.2
#define BOARD_TEENSY35_36             5101  // Teensy3.5 and Teensy3.6

//
// STM32 ARM Cortex-M4F
//

#define BOARD_ARMED                   5200  // Arm'ed STM32F4-based controller
#define BOARD_RUMBA32_V1_0            5201  // RUMBA32 STM32F446VE based controller from Aus3D
#define BOARD_RUMBA32_V1_1            5202  // RUMBA32 STM32F446VE based controller from Aus3D
#define BOARD_RUMBA32_MKS             5203  // RUMBA32 STM32F446VE based controller from Makerbase
#define BOARD_RUMBA32_BTT             5204  // RUMBA32 STM32F446VE based controller from BIGTREETECH
#define BOARD_BLACK_STM32F407VE       5205  // Black STM32F407VE development board
#define BOARD_BLACK_STM32F407ZE       5206  // Black STM32F407ZE development board
#define BOARD_BTT_SKR_MINI_E3_V3_0_1  5207  // BigTreeTech SKR Mini E3 V3.0.1 (STM32F401RC)
#define BOARD_BTT_SKR_PRO_V1_1        5208  // BigTreeTech SKR Pro v1.1 (STM32F407ZG)
#define BOARD_BTT_SKR_PRO_V1_2        5209  // BigTreeTech SKR Pro v1.2 (STM32F407ZG)
#define BOARD_BTT_BTT002_V1_0         5210  // BigTreeTech BTT002 v1.0 (STM32F407VG)
#define BOARD_BTT_E3_RRF              5211  // BigTreeTech E3 RRF (STM32F407VG)
#define BOARD_BTT_SKR_V2_0_REV_A      5212  // BigTreeTech SKR v2.0 Rev A (STM32F407VG)
#define BOARD_BTT_SKR_V2_0_REV_B      5213  // BigTreeTech SKR v2.0 Rev B (STM32F407VG/STM32F429VG)
#define BOARD_BTT_GTR_V1_0            5214  // BigTreeTech GTR v1.0 (STM32F407IGT)
#define BOARD_BTT_OCTOPUS_V1_0        5215  // BigTreeTech Octopus v1.0 (STM32F446ZE)
#define BOARD_BTT_OCTOPUS_V1_1        5216  // BigTreeTech Octopus v1.1 (STM32F446ZE)
#define BOARD_BTT_OCTOPUS_PRO_V1_0    5217  // BigTreeTech Octopus Pro v1.0 (STM32F446ZE / STM32F429ZG)
#define BOARD_LERDGE_K                5218  // Lerdge K (STM32F407ZG)
#define BOARD_LERDGE_S                5219  // Lerdge S (STM32F407VE)
#define BOARD_LERDGE_X                5220  // Lerdge X (STM32F407VE)
#define BOARD_FYSETC_S6               5221  // FYSETC S6 (STM32F446VE)
#define BOARD_FYSETC_S6_V2_0          5222  // FYSETC S6 v2.0 (STM32F446VE)
#define BOARD_FYSETC_SPIDER           5223  // FYSETC Spider (STM32F446VE)
#define BOARD_FYSETC_SPIDER_V2_2      5224  // FYSETC Spider V2.2 (STM32F446VE)
#define BOARD_FLYF407ZG               5225  // FLYmaker FLYF407ZG (STM32F407ZG)
#define BOARD_MKS_ROBIN2              5226  // MKS Robin2 V1.0 (STM32F407ZE)
#define BOARD_MKS_ROBIN_PRO_V2        5227  // MKS Robin Pro V2 (STM32F407VE)
#define BOARD_MKS_ROBIN_NANO_V3       5228  // MKS Robin Nano V3 (STM32F407VG)
#define BOARD_MKS_ROBIN_NANO_V3_1     5229  // MKS Robin Nano V3.1 (STM32F407VE)
#define BOARD_MKS_MONSTER8_V1         5230  // MKS Monster8 V1 (STM32F407VE)
#define BOARD_MKS_MONSTER8_V2         5231  // MKS Monster8 V2 (STM32F407VE)
#define BOARD_ANET_ET4                5232  // ANET ET4 V1.x (STM32F407VG)
#define BOARD_ANET_ET4P               5233  // ANET ET4P V1.x (STM32F407VG)
#define BOARD_FYSETC_CHEETAH_V20      5234  // FYSETC Cheetah V2.0 (STM32F401RC)
#define BOARD_FYSETC_CHEETAH_V30      5235  // FYSETC Cheetah V3.0 (STM32F446RC)
#define BOARD_TH3D_EZBOARD_V2         5236  // TH3D EZBoard v2.0 (STM32F405RG)
#define BOARD_OPULO_LUMEN_REV3        5237  // Opulo Lumen PnP Controller REV3 (STM32F407VE / STM32F407VG)
#define BOARD_OPULO_LUMEN_REV4        5238  // Opulo Lumen PnP Controller REV4 (STM32F407VE / STM32F407VG)
#define BOARD_MKS_ROBIN_NANO_V1_3_F4  5239  // MKS Robin Nano V1.3 and MKS Robin Nano-S V1.3 (STM32F407VE)
#define BOARD_MKS_EAGLE               5240  // MKS Eagle (STM32F407VE)
#define BOARD_ARTILLERY_RUBY          5241  // Artillery Ruby (STM32F401RC)
#define BOARD_CREALITY_V24S1_301F4    5242  // Creality v2.4.S1_301F4 (STM32F401RC) as found in the Ender-3 S1 F4
#define BOARD_CREALITY_CR4NTXXC10     5243  // Creality E3 Free-runs Silent Motherboard (STM32F401RET6)
#define BOARD_FYSETC_SPIDER_KING407   5244  // FYSETC Spider King407 (STM32F407ZG)
#define BOARD_MKS_SKIPR_V1            5245  // MKS SKIPR v1.0 all-in-one board (STM32F407VE)
#define BOARD_TRONXY_CXY_446_V10      5246  // TRONXY CXY-446-V10-220413/CXY-V6-191121 (STM32F446ZE)
#define BOARD_CREALITY_F401RE         5247  // Creality CR4NS200141C13 (STM32F401RE) as found in the Ender-5 S1
#define BOARD_BLACKPILL_CUSTOM        5248  // Custom board based on STM32F401CDU6.
#define BOARD_I3DBEEZ9_V1             5249  // I3DBEEZ9 V1 (STM32F407ZG)
#define BOARD_MELLOW_FLY_E3_V2        5250  // Mellow Fly E3 V2 (STM32F407VG)
#define BOARD_BLACKBEEZMINI_V1        5251  // BlackBeezMini V1 (STM32F401CCU6)
#define BOARD_XTLW_CLIMBER_8TH        5252  // XTLW Climber-8th (STM32F407VGT6)
#define BOARD_FLY_RRF_E3_V1           5253  // Fly RRF E3 V1.0 (STM32F407VG)
#define BOARD_ZNP_ROBIN_NANO          5254  // Elegoo Neptune 2 v1.2 board
#define BOARD_ZNP_ROBIN_NANO_V1_3     5255  // Elegoo Neptune 2 v1.3 board
#define BOARD_MKS_NEPTUNE_X           5256  // Elegoo Neptune X
#define BOARD_MKS_NEPTUNE_3           5257  // Elegoo Neptune 3

//
// Other ARM Cortex-M4
//
#define BOARD_CREALITY_CR4NS          5300  // Creality CR4NS200320C13 (GD32F303RET6) as found in the Ender-3 V3 SE

//
// ARM Cortex-M7
//

#define BOARD_REMRAM_V1               6000  // RemRam v1
#define BOARD_NUCLEO_F767ZI           6001  // ST NUCLEO-F767ZI Dev Board
#define BOARD_BTT_SKR_SE_BX_V2        6002  // BigTreeTech SKR SE BX V2.0 (STM32H743II)
#define BOARD_BTT_SKR_SE_BX_V3        6003  // BigTreeTech SKR SE BX V3.0 (STM32H743II)
#define BOARD_BTT_SKR_V3_0            6004  // BigTreeTech SKR V3.0 (STM32H743VI / STM32H723VG)
#define BOARD_BTT_SKR_V3_0_EZ         6005  // BigTreeTech SKR V3.0 EZ (STM32H743VI / STM32H723VG)
#define BOARD_BTT_OCTOPUS_MAX_EZ_V1_0 6006  // BigTreeTech Octopus Max EZ V1.0 (STM32H723ZE)
#define BOARD_BTT_OCTOPUS_PRO_V1_0_1  6007  // BigTreeTech Octopus Pro v1.0.1 (STM32H723ZE)
#define BOARD_BTT_OCTOPUS_PRO_V1_1    6008  // BigTreeTech Octopus Pro v1.1 (STM32H723ZE)
#define BOARD_BTT_MANTA_M8P_V2_0      6009  // BigTreeTech Manta M8P V2.0 (STM32H723ZE)
#define BOARD_BTT_KRAKEN_V1_0         6010  // BigTreeTech Kraken v1.0 (STM32H723ZG)
#define BOARD_TEENSY41                6011  // Teensy 4.1
#define BOARD_T41U5XBB                6012  // T41U5XBB Teensy 4.1 breakout board

//
// Espressif ESP32 WiFi
//

#define BOARD_ESPRESSIF_ESP32         7000  // Generic ESP32
#define BOARD_MRR_ESPA                7001  // MRR ESPA based on ESP32 (native pins only)
#define BOARD_MRR_ESPE                7002  // MRR ESPE based on ESP32 (with I2S stepper stream)
#define BOARD_E4D_BOX                 7003  // E4d@BOX
#define BOARD_RESP32_CUSTOM           7004  // Rutilea ESP32 custom board
#define BOARD_FYSETC_E4               7005  // FYSETC E4
#define BOARD_PANDA_ZHU               7006  // Panda_ZHU
#define BOARD_PANDA_M4                7007  // Panda_M4
#define BOARD_MKS_TINYBEE             7008  // MKS TinyBee based on ESP32 (with I2S stepper stream)
#define BOARD_ENWI_ESPNP              7009  // enwi ESPNP based on ESP32 (with I2S stepper stream)
#define BOARD_GODI_CONTROLLER_V1_0    7010  // Godi Controller based on ESP32 32-Bit V1.0
#define BOARD_MM_JOKER                7011  // MagicMaker JOKER based on ESP32 (with I2S stepper stream)

//
// SAMD51 ARM Cortex-M4
//

#define BOARD_AGCM4_RAMPS_144         7100  // RAMPS 1.4.4
#define BOARD_BRICOLEMON_V1_0         7101  // Bricolemon
#define BOARD_BRICOLEMON_LITE_V1_0    7102  // Bricolemon Lite

//
// SAMD21 ARM Cortex-M4
//

#define BOARD_MINITRONICS20           7103  // Minitronics v2.0

//
// HC32 ARM Cortex-M4
//

#define BOARD_AQUILA_V101             7200  // Voxelab Aquila V1.0.0/V1.0.1/V1.0.2/V1.0.3 as found in the Voxelab Aquila X2 and C2
#define BOARD_CREALITY_ENDER2P_V24S4  7201  // Creality Ender 2 Pro v2.4.S4_170 (HC32f460kcta)

//
// Raspberry Pi
//

#define BOARD_RP2040                  6200 // Generic RP2040 Test board
#define BOARD_BTT_SKR_PICO            6201 // BigTreeTech SKR Pico 1.x

//
// Custom board
//

#define BOARD_CUSTOM                  9998  // Custom pins definition for development and/or rare boards

//
// Simulations
//

#define BOARD_SIMULATED               9999  // Simulated 3D Printer with LCD / TFT for development

#define _MB_1(B)  (defined(BOARD_##B) && MOTHERBOARD==BOARD_##B)
#define MB(V...)  DO(MB,||,V)
