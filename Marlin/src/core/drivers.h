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

//
// Included by MarlinConfigPre.h ahead of Configuration_adv.h.
// Don't use #if in this file for anything not defined early!
//

#define _A4988              0x4988
#define _A5984              0x5984
#define _DRV8825            0x8825
#define _LV8729             0x8729
#define _TB6560             0x6560
#define _TB6600             0x6600
#define _TMC2100            0x2100
#define _TMC2130            0x2130A
#define _TMC2130_STANDALONE 0x2130B
#define _TMC2160            0x2160A
#define _TMC2160_STANDALONE 0x2160B
#define _TMC2208            0x2208A
#define _TMC2208_STANDALONE 0x2208B
#define _TMC2209            0x2209A
#define _TMC2209_STANDALONE 0x2209B
#define _TMC2660            0x2660A
#define _TMC2660_STANDALONE 0x2660B
#define _TMC5130            0x5130A
#define _TMC5130_STANDALONE 0x5130B
#define _TMC5160            0x5160A
#define _TMC5160_STANDALONE 0x5160B

#define _DRIVER_ID(V) _CAT(_, V)
#define _AXIS_DRIVER_TYPE(A,T) (_DRIVER_ID(A##_DRIVER_TYPE) == _DRIVER_ID(T))

#define AXIS_DRIVER_TYPE_X(T) _AXIS_DRIVER_TYPE(X,T)
#define AXIS_DRIVER_TYPE_Y(T) _AXIS_DRIVER_TYPE(Y,T)
#define AXIS_DRIVER_TYPE_Z(T) _AXIS_DRIVER_TYPE(Z,T)
#define AXIS_DRIVER_TYPE_I(T) _AXIS_DRIVER_TYPE(I,T)
#define AXIS_DRIVER_TYPE_J(T) _AXIS_DRIVER_TYPE(J,T)
#define AXIS_DRIVER_TYPE_K(T) _AXIS_DRIVER_TYPE(K,T)
#define AXIS_DRIVER_TYPE_U(T) _AXIS_DRIVER_TYPE(U,T)
#define AXIS_DRIVER_TYPE_V(T) _AXIS_DRIVER_TYPE(V,T)
#define AXIS_DRIVER_TYPE_W(T) _AXIS_DRIVER_TYPE(W,T)

#define AXIS_DRIVER_TYPE_X2(T) (HAS_X2_STEPPER && _AXIS_DRIVER_TYPE(X2,T))
#define AXIS_DRIVER_TYPE_Y2(T) (HAS_Y2_STEPPER && _AXIS_DRIVER_TYPE(Y2,T))
#define AXIS_DRIVER_TYPE_Z2(T) (NUM_Z_STEPPERS >= 2 && _AXIS_DRIVER_TYPE(Z2,T))
#define AXIS_DRIVER_TYPE_Z3(T) (NUM_Z_STEPPERS >= 3 && _AXIS_DRIVER_TYPE(Z3,T))
#define AXIS_DRIVER_TYPE_Z4(T) (NUM_Z_STEPPERS >= 4 && _AXIS_DRIVER_TYPE(Z4,T))

#define AXIS_DRIVER_TYPE_E(N,T) (E_STEPPERS > N && _AXIS_DRIVER_TYPE(E##N,T))
#define AXIS_DRIVER_TYPE_E0(T) AXIS_DRIVER_TYPE_E(0,T)
#define AXIS_DRIVER_TYPE_E1(T) AXIS_DRIVER_TYPE_E(1,T)
#define AXIS_DRIVER_TYPE_E2(T) AXIS_DRIVER_TYPE_E(2,T)
#define AXIS_DRIVER_TYPE_E3(T) AXIS_DRIVER_TYPE_E(3,T)
#define AXIS_DRIVER_TYPE_E4(T) AXIS_DRIVER_TYPE_E(4,T)
#define AXIS_DRIVER_TYPE_E5(T) AXIS_DRIVER_TYPE_E(5,T)
#define AXIS_DRIVER_TYPE_E6(T) AXIS_DRIVER_TYPE_E(6,T)
#define AXIS_DRIVER_TYPE_E7(T) AXIS_DRIVER_TYPE_E(7,T)

#define AXIS_DRIVER_TYPE(A,T) AXIS_DRIVER_TYPE_##A(T)

#define _OR_ADTE(N,T)   || AXIS_DRIVER_TYPE_E(N,T)
#define HAS_E_DRIVER(T) (0 RREPEAT2(E_STEPPERS, _OR_ADTE, T))

#define HAS_DRIVER(T) (  AXIS_DRIVER_TYPE_X(T)  || AXIS_DRIVER_TYPE_Y(T)  || AXIS_DRIVER_TYPE_Z(T)  \
                      || AXIS_DRIVER_TYPE_I(T)  || AXIS_DRIVER_TYPE_J(T)  || AXIS_DRIVER_TYPE_K(T)  \
                      || AXIS_DRIVER_TYPE_U(T)  || AXIS_DRIVER_TYPE_V(T)  || AXIS_DRIVER_TYPE_W(T)  \
                      || AXIS_DRIVER_TYPE_X2(T) || AXIS_DRIVER_TYPE_Y2(T) || AXIS_DRIVER_TYPE_Z2(T) \
                      || AXIS_DRIVER_TYPE_Z3(T) || AXIS_DRIVER_TYPE_Z4(T) || HAS_E_DRIVER(T) )

//
// Trinamic Stepper Drivers
//

// Test for supported TMC drivers that require advanced configuration
// Does not match standalone configurations
#if (    HAS_DRIVER(TMC2130) || HAS_DRIVER(TMC2160) \
      || HAS_DRIVER(TMC2208) || HAS_DRIVER(TMC2209) \
      || HAS_DRIVER(TMC2660) \
      || HAS_DRIVER(TMC5130) || HAS_DRIVER(TMC5160) )
  #define HAS_TRINAMIC_CONFIG 1
#endif

#define HAS_TRINAMIC HAS_TRINAMIC_CONFIG

#if (    HAS_DRIVER(TMC2100) \
      || HAS_DRIVER(TMC2130_STANDALONE) || HAS_DRIVER(TMC2160_STANDALONE) \
      || HAS_DRIVER(TMC2208_STANDALONE) || HAS_DRIVER(TMC2209_STANDALONE) \
      || HAS_DRIVER(TMC2660_STANDALONE) || HAS_DRIVER(TMC5130_STANDALONE) \
      || HAS_DRIVER(TMC5160_STANDALONE) )
  #define HAS_TRINAMIC_STANDALONE 1
#endif

#if HAS_DRIVER(TMC2130) || HAS_DRIVER(TMC2160) || HAS_DRIVER(TMC5130) || HAS_DRIVER(TMC5160)
  #define HAS_TMCX1X0 1
#endif

#if HAS_DRIVER(TMC2208) || HAS_DRIVER(TMC2209)
  #define HAS_TMC220x 1
#endif

#define AXIS_IS_TMC(A)   (    AXIS_DRIVER_TYPE(A,TMC2130) || AXIS_DRIVER_TYPE(A,TMC2160) \
                           || AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) \
                           || AXIS_DRIVER_TYPE(A,TMC2660) \
                           || AXIS_DRIVER_TYPE(A,TMC5130) || AXIS_DRIVER_TYPE(A,TMC5160) )

#define AXIS_IS_TMC_CONFIG AXIS_IS_TMC

// Test for a driver that uses SPI - this allows checking whether a _CS_ pin
// is considered sensitive
#define AXIS_HAS_SPI(A)  (    AXIS_DRIVER_TYPE(A,TMC2130) || AXIS_DRIVER_TYPE(A,TMC2160) \
                           || AXIS_DRIVER_TYPE(A,TMC2660) || AXIS_DRIVER_TYPE(A,TMC5130) \
                           || AXIS_DRIVER_TYPE(A,TMC5160) )

#define AXIS_HAS_UART(A) ( AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )

#define AXIS_HAS_RXTX AXIS_HAS_UART

#define AXIS_HAS_HW_SERIAL(A) ( AXIS_HAS_UART(A) &&  defined(A##_HARDWARE_SERIAL) )
#define AXIS_HAS_SW_SERIAL(A) ( AXIS_HAS_UART(A) && !defined(A##_HARDWARE_SERIAL) )

#define AXIS_HAS_STALLGUARD(A)   (    AXIS_DRIVER_TYPE(A,TMC2130) || AXIS_DRIVER_TYPE(A,TMC2160) \
                                   || AXIS_DRIVER_TYPE(A,TMC2209) \
                                   || AXIS_DRIVER_TYPE(A,TMC2660) \
                                   || AXIS_DRIVER_TYPE(A,TMC5130) || AXIS_DRIVER_TYPE(A,TMC5160) )

#define AXIS_HAS_STEALTHCHOP(A)  (    AXIS_DRIVER_TYPE(A,TMC2130) || AXIS_DRIVER_TYPE(A,TMC2160) \
                                   || AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) \
                                   || AXIS_DRIVER_TYPE(A,TMC5130) || AXIS_DRIVER_TYPE(A,TMC5160) )

#define AXIS_HAS_SG_RESULT(A)    (    AXIS_DRIVER_TYPE(A,TMC2130) || AXIS_DRIVER_TYPE(A,TMC2160) \
                                   || AXIS_DRIVER_TYPE(A,TMC2208) || AXIS_DRIVER_TYPE(A,TMC2209) )

#define AXIS_HAS_COOLSTEP(A)     (    AXIS_DRIVER_TYPE(A,TMC2130) \
                                   || AXIS_DRIVER_TYPE(A,TMC2209) \
                                   || AXIS_DRIVER_TYPE(A,TMC5130) || AXIS_DRIVER_TYPE(A,TMC5160) )

#define _OR_EAH(N,T)    || AXIS_HAS_##T(E##N)
#define E_AXIS_HAS(T)   (0 _OR_EAH(0,T) _OR_EAH(1,T) _OR_EAH(2,T) _OR_EAH(3,T) _OR_EAH(4,T) _OR_EAH(5,T) _OR_EAH(6,T) _OR_EAH(7,T))

#define ANY_AXIS_HAS(T) (    AXIS_HAS_##T(X) || AXIS_HAS_##T(X2) \
                          || AXIS_HAS_##T(Y) || AXIS_HAS_##T(Y2) \
                          || AXIS_HAS_##T(Z) || AXIS_HAS_##T(Z2) || AXIS_HAS_##T(Z3) || AXIS_HAS_##T(Z4) \
                          || AXIS_HAS_##T(I) || AXIS_HAS_##T(J)  || AXIS_HAS_##T(K) \
                          || AXIS_HAS_##T(U) || AXIS_HAS_##T(V)  || AXIS_HAS_##T(W) \
                          || E_AXIS_HAS(T) )

#if ANY_AXIS_HAS(STEALTHCHOP)
  #define HAS_STEALTHCHOP 1
#endif
#if ANY_AXIS_HAS(STALLGUARD)
  #define HAS_STALLGUARD 1
#endif
#if ANY_AXIS_HAS(SG_RESULT)
  #define HAS_SG_RESULT 1
#endif
#if ANY_AXIS_HAS(COOLSTEP)
  #define HAS_COOLSTEP 1
#endif
#if ANY_AXIS_HAS(RXTX)
  #define HAS_TMC_UART 1
#endif
#if ANY_AXIS_HAS(SPI)
  #define HAS_TMC_SPI 1
#endif
#if HAS_STALLGUARD || HAS_DRIVER(TMC2160_STANDALONE) || HAS_DRIVER(TMC2130_STANDALONE) \
                   || HAS_DRIVER(TMC2209_STANDALONE) || HAS_DRIVER(TMC2660_STANDALONE) \
                   || HAS_DRIVER(TMC5130_STANDALONE) || HAS_DRIVER(TMC5160_STANDALONE)
  #define HAS_DIAG_PINS 1
#endif

// Hybrid Threshold ranges
#define THRS_TMC2100     65535
#define THRS_TMC2130     65535
#define THRS_TMC2160       255
#define THRS_TMC2208       255
#define THRS_TMC2209       255
#define THRS_TMC2660     65535
#define THRS_TMC5130     65535
#define THRS_TMC5160     65535

#define _DRIVER_THRS(V) CAT(THRS_, V)
#define STEPPER_MAX_THRS(S) _DRIVER_THRS(S##_DRIVER_TYPE)
