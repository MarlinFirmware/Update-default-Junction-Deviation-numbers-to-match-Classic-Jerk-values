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

#include "../test/unit_tests.h"
#include "src/core/types.h"

MARLIN_TEST(types, XYval_const_as_bools) {
  const XYval<int> xy_const_true = {1, 2};
  TEST_ASSERT_TRUE(xy_const_true);

  const XYval<int> xy_const_false = {0, 0};
  TEST_ASSERT_FALSE(xy_const_false);
}

MARLIN_TEST(types, XYval_non_const_as_bools) {
  XYval<int> xy_true = {1, 2};
  TEST_ASSERT_TRUE(xy_true);

  XYval<int> xy_false = {0, 0};
  TEST_ASSERT_FALSE(xy_false);
}

MARLIN_TEST(types, XYval_reset) {
  XYval<int> xy = {1, 2};
  xy.reset();
  TEST_ASSERT_EQUAL(0, xy.x);
  TEST_ASSERT_EQUAL(0, xy.y);
}

MARLIN_TEST(types, XYval_set) {
  XYval<int> xy;
  xy.set(3, 4);
  TEST_ASSERT_EQUAL(3, xy.x);
  TEST_ASSERT_EQUAL(4, xy.y);
}

MARLIN_TEST(types, XYval_magnitude) {
  XYval<int> xy;

  xy.set(3, 4);
  TEST_ASSERT_EQUAL(5, xy.magnitude());

  xy.set(-3, -4);
  TEST_ASSERT_EQUAL(5, xy.magnitude());

  xy.set(-3, 4);
  TEST_ASSERT_EQUAL(5, xy.magnitude());

  xy.set(3, -4);
  TEST_ASSERT_EQUAL(5, xy.magnitude());
}

MARLIN_TEST(types, XYval_small_large) {
  XYval<int> xy;

  xy.set(3, 4);
  TEST_ASSERT_EQUAL(3, xy.small());
  TEST_ASSERT_EQUAL(4, xy.large());

  xy.set(4, 3);
  TEST_ASSERT_EQUAL(3, xy.small());
  TEST_ASSERT_EQUAL(4, xy.large());

  // BUG?: Is this behavior actually correct?
  // Does small mean "less than", or should it mean
  // "closer to zero"? If the latter, then the following
  // tests are incorrect.
  xy.set(-3, -4);
  TEST_ASSERT_EQUAL(-4, xy.small());
  TEST_ASSERT_EQUAL(-3, xy.large());

  xy.set(-3, 2);
  TEST_ASSERT_EQUAL(-3, xy.small());
  TEST_ASSERT_EQUAL(2, xy.large());

  xy.set(2, -3);
  TEST_ASSERT_EQUAL(-3, xy.small());
  TEST_ASSERT_EQUAL(2, xy.large());
}

MARLIN_TEST(types, XYval_operators) {
  XYval<int> xy1 = {2, 3}, xy2 = {6, 12};
  XYval<int> xy3 = xy1 + xy2;
  TEST_ASSERT_EQUAL(8, xy3.x);
  TEST_ASSERT_EQUAL(15, xy3.y);
  xy3 = xy1 - xy2;
  TEST_ASSERT_EQUAL(-4, xy3.x);
  TEST_ASSERT_EQUAL(-9, xy3.y);
  xy3 = xy1 * xy2;
  TEST_ASSERT_EQUAL(12, xy3.x);
  TEST_ASSERT_EQUAL(36, xy3.y);
  xy3 = xy2 / xy1;
  TEST_ASSERT_EQUAL(3, xy3.x);
  TEST_ASSERT_EQUAL(4, xy3.y);
}

MARLIN_TEST(types, XYval_ABS) {
  XYval<int> xy = {-3, -4};
  XYval<int> xy_abs = xy.ABS();
  TEST_ASSERT_EQUAL(3, xy_abs.x);
  TEST_ASSERT_EQUAL(4, xy_abs.y);
}

MARLIN_TEST(types, XYval_ROUNDL) {
  XYval<float> xy = {3.3f, 4.7f};
  auto xy_round = xy.ROUNDL();
  TEST_ASSERT_EQUAL(3, xy_round.x);
  TEST_ASSERT_EQUAL(5, xy_round.y);
}

MARLIN_TEST(types, XYval_reciprocal) {
  XYval<float> xy = {0.5f, 4.0f};
  XYval<float> xy_reciprocal = xy.reciprocal();
  TEST_ASSERT_EQUAL_FLOAT(2.0f, xy_reciprocal.x);
  TEST_ASSERT_EQUAL_FLOAT(0.25f, xy_reciprocal.y);
}

MARLIN_TEST(types, XYZval_const_as_bools) {
  const XYZval<int> xyz_const_true = {1, 2, 3};
  TEST_ASSERT_TRUE(xyz_const_true);

  const XYZval<int> xyz_const_false = {0, 0, 0};
  TEST_ASSERT_FALSE(xyz_const_false);
}

MARLIN_TEST(types, XYZval_non_const_as_bools) {
  XYZval<int> xyz_true = {1, 2, 3};
  TEST_ASSERT_TRUE(xyz_true);

  XYZval<int> xyz_false = {0, 0, 0};
  TEST_ASSERT_FALSE(xyz_false);
}

MARLIN_TEST(types, XYZval_reset) {
  XYZval<int> xyz = {1, 2, 3};
  xyz.reset();
  TEST_ASSERT_EQUAL(0, xyz.x);
  TEST_ASSERT_EQUAL(0, xyz.y);
  TEST_ASSERT_EQUAL(0, xyz.z);
}

MARLIN_TEST(types, XYZval_set) {
  XYZval<int> xyz;
  xyz.set(3, 4, 5);
  TEST_ASSERT_EQUAL(3, xyz.x);
  TEST_ASSERT_EQUAL(4, xyz.y);
  TEST_ASSERT_EQUAL(5, xyz.z);
}

MARLIN_TEST(types, XYZval_magnitude) {
  XYZval<float> xyz;

  xyz.set(3.0f, 4.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.071f, xyz.magnitude());

  xyz.set(-3.0f, -4.0f, -5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.071f, xyz.magnitude());

  xyz.set(-3.0f, 4.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.071f, xyz.magnitude());

  xyz.set(3.0f, -4.0f, 5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.071f, xyz.magnitude());

  xyz.set(3.0f, 4.0f, -5.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 7.071f, xyz.magnitude());
}

MARLIN_TEST(types, XYZval_small_large) {
  XYZval<int> xyz;

  xyz.set(3, 4, 5);
  TEST_ASSERT_EQUAL(3, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  xyz.set(5, 4, 3);
  TEST_ASSERT_EQUAL(3, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  xyz.set(4, 3, 5);
  TEST_ASSERT_EQUAL(3, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  xyz.set(3, 5, 4);
  TEST_ASSERT_EQUAL(3, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  // Test with negative numbers
  xyz.set(-3, -4, -5);
  TEST_ASSERT_EQUAL(-5, xyz.small());
  TEST_ASSERT_EQUAL(-3, xyz.large());

  // Test with mixed negative/positive numbers
  xyz.set(-3, 4, 5);
  TEST_ASSERT_EQUAL(-3, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  xyz.set(3, -4, 5);
  TEST_ASSERT_EQUAL(-4, xyz.small());
  TEST_ASSERT_EQUAL(5, xyz.large());

  xyz.set(3, 4, -5);
  TEST_ASSERT_EQUAL(-5, xyz.small());
  TEST_ASSERT_EQUAL(4, xyz.large());
}

MARLIN_TEST(types, XYZval_operators) {
  XYZval<int> xyz1 = {2, 3, 4}, xyz2 = {6, 12, 24};
  XYZval<int> xyz3 = xyz1 + xyz2;
  TEST_ASSERT_EQUAL(8, xyz3.x);
  TEST_ASSERT_EQUAL(15, xyz3.y);
  TEST_ASSERT_EQUAL(28, xyz3.z);
  xyz3 = xyz1 - xyz2;
  TEST_ASSERT_EQUAL(-4, xyz3.x);
  TEST_ASSERT_EQUAL(-9, xyz3.y);
  TEST_ASSERT_EQUAL(-20, xyz3.z);
  xyz3 = xyz1 * xyz2;
  TEST_ASSERT_EQUAL(12, xyz3.x);
  TEST_ASSERT_EQUAL(36, xyz3.y);
  TEST_ASSERT_EQUAL(96, xyz3.z);
  xyz3 = xyz2 / xyz1;
  TEST_ASSERT_EQUAL(3, xyz3.x);
  TEST_ASSERT_EQUAL(4, xyz3.y);
  TEST_ASSERT_EQUAL(6, xyz3.z);
}

MARLIN_TEST(types, XYZval_ABS) {
  XYZval<int> xyz = {-3, -4, -5};
  XYZval<int> xyz_abs = xyz.ABS();
  TEST_ASSERT_EQUAL(3, xyz_abs.x);
  TEST_ASSERT_EQUAL(4, xyz_abs.y);
  TEST_ASSERT_EQUAL(5, xyz_abs.z);
}

MARLIN_TEST(types, XYZval_ROUNDL) {
  XYZval<float> xyz = {3.3f, 4.7f, 5.5f};
  XYZval<int> xyz_round = xyz.ROUNDL();
  TEST_ASSERT_EQUAL(3, xyz_round.x);
  TEST_ASSERT_EQUAL(5, xyz_round.y);
  TEST_ASSERT_EQUAL(6, xyz_round.z);
}

MARLIN_TEST(types, XYZval_reciprocal) {
  XYZval<float> xyz = {0.5f, 2.0f, 0.33333f};
  XYZval<float> xyz_reciprocal = xyz.reciprocal();
  TEST_ASSERT_EQUAL_FLOAT(2.0f, xyz_reciprocal.x);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, xyz_reciprocal.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, xyz_reciprocal.z);
}

MARLIN_TEST(types, XYZEval_const_as_bools) {
  const XYZEval<int> xyze_const_true = {1, 2, 3, 4};
  TEST_ASSERT_TRUE(xyze_const_true);

  const XYZEval<int> xyze_const_false = {0, 0, 0, 0};
  TEST_ASSERT_FALSE(xyze_const_false);
}

MARLIN_TEST(types, XYZEval_non_const_as_bools) {
  XYZEval<int> xyze_true = {1, 2, 3, 4};
  TEST_ASSERT_TRUE(xyze_true);

  XYZEval<int> xyze_false = {0, 0, 0, 0};
  TEST_ASSERT_FALSE(xyze_false);
}

MARLIN_TEST(types, XYZEval_reset) {
  XYZEval<int> xyze = {1, 2, 3, 4};
  xyze.reset();
  TEST_ASSERT_EQUAL(0, xyze.x);
  TEST_ASSERT_EQUAL(0, xyze.y);
  TEST_ASSERT_EQUAL(0, xyze.z);
  TEST_ASSERT_EQUAL(0, xyze.e);
}

MARLIN_TEST(types, XYZEval_set) {
  XYZEval<int> xyze;
  xyze.set(3, 4, 5, 6);
  TEST_ASSERT_EQUAL(3, xyze.x);
  TEST_ASSERT_EQUAL(4, xyze.y);
  TEST_ASSERT_EQUAL(5, xyze.z);
  TEST_ASSERT_EQUAL(6, xyze.e);
}

MARLIN_TEST(types, XYZEval_magnitude) {
  XYZEval<float> xyze;

  xyze.set(3.0f, 4.0f, 5.0f, 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());

  xyze.set(-3.0f, -4.0f, -5.0f, -6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());

  xyze.set(-3.0f, 4.0f, 5.0f, 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());

  xyze.set(3.0f, -4.0f, 5.0f, 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());

  xyze.set(3.0f, 4.0f, -5.0f, 6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());

  xyze.set(3.0f, 4.0f, 5.0f, -6.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 9.274f, xyze.magnitude());
}

MARLIN_TEST(types, XYZEval_small_large) {
  XYZEval<int> xyze;

  xyze.set(3, 4, 5, 6);
  TEST_ASSERT_EQUAL(3, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(6, 5, 4, 3);
  TEST_ASSERT_EQUAL(3, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(4, 3, 6, 5);
  TEST_ASSERT_EQUAL(3, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(3, 6, 5, 4);
  TEST_ASSERT_EQUAL(3, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(-3, -4, -5, -6);
  TEST_ASSERT_EQUAL(-6, xyze.small());
  TEST_ASSERT_EQUAL(-3, xyze.large());

  xyze.set(-3, 4, 5, 6);
  TEST_ASSERT_EQUAL(-3, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(3, -4, 5, 6);
  TEST_ASSERT_EQUAL(-4, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(3, 4, -5, 6);
  TEST_ASSERT_EQUAL(-5, xyze.small());
  TEST_ASSERT_EQUAL(6, xyze.large());

  xyze.set(3, 4, 5, -6);
  TEST_ASSERT_EQUAL(-6, xyze.small());
  TEST_ASSERT_EQUAL(5, xyze.large());
}

MARLIN_TEST(types, XYZEval_operators) {
  XYZEval<int> xyze1 = {2, 3, 4, 5}, xyze2 = {6, 12, 24, 48};
  XYZEval<int> xyze3 = xyze1 + xyze2;
  TEST_ASSERT_EQUAL(8, xyze3.x);
  TEST_ASSERT_EQUAL(15, xyze3.y);
  TEST_ASSERT_EQUAL(28, xyze3.z);
  TEST_ASSERT_EQUAL(53, xyze3.e);
  xyze3 = xyze1 - xyze2;
  TEST_ASSERT_EQUAL(-4, xyze3.x);
  TEST_ASSERT_EQUAL(-9, xyze3.y);
  TEST_ASSERT_EQUAL(-20, xyze3.z);
  TEST_ASSERT_EQUAL(-43, xyze3.e);
  xyze3 = xyze1 * xyze2;
  TEST_ASSERT_EQUAL(12, xyze3.x);
  TEST_ASSERT_EQUAL(36, xyze3.y);
  TEST_ASSERT_EQUAL(96, xyze3.z);
  TEST_ASSERT_EQUAL(240, xyze3.e);
  xyze3 = xyze2 / xyze1;
  TEST_ASSERT_EQUAL(3, xyze3.x);
  TEST_ASSERT_EQUAL(4, xyze3.y);
  TEST_ASSERT_EQUAL(6, xyze3.z);
  TEST_ASSERT_EQUAL(9, xyze3.e);
}

MARLIN_TEST(types, XYZEval_ABS) {
  XYZEval<int> xyze = {-3, -4, -5, -6};
  XYZEval<int> xyze_abs = xyze.ABS();
  TEST_ASSERT_EQUAL(3, xyze_abs.x);
  TEST_ASSERT_EQUAL(4, xyze_abs.y);
  TEST_ASSERT_EQUAL(5, xyze_abs.z);
  TEST_ASSERT_EQUAL(6, xyze_abs.e);
}

MARLIN_TEST(types, XYZEval_ROUNDL) {
  XYZEval<float> xyze = {3.3f, 4.7f, 5.5f, 6.6f};
  XYZEval<int> xyze_round = xyze.ROUNDL();
  TEST_ASSERT_EQUAL(3, xyze_round.x);
  TEST_ASSERT_EQUAL(5, xyze_round.y);
  TEST_ASSERT_EQUAL(6, xyze_round.z);
  TEST_ASSERT_EQUAL(7, xyze_round.e);
}

MARLIN_TEST(types, XYZEval_reciprocal) {
  XYZEval<float> xyze = {0.5f, 2.0f, 0.33333f, 0.25f};
  XYZEval<float> xyze_reciprocal = xyze.reciprocal();
  TEST_ASSERT_EQUAL_FLOAT(2.0f, xyze_reciprocal.x);
  TEST_ASSERT_EQUAL_FLOAT(0.5f, xyze_reciprocal.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001f, 3.0f, xyze_reciprocal.z);
  TEST_ASSERT_EQUAL_FLOAT(4.0f, xyze_reciprocal.e);
}

MARLIN_TEST(types, Flags_const_as_bools) {
  const Flags<32> flags_const_false = {0};
  TEST_ASSERT_FALSE(flags_const_false);

  const Flags<32> flags_const_true = {1};
  TEST_ASSERT_TRUE(flags_const_true);
}

MARLIN_TEST(types, Flags_non_const_as_bools) {
  Flags<32> flags_false = {0};
  TEST_ASSERT_FALSE(flags_false);

  Flags<32> flags_true = {1};
  TEST_ASSERT_TRUE(flags_true);
}

MARLIN_TEST(types, Flags_1) {
  Flags<1> flags;

  flags.set(0, true);
  TEST_ASSERT_EQUAL(1, flags.b);

  flags.reset();
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.clear(0);
  TEST_ASSERT_EQUAL(0, flags.b);

  TEST_ASSERT_EQUAL(false, flags.test(0));
  flags.set(0, true);
  TEST_ASSERT_EQUAL(true, flags.test(0));

  TEST_ASSERT_EQUAL(true, flags[0]);
  flags.clear(0);
  TEST_ASSERT_EQUAL(false, flags[0]);

  TEST_ASSERT_EQUAL(1, flags.size());
}

MARLIN_TEST(types, Flags_8) {
  Flags<8> flags;

  flags.reset();
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(3, true);
  TEST_ASSERT_EQUAL(8, flags.b);

  flags.clear(3);
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(3, true);
  TEST_ASSERT_EQUAL(true, flags.test(3));
  TEST_ASSERT_EQUAL(false, flags.test(2));

  TEST_ASSERT_EQUAL(true, flags[3]);
  TEST_ASSERT_EQUAL(false, flags[2]);

  TEST_ASSERT_EQUAL(1, flags.size());
}

MARLIN_TEST(types, Flags_16) {
  Flags<16> flags;

  flags.reset();
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.set(15, true);
  TEST_ASSERT_EQUAL(32769, flags.b);

  flags.clear(0);
  TEST_ASSERT_EQUAL(32768, flags.b);

  flags.reset();
  flags.set(7, true);
  flags.set(15, true);
  TEST_ASSERT_EQUAL(true, flags.test(7));
  TEST_ASSERT_EQUAL(false, flags.test(8));
  TEST_ASSERT_EQUAL(true, flags.test(15));

  TEST_ASSERT_EQUAL(true, flags[7]);
  TEST_ASSERT_EQUAL(false, flags[8]);
  TEST_ASSERT_EQUAL(true, flags[15]);

  TEST_ASSERT_EQUAL(2, flags.size());
}

MARLIN_TEST(types, Flags_32) {
  Flags<32> flags;

  flags.reset();
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.set(31, true);
  TEST_ASSERT_EQUAL(2147483649, flags.b);

  flags.clear(0);
  flags.clear(31);
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.set(31, true);
  TEST_ASSERT_EQUAL(true, flags.test(0));
  TEST_ASSERT_EQUAL(true, flags.test(31));
  TEST_ASSERT_EQUAL(false, flags.test(1));
  TEST_ASSERT_EQUAL(false, flags.test(30));

  TEST_ASSERT_EQUAL(true, flags[0]);
  TEST_ASSERT_EQUAL(true, flags[31]);
  TEST_ASSERT_EQUAL(false, flags[1]);
  TEST_ASSERT_EQUAL(false, flags[30]);

  TEST_ASSERT_EQUAL(4, flags.size());
}

MARLIN_TEST(types, Flags_64) {
  Flags<64> flags;

  flags.reset();
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.set(63, true);
  TEST_ASSERT_EQUAL(9223372036854775809ULL, flags.b);

  flags.clear(0);
  flags.clear(63);
  TEST_ASSERT_EQUAL(0, flags.b);

  flags.set(0, true);
  flags.set(63, true);
  TEST_ASSERT_EQUAL(true, flags.test(0));
  TEST_ASSERT_EQUAL(true, flags.test(63));
  TEST_ASSERT_EQUAL(false, flags.test(1));
  TEST_ASSERT_EQUAL(false, flags.test(62));

  TEST_ASSERT_EQUAL(true, flags[0]);
  TEST_ASSERT_EQUAL(true, flags[63]);
  TEST_ASSERT_EQUAL(false, flags[1]);
  TEST_ASSERT_EQUAL(false, flags[62]);

  TEST_ASSERT_EQUAL(8, flags.size());
}

MARLIN_TEST(types, Flags_302) {
  Flags<302> flags;

  flags.reset();
  TEST_ASSERT_EQUAL(false, (bool)flags);

  flags.set(0, true);
  flags.set(301, true);

  TEST_ASSERT_EQUAL(true, (bool)flags);

  TEST_ASSERT_EQUAL(true, flags.test(0));
  TEST_ASSERT_EQUAL(true, flags.test(301));
  TEST_ASSERT_EQUAL(false, flags.test(1));
  TEST_ASSERT_EQUAL(false, flags.test(300));

  TEST_ASSERT_EQUAL(true, flags[0]);
  TEST_ASSERT_EQUAL(true, flags[301]);
  TEST_ASSERT_EQUAL(false, flags[1]);
  TEST_ASSERT_EQUAL(false, flags[300]);

  TEST_ASSERT_EQUAL(38, flags.size());
}

MARLIN_TEST(types, AxisFlags_const_as_bools) {
  const AxisFlags axis_flags_const_false = {0};
  TEST_ASSERT_FALSE(axis_flags_const_false);

  const AxisFlags axis_flags_const_true = {1};
  TEST_ASSERT_TRUE(axis_flags_const_true);
}

MARLIN_TEST(types, AxisFlags_non_const_as_bools) {
  AxisFlags axis_flags_false = {0};
  TEST_ASSERT_FALSE(axis_flags_false);

  AxisFlags axis_flags_true = {1};
  TEST_ASSERT_TRUE(axis_flags_true);
}

MARLIN_TEST(types, AxisBits_const_as_bools) {
  const AxisBits axis_bits_const_false = {0};
  TEST_ASSERT_FALSE(axis_bits_const_false);

  const AxisBits axis_bits_const_true = {1};
  TEST_ASSERT_TRUE(axis_bits_const_true);
}

MARLIN_TEST(types, AxisBits_non_const_as_bools) {
  AxisBits axis_bits_false = {0};
  TEST_ASSERT_FALSE(axis_bits_false);

  AxisBits axis_bits_true = {1};
  TEST_ASSERT_TRUE(axis_bits_true);
}

MARLIN_TEST(types, MString1) {
  // String with cutoff at 20 chars:
  // "F-string, 1234.50, 2"
  MString<20> str20;
  str20 = F("F-string, ");
  str20.append(1234.5f).append(',').append(' ')
       .append(2345.67).append(',').append(' ');

  TEST_ASSERT_TRUE(strcmp_P(str20, PSTR("F-string, 1234.50, 2")) == 0);

  // Truncate to "F-string"
  str20.trunc(8);

  TEST_ASSERT_FALSE(strcmp_P(&str20, PSTR("F-string")) != 0);
}

MARLIN_TEST(types, MString2) {
  // 100 dashes, but chopped down to DEFAULT_MSTRING_SIZE (20)
  TEST_ASSERT_TRUE(TSS(repchr_t('-', 100)).length() == 20);
}

MARLIN_TEST(types, SString) {
  // Hello World!-123456------   < spaces!33
  // ^ eol! ... 1234.50*2345.602 = 2895645.67
  SString<100> str(F("Hello"));
  str.append(F(" World!"));
  str += '-';
  str += uint8_t(123);
  str += F("456");
  str += repchr_t('-', 6);
  str += Spaces(3);
  str += "< spaces!";
  str += int8_t(33);
  str.eol();
  str += "^ eol!";
  str.append(" ... ", 1234.5f, '*', p_float_t(2345.602, 3), F(" = "), 1234.5 * 2345.602);

  TEST_ASSERT_TRUE(strcmp_P(str, PSTR("Hello World!-123456------   < spaces!33\n^ eol! ... 1234.50*2345.602 = 2895645.67")) == 0);
}
