/**
 * File:   macro_logic.h
 * Author: root
 *
 * Created on December 22, 2018, 4:49 AM
 *
 * To test:
 * gcc -DAJK_TEST_MACRO_LOGIC -E macro_logic.h
 */

#ifndef MACRO_LOGIC_H
#define MACRO_LOGIC_H

#define AJK_CAT(a, ...) AJK_PRIMITIVE_CAT(a, __VA_ARGS__)
#define AJK_PRIMITIVE_CAT(a, ...) a ## __VA_ARGS__

#define AJK_COMPL(b) AJK_PRIMITIVE_CAT(AJK_COMPL_, b)
#define AJK_COMPL_0 1
#define AJK_COMPL_1 0

#define AJK_BITAND(x) AJK_PRIMITIVE_CAT(AJK_BITAND_, x)
#define AJK_BITAND_0(y) 0
#define AJK_BITAND_1(y) y

#define AJK_INC(x) AJK_PRIMITIVE_CAT(AJK_INC_, x)
#define AJK_INC_0 1
#define AJK_INC_1 2
#define AJK_INC_2 3
#define AJK_INC_3 4
#define AJK_INC_4 5
#define AJK_INC_5 6
#define AJK_INC_6 7
#define AJK_INC_7 8
#define AJK_INC_8 9
#define AJK_INC_9 10
#define AJK_INC_10 10

#define AJK_DEC(x) AJK_PRIMITIVE_CAT(AJK_DEC_, x)
#define AJK_DEC_0 0
#define AJK_DEC_1 0
#define AJK_DEC_2 1
#define AJK_DEC_3 2
#define AJK_DEC_4 3
#define AJK_DEC_5 4
#define AJK_DEC_6 5
#define AJK_DEC_7 6
#define AJK_DEC_8 7
#define AJK_DEC_9 8
#define AJK_DEC_10 9

#define AJK_CHECK_N(x, n, ...) n
#define AJK_CHECK(...) AJK_CHECK_N(__VA_ARGS__, 0,)
#define AJK_PROBE(x) x, 1,

#define AJK_IS_PAREN(x) AJK_CHECK(AJK_IS_PAREN_PROBE x)
#define AJK_IS_PAREN_PROBE(...) AJK_PROBE(~)

#define AJK_NOT(x) AJK_CHECK(AJK_PRIMITIVE_CAT(AJK_NOT_, x))
#define AJK_NOT_0 AJK_PROBE(~)

#define AJK_COMPL(b) AJK_PRIMITIVE_CAT(AJK_COMPL_, b)
#define AJK_COMPL_0 1
#define AJK_COMPL_1 0

#define AJK_BOOL(x) AJK_COMPL(AJK_NOT(x))

#define AJK_IIF(c) AJK_PRIMITIVE_CAT(AJK_IIF_, c)
#define AJK_IIF_0(t, ...) __VA_ARGS__
#define AJK_IIF_1(t, ...) t

#define AJK_IF(c) AJK_IIF(AJK_BOOL(c))

#define AJK_EAT(...)
#define AJK_EXPAND(...) __VA_ARGS__
#define AJK_WHEN(c) AJK_IF(c)(AJK_EXPAND, AJK_EAT)

#define AJK_EMPTY()
#define AJK_DEFER(id) id AJK_EMPTY()
#define AJK_OBSTRUCT(id) id AJK_DEFER(AJK_EMPTY)()

#define AJK_EVAL(...)  AJK_EVAL1(AJK_EVAL1(AJK_EVAL1(__VA_ARGS__)))
#define AJK_EVAL1(...) AJK_EVAL2(AJK_EVAL2(AJK_EVAL2(__VA_ARGS__)))
#define AJK_EVAL2(...) AJK_EVAL3(AJK_EVAL3(AJK_EVAL3(__VA_ARGS__)))
#define AJK_EVAL3(...) AJK_EVAL4(AJK_EVAL4(AJK_EVAL4(__VA_ARGS__)))
#define AJK_EVAL4(...) AJK_EVAL5(AJK_EVAL5(AJK_EVAL5(__VA_ARGS__)))
#define AJK_EVAL5(...) __VA_ARGS__

#define AJK_REPEAT(AJK_count, AJK_macro, ...) \
    AJK_WHEN(AJK_count) \
    ( \
        AJK_OBSTRUCT(AJK_REPEAT_INDIRECT) () \
        ( \
            AJK_DEC(AJK_count), AJK_macro, __VA_ARGS__ \
        ) \
        AJK_OBSTRUCT(AJK_macro) \
        ( \
            AJK_DEC(AJK_count), __VA_ARGS__ \
        ) \
    )
#define AJK_REPEAT_INDIRECT() AJK_REPEAT

#define AJK_WHILE(AJK_pred, AJK_op, ...) \
    IF(AJK_pred(__VA_ARGS__)) \
    ( \
        AJK_OBSTRUCT(AJK_WHILE_INDIRECT) () \
        ( \
            AJK_pred, AJK_op, AJK_op(__VA_ARGS__) \
        ), \
        __VA_ARGS__ \
    )
#define AJK_WHILE_INDIRECT() AJK_WHILE

#define AJK_PRIMITIVE_COMPARE(x, y) AJK_IS_PAREN \
( \
    AJK_COMPARE_ ## x ( AJK_COMPARE_ ## y) (())  \
)

#define AJK_IS_COMPARABLE(x) AJK_IS_PAREN( AJK_CAT(AJK_COMPARE_, x) (()) )

#define AJK_NOT_EQUAL(x, y) \
AJK_IIF(AJK_BITAND(AJK_IS_COMPARABLE(x))(AJK_IS_COMPARABLE(y)) ) \
( \
   AJK_PRIMITIVE_COMPARE, \
   1 AJK_EAT \
)(x, y)

#define AJK_EQUAL(x, y) AJK_COMPL(AJK_NOT_EQUAL(x, y))

#define AJK_COMMA() ,

#define AJK_COMMA_IF(n) AJK_IF(n)(AJK_COMMA, AJK_EAT)()

#define AJK_COMMA_VAR(AJK_count, AJK_v) AJK_COMMA_IF(AJK_count) AJK_v ## AJK_count

#define AJK_MAKE_LIST(AJK_v, AJK_count) AJK_EVAL(AJK_REPEAT(AJK_count, AJK_COMMA_VAR, AJK_v))

#define AJK_FUN(AJK_count, AJK_v, AJK_args, AJK_body) AJK_v ## AJK_count (AJK_args) { AJK_body(AJK_count) }
#define AJK_MAKE_FUNS(AJK_v, AJK_args, AJK_count, AJK_body) AJK_EVAL(AJK_REPEAT(AJK_count, AJK_FUN, AJK_v, AJK_args, AJK_body))
#ifdef AJK_TEST_MACRO_LOGIC

#define BODY(AJKindex) some(C, statement); containing(a, test[AJKindex]);
#define ZERO_TIMES_TEST 0
#define THREE_TIMES_TEST 3
blank > AJK_MAKE_LIST(VARIABLE_, ZERO_TIMES_TEST) < because zero repeats
Make 3 comma separated indexed variables : AJK_MAKE_LIST(VARIABLE_, THREE_TIMES_TEST)
Make 3 bogus function bodies
AJK_MAKE_FUNS(unsigned Cfunc,(arg1, arg2),3,BODY)
#endif

#endif // MACRO_LOGIC_H
