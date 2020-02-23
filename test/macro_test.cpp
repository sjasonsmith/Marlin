#include <unity.h>
#include "../Marlin/src/core/macros.h"
#include "../Marlin/src/core/drivers.h"
/*
 * Options: A4988, A5984, DRV8825, LV8729, L6470, L6474, POWERSTEP01,
 *          TB6560, TB6600, TMC2100,
 *          TMC2130, TMC2130_STANDALONE, TMC2160, TMC2160_STANDALONE,
 *          TMC2208, TMC2208_STANDALONE, TMC2209, TMC2209_STANDALONE,
 *          TMC26X,  TMC26X_STANDALONE,  TMC2660, TMC2660_STANDALONE,
 *          TMC5130, TMC5130_STANDALONE, TMC5160, TMC5160_STANDALONE
 * :['A4988', 'A5984', 'DRV8825', 'LV8729', 'L6470', 'L6474', 'POWERSTEP01', 'TB6560', 'TB6600', 'TMC2100', 'TMC2130', 'TMC2130_STANDALONE', 'TMC2160', 'TMC2160_STANDALONE', 'TMC2208', 'TMC2208_STANDALONE', 'TMC2209', 'TMC2209_STANDALONE', 'TMC26X', 'TMC26X_STANDALONE', 'TMC2660', 'TMC2660_STANDALONE', 'TMC5130', 'TMC5130_STANDALONE', 'TMC5160', 'TMC5160_STANDALONE']
 */

void test_AXIS_IS_TMC_A4988(void) {
#define X_DRIVER_TYPE A4988
    constexpr bool is_tmc =
    #if AXIS_IS_TMC(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(is_tmc);

    constexpr bool has_uart = 
    #if AXIS_HAS_UART(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(has_uart);
#undef X_DRIVER_TYPE
}

void test_AXIS_IS_TMC_TMC2209(void) {
#define X_DRIVER_TYPE TMC2209
    constexpr bool is_tmc =
    #if AXIS_IS_TMC(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_TRUE(is_tmc);

    bool is_sw_serial =
    #if AXIS_HAS_SW_SERIAL(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_TRUE(is_sw_serial);

    constexpr bool has_uart = 
    #if AXIS_HAS_UART(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_TRUE(has_uart);

#define X_HARDWARE_SERIAL Serial1
    is_sw_serial =
    #if AXIS_HAS_SW_SERIAL(X)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(is_sw_serial);

#undef X_DRIVER_TYPE
#undef X_HARDWARE_SERIAL
}

void test_AXIS_IS_TMC_TMC2209_E0(void) {
#define EXTRUDERS 1
#define E_STEPPERS EXTRUDERS
#define E0_DRIVER_TYPE TMC2209
    constexpr bool is_tmc =
//    #if AXIS_IS_TMC(E0)
    #if (E_STEPPERS > 0 && _AXIS_DRIVER_TYPE(E0,TMC2209))
        true;
    #else
        false;
    #endif
    TEST_ASSERT_TRUE(is_tmc);

    bool is_sw_serial =
    #if AXIS_HAS_SW_SERIAL(E0)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_TRUE(is_sw_serial);

#define E0_HARDWARE_SERIAL Serial1
    is_sw_serial =
    #if AXIS_HAS_SW_SERIAL(E0)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(is_sw_serial);

    constexpr bool e_has_sw_serial =
    //#if E_AXIS_HAS(SW_SERIAL)
    //#if (0 EVAL(_RREPEAT2(0,SUB0(1),_OR_EAH,SW_SERIAL)))
    #if (0 EVAL(_RREPEAT2(0,SUB0(1),_OR_EAH,SW_SERIAL)))
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(e_has_sw_serial);

#undef E0_DRIVER_TYPE
#undef E0_HARDWARE_SERIAL
#undef EXTRUDERS
#undef E_STEPPERS
}

void test_RREPEAT2(void) {
#define _COUNT_BY_ONE(N) 1
#define _COUNT_BY_TWO(N) 2
#define _COUNT_BY_FIVE(N) 5

#define _COUNT(N,T)    + _COUNT_BY_##T(N)
#define COUNT_BY(N, T)  (0 RREPEAT2(N, _COUNT, T))

    TEST_ASSERT_EQUAL(5, COUNT_BY(5, ONE));
    TEST_ASSERT_EQUAL(1, COUNT_BY(1, ONE));
    TEST_ASSERT_EQUAL(2, COUNT_BY(1, TWO));
    TEST_ASSERT_EQUAL(18, COUNT_BY(9, TWO));
    TEST_ASSERT_EQUAL(5, COUNT_BY(1, FIVE));
    TEST_ASSERT_EQUAL(45, COUNT_BY(9, FIVE));
}

void test_something(void) {
    #define E0_HARDWARE_SERIAL Serial1
    #define _HAS_HW_SERIAL(N) defined(N##_HARDWARE_SERIAL)
    #define HAS_HW_SERIAL(N) _HAS_HW_SERIAL(N)
    #define MY_AXIS_HAS_SW_SERIAL(A) ((AXIS_HAS_UART(A) && !_HAS_HW_SERIAL(A))) //defined(A##_HARDWARE_SERIAL)))
    #define MY_OR_EAH(N)    || MY_AXIS_HAS_SW_SERIAL(E##N)
    #define MY_E_AXIS_HAS()   (0 RREPEAT(E_STEPPERS, MY_OR_EAH))

    #define EXTRUDERS 1
    #define E_STEPPERS EXTRUDERS
    #define E0_DRIVER_TYPE TMC2209

    constexpr bool has_spi =
    #if E_AXIS_HAS(SPI)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(has_spi);

    constexpr bool has_hw_serial =
    #if HAS_HW_SERIAL(E0)
        true;
    #else  
        false;
    #endif
    TEST_ASSERT_TRUE(has_hw_serial);

    constexpr bool my_axis_has_sw_serial =
    #if MY_AXIS_HAS_SW_SERIAL(E0)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(my_axis_has_sw_serial);    

    constexpr bool my_or_eah =
    #if false MY_OR_EAH(0)
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(my_or_eah);  

    constexpr bool e_has_sw_serial =
    #if MY_E_AXIS_HAS()
        true;
    #else
        false;
    #endif
    TEST_ASSERT_FALSE(e_has_sw_serial);    


}


int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_AXIS_IS_TMC_A4988);
    RUN_TEST(test_AXIS_IS_TMC_TMC2209);
    //RUN_TEST(test_AXIS_IS_TMC_TMC2209_E0);
    RUN_TEST(test_RREPEAT2);
    RUN_TEST(test_something);
    UNITY_END();

    return 0;
}
void setUp (void) {} /* Is run before every test, put unit init calls here. */
void tearDown (void) {} /* Is run after every test, put unit clean-up calls here. */
