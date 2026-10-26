#include <limits.h>
#include "unity.h"
#include "calc.h"

void setUp(void) {}
void tearDown(void) {}

void test_add(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(30, add(10, 20), 
        "Failed standard positive addition (10 + 20)");

    /* Both operands fit in a signed 8-bit int (max 127) but the sum does not.
       It fits in 16 bits (max 32767), so this only passes if int is >= 16 bits. */
    TEST_ASSERT_EQUAL_INT_MESSAGE(200, add(100, 100), 
        "Failed 8-bit overflow range addition (100 + 100)");
        
    TEST_ASSERT_EQUAL_INT_MESSAGE(254, add(127, 127), 
        "Failed boundary addition at edge of 8-bit signed int limit (127 + 127)");
        
    TEST_ASSERT_TRUE_MESSAGE(INT_MAX >= 32767, 
        "Environment restriction: int is narrower than 16 bits on this platform");
}

void test_is_equal(void)
{
    TEST_ASSERT_TRUE_MESSAGE(is_equal(5, 5), 
        "Failed validation: identical positive integers should be evaluated as equal");
        
    TEST_ASSERT_FALSE_MESSAGE(is_equal(5, 6), 
        "Failed validation: mismatching positive integers should not be evaluated as equal");
}

void test_power(void)
{
    TEST_ASSERT_EQUAL_INT_MESSAGE(1024, power(2, 10), 
        "Failed base-2 exponentiation handling (2^10)");
        
    TEST_ASSERT_EQUAL_INT_MESSAGE(1000, power(10, 3), 
        "Failed base-10 exponentiation handling (10^3)");
}

void test_magnitude(void)
{
    /* if int were unsigned, -1 < 0 would be false */
    TEST_ASSERT_TRUE_MESSAGE((int)-1 < 0, 
        "Compiler representation error: int type is unexpected behavior or not signed");

    TEST_ASSERT_EQUAL_UINT_MESSAGE(5u, magnitude(-5), 
        "Failed absolute value conversion for a standard negative integer (-5)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE(5u, magnitude(5), 
        "Failed identity conversion for a standard positive integer (5)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE(0u, magnitude(0), 
        "Failed boundary handling for zero magnitude value (0)");
        
    TEST_ASSERT_EQUAL_UINT_MESSAGE((unsigned int)INT_MAX + 1u, magnitude(INT_MIN), 
        "Failed critical boundary handling: magnitude of absolute maximum negative limits (INT_MIN)");
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_add);
    RUN_TEST(test_is_equal);
    RUN_TEST(test_power);
    RUN_TEST(test_magnitude);
    return UNITY_END();
}
