#include <limits.h>
#include "unity.h"
#include "calc.h"

void setUp(void) {}
void tearDown(void) {}

void test_add(void)
{
    TEST_ASSERT_EQUAL(30, add(10, 20));

    /* Both operands fit in a signed 8-bit int (max 127) but the sum does not.
       It fits in 16 bits (max 32767), so this only passes if int is >= 16 bits. */
    TEST_ASSERT_EQUAL(200, add(100, 100));
    TEST_ASSERT_EQUAL(254, add(127, 127));
    TEST_ASSERT_TRUE_MESSAGE(INT_MAX >= 32767, "int is narrower than 16 bits");
}

void test_is_equal(void)
{
    TEST_ASSERT_TRUE(is_equal(5, 5));
    TEST_ASSERT_FALSE(is_equal(5, 6));
}

void test_power(void)
{
    TEST_ASSERT_EQUAL(1024, power(2, 10));
    TEST_ASSERT_EQUAL(1000, power(10, 3));
}

void test_magnitude(void)
{
    /* if int were unsigned, -1 < 0 would be false */
    TEST_ASSERT_TRUE_MESSAGE((int)-1 < 0, "int is not signed");

    TEST_ASSERT_EQUAL_UINT(5u, magnitude(-5));
    TEST_ASSERT_EQUAL_UINT(5u, magnitude(5));
    TEST_ASSERT_EQUAL_UINT(0u, magnitude(0));
    TEST_ASSERT_EQUAL_UINT((unsigned int)INT_MAX + 1u, magnitude(INT_MIN));
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