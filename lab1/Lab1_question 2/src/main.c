#include <stdio.h>
#include "calc.h"

#ifndef PIO_UNIT_TESTING
int main(void)
{
    printf("10 + 2 = %d\n", add(10, 2));
    printf("10 - 2 = %d\n", sub(10, 2));
    printf("10 * 2 = %d\n", mul(10, 2));
    printf("10 power 3 = %d\n", power(10, 3));
    printf("magnitude(-10) = %u\n", magnitude(-10));
    printf("magnitude(10)  = %u\n", magnitude(10));
    return 0;
}
#endif
