#ifndef CALC_H
#define CALC_H

#include <stdbool.h>

int add(int x, int y);
int sub(int x, int y);
int mul(int x, int y);
int power(int x, int n);
bool is_equal(int x, int y);
bool is_greater(int x, int y);
bool is_smaller(int x, int y);
unsigned int magnitude(int x);

#endif