/*******************************************************************************
 * @file    math-util.c
 * @brief   Contains mathematically useful macros and functions.
 ******************************************************************************/

#include "math-util.h"

#include <math.h>

float clampf(float v, float lo, float hi) { return fmaxf(lo, fminf(v, hi)); }
