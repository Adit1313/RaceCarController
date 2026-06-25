/*******************************************************************************
 * @file    math-util.h
 * @brief   Contains mathematically useful macros and functions.
 ******************************************************************************/

#pragma once

/* Public macros ------------------------------------------------------------ */

/** Single-precision Pi for faster calculations */
#define M_PIf 3.141592653589793f

/** Converts degrees to radians. */
#define DEG2RAD(x) ((x)*M_PIf / 180.0f)

/** Converts gravitational units to m/s^2. */
#define G2MPSS(x) ((x)*9.81f)

/* Public function definitions ---------------------------------------------- */

/**
 * @brief Clamps a floating-point integer to a range [li, hi].
 *
 * The definition of this function is equivalent to std::clamp. Refer to the C++
 * referencee for more information.
 * @param v the value to clamp
 * @param lo lower boundary to clamp v to
 * @param hi upper boundary to clamp v to
 * @return value of v if lo < v < hi, lo if v < lo, hi if hi < v
 */
float clampf(float v, float lo, float hi);
