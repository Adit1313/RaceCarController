/*******************************************************************************
 * @file    adc-steering-estimator.h
 * @brief   Steering angle estimator based solely on a potentiometer reading.
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control-types.h"

/**
 * @brief Estimate the steering angle based on the voltage of the potentiometer.
 *
 * @param left_limit ADC position when left-most position is reached.
 * @param right_limit ADC position when right-most position is reached.
 * @param amplitude The amount the wheels can steer left and right (rad).
 * @param measurement The ADC measurement of the feedback voltage
 *
 * @pre left_limit and right_limit must be distinct and the amplitude must be a
 *      non-zero positive integer.
 * @returns The steering angle estimate. If the preconditions are violated, may
 *          return NaN or Infinity.
 */
SteeringAngle estimate_steer(const ADCMeasurement left_limit,
                             const ADCMeasurement right_limit,
                             const SteeringAngle amplitude,
                             const ADCMeasurement measurement);
