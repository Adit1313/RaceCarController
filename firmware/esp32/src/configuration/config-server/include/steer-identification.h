/*******************************************************************************
 * @file    steer-identification.h
 * @brief   Finds the limits of the potentiometer for the car the PCB is on.
 ******************************************************************************/

#pragma once

#include <cstdbool>

/* Public type declarations ------------------------------------------------- */

/**
 * @brief Groups the two limits of the steering potentiometer in the gearbox.
 */
struct PotentiometerLimits {
    unsigned int lower; ///< Lower limit of the steering feedback voltage [mV].
    unsigned int upper; ///< Upper limit of the steering feedback voltage [mV].
};

/**
 * @brief Triees to find the limits of the potentiometer.
 *
 * This function will first provide a constant left-turn input to the actuators,
 * followed by a constant right-turn.
 *
 * @note This function will not run asynchronously and will block the calling
 * task until the identification has completed. If this function is called from
 * the control loop task, then it will not run for the duration of this
 * identification.
 *
 * @param[out] limits Location where the identified limits should be written
 * @returns true if the identification was successful, false in case of error
 */
bool steer_identification_find_limits(struct PotentiometerLimits *limits);
