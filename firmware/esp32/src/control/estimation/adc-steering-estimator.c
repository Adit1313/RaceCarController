/*******************************************************************************
 * @file    adc-steering-estimator.c
 * @brief   Steering angle estimator based solely on a potentiometer reading.
 ******************************************************************************/

#include "adc-steering-estimator.h"

#include <math.h>

/* Local variables ---------------------------------------------------------- */

/* Public function implementation ------------------------------------------- */

SteeringAngle estimate_steer(const ADCMeasurement left_limit,
                             const ADCMeasurement right_limit,
                             const SteeringAngle amplitude,
                             const ADCMeasurement measurement) {
    // Check preconditions in order to not divide by zero
    if (left_limit - right_limit != 0 && amplitude > 0.0f) {
        // remap [right_limit, left_limit] to [-amplitude, amplitude]
        float normalized_position =
            (measurement - right_limit) / (float)(left_limit - right_limit);
        float angle = 2 * amplitude * normalized_position - amplitude;

        return angle;
    } else {
        // Existence of NAN is not guaranteed by C99, but INFINITY is.
#ifdef NAN
        return NAN;
#else
        return INFINITY;
#endif /* NAN */
    }
}
