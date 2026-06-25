/*******************************************************************************
 * @file   velocity-estimator.cpp
 * @brief  Estimates the velocity of the car based on wheel encoder readings.
 ******************************************************************************/

#include "velocity-estimator.hpp"

namespace chronos {
namespace estimation {

bool LongitudinalVelocityEstimator::step(LongitudinalVelocity &velocity,
                                         const WheelSpeed &wheel_speed) {
    // A naive estimate of longitudinal velocity using the rear wheel speed:
    // ω_(rr) = (vx + 0.5 * width * dyaw) / r
    // ω_(rl) = (vx - 0.5 * width * dyaw) / r
    // => Summed: ω_(rr) + ω_(rl) = 2 * vx / r
    float rear_average_wheel_speed =
        (wheel_speed.back_left_wheel + wheel_speed.back_right_wheel) / 2.0f;
    velocity = rear_average_wheel_speed * wheel_radius_;

    return true;
}

} // namespace estimation
} // namespace chronos
