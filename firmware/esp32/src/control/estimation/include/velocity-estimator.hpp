/*******************************************************************************
 * @file   velocity-estimator.hpp
 * @brief  Estimates the velocity of the car based on wheel encoder readings.
 ******************************************************************************/

#pragma once

#include "control-types.h"
#include "estimator_interface.hpp"

namespace chronos {
namespace estimation {

class LongitudinalVelocityEstimator
    : public interface::Estimator<LongitudinalVelocity, WheelSpeed> {
  public:
    /**
     * @brief Construct a new Longitudinal Velocity Estimator object
     * @param wheel_radius Radius of the wheel in [m].
     */
    LongitudinalVelocityEstimator(float wheel_radius)
        : wheel_radius_(wheel_radius){};

    /**
     * @brief Estimate the longitudinal velocity of the car.
     *
     * @param[out] velocity The estimated velocity.
     * @param[in] wheel_speed The wheel speed measurements.
     * @returns true if the estimation was successful, false otherwise
     */
    bool step(LongitudinalVelocity &velocity, const WheelSpeed &wheel_speed);

  private:
    const float wheel_radius_; ///< [m] radius of the wheel
};

} // namespace estimation
} // namespace chronos
