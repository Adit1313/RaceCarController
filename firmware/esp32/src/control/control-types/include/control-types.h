/*******************************************************************************
 * @file    control-types.hpp
 * @brief   Defines types for common quantities used by sensors, controllers and
 *          estimators.
 ******************************************************************************/

#pragma once

#include <stdint.h>

/* Quantity types ----------------------------------------------------------- */

/** A linear acceleration of a body [m/s^2]. */
struct LinearAcceleration {
    float x;
    float y;
    float z;
};

/** An angular velocity of a body [rad/s] */
struct AngularVelocity {
    float x;
    float y;
    float z;
};

/**
 * @brief Steering angle of a car [rad].
 *
 * By convention, the steering angle is defined by a rotation in the positive
 * direction around the z-axis, which points up.
 * This results in:
 *
 * - steering angle > 0 <=> car steers to left
 * - steering angle < 0 <=> car steers to right
 */
typedef float SteeringAngle;

/**
 * @brief Throttle torque applied to the steering wheels.
 *
 * This throttle is dimensionless and restricted to an interval of [-1, 1].
 * Positive values indicate *forward* acceleration of the vehicle, and vice
 * versa for negative values.
 */
typedef float Throttle;

typedef float LongitudinalVelocity;

/**
 * @brief Control input that correspodns to PWM duty cycle.
 */
typedef float PWMDutyCycle;

/* Measurement types -------------------------------------------------------- */

/** An inertial measurement in SI units. */
typedef struct InertialMeasurement {
    struct LinearAcceleration linear_accel;
    struct AngularVelocity angular_vel;
} InertialMeasurement;

/**
 * @brief Type bundling power measurements from a car.
 */
typedef struct PowerDiagnostics {
    float voltage_batt;  ///< Battery voltage [V]
    float current_total; ///< Total current from battery [A]
} PowerMeasurement;

/*
 * @brief A wheel speed measurement, expressed in radians per second (rad/s).
 */
struct WheelSpeed {
    union {
        struct {
            float front_right_wheel;
            float front_left_wheel;
            float back_right_wheel;
            float back_left_wheel;
        };
        float wheel_speed[4];
    };
};

/** A raw ADC measurement. */
typedef uint16_t ADCMeasurement;
