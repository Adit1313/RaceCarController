/*******************************************************************************
 * @file    control.hpp
 * @brief   Low-level control loop task for steering and driving
 ******************************************************************************/

#pragma once

#include <cstdint>

#include "lighthouse.hpp"

/** @defgroup control Control Loop */

/**
 * @addtogroup control
 * @brief The control loop which samples the sensors and calculates the control
 * input, which is then applied to the actuators.
 *
 * The control component contains the control loop, which can be started and
 * stopped. When it is started, the control loop always runs according to the
 * following schedule:
 *
 * 1. All onboard sensors are sampled.
 * 2. The new control reference is fetched from the global variable.
 * 3. The active controller iteration functions are invoked, which are provided
 *    with the current reference and the sensor measurements.
 * 4. The return value of the invoked controller functions, the plant inputs,
 *    are applied to the actuators.
 * 5. The sensor measurements and the control input are sent to the host
 *    computer.
 * 6. The control loop task delays until the sampling period has elapsed.
 *
 * The control loop exposes an API that can be used by the rest of the system to
 * manage its state: it can be started and stopped, and the reference it tries
 * to track can be changed. New references are received via Wi-Fi by different
 * components of the software, and they may use control_update_reference() to
 * make the control loop track a new one.
 * @{
 */

/* Public type declaration -------------------------------------------------- */

/**
 * @brief A reference for the throttle actuator of a car.
 */
struct ThrottleReference {

    /** Type of the reference, i.e., which of the fields of the union is valid*/
    enum ReferenceType {
        Throttle = 0,             ///< A normalized throttle torque [-1, 1]
        LongitudinalVelocity = 1, ///< A desired longitudinal velocity [m/s]
    } type = Throttle;

    union {
        /**
         * @brief The normalized torque in the range [-1, 1] applied on the
         * throttle
         *
         * The throttle torque is normalized to the range [-1, 1], because the
         * PWM signal used to drive the actuators through the actuator driver
         * chips can linearly vary the voltage applied to the motors, which
         * corresponds to the torque. The normalization to the range [-1, 1]
         * denotes a percentage of the maximum torque, which is applied.
         *
         * The negative range is reserved for reversing the car, while the
         * positive range is used to propel the car forward.
         */
        float throttle{0.0};

        /**
         * @brief The desired longitudinal velocity of the car in m/s.
         *
         * The longitudinal velocity is the velocity of the car in the
         * direction of the car's heading. It is measured in m/s.
         */
        float longitudinal_velocity;
    };
};

/**
 * @brief A steering reference that the car should track.
 *
 * A steering reference normally consists of the angle of the steering wheels.
 * However, in certain (mostly system identification) cases, a reference may
 * also be given in terms of the raw voltage of the feedback potentiometer to be
 * tracked.
 */
struct SteeringReference {
    /** Type of the reference, i.e., which of the fields of the union is valid*/
    enum ReferenceType {
        SteeringAngle = 0, ///< A steering angle reference [rad]
        Voltage = 1,       ///< A voltage reference [mV]
    } type = SteeringAngle;

    union {
        /**
         * @brief The steering angle of a vehicle in radians.
         *
         * The steering angle is measured in radians is defined as a positive
         * rotation around the yaw axis of the vehicle. This means that
         *
         * - angle > 0: car is steering to the left when driving forward
         * - angle < 0: car is steering to the right when driving forward
         */
        float steer_angle{0.0};
        uint16_t steer_voltage; ///< [mV] potentiometer reference
    };
};

/**
 * @brief A reference for a car that its low-level controller should track.
 */
struct Reference {
    /** The reference for the throttle actuator. */
    struct ThrottleReference throttle_ref {};

    /** The reference for the steering actuator. */
    struct SteeringReference steer_ref {};
};

/* Public function declaration ---------------------------------------------- */

#include "actuator_interface.hpp"
#include "configuration.h"
#include "control-types.h"
#include "control_interface.hpp"
#include "estimator_interface.hpp"
#include "periodic_timer.hpp"
#include "semaphore.hpp"
#include "sensor_interface.hpp"

namespace chronos {

class ControlLoop {

  public:
    ControlLoop(
        interface::Actuator &steer_motor, interface::Actuator &throttle_motor,
        interface::Controller<PWMDutyCycle, SteeringAngle> &steer_controller,
        interface::Controller<PWMDutyCycle, float> &steer_voltage_controller,
        interface::Controller<PWMDutyCycle, LongitudinalVelocity>
            &velocity_controller,
        interface::Estimator<LongitudinalVelocity, WheelSpeed>
            &velocity_estimator,
        interface::Sensor<ADCMeasurement> &steer_feedback,
        interface::Sensor<InertialMeasurement> &imu,
        interface::Sensor<WheelSpeed> &wheel_encoders,
        interface::Sensor<LighthouseSweepList> &lighthouse,
        interface::Sensor<ADCMeasurement> &battery_voltage_sensor,
        interface::Sensor<ADCMeasurement> &total_current_sensor)
        : steer_motor_(steer_motor), throttle_motor_(throttle_motor),
          steer_controller_(steer_controller),
          steer_voltage_controller_(steer_voltage_controller),
          velocity_controller_(velocity_controller),
          velocity_estimator_(velocity_estimator),
          steer_feedback_(steer_feedback), imu_(imu),
          wheel_encoders_(wheel_encoders), lighthouse_(lighthouse),
          battery_voltage_sensor_(battery_voltage_sensor),
          total_current_sensor_(total_current_sensor),
          timer_("control_timer", 1000 / CONFIG_CONTROL_LOOP_RATE){};

    /**
     * @brief Perform one iteration of the control loop.
     */
    void step();

    /**
     * Entry function to be called from a task to continuously execute the
     * control loop.
     * Does not return.
     */
    [[noreturn]] void loop();

    /** Starts the control loop, i.e., sampling and actuator control. */
    void start();
    /** Pauses the actuator control. */
    void pause_actuator_control();
    /** Resumes the actuator control. */
    void resume_actuator_control();
    /** Stops the control loop entirely, i.e. no samples will be sent. */
    void stop();

    bool update_reference(const Reference &reference);

  private:
    bool actuator_control_enabled_ = false;

    interface::Actuator &steer_motor_;
    interface::Actuator &throttle_motor_;

    interface::Controller<PWMDutyCycle, SteeringAngle> &steer_controller_;
    interface::Controller<PWMDutyCycle, float> &steer_voltage_controller_;
    interface::Controller<PWMDutyCycle, LongitudinalVelocity>
        &velocity_controller_;

    interface::Estimator<LongitudinalVelocity, WheelSpeed> &velocity_estimator_;

    interface::Sensor<ADCMeasurement> &steer_feedback_;
    interface::Sensor<InertialMeasurement> &imu_;
    interface::Sensor<WheelSpeed> &wheel_encoders_;
    interface::Sensor<LighthouseSweepList> &lighthouse_;

    interface::Sensor<ADCMeasurement> &battery_voltage_sensor_;
    interface::Sensor<ADCMeasurement> &total_current_sensor_;

    /** Current high-level reference that the control loop should track. */
    struct Reference reference_ {};

    rtos::PeriodicTimer timer_;

    /** Semaphore handling access to @p reference_. */
    rtos::Semaphore reference_semaphore_;

    uint32_t last_update_tick_ = 0;
};

} // namespace chronos

/**
 * @brief Sets a new reference to track.
 * @param new_ref Pointer to a non-null reference struct.
 * @param ticks_to_wait Maximum number of ticks to wait until timeout.
 * @returns true if the reference could be updated, false on error or timeout
 */
bool control_update_reference(struct Reference *new_ref,
                              TickType_t ticks_to_wait);

// The following two function definitions are not implemented in control-loop.c,
// but are free to be implemented in a user-provided file. Multiple
// implementations can be written, but only one can be linked against the
// control-loop component at any given time.

/**
 * @brief Iterate the steering controller.
 * @param r_steer The steering reference to follow
 * @param y_steer The current steering angle in radians
 * @param out Struct to write the next steering input to
 */
void control_steer_step(float r_steer, float y_steer, float *u_steer);

/**
 * @brief Iterate the driving controller.
 * @param r_drive The driving reference to follow
 * @param out Struct to write the next driving input to
 */
void control_drive_step(float r_drive, float *u_throttle);

/** @} */
