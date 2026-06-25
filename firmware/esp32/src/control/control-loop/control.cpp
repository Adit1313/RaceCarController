/*******************************************************************************
 * @file    control.cpp
 * @brief   Low-level control loop task for steering and driving
 ******************************************************************************/

#include "control.hpp"

#include <math.h>
#include <stdbool.h>
#include <string.h>

extern "C" {
#include "adc-steering-estimator.h"
#include "adc.hpp"
#include "control-types.h"
#include "esp_log.h"
#include "math-util.h"
#include "nvs-manager.h"
}
#include "bang-bang.hpp"
#include "communication-layer.h"
#include "rtos.hpp"
#include "state-machine.h"

#include "Packet.pb.h"

/* Local macros ------------------------------------------------------------- */

/** Steering range that the car's wheels can reach (one-sided). */
#define STEERING_RANGE (global_config.steer_angle_deg / 180.0f * M_PI)
/** Lowest voltage [mV] that is measured by the poti (right-most position) */
#define DISCRETE_LOWER_BOUND (global_config.steer_limit_lower)
/** Highest voltage [mV] that is measured by the poti (left-most position) */
#define DISCRETE_UPPER_BOUND (global_config.steer_limit_upper)

/* Local variable declaration ----------------------------------------------- */

//! Tag used in logging.
static const char *TAG = "control-loop";

/* Private function declaration --------------------------------------------- */

/**
 * @brief Log all measurements to the console (if verbose logging is enabled).
 * @param state The state containing the measurements
 */
static void log_measurements(const CarState *state);

/* Public function implementation ------------------------------------------- */

namespace chronos {

void ControlLoop::step() {

    Packet p = Packet_init_zero;
    p.which_contents = Packet_car_state_tag;
    CarState *state = &(p.contents.car_state);

    /** Method-static variable containing the current reference to track. */
    static struct Reference current_ref {};

    float steer_input = 0.0f;
    float throttle_input = 0.0f;

    InertialMeasurement imu_meas;
    WheelSpeed wheel_meas;
    LighthouseSweepList lighthouse_sweep_list;
    SteeringAngle steer_angle = 0.0f;
    LongitudinalVelocity speed;

    // Try to read the reference that may have been updated. If it is currently
    // being updated by another task, don't wait and use the previous reference.
    // At the end of the loop, there is another opportunity to update.
    if (reference_semaphore_.acquire(0)) {
        current_ref = reference_;
        reference_semaphore_.release();
    }

    // Detect if reference has not been updated in a while - and tell the
    // FSM that it is time to go to power-save mode
    if (chronos::rtos_get_tick_count() - last_update_tick_ >
        pdMS_TO_TICKS(CONFIG_DISCONNECT_THRESHOLD_MS)) {
        current_ref = {};

        steer_motor_.set_power(0.0f);
        throttle_motor_.set_power(0.0f);
        state_machine_input(PauseOperation);
    }

    // Constrain reference to the range the car can reach
    if (current_ref.steer_ref.type == SteeringReference::SteeringAngle) {
        current_ref.steer_ref.steer_angle = clampf(
            current_ref.steer_ref.steer_angle, -STEERING_RANGE, STEERING_RANGE);
    }

    // Sample sensors and current steering position
    ADCMeasurement discrete_steer_measurement;
    ADCMeasurement battery_voltage;
    ADCMeasurement total_current;
    PowerMeasurement power_meas;
    imu_.sample(imu_meas);
    steer_feedback_.sample(discrete_steer_measurement);
    state->has_wheel_speed_data = wheel_encoders_.sample(wheel_meas);
    lighthouse_.sample(lighthouse_sweep_list);
    state->has_battery_state =
        battery_voltage_sensor_.sample(battery_voltage) &&
        total_current_sensor_.sample(total_current);

    steer_angle = estimate_steer(DISCRETE_UPPER_BOUND, DISCRETE_LOWER_BOUND,
                                 STEERING_RANGE, discrete_steer_measurement);

    velocity_estimator_.step(speed, wheel_meas);

    if (actuator_control_enabled_) {
        // Calculate next input to the actuators
        if (current_ref.steer_ref.type == SteeringReference::SteeringAngle) {
            if (global_config.pid_enabled) {
                steer_controller_.step(steer_input,
                                       current_ref.steer_ref.steer_angle,
                                       steer_angle);
            } else {
                control_steer_step_bangbang(current_ref.steer_ref.steer_angle,
                                            steer_angle, &steer_input);
            }

        } else if (current_ref.steer_ref.type == SteeringReference::Voltage) {
            steer_voltage_controller_.step(steer_input,
                                           current_ref.steer_ref.steer_voltage,
                                           discrete_steer_measurement);
        }

        if (current_ref.throttle_ref.type == ThrottleReference::Throttle) {
            control_drive_step(current_ref.throttle_ref.throttle,
                               &throttle_input);
        } else if (current_ref.throttle_ref.type ==
                   ThrottleReference::LongitudinalVelocity) {
            // Because low speeds cannot be estimated well, threshold the speed
            auto current_speed = speed < 0.1f ? 0.0f : speed;
            velocity_controller_.step(
                throttle_input, current_ref.throttle_ref.longitudinal_velocity,
                current_speed);
        }
    } else {
        // System is inoperational – don't apply any input at all.
        // Also, the controllers are not updated and their internal states
        // are reset.
        throttle_input = 0.0f;
        steer_input = 0.0f;

        steer_controller_.reset();
        steer_voltage_controller_.reset();
        velocity_controller_.reset();
    }

    // Update input PWM to motor drivers
    steer_motor_.set_power(steer_input * 100.0f);
    throttle_motor_.set_power(throttle_input * 100.0f);

    // Send system state to host: sensor samples, inputs, etc.
    state->steer_data.adc_meas = discrete_steer_measurement;
    state->steer_data.steer_rad = steer_angle;

    state->steer_motor_input.power = steer_input;
    state->drive_motor_input.power = throttle_input;

    state->current_reference.torque_ref = current_ref.throttle_ref.throttle;
    if (current_ref.steer_ref.type == SteeringReference::SteeringAngle) {
        state->current_reference.steer_input.which_input =
            SteerInput_steer_angle_tag;
        state->current_reference.steer_input.input.steer_angle =
            current_ref.steer_ref.steer_angle;
    } else {
        state->current_reference.steer_input.which_input =
            SteerInput_steer_voltage_tag;
        state->current_reference.steer_input.input.steer_voltage =
            current_ref.steer_ref.steer_voltage;
    }
    // for compatibility with old versions
    state->current_reference.steer_ref = current_ref.steer_ref.steer_angle;

    state->imu_data.linear_acceleration = (Vector3){
        imu_meas.linear_accel.x,
        imu_meas.linear_accel.y,
        imu_meas.linear_accel.z,
    };
    state->imu_data.angular_velocity = (Vector3){
        imu_meas.angular_vel.x,
        imu_meas.angular_vel.y,
        imu_meas.angular_vel.z,
    };

    // Fill message fields that may not always be available. The wheel
    // encoders might be missing from this vehicle.
    if (state->has_wheel_speed_data) {
        state->wheel_speed_data = (WheelSpeedMeasurement){
            .front_right = wheel_meas.front_right_wheel,
            .front_left = wheel_meas.front_left_wheel,
            .back_right = wheel_meas.back_right_wheel,
            .back_left = wheel_meas.back_left_wheel,
        };
    }

    state->lighthouse_sweeps_count = lighthouse_sweep_list.length;
    for (size_t i = 0; i < lighthouse_sweep_list.length; i++) {
        state->lighthouse_sweeps[i].angle_0 =
            lighthouse_sweep_list.sweeps[i].angles[0];
        state->lighthouse_sweeps[i].angle_1 =
            lighthouse_sweep_list.sweeps[i].angles[1];
        state->lighthouse_sweeps[i].angle_2 =
            lighthouse_sweep_list.sweeps[i].angles[2];
        state->lighthouse_sweeps[i].angle_3 =
            lighthouse_sweep_list.sweeps[i].angles[3];
        state->lighthouse_sweeps[i].polynomial =
            lighthouse_sweep_list.sweeps[i].polynomial;
        state->lighthouse_sweeps[i].first_timestamp =
            lighthouse_sweep_list.sweeps[i].first_timestamp;
        state->lighthouse_sweeps[i].sync_timestamp =
            lighthouse_sweep_list.sweeps[i].sync_timestamp;
    }

    if (state->has_battery_state) {
        // Convert to SI units.
        // Battery voltage: divide by buffer gain of 2, reverse voltage div
        // (499k+499k vs. 110k)
        state->battery_state.voltage = battery_voltage / 2.0f * 1.0f /
                                       (110.0f / (499.0f + 499.0f + 110.0f)) /
                                       1000.0f;
        state->battery_state.has_current = true;
        state->battery_state.current =
            -(total_current * 1.0f - 40.0f) / 1000.0f;
    }

    state->has_longitudinal_velocity = true;
    state->longitudinal_velocity = speed;

    communication_layer_send(&p);
    log_measurements(state);
}

void ControlLoop::loop() {
    while (true) {
        timer_.wait_interval();
        step();
    }
}

void ControlLoop::start() { timer_.start(); }

void ControlLoop::pause_actuator_control() {
    actuator_control_enabled_ = false;
    steer_motor_.disable();
}

void ControlLoop::resume_actuator_control() {
    steer_motor_.enable();
    actuator_control_enabled_ = true;
}

void ControlLoop::stop() { timer_.stop(); }

bool ControlLoop::update_reference(const Reference &reference) {
    if (!actuator_control_enabled_)
        state_machine_input(Operate);

    if (reference_semaphore_.acquire(1)) {
        reference_ = reference;
        last_update_tick_ = chronos::rtos_get_tick_count();
        reference_semaphore_.release();
        return true;
    }
    return false;
}

} // namespace chronos

/* Private function implementation ----------------------------------------- */

static void log_measurements(const CarState *state) {
    ESP_LOGV(TAG, "w = %ld", state->steer_data.adc_meas);
    ESP_LOGV(TAG, "ax, ay, az = %.2f, %.2f, %.2f",
             state->imu_data.linear_acceleration.x,
             state->imu_data.linear_acceleration.y,
             state->imu_data.linear_acceleration.z);
    ESP_LOGV(TAG, "wx, wy, wz = %.2f, %.2f, %.2f",
             state->imu_data.angular_velocity.x,
             state->imu_data.angular_velocity.y,
             state->imu_data.angular_velocity.z);
}
