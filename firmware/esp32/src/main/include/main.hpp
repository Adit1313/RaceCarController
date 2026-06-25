/*******************************************************************************
 * @file    main.hpp
 * @brief   Main entry point for the Chronos Wi-Fi Car Firmware
 ******************************************************************************/

#pragma once

#include "actuator_interface.hpp"
#include "bmi088.hpp"
#include "control-types.h"
#include "control.hpp"
#include "lighthouse.hpp"
#include "sensor_interface.hpp"
#include "spi.hpp"
#include "time-synchronizer.hpp"
#include "uart.hpp"
#include "wheel-encoder.hpp"

/**
 * @brief Top-level namespaces for the firmware components of the Chronos car.
 */
namespace chronos {
/* Global, lazy-initialized objects ----------------------------------------- */

// Classes that bundle peripheral functionality or interact with the RTOS may
// depend on these external libraries being available on construction time.

// Lazy initialization of these ensures that they are only constructed on the
// first function call.

/**
 * @brief Returns a reference to the control loop singleton which runs the
 *        controllers, samples the sensors, etc.
 */
ControlLoop &control_loop();

/**
 * @brief Time synchronization singleton
 *
 * The returned singleton manages the synchronization of the system clock with
 * a remote NTP server.
 *
 * It is assumed that the "host IP" address matches the address of the NTP
 * server.
 */
time::TimeSynchronizer &time_synchronizer();

/** Returns a reference to the steering DC motor. */
interface::Actuator &steer_motor();
/** Returns a reference to the throttle DC motor. */
interface::Actuator &throttle_motor();
/** Returns a reference to the steer controller object. */
interface::Controller<PWMDutyCycle, SteeringAngle> &steer_controller();
/** Returns a reference to the steer voltage controller object. */
interface::Controller<PWMDutyCycle, float> &steer_voltage_controller();
/** Returns a reference to the velocity controller object. */
interface::Controller<PWMDutyCycle, LongitudinalVelocity> &
velocity_controller();
/** Returns a reference to the longitudinal velocity estimator. */
interface::Estimator<LongitudinalVelocity, WheelSpeed> &velocity_estimator();
/** Returns a reference to the inertial measurement unit. */
interface::Sensor<InertialMeasurement> &imu();
/** Returns a reference to the wheel encoder sensor. */
interface::Sensor<WheelSpeed> &wheel_encoders();
/** Returns a reference to the steering feedback sensor. */
interface::Sensor<ADCMeasurement> &steer_feedback();
/** Returns a reference to the Lighthouse sensor. */
interface::Sensor<chronos::LighthouseSweepList> &lighthouse();
interface::Sensor<ADCMeasurement> &battery_voltage_sensor();
interface::Sensor<ADCMeasurement> &total_current_sensor();

/** Reference to the UART0 bus. */
peripheral::UARTPeripheral &uart0();
/** Reference to the SPI2 bus master. */
SPIMasterBus &spi2();

/** Reference to the SPI device interface to the accelerometer on SPI2. */
SPIDevice &bmi_accel_device();
/** Reference to the SPI device interface to the gyroscope on SPI2. */
SPIDevice &bmi_gyro_device();
/** Reference to the SPI device interface to the wheel encoder chip on SPI2. */
SPIDevice &wheel_encoder_device();

}; // namespace chronos
