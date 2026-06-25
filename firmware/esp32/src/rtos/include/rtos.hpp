/*******************************************************************************
 * @file    rtos.hpp
 * @brief   Setup and handling of the RTOS functionality of the vehicle.
 ******************************************************************************/

#pragma once

#include "control.hpp"

#ifdef __cplusplus

#include <cstdint>

namespace chronos {

using Tick = uint32_t;

void rtos_launch(ControlLoop *control_loop);

Tick rtos_get_tick_count();

}; // namespace chronos

extern "C" {

#endif

/**
 * @brief Start iterative execution of the control loop.
 *
 * The control loop can be started using control_loop_start() and stopped using
 * control_loop_stop(). It can be started and stopped as often as required.
 */
void control_loop_start(void);

/**
 * @brief Stop iterative execution of the control loop.
 *
 * The control loop can be started using control_loop_start() and stopped using
 * control_loop_stop(). It can be started and stopped as often as required.
 */
void control_loop_stop(void);

/**
 * @brief Pause actuator control inputs.
 *
 * In contrast to @see control_loop_stop(), this function only pauses the
 * actuator control inputs. The control loop will continue to run and sample
 * sensor data.
 */
void control_loop_pause(void);

/**
 * @brief Resume actuator control inputs.
 *
 * In contrast to @see control_loop_start(), this function only resumes the
 * actuator control inputs. If the control loop was stopped using @see
 * control_loop_stop, call @see control_loop_start as well.
 */
void control_loop_resume(void);

#ifdef __cplusplus
}
#endif
