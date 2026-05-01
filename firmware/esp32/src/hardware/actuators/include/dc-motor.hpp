/*******************************************************************************
 * @file    dc-motor.hpp
 * @brief   Driver for a DC motor, based on the MCPWM drivers of ESP-IDF
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "actuator_interface.hpp"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

/**
 * @addtogroup actuators
 * @{
 */

/* Public type definitions -------------------------------------------------- */

namespace chronos {

/**
 * @brief DC motor driven by a motor driver chip.
 *
 * The DC motor which is supported by this driver is controlled with three pins:
 * - A PWM duty cycle, which is proportional to the voltage applied
 * - A phase, which changes the direction the motor is turning
 * - An enable/fault pin, which can be pulled low to disable powering the motor.
 *
 * A usual application could look like this:
 * - dcmotor_configure() to set up the motor
 * - dcmotor_enable() to enable the motor
 * - dcmotor_set_power_and_phase() to supply power and/or change the direction
 */
struct DCMotorConfig {

    gpio_num_t pwm_pin;      ///< PWM pin, duty cycle is proportional to power
    gpio_num_t phase_pin;    ///< Phase pin to change direction of motor.
    gpio_num_t en_fault_pin; ///< Pulled low in case of error or to disable

    /** Which of the two MCPWM units to use, 0 or 1. Must be unique to this
     * motor. */
    mcpwm_unit_t mcpwm_unit;

    /** Which of the timers to use, 0, 1, or 2. */
    mcpwm_timer_t mcpwm_timer;

    uint32_t pwm_freq; ///< Frequency of the PWM signal output

    /**
     * Whether to let the motor coast if the power supplied to it is zero.
     * For a driving motor, it could be desirable to let it coast. For a
     * steering actuator, the resistance in holding torque could be useful, in
     * which case the motor should not actually be disabled if no power is
     * supplied.
     */
    bool auto_coasting;

    /** If you want to change the phase mapping (power < 0 => phase == 1) to
     * power < 0 => phase == 0, set to true. */
    bool reverse_phase;

    /**
     * @brief Callback that is invoked if the motor driver indicates a fault.
     * @note Place in IRAM for best performance! (Use @c IRAM_ATTR ).
     * @note Will be called in ISR context.
     * @param pwm_unit The PWM unit that encountered a fault is passed to the
     * handler.
     */
    void (*fault_callback)(mcpwm_unit_t pwm_unit);
};

class DCMotor : public interface::Actuator {
  public:
    DCMotor(const DCMotorConfig &config);

    bool enable();
    bool disable();

    /**
     * @brief Set the amount and direction of power supplied to the motor.
     *
     * In the case of the DC motors on the vehicle, it not actually the power
     * that is set but the voltage applied to the motor. The power then is also
     * subject to current limits.
     *
     * @note If coasting is enabled (DCMotor.auto_coasting), then specifying
     * @p power equal to zero will disable the motor. In turn, this also means
     * that if @p power is larger than zero, the motor will be enabled.
     *
     * @note Will disable motor if its @c auto_coasting is set to true.
     * @param power Percentage of power in range [-100, 100] to supply to motor.
     *        Clipped to [-100, 100] in case the input exceeds these limits.
     * @returns true if the power point could successfully be changed.
     */
    bool set_power(float power);

  private:
    const chronos::DCMotorConfig config_;

    /** Whether the motor is currently enabled or not. If disabled, the
     *  MCU will pull the enable/fault line of the motor driver chip low. */
    bool enabled_ = false;
};

}; // namespace chronos

/* Public function declarations --------------------------------------------- */

/**
 * @brief Configure the motor.
 *
 * The motor driver will set up the timers and the registers required to output
 * phase, enable and PWM signals to the DC motor.
 * Initially, the motor will be disabled, meaning that the enable pin is driven
 * to logic low level. To start supplying power to the motor, call
 * dcmotor_set_power().
 *
 * @param motor Pointer to struct containing the configuration.
 * @returns true if the configuration succeeded, false otherwise.
 */
bool dcmotor_configure(struct DCMotor *motor);

/**
 * @brief Enable the motor.
 *
 * This does not change the power supplied to the motor, but simply releases the
 * EN pin of the motor to be pulled high by an external pull resistor.
 *
 * @param motor The motor to enable.
 * @returns true if the motor enable pin was successfully set high
 */
bool dcmotor_enable(struct DCMotor *motor);

/**
 * @brief Disable the motor.
 *
 * This does not deinitialize the motor, but it disables the motor and lets it
 * coast. If no call to dcmotor_set_power was made before calling dcmotor_enable
 * again, the motor will resume at the previously set power point.
 * @param motor The motor to disable.
 * @returns true if the motor driver is successfully disabled, false otherwise
 */
bool dcmotor_disable(struct DCMotor *motor);

bool dcmotor_set_power(struct DCMotor *motor, float power);

/** @} */
