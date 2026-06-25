/*******************************************************************************
 * @file    actuator_interface.hpp
 * @brief   Describes the abstract actuator interface.
 ******************************************************************************/

#pragma once

namespace chronos {

namespace interface {

/**
 * @brief Abstract interface to an actuator.
 */
class Actuator {
  public:
    /**
     * @brief Enable the actuator.
     *
     * Enabling an actuator is the prerequisite to enabling power supply to it.
     * This does snot change the power supplied to the actuator.
     *
     * @returns true if the actuator was successfully enabled
     */
    virtual bool enable() = 0;

    /**
     * @brief Disable the actuator.
     *
     * This does not deinitialize the actuator, but it disables it from
     * electrically receiving power. For some motors, this means that the motor
     * will coast.
     *
     * Does not change the power level that the actuator should follow in case
     * it is turned on, reenabling it will resume at the preivously set power
     * point.
     * @returns true if the motor driver is successfully disabled, false
     * otherwise
     */
    virtual bool disable() = 0;

    /*
     * @brief Set the amount and direction of power supplied to the motor.
     *
     * The maximum power that is delivered to the actuator depends on the
     * voltage that the motor driver chip is supplied with. However, the power
     * that is supplied at a particular instant can be varied in between the
     * maximum power and no power.
     *
     * The direction of the current flow (changed through the motor phase)
     * changes the turning direction of the motor.
     *
     * Both of these can be changed by setting the @p power parameter: for
     * values larger than zero, the phase will be set to logic high, and for
     * lower than zero it will be set to logic low.
     *
     * The amount of power to be supplied to the motor is given as a percentage,
     * so the range of valid inputs to this function is [-100.0f, +100.0f]. The
     * implementing actuator may discard or clip the input if it exceeds these
     * limits.
     */
    virtual bool set_power(float power) = 0;
};

}; // namespace interface

}; // namespace chronos
