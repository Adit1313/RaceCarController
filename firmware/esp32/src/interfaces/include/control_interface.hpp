/*******************************************************************************
 * @file    control_interface.hpp
 * @brief   Describes the abstract interface for controllers.
 ******************************************************************************/

#pragma once

namespace chronos {

namespace interface {

/**
 * @brief Abstract interface to a controller.
 *
 * @tparam U The type of the controller's output.
 * @tparam X The type of the variable to be controlled.
 */
template <typename U, typename X> class Controller {
  public:
    /**
     * @brief Perform one iteration of the control loop.
     *
     * @param output The output of the controller.
     * @param setpoint The desired value of the process variable.
     * @param process_variable The current value of the process variable.
     */
    virtual bool step(U &output, const X &setpoint,
                      const X &process_variable) = 0;

    /**
     * @brief Reset the controller's internal state.
     */
    virtual void reset() = 0;
};

} // namespace interface

} // namespace chronos
