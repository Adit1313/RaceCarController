/*******************************************************************************
 * @file    pid.hpp
 * @brief   Generic PID controller implementation.
 ******************************************************************************/

#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>

#include "control_interface.hpp"

namespace chronos {
namespace control {

static const char *TAG = "pid";

/**
 * @brief Abstract PID controller implementation.
 *
 * The PID controller implemented in this class is a template that can be used
 * to control a variety of process variables. It implements the parallel form of
 * the PID:
 *
 * u = kp * e + ki * ∫ e dt + kd * d/dt (e)
 *
 * @note Make sure that the relevant operators are defined on Input and State,
 * such as multiplication with the sample time, addition and multiplication with
 * floats for the controller's gains.
 *
 * @tparam Input The type of the controller's output.
 * @tparam State The type of the variable to be controlled.
 */
template <typename Input, typename State>
class PID : public interface::Controller<Input, State> {
  public:
    /**
     * Instantiate a PID controller.
     *
     * @param kp Proportional gain.
     * @param ki Integral gain.
     * @param kd Derivative gain.
     * @param Ts Sample time, not equal to zero.
     * @param Tf Filter time constant for the derivative.
     *
     * @note Will fail an assertion if @p Ts is zero.
     */
    constexpr PID(float kp, float ki, float kd, float Ts, float Tf = 0.0f)
        : kp_(kp), ki_(ki), kd_(kd), Ts_(Ts), kb_(0.3 * ki * Ts), Tf_(Tf) {
        assert(Ts_ != 0.0f);
    };

    /**
     * @brief Perform one iteration of the PID loop.
     *
     * @param output The output of the controller.
     * @param setpoint The desired value of the process variable.
     * @param process_variable The current value of the process variable.
     */
    virtual bool step(Input &output, const State &setpoint,
                      const State &process_variable) override {
        // Calculate cut-off frequency for the derivative filter once
        static const float alpha = Tf_ != 0.0f ? expf(-Ts_ / Tf_) : 0.0f;

        State error;
        if (normalization_divider_ != 1.0f) {
            error = (setpoint - process_variable) / normalization_divider_;
        } else {
            error = setpoint - process_variable;
        }

        // Propagate error integral prior to controller evaluation.
        // Filter derivative term using an exponential moving average.
        error_integral_ += error * Ts_;
        error_derivative_ = alpha * error_derivative_ +
                            (1 - alpha) * (error - error_previous_) / Ts_;

        Input output_raw =
            kp_ * error + ki_ * error_integral_ + kd_ * error_derivative_;

        output =
            std::clamp(output_raw, -output_saturation_, output_saturation_);

        // Anti wind-up: Subtract from the intgral term if the output is
        // saturated.
        error_integral_ -= (output - output_raw) * kb_ * 1e-3f;

        // Propagate error for next iteration
        error_previous_ = error;

        return true;
    }

    /**
     * @brief Reset the controller's error integral.
     */
    virtual void reset() override {
        error_integral_ = 0;
        error_previous_ = 0;
        error_derivative_ = 0;
    }

    virtual void set_normalization_divider(float normalization_divider) {
        normalization_divider_ = normalization_divider;
    }

  private:
    const float kp_;
    const float ki_;
    const float kd_;

    const float Ts_;

    /** Back-calculation gain for the integral term. */
    const float kb_;

    /** Filter time for the derivative term. */
    const float Tf_;

    /* Advanced configuration of the PID controller. ------------------------ */

    /** Divider for the normalization of setpoint and process variable. */
    float normalization_divider_ = 1.0f;

    /** Maximum value of the output  */
    const float output_saturation_ = 1.0f;

    /* State variables of the PID controller -------------------------------- */

    State error_integral_ = 0;
    State error_previous_ = 0;
    State error_derivative_ = 0;
};

} // namespace control
} // namespace chronos
