/*******************************************************************************
 * @file    actuator_interface.hpp
 * @brief   No-op implementation of various interfaces for testing.
 ******************************************************************************/

#pragma once

#include "actuator_interface.hpp"
#include "control_interface.hpp"
#include "estimator_interface.hpp"
#include "sensor_interface.hpp"

namespace chronos {

class NullActuator : public interface::Actuator {
  public:
    NullActuator(){};

    virtual bool enable() { return true; };
    virtual bool disable() { return true; };
    virtual bool set_power(float power) { return false; };
};

template <typename U, typename X>
class NullController : public interface::Controller<U, X> {
  public:
    virtual bool step(U &output, const X &setpoint,
                      const X &process_variable) override {
        return false;
    }

    virtual void reset() override{};
};

template <typename T> class NullSensor : public interface::Sensor<T> {

  public:
    NullSensor(){};
    virtual bool sample(T &sample) { return false; };
};

template <typename T, typename... Q>
class NullEstimator : public interface::Estimator<T, Q...> {

  public:
    NullEstimator(){};
    virtual bool step(T &estimate, const Q &...inputs) { return false; };
};

}; // namespace chronos
