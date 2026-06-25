#ifndef SENSOR_MODELS_SENSOR_MODEL_H
#define SENSOR_MODELS_SENSOR_MODEL_H

#include <Eigen/Core>
#include <string>

#include "commons/casadi_utils.h"
#include <casadi/casadi.hpp>

#include <dynamic_models/utils/data_conversion.h>

namespace crs_sensor_models
{
template <typename StateType, typename InputType>
class SensorModel
{
public:
  SensorModel(int dimension, std::string key) : dimension(dimension), sensor_model_key(key)
  {
    state_mx = commons::asCasadiSym<StateType>();
    input_mx = commons::asCasadiSym<InputType>();

    // Copy state and input into one vector
    state_and_input_mx = {};
    state_and_input_mx.insert(state_and_input_mx.end(), state_mx.begin(), state_mx.end());
    state_and_input_mx.insert(state_and_input_mx.end(), input_mx.begin(), input_mx.end());
  }
  /**
   * @brief Evaluates the measurement model at the given state
   *
   * @param state
   * @param input
   * @return double vector of measruements
   */
  virtual Eigen::Matrix<double, Eigen::Dynamic, 1> applyModel(const StateType& state, const InputType& input)
  {
    Eigen::MatrixXd measured_state;
    measured_state.resize(dimension, 1);
    measurement_function(commons::convertToConstVector(state, input), commons::convertToVector(measured_state));

    return measured_state;
  }

  /**
   * @brief Get the Numerical Jacobian for a given state
   *
   * @param state current state
   * @param input current input
   * @param H the state jacobian dh/dx
   */
  void getNumericalJacobian(const StateType& state, const InputType& input,
                            Eigen::Matrix<double, Eigen::Dynamic, StateType::NX>& H)
  {
    assert((H.rows() == dimension) && "Return by reference jacobian H has incorrect number of rows");
    Eigen::MatrixXd full_jacobian = measurement_function.evaluateJacobian(commons::convertToConstVector(state, input));
    H = full_jacobian.block(0, 0, dimension, StateType::NX);
  }

  /**
   * @brief Get the Algebraic Jacobian as casadi function.
   *
   * @return casadi::Function
   */
  virtual casadi::Function getSymbolicJacobian()
  {
    return measurement_function.getSymbolicJacobian();
  }

  /**
   * @brief Sets the measurement noise covariance matrix
   *
   * @param R
   */
  void setR(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& R)
  {
    assert((R.rows() == dimension) && "Observation noise covariance R has incorret number of rows");
    assert((R.cols() == dimension) && "RObservation noise covariance R has incorret number of cols");
    R_ = R;
  }

  /**
   * @brief Returns the measurement noise covariance matrix
   *
   * @return const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
   */
  const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> getR()
  {
    return R_;
  }
  const unsigned int dimension;

  /**
   * @brief Returns the key for this sensor model (e.g. mocap, imu, ....)
   *
   * @return std::string
   */
  std::string getKey()
  {
    return sensor_model_key;
  }

protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> R_;

  commons::CasadiFunction measurement_function;

  // Symbolic state variables
  std::vector<casadi::MX> state_mx;
  // Symbolic input variables
  std::vector<casadi::MX> input_mx;
  // Symbolic state and input variables
  std::vector<casadi::MX> state_and_input_mx;

private:
  std::string sensor_model_key;
};

}  // namespace crs_sensor_models
#endif
