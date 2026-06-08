#ifndef SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL
#define SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL
#include <Eigen/Core>
#include <casadi/casadi.hpp>
#include <iostream>

namespace crs_models
{
template <typename StateType, typename InputType>
class DiscreteDynamicModel
{
public:
  typedef Eigen::Matrix<double, StateType::NX, StateType::NX> StateMatrix;
  typedef Eigen::Matrix<double, StateType::NX, InputType::NU> InputMatrix;

  /** State and input dimensions of the Model*/
  static const int NX = StateType::NX;
  static const int NU = InputType::NU;

  DiscreteDynamicModel(Eigen::Matrix<double, StateType::NX, StateType::NX> Q) : Q_(Q){};

  /**
   * @brief Calculates the state of the system after evolving for a certain integration_time
   *
   * @param state starting state
   * @param control_input input to the system
   * @param integration_time timestep to integrate
   * @param output_state the output state
   */
  virtual StateType applyModel(const StateType state, const InputType control_input, double integration_time) = 0;

  /**
   * @brief Gets the jacobian of the discrete system
   *
   * @param state starting state
   * @param control_input control input that should be applied
   * @param integration_time integration_time
   * @param A df/dx of the discrete system
   * @param B df/du of the discrete system
   */
  virtual void getJacobian(const StateType& state, const InputType& control_input, double integration_time,
                           StateMatrix& A, InputMatrix& B) = 0;
  /**
   * @brief Sets the Process Noise Covariance Matrix associated with these dynamics.
   * Note that the Unit of Q is 1/s.
   *
   * @param Q Process Noise Covariance Matrix
   */
  void setQ(const Eigen::Matrix<double, StateType::NX, StateType::NX>& Q)
  {
    Q_ = Q;
  }

  /**
   * @brief Returns the Process Noise Covariance Matrix associated with these dynamics.
   * Note that the Unit of Q is 1/s.
   *
   * @return const Eigen::Matrix<double, StateType::NX, StateType::NX>
   */
  const Eigen::Matrix<double, StateType::NX, StateType::NX> getQ()
  {
    return Q_;
  }

private:
  Eigen::Matrix<double, StateType::NX, StateType::NX> Q_;
};
}  // namespace crs_models
#endif /* SRC_CRS_DYNAMIC_MODELS_COMMON_INCLUDE_DYNAMIC_MODELS_DISCRETE_DYNAMIC_MODEL */
