#ifndef MHE_SOLVERS_PACEJKA_MHE_SOLVER_H
#define MHE_SOLVERS_PACEJKA_MHE_SOLVER_H

#include "mhe_solver.h"

#include <pacejka_model/pacejka_params.h>
#include <pacejka_model/pacejka_car_state.h>

#include <Eigen/Core>

namespace mhe_solvers
{
namespace pacejka_solvers
{
/**
 * @brief Enum to easily reference a specific entry of the state array
 *
 */
enum vars
{
  X,
  Y,
  YAW,
  VX,
  VY,
  dYAW,
};

/**
 * @brief Enum to easily reference a specific entry of the parameter array
 * p = vertcat(lr, lf, m, I, Df, Cf, Bf, Dr, Cr, Br, Cm1, Cm2, Cd0, Cd1, Cd2, car_width, wheel_radius, T, delta, gamma,
 * eps, lighthouse parameters)
 */
enum params
{

  L_REAR,
  L_FRONT,
  m,
  I,
  Df,
  Cf,
  Bf,
  Dr,
  Cr,
  Br,
  Cm1,
  Cm2,
  Cd0,
  Cd1,
  Cd2,
  car_width,
  wheel_radius,
  torque,
  steer,
  gamma,
  eps,

  sensor_pos_1x,
  sensor_pos_2x,
  sensor_pos_3x,
  sensor_pos_4x,

  sensor_pos_1y,
  sensor_pos_2y,
  sensor_pos_3y,
  sensor_pos_4y,

  bs_position_x,
  bs_position_y,
  bs_position_z,

  bs_rotation_00,
  bs_rotation_01,
  bs_rotation_02,
  bs_rotation_10,
  bs_rotation_11,
  bs_rotation_12,
  bs_rotation_20,
  bs_rotation_21,
  bs_rotation_22,

  light_plane_tilt_1,
  light_plane_tilt_2,
};

/**
 * @brief Struct containing the information for a reference point on the track
 *
 */
struct references
{
  /**
   * @brief The state estimate from the ekf
   *
   */
  crs_models::pacejka_model::pacejka_car_state state;

  /**
   * @brief Measurement of the mocap system (x,y,yaw)
   */
  Eigen::Vector3d mocap_measurement;

  /**
   * @brief Measurement of the imu system (ax,ay,dyaw)
   */
  Eigen::Vector3d imu_measurement;

  /**
   * @brief Measurement of the imu yaw rate  system (dyaw)
   */
  Eigen::Matrix<double, 1, 1> imu_yaw_rate_measurement;

  /**
   * @brief Measurement of the wheel encoders system (w_fl, w_fr, w_rl, w_rr)
   */
  Eigen::Vector4d wheel_encoder_measurement;

  /**
   * @brief Measurement of the first sweep of lighthouse system (4 angles)
   */
  Eigen::Matrix<double, 4, 1> lighthouse_sweep_1_measurement;

  /**
   * @brief Measurement of the second sweep of lighthouse system (4 angles)
   */
  Eigen::Matrix<double, 4, 1> lighthouse_sweep_2_measurement;

  /**
   * @brief Applied input system (steer,torque)
   */
  Eigen::Vector2d input;

  /**
   * @brief Bool determining wether mocap can be used
   */
  bool valid_mocap;

  /**
   * @brief Bool determining wether imu can be used
   */
  bool valid_imu;

  /**
   * @brief Bool determining wether imu can be used
   */
  bool valid_imu_yaw_rate;

  /**
   * @brief Bool determining wether wheel encoders can be used
   */
  bool valid_wheel_encoders;

  /**
   * @brief Bool determining wether lighthouse_sweep_1 can be used
   */
  bool valid_lighthouse_sweep_1;

  /**
   * @brief Bool determining wether lighthouse_sweep_2 can be used
   */
  bool valid_lighthouse_sweep_2;
};

/**
 * @brief Struct containing the costs of the mpc problem
 *
 */
struct cost_values
{
  /**
   * @brief EKF State Covariance Matrix
   *
   */
  Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                crs_models::pacejka_model::pacejka_car_state::NX>
      P;

  /**
   * @brief Process Noise Matrix
   *
   */
  Eigen::Matrix<double, crs_models::pacejka_model::pacejka_car_state::NX,
                crs_models::pacejka_model::pacejka_car_state::NX>
      Q;

  /**
   * @brief Mocap Measurement Noise Matrix
   *
   */
  Eigen::Matrix3d R_mocap;

  /**
   * @brief IMU Measurement Noise Matrix
   *
   */
  Eigen::Matrix3d R_imu;

  /**
   * @brief IMU yaw rate Measurement Noise Matrix
   *
   */
  Eigen::Matrix<double, 1, 1> R_imu_yaw_rate;

  /**
   * @brief Wheel encoders Measurement Noise Matrix
   *
   */
  Eigen::Matrix4d R_wheel_encoders;

  /**
   * @brief Lighthouse Measurement Noise Matrix
   *
   */
  Eigen::Matrix<double, 4, 4> R_lighthouse;

  /**
   * @brief Discount factor for the MHE. This determines how heavily older measurements influence the estimate.
   * 0 < Eta < 1, a value of 0.99 means all measurements are weighted approximatly equally.
   *
   */
  double eta;
};

class PacejkaMheSolver : public MheSolver<crs_models::pacejka_model::pacejka_params, cost_values, references>
{
public:
  /**
   * @brief Get the Dimension of the state
   *
   * @return int
   */
  int getStateDimension() const override
  {
    return 6;
  }

  /**
   * @brief Get the Dimension of the input
   *
   * @return int
   */
  int getInputDimension() const override
  {
    return 6;  // inputs are now process noise values for each state (w_x, w_y, ...)
  }

  /**
   * @brief Get the Horizon Length
   *
   * @return int
   */
  virtual int getHorizonLength() const = 0;

  virtual void removeInitialState() = 0;
  /**
   * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  virtual void setInitialState(double constraint[]) = 0;
  /**
   * @brief Sets an initial guess for the state at stage "stage" of the solver.
   * The provided array must have the same length as the state dimension
   *
   * @param constraint
   */
  virtual void setStateInitialGuess(int stage, double constraint[]) = 0;
  /**
   * @brief Sets an initial guess for the input at stage "stage" of the solver.
   * The provided array must have the same length as the input dimension
   *
   * @param constraint
   */
  virtual void setInputInitialGuess(int stage, double constraint[]) = 0;

  /**
   * @brief Updates the internal params for stage "stage"
   *
   * @param stage which stage to update the parameter [0,HorizonLength)
   * @param model_dynamics  the model dynamics
   * @param costs  the cost parameters
   * @param references references measurements/inputs.
   *  In case of missing measurements, you can set references.valid_mocap = false or references.valid_imu = false to
   * ignore them.
   */
  virtual void updateParams(
      int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics, const cost_values& costs,
      const references& references,
      std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_1,
      std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_2) = 0;

  /**
   * @brief Solves the optimization problems and stores the solution in x and u.
   *
   * @param x State array or point with size N*StateDimenstion
   * @param u Input array or point with size N*Inputdimension
   * @return int, return code. If no error occurred, return code is zero
   */
  virtual int solve(double x[], double u[]) = 0;
};

}  // namespace pacejka_solvers
}  // namespace mhe_solvers

#endif /* MHE_SOLVERS_PACEJKA_MHE_SOLVER_H */
