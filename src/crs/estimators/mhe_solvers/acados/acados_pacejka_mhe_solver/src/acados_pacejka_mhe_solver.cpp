#include "acados_pacejka_mhe_solver/acados_pacejka_mhe_solver.h"
#include <Eigen/LU>
#include <pacejka_sensor_model/lighthouse_sensor_model.h>

namespace mhe_solvers
{
namespace pacejka_solvers
{
/**
 * @brief Internal function to set initial guesses for the solver output variable
 *       This function just wraps the ocp_nlp_out_set call
 *
 * @param stage current stage of the mpc (0,....,horizon-1)
 * @param type type either "x" or "u"
 * @param constraint constraints must have same dimension as x or u (depending on type)
 */
void AcadosPacejkaMheSolver::setOutputInitialGuess(int stage, std::string type, double constraint[])
{
  ocp_nlp_out_set(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), stage, type.c_str(), constraint);
}
/**
 * @brief Sets an input bound constraint.
 *  This function just wraps the ocp_nlp_constraints_model_set call
 *
 * @param stage current stage of the mpc (0,....,horizon-1)
 * @param type type either "x" or "u"
 * @param constraint constraints must have same dimension as x or u (depending on type)
 */
void AcadosPacejkaMheSolver::setInputBoundConstraint(int stage, std::string type, double constraint[])
{
  ocp_nlp_constraints_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, type.c_str(), constraint);
}
/**
 * @brief Get the Last Solution and stores it in x and u
 *
 * @param x states, size state dimension x horizon length
 * @param u input, size input dimension x horizon length
 */
void AcadosPacejkaMheSolver::getLastSolution(double x[], double u[])
{
  for (int n = 0; n < getHorizonLength(); n++)
  {
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "x", &x[n * getStateDimension()]);  // NOLINT
    ocp_nlp_out_get(nlp_config_.get(), nlp_dims_.get(), nlp_out_.get(), n, "u", &u[n * getInputDimension()]);  // NOLINT
  }
}

/**
 * @brief Construct a new Acados Pacejka Mhe Solver:: Acados Pacejka Mhe Solver object
 *
 */
AcadosPacejkaMheSolver::AcadosPacejkaMheSolver()
{
  // initialize acados solver
  acados_ocp_capsule_.reset(pacejka_model_acados_create_capsule());
  pacejka_model_acados_create(acados_ocp_capsule_.get());

  nlp_config_.reset(pacejka_model_acados_get_nlp_config(acados_ocp_capsule_.get()));
  nlp_dims_.reset(pacejka_model_acados_get_nlp_dims(acados_ocp_capsule_.get()));
  nlp_in_.reset(pacejka_model_acados_get_nlp_in(acados_ocp_capsule_.get()));
  nlp_out_.reset(pacejka_model_acados_get_nlp_out(acados_ocp_capsule_.get()));
  nlp_solver_.reset(pacejka_model_acados_get_nlp_solver(acados_ocp_capsule_.get()));

  parseCompiledSensors();
}

void AcadosPacejkaMheSolver::removeInitialState()
{
}

void AcadosPacejkaMheSolver::parseCompiledSensors()
{
  // Parse compiled sensor support.
  use_mocap_sensor = false;
  use_imu_sensor = false;
  use_imu_yaw_rate_sensor = false;
  use_wheel_encoder_sensor = false;
  use_lighthouse_sensor = false;

  n_measurements = 0;
  //
  for (int i = 0; i < SOLVER_MHE_NUM_SENSORS; i++)
  {
    if (std::string(SOLVER_MHE_SENSOR_LIST[i]) == "mocap")
    {
      use_mocap_sensor = true;
      n_measurements += 3;
    }
    if (std::string(SOLVER_MHE_SENSOR_LIST[i]) == "imu")
    {
      use_imu_sensor = true;
      n_measurements += 3;
    }
    if (std::string(SOLVER_MHE_SENSOR_LIST[i]) == "imu_yaw_rate")
    {
      use_imu_yaw_rate_sensor = true;
      n_measurements += 1;
    }
    if (std::string(SOLVER_MHE_SENSOR_LIST[i]) == "wheel_encoders")
    {
      use_wheel_encoder_sensor = true;
      n_measurements += 4;
    }
    if (std::string(SOLVER_MHE_SENSOR_LIST[i]) == "lighthouse")
    {
      use_lighthouse_sensor = true;
      n_measurements += 8;
    }
  }
}

/**
 * @brief Get the Horizon Length
 *
 * @return int
 */
int AcadosPacejkaMheSolver::getHorizonLength() const
{
  return nlp_dims_->N;
}

/**
 * @brief Set the Initial State Constraint. The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaMheSolver::setInitialState(double constraint[])
{
  setInputBoundConstraint(0, "lbx", constraint);
  setInputBoundConstraint(0, "ubx", constraint);
}
/**
 * @brief Sets an initial guess for the state at stage "stage" of the solver.
 * The provided array must have the same length as the state dimension
 *
 * @param constraint
 */
void AcadosPacejkaMheSolver::setStateInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "x", constraint);
}
/**
 * @brief Sets an initial guess for the input at stage "stage" of the solver.
 * The provided array must have the same length as the input dimension
 *
 * @param constraint
 */
void AcadosPacejkaMheSolver::setInputInitialGuess(int stage, double constraint[])
{
  setOutputInitialGuess(stage, "u", constraint);
}

/**
 * @brief Updates the internal params for stage "stage"
 *
 * @param stage which stage to update the parameter [0,HorizonLength)
 * @param model_dynamics  the model dynamics
 * @param costs  the cost parameters
 * @param references references measurements/inputs
 */
void AcadosPacejkaMheSolver::updateParams(
    int stage, const crs_models::pacejka_model::pacejka_params& model_dynamics, const cost_values& costs,
    const references& references,
    std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_1,
    std::tuple<Eigen::Matrix<double, 2, 4>, Eigen::Vector3d, Eigen::Matrix3d, double> lighthouse_params_2)
{
  // Dynamics
  mhe_parameters[params::L_REAR] = model_dynamics.lr;
  mhe_parameters[params::L_FRONT] = model_dynamics.lf;
  mhe_parameters[params::m] = model_dynamics.m;
  mhe_parameters[params::I] = model_dynamics.I;
  mhe_parameters[params::Df] = model_dynamics.Df;
  mhe_parameters[params::Cf] = model_dynamics.Cf;
  mhe_parameters[params::Bf] = model_dynamics.Bf;
  mhe_parameters[params::Dr] = model_dynamics.Dr;
  mhe_parameters[params::Cr] = model_dynamics.Cr;
  mhe_parameters[params::Br] = model_dynamics.Br;
  mhe_parameters[params::Cm1] = model_dynamics.Cm1;
  mhe_parameters[params::Cm2] = model_dynamics.Cm2;
  mhe_parameters[params::Cd0] = model_dynamics.Cd0;
  mhe_parameters[params::Cd1] = model_dynamics.Cd1;
  mhe_parameters[params::Cd2] = model_dynamics.Cd2;
  mhe_parameters[params::car_width] = model_dynamics.car_width;
  mhe_parameters[params::wheel_radius] = model_dynamics.wheel_radius;
  mhe_parameters[params::torque] = references.input[0];
  mhe_parameters[params::steer] = references.input[1];
  mhe_parameters[params::gamma] = model_dynamics.gamma;
  mhe_parameters[params::eps] = model_dynamics.eps;

  if (use_lighthouse_sensor)
  {
    mhe_parameters[params::sensor_pos_1x] = std::get<0>(lighthouse_params_1)(0, 0);
    mhe_parameters[params::sensor_pos_2x] = std::get<0>(lighthouse_params_1)(0, 1);
    mhe_parameters[params::sensor_pos_3x] = std::get<0>(lighthouse_params_1)(0, 2);
    mhe_parameters[params::sensor_pos_4x] = std::get<0>(lighthouse_params_1)(0, 3);

    mhe_parameters[params::sensor_pos_1y] = std::get<0>(lighthouse_params_1)(1, 0);
    mhe_parameters[params::sensor_pos_2y] = std::get<0>(lighthouse_params_1)(1, 1);
    mhe_parameters[params::sensor_pos_3y] = std::get<0>(lighthouse_params_1)(1, 2);
    mhe_parameters[params::sensor_pos_4y] = std::get<0>(lighthouse_params_1)(1, 3);

    mhe_parameters[params::bs_position_x] = std::get<1>(lighthouse_params_1)(0, 0);
    mhe_parameters[params::bs_position_y] = std::get<1>(lighthouse_params_1)(1, 0);
    mhe_parameters[params::bs_position_z] = std::get<1>(lighthouse_params_1)(2, 0);

    mhe_parameters[params::bs_rotation_00] = std::get<2>(lighthouse_params_1)(0, 0);
    mhe_parameters[params::bs_rotation_01] = std::get<2>(lighthouse_params_1)(0, 1);
    mhe_parameters[params::bs_rotation_02] = std::get<2>(lighthouse_params_1)(0, 2);
    mhe_parameters[params::bs_rotation_10] = std::get<2>(lighthouse_params_1)(1, 0);
    mhe_parameters[params::bs_rotation_11] = std::get<2>(lighthouse_params_1)(1, 1);
    mhe_parameters[params::bs_rotation_12] = std::get<2>(lighthouse_params_1)(1, 2);
    mhe_parameters[params::bs_rotation_20] = std::get<2>(lighthouse_params_1)(2, 0);
    mhe_parameters[params::bs_rotation_21] = std::get<2>(lighthouse_params_1)(2, 1);
    mhe_parameters[params::bs_rotation_22] = std::get<2>(lighthouse_params_1)(2, 2);

    mhe_parameters[params::light_plane_tilt_1] = std::get<3>(lighthouse_params_1);
    mhe_parameters[params::light_plane_tilt_2] = std::get<3>(lighthouse_params_2);
  }

  pacejka_model_acados_update_params(acados_ocp_capsule_.get(), stage, mhe_parameters, np_);

  std::vector<double> reference = {};
  double discount_factor = pow(costs.eta, getHorizonLength() - stage - 1);

  // Update reference for cost
  if (stage == 0)
  {
    // Add ekf state to reference
    reference.insert(reference.end(), { references.state.pos_x, references.state.pos_y, references.state.yaw,
                                        references.state.vel_x, references.state.vel_y, references.state.yaw_rate });
    // Add zeros as reference for w
    reference.insert(reference.end(), { 0, 0, 0, 0, 0, 0 });

    if (use_mocap_sensor)  // Solver supports the usage of mocap
    {
      reference.insert(reference.end(), { references.mocap_measurement(0), references.mocap_measurement(1),
                                          references.mocap_measurement(2) });
    }
    if (use_imu_sensor)  // Solver supports the usage of imu
    {
      reference.insert(reference.end(),
                       { references.imu_measurement(0), references.imu_measurement(1), references.imu_measurement(2) });
    }
    if (use_imu_yaw_rate_sensor)  // Solver supports the usage of imuyaw rate
    {
      reference.insert(reference.end(), { references.imu_yaw_rate_measurement(0) });
    }
    if (use_wheel_encoder_sensor)  // Solver supports the usage of wheel encoder
    {
      reference.insert(reference.end(),
                       { references.wheel_encoder_measurement(0), references.wheel_encoder_measurement(1),
                         references.wheel_encoder_measurement(2), references.wheel_encoder_measurement(3) });
    }
    if (use_lighthouse_sensor)  // Solver supports the usage of lighthouse
    {
      reference.insert(reference.end(),
                       { references.lighthouse_sweep_1_measurement(0), references.lighthouse_sweep_1_measurement(1),
                         references.lighthouse_sweep_1_measurement(2), references.lighthouse_sweep_1_measurement(3) });

      reference.insert(reference.end(),
                       { references.lighthouse_sweep_2_measurement(0), references.lighthouse_sweep_2_measurement(1),
                         references.lighthouse_sweep_2_measurement(2), references.lighthouse_sweep_2_measurement(3) });
    }
    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "y_ref", reference.data());

    // Update cost weights
    // 12 = dimension of state covariance matrix P + dimension of process noise covariance matrix Q
    // n_measurements = dimension of measurement covariance matrices e.g. dim(R_mocap) + dim(R_imu)
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(12 + n_measurements, 12 + n_measurements);
    W.block(0, 0, 6, 6) =
        discount_factor * costs.P.inverse();  // first values = where block starts, second values = size of block
    W.block(6, 6, 6, 6) = discount_factor * costs.Q.inverse();

    int measurement_start = 12;

    if (use_mocap_sensor)
    {
      if (references.valid_mocap)
      {
        W.block(measurement_start, measurement_start, 3, 3) = discount_factor * costs.R_mocap.inverse();
      }
      measurement_start += 3;  // Increase pointer to point to IMU measurement.
    }

    if (use_imu_sensor)
    {
      if (references.valid_imu)
      {
        W.block(measurement_start, measurement_start, 3, 3) = discount_factor * costs.R_imu.inverse();
      }
      measurement_start += 3;  // Increase pointer to point to wheel encoder measurement.
    }

    if (use_imu_yaw_rate_sensor)
    {
      if (references.valid_imu_yaw_rate)
      {
        W.block(measurement_start, measurement_start, 1, 1) = discount_factor * costs.R_imu_yaw_rate.inverse();
      }
      measurement_start += 1;  // Increase pointer to point to wheel encoder measurement.
    }

    if (use_wheel_encoder_sensor)
    {
      if (references.valid_wheel_encoders)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_wheel_encoders.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
    }

    if (use_lighthouse_sensor)
    {
      if (references.valid_lighthouse_sweep_1)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_lighthouse.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
      if (references.valid_lighthouse_sweep_2)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_lighthouse.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
    }

    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W.data());

    // Check w contains nan or inf using eigen
    if (W.array().isNaN().any() || W.array().abs().maxCoeff() > 1e10)
    {
      std::cerr << "Cost Weight Matrix W in MHE solver is ill-conditioned. Check that all cost values are set and "
                   "loaded correctly!"
                << std::endl;
      std::cerr << "W is: " << std::endl << W << std::endl;
    }
  }
  else
  {
    // Add zeros as reference for w
    reference.insert(reference.end(), { 0, 0, 0, 0, 0, 0 });

    if (use_mocap_sensor)  // Solver supports the usage of mocap
    {
      reference.insert(reference.end(), { references.mocap_measurement(0), references.mocap_measurement(1),
                                          references.mocap_measurement(2) });
    }
    if (use_imu_sensor)  // Solver supports the usage of imu
    {
      reference.insert(reference.end(),
                       { references.imu_measurement(0), references.imu_measurement(1), references.imu_measurement(2) });
    }
    if (use_imu_yaw_rate_sensor)  // Solver supports the usage of imu
    {
      reference.insert(reference.end(), { references.imu_yaw_rate_measurement(0) });
    }
    if (use_wheel_encoder_sensor)  // Solver supports the usage of wheel encoder
    {
      reference.insert(reference.end(),
                       { references.wheel_encoder_measurement(0), references.wheel_encoder_measurement(1),
                         references.wheel_encoder_measurement(2), references.wheel_encoder_measurement(3) });
    }

    if (use_lighthouse_sensor)  // Solver supports the usage of wheel encoder
    {
      reference.insert(reference.end(),
                       { references.lighthouse_sweep_1_measurement(0), references.lighthouse_sweep_1_measurement(1),
                         references.lighthouse_sweep_1_measurement(2), references.lighthouse_sweep_1_measurement(3) });

      reference.insert(reference.end(),
                       { references.lighthouse_sweep_2_measurement(0), references.lighthouse_sweep_2_measurement(1),
                         references.lighthouse_sweep_2_measurement(2), references.lighthouse_sweep_2_measurement(3) });
    }

    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "y_ref", reference.data());

    // Update cost weights
    Eigen::MatrixXd W = Eigen::MatrixXd::Zero(6 + n_measurements, 6 + n_measurements);
    W.block(0, 0, 6, 6) =
        discount_factor * costs.Q.inverse();  // first values = where block starts, second values = size of block

    int measurement_start = 6;

    if (!use_mocap_sensor && references.valid_mocap)
    {
      throw std::runtime_error(
          "PacejkaMHESolver was compiled without mocap support but mocap measurement is provided as reference!");
    }
    if (!use_imu_sensor && references.valid_imu)
    {
      throw std::runtime_error(
          "PacejkaMHESolver was compiled without imu support but imu measurement is provided as reference!");
    }
    if (!use_imu_yaw_rate_sensor && references.valid_imu_yaw_rate)
    {
      throw std::runtime_error("PacejkaMHESolver was compiled without imu yaw rate support but imu yaw rate "
                               "measurement is provided as reference!");
    }
    if (!use_wheel_encoder_sensor && references.valid_wheel_encoders)
    {
      throw std::runtime_error(
          "PacejkaMHESolver was compiled without wheel encoder support but wheel encoder measurement is provided as "
          "reference!");
    }

    if (!use_lighthouse_sensor && (references.valid_lighthouse_sweep_1 || references.valid_lighthouse_sweep_2))
    {
      throw std::runtime_error("PacejkaMHESolver was compiled without lighthouse support but lighthouse measurement is "
                               "provided as reference!");
    }

    // Mocap
    if (use_mocap_sensor)
    {
      if (references.valid_mocap)
      {
        W.block(measurement_start, measurement_start, 3, 3) = discount_factor * costs.R_mocap.inverse();
      }
      measurement_start += 3;  // Increase pointer to point to IMU measurement.
    }

    // IMU
    if (use_imu_sensor)
    {
      if (references.valid_imu)
      {
        W.block(measurement_start, measurement_start, 3, 3) = discount_factor * costs.R_imu.inverse();
      }
      measurement_start += 3;  // Increase pointer to point to Wheel encoder measurement.
    }

    // IMU yaw rate
    if (use_imu_yaw_rate_sensor)
    {
      if (references.valid_imu_yaw_rate)
      {
        W.block(measurement_start, measurement_start, 1, 1) = discount_factor * costs.R_imu_yaw_rate.inverse();
      }
      measurement_start += 1;  // Increase pointer to point to Wheel encoder measurement.
    }

    // Wheel encoders
    if (use_wheel_encoder_sensor)
    {
      if (references.valid_wheel_encoders)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_wheel_encoders.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
    }

    // Lighthouse
    if (use_lighthouse_sensor)
    {
      if (references.valid_lighthouse_sweep_1)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_lighthouse.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
      if (references.valid_lighthouse_sweep_2)
      {
        W.block(measurement_start, measurement_start, 4, 4) = discount_factor * costs.R_lighthouse.inverse();
      }
      measurement_start += 4;  // Increase pointer to point.
    }

    ocp_nlp_cost_model_set(nlp_config_.get(), nlp_dims_.get(), nlp_in_.get(), stage, "W", W.data());

    // Check w contains nan or inf using eigen
    if (W.array().isNaN().any() || W.array().abs().maxCoeff() > 1e10)
    {
      std::cerr << "Cost Weight Matrix W in MHE solver is ill-conditioned. Check that all cost values are set and "
                   "loaded correctly!"
                << std::endl;
      std::cerr << "W is: " << std::endl << W << std::endl;
    }
  }
}

/**
 * @brief Solves the optimization problems and stores the solution in x and u.
 *
 * @param x State array or point with size N*StateDimenstion
 * @param u Input array or point with size N*Inputdimension
 * @return int, return code. If no error occurred, return code is zero
 */
int AcadosPacejkaMheSolver::solve(double x[], double u[])
{
  int status = pacejka_model_acados_solve(acados_ocp_capsule_.get());
  if (status)
  {
    return status;
  }
  // Copy solution to x, u
  getLastSolution(x, u);
  return status;
}

double AcadosPacejkaMheSolver::getSamplePeriod()
{
  return *(nlp_in_->Ts);
}
}  // namespace pacejka_solvers

}  // namespace mhe_solvers
