#include <Eigen/Core>
#include <vector>

#include <dynamic_models/utils/data_conversion.h>
#include "parafoil_4dof_model/parafoil_4dof_input.h"
#include "parafoil_4dof_model/parafoil_4dof_state.h"

namespace commons
{
template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::parafoil_4dof_model::parafoil_4dof_state>()
{
  return { casadi::MX::sym("x"),   casadi::MX::sym("y"),     casadi::MX::sym("z"),
           casadi::MX::sym("phi"), casadi::MX::sym("theta"), casadi::MX::sym("psi"),
           casadi::MX::sym("u"),   casadi::MX::sym("v"),     casadi::MX::sym("w") };
}

template <>
const std::vector<casadi::MX> asCasadiSym<crs_models::parafoil_4dof_model::parafoil_4dof_input>()
{
  return { casadi::MX::sym("delta_s"), casadi::MX::sym("delta_a") };
}

template <>
std::vector<double*> convertToVector<crs_models::parafoil_4dof_model::parafoil_4dof_state>(
    crs_models::parafoil_4dof_model::parafoil_4dof_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.pos_z, &state.phi,  &state.theta,
           &state.psi,   &state.vel_u, &state.vel_v, &state.vel_w };
}
template <>

std::vector<const double*> convertToConstVector<crs_models::parafoil_4dof_model::parafoil_4dof_state>(
    const crs_models::parafoil_4dof_model::parafoil_4dof_state& state)
{
  return { &state.pos_x, &state.pos_y, &state.pos_z, &state.phi,  &state.theta,
           &state.psi,   &state.vel_u, &state.vel_v, &state.vel_w };
}
template <>

std::vector<double*> convertToVector<crs_models::parafoil_4dof_model::parafoil_4dof_input>(
    crs_models::parafoil_4dof_model::parafoil_4dof_input& input)
{
  return { &input.deflection_symmetric, &input.deflection_asymmetric };
}
template <>

std::vector<const double*> convertToConstVector<crs_models::parafoil_4dof_model::parafoil_4dof_input>(
    const crs_models::parafoil_4dof_model::parafoil_4dof_input& input)
{
  return { &input.deflection_symmetric, &input.deflection_asymmetric };
}

template <>
crs_models::parafoil_4dof_model::parafoil_4dof_state
convertToState<crs_models::parafoil_4dof_model::parafoil_4dof_state, 9>(const Eigen::Matrix<double, 9, 1>& vector)
{
  crs_models::parafoil_4dof_model::parafoil_4dof_state state;
  state.pos_x = vector(0, 0);
  state.pos_y = vector(1, 1);
  state.pos_z = vector(2, 2);
  state.phi = vector(3, 3);
  state.theta = vector(4, 4);
  state.psi = vector(5, 5);
  state.vel_u = vector(6, 6);
  state.vel_v = vector(7, 7);
  state.vel_w = vector(8, 8);
  return state;
}

template <>
Eigen::Matrix<double, 9, 1> convertToEigen<crs_models::parafoil_4dof_model::parafoil_4dof_state, 9>(
    const crs_models::parafoil_4dof_model::parafoil_4dof_state& state)
{
  Eigen::Matrix<double, 9, 1> matrix;
  matrix(0, 0) = state.pos_x;
  matrix(1, 0) = state.pos_y;
  matrix(2, 0) = state.pos_z;
  matrix(3, 0) = state.phi;
  matrix(4, 0) = state.theta;
  matrix(5, 0) = state.psi;
  matrix(6, 0) = state.vel_u;
  matrix(7, 0) = state.vel_v;
  matrix(8, 0) = state.vel_w;
  return matrix;
}

template <>
crs_models::parafoil_4dof_model::parafoil_4dof_state
convertToState<crs_models::parafoil_4dof_model::parafoil_4dof_state, -1>(const Eigen::Matrix<double, -1, 1>& vector)
{
  return convertToState<crs_models::parafoil_4dof_model::parafoil_4dof_state, 9>(vector);
}

template <>
Eigen::Matrix<double, -1, 1> convertToEigen<crs_models::parafoil_4dof_model::parafoil_4dof_state, -1>(
    const crs_models::parafoil_4dof_model::parafoil_4dof_state& state)
{
  return convertToEigen<crs_models::parafoil_4dof_model::parafoil_4dof_state, 9>(state);
}
}  // namespace commons
