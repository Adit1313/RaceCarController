/**
 * @file    casadi_utils.h
 * @brief   Implements utilities for often-used functionality with the CasADi library.
 */

#pragma once

#include <vector>

#include <Eigen/Core>
#include <casadi/casadi.hpp>

namespace commons
{

/**
 * @brief CasADi function wrapper that caches the Jacobian.
 *
 * This class provides a convenient interface for evaluating CasADi functions and their Jacobians.
 * The reason for its existence is pretty much the inability of CasADi to cache the Jacobian call
 * in a way that is understandable. We solve it by having class that wraps the function and its Jacobian.
 *
 * The class is expensive to construct, but performant in calls to evaluate and evaluateJacobian.
 */
class CasadiFunction
{
public:
  CasadiFunction(){};  // Default constructor

  /**
   * @brief Construct a new CasadiFunction object.
   * @note This call is expensive since it calculates the Jacobian.
   */
  CasadiFunction(const casadi::Function& function) : function_(function), jacobian_(function_.jacobian())
  {
  }

  /**
   * @brief Evaluate the function at a point.
   */
  void operator()(const std::vector<const double*>& x, std::vector<double*> y) const
  {
    function_(x, y);
  }

  /**
   * @brief Evaluate the function at a given point.
   *
   * @param x Numerical point for evaluation.
   * @return Eigen::VectorXd Function value.
   */
  Eigen::VectorXd evaluate(const std::vector<const double*>& x) const;

  /**
   * @brief Evaluate the Jacobian of the function at a given point.
   *
   * @param x Numerical point for evaluation.
   * @return Eigen::MatrixXd Jacobian matrix.
   */
  Eigen::MatrixXd evaluateJacobian(const std::vector<const double*>& x) const;

  /**
   * @brief Get the symbolic Jacobian of the wrapped function.
   * @return casadi::Function Symbolic function.
   */
  casadi::Function getSymbolicJacobian() const
  {
    return jacobian_;
  }

private:
  casadi::Function function_;  ///< The underlying CasADi functor
  casadi::Function jacobian_;  ///< Cached Jacobian of @ref function_.
};

}  // namespace commons
