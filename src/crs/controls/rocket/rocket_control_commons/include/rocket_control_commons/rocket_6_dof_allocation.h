#ifndef ROCKET_CONTROL_COMMONS_ROCKET_6_DOF_ALLOCATION
#define ROCKET_CONTROL_COMMONS_ROCKET_6_DOF_ALLOCATION

#include <memory>
#include <cmath>
#include <Eigen/Core>

#include "rocket_6_dof_model/rocket_6_dof_input.h"
#include "rocket_6_dof_model/rocket_6_dof_params.h"

namespace crs_controls
{

struct rocket_6_dof_allocation_config
{
  Eigen::Matrix<double, 4, 2> constraint_coefficients;
};

class Rocket6DofAllocation
{
protected:
  std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model_;
  rocket_6_dof_allocation_config config_;
  std::vector<double> debug_;

  double mapGimbalToServoAngle(const double gimbal_angle, const double rotation_axis_tilt_angle) const;

  /**
   * @brief Project a thrust, torque tuple into the set of feasible actuator inputs.
   *
   * The projection is done such that thrust is prioritized, i.e. we keep the desired
   * thrust magnitude constant and adjust the desired torque such that it lies on the
   * boundary of the feasible set.
   *
   * @param torque_x is the desired torque around the body x axis
   * @param thrust_magnitude is the desired thurst magnitude of the rocket
   * @return double which is the new constrained torque_x
   */
  double constrainTorque(const double torque_x, const double thrust_magnitude);

public:
  /**
   * @brief Construct a new Base Allocator object.
   */
  Rocket6DofAllocation(std::shared_ptr<crs_models::rocket_6_dof_model::rocket_6_dof_params> model,
                       rocket_6_dof_allocation_config config);

  /**
   * @brief Convert force and torque commands produced by the controllers into actuator commands.
   *
   * @param force is the desired force vector which should be produced by the actuators.
   * @param torque is the desired torque vector which should be produced by the actuators.
   * @return InputType depending on the system
   */
  crs_models::rocket_6_dof_model::rocket_6_dof_input getControlInput(const Eigen::Vector3d& force,
                                                                     const Eigen::Vector3d& torque);

  /**
   * @brief Return a reference(!) to the allocation config
   *
   * @return rocket_6_dof_allocation_config&
   */
  rocket_6_dof_allocation_config& getConfig()
  {
    return config_;
  }

  std::vector<double> getDebugControllerState()
  {
    return debug_;
  }

  /**
   * @brief Returns wether the allocation is initialized
   *
   * @return true if allocation is already initialized
   * @return false if allocation is not initialized
   */
  bool isInitializing()
  {
    return false;
  }
};

}  // namespace crs_controls

#endif /* ROCKET_CONTROL_COMMONS_ROCKET_6_DOF_ALLOCATION */
