#ifndef SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER
#define SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER

#include <commons/trajectory.h>
#include <memory>
#include <optional>

namespace crs_controls
{
template <typename StateType, typename InputType, typename TrajectoryType = Trajectory>
class BaseController
{
public:
  /**
   * @brief Construct a new Base Controller object
   *
   * @param trajectory the trajectory manager that should be used by this controller
   */
  BaseController(std::shared_ptr<TrajectoryType> trajectory) : trajectory_(trajectory){};

  /**
   * @brief Returns the control input for a given measured state.
   *
   * @param state measured state
   * @param timestamp current timestamp in seconds
   * @return InputType control input to apply
   */
  virtual InputType getControlInput(StateType state, double timestamp = 0) = 0;

  /**
   * @brief Returns the trajectory manager associated with this controller
   *
   * @return const std::shared_ptr<const Trajectory>
   */
  template <typename TrajectoryReturnType = Trajectory>
  const std::shared_ptr<TrajectoryReturnType> getTrajectory() const
  {
    return std::static_pointer_cast<TrajectoryReturnType>(trajectory_);
  }

  /**
   * @brief Returns wether the controller is initialized
   *
   * @return true if controller is already initialized
   * @return false if controller is not initialized
   */
  virtual bool isInitializing() = 0;

  /**
   * @brief Debug function which can be used to return some useful debug info from the controller
   *
   * @return const std::vector<double> containing relevant debug info about the controller
   */
  virtual const std::vector<double> getDebugControllerState() const
  {
    return {};
  }

  /**
   * @brief Return a planned trajectory which can be used to visualize, e.g., the planned path.
   *
   * If the controller does not provide a planned trajectory, an empty optional is returned.
   */
  virtual std::optional<std::vector<std::vector<double>>> getPlannedTrajectory() const
  {
    return {};
  };

  /*
   * @brief Set internal state variables of the controller
   *
   * @param internal_state internal state parameters to be set in the controller.
   * Could be e.g. is_running.
   */
  virtual void setInternalControllerState(const std::vector<bool>& internal_state [[maybe_unused]])
  {
  }

protected:
  std::shared_ptr<TrajectoryType> trajectory_;
};

}  // namespace crs_controls

#endif /* SRC_CRS_CONTROLS_COMMON_INCLUDE_CONTROLS_BASE_CONTROLLER */
