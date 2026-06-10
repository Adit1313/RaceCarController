#ifndef ROS_CONTROLLERS_ROS_CONTROLLER_H
#define ROS_CONTROLLERS_ROS_CONTROLLER_H
#include <ros/ros.h>

#include <trajectory_msgs/JointTrajectory.h>

#include <control_commons/base_controller.h>
#include <ros_crs_utils/state_message_conversion.h>
#include <ros_crs_utils/trajectory_message_conversion.h>

#include <visualization_msgs/Marker.h>

#include "ros_controllers/visualizers/base_visualizer.h"

#include "crs_msgs/double_array_stamped.h"
#include "crs_msgs/bool_array_stamped.h"

namespace ros_controllers
{

/**
 * @brief Ros wrapper around a BaseRosController.
 * Note, this must stay a header file in order to have dynamic template support
 *
 */
template <typename StateMsg, typename InputMsg, typename StateType, typename InputType,
          typename TrajectoryType = crs_controls::Trajectory>
class RosController
{
private:
  // Node handles.
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publisher
  ros::Publisher input_publisher_;
  ros::Publisher velocity_publisher_;
  ros::Publisher debug_publisher_;
  ros::Publisher trajectory_publisher_;

  ros::Timer visualization_timer_;

  // Subscriptions
  ros::Subscriber state_subscriber_;
  ros::Subscriber controller_settings_subscriber_;

  // Visualizer for controller
  std::unique_ptr<BaseControllerVisualizer<StateType, InputType>> visualizer_;

  // Timer which controls controller execution
  ros::Timer controller_timer_;

  bool received_state_;
  StateType current_state_;

  // Parameters to wait for the estimator to converge (initial state)
  bool is_first_callback_;
  float init_delay_;

  /** Whether to publish the trajectory computed by the controller */
  bool publish_trajectory_;
  bool publish_debug_msgs_;

public:
  /**
   * TODO UGLY, move back to private, maybe use getter() or expose differently
   *
   * @brief Reference to base controller
   *
   */
  std::shared_ptr<crs_controls::BaseController<StateType, InputType, TrajectoryType>> controller_;

  /**
   * @brief Construct a new Ros Controller object
   *
   * @param nh nodehandle point to ~
   * @param nh_private nodehandle pointing to ~/ros_controller
   * @param controller ptr to the underlying base controller that this ros wrappers wraps
   */
  RosController(ros::NodeHandle nh, ros::NodeHandle nh_private,
                std::unique_ptr<BaseControllerVisualizer<StateType, InputType>> visualizer,
                std::shared_ptr<crs_controls::BaseController<StateType, InputType, TrajectoryType>> controller)
    : nh_(nh), nh_private_(nh_private), controller_(controller), received_state_(false), is_first_callback_(true)
  {
    input_publisher_ = nh_private_.advertise<InputMsg>("control_input", 10);
    trajectory_publisher_ = nh_private_.advertise<trajectory_msgs::JointTrajectory>("trajectory", 10);
    debug_publisher_ = nh_private_.advertise<crs_msgs::double_array_stamped>("controller_debug", 10);
    state_subscriber_ = nh_private_.subscribe("state", 10, &RosController::stateCallback, this);
    controller_settings_subscriber_ =
        nh_private_.subscribe("controller_settings", 10, &RosController::setControllerSettingsCallback, this);

    double max_rate;
    if (!nh_private_.getParam("max_rate", max_rate))
    {
      ROS_WARN("[RosController] No Parameter set for max_rate. Will default to 100");
      max_rate = 100.0;
    }
    if (max_rate != 0.0)
    {
      controller_timer_ = nh_private.createTimer(ros::Duration(1 / max_rate), &RosController::controllerCallback, this);
    }
    else
    {
      ROS_ERROR("Controller can't accept a rate of 0 Hz");
      ros::shutdown();
    }

    if (!nh_private.getParam("publish_debug", publish_debug_msgs_))
    {
      publish_debug_msgs_ = false;
    }

    if (!nh_private.getParam("publish_trajectory", publish_trajectory_))
    {
      publish_trajectory_ = false;
    }

    if (!nh_private.getParam("init_delay", init_delay_))
    {
      ROS_WARN("[RosController] No Parameter set for init_delay. Will default to 0 seconds.");
      init_delay_ = 0.0;
    }

    visualizer_ = std::move(visualizer);
  }

  /**
   * @brief State callback, converts the state message to crs state message and saves it for later use
   * in the controller callback.
   *
   * @param state_msg the input state message (ros format)
   */
  void stateCallback(StateMsg state_msg)
  {
    current_state_ = message_conversion::convertMsgToState<StateType, StateMsg>(state_msg);
    received_state_ = true;
  }

  /**
   * @brief sets the internal state of the controller
   */
  void setControllerSettingsCallback(const crs_msgs::bool_array_stamped::ConstPtr& msg)
  {
    std::vector<bool> bool_vector;

    for (const bool& value : msg->data)
    {
      bool_vector.push_back(value);
    }
    controller_->setInternalControllerState(bool_vector);
  }

  /**
   * @brief Controller callback which computes a control action based on the last received state input.
   * This callback is attached to a ros timer which regulates the execution rate of this callback.
   *
   * @param event is a ros TimerEvent associated with the timer.
   */
  void controllerCallback(const ros::TimerEvent& event)
  {
    if (controller_->isInitializing() || !received_state_)
      return;

    // If first state is received we need to wait for the estimator position to converge
    // to the right initial position
    if (is_first_callback_)
    {
      // print init delay
      ros::Duration(init_delay_).sleep();
      is_first_callback_ = false;
    }

    double last_callback = ros::Time::now().toSec();
    InputType input = controller_->getControlInput(current_state_, last_callback);
    input_publisher_.publish(message_conversion::convertToRosInput<InputMsg, InputType>(input));

    if (publish_trajectory_)
    {
      // Note that not all controllers might return a trajectory – don't publish in the case of an empty optional
      auto trajectory = controller_->getPlannedTrajectory();
      if (trajectory.has_value())
      {
        auto trajectory_msg = message_conversion::convertToRosTrajectory(trajectory.value());
        trajectory_publisher_.publish(trajectory_msg);
      }
      else
      {
        ROS_WARN_THROTTLE(10.0, "Controller did not return a trajectory.");
      }
    }

    if (publish_debug_msgs_)
    {
      std::vector<double> debug_controller_states = controller_->getDebugControllerState();
      crs_msgs::double_array_stamped debug_msg;
      debug_msg.header.stamp = ros::Time::now();
      for (double elem : debug_controller_states)
      {
        debug_msg.data.push_back(elem);
      }
      debug_publisher_.publish(debug_msg);
    }
  }
};
}  // namespace ros_controllers
#endif
