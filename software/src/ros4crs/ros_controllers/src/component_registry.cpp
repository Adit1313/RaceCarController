#include "ros_controllers/component_registry.h"

#include <ros/ros.h>

#include "commons/base_trajectory.h"
#include "commons/dynamic_point_trajectory.h"
#include "crs_msgs/car_state_cart.h"
#include "crs_msgs/car_input.h"
#include "crs_msgs/rocket_state.h"
#include "crs_msgs/rocket_input.h"
#include "ros_crs_utils/parameter_io.h"
#ifdef pacejka_model_FOUND
#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#endif
#ifdef pid_controller_FOUND
#include "ros_controllers/PIDConfig.h"
#include "pid_controller/pacejka_pid_controller.h"
#include "pid_controller/pacejka_pid_const_ref_controller.h"
#endif
#ifdef ff_fb_controller_FOUND
#include "ff_fb_controller/ff_fb_controller.h"
#endif
#ifdef PACEJKA_MPCC_FOUND
#include "pacejka_mpcc/mpcc_pacejka_controller.h"
#include "pacejka_mpcc/mpcc_pacejka_config.h"
#ifdef MPCC_WITH_ACADOS
#include "pacejka_mpcc/solvers/acados_pacejka_mpcc_solver.h"
#endif
#ifdef MPCC_WITH_ACADOS_CURVILINEAR
#include "pacejka_mpcc/solvers/acados_pacejka_curvilinear_mpcc_solver.h"
#endif
#ifdef MPCC_WITH_FORCES
#include "pacejka_mocc/solvers/forces_pacejka_mpcc_solver.h"
#endif
#endif
#ifdef PACEJKA_TRACKING_MPC_FOUND
#include "pacejka_tracking_mpc/tracking_mpc_pacejka_controller.h"
#include "pacejka_tracking_mpc/tracking_mpc_pacejka_config.h"
#endif
#ifdef KINEMATIC_TRACKING_MPC_FOUND
#include "kinematic_tracking_mpc/tracking_mpc_kinematic_controller.h"
#include "kinematic_tracking_mpc/tracking_mpc_kinematic_config.h"
#endif
#ifdef kinematic_model_FOUND
#include "kinematic_model/kinematic_car_input.h"
#include "kinematic_model/kinematic_car_state.h"
#endif
#ifdef pacejka_model_FOUND
#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_params.h"
#endif
#ifdef rocket_6_dof_model_FOUND
#include "rocket_6_dof_model/rocket_6_dof_state.h"
#include "rocket_6_dof_model/rocket_6_dof_input.h"
#endif
#ifdef rocket_position_pid_FOUND
#include "rocket_position_pid/rocket_controller_specializations.h"
#endif

#include "ros_controllers/ros_controller.h"
#include "ros_controllers/dynamic_config.h"
#include "ros_controllers/visualizers/last_reference_point_visualizer.h"
#include "ros_controllers/visualizers/mpc_controller_visualizer.h"
#include "ros_controllers/visualizers/lloyd_visualizer.h"
#include "ros_controllers/visualizers/lloyd_plus_mpc_visualizer.h"

/**
 * @brief This file loads the specific controller implementation and wraps it inside a ros controller object
 *
 */
namespace ros_controllers
{
template <typename ModelType, typename StateType, typename InputType>
std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>
loadControllerVisualizer(ros::NodeHandle nh,
                         std::shared_ptr<crs_controls::BaseController<StateType, InputType>> controller)
{
  std::string type;
  if (!nh.getParam("type", type))
  {
    ROS_WARN("No Type specified for visualizer! No visualizer loaded for controller");
    return std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>(nullptr);
  }

  if (type == "last_reference")
  {
    return std::make_unique<LastReferencePointVisualizer<StateType, InputType>>(nh, controller);
  }

  if (type == "lloyd_pid")
  {
    return std::make_unique<LloydVisualizer<StateType, InputType>>(nh, controller);
  }

  if (type == "mpc")
  {
    return std::make_unique<MpcControllerVisualizer<ModelType, StateType, InputType>>(
        nh, std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(controller));
  }

  if (type == "lloyd_plus_mpc")
  {
    return std::make_unique<LloydPlusMPCVisualizer<ModelType, StateType, InputType>>(
        nh, std::dynamic_pointer_cast<crs_controls::MpcController<ModelType, StateType, InputType>>(controller));
  }

  ROS_WARN_STREAM("No Visualizer found for type: " << type << ". No visualizer loaded for controller!");
  return std::unique_ptr<BaseControllerVisualizer<StateType, InputType>>(nullptr);
};

#ifdef pacejka_model_FOUND
// Shorten type names
typedef crs_msgs::car_state_cart ros_car_state;
typedef crs_msgs::car_input ros_car_input;
typedef crs_models::pacejka_model::pacejka_car_state pacejka_state;
typedef crs_models::pacejka_model::pacejka_car_input pacejka_input;
typedef crs_models::pacejka_model::pacejka_params pacejka_model_params;
typedef RosController<ros_car_state, ros_car_input, pacejka_state, pacejka_input> pacejka_ros_controller;

// =============================================================================================
// ===============                  PID CONTROLLER                  ============================
// =============================================================================================
#ifdef pid_controller_FOUND
inline pacejka_ros_controller* getPacejkaPidController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                       void*& dynamic_callback_allocator)
{
  // config from params
  crs_controls::pid_config pid_config =
      parameter_io::getConfig<crs_controls::pid_config>(ros::NodeHandle(nh_private, "controller_params"));

  // Create controller
  auto ptr = std::make_shared<crs_controls::PacejkaPIDController>(
      pid_config, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer
  auto visualizer_ptr = loadControllerVisualizer<void, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  dynamic_callback_allocator =
      (void*)new ros_controllers::DynamicPIDConfigServer(ros::NodeHandle(nh_private, "controller_params"), ptr);
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

inline pacejka_ros_controller* getPacejkaConstReferencePidController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                                     void*& dynamic_callback_allocator)
{
  // config from params
  crs_controls::pid_const_ref_config pid_config =
      parameter_io::getConfig<crs_controls::pid_const_ref_config>(ros::NodeHandle(nh_private, "controller_params"));

  // start track points
  std::vector<double> x_start = { 0.3 };
  std::vector<double> y_start = { 0.0 };
  // Create controller
  auto ptr = std::make_shared<crs_controls::PacejkaConstRefPIDController>(
      pid_config, std::static_pointer_cast<crs_controls::Trajectory>(
                      std::make_shared<crs_controls::DynamicPointTrajectory>(x_start, y_start)));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer
  auto visualizer_ptr = loadControllerVisualizer<void, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

#endif  // pid_controller_FOUND

// =============================================================================================
// ===============                FF FB CONTROLLER                  ============================
// =============================================================================================

#ifdef ff_fb_controller_FOUND
inline pacejka_ros_controller* getPacejkaFfFbController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                        void*& dynamic_callback_allocator)
{
  // Load FfFbConfig config from params
  crs_controls::FfFbConfig ff_fb_config =
      parameter_io::getConfig<crs_controls::FfFbConfig>(ros::NodeHandle(nh_private, "controller_params"));
  // Load model from params
  std::shared_ptr<pacejka_model_params> pacejka_params = std::make_shared<pacejka_model_params>();
  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), *pacejka_params);
  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), *pacejka_params,
                                                     false);
  // Create Controller
  auto ptr = std::make_shared<crs_controls::FfFbController>(
      ff_fb_config, pacejka_params, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<crs_controls::BaseController<pacejka_state, pacejka_input>>(ptr);
  // Create Visualizer

  auto visualizer_ptr = loadControllerVisualizer<pacejka_model_params, pacejka_state, pacejka_input>(
      ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  dynamic_callback_allocator =
      (void*)new ros_controllers::DynamicFfFbConfigServer(ros::NodeHandle(nh_private, "controller_params"), ptr);
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}
#endif  // ff_fb_controller_FOUND

// =============================================================================================
// ===============                 MPCC CONTROLLER                  ============================
// =============================================================================================

#ifdef PACEJKA_MPCC_FOUND
inline pacejka_ros_controller* getPacejkaMPCCController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                        void*& dynamic_callback_allocator)
{
  crs_controls::pacejka_mpcc::mpcc_pacejka_config cfg =
      parameter_io::getConfig<crs_controls::pacejka_mpcc::mpcc_pacejka_config>(
          ros::NodeHandle(nh_private, "controller_params"));

  // Load model from params
  pacejka_model_params pacejka_params;

  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), pacejka_params);

  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), pacejka_params,
                                                     false);

  // Load model from params
  std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(pacejka_params);

  // Get the MPCC downcasted to an MpcController pointer
  auto derived_ptr = crs_controls::pacejka_mpcc::controllerFactory(
      cfg, pacejka_model, parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track")));

  auto visualizer_ptr =
      loadControllerVisualizer<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>(
          ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

// Do some funky pointer casting to register the corresponding parameter server
#ifdef DMPCC_WITH_ACADOS
  {
    auto ptr = std::dynamic_pointer_cast<crs_controls::pacejka_mpcc::PacejkaMpccController<
        crs_controls::pacejka_mpcc::solvers::acados_solver::AcadosPacejkaMpccSolver>>(derived_ptr);
    if (ptr != nullptr)
    {
      dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(
          ros::NodeHandle(nh_private, "controller_params"), ptr);
    }
  }
#endif
#ifdef MPCC_WITH_ACADOS_CURVILINEAR
  {
    auto ptr = std::dynamic_pointer_cast<crs_controls::pacejka_mpcc::PacejkaMpccController<
        crs_controls::pacejka_mpcc::solvers::acados_solver::AcadosPacejkaCurvilinearMpccSolver>>(derived_ptr);
    if (ptr != nullptr)
    {
      dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(
          ros::NodeHandle(nh_private, "controller_params"), ptr);
    }
  }
#endif
#ifdef MPCC_WITH_FORCES
  {
    auto ptr = std::dynamic_pointer_cast<crs_controls::pacejka_mpcc::PacejkaMpccController<
        crs_controls::pacejka_mpcc::solvers::ForcesPacejkaMpccSolver>>(derived_ptr);
    if (ptr != nullptr)
    {
      dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(
          ros::NodeHandle(nh_private, "controller_params"), ptr);
    }
  }
#endif
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}
#endif  // PACEJKA_MPCC_FOUND

#ifdef PACEJKA_TRACKING_MPC_FOUND
inline pacejka_ros_controller* getPacejkaTrackingMPCController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                               void*& dynamic_callback_allocator)
{
  crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config cfg =
      parameter_io::getConfig<crs_controls::pacejka_tracking_mpc::tracking_mpc_pacejka_config>(
          ros::NodeHandle(nh_private, "controller_params"));

  // Load model from params
  pacejka_model_params pacejka_params;

  // First load generic, gt model
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh, "model/model_params"), pacejka_params);

  // Patch certain params
  parameter_io::getModelParams<pacejka_model_params>(ros::NodeHandle(nh_private, "model/model_params"), pacejka_params,
                                                     false);

  // Load model from params
  std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> pacejka_model =
      std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(pacejka_params);

  std::shared_ptr<crs_controls::DynamicPointTrajectory> dynamic_ref =
      parameter_io::loadReferenceFromParams(ros::NodeHandle(nh, "reference"));

  // Create Controller
  auto ptr = std::make_shared<crs_controls::pacejka_tracking_mpc::PacejkaTrackingMpcController>(cfg, pacejka_model,
                                                                                                dynamic_ref);

  // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>>(ptr);

  auto visualizer_ptr =
      loadControllerVisualizer<crs_models::pacejka_model::DiscretePacejkaModel, pacejka_state, pacejka_input>(
          ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  // dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(ros::NodeHandle(nh_private,
  // "controller_params"), ptr); //not existant at the moment
  return new pacejka_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}
#endif  // PACEJKA_TRACKING_MPC_FOUND

#endif  // pacejka_model_FOUND

#ifdef kinematic_model_FOUND
// Shorten type names
typedef crs_msgs::car_state_cart ros_car_state;
typedef crs_msgs::car_input ros_car_input;
typedef crs_models::kinematic_model::kinematic_car_state kinematic_state;
typedef crs_models::kinematic_model::kinematic_car_input kinematic_input;
typedef crs_models::kinematic_model::kinematic_params kinematic_model_params;
typedef RosController<ros_car_state, ros_car_input, kinematic_state, kinematic_input> kinematic_ros_controller;

#ifdef KINEMATIC_TRACKING_MPC_FOUND

inline kinematic_ros_controller* getKinematicTrackingMPCController(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                                                                   void*& dynamic_callback_allocator)
{
  crs_controls::kinematic_tracking_mpc::tracking_mpc_kinematic_config cfg =
      parameter_io::getConfig<crs_controls::kinematic_tracking_mpc::tracking_mpc_kinematic_config>(
          ros::NodeHandle(nh_private, "controller_params"));

  // Load model from params
  kinematic_model_params kinematic_params;

  // First load generic, gt model
  parameter_io::getModelParams<kinematic_model_params>(ros::NodeHandle(nh, "model/model_params"), kinematic_params);

  // Patch certain params
  parameter_io::getModelParams<kinematic_model_params>(ros::NodeHandle(nh_private, "model/model_params"),
                                                       kinematic_params, false);

  // Load model from params
  std::shared_ptr<crs_models::kinematic_model::DiscreteKinematicModel> kinematic_model =
      std::make_shared<crs_models::kinematic_model::DiscreteKinematicModel>(kinematic_params);

  std::shared_ptr<crs_controls::StaticTrackTrajectory> static_track =
      parameter_io::loadTrackDescriptionFromParams(ros::NodeHandle(nh, "track"));
  std::cout << "loading track" << std::endl;

  std::shared_ptr<crs_controls::DynamicPointTrajectory> dynamic_ref =
      parameter_io::loadReferenceFromParams(ros::NodeHandle(nh, "reference"));

  std::cout << "loading reference" << std::endl;

  // Create Controller
  auto ptr = std::make_shared<crs_controls::kinematic_tracking_mpc::KinematicTrackingMpcController>(
      cfg, kinematic_model, dynamic_ref);

  // Downcast to BaseController type
  auto derived_ptr =
      std::dynamic_pointer_cast<crs_controls::MpcController<crs_models::kinematic_model::DiscreteKinematicModel,
                                                            kinematic_state, kinematic_input>>(ptr);

  auto visualizer_ptr =
      loadControllerVisualizer<crs_models::kinematic_model::DiscreteKinematicModel, kinematic_state, kinematic_input>(
          ros::NodeHandle(nh_private, "visualizer"), derived_ptr);

  // dynamic_callback_allocator = (void*)new ros_controllers::DynamicPacejkaMPCCConfigServer(ros::NodeHandle(nh_private,
  // "controller_params"), ptr); //not existant at the moment
  return new kinematic_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

#endif  // KINEMATIC_TRACKING_MPC_FOUND
#endif  // kinematic_model_FOUND

#ifdef rocket_6_dof_model_FOUND
// =============================================================================================
// ===============                   ROCKET STUFF                   ============================
// =============================================================================================

// Shorten type names
typedef crs_msgs::rocket_state ros_rocket_state;
typedef crs_msgs::rocket_input ros_rocket_input;
typedef crs_models::rocket_6_dof_model::rocket_6_dof_state rocket_6_dof_state;
typedef crs_models::rocket_6_dof_model::rocket_6_dof_input rocket_6_dof_input;
typedef crs_models::rocket_6_dof_model::rocket_6_dof_params rocket_6_dof_model_params;
typedef RosController<ros_rocket_state, ros_rocket_input, rocket_6_dof_state, rocket_6_dof_input,
                      crs_controls::ThreeDofPositionTrajectory>
    rocket_ros_controller;

inline rocket_ros_controller* getRocketAltitudePidBaseAllocationController(ros::NodeHandle& nh,
                                                                           ros::NodeHandle& nh_private,
                                                                           void*& dynamic_callback_allocator)
{
  // Load controller config
  crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                         crs_controls::RocketAttitudeController, crs_controls::Rocket6DofAllocation>
      controller_config =
          parameter_io::getConfig<crs_controls::rocket_controller_config<crs_controls::RocketHighLevelPidController,
                                                                         crs_controls::RocketAttitudeController,
                                                                         crs_controls::Rocket6DofAllocation>>(
              ros::NodeHandle(nh_private, "controller_params"));

  // Load rocket model
  std::shared_ptr<rocket_6_dof_model_params> rocket_params = std::make_shared<rocket_6_dof_model_params>();
  // First load generic, gt model
  parameter_io::getModelParams<rocket_6_dof_model_params>(ros::NodeHandle(nh, "model/model_params"), *rocket_params);

  std::vector<Eigen::Vector3d> initial_setpoint = { Eigen::Vector3d(1.4, 0.0, 0.0) };
  auto rocket_controller_ptr = std::make_shared<crs_controls::Rocket6DofPidController>(
      controller_config, rocket_params, std::make_shared<crs_controls::ThreeDofPositionTrajectory>(initial_setpoint));

  // // Downcast to BaseController type
  auto derived_ptr = std::dynamic_pointer_cast<
      crs_controls::BaseController<rocket_6_dof_state, rocket_6_dof_input, crs_controls::ThreeDofPositionTrajectory>>(
      rocket_controller_ptr);

  // Create Visualizer
  auto visualizer_ptr = std::unique_ptr<BaseControllerVisualizer<rocket_6_dof_state, rocket_6_dof_input>>(nullptr);

  dynamic_callback_allocator = (void*)new ros_controllers::DynamicRocketPidConfigServer(
      ros::NodeHandle(nh_private, "controller_params"), rocket_controller_ptr);

  return new rocket_ros_controller(nh, nh_private, std::move(visualizer_ptr), derived_ptr);
}

#endif  // rocket_6_dof_model_FOUND

// =============================================================================================
// ===============                 RESOLVE CONTROLLER               ============================
// =============================================================================================
template <>
pacejka_ros_controller* resolveController<ros_car_state, ros_car_input, pacejka_state, pacejka_input>(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& controller_type,
    void*& dynamic_callback_allocator)
{
#ifdef pacejka_model_FOUND
#ifdef pid_controller_FOUND
  if (controller_type == "PID")
  {
    return getPacejkaPidController(nh, nh_private, dynamic_callback_allocator);
  }

  if (controller_type == "CONST_REF_PID")
  {
    return getPacejkaConstReferencePidController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // pid_controller_FOUND

#ifdef ff_fb_controller_FOUND
  if (controller_type == "FF_FB")
  {
    return getPacejkaFfFbController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // ff_fb_controller_FOUND

#ifdef PACEJKA_MPCC_FOUND
  if (controller_type == "MPCC")
  {
    return getPacejkaMPCCController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // PACEJKA_MPCC_FOUND

#ifdef PACEJKA_TRACKING_MPC_FOUND  // not existent at the moment
  if (controller_type == "TRACKING_MPC")
  {
    return getPacejkaTrackingMPCController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // PACEJKA_TRACKING_MPC_FOUND
#endif  // pacejka_model_FOUND

  assert(false && "Did not find registered controller for specified controller type.");
  return nullptr;
}

#ifdef kinematic_model_FOUND
template <>
kinematic_ros_controller* resolveController<ros_car_state, ros_car_input, kinematic_state, kinematic_input>(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& controller_type,
    void*& dynamic_callback_allocator)
{
#ifdef KINEMATIC_TRACKING_MPC_FOUND
  if (controller_type == "KINEMATIC_TRACKING_MPC")
  {
    ROS_WARN("KINEMATIC_TRACKING_MPC");
    return getKinematicTrackingMPCController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // KINEMATIC_TRACKING_MPC

  assert(false && "Did not find registered controller for specified controller type.");
  return nullptr;
}
#endif  // kinematic_model_FOUND

template <>
rocket_ros_controller* resolveController<ros_rocket_state, ros_rocket_input, rocket_6_dof_state, rocket_6_dof_input>(
    ros::NodeHandle& nh, ros::NodeHandle& nh_private, const std::string& controller_type,
    void*& dynamic_callback_allocator)
{
#ifdef rocket_6_dof_model_FOUND
  std::string allocation_type;
  nh_private.getParam("allocation_type", allocation_type);
  if (controller_type == "ROCKET_ALTITUDE" && allocation_type == "BASE")
  {
    return getRocketAltitudePidBaseAllocationController(nh, nh_private, dynamic_callback_allocator);
  }
#endif  // rocket_6_dof_model_FOUND

  assert(false && "Did not find registered controller for specified controller type.");
  return nullptr;
}

}  // namespace ros_controllers
