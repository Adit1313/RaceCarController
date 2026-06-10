#include "mpc_controller/pacejka_controller/mpcc_pacejka_controller.h"

#include <fstream>
#include <iostream>

#include "gtest/gtest.h"
#include "casadi/casadi.hpp"
#include "Eigen/Core"

#include "commons/static_track_trajectory.h"
#include "pacejka_model/pacejka_car_input.h"
#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_discrete.h"
#include "pacejka_model/pacejka_params.h"

#include "mpc_controller/pacejka_controller/mpcc_pacejka_config.h"

/**
 * @brief Load a track description from a yaml file.
 *
 * @param filename indicates the filepath to the yaml file.
 *
 * @return std::shared_ptr<crs_controls::StaticTrackTrajectory> of the resulting track reference.
 */
std::shared_ptr<crs_controls::StaticTrackTrajectory> loadTrackDescription(std::string filename)
{
  YAML::Node file_content = YAML::LoadFile(filename);
  YAML::Node track = file_content["track"];
  std::vector<double> x_coords = track["xCoords"].as<std::vector<double>>();
  std::vector<double> y_coords = track["yCoords"].as<std::vector<double>>();
  std::vector<double> x_rate = track["xRate"].as<std::vector<double>>();
  std::vector<double> y_rate = track["yRate"].as<std::vector<double>>();
  std::vector<double> arc_length = track["arcLength"].as<std::vector<double>>();
  std::vector<double> tangent_angle = track["tangentAngle"].as<std::vector<double>>();
  std::vector<double> curvature = track["curvature"].as<std::vector<double>>();
  double width = track["trackWidth"].as<double>();
  double density = track["density"].as<double>();

  return std::make_shared<crs_controls::StaticTrackTrajectory>(x_coords, y_coords, x_rate, y_rate, width, curvature,
                                                               tangent_angle, arc_length, density);
}

crs_controls::mpcc_pacejka_config loadMpccParameters(std::string filename)
{
  crs_controls::mpcc_pacejka_config mpc_params;
  YAML::Node file_content = YAML::LoadFile(filename);
  mpc_params.Q1 = file_content["Q1"].as<double>();
  mpc_params.Q2 = file_content["Q2"].as<double>();
  mpc_params.R1 = file_content["R1"].as<double>();
  mpc_params.R2 = file_content["R2"].as<double>();
  mpc_params.R3 = file_content["R3"].as<double>();
  mpc_params.q = file_content["q"].as<double>();
  mpc_params.lag_compensation_time = file_content["lag_compensation_time"].as<double>();
  mpc_params.solver_type = file_content["solver_type"].as<std::string>();
  return mpc_params;
}

/**
 * @brief Print the controllers planned trajectory to a file in numpy.array format
 *
 * Very convenient for debugging. Simply copy paste the contents of
 * "/code/test_out.txt" into a python file.
 *
 * @param controller reference to the controller object
 */
void printTrajectory(crs_controls::PacejkaMpccController& controller)
{
  std::ofstream myfile;
  myfile.open("/code/test_out.txt", std::ios_base::app);
  auto planned_traj = controller.getPlannedTrajectory().value();

  myfile << "arrlist.append(np.array([" << std::endl;

  for (size_t i = 0; i < planned_traj.size(); ++i)
  {
    myfile << "\t[";
    for (size_t j = 0; j < planned_traj[i].size(); ++j)
    {
      myfile << planned_traj[i][j];
      if (j != planned_traj[i].size() - 1)
      {
        myfile << ", ";
      }
      else
      {
        myfile << "]," << std::endl;
        ;
      }
    }
  }
  myfile << "]))" << std::endl;
  myfile.close();
}

/**
 * @brief Tests initialization and solution prediciton
 *
 * The test checks whether the predicted x are close to the reference, where
 * close means that the position is still contained in the track.
 */
TEST(MpccTests, testMPCCpredictions)
{
  crs_models::pacejka_model::pacejka_params params;
  crs_models::pacejka_model::loadParamsFromFile("params/test_pacejka_params.yaml", params);
  auto model = std::make_shared<crs_models::pacejka_model::DiscretePacejkaModel>(params);

  crs_controls::mpcc_pacejka_config mpc_params = loadMpccParameters("params/test_mpcc_params.yaml");

  std::shared_ptr<crs_controls::StaticTrackTrajectory> track = loadTrackDescription("params/DEMO_TRACK.yaml");

  crs_controls::PacejkaMpccController controller(mpc_params, model, track);

  Eigen::Vector2d initial_position = track->operator[](0);
  crs_models::pacejka_model::pacejka_car_state initial_state = {
    initial_position.x(), initial_position.y(), track->getTrackAngle(0), 0.5, 0, 0  // nonzero vx
  };

  crs_models::pacejka_model::pacejka_car_state current_state = initial_state;

  for (int test_iterations = 0; test_iterations < 200; ++test_iterations)
  {
    crs_models::pacejka_model::pacejka_car_input input = controller.getControlInput(current_state);
    crs_models::pacejka_model::pacejka_car_state new_state = model->applyModel(current_state, input, 0.033);
    current_state = new_state;

    auto trajectory = controller.getPlannedTrajectory().value();

    for (size_t trajectory_idx = 0; trajectory_idx < trajectory.size(); ++trajectory_idx)
    {
      Eigen::Vector2d current_trajectory_point(trajectory[trajectory_idx][0], trajectory[trajectory_idx][1]);
      Eigen::Vector2d closest_track_point = track->getClosestTrackPoint(current_trajectory_point);

      EXPECT_TRUE((current_trajectory_point - closest_track_point).norm() < track->getWidth());
    }
  }
}

int main(int ac, char* av[])
{
  testing::InitGoogleTest(&ac, av);
  return RUN_ALL_TESTS();
}
