#ifndef ROCKET_6_DOF_MODEL_ROCKET_6_DOF_PARAMS_H
#define ROCKET_6_DOF_MODEL_ROCKET_6_DOF_PARAMS_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_models
{
namespace rocket_6_dof_model
{

struct rocket_6_dof_params
{
  // rocket_6_dof params
  double mass;
  double gravity_constant;
  double inertia_xx;
  double inertia_yy;
  double inertia_zz;
  double thrust_cog_offset;
  double thrust_magnitude_time_constant;
  double servo_angle_time_constant;
  double gimbal_a;
  double gimbal_b;
  double gimbal_c;
  double gimbal_d;
  double gimbal_e;
};

inline std::ostream& operator<<(std::ostream& os,
                                const rocket_6_dof_params& params)  // how to print struct (state) to output
                                                                    // stream (os)
{
  os << "rocket_6_dof_params:" << std::endl;
  os << " Mass: " << std::to_string(params.mass) << std::endl;
  os << " Gravity Constant: " << std::to_string(params.gravity_constant) << std::endl;
  os << " Inertia xx: " << std::to_string(params.inertia_xx) << std::endl;
  os << " Inertia yy: " << std::to_string(params.inertia_yy) << std::endl;
  os << " Inertia zz: " << std::to_string(params.inertia_zz) << std::endl;
  os << " Thrust CoG Offset: " << std::to_string(params.thrust_cog_offset) << std::endl;
  os << " Thrust Magnitude Time Constant: " << std::to_string(params.thrust_magnitude_time_constant) << std::endl;
  os << " Servo Angle Time Constant: " << std::to_string(params.servo_angle_time_constant) << std::endl;
  os << " gimbal_a: " << std::to_string(params.gimbal_a) << std::endl;
  os << " gimbal_b: " << std::to_string(params.gimbal_b) << std::endl;
  os << " gimbal_c: " << std::to_string(params.gimbal_c) << std::endl;
  os << " gimbal_d: " << std::to_string(params.gimbal_d) << std::endl;
  os << " gimbal_e: " << std::to_string(params.gimbal_e) << std::endl;
  return os;
}

inline bool loadParamsFromFile(std::string file_path, rocket_6_dof_params& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(file_path);
    params.mass = file_content["mass"].as<double>();
    params.gravity_constant = file_content["gravity_constant"].as<double>();
    params.inertia_xx = file_content["inertia_xx"].as<double>();
    params.inertia_yy = file_content["inertia_yy"].as<double>();
    params.inertia_zz = file_content["inertia_zz"].as<double>();
    params.thrust_cog_offset = file_content["thrust_cog_offset"].as<double>();
    params.thrust_magnitude_time_constant = file_content["thrust_magnitude_time_constant"].as<double>();
    params.servo_angle_time_constant = file_content["servo_angle_time_constant"].as<double>();
    params.gimbal_a = file_content["gimbal_a"].as<double>();
    params.gimbal_b = file_content["gimbal_b"].as<double>();
    params.gimbal_c = file_content["gimbal_c"].as<double>();
    params.gimbal_d = file_content["gimbal_d"].as<double>();
    params.gimbal_e = file_content["gimbal_e"].as<double>();
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Error reading file at " << file_path << e.msg << std::endl;
    return false;
  }
  return true;
}
}  // namespace rocket_6_dof_model
}  // namespace crs_models
#endif
