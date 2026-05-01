#ifndef PACEJKA_MODEL_PACEJKA_PARAMS_H
#define PACEJKA_MODEL_PACEJKA_PARAMS_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_models
{
namespace pacejka_model
{
struct pacejka_params
{
  // car params
  double lr;
  double lf;
  double m;
  double I;

  // lateral force params
  double Df;
  double Cf;
  double Bf;
  double Dr;
  double Cr;
  double Br;

  // longitudinal force params
  double Cm1;
  double Cm2;

  // friction parameters
  double Cd0;  ///< 0th order friction coefficient
  double Cd1;  ///< 1st order friction coefficient
  double Cd2;  ///< 2nd order friction coefficient

  /* Optional parameters --------------------------------------------------- */

  /**
   * @brief Torque split parameter between front and rear wheels.
   *
   * The torque split changes the force distribution from the motor to rear
   * and front wheels. All-wheel drive is 0.5, rear-wheel drive is 1.
   */
  double gamma = 1.0;

  double wheel_radius = 0;
  double car_width = 0;

  /**
   * @brief Velocity value at which the polynomial approximation of arctan(w/x) is used to avoid numerical problems with
   * the atan at zero velocity.
   */
  double eps = 0.2;
};

inline std::ostream& operator<<(std::ostream& os,
                                const pacejka_params& params)  // how to print struct (state) to output stream (os)
{
  os << "pacejka_params:" << std::endl;
  os << "lr: " << std::to_string(params.lr) << std::endl;
  os << "lf: " << std::to_string(params.lf) << std::endl;
  os << "m: " << std::to_string(params.m) << std::endl;
  os << "I: " << std::to_string(params.I) << std::endl;

  os << "Df: " << std::to_string(params.Df) << std::endl;
  os << "Cf: " << std::to_string(params.Cf) << std::endl;
  os << "Bf: " << std::to_string(params.Bf) << std::endl;
  os << "Dr: " << std::to_string(params.Dr) << std::endl;
  os << "Cr: " << std::to_string(params.Cr) << std::endl;
  os << "Br: " << std::to_string(params.Br) << std::endl;
  os << "Cm1: " << std::to_string(params.Cm1) << std::endl;
  os << "Cm2: " << std::to_string(params.Cm2) << std::endl;
  os << "Cd0: " << std::to_string(params.Cd0) << std::endl;
  os << "Cd1: " << std::to_string(params.Cd1) << std::endl;
  os << "Cd2: " << std::to_string(params.Cd2) << std::endl;

  os << "car_width: " << std::to_string(params.car_width) << std::endl;
  os << "wheel_radius: " << std::to_string(params.wheel_radius) << std::endl;
  os << "gamma: " << std::to_string(params.gamma) << std::endl;
  os << "eps: " << std::to_string(params.eps) << std::endl;
  return os;
}

inline bool loadParamsFromFile(std::string file_path, pacejka_params& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(file_path);
    params.lr = file_content["lr"].as<double>();
    params.lf = file_content["lf"].as<double>();
    params.m = file_content["m"].as<double>();
    params.I = file_content["I"].as<double>();
    params.Df = file_content["Df"].as<double>();
    params.Cf = file_content["Cf"].as<double>();
    params.Bf = file_content["Bf"].as<double>();
    params.Dr = file_content["Dr"].as<double>();
    params.Cr = file_content["Cr"].as<double>();
    params.Br = file_content["Br"].as<double>();
    params.Cm1 = file_content["Cm1"].as<double>();
    params.Cm2 = file_content["Cm2"].as<double>();
    params.Cd0 = file_content["Cd0"].as<double>();
    params.Cd1 = file_content["Cd1"].as<double>();
    params.Cd2 = file_content["Cd2"].as<double>();

    // Optional Parameters
    if (file_content["wheel_radius"])
      params.wheel_radius = file_content["wheel_radius"].as<double>();
    if (file_content["car_width"])
      params.car_width = file_content["car_width"].as<double>();
    if (file_content["eps"])
      params.eps = file_content["eps"].as<double>();
    if (file_content["gamma"])
    {
      params.gamma = file_content["gamma"].as<double>();
    }
    else
    {
      // Fall back to backwards-compatible default value: RWD
      params.gamma = 1.0;
    }
  }
  catch (std::exception& e)
  {
    std::cout << "Error reading file at " << file_path << e.what() << std::endl;
    return false;
  }
  return true;
}
}  // namespace pacejka_model
}  // namespace crs_models
#endif
