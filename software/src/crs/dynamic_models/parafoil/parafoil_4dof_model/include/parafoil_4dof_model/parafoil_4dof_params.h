#ifndef PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_PARAMS_H
#define PARAFOIL_4DOF_MODEL_PARAFOIL_4DOF_PARAMS_H

#include <iostream>
#include <yaml-cpp/yaml.h>

namespace crs_models
{
namespace parafoil_4dof_model
{

struct parafoil_4dof_params
{
  // world params
  double rho;
  // physical params
  double m;
  double S;
  // aero params
  double C_L0;
  double C_Ldelta_s;
  double C_D0;
  double C_Ddelta_s;
  // modeling params
  double T_phi;
  double K_phi;
  // actuator params
  double deflection_symmetric_max;
  double deflection_asymmetric_max;
};

inline std::ostream& operator<<(std::ostream& os,
                                const parafoil_4dof_params& params)  // how to print struct (state) to output stream
                                                                     // (os)
{
  os << "pacejka_params:" << std::endl;
  os << "rho: " << std::to_string(params.rho) << std::endl;
  os << "m: " << std::to_string(params.m) << std::endl;
  os << "S: " << std::to_string(params.S) << std::endl;
  os << "C_L0: " << std::to_string(params.C_L0) << std::endl;
  os << "C_Ldelta_s: " << std::to_string(params.C_Ldelta_s) << std::endl;
  os << "C_D0: " << std::to_string(params.C_D0) << std::endl;
  os << "C_Ddelta_s: " << std::to_string(params.C_Ddelta_s) << std::endl;
  os << "T_phi: " << std::to_string(params.T_phi) << std::endl;
  os << "K_phi: " << std::to_string(params.K_phi) << std::endl;
  os << "deflection_symmetric_max: " << std::to_string(params.deflection_symmetric_max) << std::endl;
  os << "deflection_asymmetric_max: " << std::to_string(params.deflection_asymmetric_max) << std::endl;
  return os;
}

inline bool loadParamsFromFile(std::string file_path, parafoil_4dof_params& params)
{
  try
  {
    YAML::Node file_content = YAML::LoadFile(file_path);
    params.rho = file_content["rho"].as<double>();
    params.m = file_content["m"].as<double>();
    params.S = file_content["S"].as<double>();
    params.C_L0 = file_content["C_L0"].as<double>();
    params.C_Ldelta_s = file_content["C_Ldelta_s"].as<double>();
    params.C_D0 = file_content["C_D0"].as<double>();
    params.C_Ddelta_s = file_content["C_Ddelta_s"].as<double>();
    params.T_phi = file_content["T_phi"].as<double>();
    params.K_phi = file_content["K_phi"].as<double>();
    params.deflection_symmetric_max = file_content["deflection_symmetric_max"].as<double>();
    params.deflection_asymmetric_max = file_content["deflection_asymmetric_max"].as<double>();
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Error reading file at " << file_path << e.msg << std::endl;
    return false;
  }
  return true;
}
}  // namespace parafoil_4dof_model
}  // namespace crs_models
#endif
