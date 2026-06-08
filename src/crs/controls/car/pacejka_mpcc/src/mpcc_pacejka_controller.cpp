#include "pacejka_mpcc/mpcc_pacejka_controller.h"

#ifdef BUILD_ACADOS_SOLVER
#include "pacejka_mpcc/solvers/acados_pacejka_mpcc_solver.h"
#endif
#ifdef BUILD_FORCES_SOLVER
#include "pacejka_mpcc/solvers/forces_pacejka_mpcc_solver.h"
#endif
#ifdef BUILD_ACADOS_CURVILINEAR_SOLVER
#include "pacejka_mpcc/solvers/acados_pacejka_curvilinear_mpcc_solver.h"
#endif

namespace crs_controls::pacejka_mpcc
{

std::shared_ptr<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                            crs_models::pacejka_model::pacejka_car_state,
                                            crs_models::pacejka_model::pacejka_car_input>>
controllerFactory(mpcc_pacejka_config config, std::shared_ptr<crs_models::pacejka_model::DiscretePacejkaModel> model,
                  std::shared_ptr<StaticTrackTrajectory> track)
{
  std::shared_ptr<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                              crs_models::pacejka_model::pacejka_car_state,
                                              crs_models::pacejka_model::pacejka_car_input>>
      base_controller;
#ifdef BUILD_ACADOS_SOLVER
  if (config.solver_type == "ACADOS")
  {
    auto controller =
        std::make_shared<PacejkaMpccController<solvers::acados_solver::AcadosPacejkaMpccSolver>>(config, model, track);
    base_controller =
        std::static_pointer_cast<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                                             crs_models::pacejka_model::pacejka_car_state,
                                                             crs_models::pacejka_model::pacejka_car_input>>(controller);
  }
#endif

#ifdef BUILD_ACADOS_CURVILINEAR_SOLVER
  if (config.solver_type == "ACADOS_CURVILINEAR")
  {
    auto controller =
        std::make_shared<PacejkaMpccController<solvers::acados_solver::AcadosPacejkaCurvilinearMpccSolver>>(
            config, model, track);
    base_controller =
        std::static_pointer_cast<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                                             crs_models::pacejka_model::pacejka_car_state,
                                                             crs_models::pacejka_model::pacejka_car_input>>(controller);
  }
#endif

#ifdef BUILD_FORCES_SOLVER
  else if (config.solver_type == "FORCES")
  {
    auto controller =
        std::make_shared<PacejkaMpccController<solvers::forces_solver::ForcesPacejkaMpccSolver>>(config, model, track);
    base_controller =
        std::static_pointer_cast<crs_controls::MpcController<crs_models::pacejka_model::DiscretePacejkaModel,
                                                             crs_models::pacejka_model::pacejka_car_state,
                                                             crs_models::pacejka_model::pacejka_car_input>>(controller);
  }
#endif

  return base_controller;
}
}  // namespace crs_controls::pacejka_mpcc
