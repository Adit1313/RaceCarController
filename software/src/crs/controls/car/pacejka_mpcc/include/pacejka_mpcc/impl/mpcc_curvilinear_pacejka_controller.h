#pragma once

/**
 *  Declaration of the template specializations of the PacejkaMpccController class
 *  for the curvilinear solver.
 */

#include "pacejka_mpcc/mpcc_pacejka_controller.h"

#include "pacejka_model/pacejka_car_state.h"
#include "pacejka_model/pacejka_car_input.h"

#include "pacejka_mpcc/solvers/acados_pacejka_curvilinear_mpcc_solver.h"

namespace crs_controls::pacejka_mpcc
{
using PacejkaCurvilinearMpccController =
    PacejkaMpccController<solvers::acados_solver::AcadosPacejkaCurvilinearMpccSolver>;

template <>
typename PacejkaCurvilinearMpccController::MpcParameters
PacejkaCurvilinearMpccController::generateCurrentSolverParameters();

template <>
typename PacejkaCurvilinearMpccController::StateArray
PacejkaCurvilinearMpccController::convertToLocalCoordinates(const crs_models::pacejka_model::pacejka_car_state& state,
                                                            const crs_models::pacejka_model::pacejka_car_input& input,
                                                            double theta);

template <>
crs_models::pacejka_model::pacejka_car_state PacejkaCurvilinearMpccController::convertToGlobalCoordinates(
    const PacejkaCurvilinearMpccController::StateArray& local_state);

template <>
typename PacejkaCurvilinearMpccController::StateArray
PacejkaCurvilinearMpccController::generateInitialization(const int reference_track_index,
                                                         const double current_velocity);
}  // namespace crs_controls::pacejka_mpcc
