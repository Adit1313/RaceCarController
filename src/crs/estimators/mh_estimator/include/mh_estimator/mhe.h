#ifndef SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MHE
#define SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MHE

#include <memory>

#include <dynamic_models/discrete_dynamic_model.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <Eigen/Dense>
#include <stdexcept>

#include <estimators/base_estimator.h>
#include <estimators/model_based_estimator.h>

namespace crs_estimators
{
namespace mhe
{
template <typename StateType, typename InputType>
class MHE : public ModelBasedEstimator<StateType, InputType>
{
public:
  typedef crs_models::DiscreteDynamicModel<StateType, InputType> discrete_model_type;
  // Constructor
  MHE(std::shared_ptr<discrete_model_type> discrete_model, StateType initial_state, InputType initial_input,
      std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> estimator, double start_delay,
      bool log_diagnostic_data = false)
    : discrete_model(discrete_model)
    , best_state_(initial_state)
    , previous_input_(initial_input)
    , estimator_(estimator)
    , mhe_delay_time_(start_delay)
    , log_diagnostic_data_(log_diagnostic_data){};

  InputType getLastInput() const override
  {
    return previous_input_;
  }

  double getLastValidTs() const override
  {
    return last_valid_ts_;
  }

  StateType getStateEstimate() override
  {
    return best_state_;
  }

  void resetStateEstimate(const StateType state) override
  {
    best_state_ = state;
  }

  virtual StateType getStateEstimate(const double timestamp) = 0;
  virtual std::vector<std::vector<double>> getPlannedTrajectory() = 0;

protected:
  std::shared_ptr<discrete_model_type> discrete_model;
  StateType best_state_;
  InputType previous_input_;
  std::shared_ptr<crs_estimators::ModelBasedEstimator<StateType, InputType>> estimator_;

  double last_valid_ts_ = -1;

  /**
   * @brief The timestamp when the MHE should be started.
   */
  double mhe_startup_time_ = 0.0;

  /**
   * @brief The delay time between starting the internal estimator and the MHE.
   */
  double mhe_delay_time_ = 0.0;

  /**
   * @brief If True, diagnostic data such as percentage of solver errors is logged
   */
  bool log_diagnostic_data_;

  int total_num_solves;
  int num_solver_errors;
};
}  // namespace mhe
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_MH_ESTIMATOR_INCLUDE_MHE */
