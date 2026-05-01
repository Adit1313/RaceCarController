#ifndef SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF
#define SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF

#include <memory>

#include <dynamic_models/discrete_dynamic_model.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <Eigen/Dense>
#include <stdexcept>

#include <estimators/model_based_estimator.h>
#include <kalman_estimator/kalman_estimator.h>

#define DEBUG_MODE false
#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/special_functions/gamma.hpp>

namespace crs_estimators
{
namespace kalman
{
template <typename StateType, typename InputType>
class DiscreteEKF : public KalmanEstimator<StateType, InputType>
{
  typedef std::shared_ptr<crs_models::DiscreteDynamicModel<StateType, InputType>> discrete_model_ptr;
  typedef crs_estimators::kalman::KalmanEstimator<StateType, InputType> kalman;

public:
  // Constructor with input
  DiscreteEKF(discrete_model_ptr discrete_model, StateType initial_state, InputType initial_input,
              Eigen::Matrix<double, StateType::NX, StateType::NX> P_init,
              const std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>>
                  outlier_rejection_params = {},
              bool log_diagnostic_data = false)
    : KalmanEstimator<StateType, InputType>(initial_state, initial_input, outlier_rejection_params, log_diagnostic_data)
    , P_prior(P_init)
    , P_posterior(P_init)
    , outlier_rejection_params(outlier_rejection_params)
    , discrete_model(discrete_model)
  {
    // Initialize rejection counter for each sensor to 0
    for (auto key_value : outlier_rejection_params)
    {
      auto sensor_name = key_value.first;
      rejected_consec_meas[sensor_name] = 0;
    }
  };

  // Constructor
  DiscreteEKF(discrete_model_ptr discrete_model, StateType initial_state,
              Eigen::Matrix<double, StateType::NX, StateType::NX> P_init,
              const std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>>
                  outlier_rejection_params = {},
              bool log_diagnostic_data = false)
    : KalmanEstimator<StateType, InputType>(initial_state, outlier_rejection_params, log_diagnostic_data)
    , P_prior(P_init)
    , P_posterior(P_init)
    , outlier_rejection_params(outlier_rejection_params)
    , discrete_model(discrete_model)
  {
    // Initialize rejection counter for each sensor to 0
    for (auto key_value : outlier_rejection_params)
    {
      auto sensor_name = key_value.first;
      rejected_consec_meas[sensor_name] = 0;
    }
  };

  /**
   * @brief This is the prediction step of the EKF. The state is predicated based on the model.
   *        This function only updates the prior state of the ekf
   *
   * @param input e.g. control input
   * @param timestep how long to apply the model for (how long to integrate for)
   */
  void predict(const InputType& input, const double timestep)
  {
    // F and B are the martices that will be filled with the jacobian values. F = df/dx, B = df/du
    Eigen::Matrix<double, StateType::NX, StateType::NX> F = Eigen::Matrix<double, StateType::NX, StateType::NX>::Zero();
    Eigen::Matrix<double, StateType::NX, InputType::NU> B = Eigen::Matrix<double, StateType::NX, InputType::NU>::Zero();

    discrete_model->getJacobian(kalman::x_prior_, input, timestep, F, B);
    // get max value of F using eigen

    // tax abs value of f
    if (!F.allFinite() || F.cwiseAbs().maxCoeff() > 1e3)
    {
      std::cout << "[WARN] NaN values or ill conditioned jacobian detected in predict step from ekf. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "F: " << F << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      // Do not update anything to not publish NaN values
      return;
    }

    if (DEBUG_MODE)
    {
      std::cout << "---------------------------------------------------" << std::endl;
      std::cout << "DiscreteEKF::predict for " << timestep << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "input: " << input << std::endl;
      // std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "F: " << F << std::endl;
    }

    if (!F.allFinite() || F.cwiseAbs().maxCoeff() > 1e3)
    {
      std::cout << "[WARN] NaN values or ill conditioned jacobian detected in predict step from ekf. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "F: " << F << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      // Do not update anything to not publish NaN values
      return;
    }

    auto x_predicted_ = discrete_model->applyModel(kalman::x_prior_, input, timestep);

    if (!commons::convertToEigen(x_predicted_).allFinite())
    {
      std::cout << "[WARN] NaN values detected in predict step for state. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_predicted: " << x_predicted_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "F: " << F << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      // Do not update anything to not publish NaN values
      return;
    }

    Eigen::MatrixXd P_prior_tmp = F * P_prior * F.transpose() + timestep * discrete_model->getQ();
    if (!P_prior_tmp.allFinite())
    {
      std::cout << "[WARN] NaN values detected in predict step for covariance. " << std::endl;
      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_predicted: " << x_predicted_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "F: " << F << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      std::cout << "P_prior_tmp: " << P_prior_tmp << std::endl;

      // Do not update anything to not publish NaN values
      return;
    }

    kalman::x_prior_ = x_predicted_;
    kalman::best_state_ = kalman::x_prior_;
    P_prior = P_prior_tmp;
  }

  /**
   * @brief This is the measurement update step of the EKF.
   *        The state prediction based on the model is updated using measurement inofrmation.
   *        This function updates the prior and posterior state of the ekf
   *
   * @param data measurement information, e.g. type of sensor, measurement data
   */
  void measurementUpdate(const crs_sensor_models::measurement& data)
  {
    // Get sensor model for key with validity checks
    auto entry = key_to_sensor_model_.find(data.sensor_key);  // entry is pair containing (key, sensor_model)
    if (entry == key_to_sensor_model_.end())
    {  // Sensor key did not exist
      throw std::invalid_argument("measurementUpdate. No SensorModel found for given key: " + data.sensor_key);
    }
    std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>> sensor_model = entry->second;
    // Start real measurement update
    // H is the martices that will be filled with the jacobian values. H = df_sensor/dx
    Eigen::Matrix<double, -1, StateType::NX> H =
        Eigen::Matrix<double, -1, StateType::NX>::Zero(sensor_model->dimension, StateType::NX);

    sensor_model->getNumericalJacobian(kalman::x_prior_, kalman::previous_input_, H);
    Eigen::Matrix<double, -1, 1> z_hat = sensor_model->applyModel(kalman::x_prior_, kalman::previous_input_);

    auto y_hat = (data.measurement_data - z_hat);
    // // Update P. We use the Joseph form to avoid numerical issues, particularly with the Pacejka model.
    Eigen::Matrix<double, -1, -1> S = H * P_prior * H.transpose() + sensor_model->getR();

    // ============ OUTLIER REJECTION ============
    float use_outlier_rejection = 0.0;
    float outlier_threshold = 1.0;
    std::string outlier_rejection_type;
    int max_consecutive_outliers = 0;
    total_num_meas[data.sensor_key] += 1;

    // Load parameters for outlier rejection out of map
    for (auto key_value :
         kalman::outlier_rejection_params_)  // outlier_rejection_params_ is a vector of pairs: <sensor_key,
                                             // tuple<use_outlier_rejection, outlier_rejection_type, outlier_threshold>>
    {
      if (key_value.first == data.sensor_key)
      {
        use_outlier_rejection = std::get<0>(key_value.second);

        if (use_outlier_rejection)
        {
          outlier_rejection_type = std::get<1>(key_value.second);
          outlier_threshold = std::get<2>(key_value.second);
          max_consecutive_outliers = std::get<3>(key_value.second);
        }
      }
    }

    if (use_outlier_rejection)
    {
      // ---------- ADD DIAGNOSTICS ----------
      if (kalman::log_diagnostic_data_)
      {
        // add number of rejected measurements and total number of measurements
        std::vector<float> rejected_data = { float(rejected_num_meas[data.sensor_key]),
                                             float(total_num_meas[data.sensor_key]) };
        std::string rejected_data_name = data.sensor_key + "/rejection_stats";
        BaseEstimator<StateType>::logDiagnosticData(rejected_data_name, rejected_data);
      }

      // ============ COVARIANCE THRESHOLD ============
      if (outlier_rejection_type == "cov_threshold")
      {
        // ------------- Add correct threshod data to diagnostic data -------------
        if (kalman::log_diagnostic_data_)
        {
          // add norm of innovation as well as used threshold
          std::vector<float> threshold_data = { float(y_hat.norm()),
                                                float(outlier_threshold * std::sqrt(S.diagonal().sum())) };
          std::string threshold_data_name = data.sensor_key + "/threshold_data";
          BaseEstimator<StateType>::logDiagnosticData(threshold_data_name, threshold_data);
        }
        // -------------------------------------------------------------------------
        // 3σ outlier rejection based on prior covariance
        if (y_hat.norm() > outlier_threshold * std::sqrt(S.diagonal().sum()))
        {
          rejected_consec_meas[data.sensor_key] += 1;
          if (rejected_consec_meas[data.sensor_key] < max_consecutive_outliers)  // Only reject measurement if we didnt
                                                                                 // already reject the max allow number
          {
            rejected_num_meas[data.sensor_key] += 1;
            return;
          }
          else
          {
            std::cout << "[WARN] NOT rejecting outlier. Too many consecutive outliers detected in " << data.sensor_key
                      << std::endl;
            rejected_consec_meas[data.sensor_key] = 0;
          }
        }
        else
        {
          rejected_consec_meas[data.sensor_key] = 0;
        }
      }

      // ============ CHI SQUARED ============
      else if ((outlier_rejection_type) == "chi_squared")
      {
        double mahalanobis_dist_squared = y_hat.transpose() * S.inverse() * y_hat;
        double mahalanobis_dist = sqrt(mahalanobis_dist_squared);
        // double eps = 0.1;
        // double weighted_mahalanobis_dist = 1 / (1 + exp(-mahalanobis_dist + eps));

        // t = 2 * gamma_p_inv(v / 2, p)
        int nr_deg = sensor_model->dimension;
        double p = outlier_threshold;
        auto t = 2 * boost::math::gamma_p_inv(nr_deg / 2, p * tgamma(nr_deg / 2));

        // ------------- Add correct threshod data to diagnostic data -------------
        if (kalman::log_diagnostic_data_)
        {
          // add norm of innovation as well as used threshold
          std::vector<float> threshold_data = { float(mahalanobis_dist), float(t) };
          std::string threshold_data_name = data.sensor_key + "/threshold_data";
          BaseEstimator<StateType>::logDiagnosticData(threshold_data_name, threshold_data);
        }
        // -------------------------------------------------------------------------

        if ((mahalanobis_dist > t))
        {
          rejected_consec_meas[data.sensor_key] += 1;
          if (rejected_consec_meas[data.sensor_key] < max_consecutive_outliers)  // Only reject measurement if we
                                                                                 // didnt already reject the past 5
                                                                                 // measurements
          {
            rejected_num_meas[data.sensor_key] += 1;
            return;
          }
          else
          {
            std::cout << "[WARN] NOT rejecting outlier. Too many consecutive outliers detected in " << data.sensor_key
                      << std::endl;
            rejected_consec_meas[data.sensor_key] = 0;
          }
        }
        else
        {
          rejected_consec_meas[data.sensor_key] = 0;
        }
      }
      // ============ CHI SQUARED ============
      else if ((outlier_rejection_type) == "chi_squared")
      {
        double mahalanobis_dist_squared = y_hat.transpose() * S.inverse() * y_hat;
        double mahalanobis_dist = sqrt(mahalanobis_dist_squared);
        // double eps = 0.1;
        // double weighted_mahalanobis_dist = 1 / (1 + exp(-mahalanobis_dist + eps));

        // t = 2 * gamma_p_inv(v / 2, p)
        int nr_deg = sensor_model->dimension;
        double p = outlier_threshold;
        auto t = 2 * boost::math::gamma_p_inv(nr_deg / 2, p * tgamma(nr_deg / 2));

        if ((mahalanobis_dist > t))
        {
          rejected_consec_meas[data.sensor_key] += 1;
          if (rejected_consec_meas[data.sensor_key] < max_consecutive_outliers)  // Only reject measurement if we didnt
                                                                                 // already reject the past 5
                                                                                 // measurements
          {
            std::cout << "Rejecting outlier for Sensor: " << data.sensor_key << " Innovation norm: " << y_hat.norm()
                      << " Threshold: " << outlier_threshold * std::sqrt(S.diagonal().sum()) << std::endl;
            return;
          }
          else
          {
            std::cout << "[WARN] NOT rejecting outlier. Too many consecutive outliers detected in " << data.sensor_key
                      << std::endl;
            rejected_consec_meas[data.sensor_key] = 0;
          }
        }
        else
        {
          rejected_consec_meas[data.sensor_key] = 0;
        }
      }
      // ============ UNKNOWN TYPE ============
      else
      {
        std::cout << "Unknown outlier rejection type: " << outlier_rejection_type << std::endl;
      }
    }

    auto K = P_prior * H.transpose() * S.inverse();

    Eigen::Matrix<double, StateType::NX, StateType::NX> I_KH =
        Eigen::MatrixXd::Identity(StateType::NX, StateType::NX) - K * H;

    kalman::x_posterior_ =
        commons::convertToState<StateType, -1>(commons::convertToEigen(kalman::x_prior_) + K * y_hat);

    double max_post_state = commons::convertToEigen(kalman::x_posterior_).cwiseAbs().maxCoeff();
    if (!commons::convertToEigen(kalman::x_posterior_).allFinite() || max_post_state > 1e3)
    {
      std::cout << "[WARN] NaN values detected in measurement udpate step. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_posterior_: " << kalman::x_posterior_ << std::endl;
      std::cout << "measured meas: " << data.measurement_data << std::endl;
      std::cout << "estimated mas: " << z_hat << std::endl;
      std::cout << "K: " << K << std::endl;
      std::cout << "H: " << H << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "S: " << S << std::endl;
      std::cout << "R: " << sensor_model->getR() << std::endl;

      // Do not update state to not introduce nan values
      kalman::x_posterior_ = kalman::x_prior_;
      return;
    }

    Eigen::MatrixXd P_posterior_tmp =
        I_KH * P_prior * I_KH.transpose() + K * sensor_model->getR() * K.transpose();  // more stable

    if (DEBUG_MODE)
    {
      std::cout << "---------------------------------------------------" << std::endl;
      std::cout << "DiscreteEKF::measurementUpdate for " << data.sensor_key << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "measured meas: " << data.measurement_data << std::endl;
      std::cout << "estimated mas: " << z_hat << std::endl;
      std::cout << "K: " << K << std::endl;
      std::cout << "H: " << H << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "S: " << S << std::endl;
      std::cout << "R: " << sensor_model->getR() << std::endl;
    }

    if (!P_posterior_tmp.allFinite())
    {
      std::cout << "[WARN] NaN values detected in measurement udpate step. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_posterior_: " << kalman::x_posterior_ << std::endl;
      std::cout << "y_hat: " << y_hat << std::endl;
      std::cout << "K: " << K << std::endl;
      std::cout << "H: " << H << std::endl;
      std::cout << "P_prior: " << P_prior << std::endl;
      std::cout << "S: " << S << std::endl;
      std::cout << "R: " << sensor_model->getR() << std::endl;
      std::cout << "P_posterior_tmp: " << P_posterior_tmp << std::endl;

      // Do not update state to not introduce nan values
      return;
    }
    P_posterior = P_posterior_tmp;
    P_prior = P_posterior;

    kalman::x_prior_ = kalman::x_posterior_;
    kalman::best_state_ = kalman::x_posterior_;
  }

  /**
   * @brief Adds a sensor model to the map (key and sensor_model)
   *
   * @param sensor_key
   * @param sensors_model
   */
  void addSensorModel(std::string sensor_key,
                      std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>> sensors_model)
  {
    key_to_sensor_model_.insert(std::make_pair<>(sensor_key, sensors_model));
  }

  /**
   * @brief Returns the posterior state estimate covariance matrix of the EKF
   *
   * @return P_Posterior
   */
  Eigen::Matrix<double, StateType::NX, StateType::NX> getPosteriorCovariance() const
  {
    return P_posterior;
  }

  // map saves objects that have a key and a value (here key = string, e.g. mocap, value = sensor_model e.g.
  // mocap_sensor_model)
  std::map<std::string, std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>>> key_to_sensor_model_;

private:
  Eigen::Matrix<double, StateType::NX, StateType::NX> P_prior;
  Eigen::Matrix<double, StateType::NX, StateType::NX> P_posterior;

  std::map<std::string, int> rejected_consec_meas;
  std::map<std::string, int> total_num_meas;
  std::map<std::string, int> rejected_num_meas;
  // Map that saves the number of consecutive rejected measurements for each sensor
  std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>> outlier_rejection_params;

  discrete_model_ptr discrete_model;
};  // namespace kalman
}  // namespace kalman
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_DISCRETE_EKF */
