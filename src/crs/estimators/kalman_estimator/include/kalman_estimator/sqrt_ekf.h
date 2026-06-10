#ifndef SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_SQRT_EKF
#define SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_SQRT_EKF

#include <memory>

#include <dynamic_models/discrete_dynamic_model.h>
#include <dynamic_models/utils/data_conversion.h>
#include <sensor_models/sensor_measurement.h>
#include <sensor_models/sensor_model.h>

#include <Eigen/Dense>
#include <stdexcept>

#include <estimators/model_based_estimator.h>
#include <kalman_estimator/kalman_estimator.h>

#include <boost/math/distributions/chi_squared.hpp>
#include <boost/math/special_functions/gamma.hpp>

/**
The following square root ekf implementation is based on the following paper:
Tracy, K. (2022). "A Square-Root Kalman Filter Using Only QR Decompositions". arXiv preprint arXiv:2208.06452.

Note that the paper uses the following notation:
State = mu
State estimate covariance = Sigma
Sigma = F^T*F, i.e. F is the square root of the estimate covariance Sigma

Process noise covariance = W
Gamma_W = sqrt(W) = Cholesky decomposition of W

Measurement noise covariance = V
Gamma_V = sqrt(V) = Cholesky decomposition of V
*/

namespace crs_estimators
{
namespace kalman
{
template <typename StateType, typename InputType>
class SqrtEKF : public KalmanEstimator<StateType, InputType>
{
  typedef std::shared_ptr<crs_models::DiscreteDynamicModel<StateType, InputType>> discrete_model_ptr;
  typedef crs_estimators::kalman::KalmanEstimator<StateType, InputType> kalman;

public:
  // Constructor
  SqrtEKF(discrete_model_ptr discrete_model, StateType initial_state, InputType initial_input,
          Eigen::Matrix<double, StateType::NX, StateType::NX> Sigma_init,
          const std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>>
              outlier_rejection_params = {},
          bool log_diagnostic_data = false)
    : KalmanEstimator<StateType, InputType>(initial_state, initial_input, outlier_rejection_params, log_diagnostic_data)
    , discrete_model(discrete_model)
    , Sigma_prior(Sigma_init)
    , Sigma_posterior(Sigma_init)
    , F_prior(Sigma_init)
    , F_posterior(Sigma_init)
    , outlier_rejection_params(outlier_rejection_params)
  {
    // Compute Cholesky decomposition of process noise covariance W (In the paper process noise covariance is denoted as
    // the matrix W opposed to Q)
    Eigen::LLT<Eigen::MatrixXd> lltOfQ(discrete_model->getQ());  // compute the Cholesky decomposition of W
    Gamma_W = lltOfQ.matrixL();  // retrieve factor Gamma_W = sqrt(W) in the decomposition

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
    // A and B are the martices that will be filled with the jacobian values. A = df/dx, B = df/du
    Eigen::Matrix<double, StateType::NX, StateType::NX> A = Eigen::Matrix<double, StateType::NX, StateType::NX>::Zero();
    Eigen::Matrix<double, StateType::NX, InputType::NU> B = Eigen::Matrix<double, StateType::NX, InputType::NU>::Zero();
    discrete_model->getJacobian(kalman::x_prior_, input, timestep, A, B);

    if (!A.allFinite() || A.cwiseAbs().maxCoeff() > 1e3)
    {
      std::cout << "[WARN] NaN values or ill conditioned jacobian detected in predict step of ekf. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "F_prior: " << F_prior << std::endl;
      std::cout << "A: " << A << std::endl;
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
      std::cout << "F_prior: " << F_prior << std::endl;
      std::cout << "A: " << A << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      // Do not update anything to not publish NaN values
      return;
    }

    kalman::x_prior_ = x_predicted_;
    kalman::best_state_ = kalman::x_prior_;
    auto Gamma_W_disc = Gamma_W * sqrt(timestep);

    // Compute QR decomposition of Sigma:
    // Simga = F^T*F, where F = R of QR decomposition of stacked matrix [F*A^T; Gamma_W_disc]
    Eigen::MatrixXd stacked_1((F_prior * A.transpose()).rows() + Gamma_W_disc.rows(), (F_prior * A.transpose()).cols());
    stacked_1 << F_prior * A.transpose(), Gamma_W_disc;

    Eigen::HouseholderQR<Eigen::MatrixXd> householderQR_E(stacked_1);
    // Get R component (QR_R_1) of the QR decomposition
    Eigen::MatrixXd QR_R_1 =
        householderQR_E.matrixQR().topLeftCorner(StateType::NX, StateType::NX).template triangularView<Eigen::Upper>();

    Eigen::MatrixXd F_prior_tmp = QR_R_1;

    if (!F_prior_tmp.allFinite())
    {
      std::cout << "[WARN] NaN values detected in predict step for covariance. " << std::endl;
      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_predicted: " << x_predicted_ << std::endl;
      std::cout << "F_prior: " << F_prior << std::endl;
      std::cout << "A: " << A << std::endl;
      std::cout << "B: " << B << std::endl;
      std::cout << "Q: " << discrete_model->getQ() << std::endl;
      std::cout << "timestep: " << timestep << std::endl;
      std::cout << "F_prior_tmp: " << F_prior_tmp << std::endl;

      // Do not update anything to not publish NaN values
      return;
    }

    F_prior = F_prior_tmp;
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
    // C is the martices that will be filled with the jacobian values. C = df_sensor/dx
    Eigen::Matrix<double, -1, StateType::NX> C =
        Eigen::Matrix<double, -1, StateType::NX>::Zero(sensor_model->dimension, StateType::NX);
    sensor_model->getNumericalJacobian(kalman::x_prior_, kalman::previous_input_, C);
    Eigen::Matrix<double, -1, 1> z_hat = sensor_model->applyModel(kalman::x_prior_, kalman::previous_input_);

    auto y_hat = (data.measurement_data - z_hat);

    Eigen::MatrixXd Gamma_V = key_to_sensor_r_sqrt.find(data.sensor_key)->second;  // Cholesky decomposition of R

    // Compute QR decomposition of the innovation S:
    // S = G^T*G, where G = R of QR decomposition of stacked matrix [F*C^T; Gamma_V]
    Eigen::MatrixXd stacked_2((F_prior * C.transpose()).rows() + Gamma_V.rows(), (F_prior * C.transpose()).cols());
    stacked_2 << F_prior * C.transpose(), Gamma_V;

    Eigen::HouseholderQR<Eigen::MatrixXd> householderQR_2(stacked_2);
    // Get R component (QR_R_2) of the QR decomposition
    Eigen::MatrixXd QR_R_2 = householderQR_2.matrixQR()
                                 .topLeftCorner(Gamma_V.rows(), Gamma_V.rows())
                                 .template triangularView<Eigen::Upper>();

    Eigen::MatrixXd G = QR_R_2;

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
      Eigen::Matrix<double, -1, -1> S = G.transpose() * G;  // covariance of the innovation
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
                                                                                 // already reject the past 5
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
        double eps = 0.1;
        double weighted_mahalanobis_dist = 1 / (1 + exp(-mahalanobis_dist + eps));

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
          if (rejected_consec_meas[data.sensor_key] < max_consecutive_outliers)  // Only reject measurement if we didnt
                                                                                 // already reject the past 5
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

      // ============ UNKNOWN TYPE ============
      else
      {
        std::cout << "Unknown outlier rejection type: " << outlier_rejection_type << std::endl;
      }
    }

    // Kalman Gain L
    auto L = (G.inverse() * ((G.inverse()).transpose() * C) * F_prior.transpose() * F_prior).transpose();

    kalman::x_posterior_ =
        commons::convertToState<StateType, -1>(commons::convertToEigen(kalman::x_prior_) + L * y_hat);

    if (!commons::convertToEigen(kalman::x_posterior_).allFinite())
    {
      std::cout << "[WARN] NaN values detected in measurement udpate step. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_posterior_: " << kalman::x_posterior_ << std::endl;
      std::cout << "y_hat: " << y_hat << std::endl;
      std::cout << "L: " << L << std::endl;
      std::cout << "F_prior: " << F_prior << std::endl;
      std::cout << "R: " << sensor_model->getR() << std::endl;

      // Do not update state to not introduce nan values
      kalman::x_posterior_ = kalman::x_prior_;
      return;
    }

    // Compute QR decomposition of Sigma_posterior:
    // Sigma_posterior = F_posterior^T*F_posterior, where F_posterior = R of QR decomposition of stacked matrix:
    // [F_prior*(I-L*C)^T; Gamma_V*L^T]

    // Let T = F_prior*(I-L*C)^T
    Eigen::MatrixXd T = F_prior * (Eigen::MatrixXd::Identity(StateType::NX, StateType::NX) - L * C).transpose();
    // Stack [T; Gamma_V*L^T]
    Eigen::MatrixXd stacked_3(T.rows() + (Gamma_V * L.transpose()).rows(), T.cols());
    stacked_3 << T, Gamma_V * L.transpose();

    Eigen::HouseholderQR<Eigen::MatrixXd> householderQR_3(stacked_3);
    // Get R component (QR_R_3) of the QR decomposition
    Eigen::MatrixXd QR_R_3 =
        householderQR_3.matrixQR().topLeftCorner(StateType::NX, StateType::NX).template triangularView<Eigen::Upper>();
    Eigen::MatrixXd F_posterior_tmp = QR_R_3;

    if (!F_posterior_tmp.allFinite())
    {
      std::cout << "[WARN] NaN values detected in measurement udpate step. " << std::endl;

      std::cout << "Dumping states and covariance: " << std::endl;
      std::cout << "x_prior: " << kalman::x_prior_ << std::endl;
      std::cout << "x_posterior_: " << kalman::x_posterior_ << std::endl;
      std::cout << "y_hat: " << y_hat << std::endl;
      std::cout << "L: " << L << std::endl;
      std::cout << "F_prior: " << F_prior << std::endl;
      std::cout << "R: " << sensor_model->getR() << std::endl;
      std::cout << "F_posterior_tmp: " << F_posterior_tmp << std::endl;

      // Do not update state to not introduce nan values
      return;
    }

    F_posterior = F_posterior_tmp;
    F_prior = F_posterior;

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
                      std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>> sensor_model)
  {
    key_to_sensor_model_.insert(std::make_pair<>(sensor_key, sensor_model));

    Eigen::LLT<Eigen::MatrixXd> lltOfR(sensor_model->getR());  // compute the Cholesky decomposition of measruement
                                                               // noise covaraince Gamma_V
    Eigen::MatrixXd Gamma_V = lltOfR.matrixL();                // retrieve factor Gamma_R in the decomposition
    key_to_sensor_r_sqrt.insert(std::make_pair<>(sensor_key, Gamma_V));
  }

  /**
   * @brief Returns the posterior state estimate covariance matrix of the EKF
   *
   * @return Sigma_posterior
   */
  Eigen::Matrix<double, StateType::NX, StateType::NX> getPosteriorCovariance() const
  {
    // Sigma_posterior = F_posterior^T*F_posterior
    return F_posterior.transpose() * F_posterior;
  }

  // map saves objects that have a key and a value (here key = string, e.g. mocap, value = sensor_model e.g.
  // mocap_sensor_model)
  std::map<std::string, std::shared_ptr<crs_sensor_models::SensorModel<StateType, InputType>>> key_to_sensor_model_;
  std::map<std::string, Eigen::MatrixXd> key_to_sensor_r_sqrt;

private:
  /**
   * @brief Prior refers to before the measurement update step of the EKF is done.
   *        It is based solely on the forward dynamic predictions.
   */
  Eigen::Matrix<double, StateType::NX, StateType::NX> Sigma_prior;
  Eigen::Matrix<double, StateType::NX, StateType::NX> Sigma_posterior;
  Eigen::Matrix<double, StateType::NX, StateType::NX> F_prior;
  Eigen::Matrix<double, StateType::NX, StateType::NX> F_posterior;

  Eigen::Matrix<double, -1, 1> innovation;

  Eigen::MatrixXd Gamma_W;

  discrete_model_ptr discrete_model;

  std::map<std::string, int> rejected_consec_meas;
  std::map<std::string, int> total_num_meas;
  std::map<std::string, int> rejected_num_meas;
  // Map that saves the number of consecutive rejected measurements for each sensor
  std::vector<std::pair<std::string, std::tuple<bool, std::string, double, int>>> outlier_rejection_params;
};
}  // namespace kalman
}  // namespace crs_estimators
#endif /* SRC_CRS_ESTIMATORS_KALMAN_ESTIMATOR_INCLUDE_KALMAN_ESTIMATOR_SQRT_EKF */
