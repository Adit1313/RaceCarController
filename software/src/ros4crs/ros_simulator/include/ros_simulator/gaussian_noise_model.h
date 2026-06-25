#ifndef ROS_SIMULATOR_GAUSSIAN_NOISE_MODEL_H
#define ROS_SIMULATOR_GAUSSIAN_NOISE_MODEL_H

#include "ros_simulator/common/noise_model.h"
#include <random>
#include <Eigen/Dense>

namespace ros_simulator
{
class GaussianNoiseModel : NoiseModel
{
private:
  Eigen::VectorXd mean_;
  bool has_mean = false;
  double outlier_proba_ = 0.0;
  double outlier_scale_ = 0.0;

  // Used to cache the cholesky decomposition of the covariance matrix
  // and the mean
  Eigen::MatrixXd last_Q_;
  Eigen::MatrixXd last_chol_decomp_;
  Eigen::VectorXd last_mean_;
  bool is_initialized_ = false;

  // Random number generator
  // Seed the random number generator
  std::normal_distribution<double> distribution;

  std::unique_ptr<std::mt19937> generator;
  std::function<double()> normal_distribution_fnc_;

public:
  GaussianNoiseModel(int seed)  // Option 1: Set a seed, mean will default to zero
  {
    generator = std::make_unique<std::mt19937>(seed);
  };

  GaussianNoiseModel(int seed, Eigen::VectorXd mean) : GaussianNoiseModel(seed)  // Option 2: Set a seed and a mean
  {
    mean_ = mean;
    has_mean = true;
  };

  GaussianNoiseModel(int seed, double outlier_proba, double outlier_scale) : GaussianNoiseModel(seed)
  {
    outlier_proba_ = outlier_proba;
    outlier_scale_ = outlier_scale;
  };

  GaussianNoiseModel(int seed, Eigen::VectorXd mean, double outlier_proba, double outlier_scale)
    : GaussianNoiseModel(seed)
  {
    mean_ = mean;
    has_mean = true;
    outlier_proba_ = outlier_proba;
    outlier_scale_ = outlier_scale;
  };

  Eigen::MatrixXd sampleNoiseFromCovMatrix(const Eigen::MatrixXd& Q) override
  {
    int size = Q.rows();  // Dimensionality (rows)

    if (!is_initialized_ || Q.rows() != last_Q_.rows() || Q != last_Q_)
    {
      // Cache miss, we need to recompute the cholesky decomposition

      is_initialized_ = true;

      // Q changed, we need to recompute the cholesky decomposition
      if (!has_mean)
      {
        mean_.setZero(size);
        has_mean = true;
      }

      Eigen::MatrixXd normTransform(size, size);
      Eigen::LLT<Eigen::MatrixXd> cholSolver(Q);

      // We can only use the cholesky decomposition if
      // the covariance matrix is symmetric, pos-definite.
      // But a covariance matrix might be pos-semi-definite.
      // In that case, we'll go to an EigenSolver
      if (cholSolver.info() == Eigen::Success)
      {
        // Use cholesky solver
        normTransform = cholSolver.matrixL();
      }
      else
      {
        // Use eigen solver
        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(Q);
        normTransform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
      }

      // Update the cached values
      last_Q_ = Q;
      last_chol_decomp_ = normTransform;
      last_mean_ = mean_;
    }

    auto rnd_norm = [&]() { return distribution(*generator); };
    Eigen::VectorXd noise_norm = Eigen::VectorXd::NullaryExpr(size, rnd_norm);

    // Add outliers by adding outlier scale to the diagonal of the cholesky decomposition
    if (outlier_proba_ > 0)
    {
      Eigen::MatrixXd outlier_decomp_ = Eigen::MatrixXd::Zero(size, size);
      for (int i = 0; i < size; i++)
      {
        if ((double(std::rand()) / double(RAND_MAX)) < outlier_proba_)
        {
          outlier_decomp_(i, i) = outlier_scale_;
        }
      }
      return (last_chol_decomp_ + outlier_decomp_) * noise_norm + last_mean_;
    }
    else
    {
      return last_chol_decomp_ * noise_norm + last_mean_;
    }
  }
};
};  // namespace ros_simulator
#endif
