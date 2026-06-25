/*******************************************************************************
 * @file    estimator_interface.hpp
 * @brief   Describes abstract estimator interfaces.
 ******************************************************************************/

#pragma once

namespace chronos {

namespace interface {

/**
 * @brief Template class for an estimator that produces an estimate of T.
 *        The quantities used in estimation are grouped into I.
 * @tparam T The estimator's estimate type.
 * @tparam Q The estimator's input types.
 */
template <typename T, typename... Q> class Estimator {
  public:
    /**
     * @brief Obtain an estimate from the estimator.
     *
     * The estimation is based on the inputs and may fail, for example if the
     * inputs are out of range or otherwise invalid.
     * @param[out] estimate The estimate data.
     * @param[in] inputs The input data.
     * @returns true if the estimation was successful, false otherwise
     */
    virtual bool step(T &estimate, const Q &...inputs) = 0;
};

}; // namespace interface

}; // namespace chronos
