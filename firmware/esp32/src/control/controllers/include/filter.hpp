/*******************************************************************************
 * @file    filter.hpp
 * @brief   Implementation of a DSP filter.
 ******************************************************************************/

#pragma once

namespace chronos {

namespace filter {

/**
 * @brief Implementation of a digital signal processing filter with an IIR.
 *
 * @tparam N Order of the filter
 */
template <int N> class DSPFilter {
  public:
    /**
     * @brief Construct a new DSPFilter object
     * @param b Numerator coefficients of the IIR transfer function.
     * @param a Denominator coefficients of the IIR transfer function.
     */
    constexpr DSPFilter(const std::array<float, N + 1> &b,
                        const std::array<float, N + 1> &a)
        : b_(b), a_(a), z_({0}){};

    /**
     * @brief Apply the filter to the input signal.
     * @param input The input signal.
     * @return The filtered signal.
     */
    float step(float x) {
        float y = b_[0] * x + z_[0];
        for (size_t i = 1; i != a_.size(); i++) {
            z_[i - 1] = b_[i] * x + z_[i] - a_[i] * y;
        }
        return y;
    }

  private:
    /** Numerator coefficients of the IIR transfer function. */
    const std::array<float, N + 1> b_;
    /** Denominator coefficients of the IIR transfer function. */
    const std::array<float, N + 1> a_;
    /** Delay registers. */
    std::array<float, N + 1> z_;
};

} // namespace filter
} // namespace chronos
