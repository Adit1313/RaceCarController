#pragma once

#include <algorithm>
#include <array>
#include <iostream>

namespace crs_controls::control_commons
{

enum class MpcExitCode : std::uint8_t
{
  SUCCEEDED = 0,
  FAILURE = 1,
  SOLVER_TIMEOUT = 2,
};

/**
 * @brief Shift the input sequence by one index
 */
template <typename T>
void shiftSequence(T& t)
{
  std::copy(t.begin() + 1, t.end(), t.begin());
}

}  // namespace crs_controls::control_commons
