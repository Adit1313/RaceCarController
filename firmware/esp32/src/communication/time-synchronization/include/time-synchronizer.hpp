/*******************************************************************************
 * @file    time-synchronizer.hpp
 * @brief   Synchronization with a local NTP server.
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <ctime>

namespace chronos {
namespace time {

/**
 * @brief Synchronizes the local clock with an NTP server.
 */
class TimeSynchronizer {
  public:
    /**
     * @brief Construct a TimeSynchronizer.
     * @note This will immediately start synchronizing time with an NTP server.
     */
    TimeSynchronizer();

    /**
     * @brief Destroy the Time Synchronizer object
     * @note This will stop the time synchronization.
     */
    ~TimeSynchronizer();

    /**
     * @brief Get the current time.
     * @return The current time in microseconds since the epoch.
     *         Only valid if synchronized.
     */
    uint64_t get_time();

    /**
     * @brief Check if the time is synchronized.
     * @returns True if the time is synchronized, false otherwise.
     */
    bool is_synchronized();

    /**
     * @brief Internal callback for the SNTP client.
     * @param tv The time value that was received from the NTP server.
     */
    void synchronization_callback(struct timeval *tv);

  private:
    bool synchronized_ = false;
};

} // namespace time
} // namespace chronos
