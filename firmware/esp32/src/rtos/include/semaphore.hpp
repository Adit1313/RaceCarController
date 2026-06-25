/*******************************************************************************
 * @file    semaphore.hpp
 * @brief   Abstraction for an RTOS binary semaphore.
 ******************************************************************************/

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace chronos {

namespace rtos {

class Semaphore {
  public:
    /**
     * @brief Construct a new Semaphore.
     */
    Semaphore();

    // No copy constructor / copy assignment possible with this RTOS semaphore.
    Semaphore(const Semaphore &sem) = delete;
    Semaphore &operator=(const Semaphore &sem) = delete;

    /**
     * @brief Attempt to acquire the semaphore, waiting for timeout if necessary
     * @param tick_timeout Number of milliseconds to wait for timeout.
     */
    bool acquire(uint32_t timeout_ms);

    /** Release the semaphore and allow it to be acquired again. */
    void release();

    ~Semaphore();

  private:
    SemaphoreHandle_t semaphore_handle_; ///< Underlying RTOS semaphore handle
    StaticSemaphore_t static_buffer_;    ///< Memory used for the semaphore
};

}; // namespace rtos

}; // namespace chronos
