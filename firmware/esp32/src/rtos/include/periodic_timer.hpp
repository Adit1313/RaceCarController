/*******************************************************************************
 * @file    periodic_timer.hpp
 * @brief   Interface for an RTOS tick-based periodic timer.
 ******************************************************************************/

#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

namespace chronos {

namespace rtos {

/**
 * @brief Class that wraps an RTOS tick-based periodic timer.
 *
 * This class bundles all the memory needed for allocating an RTOS timer as well
 * as the setup and destruction functionality.
 *
 * This class can be used by first instantiating the object, then calling
 * start(). This will start the underlying RTOS timer. Subsequently, the task
 * that instantiated this timer may call wait_interval() to block until the next
 * interval period begins.
 *
 * @note The task that instantiates this object must be the same one waiting for
 *       the timer to expire.
 */
class PeriodicTimer {

    // Todo: follow rule of three
  public:
    /**
     * @brief Construct a new periodic timer.
     *
     * @param name A descriptive name to be given to the timer.
     * @param period_ms The period of the timer in milliseconds.
     */
    PeriodicTimer(const char *name, uint16_t period_ms);

    /**
     * @brief Destructor. Deletes the timer.
     * @note If the destructor is called while a task is still waiting, this
     *       will result in undefined behaviour.
     */
    ~PeriodicTimer();

    /**
     * @brief Wait for the next interval of the period to begin.
     *
     * Will put the calling task in a blocking state and wait for the next
     * timer period to begin.
     *
     * @note This is not thread-safe! In general, only the last simultaneously
     *       waiting task will be unblocked.
     * @note If the timer has not been started via start(), will block
     *       indefinitely until another task calls start().
     */
    void wait_interval();

    /**
     * @brief Start the timer's periodic firing.
     *
     * If this method is not called until wait_interval() is invoked, the task
     * calling wait_interval() will block indefinitely.
     */
    void start();

    /**
     * @brief Stop the timer from firing.
     *
     * This will block the task that is waiting for the next interval from
     * executing until the timer is restarted.
     */
    void stop();

    bool is_running() { return running_; };

    TaskHandle_t get_task_handle() { return associated_task; };

  private:
    uint16_t period_ms_; ///< Period in milliseconds

    StaticTimer_t timer_buf; ///< Memory buffer for timer
    TimerHandle_t timer;     ///< Underlying RTOS timer

    TaskHandle_t associated_task;

    bool running_;
};

}; // namespace rtos

}; // namespace chronos
