/*******************************************************************************
 * @file    periodic_timer.cpp
 * @brief   Interface for an RTOS tick-based periodic timer.
 ******************************************************************************/

#include "periodic_timer.hpp"

/* Local macros ------------------------------------------------------------- */

/** "Magic" task notification bit set to indicate the timer has fired. */
#define TIMER_FIRED_BIT (1 << 5)

/* Local function declaration ----------------------------------------------- */

/**
 * @brief Callback function that is executed if a chronos::PeriodicTimer fires.
 *
 * This can not be a C++ method as an underlying C API will invoke this
 * callback. This will dereference timer and send a notification to the
 * @c associated_task.
 * @param timer Pointer to the RTOS timer that fired.
 */
static void rtos_timer_fired_callback(TimerHandle_t expired_timer);

/* PeriodicTimer class implementation --------------------------------------- */

using namespace chronos::rtos;

PeriodicTimer::PeriodicTimer(const char *name, uint16_t period_ms)
    : period_ms_(period_ms), running_(false) {

    associated_task = xTaskGetCurrentTaskHandle();
    timer = xTimerCreateStatic(name,                       // name
                               pdMS_TO_TICKS(period_ms_),  // trigger period
                               pdTRUE,                     // repeating timer
                               (void *)this,               // callback argument
                               &rtos_timer_fired_callback, // callback function
                               &timer_buf                  // memory for timer
    );
}

PeriodicTimer::~PeriodicTimer() { xTimerDelete(timer, 0); }

void PeriodicTimer::wait_interval() {
    associated_task = xTaskGetCurrentTaskHandle();
    while (true) {
        // Wait for the timer to fire, i.e. time to execute another loop.
        uint32_t not_val = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Other notifications could be sent to control loop task as well,
        // ignore them & wait again
        if ((not_val & TIMER_FIRED_BIT) != 0) {
            return;
        }
    }
}

void PeriodicTimer::start() {
    xTimerStart(timer, 0);
    running_ = true;
}

void PeriodicTimer::stop() {
    xTimerStop(timer, 0);
    running_ = false;
}

/* Local function implementation -------------------------------------------- */

static void rtos_timer_fired_callback(TimerHandle_t expired_timer) {
    // Argument is a (void *) pointer to the chronos::PeriodicTimer instance
    // that fired. Need to call method of the PeriodicTimer
    PeriodicTimer *period_timer =
        (PeriodicTimer *)pvTimerGetTimerID(expired_timer);
    xTaskNotify(period_timer->get_task_handle(), TIMER_FIRED_BIT, eSetBits);
}
