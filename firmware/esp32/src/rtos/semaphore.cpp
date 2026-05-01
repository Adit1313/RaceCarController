/*******************************************************************************
 * @file    semaphore.cpp
 * @brief   Abstraction for an RTOS binary semaphore.
 ******************************************************************************/

#include "semaphore.hpp"

using namespace chronos::rtos;

Semaphore::Semaphore() {
    semaphore_handle_ = xSemaphoreCreateMutexStatic(&static_buffer_);
}

bool Semaphore::acquire(uint32_t timeout_ms) {
    return xSemaphoreTake(semaphore_handle_, pdMS_TO_TICKS(timeout_ms)) ==
           pdTRUE;
}

void Semaphore::release() { xSemaphoreGive(semaphore_handle_); }

Semaphore::~Semaphore() {
    // Maybe the underlying RTOS needs to clean up resources
    vSemaphoreDelete(semaphore_handle_);
}
