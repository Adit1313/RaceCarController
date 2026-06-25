/*******************************************************************************
 * @file    rtos.cpp
 * @brief   Setup and handling of the RTOS functionality of the vehicle.
 ******************************************************************************/

#include "rtos.hpp"

#include "configuration.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "udp-client.h"
}
#include "control.hpp"
#include "state-machine.h"

using namespace chronos;

/* Static stack allocation for tasks ---------------------------------------- */

// To reduce dynamic memory allocation, the task's stacks are allocated during
// linking. If they don't fit into memory, a build error instead of a runtime
// error will occur.

/** Task Control Block for UDP task. */
StaticTask_t udp_task_buffer;
/** Task Control Block for UDP receival task. */
StaticTask_t udp_recv_task_buffer;
/** Task Control Block for control loop task. */
StaticTask_t control_loop_task_buffer;
/** Task Control Block for state machine task. */
StaticTask_t state_machine_task_buffer;

/* Stack of UDP task. */
StackType_t udp_stack_buffer[CONFIG_TASK_STACK_SIZE_UDP];
/* Stack of UDP receival task. */
StackType_t udp_recv_stack_buffer[CONFIG_TASK_STACK_SIZE_UDP_RECV];
/* Stack of control loop task. */
StackType_t control_loop_stack_buffer[CONFIG_TASK_STACK_SIZE_CONTROL_LOOP];
/** Stack of Finite State Machine task. */
StackType_t state_machine_stack_buffer[CONFIG_TASK_STACK_SIZE_STATE_MACHINE];

/* Local variables ---------------------------------------------------------- */

/** Tag used in logging */
static const char *TAG = "rtos";
/** Handle to UDP client task that handles the UDP socket communication with the
 * CRS host. */
static TaskHandle_t udp_task_handle;
/** Handle to UDP receival task that handles incoming data. */
static TaskHandle_t udp_recv_task_handle;
/** Handle to control loop task, which runs the control algorithm. */
static TaskHandle_t control_loop_task_handle;
/** Handle to the finite state machine task. */
static TaskHandle_t state_machine_task_handle;

static ControlLoop *control_loop_;

/* Local macros ------------------------------------------------------------- */

/** Specifies that a task can run on core 0 only. "PRO CPU". */
#define CORE_0 0
/** Specifies that a task can run on core 1 only. "APP CPU". */
#define CORE_1 1
/** Specifies that a task can run on both cores. */
#define CORE_BOTH tskNO_AFFINITY

/* Local function declarations ---------------------------------------------- */

/**
 * @brief Entry function of the control loop task.
 *
 * Commands the actuator setup and starts listening for control commands from
 * the host computer.
 * @note Never call directly, only invoke using xTaskCreate().
 * @param pvParameters Argument to be passed on task instantiation. Unused.
 */
static void control_loop_task(void *pvParameters);

/* Public function implementation ------------------------------------------- */

void chronos::rtos_launch(ControlLoop *control_loop) {
    control_loop_ = control_loop;

    // First task to get instantiated is the finite state machine task.
    state_machine_task_handle = xTaskCreateStaticPinnedToCore(
        &state_machine_task,                  // entry function of the task
        "state machine",                      // task name
        CONFIG_TASK_STACK_SIZE_STATE_MACHINE, // stack size
        NULL,                                 // arguments to pass
        CONFIG_TASK_PRIORITY_STATE_MACHINE,   // priority
        state_machine_stack_buffer,           // static buffer used as stack
        &state_machine_task_buffer,           // static buffer for TCB
        CORE_BOTH // core the task is bound to (none)
    );

    // Start tasks that deal with the Wi-Fi: UDP data sending.
    udp_task_handle = xTaskCreateStaticPinnedToCore(
        &udp_client_task,           // entry function of the task
        "udp client",               // task name
        CONFIG_TASK_STACK_SIZE_UDP, // stack size
        NULL,                       // arguments to pass
        CONFIG_TASK_PRIORITY_UDP,   // priority
        udp_stack_buffer,           // static buffer used as stack
        &udp_task_buffer,           // static buffer for TCB
        CORE_0                      // core the task is bound to
    );

    // Start the task to receive UDP data from the socket that the task above
    // manages.
    udp_recv_task_handle = xTaskCreateStaticPinnedToCore(
        &udp_client_receive_task,        // entry function of the task
        "udp client recv",               // task name
        CONFIG_TASK_STACK_SIZE_UDP_RECV, // stack size
        NULL,                            // arguments to pass
        CONFIG_TASK_PRIORITY_UDP_RECV,   // priority
        udp_recv_stack_buffer,           // static buffer used as stack
        &udp_recv_task_buffer,           // static buffer for TCB
        CORE_0                           // core the task is bound to
    );

    // Start the control loop. The control loop depends on both sending and
    // receiving tasks being available.
    control_loop_task_handle = xTaskCreateStaticPinnedToCore(
        &control_loop_task,                  // entry function of the task
        "control loop",                      // task name
        CONFIG_TASK_STACK_SIZE_CONTROL_LOOP, // stack size
        NULL,                                // arguments to pass
        CONFIG_TASK_PRIORITY_CONTROL_LOOP,   // priority
        control_loop_stack_buffer,           // static buffer used as stack
        &control_loop_task_buffer,           // static buffer for TCB
        CORE_1                               // core the task is bound to
    );
}

chronos::Tick chronos::rtos_get_tick_count() { return xTaskGetTickCount(); }

void control_loop_start(void) { control_loop_->start(); }
void control_loop_pause(void) { control_loop_->pause_actuator_control(); }
void control_loop_resume(void) { control_loop_->resume_actuator_control(); }
void control_loop_stop(void) { control_loop_->stop(); }

/* Local function implementation -------------------------------------------- */

/**
 * @brief Entry function of the control loop task.
 *
 * Commands the actuator setup and starts listening for control commands from
 * the host computer.
 * @note Never call directly, only invoke using xTaskCreate().
 * @param pvParameters Argument to be passed on task instantiation. Unused.
 */
static void control_loop_task(void *pvParameters) { control_loop_->loop(); };
