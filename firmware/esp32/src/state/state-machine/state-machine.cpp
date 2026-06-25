/*******************************************************************************
 * @file    state-machine.c
 * @brief   Runs a finite state machine.
 ******************************************************************************/

#include "state-machine.h"

#include <stdint.h>

#include "config-server.h"
#include "control.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rtos.hpp"

extern "C" {
#include "wifi.h"
}

#define NUM_TRANSITIONS 5

/**
 * @addtogroup state-machine
 * @{
 */

/* Private function declaration --------------------------------------------- */

/** Handler for the transition from the initial state to the paused state. */
static void launch_control_loop(void);

/** Handler for the transition from paused mode to the configuration mode. */
static void start_configuration(void);

/** Handler for the transition from paused mode to the operation mode. */
static void resume_operation(void);

/** Handler for the transition from operation mode to paused mode. */
static void pause_operation(void);

/* Local variables ---------------------------------------------------------- */

static const char *TAG = "state-machine";

/** Includes all allowed transitions of the finite state machine. If it isn't in
  the array, it isn't allowed and should lead to the FSM being put in the
  "stuck" state. */
static const struct SystemTransition allowed_transitions[NUM_TRANSITIONS] = {
    {Initial, Paused, StartupCompleted, &launch_control_loop},
    {Paused, Operational, Operate, &resume_operation},
    {Operational, Paused, PauseOperation, &pause_operation},
    {Paused, Config, Configure, &start_configuration},
    {Initial, Config, StartupFailure, &start_configuration},
};

/** Handle used by state_machine_input() to send task notifications. */
static TaskHandle_t state_machine_task_handle = NULL;

/* Public function implementation ------------------------------------------- */

void state_machine_task(void *pvParameter) {
    uint32_t notification_value = 0;
    enum SystemState current_state = Initial;
    state_machine_task_handle = xTaskGetCurrentTaskHandle();

    ESP_LOGV(TAG, "Entered task.");

    while (true) {
        // Wait indefinitely on a notificiation (= input) being received
        notification_value = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (notification_value != 0) {

            // Find whether there is a valid transition for this input. If there
            // is, apply it and switch to the next state. Call the handler
            // responsible for performing all the transition actions.
            for (uint8_t i = 0; i < NUM_TRANSITIONS; i++) {
                if (allowed_transitions[i].initial_state == current_state) {
                    if ((uint32_t)allowed_transitions[i].input &
                        notification_value) {
                        // Match!
                        struct SystemTransition active_transition =
                            allowed_transitions[i];

                        ESP_LOGI(TAG, "(%u) -- %u --> (%u)",
                                 allowed_transitions[i].initial_state,
                                 allowed_transitions[i].input,
                                 allowed_transitions[i].final_state);

                        if (active_transition.handler != NULL) {
                            active_transition.handler();
                        }

                        current_state = active_transition.final_state;
                    }
                }
            }

        } else {
            ESP_LOGE(TAG, "Received null input.");
        }
    }
}

void state_machine_input(enum SystemInput input) {
    if (state_machine_task_handle) {
        xTaskNotify(state_machine_task_handle, (uint32_t)input, eSetBits);
    }
}

/* Private function implementation ------------------------------------------ */

static void launch_control_loop(void) { control_loop_start(); }

static void start_configuration(void) {
    control_loop_stop();
    wifi_stop();
    wifi_create_ap();
    config_server_start();
}

static void resume_operation(void) { control_loop_resume(); }
static void pause_operation(void) { control_loop_pause(); }

/** @} */
