/*******************************************************************************
 * @file    state-machine.h
 * @brief   Runs a finite state machine.
 ******************************************************************************/

#pragma once

/** @defgroup state-machine Finite State Machine */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @addtogroup state-machine
 * @brief The finite state machine contains the code that monitors and changes
 * the vehicle's current state.
 *
 * - The system starts out in the initial state on boot. On nominal startup
 * without failure, the system connects to the Wi-Fi router and enables the
 * paused mode.
 * - If an error occurs, the system will enter config mode and launch the web
 *   server so that it can be re-configured or the error investigated.
 * - In paused mode, the system waits for new control input to arrive. As
 *   soon as a reference arrives, it enables the actuators and starts driving.
 *   Even in paused mode, the sensors are continuously sampled and transmitted
 *   to the host.
 * @{
 */

/* Public type declarations ------------------------------------------------- */

/**
 * @brief An enum containing the states the vehicle could be in.
 */
enum SystemState {
    Initial = 0,     ///< Initial state the system starts out with. Not ready to
                     ///< operate and not all components initialized.
    Paused = 1,      ///< System has booted without error. Waits for input.
    Operational = 2, ///< System received input and is currently driving.
    Config = 3,      ///< Failure on boot or manually started configuration.
};

/**
 * @brief An enum containing the possible external inputs to the finite state
 * machine.
 * @note These possible inputs should all be single shifted bits as they are
 * used as bitsets internally.
 */
enum SystemInput {
    StartupCompleted = 0x01, ///< Nominal startup succeeded.
    StartupFailure = 0x02,   ///< Startup failure.
    Configure = 0x04,        ///< Start configuration webserver.
    Operate = 0x08,          ///< Start operation and actuation.
    PauseOperation = 0x10,   ///< No input in a while, disable actuators.
};

/**
 * @brief A struct declaring a possible transition of a system from one
 * state to another.
 *
 * A transition consists of two states and one input: The initial state, an
 * input that can be processed in that state and the next state the system
 * will be in. The handler designates the function that will execute the
 * transition.
 */
struct SystemTransition {
    enum SystemState initial_state; ///< The state the system should be in.
    enum SystemState final_state; ///< The state that the system should move to.
    enum SystemInput input;       ///< The correspondig system input.
    void (*handler)(void);        ///< The handler implementing the transition.
};

/* Public function declaration ---------------------------------------------- */

/**
 * @brief Entry function of the finite state machine task.
 */
void state_machine_task(void *pvParameter);

/**
 * @brief Apply an input to the state machine.
 *
 * @param input
 */
void state_machine_input(enum SystemInput input);

/** @} */

#ifdef __cplusplus
}
#endif
