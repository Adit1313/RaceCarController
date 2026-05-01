/*******************************************************************************
 * @file    configuration.h
 * @brief   Compile-time configuration of the firmware.
 ******************************************************************************/

#pragma once

/* RTOS Configuration ------------------------------------------------------- */

// Task priorities: these range from 0 (lowest) to  4 (highest)
#define CONFIG_TASK_PRIORITY_UDP              (2)
#define CONFIG_TASK_PRIORITY_UDP_RECV         (2)
#define CONFIG_TASK_PRIORITY_CONTROL_LOOP     (3)
#define CONFIG_TASK_PRIORITY_POWER_MONITORING (1)
#define CONFIG_TASK_PRIORITY_STATE_MACHINE    (4)

// Stack size of the various RTOS tasks in bytes
#define CONFIG_TASK_STACK_SIZE_UDP              (3000)
#define CONFIG_TASK_STACK_SIZE_UDP_RECV         (3000)
#define CONFIG_TASK_STACK_SIZE_CONTROL_LOOP     (4000)
#define CONFIG_TASK_STACK_SIZE_POWER_MONITORING (2000)
#define CONFIG_TASK_STACK_SIZE_STATE_MACHINE    (3000)

// Control Loop Rate in Hz
#define CONFIG_CONTROL_LOOP_RATE (250)

// Timeout for the UDP connection in milliseconds
#define CONFIG_DISCONNECT_THRESHOLD_MS (200)

/* Hardware Configuration --------------------------------------------------- */

/**
 * @brief Disable DC motor coasting if true.
 *
 * If set to true, the DC motor will passively brake when the PWM signal is set
 * to 0. Usually leads to shorter coasting.
 */
#define CONFIG_DISABLE_DC_COASTING (0)

/** Oversampling rate of ADC. */
#define CONFIG_ADC_MULTISAMPLING_BATCH (5)

/* Raw steer voltage controller configuration ------------------------------- */

#define CONFIG_STEER_VOLTAGE_KP (7.5f / 1000.0f)
#define CONFIG_STEER_VOLTAGE_KI (20.0f / 1000.0f)
#define CONFIG_STEER_VOLTAGE_KD (0.2f / 1000.0f)
// Higher => heavier filtering
#define CONFIG_STEER_VOLTAGE_TF (1e-3f)

/* NVS Storage Defaults ----------------------------------------------------- */

// These configurations are default values for the NVS storage. They are used
// when the storage is either empty or corrupted as fallback. The user-defined
// stored values take priority if they exist.

#define CONFIG_DEFAULT_WIFI_SSID              ""
#define CONFIG_DEFAULT_WIFI_PASSWORD          ""
#define CONFIG_DEFAULT_UDP_HOST_IP            "192.168.1.160"
#define CONFIG_DEFAULT_UDP_PORT               (20211)
#define CONFIG_DEFAULT_CONTROL_STEER_BANGBANG (0)
#define CONFIG_DEFAULT_CONTROL_STEER_PID      (1)

// Default controller parameters, also overwritten by the web interface.
#define CONFIG_DEFAULT_CONTROL_STEER_PID_KP (1.25f)
#define CONFIG_DEFAULT_CONTROL_STEER_PID_KI (3.33f)
#define CONFIG_DEFAULT_CONTROL_STEER_PID_KD (0.033f)
#define CONFIG_DEFAULT_CONTROL_STEER_PID_TF (1e-3f)

#define CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KP (1.5f)
#define CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KI (0.0f)
#define CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KD (0.1f)

#define CONFIG_DEFAULT_CONTROL_BANGBANG_DEADBAND (0.01f)
#define CONFIG_DEFAULT_CONTROL_BANGBANG_TORQUE   (55.0f)

#define CONFIG_DEFAULT_CONTROL_THROTTLE_GAIN (1.0f)

#define CONFIG_DEFAULT_STEER_ADC_RIGHT_LIMIT (1100)
#define CONFIG_DEFAULT_STEER_ADC_LEFT_LIMIT  (2300)
#define CONFIG_DEFAULT_STEERING_RANGE_DEG    (20)

#define CONFIG_DEFAULT_WHEEL_RADIUS       (0.0175f)
#define CONFIG_DEFAULT_WHEEL_MAGNET_COUNT (2)

#define CONFIG_DEFAULT_STEER_ID_TORQUE (42)
