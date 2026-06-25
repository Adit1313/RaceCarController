/*******************************************************************************
 * @file    nvs-manager.h
 * @brief   Initializes and manages the non-volatile storage (NVS).
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <sys/types.h>

/** @defgroup nvs-manager Non-volatile Storage Manager */

/**
 * @addtogroup nvs-manager
 * @brief The NVS Manager manages the storage of non-volatile variables.
 *
 * Initialize the NVS by calling nvs_init(). If the function returns,
 * setup was successful.
 * @{
 */

/**
 * @brief Set of configuration values that are stored non-volatile.
 *
 * When the device's non-volatile storage is initialized, the set of
 * configuration values is loaded from storage. They can be changed during
 * runtime and committed back to NVS.
 */
struct Configuration {
    char host_ip[40];           ///< Host IP address in human-readable format.
                                ///< Nul-terminated.
    uint16_t host_port;         ///< UDP port of host computer
    char ssid[33];              ///< SSID
    char pwd[100];              ///< Password
    uint16_t steer_limit_lower; ///< Lower limit of the steering potentiometer.
    uint16_t steer_limit_upper; ///< Upper limit of the steering potentiometer.
    uint8_t steer_angle_deg;    ///< Steering angle in degrees.

    float pid_kp; ///< Steer controller proportional gain
    float pid_ki; ///< Steer controller integral gain
    float pid_kd; ///< Steer controller derivative gain
    float pid_Tf; ///< Steer controller filter time constant

    float pid_vel_kp; ///< PID velocity k_p
    float pid_vel_ki; ///< PID velocity k_i
    float pid_vel_kd; ///< PID velocity k_d

    float bb_deadband; ///< Bang-bang deadbang
    uint8_t bb_torque; ///< Percentage torque of bang-bang controller

    bool pid_enabled; ///< Is the PID controller enabled?

    /** Full scale range of accelerometer: 0x00 (±3g) to 0x03 (±24g) */
    uint8_t accel_fsr;
    /** Full scale range of gyroscope: 0x04 (2000°/s) to 0x00 (125°/s) */
    uint8_t gyro_fsr;

    uint8_t steer_id_torque;    ///< Percentage of torque during steering ID
    uint8_t wheel_magnet_count; ///< Number of magnets on one wheel

    float wheel_radius; ///< Wheel radius in meters
};

/**
 * @brief Global configuration struct.
 */
extern struct Configuration global_config;

/* Public function declaration ---------------------------------------------- */

/**
 * @brief Initializes the non-volatile storage unit of the ESP32.
 *
 * This calls the ESP-IDF setup functionality of the nvs. If this fails, boot
 * is aborted, since it is very likely that a component of the ESP32 is damaged
 * or there is a more serious issue with the flash memory (overwritten by
 * binary?).
 */
void nvs_init(void);

/**
 * @brief Loads the current configuration from NVS.
 */
void nvs_load_config(void);

/**
 * @brief Commit the changes of the current configuration to NVS.
 */
void nvs_store_config(void);

/** @} */
