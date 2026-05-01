/*******************************************************************************
 * @file    wifi.h
 * @brief   Enable Wi-Fi module and manage Wi-Fi mode (station, AP, ...).
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"

/** @defgroup wifi Wi-Fi driver */

/**
 * @addtogroup wifi
 * @brief The Wi-Fi driver enables the Wi-Fi module and manages connections.
 *
 * The Wi-Fi module can run in multiple modes:
 * - Station mode: the ESP32 connects to a wireless access point.
 * - AP mode: the ESP32 is the access point.
 *
 * These two are used for different scenarios: In standard operation, the ESP32
 * connects to an access point (the router in CRS lab by default). The other one
 * is not used currently, but in future applications it might be used to
 * configure system settings.
 *
 * How to use the driver:
 *
 * 1. Call wifi_init() to create all prerequisites for starting the Wi-Fi
 * 2. Choose whether you want the ESP32 to connect to an access point or create
 *    a network.
 *
 * Depending on your choice in (2), do either of the two following:
 * - Call wifi_connect_to_ap() to connect to a wireless access point using an
 *   SSID & password
 * - Call wifi_create_ap() to create a wireless network.
 *
 * It is also possible to switch between the modes. This can be used if
 * connection to the AP fails. First, wifi_stop() should be called. Afterwards,
 * either wifi_connect_to_ap() or wifi_create_ap() can be called to start a
 * fresh configuration.
 * @{
 */

/**
 * @brief Sets up the Wi-Fi driver.
 *
 * This function creates the event loop that is needed for callbacks from the
 * ESP-IDF Wi-Fi driver, then proceeds to initialize the Wi-Fi driver from a
 * default configuration.
 */
void wifi_init(void);

/**
 * @brief Set up the Wi-Fi module and connect to a wireless access point.
 * @note This function will run on the calling task and will block this task
 * until a connection has been established or the connection attempt has failed.
 * @note wifi_init() needs to be called prior to this, otherwise it fails
 * instantly.
 * @param timeout Timeout (in seconds) of trying to connect to the AP.
 * @returns true if the Wi-Fi was successfully set up and connected, else false
 */
bool wifi_connect_to_ap(TickType_t timeout);

/**
 * @brief Sets up an access point that other devices may connect to.
 * @note wifi_init() needs to be called prior to this, otherwise it fails
 * instantly.
 * @return true if the access point was set up, false otherwise.
 */
bool wifi_create_ap(void);

/**
 * @brief Disconnects any Wi-Fi connection and stops the Wi-Fi module.
 * @note If Wi-Fi has not previously been started with wifi_connect_to_ap() or
 * wifi_create_ap(), this does nothing.
 */
void wifi_stop(void);

/** @} */
