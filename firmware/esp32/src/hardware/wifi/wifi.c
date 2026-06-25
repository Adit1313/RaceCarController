/*******************************************************************************
 * @file    wifi.c
 * @brief   Enable Wi-Fi module and manage Wi-Fi mode (station, AP, ...).
 ******************************************************************************/

#include "wifi.h"

#include <string.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs-manager.h"
#include "sdkconfig.h"

/** @addtogroup wifi
 *  @{
 */

/* Local macros ------------------------------------------------------------- */

// The following are event bits set by this driver to unlock the calling task in
// case a connection was successful or failed entirely.

/** Event bit set when successfully connected */
#define WIFI_CONNECTED (1 << 0)
/** Event bit set when connection failed after given amount of retries */
#define WIFI_CONNECTION_FAILED (1 << 1)

/* Local variables ---------------------------------------------------------- */

/** Tag used in printing logging */
static const char *TAG = "wifi";

/** Event group that is used by the this driver to signal when certain
 * events (e.g., Wi-Fi connection succeeded) happened. */
static EventGroupHandle_t wifi_event_group = NULL;

/** Statically allocated memory for wifi_event_group. */
static StaticEventGroup_t wifi_event_group_buffer;

/** Stores the currently active network interface. If set, the network stack has
 * been set up. */
static esp_netif_t *active_net_if = NULL;

/**
 * @brief Whether the driver should try to re-connect if the connection breaks.
 *
 * The driver should try to reconnect if the connection is broken from elsewhere
 * but obviously not when the Wi-Fi connection is shut down from the ESP32 side.
 * Also, this can be set if the connection times out and the Wi-Fi driver should
 * be stopped from running in the background.
 */
static bool should_reconnect = false;

/** Event handle for any event. */
static esp_event_handler_instance_t instance_any_id = NULL;
/** Event handler for when an IP is received when connected to a station. */
static esp_event_handler_instance_t instance_got_ip = NULL;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Callback handler for events happening on the Wi-Fi.
 */
static void wifi_event_callback(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data);

/* Public function implementation ------------------------------------------- */

void wifi_init() {
    // The event group created here is used by this driver for callbacks. Never
    // fails because statically allocated.
    wifi_event_group = xEventGroupCreateStatic(&wifi_event_group_buffer);

    // The event loop is used by the driver internally to signal changes in
    // Wi-Fi (e.g., connected to the network.)
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Start from valid default config, then enable NVS flash to store
    // credentials.
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = 1;

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

bool wifi_connect_to_ap(TickType_t timeout) {

    if (!wifi_event_group) {
        // Setup not correctly called :)
        ESP_LOGW(TAG, "wifi_event_group not created. Probably forgot to call "
                      "wifi_init()?");
        return false;
    }

    if (active_net_if) {
        // There already exists a setup, abort.
        ESP_LOGW(TAG, "Existing network interface present. Call wifi_stop() "
                      "before starting in another mode.");
        return false;
    }

    active_net_if = esp_netif_create_default_wifi_sta();

    // If this point is reached, the Wi-Fi module is successfully enabled.
    // Disable sleep to have minimum latency.
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_FLASH));

    // Register for events so that we get a callback if an IP is received or
    // something
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_callback, NULL,
        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_callback, NULL,
        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid = "", // these are assigned by copying below
                .password = "",
                .threshold.authmode = WIFI_AUTH_WPA2_PSK,

                .pmf_cfg = {.capable = true, .required = false},
            },
    };
    strcpy((char *)wifi_config.sta.ssid, global_config.ssid);
    strcpy((char *)wifi_config.sta.password, global_config.pwd);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    // Make the driver try re-connecting until time runs out.
    should_reconnect = true;

    esp_err_t ret;
    if ((ret = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi could not be started. Reason: (%i)", ret);
        wifi_stop();
        return false;
    }

    // The driver will now try to connect. All further events are handled
    // directly in the wifi_event_callback function. If this times out, then
    // it's time to return to the calling function.
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group, WIFI_CONNECTED | WIFI_CONNECTION_FAILED, pdFALSE,
        pdFALSE, timeout);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we
     * can test which event actually happened. */
    if (bits & WIFI_CONNECTED) {
        ESP_LOGI(TAG, "Connected to '%s'.", global_config.ssid);
    } else {
        // Timeout, set should_reconnect() to false so the driver doesn't try
        // re-connecting
        ESP_LOGI(TAG, "Failed to connect to '%s'", global_config.ssid);
        ESP_LOGE(TAG, "Timed out while connecting to AP. Stopping Wi-Fi.");

        // Safely stop Wi-Fi
        wifi_stop();

        return false;
    }

    return true;
}

bool wifi_create_ap(void) {

    if (!wifi_event_group) {
        // Setup not correctly called :)
        ESP_LOGW(TAG, "wifi_event_group not created. Probably forgot to call "
                      "wifi_init()?");
        return false;
    }

    if (active_net_if) {
        // There already exists a setup, abort.
        ESP_LOGW(TAG, "Existing network interface present. Call wifi_stop() "
                      "before starting in another mode.");
        return false;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_callback, NULL,
        &instance_any_id));

    active_net_if = esp_netif_create_default_wifi_ap();
    esp_wifi_set_storage(WIFI_STORAGE_RAM);

    wifi_config_t cfg = {
        .ap =
            {
                .ssid = "crs-car",
                .ssid_len = 7,
                .channel = 6,
                .max_connection = 2,
                .authmode = WIFI_AUTH_OPEN,
            },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));

    esp_err_t ret;
    if ((ret = esp_wifi_start()) != ESP_OK) {
        ESP_LOGE(TAG, "WiFi could not be started. Reason: (%i)", ret);
        wifi_stop();
        return false;
    }

    return true;
}

void wifi_stop(void) {

    // Check if Wi-Fi is enabled, and which mode it is in.
    wifi_mode_t mode = WIFI_MODE_NULL;
    esp_err_t ret = esp_wifi_get_mode(&mode);

    if (ret != ESP_ERR_WIFI_NOT_INIT) {
        // Safe to call other Wi-Fi methods – the driver was initialized
        should_reconnect = false;
        esp_wifi_disconnect();
        esp_wifi_stop();
    }

    // Is a Wi-Fi connection even active? Otherwise return.
    if (active_net_if != NULL) {
        esp_netif_destroy(active_net_if);
        active_net_if = NULL;
    }

    // Unregister from callbacks, since we're not interested in trying
    // further reconnects.
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(
        WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
}

/* Private function implementation ------------------------------------------ */

static void wifi_event_callback(void *arg, esp_event_base_t event_base,
                                int32_t event_id, void *event_data) {

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        // In this scenario, the driver is trying to connect to a station.
        // The only thing we need to do is connect.
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {

        // This may be called if the connection upon esp_wifi_connect() failed
        // or if esp_wifi_disconnect() was called.
        // Todo: Check if esp_wifi_disconnect() was called. If it was called,
        // obviously don't try to reconnect.
        if (should_reconnect) {
            ESP_LOGI(TAG, "retry to connect to the AP");
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG, "connect to the AP fail");
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTION_FAILED);
        }

    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));

        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED);
    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_AP_STACONNECTED) {
        // A client connected!
    }
}

/** @} */
