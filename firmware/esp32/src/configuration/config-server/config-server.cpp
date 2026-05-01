/*******************************************************************************
 * @file    config-server.cpp
 * @brief   A HTTP server that can be used to configure the car.
 ******************************************************************************/

#include "config-server.h"

#include <algorithm>
#include <ctype.h>
#include <stdlib.h>

#include "esp_http_server.h"
#include "esp_log.h"
#include "main.hpp"
#include "steer-identification.h"

extern "C" {
#include "nvs-manager.h"
}

/* Externally defined git version strings ----------------------------------- */

extern const char *GIT_TAG;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Handler for HTTP GET requests.
 * @param req The HTTP request to serve
 * @returns esp_err_t ESP_OK if the request could be processed, error otherwise.
 */
static esp_err_t config_server_get_handler(httpd_req_t *req);

/**
 * @brief Handler for HTTP POST requests.
 * @param req The HTTP request to serve
 * @return esp_err_t ESP_OK if the request could be processed, error otherwise.
 */
static esp_err_t config_server_post_handler(httpd_req_t *req);

/**
 * @brief Handler for serving a raw text file containing the current
 * configuration values.
 *
 * The current configuration values are filled into the form to make it more
 * obvious how the car is currently configured.
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_Ok if the request could be processed, error otherwise
 */
static esp_err_t config_server_get_config_handler(httpd_req_t *req);

/**
 * @brief Handler for invoking the self-trim of the steering.
 *
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_Ok if the request could be processed, error otherwise
 */
static esp_err_t config_server_put_handler(httpd_req_t *req);

/**
 * @brief Handler for invoking the manual steer action handler.
 *
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_OK if the request could be processed, error otherwise
 */
static esp_err_t manual_steer_handler(httpd_req_t *req);

/**
 * @brief Handler for returning the current steering position.
 *
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_OK if the request could be processed, error otherwise
 */
static esp_err_t steer_pos_handler(httpd_req_t *req);

/**
 * @brief Handler for returning the current wheel encoder values.
 * @param req The HTTP request to serve.
 * @return esp_err_t ESP_OK if the request could be processed, error otherwise
 */
static esp_err_t wheel_encoders_handler(httpd_req_t *req);

/**
 * @brief Decodes a percentage-encoded string into 'normal' C
 * @param dst The destination where the string's decoded contents should be
 * written
 * @param src The source string, nul-terminated
 * @copyright Source: https://stackoverflow.com/a/14530993
 */
void url_decode(char *dst, const char *src);

/**
 * @brief Updates the global configuration with a new value.
 * @param key Pointer to a string that contains the key for the config value to
 * be updated.
 * @param value The new value to for the attribute name in @p key.
 * @returns true if the config was updated and needs committing, false
 * otherwise. If no update is necessary because @p value is empty, returns
 * false.
 */
static bool config_server_update_config(const char *key, const char *value);

/* Local variables ---------------------------------------------------------- */

// The HTML file that is served to any client connecting on /config is embedded
// into the flash memory on compile time and can be extracted.
// The following two pointers point to the beginning and end of the flash
// section where the file is stored.

/** Pointer to start location of embedded html file. */
extern const uint8_t
    configserver_start[] asm("_binary_configserver_html_start");
/** Pointer to end location of embedded html file. */
extern const uint8_t configserver_end[] asm("_binary_configserver_html_end");

/** URI handler for all GET requests for /config */
static httpd_uri_t uri_get = {.uri = "/config",
                              .method = HTTP_GET,
                              .handler = &config_server_get_handler,
                              .user_ctx = NULL};

/** URI handler for all GET requests for /config/current_config.txt */
static httpd_uri_t uri_cfg_get = {.uri = "/config/current_config.txt",
                                  .method = HTTP_GET,
                                  .handler = &config_server_get_config_handler,
                                  .user_ctx = NULL};

/** URI handler for all POST requests for /config */
static httpd_uri_t uri_post = {.uri = "/config",
                               .method = HTTP_POST,
                               .handler = &config_server_post_handler,
                               .user_ctx = NULL};

static httpd_uri_t uri_put = {.uri = "/config/trim",
                              .method = HTTP_PUT,
                              .handler = &config_server_put_handler,
                              .user_ctx = NULL};

static httpd_uri_t uri_manual_put = {.uri = "/manual",
                                     .method = HTTP_POST,
                                     .handler = &manual_steer_handler,
                                     .user_ctx = NULL};

static httpd_uri_t uri_steer_pos_get = {.uri = "/steer_position",
                                        .method = HTTP_GET,
                                        .handler = &steer_pos_handler,
                                        .user_ctx = NULL

};

static httpd_uri_t uri_wheel_encoders_get = {.uri = "/wheel_encoders",
                                             .method = HTTP_GET,
                                             .handler = &wheel_encoders_handler,
                                             .user_ctx = NULL};

/** Active HTTP server. */
httpd_handle_t http_server = NULL;

/** Tag used in logging. */
static const char *TAG = "config-server";

/* Public function implementation ------------------------------------------- */

bool config_server_start() {
    // Start from default configuration
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    esp_err_t err;
    if ((err = httpd_start(&http_server, &config) == ESP_OK)) {

        err = httpd_register_uri_handler(http_server, &uri_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler (%d)", err);
            return false;
        }

        err = httpd_register_uri_handler(http_server, &uri_post);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add POST handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_cfg_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_put);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add PUT handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_manual_put);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add PUT handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_steer_pos_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler  (%d)", err);
        }

        err = httpd_register_uri_handler(http_server, &uri_wheel_encoders_get);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add GET handler  (%d)", err);
        }

    } else {
        ESP_LOGE(TAG, "Failed to start! (%d).", err);
        return false;
    }

    return true;
}

/* Private function implementation ------------------------------------------ */

static esp_err_t config_server_get_handler(httpd_req_t *req) {
    // Currently sends a hard-coded response.
    httpd_resp_send(req, (const char *)configserver_start,
                    configserver_end - configserver_start);
    return ESP_OK;
}

static esp_err_t config_server_post_handler(httpd_req_t *req) {
    // Buffer for received data (could also be binary). Assume it is a string.
    // Actual length of data will be returned by req_recv(), and then
    // zero-termination is ensured. If it is binary, the things below will just
    // parse garbage, but that's okay.
    char query_string[1000];

    /* Truncate if content length larger than the buffer */
    size_t recv_size = (req->content_len < sizeof(query_string))
                           ? req->content_len
                           : sizeof(query_string);

    int ret = httpd_req_recv(req, query_string, recv_size);
    if (ret <= 0) {
        // connection closed for some reason
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            // If client times out, we just send timeout notice
            httpd_resp_send_408(req);
        }
        // return ESP_FAIL to close underlying socket
        return ESP_FAIL;
    }

    // Zero-terminate to be nice
    query_string[ret] = '\0';
    ESP_LOGI(TAG, "Rx POST req: '%s'", query_string);

    // Received something like ip=[...]&port=[...]
    // Use ESP-IDF functionality to fetch the value for each defined attribute,
    // decode it (may%20be%20encoded), then pass to
    // config_server_update_config().

    // Attributes that are currently supported in POST requests
    const char *attribute_keys[] = {
        "ip",          "port",         "ssid",
        "pwd",         "adc_lower",    "adc_upper",
        "steer_range", "pid_p",        "pid_i",
        "pid_d",       "pid_tf",       "speed_p",
        "speed_i",     "speed_d",      "bb_torque",
        "bb_deadband", "control_type", "imu_range",
        "gyro_range",  "steer_torque", "wheel_magnet_count",
        "wheel_radius"};

    size_t num_attrs = sizeof(attribute_keys) / sizeof(attribute_keys[0]);

    // Value of the attribute. Limited to 128 bytes including nul-termination.
    char value[128];
    char decoded_value[128]; // holds the value of the attribute after decoding
    bool should_commit_nvs = false; // whether the NVS needs a commit()
    for (uint_fast8_t i = 0; i < num_attrs; i++) {
        esp_err_t ret = httpd_query_key_value(query_string, attribute_keys[i],
                                              value, sizeof(value));
        if (ret == ESP_ERR_NOT_FOUND) {
            continue;
        } else if (ret == ESP_ERR_HTTPD_RESULT_TRUNC) {
            ESP_LOGW(TAG,
                     "HTTP Server buffer is not long enough to hold passed "
                     "argument. Ignoring.");
            continue;
        } else if (ret == ESP_OK) {
            // something found, maybe time to update?
            url_decode(decoded_value, value);
            if (config_server_update_config(attribute_keys[i], decoded_value))
                should_commit_nvs = true;
        }
    }

    if (should_commit_nvs) {
        nvs_store_config();
        const char resp[] =
            "Received and saved data. Make sure to reset the car PCB.";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    } else {
        const char resp[] = "Nothing to update.";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}

static esp_err_t config_server_get_config_handler(httpd_req_t *req) {
    char response[1000];
    snprintf(response, sizeof(response),
             "ip=%s\n"
             "ssid=%s\n"
             "pwd=%s\n"
             "port=%d\n"
             "adc_upper=%d\n"
             "adc_lower=%d\n"
             "steer_range=%d\n"
             "pid_p=%.3f\n"
             "pid_i=%.3f\n"
             "pid_d=%.3f\n"
             "pid_tf=%.3f\n"
             "speed_p=%.3f\n"
             "speed_i=%.3f\n"
             "speed_d=%.3f\n"
             "bb_deadband=%.2f\n"
             "bb_torque=%d\n"
             "choice_control_type=%d\n"
             "choice_imu_range=%d\n"
             "choice_gyro_range=%d\n"
             "steer_torque=%d\n"
             "wheel_magnet_count=%d\n"
             "wheel_radius=%.5f\n"
             "software_version=%s",
             global_config.host_ip, global_config.ssid, global_config.pwd,
             global_config.host_port, global_config.steer_limit_upper,
             global_config.steer_limit_lower, global_config.steer_angle_deg,
             global_config.pid_kp, global_config.pid_ki, global_config.pid_kd,
             global_config.pid_Tf, global_config.pid_vel_kp,
             global_config.pid_vel_ki, global_config.pid_vel_kd,
             global_config.bb_deadband, global_config.bb_torque,
             (global_config.pid_enabled) ? 0 : 1, global_config.accel_fsr,
             0x04 - global_config.gyro_fsr, global_config.steer_id_torque,
             global_config.wheel_magnet_count, global_config.wheel_radius,
             GIT_TAG);
    httpd_resp_send(req, (const char *)response, strlen(response));
    return ESP_OK;
}

static esp_err_t config_server_put_handler(httpd_req_t *req) {
    const char *response = "Ok.";
    httpd_resp_send(req, (const char *)response, strlen(response));

    struct PotentiometerLimits limits {
        0, 0
    };

    if (steer_identification_find_limits(&limits)) {
        global_config.steer_limit_lower = limits.lower;
        global_config.steer_limit_upper = limits.upper;
    }

    nvs_store_config();

    return ESP_OK;
}

static esp_err_t manual_steer_handler(httpd_req_t *req) {
    const char *response = "OK";
    httpd_resp_send(req, (const char *)response, strlen(response));

    char query_string[1000];

    // Truncate if content length larger than the buffer
    size_t recv_size = (req->content_len < sizeof(query_string))
                           ? req->content_len
                           : sizeof(query_string);

    int ret = httpd_req_recv(req, query_string, recv_size);

    // Zero-terminate to be nice
    query_string[ret] = '\0';
    ESP_LOGI(TAG, "Rx POST req: '%s'", query_string);

    // Received something like action=[...]
    // Use ESP-IDF functionality to fetch the value for each defined attribute,
    // decode it (may%20be%20encoded), then pass to
    // config_server_update_config().

    // Value of the attribute. Limited to 128 bytes including nul-termination.
    char value[128];
    char decoded_value[128]; // holds the value of the attribute after decoding
    ret = httpd_query_key_value(query_string, "action", value, sizeof(value));

    if (ret == ESP_OK) {
        // something found, maybe time to update?
        url_decode(decoded_value, value);
        ESP_LOGE(TAG, "Value: %s", decoded_value);

        if (strncmp(decoded_value, "forward", 7) == 0) {
            chronos::throttle_motor().set_power(30.0f);
        } else if (strncmp(decoded_value, "backward", 8) == 0) {
            chronos::throttle_motor().set_power(-30.0f);
        } else if (strncmp(decoded_value, "left", 4) == 0) {
            chronos::steer_motor().set_power(20.0f);
        } else if (strncmp(decoded_value, "right", 5) == 0) {
            chronos::steer_motor().set_power(-20.0f);
        } else {
            chronos::steer_motor().set_power(0.0f);
            chronos::throttle_motor().set_power(0.0f);
        }
        return ESP_OK;

    } else if (ret == ESP_ERR_HTTPD_RESULT_TRUNC) {
        ESP_LOGW(TAG, "HTTP Server buffer is not long enough to hold passed "
                      "argument. Ignoring.");
        return ESP_FAIL;
    } else {
        return ESP_FAIL;
    }
}

static esp_err_t steer_pos_handler(httpd_req_t *req) {
    char response[10];
    ADCMeasurement sample;
    chronos::steer_feedback().sample(sample);
    snprintf(response, 10, "%d", sample);
    httpd_resp_send(req, (const char *)response, strlen(response));

    return ESP_OK;
}

static esp_err_t wheel_encoders_handler(httpd_req_t *req) {
    char response[100];

    WheelSpeed sample;
    chronos::wheel_encoders().sample(sample);

    snprintf(response, 100, "%.3f,%.3f,%.3f,%.3f", sample.front_left_wheel,
             sample.front_right_wheel, sample.back_left_wheel,
             sample.back_right_wheel);
    httpd_resp_send(req, (const char *)response, strlen(response));

    return ESP_OK;
}

// The following function was taken from StackOverflow
// Source: https://stackoverflow.com/a/14530993
void url_decode(char *dst, const char *src) {
    char a, b;
    while (*src) {
        if ((*src == '%') && ((a = src[1]) && (b = src[2])) &&
            (isxdigit(a) && isxdigit(b))) {
            if (a >= 'a')
                a -= 'a' - 'A';
            if (a >= 'A')
                a -= ('A' - 10);
            else
                a -= '0';
            if (b >= 'a')
                b -= 'a' - 'A';
            if (b >= 'A')
                b -= ('A' - 10);
            else
                b -= '0';
            *dst++ = 16 * a + b;
            src += 3;
        } else if (*src == '+') {
            *dst++ = ' ';
            src++;
        } else {
            *dst++ = *src++;
        }
    }
    *dst++ = '\0';
}

static bool config_server_update_config(const char *key, const char *value) {
    // Reject empty fields
    if (!key || !value || value[0] == '\0')
        return false;

    if (strncmp(key, "ip", 2) == 0) {
        // update global IP attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.host_ip, value, sizeof(global_config.host_ip));
        return true;
    } else if (strncmp(key, "port", 4) == 0) {
        // update global port attribute
        // copy value into buffer, zero-termination by strlcpy
        global_config.host_port = atoi(value);
        return true;
    } else if (strncmp(key, "ssid", 4) == 0) {
        // update global ssid attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.ssid, value, sizeof(global_config.ssid));
        return true;
    } else if (strncmp(key, "pwd", 3) == 0) {
        // update glboal pwd attribute
        // copy value into buffer, zero-termination by strlcpy
        strlcpy(global_config.pwd, value, sizeof(global_config.pwd));
        return true;
    } else if (strncmp(key, "adc_lower", 9) == 0) {
        global_config.steer_limit_lower = atoi(value);
        return true;
    } else if (strncmp(key, "adc_upper", 9) == 0) {
        global_config.steer_limit_upper = atoi(value);
        return true;
    } else if (strncmp(key, "steer_range", 12) == 0) {
        global_config.steer_angle_deg = atoi(value);
        return true;
    } else if (strncmp(key, "pid_p", 5) == 0) {
        global_config.pid_kp = atof(value);
        return true;
    } else if (strncmp(key, "pid_i", 5) == 0) {
        global_config.pid_ki = atof(value);
        return true;
    } else if (strncmp(key, "pid_d", 5) == 0) {
        global_config.pid_kd = atof(value);
        return true;
    } else if (strncmp(key, "pid_tf", 6) == 0) {
        global_config.pid_Tf = atof(value);
        return true;
    } else if (strncmp(key, "speed_p", 7) == 0) {
        global_config.pid_vel_kp = atof(value);
        return true;
    } else if (strncmp(key, "speed_i", 7) == 0) {
        global_config.pid_vel_ki = atof(value);
        return true;
    } else if (strncmp(key, "speed_d", 7) == 0) {
        global_config.pid_vel_kd = atof(value);
        return true;
    } else if (strncmp(key, "bb_deadband", 12) == 0) {
        global_config.bb_deadband = atof(value);
        return true;
    } else if (strncmp(key, "bb_torque", 10) == 0) {
        global_config.bb_torque = atoi(value);
        return true;
    } else if (strncmp(key, "control_type", 13) == 0) {
        if (strncmp(value, "pid", 3) == 0) {
            global_config.pid_enabled = true;
        } else {
            global_config.pid_enabled = false;
        }
        return true;
    } else if (strncmp(key, "imu_range", 9) == 0) {
        // Find which g range it belongs to
        if (strncmp(value, "3g", 2) == 0)
            global_config.accel_fsr = 0x00;
        else if (strncmp(value, "6g", 2) == 0)
            global_config.accel_fsr = 0x01;
        else if (strncmp(value, "12g", 3) == 0)
            global_config.accel_fsr = 0x02;
        else if ((strncmp(value, "24g", 3) == 0))
            global_config.accel_fsr = 0x03;

        return true;
    } else if (strncmp(key, "gyro_range", 10) == 0) {
        if (strncmp(value, "125dps", 6) == 0)
            global_config.gyro_fsr = 0x04;
        else if (strncmp(value, "250dps", 6) == 0)
            global_config.gyro_fsr = 0x03;
        else if (strncmp(value, "500dps", 6) == 0)
            global_config.gyro_fsr = 0x02;
        else if (strncmp(value, "1000dps", 7) == 0)
            global_config.gyro_fsr = 0x01;
        else if (strncmp(value, "2000dps", 6) == 0)
            global_config.gyro_fsr = 0x00;

        return true;
    } else if (strncmp(key, "steer_torque", 12) == 0) {
        global_config.steer_id_torque = std::clamp(atoi(value), 0, 100);
        return true;
    } else if (strncmp(key, "wheel_magnet_count", 18) == 0) {
        global_config.wheel_magnet_count = std::clamp(atoi(value), 1, 255);
        return true;
    } else if (strncmp(key, "wheel_radius", 12) == 0) {
        global_config.wheel_radius = atof(value);
        return true;
    }
    return false;
}
