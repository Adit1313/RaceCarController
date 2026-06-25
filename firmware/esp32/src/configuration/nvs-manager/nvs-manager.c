/*******************************************************************************
 * @file    nvs-manager.c
 * @brief   Initializes and manages the non-volatile storage (NVS).
 ******************************************************************************/

#include "nvs-manager.h"

#include <string.h>

#include "configuration.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* Global variable declaration ---------------------------------------------- */

struct Configuration global_config = {0};

/* Local variables ---------------------------------------------------------- */

/** Handle to the storage where the vehicle config is stored. */
nvs_handle_t vehicle_config_handle = 0;

/* Public function implementation ------------------------------------------- */

void nvs_init(void) {

    // Call ESP-IDF initialization. If it fails, need to erase flash and
    // re-init.
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased. If it fails,
        // abort() the boot, because it is a more serious issue.
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    // Check again if it worked. If it didn't, abort.
    ESP_ERROR_CHECK(err);

    // Open the container used to store the configuration
    err = nvs_open("vehicle-storage", NVS_READWRITE, &vehicle_config_handle);
    ESP_ERROR_CHECK(err);

    return;
}

void nvs_load_config(void) {
    size_t ip_len = sizeof(global_config.host_ip);
    esp_err_t ret = nvs_get_str(vehicle_config_handle, "ip",
                                global_config.host_ip, &ip_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.host_ip, CONFIG_DEFAULT_UDP_HOST_IP,
                sizeof(global_config.host_ip));
        // Zero-terminate in case the value in the config would be an overflow.
        // What will happen is that it just doesn't work, but instead can still
        // be set via the web interface.
        global_config.host_ip[sizeof(global_config.host_ip) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t ssid_len = sizeof(global_config.ssid);
    ret = nvs_get_str(vehicle_config_handle, "ssid", global_config.ssid,
                      &ssid_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.ssid, CONFIG_DEFAULT_WIFI_SSID,
                sizeof(global_config.ssid));
        global_config.ssid[sizeof(global_config.ssid) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    size_t pwd_len = sizeof(global_config.pwd);
    ret =
        nvs_get_str(vehicle_config_handle, "pwd", global_config.pwd, &pwd_len);

    switch (ret) {
    case ESP_OK:
        break;
    case ESP_ERR_NVS_NOT_FOUND:
        strncpy(global_config.pwd, CONFIG_DEFAULT_WIFI_PASSWORD,
                sizeof(global_config.pwd));
        global_config.pwd[sizeof(global_config.pwd) - 1] = '\0';
        break;
    default:
        ESP_ERROR_CHECK(ret);
    }

    ret = nvs_get_u16(vehicle_config_handle, "port", &global_config.host_port);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.host_port = CONFIG_DEFAULT_UDP_PORT;
    }

    ret = nvs_get_u16(vehicle_config_handle, "adcl",
                      &global_config.steer_limit_lower);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_limit_lower = CONFIG_DEFAULT_STEER_ADC_RIGHT_LIMIT;
    }
    ret = nvs_get_u16(vehicle_config_handle, "adch",
                      &global_config.steer_limit_upper);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_limit_upper = CONFIG_DEFAULT_STEER_ADC_LEFT_LIMIT;
    }

    ret = nvs_get_u8(vehicle_config_handle, "stang",
                     &global_config.steer_angle_deg);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_angle_deg = CONFIG_DEFAULT_STEERING_RANGE_DEG;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidp",
                      (uint32_t *)&global_config.pid_kp);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_kp = CONFIG_DEFAULT_CONTROL_STEER_PID_KP;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidi",
                      (uint32_t *)&global_config.pid_ki);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_ki = CONFIG_DEFAULT_CONTROL_STEER_PID_KI;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidd",
                      (uint32_t *)&global_config.pid_kd);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_kd = CONFIG_DEFAULT_CONTROL_STEER_PID_KD;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pidtf",
                      (uint32_t *)&global_config.pid_Tf);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_Tf = CONFIG_DEFAULT_CONTROL_STEER_PID_TF;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pid_vel_kp",
                      (uint32_t *)&global_config.pid_vel_kp);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_vel_kp = CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KP;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pid_vel_ki",
                      (uint32_t *)&global_config.pid_vel_ki);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_vel_ki = CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KI;
    }

    ret = nvs_get_u32(vehicle_config_handle, "pid_vel_kd",
                      (uint32_t *)&global_config.pid_vel_kd);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.pid_vel_kd = CONFIG_DEFAULT_CONTROL_VELOCITY_PID_KD;
    }

    ret = nvs_get_u32(vehicle_config_handle, "bbdb",
                      (uint32_t *)&global_config.bb_deadband);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.bb_deadband = CONFIG_DEFAULT_CONTROL_BANGBANG_DEADBAND;
    }

    ret = nvs_get_u8(vehicle_config_handle, "bbtq", &global_config.bb_torque);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.bb_torque = CONFIG_DEFAULT_CONTROL_BANGBANG_TORQUE;
    }

    ret = nvs_get_u32(vehicle_config_handle, "whrd",
                      (uint32_t *)&global_config.wheel_radius);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.wheel_radius = CONFIG_DEFAULT_WHEEL_RADIUS;
    }

    uint8_t controller;
    ret = nvs_get_u8(vehicle_config_handle, "ctrl", &controller);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {

        // Fall back to default controller
#if CONFIG_DEFAULT_CONTROL_STEER_BANGBANG == 1
        controller = 1;
#elif CONFIG_DEFAULT_CONTROL_STEER_PID == 1
        controller = 2;
#else
        controller = 3;
#endif
    }

    if (controller == 2)
        global_config.pid_enabled = true;
    else {
        global_config.pid_enabled = false;
    }

    uint8_t accel_fsr, gyro_fsr;
    ret = nvs_get_u8(vehicle_config_handle, "gfsr", &gyro_fsr);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        gyro_fsr = 0x03;
    }

    ret = nvs_get_u8(vehicle_config_handle, "afsr", &accel_fsr);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        accel_fsr = 0x00;
    }

    ret = nvs_get_u8(vehicle_config_handle, "strtq",
                     &global_config.steer_id_torque);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        global_config.steer_id_torque = CONFIG_DEFAULT_STEER_ID_TORQUE;
    }

    global_config.gyro_fsr = gyro_fsr;
    global_config.accel_fsr = accel_fsr;

    uint8_t wheel_magnet_count;
    ret = nvs_get_u8(vehicle_config_handle, "whmc", &wheel_magnet_count);
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        wheel_magnet_count = CONFIG_DEFAULT_WHEEL_MAGNET_COUNT;
    }
    global_config.wheel_magnet_count = wheel_magnet_count;
}

void nvs_store_config() {
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "ip", global_config.host_ip));
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "ssid", global_config.ssid));
    ESP_ERROR_CHECK(
        nvs_set_str(vehicle_config_handle, "pwd", global_config.pwd));
    ESP_ERROR_CHECK(
        nvs_set_u16(vehicle_config_handle, "port", global_config.host_port));
    ESP_ERROR_CHECK(nvs_set_u16(vehicle_config_handle, "adcl",
                                global_config.steer_limit_lower));
    ESP_ERROR_CHECK(nvs_set_u16(vehicle_config_handle, "adch",
                                global_config.steer_limit_upper));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "stang",
                               global_config.steer_angle_deg));

    uint32_t pidp;
    uint32_t pidi;
    uint32_t pidd;
    uint32_t pidtf;
    uint32_t bbdb;
    uint8_t bbtq;
    uint8_t afsr;
    uint8_t gfsr;
    uint8_t strtq;
    uint8_t whmc;
    uint32_t whrd;

    uint32_t pid_vel_kp;
    uint32_t pid_vel_ki;
    uint32_t pid_vel_kd;

    // Copy the floats to uint32_t's to store them in NVS
    // A compile-time assert below is used to ensure that the size of a float
    // matches the size of a uint32_t
    memcpy(&pidp, &global_config.pid_kp, 4);
    memcpy(&pidi, &global_config.pid_ki, 4);
    memcpy(&pidd, &global_config.pid_kd, 4);
    memcpy(&pidtf, &global_config.pid_Tf, 4);
    memcpy(&bbdb, &global_config.bb_deadband, 4);
    memcpy(&bbtq, &global_config.bb_torque, 1);
    memcpy(&afsr, &global_config.accel_fsr, 1);
    memcpy(&gfsr, &global_config.gyro_fsr, 1);
    memcpy(&strtq, &global_config.steer_id_torque, 1);
    memcpy(&whmc, &global_config.wheel_magnet_count, 1);
    memcpy(&whrd, &global_config.wheel_radius, 4);

    memcpy(&pid_vel_kp, &global_config.pid_vel_kp, 4);
    memcpy(&pid_vel_ki, &global_config.pid_vel_ki, 4);
    memcpy(&pid_vel_kd, &global_config.pid_vel_kd, 4);

    uint8_t ctrl = (global_config.pid_enabled) ? 2 : 0;

    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidp", pidp));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidi", pidi));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidd", pidd));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "pidtf", pidtf));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "bbdb", bbdb));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "bbtq", bbtq));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "ctrl", ctrl));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "afsr", afsr));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "gfsr", gfsr));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "strtq", strtq));
    ESP_ERROR_CHECK(nvs_set_u8(vehicle_config_handle, "whmc", whmc));
    ESP_ERROR_CHECK(nvs_set_u32(vehicle_config_handle, "whrd", whrd));

    ESP_ERROR_CHECK(
        nvs_set_u32(vehicle_config_handle, "pid_vel_kp", pid_vel_kp));
    ESP_ERROR_CHECK(
        nvs_set_u32(vehicle_config_handle, "pid_vel_ki", pid_vel_ki));
    ESP_ERROR_CHECK(
        nvs_set_u32(vehicle_config_handle, "pid_vel_kd", pid_vel_kd));

    ESP_ERROR_CHECK(nvs_commit(vehicle_config_handle));
}

/* Compile-time asserts ----------------------------------------------------- */

// These asserts are needed because floats are stored as uint32_t's in NVS as
// they're assumed to be the same size. If this assumption is broken, stop
// compilation.
_Static_assert(sizeof(float) == sizeof(uint32_t), "float is not four bytes");
