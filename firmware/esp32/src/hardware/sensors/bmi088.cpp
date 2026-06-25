/*******************************************************************************
 * @file    bmi088.cpp
 * @brief   Driver for the BMI088 Inertial Measurement Unit
 ******************************************************************************/

#include "bmi088.hpp"

#include <cstring>
#include <map>
#include <rom/ets_sys.h>

#include "esp_log.h"
#include "math-util.h"

/* Private function declaration --------------------------------------------- */

/**
 * @brief Does a busy wait for some microseconds.
 * @param us Number of microseconds to wait.
 * @param intf_ptr pass whatever you want
 */
static void delay_us(uint32_t us, void *intf_ptr);

static int8_t bmi_bus_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                            void *handle);
static int8_t bmi_bus_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                           void *handle);

/* Local variable declaration ----------------------------------------------- */

static const char *TAG = "bmi088-driver";

/** Lookup table for the accelerometer dividers given the current range. */
static const std::map<BMI088AccelFullscaleRange, float> accel_divider{
    {FSR_3G, 10920.0f},
    {FSR_6G, 5460.0f},
    {FSR_12G, 2730.0f},
    {FSR_24G, 1365.0f},
};

/** Lookup table for the gyroscope dividers given the current range. */
static const std::map<BMI088GyroFullscaleRange, float> gyro_divider{
    {FSR_125DPS, 262.144f}, {FSR_250DPS, 131.072f}, {FSR_500DPS, 65.536f},
    {FSR_1000DPS, 32.768f}, {FSR_2000DPS, 16.384f},
};

/* Public function implementation ------------------------------------------- */

namespace chronos {

BMI088::BMI088(const SPIDevice &accel_spi_device,
               const SPIDevice &gyro_spi_device,
               BMI088AccelFullscaleRange accel_fsr,
               BMI088GyroFullscaleRange gyro_fsr)
    : accel_spi_device_(accel_spi_device), gyro_spi_device_(gyro_spi_device),
      accel_fsr_(accel_fsr), gyro_fsr_(gyro_fsr) {
    memset(&dev_, 0, sizeof(struct bmi08_dev));

    // When the callbacks to negotiate the bus communication between the
    // Bosch API and the SPI bus are invoked, this pointer will allow finding
    // the correct SPIDevice to talk to.
    dev_.intf_ptr_accel = (void *)&accel_spi_device_;
    dev_.intf_ptr_gyro = (void *)&gyro_spi_device_;

    dev_.read = &bmi_bus_read;
    dev_.write = &bmi_bus_write;
    dev_.delay_us = &delay_us;

    dev_.variant = BMI088_VARIANT;
    dev_.intf = BMI08_SPI_INTF;

    // Initiate communication
    int8_t rslt;

    if ((rslt = bmi08a_init(&dev_)) != BMI08_OK) {
        ESP_LOGE(TAG, "Failed to initialize accelerometer!");
        return;
    }

    if ((rslt = bmi08g_init(&dev_)) != BMI08_OK) {
        ESP_LOGE(TAG, "Failed to initialize gyroscope!");
    }

    // Soft-reset the accelerometer and gyroscope to a known state.
    // This should clear any error conditions from earlier runs.
    if ((rslt = bmi08a_soft_reset(&dev_) != BMI08_OK)) {
        ESP_LOGE(TAG, "Failed to soft-reset accelerometer!");
        return;
    }

    if ((rslt = bmi08g_soft_reset(&dev_) != BMI08_OK)) {
        ESP_LOGE(TAG, "Failed to soft-reset gyroscope!");
        return;
    }

    // Fetch defaults for power mode and measurement config.
    bmi08a_get_power_mode(&dev_);
    bmi08g_get_power_mode(&dev_);
    bmi08a_get_meas_conf(&dev_);
    bmi08g_get_meas_conf(&dev_);

    // Change power, output data rate and bandwidth to our application
    dev_.accel_cfg.power = BMI08_ACCEL_PM_ACTIVE;
    dev_.accel_cfg.odr = BMI08_ACCEL_ODR_400_HZ;
    dev_.accel_cfg.bw = BMI08_ACCEL_BW_OSR4;

    dev_.gyro_cfg.power = BMI08_GYRO_PM_NORMAL;
    dev_.gyro_cfg.odr = BMI08_GYRO_BW_47_ODR_400_HZ;

    // This is a wonky casting between the Bosch API, which uses
    // 3G … 24G as 0x00 ... 0x03 and 2000 dps ... 125 dps as 0x04 … 0x00
    dev_.accel_cfg.range = (int)accel_fsr_; // (uint8_t)accel_fsr;
    dev_.gyro_cfg.range = (int)gyro_fsr_;   // 4 - (uint8_t)gyro_fsr;
    ESP_LOGD(TAG, "Configuring BMI088 to 0x0%X 0x0%X", (int)accel_fsr_,
             (int)gyro_fsr_);

    bmi08a_set_power_mode(&dev_);
    bmi08g_set_power_mode(&dev_);
    bmi08a_set_meas_conf(&dev_);
    bmi08g_set_meas_conf(&dev_);
}

bool BMI088::sample(InertialMeasurement &sample) {
    int8_t result = BMI08_OK;
    struct bmi08_sensor_data data;

    struct LinearAcceleration accel_meas = {};
    struct AngularVelocity gyro_meas = {};

    // Sample accelerometer
    result = bmi08a_get_data(&data, &dev_);

    // Dividers are according to the datasheet for LSB/g
    if (result == BMI08_OK) {
        accel_meas.x = G2MPSS(data.x / accel_divider.at(accel_fsr_));
        accel_meas.y = G2MPSS(data.y / accel_divider.at(accel_fsr_));
        accel_meas.z = G2MPSS(data.z / accel_divider.at(accel_fsr_));
    } else {
        ESP_LOGE(TAG, "Accelerometer sampling error (%d)", result);
        return false;
    }

    ESP_LOGV(TAG, "Accelerations: a_x = %+.2f, g, a_y = %+.2f g, a_z = %+.2f g",
             accel_meas.x, accel_meas.y, accel_meas.z);

    // Sample gyroscope
    result = bmi08g_get_data(&data, &dev_);
    if (result == BMI08_OK) {
        // Dividers according to the datasheet for LSB/°/s
        gyro_meas.x = DEG2RAD(data.x / gyro_divider.at(gyro_fsr_));
        gyro_meas.y = DEG2RAD(data.y / gyro_divider.at(gyro_fsr_));
        gyro_meas.z = DEG2RAD(data.z / gyro_divider.at(gyro_fsr_));
    } else {
        ESP_LOGE(TAG, "Gyroscope sampling error (%d)", result);
        return false;
    }

    ESP_LOGV(TAG, "Angular velocities: Ω_z = %+.2f deg/s", gyro_meas.z);

    sample.linear_accel = accel_meas;
    sample.angular_vel = gyro_meas;

    return true;
}

}; // namespace chronos

/* Private function implementation ------------------------------------------ */

static void delay_us(uint32_t us, void *intf_ptr) { ets_delay_us(us); }

static int8_t bmi_bus_write(uint8_t reg_addr, const uint8_t *data, uint32_t len,
                            void *handle) {
    const chronos::SPIDevice *spi_device = (const chronos::SPIDevice *)handle;
    return spi_device->write(reg_addr, data, len) ? 0 : -1;
}

static int8_t bmi_bus_read(uint8_t reg_addr, uint8_t *data, uint32_t len,
                           void *handle) {
    const chronos::SPIDevice *spi_device = (const chronos::SPIDevice *)handle;
    return spi_device->read(reg_addr, data, len) ? 0 : -1;
}
