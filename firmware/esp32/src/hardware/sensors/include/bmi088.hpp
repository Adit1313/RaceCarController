/*******************************************************************************
 * @file    bmi088.hpp
 * @brief   Driver for the BMI088 Inertial Measurement Unit
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "bmi08x.h"
#include "control-types.h"
#include "sensor_interface.hpp"
#include "spi.hpp"

/**
 * An acceleration measurement from the built-in IMU has 16-bit resolution in
 * each direction. Depending on the full-scale range, accelerations up to ±24g
 * can be measured.
 *
 * The entries for the three dimensions have the unit LSB/g – to calculate the
 * acceleration in units of g, it is necessary to divide by the following
 * divisors:
 *
 * - ± 3g: 10920 LSB/g
 * - ± 6g:  5460 LSB/g
 * - ±12g:  2730 LSB/g
 * - ±24g:  1365 LSB/g
 */
enum BMI088AccelFullscaleRange {
    FSR_3G = 0x00,
    FSR_6G = 0x01,
    FSR_12G = 0x02,
    FSR_24G = 0x03,
};

/**
 * @brief Data type for angular velocity measurements from an IMU.
 * An angular velocity measurement from the built-in IMU has 16-bit resolution
 * in each direction. Depending on the full-scale range, angular velocities up
 * to ±2000 deg/s can be measured.
 *
 * The entries for the three dimensions have the unit LSB/g – to calculate the
 * angular velocities in units of deg/s, it is necessary to divide by the
 * following divisors:
 *
 * - ± 125 deg/s: 262.144 LSB*s/deg
 * - ± 250 deg/s: 131.072 LSB*s/deg
 * - ± 500 deg/s:  65.536 LSB*s/deg
 * - ±1000 deg/s:  32.768 LSB*s/deg
 * - ±2000 deg/s:  16.384 LSB*s/deg
 *
 */
enum BMI088GyroFullscaleRange {
    FSR_125DPS = 0x04,
    FSR_250DPS = 0x03,
    FSR_500DPS = 0x02,
    FSR_1000DPS = 0x01,
    FSR_2000DPS = 0x00
};

typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t *, uint32_t, void *);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t *, uint32_t,
                                         void *);

namespace chronos {

/**
 * @brief A BMI088 inertial measurement unit with an accelerometer and a
 * gyroscope.
 *
 * The BMI088 driver needs two function pointers for bus transactions: one for
 * writing data to a register over SPI and one to read data. In addition, the
 * intf_ptr field can be set before calling bmi088_setup(). The value of this
 * parameter will be passed to the bus read function as well.
 *
 * The BMI088 is set up in the following way:
 * - The Bosch SensorTec API is initialized.
 * - Communication is initialized and the chip identifiers are read and compared
 *   to the expected values.
 * - A soft-reset is issued to revert the chip to its default configuration.
 * - Desired power settings and sampling rates are applied.
 */
class BMI088 : public interface::Sensor<InertialMeasurement> {
  public:
    /**
     * @brief Establish communication with a new BMI088 and configure it.
     *
     * The constructor will automatically try to set up the BMI088 via the SPI
     * bus. It the setup fails, the constructor will silently return and print
     * an error to the ESP log.
     *
     * @param accel_spi_device The SPI device to use for the accelerometer.
     * @param gyro_spi_device  The SPI device to use for the gyroscope.
     * @param accel_fsr        Desired full-scale range for the accelerometer.
     * @param gyro_fsr         Desired full-scale range for the gyroscope.
     */
    BMI088(const SPIDevice &accel_spi_device, const SPIDevice &gyro_spi_device,
           BMI088AccelFullscaleRange accel_fsr = FSR_6G,
           BMI088GyroFullscaleRange gyro_fsr = FSR_1000DPS);

    /**
     * @brief Sample the BMI088 IMU.
     * @param sample The measurement obtained from the BMI088
     * @returns true if the sampling was successful, false otherwise
     */
    virtual bool sample(InertialMeasurement &sample);

  private:
    /** Handle to an SPI bus device driver. */
    const SPIDevice &accel_spi_device_;
    const SPIDevice &gyro_spi_device_;

    BMI088AccelFullscaleRange accel_fsr_;
    BMI088GyroFullscaleRange gyro_fsr_;

    /** Lower-level driver configuration struct. */
    struct bmi08_dev dev_;
};

}; // namespace chronos
