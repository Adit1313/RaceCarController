/*******************************************************************************
 * @file    wheel-encoder.hpp
 * @brief   Driver to sample an on-board wheel speed sensor.
 ******************************************************************************/

#pragma once

#include "control-types.h"
#include "sensor_interface.hpp"
#include "spi.hpp"

namespace chronos {

/**
 * @brief Driver to sample an on-board wheel speed sensor.
 *
 * The wheel encoder "sensor" communicates with an SPI device
 * placed on the PCB. The embedded device returns the amount
 * of clock cycles that have passed since the last magnet triggered
 * a Hall effect sensor. Using the clock frequency of the timer and
 * the magnet count, the speed of the wheels can be calculated.
 */
class WheelEncoder : public interface::Sensor<WheelSpeed> {

  public:
    /**
     * @brief Construct a wheel encoder sensor.
     * @param spi_device    SPI device to use for communication.
     * @param magnet_count  Number of magnets on one wheel.
     */
    WheelEncoder(const SPIDevice &spi_device, unsigned int magnet_count)
        : spi_device_(spi_device), magnet_count_(magnet_count){};
    bool sample(WheelSpeed &sample);

  private:
    /**
     * @brief Convert the amount of clock cycles to radians per second.
     * @param ticks Clock cycles passed since last Hall effect sensor trigger.
     */
    inline float tick2rads(uint16_t ticks);

    const SPIDevice &spi_device_;
    unsigned int magnet_count_;
};

}; // namespace chronos
