/*******************************************************************************
 * @file    sensor_interface.hpp
 * @brief   Describes abstract sensor interfaces.
 ******************************************************************************/

#pragma once

namespace chronos {

namespace interface {

/**
 * @brief Template class for a sensor that obtains a measurement of T.
 * @tparam T The sensor's measurement type.
 */
template <typename T> class Sensor {
  public:
    /**
     * @brief Obtain a sample from the sensor.
     * @param[out] sample The sample data.
     * @returns true if the sampling was successful, false otherwise
     */
    virtual bool sample(T &sample) = 0;
};

}; // namespace interface

}; // namespace chronos
