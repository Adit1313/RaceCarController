/*******************************************************************************
 * @file    spi.hpp
 * @brief   Driver for the ESP32 SPI peripheral.
 ******************************************************************************/

#pragma once

#include <cstdint>

/* Public type definitions -------------------------------------------------- */

namespace chronos {

class SPIDevice;
using SPIDeviceHandle = void *;

class SPIMasterBus {

  public:
    enum SPIPort {
        SPI_PORT_1 = 0,
        SPI_PORT_2 = 1,
        SPI_PORT_3 = 2,
    };
    SPIMasterBus(SPIPort port, int mosi_pin, int miso_pin, int clk_pin);
    ~SPIMasterBus();

    bool write(const SPIDevice &target, uint8_t reg_addr, const uint8_t *data,
               uint32_t len) const;
    bool read(const SPIDevice &source, uint8_t reg_addr, uint8_t *data,
              uint32_t len) const;

    SPIDeviceHandle register_device(const SPIDevice &device);
    void deregister_device(const SPIDevice &device);

  private:
    enum SPIPort port_;
    int mosi_pin_;
    int miso_pin_;
    int clk_pin_;
};

class SPIDevice {
  public:
    /**
     * @brief Construct a new SPIBusDevice object.
     *
     * Upon construction of the object, also registers with the SPI bus master
     * and retrieves its device handle.
     *
     * @param master
     * @param cs_pin
     * @param clock_speed
     * @param required_address
     * @note The referenced object of @p master must not go out of scope until
     *       the lifecycle of the SPIDevice has ended (i.e., it goes out of
     *       scope).
     */
    SPIDevice(SPIMasterBus &master, int cs_pin, uint32_t clock_speed,
              bool requires_address);
    ~SPIDevice();

    bool write(uint8_t reg_addr, const uint8_t *data, uint32_t len) const;
    bool read(uint8_t reg_addr, uint8_t *data, uint32_t len) const;

    /**
     * @brief Retrieve the handle that identifies this SPI device.
     * @returns The device's SPIDeviceHandle
     */
    SPIDeviceHandle get_handle() const { return device_handle; };

    /** Retrieve the device's chip select pin. */
    int get_cs_pin() const { return cs_pin_; };

    /** Find if the device needs address/register bits before writing data. */
    bool get_requires_address() const { return requires_address_; };

    /** Get the clock speed of the device. */
    int get_clock_speed() const { return clock_speed_; };

  private:
    /**
     * @brief Handle to the bus master
     *
     * This handle is set during construction and used when the write() and
     * read() methods are invoked.
     */
    SPIMasterBus &master_;
    int cs_pin_;
    uint32_t clock_speed_;
    bool requires_address_;

    /** Abstract handle from the master driver that identifies this device. */
    SPIDeviceHandle device_handle;
};

}; // namespace chronos
