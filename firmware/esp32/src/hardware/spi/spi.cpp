/*******************************************************************************
 * @file    spi.cpp
 * @brief   Driver for the ESP32 SPI peripheral.
 ******************************************************************************/

#include "spi.hpp"

#include <cstring>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"

/* Private function declaration --------------------------------------------- */

/**
 * @brief Callback for pre-transaction actions on the SPI driver.
 *
 * This function is needed to fulfill the timing requirements of the BMI088
 * sensor: between the CS falling edge and the first CLK edge, there have to be
 * at least 40ns (the ESP32 does them at the same time). This function manually
 * puts CS low and then delays for >= 40ns before returning.
 * @note The pin that will be pulled low is set by the @link{spi_bus_write} or
 *  @link{spi_bus_read} function that is called by the BMI088 driver. This is
 *  NOT thread-safe, so the SPI interactions with the BMI088 can only come from
 *  a single thread! The pin to be pulled low is stored in @link{active_cs_pin}.
 * @param trans parameter from the SPI driver with metadata of the transaction
 */
void IRAM_ATTR pull_cs_low(spi_transaction_t *trans);

/**
 * @brief Counterpart callback for after SPI transaction has happened.
 *
 * Pulls CS back up after a transaction on the bus.
 * @note The pin that will be pulled low is set by the @link{spi_bus_write} or
 *  @link{spi_bus_read} function that is called by the BMI088 driver. This is
 *  NOT thread-safe, so the SPI interactions with the BMI088 can only come from
 *  a single thread! The pin to be pulled low is stored in @link{active_cs_pin}.
 * @param trans parameter from the SPI driver with metadata of the transaction
 */
void IRAM_ATTR pull_cs_high(spi_transaction_t *trans);

/* Public type implementation ----------------------------------------------- */

namespace chronos {

SPIMasterBus::SPIMasterBus(SPIPort port, int mosi_pin, int miso_pin,
                           int clk_pin)
    : port_(port), mosi_pin_(mosi_pin), miso_pin_(miso_pin), clk_pin_(clk_pin) {

    spi_bus_config_t bus_config;
    memset(&bus_config, 0, sizeof(spi_bus_config_t));
    bus_config.mosi_io_num = (gpio_num_t)mosi_pin_;
    bus_config.miso_io_num = (gpio_num_t)miso_pin_;
    bus_config.sclk_io_num = (gpio_num_t)clk_pin_;
    bus_config.quadwp_io_num = -1; // don't use write protect or
    bus_config.quadhd_io_num = -1; // hold signals

    // Initialize on SPI2 host, without DMA (set dma_chan to 0)
    ESP_ERROR_CHECK(
        spi_bus_initialize((spi_host_device_t)port_, &bus_config, 0));
}

bool SPIMasterBus::write(const SPIDevice &target, uint8_t reg_addr,
                         const uint8_t *data, uint32_t len) const {
    // Set up new transaction, chip select pin to be pulled low is passed as the
    // user field and then read in the .pre and .post callbacks
    spi_transaction_t trans = {
        .flags = 0,
        .cmd = 0,
        .addr = reg_addr,
        .length = len * 8,
        .rxlength = 0,
        .user = (void *)(target.get_cs_pin()),
        .tx_buffer = data,
        .rx_buffer = nullptr,
    };

    return spi_device_transmit((spi_device_handle_t)target.get_handle(),
                               &trans) == ESP_OK;
}

bool SPIMasterBus::read(const SPIDevice &source, uint8_t reg_addr,
                        uint8_t *data, uint32_t len) const {
    // Set up new transaction, chip select pin to be pulled low is passed as the
    // user field and then read in the .pre and .post callbacks
    spi_transaction_t trans = {
        .flags = 0,
        .cmd = 0,
        .addr = reg_addr,
        .length = len * 8,
        .rxlength = len * 8,
        .user = (void *)source.get_cs_pin(),
        .tx_buffer = nullptr,
        .rx_buffer = data,
    };

    return spi_device_transmit((spi_device_handle_t)source.get_handle(),
                               &trans) == ESP_OK;
}

SPIDeviceHandle SPIMasterBus::register_device(const SPIDevice &device) {

    spi_device_interface_config_t dev_config;
    memset(&dev_config, 0, sizeof(spi_device_interface_config_t));

    dev_config.address_bits = device.get_requires_address() ? 8u : 0u;
    dev_config.mode = 0;
    dev_config.spics_io_num = -1; // don't use hardware CS from SPI driver
    dev_config.clock_speed_hz = device.get_clock_speed();
    dev_config.queue_size = 1;
    dev_config.pre_cb = &pull_cs_low; // use own CS drivers
    dev_config.post_cb = &pull_cs_high;

    // Here the device gets added and a handle is returned, which is then in
    // turn also returned to the user of this driver
    spi_device_handle_t handle;
    ESP_ERROR_CHECK(
        spi_bus_add_device((spi_host_device_t)port_, &dev_config, &handle));

    // Initialize CS pin
    gpio_num_t cs = (gpio_num_t)device.get_cs_pin();
    gpio_reset_pin(cs);
    gpio_set_direction(cs, GPIO_MODE_OUTPUT);
    gpio_set_level(cs, 1);

    return (SPIDeviceHandle)handle;
}

void SPIMasterBus::deregister_device(const SPIDevice &device) {
    spi_bus_remove_device((spi_device_handle_t)device.get_handle());
}

SPIMasterBus::~SPIMasterBus() { spi_bus_free((spi_host_device_t)port_); }

SPIDevice::SPIDevice(SPIMasterBus &master, int cs_pin, uint32_t clock_speed,
                     bool requires_address)
    : master_(master), cs_pin_(cs_pin), clock_speed_(clock_speed),
      requires_address_(requires_address) {

    device_handle = master_.register_device(*this);
}

SPIDevice::~SPIDevice() { master_.deregister_device(*this); }

bool SPIDevice::write(uint8_t reg_addr, const uint8_t *data,
                      uint32_t len) const {
    return master_.write(*this, reg_addr, data, len);
}

bool SPIDevice::read(uint8_t reg_addr, uint8_t *data, uint32_t len) const {
    return master_.read(*this, reg_addr, data, len);
}
}; // namespace chronos

/* Private function implementation ------------------------------------------ */

void IRAM_ATTR pull_cs_low(spi_transaction_t *trans) {
    intptr_t cs = reinterpret_cast<intptr_t>(trans->user);
    if (cs != -1) {
        gpio_set_level((gpio_num_t)cs, 0);
        // BMI088 requires >= 40ns before clock starts
        // TODO: reduce this to 40ns, currently 1µs
        esp_rom_delay_us(1);
    }
}

void IRAM_ATTR pull_cs_high(spi_transaction_t *trans) {
    intptr_t cs = reinterpret_cast<intptr_t>(trans->user);
    if (cs != -1) {
        gpio_set_level((gpio_num_t)cs, 1);
    }
};
