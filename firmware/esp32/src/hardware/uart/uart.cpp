/*******************************************************************************
 * @file    uart.cpp
 * @brief   Wrapper for using the ESP32 UART peripheral in an asynchronous mode.
 ******************************************************************************/

#include "uart.hpp"

#include "driver/uart.h"
#include "esp_log.h"
#include "soc/uart_reg.h"

namespace chronos {

namespace peripheral {

/** Tag used in logging. */
static const char *TAG = "async_uart";

UARTPeripheral::UARTPeripheral(int uart_num, int baudrate, int rx_buf_sz,
                               int tx_buf_sz, int rx_io_num, int tx_io_num)
    : uart_num_(uart_num), baudrate_(baudrate), rx_buf_sz_(rx_buf_sz),
      tx_buf_sz_(tx_buf_sz), rx_io_num_(rx_io_num), tx_io_num_(tx_io_num) {

    // Reset if for some reason the port was already configured (e.g., logging).
    uart_driver_delete(uart_num_);

    uart_config_t cfg = {
        .baud_rate = baudrate_,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(uart_num_, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_io_num_, rx_io_num_,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install UART driver, without using event queue. Uses malloc() internally,
    // can't be avoided. Therefore, the destructor of this peripheral is used to
    // uninstall the driver explicitly on deconstruction.
    ESP_ERROR_CHECK(
        uart_driver_install(uart_num_, rx_buf_sz_, tx_buf_sz_, 0, NULL, 0));
};

void UARTPeripheral::send(const uint8_t *buf, size_t buf_len) {
    if (!buf) {
        return;
    }

    // Todo: actually return the number of sent characters and handle that case
    // correctly
    uart_tx_chars(uart_num_, (const char *)buf, buf_len);
};

void UARTPeripheral::wait_tx_done() {
    uart_wait_tx_done(uart_num_, portMAX_DELAY);
}

size_t UARTPeripheral::available() {
    size_t len = 0;
    esp_err_t ret = ESP_OK;

    if ((ret = uart_get_buffered_data_len(uart_num_, &len)) != ESP_OK) {
        ESP_LOGE(TAG, "Could not get Rx buffer data length (%u)", ret);
        len = 0;
    }

    return len;
};

size_t UARTPeripheral::receive(uint8_t *buf, size_t buf_len, size_t requested) {
    if (!buf)
        return 0;

    int read_bytes = uart_read_bytes(uart_num_, buf, buf_len, 0);

    if (read_bytes < 0) {
        ESP_LOGE(TAG, "Error reading from UART buffer (%d)", read_bytes);
        return 0;
    }

    return read_bytes;
};

void UARTPeripheral::clear_rx_buffer() {
    ; // do nothing currently
};

bool UARTPeripheral::set_baudrate(int baudrate) {
    if (uart_set_baudrate(uart_num_, baudrate) == ESP_OK) {
        baudrate_ = baudrate;
        return true;
    }
    return false;
}

UARTPeripheral::~UARTPeripheral() { uart_driver_delete(uart_num_); }

}; // namespace peripheral

}; // namespace chronos
