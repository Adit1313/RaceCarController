/*******************************************************************************
 * @file    uart.hpp
 * @brief   Wrapper for using the ESP32 UART peripheral in an asynchronous mode.
 ******************************************************************************/

#pragma once

#include <cstdint>
#include <cstdlib>

namespace chronos {

namespace peripheral {

/**
 * @brief UART peripheral that operates using FIFO buffers in the background.
 *
 * The UART peripheral uses FIFO buffer both for sending and receiving data –
 * this means it can operate without needing a second task or user-defined
 * interrupts to supply data.
 *
 * Data received over the wire is stored in the peripheral and the amount of
 * available bytes can be queried.
 *
 * The downside of using this completely asynchronously is that if the FIFO
 * buffer overflows, data will be lost.
 *
 * The peripheral will be configured to use standard 8N1 UART communication.
 */
class UARTPeripheral {
  public:
    /**
     * @brief Construct a new Async UART Peripheral object.
     *
     * @param uart_num The UART peripheral port to be used (hardware-dependent).
     * @param baudrate The baudrate of the UART peripheral (in baud/s).
     * @param rx_buf_sz Size of the Rx FIFO buffer to be initialized.
     * @param tx_buf_sz Size of the Tx FIFO buffer to be initialized.
     *
     * @param rx_io_num Number of the I/O pin to be used for Rx.
     * @param tx_io_num Number of the I/O pin to be used for Tx.
     *
     * @note During configuration, the size of Rx and Tx buffers will be
     *       reserved through dynamic allocation (by the underlying driver)!
     * @note `rx_buf_sz` and `tx_buf_sz` need to exceed the hardware Rx and Tx
     *       FIFO buffer sizes, otherwise the underlying hardware configuration
     *       in `configure()` may fail an assertion.
     * @note This constructor accesses the hardware peripheral. Make sure that
     *       construction only happens when the hardware is ready.
     */
    UARTPeripheral(int uart_num, int baudrate, int rx_buf_sz, int tx_buf_sz,
                   int rx_io_num, int tx_io_num);

    /** Send a number of bytes over UART. */
    void send(const uint8_t *buf, size_t buf_len);

    /** Waits until the current UART transmission has completed. */
    void wait_tx_done();

    /** Available bytes in Rx FIFO buffer. */
    size_t available();

    /** Read a number of bytes from the Rx FIFO buffer. */
    size_t receive(uint8_t *buf, size_t buf_len, size_t requested);

    /** Clear the Rx FIFO buffer. */
    void clear_rx_buffer();

    /** Switch the baudrate of the peripheral. */
    bool set_baudrate(int baudrate);

    ~UARTPeripheral();

  private:
    const int uart_num_; ///< UART peripheral number, [0...2] on ESP32
    int baudrate_;       ///< Baudrate [baud/s]

    const int rx_buf_sz_; ///< Size of Rx buffer
    const int tx_buf_sz_; ///< Size of Tx buffer

    const int rx_io_num_; ///< Pin for Rx input
    const int tx_io_num_; ///< Pin for Tx output
};

}; // namespace peripheral

}; // namespace chronos
