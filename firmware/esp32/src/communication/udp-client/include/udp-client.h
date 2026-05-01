/*******************************************************************************
 * @file    udp-client.h
 * @brief   Task that sends and receives UDP frames to the CRS host application.
 *
 * As a transport layer, plain UDP frames are exchanged between the embedded
 * platform and the host. The UDP layer does not look at the encapsulated data
 * in the frames.
 *
 * Even though UDP is connection-less, the embedded platform still acts as a
 * "client" to a fixed host. Therefore, this API also doesn't (yet) allow
 * sending messages to arbitrary peers on the network. The host's IP address and
 * the port are configurable through the web interface.
 *
 * The UDP client runs as an independent task at a high rate (~5kHz) to ensure
 * low latency when receiving / sending frames.
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

/** @addtogroup communication
 * @{
 */

/* Public type definitions -------------------------------------------------- */

/** Function pointer that can act as a callback for when a packet is received */
typedef void (*udp_reception_callback)(uint16_t len, const uint8_t *data);

/* Public function declarations --------------------------------------------- */

/**
 * @brief Entry function of the UDP client task.
 * @param pvParameters parameters that can be passed through task instantiation.
 */
void udp_client_task(void *pvParameters);

/**
 * @brief Entry function of the UDP client receive task.
 *
 * The client receive task waits for incoming UDP packets and calls the callback
 * function when a packet is received. It is not responsible for managing the
 * socket, which is done by the main UDP client task.
 * @param pvParameters parameters that can be passed through task instantiation.
 */
void udp_client_receive_task(void *pvParameters);

/**
 * @brief Set the callback function that is called when a packet is received.
 * @note Your callback function MUST copy the data it is passed, as it may be
 *       overwritten as soon as you return from the callback.
 * @param cb Callback invoked when a UDP packet was received.
 */
void udp_client_register_callback(udp_reception_callback cb);

/**
 * @brief Send a datagram to the connected remote host.
 *
 * This can fail, particularly when too many packets are queued for sending. If
 * too many packets are queued, this will return false and drop the packet. Also
 * the return value says whether *staging* was successful. The sending might
 * still (silently).
 * @param len Length of the datagram to be sent. Must be less than 508B!
 * @param data The data to be sent.
 * @note The length of the data to be sent should be less than 508B, which is
 *      the minimum size that any UDP implementation needs to support. Larger
 *      packets will be dropped by the UDP client task.
 * @returns True if the packet could be queued for sending or not.
 */
bool udp_client_send_packet(uint16_t len, const uint8_t *data);

/** @} */
