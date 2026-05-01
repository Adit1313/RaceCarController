/*******************************************************************************
 * @file    communication-layer.h
 * @brief   Set of callbacks that mediate between the control loop and the
 *          network layer.
 ******************************************************************************/

#pragma once

#include "control.hpp"

#include "Packet.pb.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @defgroup communication Communication */

/**
 * @addtogroup communication
 * @brief The communication components handle the sending and receiving of
 * messages between the CRS host and the embedded platforms.
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
 *
 * On the application layer, packets are serialized/deserialized by employing
 * the Google Protocol Buffers specification.
 * @{
 */

/**
 * @brief Registers as a callback for received UDP datagrams and sets up the
 *  discovery beacon.
 *
 * The communication layer will receive all UDP datagrams, deserialize and
 * forward to the application. To allow the host in CRS to discover cars, every
 * five seconds a beacon packet is sent out, which contains no data but allows
 * the server to find the local IP and local port of the car.
 */
void communication_layer_setup(chronos::ControlLoop *control_loop);

/**
 * @brief Send a packet via the communication layer.
 *
 * The packet is sent to the previously configured host. Also, the packet's
 * timestamp is updated with the time of sending.
 * @param p The packet to send.
 */
void communication_layer_send(Packet *p);

/** @} */

#ifdef __cplusplus
}
#endif
