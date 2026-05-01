#ifndef ROS_WIFI_COM_UDPSERVER_H
#define ROS_WIFI_COM_UDPSERVER_H

/**
 * @file    udp_server.h
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Implementation of a UDP server based on Boost's ASIO.
 */

#pragma once

#include <boost/asio.hpp>

/** Size of the server's receive buffer. This limits the length of packets it can handle. */
#define UDP_SERVER_RECEIVE_BUFFER_SIZE 512UL

/**
 * @brief Lightweight UDP server for exchanging data with one connecting instance.
 *
 * This server is used to communicate with the robotic systems that are controlled
 * through CRS. It runs asynchronously and allows simultaneously sending and receiving data.
 * It is based on the Boost ASIO library and uses callbacks to notify about incoming data.
 *
 * To set up a UDP server, provide a Boost IO context and specify the port number.
 * At instantiation, the callback must be provided. The class follows RAII and will
 * properly allocate and destroy its resources over its lifecycle.
 */
class UDPServer
{
public:
  /**
   * @brief Instantiate a server and open a UDP socket to receive data.
   * @note Data sent to the socket will not actually be processed
   *       until the io_context's event loop is run. This is the caller's responsibility.
   * @param io_context The boost I/O context the socket is run on.
   * @param port The local port the UDP socket should be opened on.
   * @param receive_callback Function that will be invoked with every received packet.
   */
  UDPServer(boost::asio::io_context& io_context, int port,
            std::function<void(std::string, const boost::asio::ip::udp::endpoint&)> receive_callback);

  /**
   * @brief Send data to an endpoint.
   * @param data The data to send.
   * @param endpoint The endpoint (i.e. IP/port combination) to send the data to.
   */
  void send(const std::string& data, const boost::asio::ip::udp::endpoint& endpoint);

private:
  /** Socket resource that is opened on construction. Automatically closed on destruction. */
  boost::asio::ip::udp::socket socket_;
  /** Last endpoint that the server was connected to. Modified by the async receive process.*/
  boost::asio::ip::udp::endpoint remote_endpoint_;

  /** Internal storage of the callback handle where the received data should be relayed to. */
  std::function<void(std::string, const boost::asio::ip::udp::endpoint&)> receive_callback_;

  /** Byte buffer where data is received. */
  std::array<char, UDP_SERVER_RECEIVE_BUFFER_SIZE> recv_buffer_;

  /**
   * @brief Starts the asynchronous receive process.
   *
   * This is called both during construction and immediately after receiving data. That way
   * it is ensured no data is lost and that the event loop keeps running.
   */
  void startAsyncReceive();

  /** Callback function invoked by the I/O context's event loop when data was received. */
  void receiveCallback(const boost::system::error_code& error, std::size_t bytes_received);
};

#endif /* ROS_WIFI_COM_UDPSERVER_H */
