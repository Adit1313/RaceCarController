/**
 * @file    udp_server.cpp
 * @author  Lukas Vogel (vogellu@ethz.ch)
 * @brief   Implementation of a UDP server based on Boost's ASIO.
 */

#include "wifi_com/udp_server.h"

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <iostream>

using namespace boost::asio::ip;

UDPServer::UDPServer(boost::asio::io_context& io_context, int port,
                     std::function<void(std::string, const udp::endpoint&)> receive_callback)
  : socket_(io_context, udp::endpoint(udp::v4(), port)), receive_callback_(receive_callback)
{
  // The socket is opened on construction to follow RAII – and it will be closed
  // by it's default destructor when UDPServer's destructor is called.
  startAsyncReceive();
}

void UDPServer::startAsyncReceive()
{
  socket_.async_receive_from(boost::asio::buffer(recv_buffer_), remote_endpoint_,
                             boost::bind(&UDPServer::receiveCallback, this, boost::asio::placeholders::error,
                                         boost::asio::placeholders::bytes_transferred));
}

void UDPServer::receiveCallback(const boost::system::error_code& error, std::size_t bytes_received)
{
  if (!error)
  {
    // Pass received data in correct length to the callback.
    std::string data(recv_buffer_.data(), bytes_received);
    receive_callback_(data, remote_endpoint_);

    // Schedule next async receive so that the I/O context keeps running
    startAsyncReceive();
  }
  if (error == boost::asio::error::message_size)
  {
    std::cerr << "UDP server: Incoming message size exceeds buffer capacity!" << std::endl;
  }
}

void UDPServer::send(const std::string& data, const udp::endpoint& endpoint)
{
  std::size_t sent = socket_.send_to(boost::asio::buffer(data), endpoint);

  if (sent < data.size())
  {
    std::cerr << "UDP server: Not all outgoing bytes were transmitted!" << std::endl;
  }
}
