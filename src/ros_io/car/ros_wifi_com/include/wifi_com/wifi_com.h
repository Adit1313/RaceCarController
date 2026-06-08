#ifndef ROS_WIFI_COM_WIFI_COM_H
#define ROS_WIFI_COM_WIFI_COM_H

/**
 * @file wifi_com.h
 * @author Lukas Vogel (vogellu@ethz.ch)
 * @brief Class that communicates with cars over Wi-Fi using a UDPServer.
 */

#pragma once

#include <thread>
#include <optional>

#include <ros/ros.h>
#include <boost/asio.hpp>

#include <wifi_com/udp_server.h>
#include <crs_msgs/car_input.h>
#include <crs_msgs/lighthouse_sweep.h>
#include <crs_msgs/double_array_stamped.h>

#include "Packet.pb.h"

/* Number of supported Lighthouse base stations */
#define LIGHTHOUSE_BASE_STATIONS 16

/**
 * @brief Class that communicates with cars over Wi-Fi using a UDPServer.
 *
 * The WiFiCom class subscribes to the control_input topic and sends the torque
 * and steer commands it receives to the car. The cars send the following data:
 *
 * - IMU data, which is published on /ns/imu;
 * - the steer position, which is published on /ns/car_steer_state;
 * - the low-level control inputs that the car gave for logging/debugging, which
 *    is published on /ns/telemetry/actuator_inputs.
 *
 * The WiFiCom node communicates with the cars via UDP sockets and protocol
 * buffers. To ensure that incoming messages are processed in a timely manner,
 * a worker thread is spawned to process the Boost ASIO's event loop. It is
 * terminated when the WiFiCom object is deconstructed.
 *
 * Internally, the WiFiCom class uses Google Protocol Buffers for communication
 * with the car. The messages that are sent are defined in ./msgs/proto/ and are
 * compiled into C++ header/source files that deal with the serialization.
 * To add/modify messages, refer to the documentation of the protobuf:
 * https://developers.google.com/protocol-buffers/
 */
class WiFiCom
{
public:
  /**
   * @brief Constructor
   * @param nh Node Handle
   */
  WiFiCom(ros::NodeHandle& nh);

  /**
   * @brief Callback for incoming /ns/control_input messages.
   *
   * Every /ns/control_input message gets sent to the UDP socket.
   * @param msg the input
   */
  void controllerCallback(const crs_msgs::car_input::ConstPtr& msg);

  /**
   * @brief Send data to an endpoint.
   * @param data The data to send.
   * @param endpoint The endpoint that the data was received from.
   */
  void dataReceivedCallback(std::string data, const boost::asio::ip::udp::endpoint& endpoint);

private:
  double vel_ref = 0.0;
  /* Private methods -------------------------------------------------------- */
  /** Load the parameters from the configuration file */
  void loadParameters();

  /** Set up all subscriber objects. */
  void setupSubscribers();

  /** Set up all publisher objects. */
  void setupPublishers();

  /** Send a Protocol Buffers packet to the current connected car. */
  void sendPacket(const Packet& packet);

  /** Send a control input message from CRS to the car. */
  void sendControlInput(const crs_msgs::car_input::ConstPtr& msg);

  /** Publish the received IMU data on /ns/imu. */
  void publishImuData(const IMUMeasurement& data, uint64_t stamp);

  /** Publish the car's low-level motor inputs on telemetry/actuator_inputs. */
  void publishActuatorInputs(const SingleControlInput& reference, const MotorInput& drive_input,
                             const MotorInput& steer_input, uint64_t stamp);

  /** Publish the car's steering position on /ns/car_steer_state */
  void publishSteerState(const SteeringPositionMeasurement& steer_state, uint64_t stamp);

  /** Converts a timestamp from a car message to the corresponding ROS time. */
  ros::Time timestampToRosTime(uint64_t stamp);

  /** Publish the car's raw received Lighthouse frames on /ns/lighthouse_raw */
  void publishLighthouseData(const LighthouseFrame& lighthouse_frame);

  /** Publish the car's received Lighthouse sweeps on /ns/lighthouse_sweep */
  void publishLighthouseSweep(const LighthouseSweep& lighthouse_sweep, bool first_sweep);

  /** Publish the car's estimated longitudinal speed. */
  void publishSpeedEstimate(const float speed_estimate, uint64_t stamp);

  /** Determines if recived sweep is first or second sweep and publishes it */
  void handleLighthouseSweep(const LighthouseSweep& lighthouse_sweep);
  /* Private member variables ----------------------------------------------- */

  /** Flag to set if current lighthouse sweep is valid and should be published */
  bool valid_last_lighthouse_sweep_;

  /** Caches the last received first lighthouse sweep */
  crs_msgs::lighthouse_sweep last_first_lighthouse_sweep_;

  /** Retains the node handle passed to the constructor. */
  ros::NodeHandle& node_handle_;

  /** Subscriber object to control_input topic */
  ros::Subscriber sub_control_input_;
  ros::Subscriber sub_velocity_reference_;

  /** Publisher on the imu topic */
  ros::Publisher pub_imu_;

  /** Publisher on the wheel_speed topic */
  ros::Publisher pub_wheel_speed_;

  /** Publisher on the actuator_input topic */
  ros::Publisher pub_actuator_inputs_;

  /** Publisher on the car_steer_state topic */
  ros::Publisher pub_steer_state_;

  /** Publisher on the battery_state topic */
  ros::Publisher pub_battery_;

  /** Publisher on the embedded_clock topic */
  ros::Publisher pub_embedded_clock_;

  /** Publisher on the telemetry/speed_estimate topic. */
  ros::Publisher pub_speed_estimate_;

  /* UDP server related variables ------------------------------------------- */

  /** I/O context that is created automatically. The UDPServer's socket runs through this.*/
  boost::asio::io_context io_context_;
  std::thread async_worker_;  ///< Thread to run io_context_'s event loop.

  /** The endpoint (= IP/port) the WifiCom is communicating with.
   *  As long as no car has connected, this is not set. */
  std::optional<boost::asio::ip::udp::endpoint> endpoint_ = {};

  /** Publisher on the lighthouse raw frame topic */
  ros::Publisher pub_lighthouse_frame_;

  /** Publisher on the lighthouse sweep topic */
  ros::Publisher pub_lighthouse_sweep_;

  /**
   * UDP server object that handles the sending and receiving of UDP packets at
   * the byte level. Does not know about the protocol buffer layer.
   */
  std::unique_ptr<UDPServer> udp_server_;

  /** Port number that the server listens on. */
  int udp_port_;

  /** The last recived first lighthouse sweep per basestation */
  LighthouseSweep last_first_lighthouse_sweep[LIGHTHOUSE_BASE_STATIONS];
  /** Indicates if the last lighthouse sweep was already published or still needs to be published, per basestation*/
  bool last_lighthouse_sweep_published[LIGHTHOUSE_BASE_STATIONS];
};
#endif
