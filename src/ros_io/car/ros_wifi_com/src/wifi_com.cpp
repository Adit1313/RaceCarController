/**
 * @file wifi_com.cpp
 * @author Lukas Vogel (vogellu@ethz.ch)
 * @brief Class that communicates with cars over Wi-Fi using a UDPServer.
 */

#include <wifi_com/wifi_com.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/BatteryState.h>

#include <crs_msgs/car_input.h>
#include <crs_msgs/car_actuator_inputs.h>
#include <crs_msgs/car_clock.h>
#include <crs_msgs/car_steer_state.h>
#include <crs_msgs/car_wheel_speed.h>
#include <crs_msgs/lighthouse_frame.h>
#include <crs_msgs/lighthouse_sweep.h>
#include <crs_msgs/double_array_stamped.h>

#include "Packet.pb.h"

/** Default port number to open the UDP server on in case nothing was specified
    in the config file. */
#define DEFAULT_PORT_NUM 20211

/** Maximum recommended size of a UDP packet that is sent without complaining. */
#define UDP_MAX_RECOMMENDED_SIZE 256

/* Static helper functions -------------------------------------------------- */

// The following functions are declared object-file static because including them in the
// header file would mean all the Protobuf declared types need to be visible to other
// files including the WiFiCom.h file.

/** Publish the wheel speed data to the namespace of this node. */
static void publishWheelSpeedData(const ros::Publisher& publisher, const WheelSpeedMeasurement& wheel_speed_measurement,
                                  ros::Time stamp);
/** Publish the battery data to the namespace of this node. */
static void publishBatteryState(const ros::Publisher& publisher, const BatteryState& battery_state, ros::Time stamp);

/* Public method implementation --------------------------------------------- */

WiFiCom::WiFiCom(ros::NodeHandle& nh) : node_handle_(nh), udp_port_(DEFAULT_PORT_NUM)
{
  loadParameters();
  setupSubscribers();
  setupPublishers();

  for (size_t i = 0; i < LIGHTHOUSE_BASE_STATIONS; i++)
  {
    last_first_lighthouse_sweep[i] = LighthouseSweep();
    last_lighthouse_sweep_published[i] = true;
  }

  try
  {
    // Launch the UDP server and open the socket. Register `dataReceivedCallback`
    // as the associated callback function. This may throw if the port is already
    // occupied or invalid or for any other reason.
    udp_server_ = std::make_unique<UDPServer>(io_context_, udp_port_,
                                              [=](std::string data, const boost::asio::ip::udp::endpoint& endpoint) {
                                                dataReceivedCallback(data, endpoint);
                                              });
  }
  catch (std::exception& e)
  {
    // Make sure it shows up in red in the console! Then rethrow to terminate.
    ROS_ERROR_STREAM("Could not start UDP server: " << e.what());
    throw;
  }

  ROS_INFO_STREAM("Opened a socket on port " << udp_port_);

  // Start a thread that works the I/O context. Automatically terminated when
  // WiFiCom is destructed.
  async_worker_ = std::thread([=] { io_context_.run(); });
}

void WiFiCom::dataReceivedCallback(std::string data, const boost::asio::ip::udp::endpoint& endpoint)
{
  endpoint_ = endpoint;

  Packet p;
  if (!p.ParseFromString(data))
  {
    ROS_ERROR("Could not parse packet.");
  }

  // Timestamp is not a required field, old car firmware might not send them. Therefore, set to 0 if not present.
  uint64_t stamp = p.has_timestamp() ? p.timestamp() : 0;

  if (p.has_car_state())
  {
    const CarState& state = p.car_state();
    if (state.has_drive_motor_input() && state.has_steer_motor_input() && state.has_current_reference())
    {
      publishActuatorInputs(state.current_reference(), state.drive_motor_input(), state.steer_motor_input(), stamp);
    }

    if (state.has_imu_data())
    {
      publishImuData(state.imu_data(), stamp);
    }

    if (state.has_steer_data())
    {
      publishSteerState(state.steer_data(), stamp);
    }

    if (state.has_wheel_speed_data())
    {
      publishWheelSpeedData(pub_wheel_speed_, state.wheel_speed_data(), timestampToRosTime(stamp));
    }

    if (state.has_battery_state())
    {
      publishBatteryState(pub_battery_, state.battery_state(), timestampToRosTime(stamp));
    }

    for (int i = 0; i < state.lighthouse_data_size(); i++)
    {
      publishLighthouseData(state.lighthouse_data(i));
    }

    for (int i = 0; i < state.lighthouse_sweeps_size(); i++)
    {
      handleLighthouseSweep(state.lighthouse_sweeps(i));
    }

    if (state.has_longitudinal_velocity())
    {
      publishSpeedEstimate(state.longitudinal_velocity(), stamp);
    }
  }
  else if (p.has_ping())
  {
    if (p.ping().should_bounce())
    {
      ROS_INFO_STREAM_THROTTLE(60, "Connection to " << ros::this_node::getNamespace() << " is alive.");
      p.mutable_ping()->set_should_bounce(false);

      sendPacket(p);
    }
  }
  else
  {
    ROS_WARN("Received unknown packet type, dropping...");
  }

  // Always publish embedded clock
  crs_msgs::car_clock msg;
  msg.header.stamp = ros::Time::now();
  msg.clock = ros::Time(stamp / 1.0e6);
  pub_embedded_clock_.publish(msg);
}

void WiFiCom::controllerCallback(const crs_msgs::car_input::ConstPtr& msg)
{
  sendControlInput(msg);
}

ros::Time WiFiCom::timestampToRosTime(uint64_t stamp)
{
  if (stamp > 0)
  {
    // Convert the microsecond timestamp to a ROS time
    return ros::Time(stamp / 1.0e6);
  }
  return ros::Time::now();
}

/* Private method implementation -------------------------------------------- */

void WiFiCom::loadParameters()
{
  if (!node_handle_.getParam("udp_port", udp_port_))
  {
    ROS_WARN_STREAM("No port number from config, using standard port: " << udp_port_);
  }
}

void WiFiCom::setupSubscribers()
{
  sub_control_input_ = node_handle_.subscribe("input", 1, &WiFiCom::controllerCallback, this);
}

void WiFiCom::setupPublishers()
{
  // While all these messages might arrive in the same packet from the car, each
  // belongs to a separate CRS topic and they are thus published on those:
  pub_imu_ = node_handle_.advertise<sensor_msgs::Imu>("imu", 1);
  pub_wheel_speed_ = node_handle_.advertise<crs_msgs::car_wheel_speed>("wheel_encoders", 1);
  pub_lighthouse_frame_ = node_handle_.advertise<crs_msgs::lighthouse_frame>("lighthouse_raw", 8);
  pub_lighthouse_sweep_ = node_handle_.advertise<crs_msgs::lighthouse_sweep>("lighthouse", 10);

  // These two topics are mostly for debugging the functionality of the car, and/or tuning. They are published
  // under /telemetry to emphasize that they are not used in estimators/controllers.
  pub_actuator_inputs_ = node_handle_.advertise<crs_msgs::car_actuator_inputs>("telemetry/actuator_inputs", 1);
  pub_steer_state_ = node_handle_.advertise<crs_msgs::car_steer_state>("telemetry/steer_state", 1);
  pub_battery_ = node_handle_.advertise<sensor_msgs::BatteryState>("telemetry/battery", 1);
  pub_embedded_clock_ = node_handle_.advertise<crs_msgs::car_clock>("telemetry/clock", 1);
  pub_speed_estimate_ = node_handle_.advertise<crs_msgs::double_array_stamped>("telemetry/speed_estimate", 1);
}

void WiFiCom::sendPacket(const Packet& packet)
{
  if (!endpoint_.has_value())
  {
    ROS_WARN_STREAM_THROTTLE(1, "Cannot send packet since no car is connected.");
    return;
  }

  // Timestamp the packet with the ROS time in microseconds
  Packet stamped_packet = packet;
  stamped_packet.set_timestamp(ros::Time::now().toNSec() / 1000);

  std::string serialized_bytes;
  stamped_packet.SerializeToString(&serialized_bytes);

  // Packets that are too long may be transferred in more than one transaction,
  // which is untested both in the server as in the client code.
  if (serialized_bytes.length() > UDP_MAX_RECOMMENDED_SIZE)
  {
    ROS_WARN_STREAM("Packet length is " << serialized_bytes.length() << "B, while recommended limit is "
                                        << UDP_MAX_RECOMMENDED_SIZE << "B!");
  }

  // Send packet
  udp_server_->send(serialized_bytes, endpoint_.value());
}

void WiFiCom::sendControlInput(const crs_msgs::car_input::ConstPtr& msg)
{
  Packet p;
  CarReference* ref = p.mutable_reference();

  // Choose the right forward control input: torque or velocity
  // Convention: If both are set, the car will use the torque.
  if (!isnan(msg->torque))
  {
    ref->set_throttle(msg->torque);
  }
  else if (!isnan(msg->velocity))
  {
    ref->set_longitudinal_velocity(msg->velocity);
  }
  else
  {
    ROS_WARN("Received control input with neither torque nor velocity set, dropping...");
    return;
  }

  // Choose the right steering control input: angle or voltage.
  // "Override" is a flag that tells the car to use the voltage instead of the angle.
  if (msg->steer_override)
  {
    ref->set_voltage(msg->steer);
  }
  else
  {
    ref->set_angle(msg->steer);
  }

  sendPacket(p);
}

void WiFiCom::publishImuData(const IMUMeasurement& data, uint64_t stamp)
{
  sensor_msgs::Imu msg;

  // Convention: Set covariance of sensor measurement "orientation" to -1 if
  // this message field is invalid (which it is here, we don't have an absolute
  // orientation measurement yet). See ROS documentation for sensor_msgs/imu.
  msg.orientation_covariance[0] = -1;

  // Remap axes and adjust units:
  // - x and y axes need to be reversed
  msg.linear_acceleration.x = -data.linear_acceleration().x();
  msg.linear_acceleration.y = -data.linear_acceleration().y();
  msg.linear_acceleration.z = +data.linear_acceleration().z();
  msg.angular_velocity.x = -data.angular_velocity().x();
  msg.angular_velocity.y = -data.angular_velocity().y();
  msg.angular_velocity.z = +data.angular_velocity().z();

  msg.header.stamp = timestampToRosTime(stamp);

  pub_imu_.publish(msg);
}

void WiFiCom::publishActuatorInputs(const SingleControlInput& reference, const MotorInput& drive_input,
                                    const MotorInput& steer_input, uint64_t stamp)
{
  crs_msgs::car_actuator_inputs msg;

  msg.drive_power = drive_input.power();
  msg.steer_power = steer_input.power();

  msg.steer_ref = reference.steer_ref();
  msg.torque_ref = reference.torque_ref();

  msg.header.stamp = timestampToRosTime(stamp);
  pub_actuator_inputs_.publish(msg);
}

void WiFiCom::publishSteerState(const SteeringPositionMeasurement& steer_state, uint64_t stamp)
{
  crs_msgs::car_steer_state msg;
  msg.steer_angle = steer_state.steer_rad();
  msg.steer_discrete_pos = steer_state.adc_meas();

  msg.header.stamp = timestampToRosTime(stamp);

  pub_steer_state_.publish(msg);
}

void WiFiCom::publishSpeedEstimate(const float speed_estimate, uint64_t stamp)
{
  crs_msgs::double_array_stamped msg;
  msg.header.stamp = timestampToRosTime(stamp);
  msg.data.push_back(speed_estimate);
  pub_speed_estimate_.publish(msg);
}

/* Static helper function implementation ---------------------------------------------------------------------------- */

static void publishWheelSpeedData(const ros::Publisher& publisher, const WheelSpeedMeasurement& wheel_speed_measurement,
                                  ros::Time stamp)
{
  crs_msgs::car_wheel_speed msg;
  msg.header.stamp = stamp;

  msg.front_left = wheel_speed_measurement.front_left();
  msg.front_right = wheel_speed_measurement.front_right();
  msg.rear_left = wheel_speed_measurement.rear_left();
  msg.rear_right = wheel_speed_measurement.rear_right();

  publisher.publish(msg);
}

static void publishBatteryState(const ros::Publisher& publisher, const BatteryState& battery_state, ros::Time stamp)
{
  sensor_msgs::BatteryState msg;

  msg.header.stamp = stamp;

  // Basic, usually known
  msg.voltage = battery_state.voltage();
  msg.current = battery_state.has_current() ? battery_state.current() : NAN;

  // Set other fields according to specification if unmeasured
  msg.temperature = NAN;
  msg.charge = NAN;
  msg.capacity = NAN;
  msg.percentage = NAN;
  msg.power_supply_status = sensor_msgs::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  msg.power_supply_health = sensor_msgs::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;

  msg.location = "unknown";
  msg.serial_number = "unknown";

  publisher.publish(msg);
}

void WiFiCom::publishLighthouseData(const LighthouseFrame& lighthouse_frame)
{
  crs_msgs::lighthouse_frame msg;
  msg.sensor_id = lighthouse_frame.sensor_id();
  msg.polynomial = lighthouse_frame.polynomial();
  msg.pulse_width = lighthouse_frame.pulse_width();
  msg.sync_offset = lighthouse_frame.sync_offset();
  msg.beam_word = lighthouse_frame.beam_word();
  msg.timestamp = lighthouse_frame.timestamp();

  msg.header.stamp = ros::Time::now();

  pub_lighthouse_frame_.publish(msg);
}

void WiFiCom::publishLighthouseSweep(const LighthouseSweep& lighthouse_sweep, bool first_sweep)
{
  crs_msgs::lighthouse_sweep msg;

  double angle_offset;
  if (first_sweep)
  {
    angle_offset = -M_PI + M_PI / 3;
  }
  else
  {
    angle_offset = -M_PI - M_PI / 3;
  }

  msg.angle_0 = lighthouse_sweep.angle_0() + angle_offset;
  msg.angle_1 = lighthouse_sweep.angle_1() + angle_offset;
  msg.angle_2 = lighthouse_sweep.angle_2() + angle_offset;
  msg.angle_3 = lighthouse_sweep.angle_3() + angle_offset;
  msg.first_timestamp = lighthouse_sweep.first_timestamp();
  msg.polynomial = lighthouse_sweep.polynomial();
  msg.sync_timestamp = lighthouse_sweep.sync_timestamp();

  // Check if angle is inside field of view
  if (msg.angle_0 < -1.5 || msg.angle_0 > 1.5)
  {
    return;
  }

  msg.first_sweep = first_sweep;

  msg.header.stamp = ros::Time::now();

  pub_lighthouse_sweep_.publish(msg);
}

void WiFiCom::handleLighthouseSweep(const LighthouseSweep& lighthouse_sweep)
{
  size_t base_station = lighthouse_sweep.polynomial() >> 1;
  if (base_station >= LIGHTHOUSE_BASE_STATIONS)
  {
    // Invalid base station id
    return;
  }
  // Time difference between last and current sweeps sync event
  int32_t time_diff = lighthouse_sweep.sync_timestamp() - last_first_lighthouse_sweep[base_station].sync_timestamp();
  // Check if abs(time_diff) < 100 in 24 bit representation
  if (((time_diff + 100) & 0x00FFFFFF) < 200)
  {
    // Last sweep was first this is second
    if (!last_lighthouse_sweep_published[base_station])
    {
      publishLighthouseSweep(last_first_lighthouse_sweep[base_station], true);
      last_lighthouse_sweep_published[base_station] = true;
    }
    publishLighthouseSweep(lighthouse_sweep, false);
  }
  else
  {
    // Curent sweep dose not belong to last sweep
    // Check if last_first_lighthouse_sweep angle is within ~6 deg of current sweep angle
    if (abs(last_first_lighthouse_sweep[base_station].angle_0() - lighthouse_sweep.angle_0()) < 0.1)
    {
      // Current sweep is probably first sweep
      publishLighthouseSweep(lighthouse_sweep, true);
      last_first_lighthouse_sweep[base_station] = LighthouseSweep(lighthouse_sweep);
      last_lighthouse_sweep_published[base_station] = true;
    }
    else
    {
      // Some sweeps were dropped, reset
      last_first_lighthouse_sweep[base_station] = LighthouseSweep(lighthouse_sweep);
      last_lighthouse_sweep_published[base_station] = false;
    }
  }
}
