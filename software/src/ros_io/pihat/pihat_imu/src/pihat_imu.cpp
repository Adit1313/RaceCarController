#include <functional>
#include <string>

#include "bmi08x.h"
#include "pihat_imu/bmi088.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"

using Vector3Stamped = geometry_msgs::Vector3Stamped;
typedef int8_t (*bmi_bus_read_function)(uint8_t, uint8_t*, uint32_t, void*);
typedef int8_t (*bmi_bus_write_function)(uint8_t, const uint8_t*, uint32_t, void*);

uint8_t mode = 0;
uint32_t speed = 100000;
uint8_t bits = 8;

double gravity_acc = 9.8;  // m/s2
#define PI 3.14159265358979323846

int8_t spi_read(uint8_t reg_addr, uint8_t* data, uint32_t len, uint8_t* fd)
{
  struct spi_ioc_transfer spi[len];

  data[0] = reg_addr;
  memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
  spi[0].tx_buf = (unsigned long long)((&reg_addr));
  // spi[0].rx_buf = (unsigned long long)(data);
  spi[0].len = 1;
  spi[0].speed_hz = speed;
  spi[0].bits_per_word = bits;

  for (uint32_t i = 1; i < len; ++i)
  {
    memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
    spi[i].rx_buf = (unsigned long long)(data + i);
    spi[i].len = 1;
    spi[i].speed_hz = speed;
    spi[i].bits_per_word = bits;
  }
  int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len), spi);

  if (rslt < 0)
  {
    perror("Error transfering data over SPI bus\n");
    close(*fd);
    return -1;
  }

  return 0;
}

int8_t spi_write(uint8_t reg_addr, const uint8_t* data, uint32_t len, uint8_t* fd)
{
  struct spi_ioc_transfer spi[len + 1];

  memset(&spi[0], 0, sizeof(struct spi_ioc_transfer));
  spi[0].tx_buf = (unsigned long long)(&reg_addr);
  spi[0].len = 1;
  spi[0].speed_hz = speed;
  spi[0].bits_per_word = bits;

  for (uint32_t i = 1; i < len + 1; ++i)
  {
    memset(&spi[i], 0, sizeof(struct spi_ioc_transfer));
    spi[i].tx_buf = (unsigned long long)(data + i - 1);
    // spi[i].rx_buf = (unsigned long long)(data+i-1);
    spi[i].len = 1;
    spi[i].speed_hz = speed;
    spi[i].bits_per_word = bits;
  }
  int8_t rslt = ioctl(*fd, SPI_IOC_MESSAGE(len + 1), spi);

  if (rslt < 0)
  {
    perror("Error transfering data over SPI bus\n");
    close(*fd);
    return -1;
  }

  return 0;
}

class PihatIMU
{
public:
  PihatIMU(ros::NodeHandle& n) : node_handle_(n)
  {
    loadParameters();
    imu_callback_rate_s_ = 1.f / imu_callback_freq_hz_;
    acc_pub = n.advertise<Vector3Stamped>("body_acc", 1);
    ang_pub = n.advertise<Vector3Stamped>("body_ang_vel", 1);
    timer_ = n.createWallTimer(ros::WallDuration(imu_callback_rate_s_),  // update rate is 1/50ms = 20Hz
                               std::bind(&PihatIMU::timer_callback, this, std::placeholders::_1));

    acc_dev_addr = open("/dev/spidev0.0", O_RDWR);
    gyro_dev_addr = open("/dev/spidev0.1", O_RDWR);

    if (set_and_check())
    {
      ROS_INFO("acc addr: %d, gyro addr: %d, successful initiation!\n", acc_dev_addr, gyro_dev_addr);
    }
  }

private:
  void loadParameters()
  {
    if (!node_handle_.getParam("imu_callback_freq_hz", imu_callback_freq_hz_))
    {
      imu_callback_freq_hz_ = 200.f;
      ROS_WARN("Unable to load IMU parameters. Falling back to default callback frequency %f", imu_callback_freq_hz_);
    }

    ROS_INFO("IMU callback frequency set to %f Hz", imu_callback_freq_hz_);
  }

  void timer_callback(const ros::WallTimerEvent& event)
  {
    InertialMeasurement meas;
    bmi088_sample(&device, &meas);
    auto current_time = ros::Time(event.current_real.sec, event.current_real.nsec);

    // imu frame transfer to body frame!!
    // standing still (x-axis points to the sky), the rocket will
    //  measure a positive acceleration in x-direction
    auto acc_msg = Vector3Stamped();
    acc_msg.header.stamp = current_time;
    acc_msg.vector.y = -meas.linear_accel.x * gravity_acc;
    acc_msg.vector.x = -meas.linear_accel.y * gravity_acc;
    acc_msg.vector.z = -meas.linear_accel.z * gravity_acc;

    auto ang_msg = Vector3Stamped();
    ang_msg.header.stamp = current_time;
    ang_msg.vector.y = -meas.angular_vel.x / 180.0 * PI;
    ang_msg.vector.x = -meas.angular_vel.y / 180.0 * PI;
    ang_msg.vector.z = -meas.angular_vel.z / 180.0 * PI;

    acc_pub.publish(acc_msg);
    ang_pub.publish(ang_msg);
  }

  bool set_and_check()
  {
    uint8_t fds[2] = { acc_dev_addr, gyro_dev_addr };
    for (int i = 0; i < 2; i++)
    {
      uint8_t fd = fds[i];
      if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0)
      {
        perror("bad mode\n");
        close(fd);
        return false;
      }

      if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
      {
        perror("err set speed\n");
        close(fd);
        return false;
      }

      if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0)
      {
        perror("err set bits per word\n");
        close(fd);
        return false;
      }
    }

    device.bus_read = (bmi_bus_read_function)&spi_read;
    device.bus_write = (bmi_bus_write_function)&spi_write;
    device.intf_ptr_accel = &acc_dev_addr;
    device.intf_ptr_gyro = &gyro_dev_addr;

    bool rslt = bmi088_init(&device);
    return true;
  }

  // class members
  float imu_callback_rate_s_;
  float imu_callback_freq_hz_;
  ros::NodeHandle& node_handle_;
  ros::WallTimer timer_;
  ros::Publisher acc_pub;
  ros::Publisher ang_pub;
  uint8_t acc_dev_addr;
  uint8_t gyro_dev_addr;
  BMI088 device;
};

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pihat_imu");
  ros::NodeHandle nh_private = ros::NodeHandle("~");

  PihatIMU imu_node(nh_private);

  ros::spin();

  return 0;
}
