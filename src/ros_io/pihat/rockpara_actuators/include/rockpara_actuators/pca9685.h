#ifndef ROCKPARA_ACTUATORS_PCA9685_H
#define ROCKPARA_ACTUATORS_PCA9685_H

#include "pigpio.h"

#include <unistd.h>

#include <numeric>
#include <vector>
#include <cstdio>
#include <stdexcept>
#include <atomic>
#include <cmath>
#include <chrono>
#include <thread>

#define PCA9685_ADDR 0x40  // device's I2C address
#define PCA9685_MODE1 0x00
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x06  // First LED channel
#define LED0_ON_H 0x07
#define LED0_OFF_L 0x08
#define LED0_OFF_H 0x09
#define REF_CLOCK_SPEED 25000000  // 25MHz
// Ref:
// https://github.com/adafruit/Adafruit_CircuitPython_PCA9685/blob/80b9d29814f9cde79326b2941f1e3dd1c5d3e55a/examples/pca9685_servo.py#L39
#define PWM_LENGTH_MIN 750
#define PWM_LENGTH_MAX 2250

namespace ns_rockpara_actuators
{

class PCA9685
{
public:
  /** PCA9685 constructor.
   * @param address I2C address
   * @see PCA9685_ADDR
   */
  PCA9685(uint8_t address = PCA9685_ADDR);
  ~PCA9685();

  /* Initialize I2C comm and set ref clock speed */
  void init();

  /** Verify the I2C connection.
   * @return True if connection is valid, false otherwise
   */
  bool testConn();

  /** Calculate prescale value based on the specified frequency and write it to the device.
   * @return Frequency in Hz
   * @see PCA9685_PRESCALE
   */
  float getFrequency();

  /** Calculate prescale value based on the specified frequency and write it to the device.
   * @param frequency in Hz
   * @see PCA9685_PRESCALE
   * @return The newly set frequency
   */
  float setFrequency(float frequency);

  /** Set the duty cycle of a channel
   * @param channel the index of the output channel on PCA9685 (0-15)
   * @param offset start offset of the pulse (0-4095)
   * @param length length of the pulse (0-4095)
   */
  void setPWM(uint8_t channel, uint16_t offset, uint16_t length);

  /** Set channel's pulse length (assuming no offset)
   * @param channel the index of the output channel on PCA9685 (0-15)
   * @param Length length of the pulse (0-4095)
   */
  void setPWM(uint8_t channel, uint16_t length);

  /** Set channel's pulse length in milliseconds
   * @param channel the index of the output channel on PCA9685 (0-15)
   * @param length the duty cycle in milliseconds
   */
  void setPWMmS(uint8_t channel, float length_mS);

  /* Reset the device */
  void reset();

private:
  int handle_i2c;
  uint8_t devAddr;
  float frequency;
  float ref_clock_speed;
};

}  // namespace ns_rockpara_actuators

#endif
