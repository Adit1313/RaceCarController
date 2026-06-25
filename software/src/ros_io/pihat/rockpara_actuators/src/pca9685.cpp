#include "rockpara_actuators/pca9685.h"

using namespace ns_rockpara_actuators;

PCA9685::PCA9685(uint8_t address)
{
  this->devAddr = address;
}

PCA9685::~PCA9685()
{
  this->reset();
  i2cClose(this->handle_i2c);
  printf("I2C driver handles released\n");
}

void PCA9685::init()
{
  this->handle_i2c = i2cOpen(1, this->devAddr, 0);  // Open device at address on bus 1
  this->ref_clock_speed = float(REF_CLOCK_SPEED);
  this->reset();
}

void PCA9685::reset()
{
  i2cWriteByteData(this->handle_i2c, PCA9685_MODE1, 0x00);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

bool PCA9685::testConn()
{
  bool success = false;
  try
  {
    this->getFrequency();
    success = true;
  }
  catch (std::exception& e)
  {
    printf("Error conencting to the device: %s\n", e.what());
  }
  return success;
}

float PCA9685::getFrequency()
{
  uint8_t data = i2cReadByteData(this->handle_i2c, PCA9685_PRESCALE);
  return this->ref_clock_speed / 4096.f / (data + 1);
}

float PCA9685::setFrequency(float freq)
{
  float prescale_val = this->ref_clock_speed;  // 25MHz
  prescale_val /= 4096.0;                      // 12-bit
  prescale_val /= float(freq);
  prescale_val -= 1.0;
  int prescale = static_cast<int>(prescale_val + 0.5f);

  char old_mode = i2cReadByteData(this->handle_i2c, PCA9685_MODE1);
  char new_mode = (old_mode & 0x7F) | 0x10;                     // Sleep
  i2cWriteByteData(this->handle_i2c, PCA9685_MODE1, new_mode);  // Go to sleep

  i2cWriteByteData(this->handle_i2c, PCA9685_PRESCALE, prescale);
  i2cWriteByteData(this->handle_i2c, PCA9685_MODE1, old_mode);
  usleep(5000);  // 5ms delay

  i2cWriteByteData(this->handle_i2c, PCA9685_MODE1, old_mode | 0xA0);  // Auto-increment on

  this->frequency = getFrequency();
  return this->frequency;
}

void PCA9685::setPWM(uint8_t channel, uint16_t offset, uint16_t length)
{
  char data[4] = { 0, 0, 0, 0 };
  if (length == 0)
  {
    data[3] = 0x10;
  }
  else if (length >= 4096)
  {
    data[1] = 0x10;
  }
  else
  {
    data[0] = offset & 0xFF;
    data[1] = offset >> 8;
    data[2] = length & 0xFF;
    data[3] = length >> 8;
  }
  i2cWriteI2CBlockData(this->handle_i2c, LED0_ON_L + 4 * channel, data, 4);
}

void PCA9685::setPWM(uint8_t channel, uint16_t length)
{
  setPWM(channel, 0, length);
}

void PCA9685::setPWMmS(uint8_t channel, float length_mS)
{
  uint16_t length = round((length_mS * 4096.f) / (1000.f / this->frequency));
  setPWM(channel, length);
}

int main()
{
  /* example how to use the driver */

  // initialize pigpio
  int gpio_status = gpioInitialise();

  if (gpio_status < 0)
  {
    fprintf(stderr, "pigpio initialisation failed.\n");
    return 1;
  }

  PCA9685 driver_pca9685;
  driver_pca9685.init();
  usleep(10000);
  if (driver_pca9685.testConn())
  {
    printf("Connection test passed.\n");
  }
  else
  {
    return 1;
  }

  float freq = 50.0;
  printf("Set frequencey to: %f\n", driver_pca9685.setFrequency(freq));

  int channel = 0;

  // interactively select channel
  printf("Select channel (0 - 15): ");
  int status = scanf("%d", &channel);
  if (channel < 0 || channel > 15)
  {
    printf("Invalid channel. Abort.\n");
    return 1;
  }

  // // TEST: sweep between given values (use interacive input below to find the range)
  // for (register int i = 200; i < 400; i++) {
  //     driver_pca9685.setPWM(channel, i);
  //     std::this_thread::sleep_for(std::chrono::microseconds(20000));
  //     printf("Curr val: %d\n", i);
  // }

  // // TEST: sweep test with mS (usually from datasheet)
  // //   pay attention that both frequency and range should match with the datasheet
  // for (register int i = 1000; i < 2000; i++) {
  //     driver_pca9685.setPWMmS(channel, (float)i / 1000.0);
  //     std::this_thread::sleep_for(std::chrono::microseconds(2000));
  // }

  // // TEST: sweep full input range
  // std::vector<int> range(1 << 12);
  // std::iota(std::begin(range), std::end(range), 0);
  // for (auto& i : range) {
  //     driver_pca9685.setPWM(channel, i);
  //     usleep(1000);
  //     printf("Curr val: %d\n", i);
  // }

  // TEST: interactive input
  int num;
  printf("Input (0-4095): ");
  status = scanf("%d", &num);
  while (num != -1)
  {
    driver_pca9685.setPWM(channel, num);
    printf("Input: ");
    status = scanf("%d", &num);
  }

  // before exiting
  driver_pca9685.setPWM(channel, 0);

  std::this_thread::sleep_for(std::chrono::seconds(1));
  return 0;
}
