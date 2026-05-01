/*******************************************************************************
 * @file    main.cpp
 * @brief   Main entry point for the Chronos Wi-Fi Car Firmware
 ******************************************************************************/

#include "main.hpp"

#include <float.h>
#include <stdio.h>

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs-manager.h"
#include "sdkconfig.h"
#include "udp-client.h"
#include "wifi.h"
}
#include "adc.hpp"
#include "communication-layer.h"
#include "control.hpp"
#include "dc-motor.hpp"
#include "lighthouse.hpp"
#include "null_interfaces.hpp"
#include "pid.hpp"
#include "pinout.h"
#include "rtos.hpp"
#include "soc/adc_channel.h"
#include "state-machine.h"
#include "uart.hpp"
#include "velocity-estimator.hpp"

/** Tag used in logging */
static const char *TAG = "main";

/* Local function declarations ---------------------------------------------- */

/**
 * @brief Callback that is invoked if the motors encounter a fault.
 * @param pwm_unit The PWM unit that encountered the fault.
 */
static void IRAM_ATTR actuator_fault_callback(mcpwm_unit_t pwm_unit);

/* Public function implementation ------------------------------------------- */

extern "C" void app_main(void) {

    // Initialize non-volatile storage. This enables reading the stored
    // settings, Wi-Fi calibration data etc.
    nvs_init();
    nvs_load_config();
    ESP_LOGI(TAG, "Config ip: '%s', port: %u, ssid: '%s', pwd: '%s'",
             global_config.host_ip, global_config.host_port, global_config.ssid,
             global_config.pwd);

    adc_setup();

    // Internal stuff is set up. Now, a Wi-Fi connection can be established,
    // communication with other hosts can be started.
    wifi_init();

    chronos::rtos_launch(&chronos::control_loop());

    if (!wifi_connect_to_ap(pdMS_TO_TICKS(20000))) {
        // If the Wi-Fi connection fails, this usually means that the
        // credentials are bad. In that case, deviate from the launch and start
        // the configuration server so that new credentials can be entered.
        state_machine_input(StartupFailure);
    }

    // Set up the time synchronizer. This improves the measurement timestamps.
    (void)chronos::time_synchronizer();

    communication_layer_setup(&chronos::control_loop());

    // Set up an interrupt on pressing the BOOT button: if it is pressed, enter
    // the configuration mode
    // Configure EN/FAULT pin as open drain output
    gpio_config_t gpio0_cfg = {
        .pin_bit_mask = (1ULL << 0),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&gpio0_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add((gpio_num_t)0, (void (*)(void *))(state_machine_input),
                         (void *)Configure);

    // As per the ESP-IDF documentation, it is allowed to return from app_main
    // once the setup work is done. The task's resources will get cleaned up.
    state_machine_input(StartupCompleted);

    ESP_LOGI(TAG, "Finished setup of system. Returning from main task.");
}

/* Global, lazy-initialized objects ----------------------------------------- */

chronos::ControlLoop &chronos::control_loop() {
    using namespace chronos;

    static ControlLoop loop(steer_motor(), throttle_motor(), steer_controller(),
                            steer_voltage_controller(), velocity_controller(),
                            velocity_estimator(), steer_feedback(), imu(),
                            wheel_encoders(), lighthouse(),
                            battery_voltage_sensor(), total_current_sensor());

    return loop;
}

chronos::time::TimeSynchronizer &chronos::time_synchronizer() {
    static time::TimeSynchronizer time_synchronizer_;
    return time_synchronizer_;
}

chronos::interface::Actuator &chronos::steer_motor() {
    static chronos::DCMotor steer_motor_((chronos::DCMotorConfig){
        .pwm_pin = (gpio_num_t)STEER_PWM_PIN,
        .phase_pin = (gpio_num_t)STEER_PH_PIN,
        .en_fault_pin = (gpio_num_t)STEER_EN_PIN,

        .mcpwm_unit = MCPWM_UNIT_1,
        .mcpwm_timer = MCPWM_TIMER_1,

        .pwm_freq = 1000,
        .auto_coasting = false,
        .reverse_phase = false,

        .fault_callback = &actuator_fault_callback,
    });
    steer_motor_.enable();

    return steer_motor_;
}

chronos::interface::Actuator &chronos::throttle_motor() {
    static chronos::DCMotor throttle_motor_((chronos::DCMotorConfig){
        .pwm_pin = (gpio_num_t)DRIVE_PWM_PIN,
        .phase_pin = (gpio_num_t)DRIVE_PH_PIN,
        .en_fault_pin = (gpio_num_t)DRIVE_EN_PIN,

        .mcpwm_unit = MCPWM_UNIT_0,
        .mcpwm_timer = MCPWM_TIMER_0,

        .pwm_freq = 5000,
        .auto_coasting = true,
        .reverse_phase = false,

        .fault_callback = &actuator_fault_callback,
    });
    return throttle_motor_;
}

chronos::interface::Controller<PWMDutyCycle, SteeringAngle> &
chronos::steer_controller() {
    static chronos::control::PID<PWMDutyCycle, SteeringAngle>
        steer_angle_controller_(global_config.pid_kp, // P gain
                                global_config.pid_ki, // I gain
                                global_config.pid_kd, // D gain
                                (1.0f /
                                 CONFIG_CONTROL_LOOP_RATE), // Sample interval
                                global_config.pid_Tf);
    static bool is_initialized = false;

    // The steer controller is initialized to a normalization divider
    // of 0.1 rad.
    if (!is_initialized) {

        steer_angle_controller_.set_normalization_divider(0.1);
        is_initialized = true;
    }
    return steer_angle_controller_;
}

chronos::interface::Controller<PWMDutyCycle, float> &
chronos::steer_voltage_controller() {
    static chronos::control::PID<PWMDutyCycle, float> steer_voltage_controller_(
        CONFIG_STEER_VOLTAGE_KP,           // P gain
        CONFIG_STEER_VOLTAGE_KI,           // I gain
        CONFIG_STEER_VOLTAGE_KD,           // D gain
        (1.0f / CONFIG_CONTROL_LOOP_RATE), // Sample interval,
        CONFIG_STEER_VOLTAGE_TF            // Derivative filter time constant
    );

    return steer_voltage_controller_;
}

/** Returns a reference to the velocity controller object. */
chronos::interface::Controller<PWMDutyCycle, LongitudinalVelocity> &
chronos::velocity_controller() {
    static chronos::control::PID<PWMDutyCycle, LongitudinalVelocity>
        velocity_controller_(global_config.pid_vel_kp, global_config.pid_vel_ki,
                             global_config.pid_vel_kd,
                             (1.0f / CONFIG_CONTROL_LOOP_RATE));
    return velocity_controller_;
}

chronos::interface::Estimator<LongitudinalVelocity, WheelSpeed> &
chronos::velocity_estimator() {
    static chronos::estimation::LongitudinalVelocityEstimator
        velocity_estimator_(global_config.wheel_radius);
    return velocity_estimator_;
}

chronos::interface::Sensor<InertialMeasurement> &
chronos::imu() {
    static chronos::BMI088
        imu_(chronos::bmi_accel_device(), chronos::bmi_gyro_device(),
             (BMI088AccelFullscaleRange)global_config.accel_fsr,
             (BMI088GyroFullscaleRange)global_config.gyro_fsr);
    return imu_;
}

chronos::interface::Sensor<WheelSpeed> &
chronos::wheel_encoders() {
    static chronos::WheelEncoder
        wheel_encoders_(chronos::wheel_encoder_device(),
                        global_config.wheel_magnet_count);
    return wheel_encoders_;
}

chronos::interface::Sensor<ADCMeasurement> &
chronos::steer_feedback() {
    static struct ADCChannel steering_fb_adc = {
        .adc = ADCPeripheral1,
        .channel1 = ADC1_CHANNEL_6,
        .attenuation = ADC_ATTEN_DB_11,
    };
    static chronos::ADCSensor steer_feedback_(steering_fb_adc);
    return steer_feedback_;
}

chronos::interface::Sensor<chronos::LighthouseSweepList> &
chronos::lighthouse() {
    static chronos::Lighthouse lighthouse_(uart0());
    return lighthouse_;
}

/** Reference to the UART0 bus. */
chronos::peripheral::UARTPeripheral &
chronos::uart0() {
    static chronos::peripheral::UARTPeripheral uart0_(
        0, 115200, 240, 240,
        3, // RX pin is GPIO3 == RXD0
        1  // TX pin is GPIO1 == TXD0
    );
    return uart0_;
}

chronos::interface::Sensor<ADCMeasurement> &chronos::battery_voltage_sensor() {
    /** Configuration for the battery sensing ADC. Needs to read a range of
     * [0.4...1] V*/
    static struct ADCChannel voltage_adc = {
        .adc = ADCPeripheral1,
        .channel1 = (adc1_channel_t)ADC1_GPIO36_CHANNEL,
        .attenuation = ADC_ATTEN_DB_11,
    };
    static chronos::ADCSensor battery_voltage_sensor_(voltage_adc);
    return battery_voltage_sensor_;
}

chronos::interface::Sensor<ADCMeasurement> &
chronos::total_current_sensor() {
    /** Configuration for the current sensing ADC. Needs to read a range of
     * [0...2.5] V */
    static struct ADCChannel current_adc = {
        .adc = ADCPeripheral1,
        .channel1 = (adc1_channel_t)ADC1_GPIO35_CHANNEL,
        .attenuation = ADC_ATTEN_DB_11,
    };
    static chronos::ADCSensor total_current_sensor_(current_adc);
    return total_current_sensor_;
}

chronos::SPIMasterBus &
chronos::spi2() {
    static chronos::SPIMasterBus spi2_(
        chronos::SPIMasterBus::SPIPort::SPI_PORT_2, SPI2_MOSI_PIN,
        SPI2_MISO_PIN, SPI2_CLK_PIN);
    return spi2_;
}

chronos::SPIDevice &chronos::bmi_accel_device() {
    static chronos::SPIDevice bmi_accel_device_(spi2(), SPI2_ACCEL_CS_PIN,
                                                SPI2_CLOCK_SPEED, true);
    return bmi_accel_device_;
}

chronos::SPIDevice &chronos::bmi_gyro_device() {
    static chronos::SPIDevice bmi_gyro_device_(spi2(), SPI2_GYRO_CS_PIN,
                                               SPI2_CLOCK_SPEED, true);
    return bmi_gyro_device_;
}

chronos::SPIDevice &chronos::wheel_encoder_device() {
    static chronos::SPIDevice wheel_encoder_device_(spi2(), SPI2_STM32_CS_PIN,
                                                    SPI2_CLOCK_SPEED, false);
    return wheel_encoder_device_;
}

/* Interrupt callbacks ------------------------------------------------------ */

static void IRAM_ATTR actuator_fault_callback(mcpwm_unit_t pwm_unit) {
    // nothing happens currently
    return;
}
