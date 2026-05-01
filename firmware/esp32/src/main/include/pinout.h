/*******************************************************************************
 * @file    pinout.hpp
 * @brief   Macros that map to the pinout of the ESP32 on the board.
 ******************************************************************************/

#pragma once

/* Steering feedback pins --------------------------------------------------- */

// TODO: This is not used yet, since for the ADC actually the channels have
// to be configured.
#define STEER_FEEDBACK_PIN (34)

/* Motor driver pins -------------------------------------------------------- */

/**
 * @brief Enable / fault pin of the steering BLDC.
 *
 * Must be high for the motor to turn. Can be pulled low to disable the motor,
 * or will be pulled low in case of overcurrent.
 */
#define STEER_EN_PIN (25)

/**
 * @brief PWM pin of the steering motor.
 *
 * The STSPIN250 motor driver, which supplies current to the BLDC, measures the
 * duty cycle and supplies power proportional to the duty cycle of the PWM on
 * this pin. If duty cycle = 100%, then it uses max power, if 0% it doesn't
 * power the motor.
 */
#define STEER_PWM_PIN (33)

/** Steering phase pin. Sets the turning direction of the steering BLDC.
                            Low for turning right, high for turning left. */
#define STEER_PH_PIN (32)

/**
 * @brief Enable / fault pin of the driving BLDC.
 *
 * Must be high for the motor to turn. Can be pulled low to disable the motor,
 * or will be pulled low in case of overcurrent.
 */
#define DRIVE_EN_PIN (16)

/**
 * @brief PWM pin of the driving motor.
 *
 * The STSPIN250 motor driver, which supplies current to the BLDC, measures the
 * duty cycle and supplies power proportional to the duty cycle of the PWM on
 * this pin. If duty cycle = 100%, then it uses max power, if 0% it doesn't
 * power the motor.
 */
#define DRIVE_PWM_PIN (27)

/** Driving phase pin. Sets the turning direction of the driving BLDC.
 *                                     High for forward, low for reverse. */
#define DRIVE_PH_PIN (26)

/* SPI2 pins ---------------------------------------------------------------- */

// SPI2 bus and clock speed
#define SPI2_CLK_PIN     (18)
#define SPI2_MOSI_PIN    (23)
#define SPI2_MISO_PIN    (19)
#define SPI2_CLOCK_SPEED (1000000)

// SPI2 chip select pins
#define SPI2_ACCEL_CS_PIN (5)
#define SPI2_GYRO_CS_PIN  (21)
#define SPI2_STM32_CS_PIN (22)
