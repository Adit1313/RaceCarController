/*******************************************************************************
 * @file    dc-motor.cpp
 * @brief   Drivers for a DC motor, based on the MCPWM drivers of ESP-IDF
 ******************************************************************************/

#include "dc-motor.hpp"

#include <algorithm>
#include <math.h>
#include <rom/ets_sys.h>
#include <stdbool.h>
#include <stdint.h>

#include "configuration.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"
#include "esp_log.h"
extern "C" {
#include "math-util.h"
}

/* Local macros ------------------------------------------------------------- */

/** Timer unit used to set the duty cycle of the PWM pin. */
#define POWER_TIMER MCPWM_TIMER_0
/** Timer unit used for phase of motor. A PWM unit is abused for this.
 * (Constantly held low or high <=> 0% or 100% duty cycle. )*/
#define PHASE_TIMER MCPWM_TIMER_1
/** For this flag, see the gpio_example of ESP-IDF */
#define ESP_INTR_FLAG_DEFAULT 0

/** Tag used in logging. */
const char *TAG = "dc-motor";

/* Public function implementation ------------------------------------------- */

namespace chronos {

DCMotor::DCMotor(const DCMotorConfig &config) : config_(config) {
    assert(config_.mcpwm_unit < MCPWM_UNIT_MAX);
    assert(config_.mcpwm_timer < MCPWM_TIMER_MAX);

    // Spaghetti code here is a bit unavoidable, since one needs to know the
    // macros associated with each unit.
    mcpwm_io_signals_t pwm_signal_route;
    mcpwm_io_signals_t pha_signal_route;

    switch (config_.mcpwm_unit) {
    case MCPWM_UNIT_0:
        pwm_signal_route = MCPWM0A;
        pha_signal_route = MCPWM0B;
        break;
    case MCPWM_UNIT_1:
        pwm_signal_route = MCPWM1A;
        pha_signal_route = MCPWM1B;
        break;
    default:
        assert(false);
    }

    // Configure the pins

    ESP_ERROR_CHECK(
        mcpwm_gpio_init(config_.mcpwm_unit, pwm_signal_route, config_.pwm_pin));
    ESP_ERROR_CHECK(mcpwm_gpio_init(config_.mcpwm_unit, pha_signal_route,
                                    config_.phase_pin));

    // Configure the PWM frequency and initial conditions
    mcpwm_config_t pwm_config = {
        .frequency = config_.pwm_freq,
        // both PWM channels default to zero duty cycle (phase 0, no power)
        .cmpr_a = 0.0,
        .cmpr_b = 0.0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    ESP_ERROR_CHECK(
        mcpwm_init(config_.mcpwm_unit, config_.mcpwm_timer, &pwm_config));

    // Configure EN/FAULT pin as open drain output
    gpio_config_t en_cfg = {
        .pin_bit_mask = (1ULL << config_.en_fault_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };

    ESP_ERROR_CHECK(gpio_config(&en_cfg));

    // Pull down enable pins to disable motor initialls
    if (config_.fault_callback != NULL) {
        gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
        gpio_isr_handler_add(config_.en_fault_pin,
                             (void (*)(void *))(config_.fault_callback),
                             (void *)config_.mcpwm_unit);
    }
}

bool DCMotor::enable() {
    // To re-enable the motor, just let go of the pin and it will be pulled high
    // by the external pull-up resistor. Since it is open-drain, this lets go.
    gpio_set_level(config_.en_fault_pin, 1);
    esp_rom_delay_us(1);
    gpio_set_intr_type(config_.en_fault_pin, GPIO_INTR_NEGEDGE);
    gpio_intr_enable(config_.en_fault_pin);

    enabled_ = true;

    return true;
}

bool DCMotor::disable() {
    // The EN/FAULT pin is only ever driven LOW, never high. If driving high and
    // the motor driver chip encounters a fault condition, you got yourself a
    // nice short.

#if CONFIG_DISABLE_DC_COASTING
#else
    enabled_ = false;
    gpio_set_intr_type(config_.en_fault_pin, GPIO_INTR_DISABLE);
    gpio_set_level(config_.en_fault_pin, 0);
#endif

    return true;
}

bool DCMotor::set_power(float power) {
    power = clampf(power, -100.0f, 100.0f);

    // the PWM signal for power is on output A of the MCPWM, phase is output B
    mcpwm_set_duty(config_.mcpwm_unit, config_.mcpwm_timer, MCPWM_GEN_A,
                   fabs(power));

    if (config_.reverse_phase)
        power = -power;

    if (power < 0.0f) {
        mcpwm_set_signal_high(config_.mcpwm_unit, config_.mcpwm_timer,
                              MCPWM_GEN_B);
    } else {
        mcpwm_set_signal_low(config_.mcpwm_unit, config_.mcpwm_timer,
                             MCPWM_GEN_B);
    }

    if (config_.auto_coasting) {
        if (fabs(power) <= 0.01 && enabled_)
            disable();
        else if (fabs(power) > 0.01 && !enabled_)
            enable();
    }

    return true;
}

}; // namespace chronos
