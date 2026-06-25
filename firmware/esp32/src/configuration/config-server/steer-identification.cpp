/*******************************************************************************
 * @file    steer-identification.cpp
 * @brief   Finds the limits of the potentiometer for the car the PCB is on.
 ******************************************************************************/

#include "steer-identification.h"

#include <stdbool.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "main.hpp"
#include "nvs-manager.h"

/* Local variables ---------------------------------------------------------- */

static const char *TAG = "steer-id";

/**
 * Amount of torque used to drive the wheels left/right. If set too low, the
 * wheels may not move. If set too high, internal bending may give ADC angle
 * estimates that overestimate the actual steering range.
 * Given as a percentage.
 */
#define STEER_IDENTIFICATION_TORQUE (global_config.steer_id_torque)

/* Public function implementation ------------------------------------------- */

bool steer_identification_find_limits(struct PotentiometerLimits *limits) {
    using namespace chronos;

    if (!limits)
        return false;

    // Start steering left continuously, let the angle settle
    steer_motor().set_power(+STEER_IDENTIFICATION_TORQUE);

    vTaskDelay(pdMS_TO_TICKS(1000));

    unsigned long adc_sum = 0;
    ADCMeasurement meas;
    for (unsigned int i = 0; i < 100; i++) {
        steer_feedback().sample(meas);
        adc_sum += (uint16_t)meas;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    limits->upper = adc_sum / (100UL);

    // Start steering right continously, let the angle settle
    steer_motor().set_power(-STEER_IDENTIFICATION_TORQUE);
    vTaskDelay(pdMS_TO_TICKS(1000));

    adc_sum = 0;
    for (unsigned int i = 0; i < 100; i++) {
        steer_feedback().sample(meas);
        adc_sum += (uint16_t)meas;
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    limits->lower = adc_sum / (100UL);
    ESP_LOGI(TAG, "Lower/upper poti limit: %u/%u", limits->lower,
             limits->upper);

    steer_motor().set_power(0);

    return true;
}
