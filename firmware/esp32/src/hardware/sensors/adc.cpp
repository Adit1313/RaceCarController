/*******************************************************************************
 * @file    adc.cpp
 * @brief   Configure and read out a built-in ADC peripheral
 ******************************************************************************/

#include "adc.hpp"

#include <stdint.h>

#include "configuration.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* Local macros ------------------------------------------------------------- */

/** The default reference voltage that the ADC input is compared against.
        Normally not used since there is a more accurate value in the eFuse. */
#define DEFAULT_VREF 1100

/* Local variables ---------------------------------------------------------- */

namespace chronos {
ADCSensor::ADCSensor(const struct ADCChannel &adc_channel)
    : adc_channel_(adc_channel) {
    adc_channel_configure(&adc_channel_);
};

bool ADCSensor::sample(ADCMeasurement &sample) {
    sample = adc_channel_sample(&adc_channel_);
    return true;
}
} // namespace chronos

/** Stores the characteristics of the ADC calibration: whether the reference
 * voltage from the eFuse is used or some other. */
static esp_adc_cal_characteristics_t *adc_chars_1 = NULL;
static esp_adc_cal_characteristics_t *adc_chars_2 = NULL;

/** Tag used in logging. */
static const char *TAG = "adc";

/** Mutexes to stop two threads from accessing the ADC at the same time. */
SemaphoreHandle_t adc1_mutex = NULL;
SemaphoreHandle_t adc2_mutex = NULL;

/** Statically allocated memory for the two ADC mutexes. */
StaticSemaphore_t adc1_mutex_buffer;
StaticSemaphore_t adc2_mutex_buffer;

/* Public function implementation ------------------------------------------- */

void adc_setup(void) {
    // Create handles for the mutexes. Cannot fail since the memory has been
    // statically allocated.
    adc1_mutex = xSemaphoreCreateMutexStatic(&adc1_mutex_buffer);
    adc2_mutex = xSemaphoreCreateMutexStatic(&adc2_mutex_buffer);

    // Determine the reference voltage (maybe it's in the eFuse, maybe not)
    // to use during conversion from raw to the actual voltage

    // ADC1
    adc_chars_1 = (esp_adc_cal_characteristics_t *)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type1 =
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                                 DEFAULT_VREF, adc_chars_1);

    /// ADC2
    adc_chars_2 = (esp_adc_cal_characteristics_t *)calloc(
        1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type2 =
        esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
                                 DEFAULT_VREF, adc_chars_2);

    // Configure bit width of ADC1. ADC2 has to be set on every sample.
    adc1_config_width((adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT);

    switch (val_type1) {
    case ESP_ADC_CAL_VAL_EFUSE_VREF:
        ESP_LOGI(TAG, "VREF is in fuse for ADC 1.");
        break;
    case ESP_ADC_CAL_VAL_EFUSE_TP:
        ESP_LOGI(TAG, "Two point VREFs in fuse for ADC1.");
        break;
    default:
        ESP_LOGI(TAG, "No VREF in fuse for ADC1, using default.");
        break;
    }

    switch (val_type2) {
    case ESP_ADC_CAL_VAL_EFUSE_VREF:
        ESP_LOGI(TAG, "VREF is in fuse for ADC 2.");
        break;
    case ESP_ADC_CAL_VAL_EFUSE_TP:
        ESP_LOGI(TAG, "Two point VREFs in fuse for ADC2.");
        break;
    default:
        ESP_LOGI(TAG, "No VREF in fuse for ADC2, using default.");
        break;
    }
}

bool adc_channel_configure(struct ADCChannel *chn) {
    if (!chn)
        return false;

    // If the mutexes are not set up, we cannot guarantee thread safety. Return
    // false to indicate invalid use of API.
    if (!adc1_mutex) {
        return false;
    }

    // taskEXIT_CRITICAL(&mux);

    esp_err_t ret;
    // Set up the channel with the specs from the struct.
    switch (chn->adc) {
    case ADCPeripheral1:
        ret = adc1_config_channel_atten(chn->channel1, chn->attenuation);
        break;
    case ADCPeripheral2:
        ret = adc2_config_channel_atten(chn->channel2, chn->attenuation);
        break;
    default:
        return false;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setup of ADC failed. Reason: %s", esp_err_to_name(ret));
        return false;
    }

    return true;
}

uint16_t adc_channel_sample(struct ADCChannel *chn) {
    if (!chn || !adc1_mutex)
        return 0;

    uint_fast32_t sum = 0;
    int adc_reading;
    uint_fast16_t voltage;

    SemaphoreHandle_t sem_to_take = NULL;
    esp_adc_cal_characteristics_t *adc_chars = NULL;

    switch (chn->adc) {
    case ADCPeripheral1:
        sem_to_take = adc1_mutex;
        adc_chars = adc_chars_1;
        break;
    case ADCPeripheral2:
        sem_to_take = adc2_mutex;
        adc_chars = adc_chars_2;
        break;
    default:
        return 0;
    }

    xSemaphoreTake(sem_to_take, portMAX_DELAY);

    for (uint_fast8_t i = 0; i < CONFIG_ADC_MULTISAMPLING_BATCH; i++) {
        if (chn->adc == ADCPeripheral1) {
            adc_reading = adc1_get_raw(chn->channel1);
        } else {
            adc2_config_channel_atten(chn->channel2, chn->attenuation);
            esp_err_t ret = adc_reading = adc2_get_raw(
                chn->channel2, (adc_bits_width_t)ADC_WIDTH_BIT_DEFAULT,
                &adc_reading);

            if (ret != ESP_OK) {
                // Probably Wi-Fi is running
                xSemaphoreGive(sem_to_take);
                return 0;
            }
        }

        voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        sum += voltage;
    }

    xSemaphoreGive(sem_to_take);
    return (uint16_t)(sum / CONFIG_ADC_MULTISAMPLING_BATCH);
}
