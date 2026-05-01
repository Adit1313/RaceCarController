/*******************************************************************************
 * @file    adc.hpp
 * @brief   Configure and read out a built-in ADC peripheral
 ******************************************************************************/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "control-types.h"
#include "driver/adc.h"
#include "sensor_interface.hpp"

/* Public type declarations ------------------------------------------------- */

/** Refers to an ADC on the microcontroller where multiple might be available.*/
enum ADCPeripheral {
    ADCPeripheralInvalid = 0, ///< enum not set
    ADCPeripheral1 = 1,       ///< ADC 1
    ADCPeripheral2 = 2,       ///< ADC 2
};

/**
 * @brief Configuration for an ADC channel to take readings from.
 */
struct ADCChannel {
    enum ADCPeripheral adc;
    /** Holds the channel number. If adc == 1, channel1 should be set, if adc ==
     * 2, channel2. */
    union {
        adc1_channel_t channel1;
        adc2_channel_t channel2;
    };
    /** How much attenuation should be used on this ADC configuration. See:
     * https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html#adc-attenuation
     */
    adc_atten_t attenuation;
};

namespace chronos {

/**
 * @brief Analog-to-digital converter sensor interface.
 */
class ADCSensor : public interface::Sensor<ADCMeasurement> {
  public:
    ADCSensor(const struct ADCChannel &adc_channel);
    bool sample(ADCMeasurement &sample);

  private:
    struct ADCChannel adc_channel_;
};

}; // namespace chronos

/* Public function declarations --------------------------------------------- */

/**
 * @brief Set up the ADC channel API.
 *
 * This sets up the mutexes that ensure thread safety. Then it also fetches the
 * calibration values for the two ADCs from the eFuse (if present) and prepares
 * the calibration curves.
 */
void adc_setup(void);

/**
 * @brief Configure an ADC channel so that one can take readings from it.
 * The attenuation settings will be set.
 * @param chn The channel to configure.
 * @note adc_setup() needs to be called beforehand, otherwise this fails.
 * @returns true if the setup succeeded, false otherwise.
 */
bool adc_channel_configure(struct ADCChannel *chn);

/**
 * @brief Get the discrete voltage reading from the ADC pin.
 *
 * The discrete result is given in mV, in a range depending on the attenuation
 * setting.
 * @note Since the ADC peripherals are protected by mutexes, this may block task
 * execution for an arbitrarily long time!
 * @pre adc_channel_configure() must have been called prior to this.
 * @note adc_setup() needs to be called beforehand, otherwise this fails.
 * @returns A 12-bit ADC reading from the ADC channel in the range [0, 2500].
 */
uint16_t adc_channel_sample(struct ADCChannel *chn);
