/*******************************************************************************
 * @file    wheel-encoder.cpp
 * @brief   Driver to sample an on-board wheel speed sensor.
 ******************************************************************************/

#include "wheel-encoder.hpp"

#include "esp_log.h"
#include "math-util.h"
#include "spi.hpp"

/** Clock of the STM32 chip that measures the tick differences in [Hz] */
#define WHEEL_SPEED_TIMER_CLOCK (262125.0f)

/* Local variables ---------------------------------------------------------- */

static const char *TAG = "wheel-encoder";

/* Public function implementation ------------------------------------------- */

namespace chronos {

bool WheelEncoder::sample(WheelSpeed &sample) {
    uint8_t bytes[8];
    if (spi_device_.read(0x00, bytes, 8)) {
        uint16_t tick_diffs[4];

        // Data is transmitted by sensor in 4x 2B, MSB first. Re-assemble to 2B.
        for (int i = 0; i < 4; i++) {
            tick_diffs[i] = (bytes[2 * i] << 8) + bytes[2 * i + 1];
        }

        if ((tick_diffs[0] | tick_diffs[1] | tick_diffs[2] | tick_diffs[3]) ==
            0x0000) {
            // If a measurement is 0x00, that wheel has never triggered or is
            // turning at infinite speed. The first case is much more likely,
            // and if it's all four wheels, then the encoders are not present.
            return false;
        }

        // Order of measurements: front right/left, back right/left
        sample.front_right_wheel = tick2rads(tick_diffs[0]);
        sample.front_left_wheel = tick2rads(tick_diffs[1]);
        sample.back_right_wheel = tick2rads(tick_diffs[2]);
        sample.back_left_wheel = tick2rads(tick_diffs[3]);

        ESP_LOGV(TAG, "Front right wheel speed: %f RPM",
                 sample.front_right_wheel);
        return true;
    } else
        return false;
}

float WheelEncoder::tick2rads(uint16_t ticks) {
    return WHEEL_SPEED_TIMER_CLOCK / ticks * 2 * M_PIf / magnet_count_;
}

}; // namespace chronos
