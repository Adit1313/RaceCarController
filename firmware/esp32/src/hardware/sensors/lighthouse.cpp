/*******************************************************************************
 * @file    lighthouse.cpp
 * @brief   Driver for obtaining positioning data from a Lighthouse positioning
 *          deck via UART.
 ******************************************************************************/

#include "lighthouse.hpp"

#include <math.h>
#include <string.h>

#include "esp_log.h"
#include "uart.hpp"

static const char *TAG = "lighthouse";

/* Lighthouse Driver -------------------------------------------------------- */

namespace chronos {

// Values are from
// https://github.com/bitcraze/crazyflie-firmware/blob/master/src/utils/src/lighthouse/pulse_processor_v2.c
const float Lighthouse::angle_per_timestep[] = {
    2 * M_PI / 479500, 2 * M_PI / 478500, 2 * M_PI / 476500, 2 * M_PI / 474500,
    2 * M_PI / 473500, 2 * M_PI / 471500, 2 * M_PI / 470500, 2 * M_PI / 469500,
    2 * M_PI / 468500, 2 * M_PI / 464500, 2 * M_PI / 459500, 2 * M_PI / 455500,
    2 * M_PI / 453500, 2 * M_PI / 450500, 2 * M_PI / 446500, 2 * M_PI / 443500};

Lighthouse::Lighthouse(peripheral::UARTPeripheral &uart_peripheral)
    : uart_peripheral_(uart_peripheral), byte_buffer_() {

    // Attempt to boot the lighthouse FPGA deck into application mode
    uart_peripheral_.set_baudrate(115200);

    const uint8_t boot_command[] = {
        0x00, 0x00, // UART break to sync device
        0xBC, 0x02, // protocol_reset(), get_version()
        0x00        // boot()
    };

    uart_peripheral.send(boot_command, sizeof(boot_command));

    // On the boot() byte being clocked out, the FPGA will send its boot version
    uart_peripheral_.wait_tx_done();
    uint8_t version = 0x00;
    uart_peripheral_.receive(&version, 1, 1);

    ESP_LOGI(TAG, "Lighthouse bootloader version: %x", version);

    // Done, ready to receive and start parsing. In application mode, the
    // device operates at double the baudrate.
    uart_peripheral_.set_baudrate(230400);
    uart_peripheral_.clear_rx_buffer();
}

bool Lighthouse::sample(LighthouseSweepList &sample) {
    sample.length = 0;
    return parse_available(sample);
}

bool Lighthouse::parse_available(LighthouseSweepList &sweeps) {
    // This variable keeps track of whether one sweep was successfully parsed,
    // ultimately forming the return value of the method.
    bool new_sweep_parsed = false;

    while (uart_peripheral_.available() > 0) {

        if (!synchronized_) {
            // Parse exactly one byte, which is less performant but allows
            // exactly finding the sequence of 12x 0xFF to sync.

            if (byte_buffer_index_ < 12) {
                uint8_t byte;
                uart_peripheral_.receive(&byte, 1, 1);

                if (byte == 0xFF) {
                    byte_buffer_[byte_buffer_index_++] = byte;
                } else {
                    reset_buffer();
                }
            }

            if (byte_buffer_index_ >= 12) {
                // Double check that everything is 0xFF
                synchronized_ = true;
                for (uint8_t i = 0; i < 12; ++i) {
                    if (byte_buffer_[i] != 0xFF) {
                        // Back to square one
                        synchronized_ = false;
                        reset_buffer();
                        break;
                    }
                }
            }
        }

        if (synchronized_) {
            int current_frame_missing = 12 - byte_buffer_index_;

            if (current_frame_missing > 0) {
                size_t read_bytes = uart_peripheral_.receive(
                    byte_buffer_ + byte_buffer_index_, current_frame_missing,
                    current_frame_missing);

                // Guaranteed not to underflow into +uint8_max since the return
                // value is checked for negative error codes
                byte_buffer_index_ += read_bytes;
            }

            if (byte_buffer_index_ >= 12) {
                // Full frame! Parse it. If it isn't valid, the method will set
                // synchronized_ = false. If it's a sync frame, it will be
                // ignored, return false but not unsync.
                if (parse_full_frame(sweeps)) {
                    new_sweep_parsed = true;
                }

                reset_buffer();
            }
        }
    }

    return new_sweep_parsed;
}

bool Lighthouse::parse_full_frame(LighthouseSweepList &sweeps) {
    // This variable keeps track of whether one sweep was successfully parsed,
    // ultimately forming the return value of the method.
    bool new_sweep_parsed = false;

    // Check invariants used in this method to parse the frame.
    // (1) The packed LighthouseRawFrame struct must be bit-to-bit compatible
    //     with the UART messages sent by the FPGA => 12B long
    // (2) All integers extracted from the bitfield must be little-endian, as
    //     they are sent.
    static_assert(sizeof(LighthouseRawFrame) == 12);
    static_assert(__BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__);

    // Invariant violated.
    if (!synchronized_) {
        return false;
    }

    bool is_sync_frame = true;
    for (uint_fast8_t i = 0; i < 12; ++i) {
        if (byte_buffer_[i] != 0xFF) {
            is_sync_frame = false;
            break;
        }
    }

    if (is_sync_frame) {
        // No data frame – but also isn't invalid. Don't change synchronized_.
        return false;
    }

    LighthouseRawFrame frame_local;
    memcpy((void *)&frame_local, byte_buffer_, sizeof(LighthouseRawFrame));

    // Check if invariants are satisfied
    if (frame_local.padding_1 != 0 || frame_local.padding_2 != 0) {
        synchronized_ = false;
        return false;
    }

    // Correct frame, copy into buffer
    if (frame_buffer_index == 0) {
        // First frame in frame buffer
        frame_buffer[0] = frame_local;
        frame_buffer_index++;
    } else {
        // Check if frame belongs to sweep that is currently in frame buffer.
        // Max time from first to last sensor hit:
        //  - max distance between sensors = 33.6 mm
        //  - at 1m from basestation that is a time difference of 33.6 mm / (2 *
        //  pi * 1000 mm * 50 Hz) = 106 us
        //  - at a 24 MHz clock that are 2567 cycles ~ 2600
        //  - it is possible to recive the frames out of order if the timestamps
        //  are verry close to each other (< 100 cycles)
        uint32_t first_frame_time = frame_buffer[0].timestamp;
        uint32_t current_frame_time = frame_local.timestamp + 100;
        current_frame_time &= 0x00FFFFFF; // 24bit
        uint32_t time_diff = current_frame_time - first_frame_time;
        time_diff &= 0x00FFFFFF; // 24bit
        if (time_diff < 2600 + 100) {
            // Frame belongs to sweep
            frame_buffer[frame_buffer_index] = frame_local;
            frame_buffer_index++;
            // Parse sweep if this is the final frame of the sweep
            if (frame_buffer_index == 4) {
                if (parse_sweep(sweeps)) {
                    new_sweep_parsed = true;
                }
            }
        } else {
            // Discard frame buffer and start to collect frames for a new sweep
            frame_buffer[0] = frame_local;
            frame_buffer_index = 1;
        }
    }
    return new_sweep_parsed;
}

bool Lighthouse::parse_sweep(LighthouseSweepList &sweeps) {
    if (frame_buffer_index != 4) {
        // Not the right number of frames in the frame buffer, skipping sweep
        frame_buffer_index = 0;
        return false;
    }
    frame_buffer_index = 0;

    if (sweeps.length >= 5) {
        // Sweep List already full, skipping sweep
        return false;
    }

    // Sort frames by sensor id
    LighthouseRawFrame *sensor_frame[4] = {nullptr, nullptr, nullptr, nullptr};
    for (size_t i = 0; i < 4; i++) {
        uint8_t sensor_id = frame_buffer[i].sid;
        sensor_frame[sensor_id] = &frame_buffer[i];
    }

    // Check if all sensor ids are present
    for (size_t i = 0; i < 4; i++) {
        if (sensor_frame[i] == nullptr) {
            // Sensor id is missing in sweep
            return false;
        }
    }

    // Find valid frame with sync offset
    /* The frame format the FPGA lighthouse board outputs has some quirks that
     * make it quite hard to gather all necessary data. If the time difference
     * between tow consecutive sensor hits is small enough the bord uses the
     * info to calculate the polynomial and sync offset. If it can calculate the
     * sync offset the offset and polynomial get outputted together with the
     * second frame. Because of that the first frame of a sweep never has a
     * valid polynomial or sync offset. Furthermore, the sync offset is only
     * outputted for the first frame for which the polynomial was calculated. If
     * two sensors are hit nearly at the same time because they line up with one
     * of the laser beams the board isn’t able to calculate the polynomial
     * properly and outputs nonsense. Therefore, it is possible to have frames
     * with different polynomials in the same sweep. To resolve this  you have
     * to search for the two consecutive frames in the sweep with the biggest
     * time difference and determine the corresponding polynomial and sync
     * offset.
     */
    LighthouseRawFrame *ref_frame = nullptr;
    int32_t max_time_diff = 0;
    const int32_t wrap_timestamp = 0x01000000; // 2^24
    for (size_t i = 1; i < 4; i++) {
        if (frame_buffer[i].nPoly == 63) {
            // Invalid polynomial in frame
            continue;
        }

        // calculate time difference between frames in 24 bit centered around 0
        int32_t start_timestamp = frame_buffer[i - 1].timestamp;
        int32_t end_timestamp = frame_buffer[i].timestamp;
        int32_t time_diff = end_timestamp - start_timestamp;
        if (time_diff < -wrap_timestamp / 2) {
            time_diff += wrap_timestamp;
        } else if (time_diff > wrap_timestamp / 2) {
            time_diff -= wrap_timestamp;
        }

        if (frame_buffer[i].sync_offset != 0) {
            if (time_diff > max_time_diff) {
                max_time_diff = time_diff;
                ref_frame = &frame_buffer[i];
            }
        } else {
            if (time_diff > max_time_diff) {
                max_time_diff = time_diff;
            }
        }
    }

    if (ref_frame == nullptr) {
        // No valid frame with sync offset was found
        return false;
    }

    // Calculate timestep of sync event. sync offset in 6 MHz clock, timestamp
    // in 24 MHz clock
    int32_t sync_timestamp = ref_frame->timestamp;
    sync_timestamp -= ref_frame->sync_offset * 4;
    if (sync_timestamp < 0) {
        sync_timestamp += wrap_timestamp;
    }

    // Calculate base station id: bit (2..6) of polynomial
    uint8_t base_station_id = ref_frame->nPoly >> 1;

    // Populate sweep data
    sweeps.sweeps[sweeps.length].polynomial = ref_frame->nPoly;
    sweeps.sweeps[sweeps.length].first_timestamp = frame_buffer[0].timestamp;
    sweeps.sweeps[sweeps.length].sync_timestamp = sync_timestamp;
    // Calculate angles
    for (size_t i = 0; i < 4; i++) {
        int32_t center_timestamp =
            sensor_frame[i]->timestamp + sensor_frame[i]->width / 2;
        int32_t time_diff = center_timestamp - sync_timestamp;
        if (time_diff < 0) {
            time_diff += wrap_timestamp;
        }
        sweeps.sweeps[sweeps.length].angles[i] =
            time_diff * angle_per_timestep[base_station_id];
    }

    sweeps.length++;
    return true;
}

void Lighthouse::reset_buffer() {
    memset(byte_buffer_, 0, sizeof(byte_buffer_));
    byte_buffer_index_ = 0;
}

}; // namespace chronos
