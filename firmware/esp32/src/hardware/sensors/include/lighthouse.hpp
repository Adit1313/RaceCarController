/*******************************************************************************
 * @file    lighthouse.hpp
 * @brief   Driver for obtaining positioning data from a Lighthouse positioning
 *          deck via UART.
 ******************************************************************************/

#pragma once

#include <cstdint>

#include "sensor_interface.hpp"
#include "uart.hpp"

namespace chronos {

/**
 * @brief Frame containing just the populated data parts from a frame passed by
 *        a Lighthouse positioning deck FPGA.
 * @see LighthouseFrameParser for further documentation on the format.
 */
struct LighthouseRawFrame {
    uint8_t sid : 2;
    uint8_t nPoly : 6;
    uint16_t width : 16;

    uint32_t sync_offset : 17;
    uint8_t padding_1 : 7; ///< Reserved. All zeros.

    uint32_t beam_word : 17;
    uint8_t padding_2 : 7; ///< Reserved. All zeros.

    uint32_t timestamp : 24;
} __attribute__((packed));

/**
 * @brief A processed Lighthouse sweep that results from processing 4 raw
 * lighthouse frames
 */
struct LighthouseSweep {
    float angles[4];
    uint8_t polynomial;
    uint32_t first_timestamp;
    uint32_t sync_timestamp;
};

/**
 * @brief A list of up to 2 Lighthouse sweeps
 */
struct LighthouseSweepList {
    size_t length;
    LighthouseSweep sweeps[5];
};

/**
 * @brief Class operating on a serial byte stream to decode Lighthouse messages.
 *
 * The data format that the parser recognizes is described by the following
 * diagram from <a href="https://github.com/bitcraze/lighthouse-fpga/tree/V6">
 * the Lighthouse receiver GitHub page</a>:
 *
 * @verbatim
    Bit
    0                   1                   2
    0 1 2 3 4 5 6 7 8 9 0 1 2 3 4 5 6 7 8 9 0 1 2 3
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |SID|   nPoly   |          Width                |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |          Sync Offset            |0 0 0 0 0 0 0|
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |          Beam Word              |0 0 0 0 0 0 0|
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
   |                 Timestamp                     |
   +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+
 * @endverbatim
 *
 * The returned structure will be checked for all the following conditions:
 *
 * - Sync frames (2x per second) which consist of [0xFF] x 12 are correctly
 *   aligned.
 * - The padding bits are correctly aligned and always zero.
 *
 * If any of these conditions are not met, the parser will try to re-align in
 * the byte stream.
 */
class Lighthouse : public interface::Sensor<LighthouseSweepList> {
  public:
    Lighthouse(peripheral::UARTPeripheral &uart_peripheral);

    /**
     * @brief Query the parser if it is synchronized to the data currently.
     * @returns True if the parser is synchronized, false if it is trying to
     *          re-synchronize.
     */
    bool is_synchronized() const { return synchronized_; };

    bool sample(LighthouseSweepList &sample);

  private:
    /**
     * @brief Parse all currently available data in the byte stream.
     * @param sweeps If the parsing resulted in any new sweeps being computed,
     *                  they are returned as an output. In that case, the return
     *                  value is true.
     * @returns True if @p sweeps was updated. False if either no new valid
     * sweeps are available or the parser is currently not synchronized with the
     * byte stream.
     */
    bool parse_available(LighthouseSweepList &sweeps);

    /**
     * @brief Parse a full frame if the byte_buffer is filled with a frame.
     *
     * This method should be invoked only if @c synchronized_ is true, and a
     * full frame has been received. In that case, this method will try to parse
     * the complete frame.
     *
     * If a parsing error occurs, this will de-synchronize the parser. If a sync
     * frame is encountered, will return false but not desynchronize the parser.
     *
     * If a correct non-sync frame is parsed, and it is the last frame of a
     * valid sweep the sweep will be added to @p sweeps.
     * @post @c byte_buffer and @c byte_buffer_index will not be modified.
     */
    bool parse_full_frame(LighthouseSweepList &sweeps);

    /** Resets the buffer if either the current message should be discarded or
     * one message was successfully parsed. */
    void reset_buffer();

    /**
     * @brief Parse a sweep if the frame buffer is filled with all the frames of
     * one sweep.
     *
     * This method should be invoked only if the frame buffer is filled with 4
     * frames that belong to one sweep. In that case, this method will try to
     * parse the sweep and clear thr frame buffer.
     *
     * If a complete sweep is parsed, the sweep will be added to @p sweeps.
     * @returns True if @p sweeps was updated.
     */
    bool parse_sweep(LighthouseSweepList &sweeps);

    /** UART byte stream that is polled for bytes. */
    peripheral::UARTPeripheral &uart_peripheral_;

    /** In-progress buffer of message to be decoded. */
    uint8_t byte_buffer_[12];
    /** Index of byte buffer that should be written to next. If >=12, full! */
    uint_fast8_t byte_buffer_index_ = 0;

    /** Parser's current belief whether it is aligned with the byte stream. */
    bool synchronized_ = false;

    /** Buffer to store the last received Lighthouse frames until all frames of
     * a sweep are received. */
    LighthouseRawFrame frame_buffer[4];
    /** The index of the frame buffer where the next frame should be stored. */
    size_t frame_buffer_index = 0;

    /** Angle per 24 MHz timestep since sync event for each base station id. */
    static const float angle_per_timestep[];
};

}; // namespace chronos
