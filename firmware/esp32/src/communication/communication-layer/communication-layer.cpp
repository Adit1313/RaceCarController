/**
 *******************************************************************************
 * @file    communication-layer.c
 * @brief   Set of callbacks that mediate between the control loop and the
 *          network layer.
 *******************************************************************************
 */

#include "communication-layer.h"

#include <pb_decode.h>
#include <pb_encode.h>
#include <string.h>

#include "control.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "main.hpp"

extern "C" {
#include "udp-client.h"
}

/* Private function declaration --------------------------------------------- */

/**
 * @brief Quickly and efficiently handle data received via UDP.
 *
 * This parses the incoming data into a packet struct and then decides which
 * action to take depending on the packet type.
 * @note This callback is called on Core 0, notabene on the runtime of the UDP
 *  client task, and has to be kept short.
 */
void handle_incoming_datagram(uint16_t len, const uint8_t *data);

/* Private variable declaration --------------------------------------------- */

/** Tag used in logging. */
static const char *TAG = "comm-layer";

static chronos::ControlLoop *control_loop_ = nullptr;

/* Private function declaration --------------------------------------------- */

/**
 * @brief Sends a discovery packet out to the host upon a timer firing.
 *
 * This discovery packet is needed because the CRS host needs to first receive
 * a datagram from the car before sending its commands to the IP/port it
 * receives traffic from. If the discovery packet is missing, the embedded
 * system can't pair to the host.
 * @param t The timer that fired (will be 100% ignored)
 */
void send_discovery_packet(TimerHandle_t t);

/* Public function implementation ------------------------------------------- */

void communication_layer_setup(chronos::ControlLoop *control_loop) {
    // Register as callback for incoming UDP datagrams
    udp_client_register_callback(&handle_incoming_datagram);
    if (control_loop)
        control_loop_ = control_loop;

    // Create and start timer to send out periodic discovery packets
    TimerHandle_t discovery_timer;
    if ((discovery_timer =
             xTimerCreate("discovery-timer", 5000 / portTICK_PERIOD_MS, pdTRUE,
                          NULL, &send_discovery_packet)) != NULL) {
        if (xTimerStart(discovery_timer, 0) == pdPASS) {
            ESP_LOGI(TAG, "Started discovery timer...");
            return;
        }
    }
    ESP_LOGE(TAG, "Failed to start discovery timer!");
}

/* Private function implementation ------------------------------------------ */

void communication_layer_send(Packet *p) {
    if (!p)
        return;

    // Buffer where the serialized data goes
    uint8_t buffer[255];

    // Timestamp packet with current time. Set the `has_timestamp` field to
    // true, since the field is optional and will otherwise not be encoded.
    if (chronos::time_synchronizer().is_synchronized()) {
        p->has_timestamp = true;
        p->timestamp = chronos::time_synchronizer().get_time();
    }

    // Create a stream handle from the buffer, and encode the data into the
    // buffer
    pb_ostream_t ostream =
        pb_ostream_from_buffer((uint_least8_t *)&buffer, sizeof(buffer));
    bool status = pb_encode(&ostream, Packet_fields, p);

    if (!status) {
        // encoding successful
        ESP_LOGE(TAG, "Failed to encode packet. Reason: %s",
                 PB_GET_ERROR(&ostream));
        return;
    }

    udp_client_send_packet(ostream.bytes_written, buffer);
}

/* Private function implementation ------------------------------------------ */

void handle_incoming_datagram(uint16_t len, const uint8_t *data) {
    if (len < 2) {
        // insufficient length to gather type and payload size, reject
        ESP_LOGE(TAG, "Incoming datagram has insufficient length (<2).");
        return; // packet is dropped by UDP client
    }

    Packet p = Packet_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(data, len);

    bool status = pb_decode(&istream, Packet_fields, &p);
    if (!status) {
        ESP_LOGE(TAG, "Failed to decode message. Reason: %s",
                 PB_GET_ERROR(&istream));
        return; // packet is dropped by UDP client
    }

    switch (p.which_contents) {
    case Packet_ping_tag:
        if (p.contents.ping.should_bounce) {
            p.contents.ping.should_bounce = false;
            communication_layer_send(&p);
        }
        break;
    case Packet_control_input_tag: {
        struct Reference new_ref {};

        // Legacy reference type
        new_ref.throttle_ref.throttle = p.contents.control_input.torque_ref;
        new_ref.throttle_ref.type = ThrottleReference::Throttle;

        if (p.contents.control_input.has_steer_input) {
            // Newer version of message format that uses the new oneof format
            if (p.contents.control_input.steer_input.which_input ==
                SteerInput_steer_angle_tag) {
                new_ref.steer_ref.type = SteeringReference::SteeringAngle;
                new_ref.steer_ref.steer_angle =
                    p.contents.control_input.steer_input.input.steer_angle;
            } else {
                new_ref.steer_ref.type = SteeringReference::Voltage;
                new_ref.steer_ref.steer_voltage =
                    p.contents.control_input.steer_input.input.steer_voltage;
            }
        } else {
            // Fallback to the old version
            new_ref.steer_ref.type = SteeringReference::SteeringAngle;
            new_ref.steer_ref.steer_angle = p.contents.control_input.steer_ref;
        }

        if (control_loop_)
            control_loop_->update_reference(new_ref);

        break;
    }
    case Packet_reference_tag: {
        struct Reference new_ref {};
        CarReference &ref_packet = p.contents.reference;

        // Parse throttle actuator reference.
        switch (ref_packet.which_throttle_reference) {
        case CarReference_throttle_tag:
            new_ref.throttle_ref.type = ThrottleReference::Throttle;
            new_ref.throttle_ref.throttle =
                ref_packet.throttle_reference.throttle;
            break;
        case CarReference_longitudinal_velocity_tag:
            new_ref.throttle_ref.type = ThrottleReference::LongitudinalVelocity;
            new_ref.throttle_ref.longitudinal_velocity =
                ref_packet.throttle_reference.longitudinal_velocity;
            break;
        default:
            ESP_LOGE(TAG, "Unknown throttle reference type!");
            break;
        }

        // Parse steering actuator reference.
        switch (ref_packet.which_steering_reference) {
        case CarReference_angle_tag:
            new_ref.steer_ref.type = SteeringReference::SteeringAngle;
            new_ref.steer_ref.steer_angle = ref_packet.steering_reference.angle;
            break;
        case CarReference_voltage_tag:
            new_ref.steer_ref.type = SteeringReference::Voltage;
            new_ref.steer_ref.steer_voltage =
                ref_packet.steering_reference.voltage;
            break;
        default:
            ESP_LOGE(TAG, "Unknown steering reference type!");
            break;
        }

        if (control_loop_)
            control_loop_->update_reference(new_ref);

        break;
    }
    default:
        ESP_LOGW(TAG, "Received unknown packet type. Dropping...");
        break;
    }

    return;
}

void send_discovery_packet(TimerHandle_t t) {
    ESP_LOGI(TAG, "Sending a discovery beacon packet...");

    Packet p = Packet_init_zero;
    p.which_contents = Packet_ping_tag;
    p.contents.ping.should_bounce = true;

    communication_layer_send(&p);
}
