/*******************************************************************************
 * @file    udp-client.c
 * @brief   Implementation of the UDP client task.
 ******************************************************************************/

#include "udp-client.h"

#include <lwip/netdb.h>
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"
#include "freertos/task.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "nvs-manager.h"

/* Local macros ------------------------------------------------------------- */

#define HOST_IP_ADDR (global_config.host_ip)
#define HOST_PORT    (global_config.host_port)

// Maximum UDP packet size. 508B + 60B IP header + 8B UDP header = 576B
// ("minimum maximum reassembly buffer size")
#define UDP_MAX_SIZE 508

/* Local variables ---------------------------------------------------------- */

/** Logging tag. */
static const char *TAG = "udp-client";
/**  Callback that can be set via @link{udp_client_register_callback} */
static udp_reception_callback callback = NULL;
/** Ring buffer used to store data to be sent out */
static RingbufHandle_t ring_buffer_out = NULL;
/** Continuous timer providing high-frequency interrupts to start UDP poll */
static esp_timer_handle_t poll_timer = NULL;
/** Socket file descriptor used to communicate with the remote host. */
static int sock = -1;
/** Whether @see sock is initialized and ready. */
bool sock_ready = false;

/* Public function implementation ------------------------------------------- */

void udp_client_task(void *pvParameters) {

    ESP_LOGI(TAG, "Entered UDP Client Task!");

    // Preparation: create ring buffer that can hold up to 2kB of packets and
    // headers. Don't split so that on item acquiring it can directly be sent to
    // the socket
    ring_buffer_out = xRingbufferCreate(2048, RINGBUF_TYPE_NOSPLIT);

    // Outer loop: when the connection breaks, keep trying to reconnect.
    while (true) {

        // Open UDP socket (use SOCK_STREAM for TCP experimentation)
        sock = socket(PF_INET, SOCK_DGRAM, 0);

        if (sock < 0) {
            // If a socket can't be opened, the task might as well delete itself
            ESP_LOGE(TAG, "Unable to create socket: errno is %d", errno);
            break;
        }

        // Socket address information is filled in outer loop, does not need to
        // be set on each loop since only one host is connected to a car.
        struct sockaddr_in si;
        memset(&si, 0, sizeof(si));
        si.sin_family = AF_INET;
        si.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        si.sin_port = htons(HOST_PORT);

        // Make socket reusable, so that if the connection breaks, it can be
        // re-opened immediately.
        int reuse = 1;
        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse,
                       sizeof(int)) < 0) {
            ESP_LOGW(TAG, "Could not set socket re-use option!");
        }

        // Set the priority for all packets that are sent to maximum priority
        // see:
        // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/
        // api-guides/wifi.html#esp32-wi-fi-throughput
        const int ip_precedence_vi = 7;
        const int ip_precedence_offset = 5;
        int priority = (ip_precedence_vi << ip_precedence_offset);
        if (setsockopt(sock, IPPROTO_IP, IP_TOS, &priority, sizeof(priority)) <
            0) {
            ESP_LOGW(TAG, "Could not set socket priority option");
        }

        ESP_LOGI(TAG, "Socket created, try to connect to %s:%d", HOST_IP_ADDR,
                 HOST_PORT);

        if (connect(sock, (struct sockaddr *)&si, sizeof(si)) < 0) {
            ESP_LOGE(TAG, "Can't connect to socket: errno is %d (s)", errno);
            // If connect() fails, this isn't too bad. It just means that later
            // calls to send() will potentially be slower.
        }

        sock_ready = true;

        // Now to actually send and receive data!
        size_t send_sz = 0; ///< Number of bytes to send next
        void *send_buf;     ///< Buffer to send next

        // Send phase is entered as soon as data is available in the ring
        // buffer.Skip if there is none.
        while ((send_buf = xRingbufferReceive(ring_buffer_out, &send_sz,
                                              portMAX_DELAY)) != NULL) {

            // new item in queue, now send_buf points into the ring
            // buffer's memory, and we can directly send the data out
            if (send_sz <= UDP_MAX_SIZE) {
                int sent = send(sock, send_buf, send_sz, 0);

                if (sent < 0) {
                    ESP_LOGE(TAG,
                             "Error occurred during sending:"
                             "errno is %d",
                             errno);
                    vRingbufferReturnItem(ring_buffer_out, send_buf);
                    break;
                } else if (sent < send_sz) {
                    // Since the UDP implementation should be able to send
                    // 508B of payload, this "shouldn't" happen.
                    ESP_LOGW(TAG, "Not all bytes sent!");
                } else {
                    ESP_LOGV(TAG, "Sent %dB", send_sz);
                }

            } else {
                ESP_LOGE(TAG, "UDP packet exceeded maximum allowed "
                              "size. Dropping.");
            }

            vRingbufferReturnItem(ring_buffer_out, send_buf);
        }

        // Connection broke or there's an issue with the ring buffer.
        // Usually, the connection needs to be re-established.
        sock_ready = false;

        if (sock != -1) {
            shutdown(sock, 0);
            close(sock);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Goodbye Cruel World
    vTaskDelete(NULL);
}

void udp_client_receive_task(void *pvParameters) {
    // The receive task is not responsible for managing the socket, this is
    // performed by the main task. Instead, it simply waits for the socket to
    // become available and then blocks on receiving data.

    uint8_t rx_buffer[UDP_MAX_SIZE + 1];

    while (true) {
        // While socket is available, block on data receiving.
        while (sock_ready && sock > 0) {
            // Block on receiving data from the socket
            int recv_len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);

            if (recv_len <= 0) {
                // connection broke/was closed, or other error
                ESP_LOGE(TAG, "Receival failed. Errno: %u", errno);
                break;
            } else {
                // we got data! null-terminate and hand over
                rx_buffer[recv_len] = '\0';
                if (callback != NULL) {
                    (*callback)(recv_len, rx_buffer);
                }
            }
        }

        // Wait for 100ms before trying again
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void udp_client_register_callback(udp_reception_callback cb) { callback = cb; }

bool udp_client_send_packet(uint16_t len, const uint8_t *data) {

    if (ring_buffer_out) {
        if (len > 0 && len <= UDP_MAX_SIZE && data != NULL) {

            BaseType_t ret =
                xRingbufferSend(ring_buffer_out, // buffer to send to
                                (void *)data,    // data (will be copied)
                                len,             // size of data
                                0 // non-blocking, so return if full
                );

            if (ret == pdFALSE) {
                ESP_LOGE(TAG, "Could not queue data to send (buffer full!)");
            }

            // convert to bool
            return ret == pdTRUE;

        } else
            return false;
    } else {
        ESP_LOGE(TAG, "Attempted to send item via UDP before ring"
                      " buffer was created!");
        return false;
    }
}
