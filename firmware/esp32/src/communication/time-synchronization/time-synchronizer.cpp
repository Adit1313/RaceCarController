/*******************************************************************************
 * @file    time-synchronizer.hpp
 * @brief   Synchronization with a local NTP server.
 ******************************************************************************/

#include "time-synchronizer.hpp"

#include <sys/time.h>

extern "C" {
#include "esp_netif.h"
#include "esp_netif_sntp.h"
}
#include "main.hpp"
#include "nvs-manager.h"

/* Private function implementation ----------------------------------------- */

static void time_sync_notification_cb(struct timeval *tv) {
    chronos::time_synchronizer().synchronization_callback(tv);
}

/* TimeSynchronizer implementation -------------------------------------------
 */

namespace chronos {
namespace time {

TimeSynchronizer::TimeSynchronizer() {
    // Start the SNTP client to synchronize the system time.
    // Don't wait for the time sync, as the host might not even be up.
    esp_sntp_config_t sntp_config = {
        .smooth_sync = true,
        .server_from_dhcp = false,
        .wait_for_sync = true,
        .start = true,
        .sync_cb = &time_sync_notification_cb,
        .renew_servers_after_new_IP = false,
        .ip_event_to_renew = IP_EVENT_STA_GOT_IP,
        .index_of_first_server = 0,
        .num_of_servers = 1,
        .servers = {global_config.host_ip},
    };
    esp_netif_sntp_init(&sntp_config);
}

TimeSynchronizer::~TimeSynchronizer() { esp_netif_sntp_deinit(); }

uint64_t TimeSynchronizer::get_time() {
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    int64_t time_us =
        (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
    return time_us;
}

bool TimeSynchronizer::is_synchronized() { return synchronized_; }

void TimeSynchronizer::synchronization_callback(struct timeval *tv) {
    synchronized_ = true;
}

} // namespace time

} // namespace chronos
