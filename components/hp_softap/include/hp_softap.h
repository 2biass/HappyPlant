#ifndef _HP_SOFTAP
#define _HP_SOFTAP

#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_log.h"

#define SOFTAP_SSID             "HappyPlant"
#define SOFTAP_PASS             "adminjensen"
#define SOFTAP_WIFI_CHANNEL     1 /* Physical wifi-channel (1-13) */
#define SOFTAP_MAX_STA_CONN     2 /* Maximal Station-connections */

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data);

void wifi_init_softap(void);

#endif