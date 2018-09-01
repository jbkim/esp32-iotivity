/*
// Copyright (c) 2016 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
*/
#include <stdio.h>
#include <pthread.h>
 #include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"

#include "oc_api.h"
#include "port/oc_clock.h"
#include "debug_print.h"
#include "lightbulb.h"

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static struct timespec ts;
static int quit = 0;
static const char *TAG = "iotivity client";

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

static EventGroupHandle_t wifi_event_group;

static const int IPV4_CONNECTED_BIT = BIT0;
static const int IPV6_CONNECTED_BIT = BIT1;

static const char *RTYPE = "oic.r.tempsensor";

static int
app_init(void)
{
  int ret = oc_init_platform("ATEAM Ventures", NULL, NULL);
  // ret |= oc_add_device("/oic/d", "oic.wk.d", "Generic Client", "ocf.1.0.0",
  ret |= oc_add_device("/oic/d", "oic.d.smartmist", "Generic Client", "ocf.1.0.0",    
                       "ocf.res.1.3.0", NULL, NULL);
  return ret;
}

#define MAX_URI_LENGTH (30)
static char temp_1[MAX_URI_LENGTH]; // uri
static oc_endpoint_t *temp_sensor;
static int temperature;
// static bool light_state = false;

static oc_event_callback_retval_t
stop_observe(void *data)
{
  (void)data;
  PRINT("Stopping OBSERVE\n");
  oc_stop_observe(temp_1, temp_sensor);
  return OC_EVENT_DONE;
}

static void
get_temp(oc_client_response_t *data)
{
  //  key temperature, value key units, value key range, value
  
  oc_rep_t *rep = data->payload;
  while (rep != NULL) {
    PRINT("key %s, value ", oc_string(rep->name));
    switch (rep->type) {
    case OC_REP_INT:
      PRINT("%d\n", rep->value.integer);
      temperature = rep->value.integer;
      break;
    default:
      break;
    }
    rep = rep->next;
  }
}

#if 0
static void
observe_mist(oc_client_response_t *data)
{
  PRINT("OBSERVE_mist:\n");
  oc_rep_t *rep = data->payload;
  while (rep != NULL) {
    PRINT("key %s, value ", oc_string(rep->name));
    switch (rep->type) {
    case OC_REP_BOOL:
      PRINT("%d\n", rep->value.boolean);
      // light_state = rep->value.boolean;
      break;
    default:
      break;
    }
    rep = rep->next;
  }

  // prepare send data to server
  int r_value = BULB_STATE_RED;
  int g_value = BULB_STATE_GREEN;
  int b_value = BULB_STATE_BLUE;
  int w_value = BULB_STATE_OTHERS;

  if (oc_init_post(temp_1, temp_sensor, NULL, &get_temp, LOW_QOS, NULL)) {
    oc_rep_start_root_object();

    // oc_rep_set_boolean(root, state, !light_state);
#ifdef ENABLE_LIGHT
    oc_rep_set_int(root, r_value, r_value);
    oc_rep_set_int(root, g_value, g_value);
    oc_rep_set_int(root, b_value, b_value);
    oc_rep_set_int(root, w_value, w_value);
#endif
    // set more param to struct linklist
    oc_rep_end_root_object();

    if (oc_do_post())
      PRINT("Sent POST request\n");
    else
      PRINT("Could not send POST\n");
  } else
    PRINT("Could not init POST\n");
}


static oc_discovery_flags_t
discovery(const char *di, const char *uri, oc_string_array_t types,
          oc_interface_mask_t interfaces, oc_endpoint_t *endpoint,
          oc_resource_properties_t bm, void *user_data)
{
  (void)di;
  (void)interfaces;
  (void)user_data;
  (void)bm;
  int i;

  int uri_len = strlen(uri);
  uri_len = (uri_len >= MAX_URI_LENGTH) ? MAX_URI_LENGTH - 1 : uri_len;

  for (i = 0; i < (int)oc_string_array_get_allocated_size(types); i++) {
    char *t = oc_string_array_get_item(types, i);
    if (strlen(t) == 11 && strncmp(t, RTYPE, 11) == 0) {
      strncpy(temp_1, uri, uri_len);
      temp_1[uri_len] = '\0';
      temp_sensor = endpoint;

      PRINT("Resource %s hosted at endpoints:\n", temp_1);
      oc_endpoint_t *ep = endpoint;
      while (ep != NULL) {
        PRINTipaddr(*ep);
        PRINT("\n");
        ep = ep->next;
      }
      oc_do_observe(temp_1, temp_sensor, NULL, &observe_mist, LOW_QOS, NULL);
      oc_set_delayed_callback(NULL, &stop_observe, 10);
      return OC_STOP_DISCOVERY;
    }
  }

  oc_free_server_endpoints(endpoint);
  return OC_CONTINUE_DISCOVERY;
}


#else

static oc_discovery_flags_t
discovery(const char *anchor, const char *uri, oc_string_array_t types,
          oc_interface_mask_t interfaces, oc_endpoint_t *endpoint,
          oc_resource_properties_t bm, void *user_data)
{
  (void)anchor;
  (void)interfaces;
  (void)user_data;
  (void)bm;

  int i;
  int uri_len = strlen(uri);
  uri_len = (uri_len >= MAX_URI_LENGTH) ? MAX_URI_LENGTH - 1 : uri_len;

  for (i = 0; i < (int)oc_string_array_get_allocated_size(types); i++) {
    char *t = oc_string_array_get_item(types, i);
    if (strlen(t) == 16 && strncmp(t, "oic.r.tempsensor", 16) == 0) {
      temp_sensor = endpoint;
      strncpy(temp_1, uri, uri_len);
      temp_1[uri_len] = '\0';

      PRINT("Resource %s hosted at endpoints:\n", temp_1);
      oc_endpoint_t *ep = endpoint;
      while (ep != NULL) {
        PRINTipaddr(*ep);
        PRINT("\n");
        ep = ep->next;
      }

      oc_do_observe(temp_1, temp_sensor, NULL, &get_temp, HIGH_QOS, NULL);
      oc_set_delayed_callback(NULL, &stop_observe, 30);

      return OC_STOP_DISCOVERY;
    }
  }
  oc_free_server_endpoints(endpoint);
  return OC_CONTINUE_DISCOVERY;
}
#endif

static void
issue_requests(void)
{
  oc_do_ip_discovery(RTYPE, &discovery, NULL);
  ESP_LOGI(TAG, " Discovery !!\n");
}

static void
signal_event_loop(void)
{
  pthread_mutex_lock(&mutex);
  pthread_cond_signal(&cv);
  pthread_mutex_unlock(&mutex);
}

static void
handle_signal(int signal)
{
  (void)signal;
  signal_event_loop();
  quit = 1;
}

static int client_main(void)
{
    int init;
    tcpip_adapter_ip_info_t ip4_info = { 0 };
    struct ip6_addr if_ipaddr_ip6 = { 0 };
    ESP_LOGI(TAG, "iotivity client task started");

    // wait to fetch IPv4 && ipv6 address
#ifdef OC_IPV4
    xEventGroupWaitBits(wifi_event_group, IPV4_CONNECTED_BIT, false, true, portMAX_DELAY);
#else
    xEventGroupWaitBits(wifi_event_group, IPV4_CONNECTED_BIT | IPV6_CONNECTED_BIT, false, true, portMAX_DELAY);
#endif

    if ( tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &ip4_info) != ESP_OK) {
        print_error("get IPv4 address failed");
    } else {
        ESP_LOGI(TAG, "got IPv4 addr:%s", ip4addr_ntoa(&(ip4_info.ip)));
    }

#ifndef OC_IPV4
    if ( tcpip_adapter_get_ip6_linklocal(TCPIP_ADAPTER_IF_STA, &if_ipaddr_ip6) != ESP_OK) {
        print_error("get IPv6 address failed");
    } else {
        ESP_LOGI(TAG, "got IPv6 addr:%s", ip6addr_ntoa(&if_ipaddr_ip6));
    }
#endif

  static const oc_handler_t handler = {.init = app_init,
                                       .signal_event_loop = signal_event_loop,
                                       .requests_entry = issue_requests };

  oc_clock_time_t next_event;

#ifdef OC_SECURITY
  oc_storage_config("./temp_sensor_creds");
#endif /* OC_SECURITY */

  oc_set_con_res_announced(false);
  
  init = oc_main_init(&handler);
  if (init < 0)
    return init;

  while (quit != 1) {
    next_event = oc_main_poll();
    pthread_mutex_lock(&mutex);
    if (next_event == 0) {
      pthread_cond_wait(&cv, &mutex);
    } else {
      ts.tv_sec = (next_event / OC_CLOCK_SECOND);
      ts.tv_nsec = (next_event % OC_CLOCK_SECOND) * 1.e09 / OC_CLOCK_SECOND;
      pthread_cond_timedwait(&cv, &mutex, &ts);
    }
    pthread_mutex_unlock(&mutex);
  }

  oc_main_shutdown();
  return 0;
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        esp_wifi_connect();
        break;

    case SYSTEM_EVENT_STA_GOT_IP:
        xEventGroupSetBits(wifi_event_group, IPV4_CONNECTED_BIT);
        break;

    case SYSTEM_EVENT_STA_DISCONNECTED:
        /* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, IPV4_CONNECTED_BIT);
        xEventGroupClearBits(wifi_event_group, IPV6_CONNECTED_BIT);
        break;

    case SYSTEM_EVENT_STA_CONNECTED:    // ipv4 had connected
#ifndef OC_IPV4
        tcpip_adapter_create_ip6_linklocal(TCPIP_ADAPTER_IF_STA);
#endif
        break;

    case SYSTEM_EVENT_AP_STA_GOT_IP6:
        xEventGroupSetBits(wifi_event_group, IPV6_CONNECTED_BIT);
        break;

    default:
        break;
    }

    return ESP_OK;
}

static void initialise_wifi(void)
{
    tcpip_adapter_init();

    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}


void app_main(void)
{
    if (nvs_flash_init() != ESP_OK){
        print_error("nvs init failed");
    }
    pthread_cond_init(&cv, NULL);
    print_macro_info();
    initialise_wifi();

    if ( xTaskCreate(&client_main, "client_main", 15*1024, NULL, 5, NULL) != pdPASS ) {
        print_error("task create failed");
    }
}
