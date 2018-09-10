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
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "driver/i2c.h"

#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "tcpip_adapter.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_smartconfig.h"

#include "oc_api.h"
#include "port/oc_clock.h"
#include "debug_print.h"

#include "ssd1366.h"
#include "font8x8_basic.h"

#define SMART_CONFIG
#define BTN_EVENT 

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static struct timespec ts;
static int quit = 0;
static const char *TAG = "iotivity client";

#define SDA_PIN  GPIO_NUM_23
#define SCL_PIN  GPIO_NUM_22

#define SSD1306_LCDHEIGHT 32

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

static EventGroupHandle_t wifi_event_group;

static const int IPV4_CONNECTED_BIT = BIT0;
static const int IPV6_CONNECTED_BIT = BIT1;

static const int CONNECTED_BIT = BIT2;
static const int ESPTOUCH_DONE_BIT = BIT3;

static const char *RTYPE = "oic.r.tempsensor";
static const char *RTYPE2 = "oic.r.switch.binary";

tcpip_adapter_ip_info_t ip4_info = { 0 };

#ifdef BTN_EVENT
  #define A_BTN   15
  #define B_BTN   32
  #define C_BTN   14

  #define GPIO_INPUT_IO_0     A_BTN // for temp
  #define GPIO_INPUT_IO_1     B_BTN // for sw status
  #define GPIO_INPUT_IO_2     C_BTN // for smartconfig

  #define GPIO_INPUT_PIN_SEL  ((1ULL << GPIO_INPUT_IO_0) | (1ULL << GPIO_INPUT_IO_1))
  #define ESP_INTR_FLAG_DEFAULT 0

  static xQueueHandle gpio_evt_queue = NULL;
  // static bool btn_pressed = false;  
   uint32_t pressed_btn = 0;
#endif

#ifdef SMART_CONFIG
  static bool wifi_config_needed = false;
#endif
  size_t required_size;
  char *ssid;
  char *password;
// #endif

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
static bool switch_state = false;

static oc_event_callback_retval_t
stop_observe(void *data)
{
  (void)data;
  PRINT("Stopping OBSERVE temp\n");
  oc_stop_observe(temp_1, temp_sensor);
  return OC_EVENT_DONE;
}

static void
observe_temp(oc_client_response_t *data)
{
  PRINT("OBSERVE_temp:\n");
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

      oc_do_observe(temp_1, temp_sensor, NULL, &observe_temp, HIGH_QOS, NULL);
      oc_set_delayed_callback(NULL, &stop_observe, 30);
      return OC_STOP_DISCOVERY;
    }
  }
  oc_free_server_endpoints(endpoint);
  return OC_CONTINUE_DISCOVERY;
}


static void
post_switch(oc_client_response_t *data)
{
  PRINT("POST_sw:\n");
  if (data->code == OC_STATUS_CHANGED)
    PRINT("POST response OK\n");
  else
    PRINT("POST response code %d\n", data->code);
}


// activate when the btn c is pressed
static void
observe_switch(oc_client_response_t *data)
{
  PRINT("OBSERVE_sw:\n");
  oc_rep_t *rep = data->payload;
  while (rep != NULL) {
    PRINT("key %s, value ", oc_string(rep->name));
    switch (rep->type) {
    case OC_REP_BOOL:
      PRINT("%d\n", rep->value.boolean);
      switch_state = rep->value.boolean;
      break;
    default:
      break;
    }
    rep = rep->next;
  }

  if (oc_init_post(temp_1, temp_sensor, NULL, &post_switch, LOW_QOS, NULL)) {
    oc_rep_start_root_object();

    oc_rep_set_boolean(root, state, !switch_state);

    oc_rep_end_root_object();
    if (oc_do_post())
      PRINT("Sent POST request\n");
    else
      PRINT("Could not send POST\n");
  } else
    PRINT("Could not init POST\n");
}

static oc_discovery_flags_t
discovery2(const char *anchor, const char *uri, oc_string_array_t types,
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
    if (strlen(t) == 19 && strncmp(t, "oic.r.switch.binary", 19) == 0) {
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

      oc_do_observe(temp_1, temp_sensor, NULL, &observe_switch, HIGH_QOS, NULL);
      oc_set_delayed_callback(NULL, &stop_observe, 30);
      return OC_STOP_DISCOVERY;
    }
  }
  oc_free_server_endpoints(endpoint);
  return OC_CONTINUE_DISCOVERY;
}

static void
issue_requests(void)
{
   oc_do_ip_discovery(RTYPE, &discovery, NULL);
   ESP_LOGI(TAG, "\r\nDiscovery\n");
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
    // tcpip_adapter_ip_info_t ip4_info = { 0 };
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

    TIMERG0.wdt_wprotect=TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed=1;
    TIMERG0.wdt_wprotect=0;
  }

  oc_main_shutdown();
  return 0;
}

// #ifdef SMART_CONFIG
void nvs_read()
{
  nvs_handle my_handle;

  ESP_LOGI(TAG, "nvs_read !!");    

  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
        nvs_get_str(my_handle, "ssid", NULL, &required_size);
        ssid = malloc(required_size);
        err = nvs_get_str(my_handle, "ssid", ssid, &required_size);

        nvs_get_str(my_handle, "password", NULL, &required_size);
        password = malloc(required_size);
        err = nvs_get_str(my_handle, "password", password, &required_size);

        switch (err) {
            case ESP_OK:
                printf("NVS read done\n");
                printf("SSID = %s\n", ssid);
                printf("PW = %s\n", password);
                break;
            case ESP_ERR_NVS_NOT_FOUND:
                printf("The value is not initialized yet!\n");
                break;
            default :
                printf("Error (%s) reading!\n", esp_err_to_name(err));
        }
  }

  nvs_close(my_handle);
}

#ifdef SMART_CONFIG

void nvs_write(char * wifi_ssid, char * wifi_pass)
{
  nvs_handle my_handle;

  esp_err_t err = nvs_open("storage", NVS_READWRITE, &my_handle);
  if (err != ESP_OK) {
      printf("Error (%s) opening NVS handle!\n", esp_err_to_name(err));
  } else {
      err = nvs_set_str(my_handle, "ssid", wifi_ssid);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

      err = nvs_set_str(my_handle, "password", wifi_pass);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");

      // Commit written value.
      // After setting any values, nvs_commit() must be called to ensure changes are written
      // to flash storage. Implementations may write to storage at other times,
      // but this is not guaranteed.
      printf("Committing updates in NVS ... ");
      err = nvs_commit(my_handle);
      printf((err != ESP_OK) ? "Failed!\n" : "Done\n");
  }
  nvs_close(my_handle);
}

static void sc_callback(smartconfig_status_t status, void *pdata)
{
    switch (status) {
        case SC_STATUS_WAIT:
            ESP_LOGI(TAG, "SC_STATUS_WAIT");
            break;
        case SC_STATUS_FIND_CHANNEL:
            ESP_LOGI(TAG, "SC_STATUS_FINDING_CHANNEL");
            break;
        case SC_STATUS_GETTING_SSID_PSWD:
            ESP_LOGI(TAG, "SC_STATUS_GETTING_SSID_PSWD");
            break;
        case SC_STATUS_LINK:
            ESP_LOGI(TAG, "SC_STATUS_LINK");
            wifi_config_t *wifi_config = pdata;
            ESP_LOGI(TAG, "SSID:%s", wifi_config->sta.ssid);
            ESP_LOGI(TAG, "PASSWORD:%s", wifi_config->sta.password); 

            nvs_write(&wifi_config->sta.ssid, &wifi_config->sta.password);

            ESP_ERROR_CHECK( esp_wifi_disconnect() );
            ESP_ERROR_CHECK( esp_wifi_set_config(ESP_IF_WIFI_STA, wifi_config) );
            ESP_ERROR_CHECK( esp_wifi_connect() );
            break;
        case SC_STATUS_LINK_OVER:
            ESP_LOGI(TAG, "SC_STATUS_LINK_OVER");
            if (pdata != NULL) {
                uint8_t phone_ip[4] = { 0 };
                memcpy(phone_ip, (uint8_t* )pdata, 4);
                ESP_LOGI(TAG, "Phone ip: %d.%d.%d.%d\n", phone_ip[0], phone_ip[1], phone_ip[2], phone_ip[3]);
            }
            xEventGroupSetBits(wifi_event_group, ESPTOUCH_DONE_BIT);
            break;
        default:
            break;
    }
}

void smartconfig_task(void * parm)
{
    EventBits_t uxBits;

    ESP_ERROR_CHECK( esp_smartconfig_set_type(SC_TYPE_ESPTOUCH) );
    ESP_ERROR_CHECK( esp_smartconfig_start(sc_callback) );

    while (1) {
        uxBits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false, portMAX_DELAY); 
        if(uxBits & CONNECTED_BIT) {
            ESP_LOGI(TAG, "WiFi Connected to ap");
        }
        if(uxBits & ESPTOUCH_DONE_BIT) {
            ESP_LOGI(TAG, "smartconfig over");
            esp_smartconfig_stop();
            wifi_config_needed = false;
            // reboot
            esp_restart();

            vTaskDelete(NULL);            
        }
    }
}
#endif 


static esp_err_t event_handler(void *ctx, system_event_t *event)
{
    switch (event->event_id) {
    case SYSTEM_EVENT_STA_START:
        #ifdef SMART_CONFIG
          // smart config
          if (wifi_config_needed == true) {
            xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
          } else {
            esp_wifi_connect();
          }
        #else
            esp_wifi_connect();          
        #endif
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

#ifdef SMART_CONFIG

    if (wifi_config_needed == true) {
      // smart_config
      ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );

    } else {
      ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

      wifi_config_t wifi_config = {
          .sta = {
              .ssid = EXAMPLE_WIFI_SSID,
              .password = EXAMPLE_WIFI_PASS,
          },
      };

      nvs_read();

      strncpy((char*) wifi_config.sta.ssid, ssid, strlen(ssid));
      wifi_config.sta.ssid[strlen(ssid)] = '\0';
      strncpy((char*) wifi_config.sta.password, password, strlen(password));
      wifi_config.sta.password[strlen(password)] = '\0';      

      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    }   

#else
      ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));


      wifi_config_t wifi_config = {
          .sta = {
              .ssid = EXAMPLE_WIFI_SSID,
              .password = EXAMPLE_WIFI_PASS,
          },
      };

      nvs_read();

      strncpy((char*) wifi_config.sta.ssid, ssid, strlen(ssid));
      wifi_config.sta.ssid[strlen(ssid)] = '\0';
      strncpy((char*) wifi_config.sta.password, password, strlen(password));
      wifi_config.sta.password[strlen(password)] = '\0';      

      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
#endif    

      ESP_ERROR_CHECK(esp_wifi_start());
}


// oled
void i2c_master_init()
{
  i2c_config_t i2c_config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_PIN,
    .scl_io_num = SCL_PIN,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = 1000000
  };
  i2c_param_config(I2C_NUM_0, &i2c_config);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void ssd1306_init() {
  esp_err_t espRc;

  i2c_cmd_handle_t cmd = i2c_cmd_link_create();

  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
#if 1
  i2c_master_write_byte(cmd, SSD1306_DISPLAYOFF, true);                    // 0xAE
  i2c_master_write_byte(cmd, SSD1306_SETDISPLAYCLOCKDIV, true);            // 0xD5
  i2c_master_write_byte(cmd, 0x80, true);                                  // the suggested ratio 0x80

  i2c_master_write_byte(cmd, SSD1306_SETMULTIPLEX, true);                  // 0xA8
  i2c_master_write_byte(cmd, SSD1306_LCDHEIGHT - 1, true);

  i2c_master_write_byte(cmd, SSD1306_SETDISPLAYOFFSET, true);              // 0xD3
  i2c_master_write_byte(cmd, 0x0, true);                                   // no offset
  i2c_master_write_byte(cmd, SSD1306_SETSTARTLINE | 0x0, true);            // line #0
  i2c_master_write_byte(cmd, SSD1306_CHARGEPUMP, true);                    // 0x8D


  i2c_master_write_byte(cmd, 0x14, true);

  i2c_master_write_byte(cmd, SSD1306_MEMORYMODE, true);                    // 0x20
  i2c_master_write_byte(cmd, 0x00, true);                                  // 0x0 act like ks0108
  i2c_master_write_byte(cmd, SSD1306_SEGREMAP | 0x1, true);
  i2c_master_write_byte(cmd, SSD1306_COMSCANDEC, true);


  i2c_master_write_byte(cmd, SSD1306_SETCOMPINS, true);                    // 0xDA
  i2c_master_write_byte(cmd, 0x02, true);
  i2c_master_write_byte(cmd, SSD1306_SETCONTRAST, true);                   // 0x81
  i2c_master_write_byte(cmd, 0x8F, true);

  i2c_master_write_byte(cmd, SSD1306_SETPRECHARGE, true);                  // 0xd9

i2c_master_write_byte(cmd, 0xF1, true);


  i2c_master_write_byte(cmd, SSD1306_SETVCOMDETECT, true);                 // 0xDB
  i2c_master_write_byte(cmd, 0x40, true);
  i2c_master_write_byte(cmd, SSD1306_DISPLAYALLON_RESUME, true);           // 0xA4
  i2c_master_write_byte(cmd, SSD1306_NORMALDISPLAY, true);                 // 0xA6

  i2c_master_write_byte(cmd, SSD1306_DEACTIVATE_SCROLL, true);

  i2c_master_write_byte(cmd, SSD1306_DISPLAYON, true);//--turn on oled panel  

#else
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_OFF, true);                    // 0xAE  
  i2c_master_write_byte(cmd, OLED_CMD_SET_MUX_RATIO, true);                  // 0xA8
  i2c_master_write_byte(cmd, SSD1306_LCDHEIGHT - 1, true);

  i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true); // 0x8d
  i2c_master_write_byte(cmd, 0x14, true);

  // 0xa1, 0xc8
  i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
  i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

  i2c_master_write_byte(cmd, 0x2E, true); // deactivate scroll 

  // 0xaf
  i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
#endif

  i2c_master_stop(cmd);

  espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
  if (espRc == ESP_OK) {
    ESP_LOGI(TAG, "OLED configured successfully");
  } else {
    ESP_LOGE(TAG, "OLED configuration failed. code: 0x%.2X", espRc);
  }
  i2c_cmd_link_delete(cmd);
}


void task_ssd1306_display_clear(void *ignore) {
  i2c_cmd_handle_t cmd;

  uint8_t zero[128]={0,};
  for (uint8_t i = 0; i < 8; i++) {
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
    i2c_master_write_byte(cmd, 0xB0 | i, true);

    i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
    i2c_master_write(cmd, zero, 128, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
  }

  // vTaskDelete(NULL);
}

void task_ssd1306_display_text(const void *arg_text) {
  char *text = (char*)arg_text;
  uint8_t text_len = strlen(text);

  i2c_cmd_handle_t cmd;

  uint8_t cur_page = 0;

  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

  i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
  i2c_master_write_byte(cmd, 0x00, true); // reset column
  i2c_master_write_byte(cmd, 0x10, true);
  i2c_master_write_byte(cmd, 0xB0 | cur_page, true); // reset page

  i2c_master_stop(cmd);
  i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

  for (uint8_t i = 0; i < text_len; i++) {
    if (text[i] == '\n') {
      cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

      i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);
      i2c_master_write_byte(cmd, 0x00, true); // reset column
      i2c_master_write_byte(cmd, 0x10, true);
      i2c_master_write_byte(cmd, 0xB0 | ++cur_page, true); // increment page

      i2c_master_stop(cmd);
      i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
      i2c_cmd_link_delete(cmd);
    } else {
      cmd = i2c_cmd_link_create();
      i2c_master_start(cmd);
      i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);

      i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
      i2c_master_write(cmd, font8x8_basic_tr[(uint8_t)text[i]], 8, true);

      i2c_master_stop(cmd);
      i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
      i2c_cmd_link_delete(cmd);
    }
  }

  // vTaskDelete(NULL);
}

void oled_task(void *pvParameter)
{
  char  ptr[64];

  vTaskDelay(1000/portTICK_PERIOD_MS);
  // i2c_master_init();
  // ssd1306_init();

  task_ssd1306_display_clear(NULL);
  vTaskDelay(100/portTICK_PERIOD_MS);

  while(1) {
#ifdef SMART_CONFIG
    if (wifi_config_needed == true) {
    sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s", 
        "Smart Config", ip4addr_ntoa(&(ip4_info.ip)), 0, "Off");
    }else {
      sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s", 
        ssid, ip4addr_ntoa(&(ip4_info.ip)), temperature, switch_state?"Off":"On ");
    }
#else      
    sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s",     
      EXAMPLE_WIFI_SSID, ip4addr_ntoa(&(ip4_info.ip)), temperature, switch_state?"Off":"On ");
#endif    
    
    task_ssd1306_display_text((void *)ptr);

    if (pressed_btn == A_BTN) {
      oc_do_ip_discovery(RTYPE, &discovery, NULL);  

      pressed_btn = 0;
    }
    if (pressed_btn == B_BTN) {
      oc_do_ip_discovery(RTYPE2, &discovery2, NULL);  
      pressed_btn = 0;
    }

    vTaskDelay(1000 / portTICK_RATE_MS);
  }

}

#ifdef BTN_EVENT

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void btn_handler(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("\r\nGPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
            pressed_btn = io_num;
        }
    }
}

static void init_btn()
{
    gpio_config_t io_conf;  

    //interrupt of rising edge
    io_conf.intr_type = GPIO_PIN_INTR_POSEDGE; // 
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //start gpio task
    if (xTaskCreate(&btn_handler, "btn_handler", 2048, NULL, 10, NULL) != pdPASS) {
        print_error("task create failed");
    }

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

}
#endif


void app_main(void)
{
    if (nvs_flash_init() != ESP_OK){
        print_error("nvs init failed");
    }

    pthread_cond_init(&cv, NULL);
    print_macro_info();

    i2c_master_init();
    ssd1306_init();

#ifdef SMART_CONFIG
    // wifi config btn
    gpio_pad_select_gpio(GPIO_INPUT_IO_2);
    gpio_set_direction(GPIO_INPUT_IO_2, GPIO_MODE_INPUT);

    if (gpio_get_level(GPIO_INPUT_IO_2) == 0) {
      ESP_LOGI(TAG, "\r\nSmart config start\r\n");
      wifi_config_needed = true;    

      vTaskDelay(1000/portTICK_PERIOD_MS);
    }
#endif
      if (xTaskCreate(&oled_task, "oled_task", 12*1024, NULL, 5, NULL) != pdPASS ) {
            print_error("task create failed");
      }     

    initialise_wifi();

    if (wifi_config_needed == false) {
      init_btn();

      if ( xTaskCreate(&client_main, "client_main", 16*1024, NULL, 5, NULL) != pdPASS ) {
          print_error("task create failed");
      }

      // if (xTaskCreate(&oled_task, "oled_task", 12*1024, NULL, 5, NULL) != pdPASS ) {
      //       print_error("task create failed");
      // }     
    }
    
}
