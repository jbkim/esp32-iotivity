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

#include <pthread.h>
#include <stdio.h>
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
#include "esp_event_loop.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_smartconfig.h"

#include "oc_api.h"
#include "port/oc_clock.h"

#include "dht.h"
#include "ssd1366.h"
#include "font8x8_basic.h"

#define SDA_PIN     GPIO_NUM_23
#define SCL_PIN     GPIO_NUM_22
#define DHT_GPIO    GPIO_NUM_27
#define RELAY_GPIO  GPIO_NUM_19
#define RELAY_IN_GPIO GPIO_NUM_18

#define SSD1306_LCDHEIGHT 32

#include "debug_print.h"

#define EXAMPLE_WIFI_SSID CONFIG_WIFI_SSID
#define EXAMPLE_WIFI_PASS CONFIG_WIFI_PASSWORD

static EventGroupHandle_t wifi_event_group;

static const int IPV4_CONNECTED_BIT = BIT0;
static const int IPV6_CONNECTED_BIT = BIT1;

static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv = PTHREAD_COND_INITIALIZER;
static struct timespec ts;
static int quit = 0;

tcpip_adapter_ip_info_t ip4_info = { 0 };

#define SMART_CONFIG 
#ifdef SMART_CONFIG

static const int CONNECTED_BIT = BIT2;
static const int ESPTOUCH_DONE_BIT = BIT3;

  #define C_BTN   14
  #define GPIO_INPUT_IO_2     C_BTN // for smartconfig

  static bool wifi_config_needed = false;
  size_t required_size;
  char *ssid;
  char *password;  
#endif


int temp;
bool switch_state = false;

static const char *TAG = "iotivity server";

static int
app_init(void)
{
  int err = oc_init_platform("ATEAM Ventures", NULL, NULL);

  err |= oc_add_device("/oic/d", "oic.d.switch", "Temp_sensor", "ocf.1.0.0",
                       "ocf.res.1.3.0", NULL, NULL);
  return err;
}

static void
get_temp(oc_request_t *request, oc_interface_mask_t interface, void *user_data)
{
  (void)user_data;
  PRINT("GET_temp: ");

  oc_rep_start_root_object();
  switch (interface) {
  case OC_IF_BASELINE:
    oc_process_baseline_interface(request->resource);
    // oc_rep_set_text_string(root, id, "home_thermostat");
  /* fall through */
  case OC_IF_R:
    oc_rep_set_int(root, temperature, temp);
  default:
    break;
  }

  oc_rep_end_root_object();
  oc_send_response(request, OC_STATUS_OK);
  PRINT("Temp = %d C\n", temp);
}

#if 0
static void
post_temp(oc_request_t *request, oc_interface_mask_t interface, void *user_data)
{
  (void)interface;
  (void)user_data;
  PRINT("POST_temp:\n");
  bool out_of_range = false;
  double temp = -1;

  oc_rep_t *rep = request->request_payload;
  while (rep != NULL) {
    PRINT("key: %s ", oc_string(rep->name));
    switch (rep->type) {
    case OC_REP_DOUBLE:
      temp = rep->value.double_p;
      PRINT("value: %d\n", temp);
      break;
    default:
      break;
    }
    rep = rep->next;
  }

  oc_send_response(request, OC_STATUS_CHANGED);
  //TODO 
  // light_state = state;

}
#endif

static void
get_switch(oc_request_t *request, oc_interface_mask_t interface,
           void *user_data)
{
  (void)user_data;
  PRINT("GET_switch:\n");
  oc_rep_start_root_object();
  switch (interface) {
  case OC_IF_BASELINE:
    oc_process_baseline_interface(request->resource);
    oc_rep_set_text_string(root, id, "mist_switch");
  /* fall through */
  case OC_IF_A:
    oc_rep_set_boolean(root, value, switch_state);
    break;
  default:
    break;
  }
  oc_rep_end_root_object();

  oc_send_response(request, OC_STATUS_OK);
}

static void
post_switch(oc_request_t *request, oc_interface_mask_t interface,
            void *user_data)
{
  (void)interface;
  (void)user_data;
  PRINT("POST_switch:\n");
  bool state = false, bad_request = false;
  oc_rep_t *rep = request->request_payload;
  while (rep != NULL) {
    switch (rep->type) {
    case OC_REP_BOOL:
      state = rep->value.boolean;     
      switch_state = state;
      PRINT("switch_state = %s\n", switch_state?"On ":"Off" );
      // PRINT("\r\nswitch_state = %s\n", switch_state?"Off":"On " );      

      if (switch_state == true) {
        gpio_set_level(RELAY_GPIO, 1);  // relay on
      }else {
        gpio_set_level(RELAY_GPIO, 0);  // relay off
      }
      
      break;
    default:
      if (oc_string_len(rep->name) > 2) {
        if (strncmp(oc_string(rep->name), "x.", 2) == 0) {
          break;
        }
      }
      bad_request = true;
      break;
    }
    rep = rep->next;
  }

  if (!bad_request) {
    switch_state = state;
  }

  oc_rep_start_root_object();
  oc_rep_set_boolean(root, value, switch_state);
  oc_rep_end_root_object();

  if (!bad_request) {
    oc_send_response(request, OC_STATUS_CHANGED);
  } else {
    oc_send_response(request, OC_STATUS_BAD_REQUEST);
  }
}

static void register_resources(void)
{
  oc_resource_t *temp = oc_new_resource("tempsensor", "/temp", 1, 0);
  oc_resource_bind_resource_type(temp, "oic.r.tempsensor");
  
  oc_resource_bind_resource_interface(temp, OC_IF_R);
  oc_resource_set_default_interface(temp, OC_IF_R);

  oc_resource_set_discoverable(temp, true);
  oc_resource_set_periodic_observable(temp, 1);
  oc_resource_set_request_handler(temp, OC_GET, get_temp, NULL);
  // oc_resource_set_request_handler(temp, OC_POST, post_temp, NULL);  
  oc_add_resource(temp);

  oc_resource_t *bswitch = oc_new_resource("smartbutton", "/switch", 1, 0);
  oc_resource_bind_resource_type(bswitch, "oic.r.switch.binary");
  oc_resource_bind_resource_interface(bswitch, OC_IF_A);
  oc_resource_set_default_interface(bswitch, OC_IF_A);
  oc_resource_set_discoverable(bswitch, true);
  oc_resource_set_request_handler(bswitch, OC_GET, get_switch, NULL);
  oc_resource_set_request_handler(bswitch, OC_POST, post_switch, NULL);
  oc_add_resource(bswitch);
}


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
#ifndef OC_IPV4
        xEventGroupClearBits(wifi_event_group, IPV6_CONNECTED_BIT);
#endif
        break;

    case SYSTEM_EVENT_STA_CONNECTED:
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

// static void initialise_wifi(void)
// {
//     tcpip_adapter_init();
//     wifi_event_group = xEventGroupCreate();
//     ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));
//     ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = EXAMPLE_WIFI_SSID,
//             .password = EXAMPLE_WIFI_PASS,
//         },
//     };

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//     ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
// }

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


//
void gpio_init()
{
   // DHT11
    gpio_pad_select_gpio(DHT_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_GPIO, GPIO_FLOATING);  

   // Relay
    gpio_pad_select_gpio(RELAY_GPIO);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(RELAY_GPIO, GPIO_PULLUP_ONLY);  
    gpio_set_level(RELAY_GPIO, 0);  // relay off

   // To read GPIO outpur 
    gpio_pad_select_gpio(RELAY_IN_GPIO);
    gpio_set_direction(DHT_GPIO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(DHT_GPIO, GPIO_FLOATING);  

}

// Display IP info & Temp, Humidity
void oled_task(void *pvParameter)
{
  char  ptr[64];
  float f_humidity, f_temp;

  gpio_init();
  switch_state = false;

  vTaskDelay(1000/portTICK_PERIOD_MS);
  // i2c_master_init();
  // ssd1306_init();

  task_ssd1306_display_clear(NULL);
  vTaskDelay(100/portTICK_PERIOD_MS);

  while(1) {
    
    // if (gpio_get_level(RELAY_IN_GPIO) == 0) {
    //   switch_state = false;
    // }else {
    //   switch_state = true;
    // }

    dht_read_float_data(GPIO_ID_PIN(DHT_GPIO), &f_humidity, &f_temp);
    temp = (int)f_temp;

#ifdef SMART_CONFIG
    if (wifi_config_needed == true) {
    sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s", 
        "Smart Config", ip4addr_ntoa(&(ip4_info.ip)), 0, "Off");
    }else {
      sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s", 
        ssid, ip4addr_ntoa(&(ip4_info.ip)), temp, switch_state?"On ":"Off");
    }
#else      
    sprintf(ptr, "%s\nIP:%s\nTemp : %2dC \nMist : %s",     
      EXAMPLE_WIFI_SSID, ip4addr_ntoa(&(ip4_info.ip)), temp, switch_state?"On ":"Off");
#endif    

    task_ssd1306_display_text((void *)ptr);

    vTaskDelay(1000 / portTICK_RATE_MS);
  }

}


static int server_main(void* pvParameter)
{
  int init;
  struct ip6_addr if_ipaddr_ip6 = { 0 };

  ESP_LOGI(TAG, "iotivity server task started");
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
                                       .register_resources = register_resources };

  oc_clock_time_t next_event;

#ifdef OC_SECURITY
  oc_storage_config("./server_creds");
#endif /* OC_SECURITY */

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

void app_main(void)
{
    if (nvs_flash_init() != ESP_OK){
        print_error("nvs_flash_init failed");
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
    if (xTaskCreate(&oled_task, "oled_task", 2048, NULL, 5, NULL) != pdPASS ) {
        print_error("task create failed");
    }     

    initialise_wifi();

    if (wifi_config_needed == false) {    
      if ( xTaskCreate(&server_main, "server_main", 15*1024, NULL, 5, NULL) != pdPASS ) {
          print_error("task create failed");
      }
    } 

    // if (xTaskCreate(&oled_task, "oled_task", 2048, NULL, 5, NULL) != pdPASS ) {
    //     print_error("task create failed");
    // }     

}

