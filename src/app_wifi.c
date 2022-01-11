/* ESPRESSIF MIT License
 * 
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 * 
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <string.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include "app_wifi.h"

/* The examples use WiFi configuration that you can set via 'make menuconfig'.

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define ESP_WIFI_SSID "mywifissid"
*/

#define ESP_WIFI_SSID       "Anh Nhan"
#define ESP_WIFI_PASS       "0905866086"
#define ESP_MAXIMUM_RETRY   CONFIG_ESP_MAXIMUM_RETRY

#define ESP_WIFI_AP_SSID    CONFIG_ESP_WIFI_AP_SSID
#define ESP_WIFI_AP_PASS    CONFIG_ESP_WIFI_AP_PASSWORD
#define MAX_STA_CONN        CONFIG_MAX_STA_CONN
#define SERVER_IP_ADDR      CONFIG_SERVER_IP
#define ESP_WIFI_AP_CHANNEL CONFIG_ESP_WIFI_AP_CHANNEL

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event,
 * For STA events handling, we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

typedef struct {
    int8_t rssi;
    uint8_t ssid_length;
    uint8_t ssid_idx;
    uint8_t ssid[WIFI_MAX_CHARACTER_LEN];
    uint8_t pass[2 * WIFI_MAX_CHARACTER_LEN];
} wifi_ap_t;

uint8_t sta_ssid[WIFI_MAX_CHARACTER_LEN];
uint8_t sta_pass[2 * WIFI_MAX_CHARACTER_LEN];
uint8_t sta_ip[WIFI_MAX_IP_LEN];

typedef struct {
    uint8_t num_aps;
    wifi_ap_t wifi_aps[WIFI_MAX_AVAILABLE_APS];
} wifi_ap_results_t;

static const char *TAG = "App_Wifi";
static bool is_connect_wifi = false;
static bool wifi_init_complete = false;

static int s_retry_num = 0;

esp_netif_t *AP_netif;
esp_netif_t *STA_netif;
// Set up parameters for Wifi APs get list
wifi_ap_results_t wifi_ap_results;
// Set up parameters for UART communication
uart_config_t uart_config = {
    .baud_rate = 115200,
    .data_bits = UART_DATA_8_BITS,
    .parity    = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
};

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station: " MACSTR "join, AID=%d",
                 MAC2STR(event->mac), 
                 event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station:" MACSTR "leave, AID=%d",
                 MAC2STR(event->mac), 
                 event->aid);
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        if(esp_wifi_connect() == ESP_OK)
        {
            is_connect_wifi = true;
        }
        else
        {
            ESP_LOGI(TAG,"Fail to connect to WIFI - line 124\r\n");
            is_connect_wifi = false;
        }
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            if(esp_wifi_connect() == ESP_OK)
            {
                is_connect_wifi = true;
            }
            else
            {
                ESP_LOGI(TAG,"Fail to connect to WIFI - line 136\r\n");
                is_connect_wifi = false;
            }            
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));

        // Store the local IP
        memset(sta_ip, 0, sizeof(sta_ip));
        snprintf((char *)sta_ip, sizeof(sta_ip), "http://%d.%d.%d.%d", (((event->ip_info.ip.addr)) & (0xFF)), (((event->ip_info.ip.addr) >> 8) & (0xFF)),
                            (((event->ip_info.ip.addr) >> 16) & (0xFF)), (((event->ip_info.ip.addr) >> 24) & (0xFF)));
        ESP_LOGI(TAG, "COPY LOCAL IP: %s\r\n", sta_ip);          

        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_softap(wifi_mode_t mode)
{
    /* default event loop from esp_event library */
    ESP_ERROR_CHECK(esp_netif_init());
    // ESP_ERROR_CHECK(esp_event_loop_create_default());

    AP_netif = esp_netif_create_default_wifi_ap();
    assert(AP_netif);

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    if (strcmp(SERVER_IP_ADDR, "192.168.4.1"))
    {
        int a, b, c, d;
        sscanf(SERVER_IP_ADDR, "%d.%d.%d.%d", &a, &b, &c, &d);
        esp_netif_ip_info_t ip_info;
        esp_netif_set_ip4_addr(&ip_info.ip, a, b, c, d);
        esp_netif_set_ip4_addr(&ip_info.gw, a, b, c, d);
        esp_netif_set_ip4_addr(&ip_info.netmask, 255, 255, 255, 0);
        ESP_ERROR_CHECK(esp_netif_dhcps_stop(AP_netif));
        ESP_ERROR_CHECK(esp_netif_set_ip_info(AP_netif, &ip_info));
        ESP_ERROR_CHECK(esp_netif_dhcps_start(AP_netif));
    }

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config_t));

    snprintf((char*)wifi_config.ap.ssid, 32, "%s", ESP_WIFI_AP_SSID);
    wifi_config.ap.ssid_len = strlen((char*)wifi_config.ap.ssid);
    snprintf((char*)wifi_config.ap.password, 64, "%s", ESP_WIFI_AP_PASS);
    
    wifi_config.ap.max_connection = MAX_STA_CONN;
    wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;

    if (strlen(ESP_WIFI_AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    if (strlen(ESP_WIFI_AP_CHANNEL)) {
        int channel;
        sscanf(ESP_WIFI_AP_CHANNEL, "%d", &channel);
        wifi_config.ap.channel = channel;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    // "esp_wifi_set_config" can be called only when specified interface is enabled, otherwise, API fail
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID : %s password : %s channel : %s",
             ESP_WIFI_AP_SSID, ESP_WIFI_AP_PASS, ESP_WIFI_AP_CHANNEL);
}

void wifi_init_sta(wifi_mode_t mode)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    STA_netif = esp_netif_create_default_wifi_sta();
    assert(STA_netif);

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    
    snprintf((char*)wifi_config.sta.ssid, 32, "%s", sta_ssid);
    snprintf((char*)wifi_config.sta.password, 64, "%s", sta_pass);

    ESP_LOGI(TAG, "ESP_WIFI_SSID: %s", wifi_config.sta.ssid);
    ESP_LOGI(TAG, "ESP_WIFI_PASS: %s", wifi_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_set_mode(mode));
    // "esp_wifi_set_config" can be called only when specified interface is enabled, otherwise, API fail
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_STA finished.");

    // Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above)
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
    
    // xEventGroupWaitBits() returns the bits before the call returned, 
    // hence we can test which event actually happened.

    if (bits & WIFI_CONNECTED_BIT) {
        wifi_ap_record_t AP_info;
        ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&AP_info));
        ESP_LOGI(TAG, "connected to AP, SSID : %s Channel : %d Strength : %d Authmode : %d",
                 AP_info.ssid, AP_info.primary, AP_info.rssi, AP_info.authmode);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID : %s, password : %s",
                 sta_ssid, sta_pass);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    // The event will not be processed after unregister
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    
    vEventGroupDelete(s_wifi_event_group);
}

// A task to manage esp communication
static void esp_com_task(void * arg)
{
    int len;
    uint8_t data [BUF_SIZE];
    uint8_t tx_packet[30];
    tx_packet[0] = ESP_CMD_HEADER1;
    tx_packet[1] = ESP_CMD_HEADER2;
    uint8_t retry_cnt;

    /*
    "============================================================================"
    "      HEADER 1   | HEADER 2  |    LENGTH   | CMD |            DATA          "
    "============================================================================"
    */
    while(1)
    {   
        len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        // Check the reponse payload from MCU
        if((data[0] == ESP_CMD_HEADER1) && (data[1] == ESP_CMD_HEADER2))
        {
            switch(data[3])
            {
                case ESP_CMD_CONNECT:
                    if (!is_connect_wifi)
                    {
                        // memset(sta_ip, 0, sizeof(sta_ip));
                        app_wifi_establish_connection();
                        vTaskDelay(500 / portTICK_RATE_MS);
                        is_connect_wifi = true;
                    }

                    // Send response
                    tx_packet[2] = ESP_CMD_SIZE + 1;
                    tx_packet[3] = ESP_CMD_CONNECT;
                    if(is_connect_wifi)
                    {
                        tx_packet[4] = 0x01;
                    }
                    else
                    {
                        tx_packet[4] = 0x00;
                    }
                    uart_write_bytes(UART_NUM_0, (const char *)tx_packet, ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE + ESP_CMD_SIZE + 1);
                    uart_wait_tx_done(UART_NUM_0, 500);
                    if(!wifi_init_complete)
                    {
                        wifi_init_complete = true;
                    }
                    break;

                case ESP_CMD_DISCONNECT:

                    if(is_connect_wifi)
                    {
                        if(esp_wifi_disconnect() == ESP_OK)
                        {
                            is_connect_wifi = false;
                        }
                        else
                        {
                            ESP_LOGI(TAG,"Fail to disconnect to WIFI\r\n");
                            is_connect_wifi = true;
                        }    
                    }

                    // Send response
                    tx_packet[2] = ESP_CMD_SIZE + 1;
                    tx_packet[3] = ESP_CMD_DISCONNECT;
                    if(!is_connect_wifi)
                    {
                        tx_packet[4] = 0x01;
                    }
                    else
                    {
                        tx_packet[4] = 0x00;
                    }
                    uart_write_bytes(UART_NUM_0, (const char *)tx_packet, ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE + ESP_CMD_SIZE + 1);
                    uart_wait_tx_done(UART_NUM_0, 500);
                    break;

                case ESP_CMD_GET_AVAILABLE_SSID:
                    app_wifi_scan_aps();
                    break;

                case ESP_CMD_SET_SSID:
                    uart_flush(UART_NUM_0);
                    memcpy(sta_ssid, wifi_ap_results.wifi_aps[data[4]].ssid, wifi_ap_results.wifi_aps[data[4]].ssid_length);
                    memcpy(sta_pass, &data[5], data[2] - ESP_SSID_IDX_SIZE - ESP_CMD_SIZE);
                    ESP_LOGI(TAG, "Receive Accepted SSID: %s\r\n", sta_ssid);
                    ESP_LOGI(TAG, "Receive Accepted PASSWORD: %s\r\n", sta_pass);
                    
                    // Clear all parameters related to wifi scanning
                    // memset(&wifi_ap_results, 0, sizeof(wifi_ap_results));

                    // Send response
                    tx_packet[2] = ESP_CMD_SIZE + 1;
                    tx_packet[3] = ESP_CMD_SET_SSID;
                    tx_packet[4] = 0x01;
                    uart_write_bytes(UART_NUM_0, (const char *)tx_packet, ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE + ESP_CMD_SIZE + 1);
                    uart_wait_tx_done(UART_NUM_0, 500);                    
                    break;

                case ESP_CMD_GET_IP:
                    // retry_cnt = 30;
                    // while(retry_cnt > 0)
                    // {
                    //     retry_cnt--;
                    //     if(sta_ip[0] != 0)
                    //     {
                    //         break;
                    //     }

                    //     vTaskDelay(100 / portTICK_RATE_MS);
                    // }

                    if(sta_ip[0] != 0)
                    {
                        tx_packet[4] = 0x01;
                        tx_packet[2] = ESP_CMD_SIZE + 1 + strlen((char *)sta_ip);
                        memcpy(&tx_packet[5], (char *)sta_ip, strlen((char *)sta_ip));
                        ESP_LOGI(TAG, "Send Local IP to MCU: %s\r\n", &tx_packet[5]);
                    }
                    else
                    {
                        tx_packet[4] = 0x00;
                        tx_packet[2] = ESP_CMD_SIZE + 1;
                    }

                    // Send response
                    tx_packet[3] = ESP_CMD_GET_IP;
                    
                    // Send back local id
                    uart_write_bytes(UART_NUM_0, (const char *)tx_packet, ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE +  tx_packet[2]);
                    uart_wait_tx_done(UART_NUM_0, 500);
                    break;

                case ESP_CMD_GET_RSSI:
                    break;

                case ESP_CMD_RESPONSE_ACK:
                    break;

                default:
                    break;
            }
            // memset(data, 0, sizeof(data));
            // uart_flush(UART_NUM_0);
            // ESP_LOGI(TAG, "Receive ACK: %s\r\n", data);
            // break;
        }
        else
        {
            // uart_write_bytes(UART_NUM_0, (const char *) "FAIL ACK", strlen("FAIL ACK"));
        }

        memset(data, 0, sizeof(data));
        vTaskDelay(20 / portTICK_RATE_MS);
    }
}

void app_wifi_set_ssid(void)
{
    uint8_t data [BUF_SIZE];
    uint8_t tx_packet[30];
    int len;
    tx_packet[0] = ESP_CMD_HEADER1;
    tx_packet[1] = ESP_CMD_HEADER2;

    // wait for response here from MCU to choose the Wifi network
    /*
    "================================================================================="
    "      HEADER 1   | HEADER 2  |    LENGTH   | CMD |  SSID INDEX  |   SSID_PASS    "
    "================================================================================="
    */
    while (1) 
    {
        len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);

        // Check received header
        if((data[0] == ESP_CMD_HEADER1) && (data[1] == ESP_CMD_HEADER2) && (data[3] == ESP_CMD_SET_SSID))
        {
            uart_flush(UART_NUM_0);
            memcpy(sta_ssid, wifi_ap_results.wifi_aps[data[4]].ssid, wifi_ap_results.wifi_aps[data[4]].ssid_length);
            memcpy(sta_pass, &data[5], data[2] - ESP_SSID_IDX_SIZE - ESP_CMD_SIZE);
            ESP_LOGI(TAG, "Receive Accepted SSID: %s\r\n", sta_ssid);
            ESP_LOGI(TAG, "Receive Accepted PASSWORD: %s\r\n", sta_pass);
            
            // Clear all parameters related to wifi scanning
            memset(data, 0, sizeof(data));
            memset(&wifi_ap_results, 0, sizeof(wifi_ap_results));

            // Send response
            tx_packet[2] = ESP_CMD_SIZE + 1;
            tx_packet[3] = ESP_CMD_SET_SSID;
            tx_packet[4] = 0x01;
            uart_write_bytes(UART_NUM_0, (const char *)tx_packet, ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE + ESP_CMD_SIZE + 1);
            uart_wait_tx_done(UART_NUM_0, 500);
            break;
        }
        else
        {

        }
    }
}

void app_wifi_scan_aps(void)
{
    if(is_connect_wifi)
    {
        if(esp_wifi_disconnect() == ESP_OK)
        {
            is_connect_wifi = false;
        }
        else
        {
            ESP_LOGI(TAG,"Fail to disconnect to WIFI\r\n");
            is_connect_wifi = true;
        }    
    }

    // Start scan all available APs
    ESP_ERROR_CHECK(esp_wifi_stop());
    ESP_ERROR_CHECK(esp_wifi_set_mode (WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Reset all parameters incase being disconnected
    memset(&wifi_ap_results, 0, sizeof(wifi_ap_results));
    wifi_ap_results.num_aps = 0;
    for(int ch_idx = 0; ch_idx < 14; ch_idx++)
    {
        wifi_scan_config_t scan_config = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = ch_idx,
            .show_hidden = true,
            .scan_type = WIFI_SCAN_TYPE_ACTIVE,
            .scan_time.active.min = 10,
            .scan_time.active.max = 500        
        };

        ESP_LOGI(TAG, "Start Scan");
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

        // Scan all available APs
        uint16_t apCount = 0;
        esp_wifi_scan_get_ap_num(&apCount);
        ESP_LOGI(TAG, "Number of access points found: %d", apCount);
        if (apCount > 0) 
        {
            wifi_ap_record_t *list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * apCount);
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&apCount, list));
            int i;
            ESP_LOGI(TAG, "======================================================================");
            ESP_LOGI(TAG, "             SSID             |    RSSI    |           AUTH           ");
            ESP_LOGI(TAG, "======================================================================");
            for (i=0; i<apCount; i++) {
                char *authmode;
                switch(list[i].authmode) {
                case WIFI_AUTH_OPEN:
                    authmode = "WIFI_AUTH_OPEN";
                    break;
                case WIFI_AUTH_WEP:
                    authmode = "WIFI_AUTH_WEP";
                    break;           
                case WIFI_AUTH_WPA_PSK:
                    authmode = "WIFI_AUTH_WPA_PSK";
                    break;           
                case WIFI_AUTH_WPA2_PSK:
                    authmode = "WIFI_AUTH_WPA2_PSK";
                    break;           
                case WIFI_AUTH_WPA_WPA2_PSK:
                    authmode = "WIFI_AUTH_WPA_WPA2_PSK";
                    break;
                default:
                    authmode = "Unknown";
                    break;
                }
                ESP_LOGI(TAG, "%26.26s    |    % 4d    |    %22.22s",list[i].ssid, list[i].rssi, authmode);

                // Append all available APs to send to the MCU
                if(strlen((char *)list[i].ssid) < WIFI_MAX_CHARACTER_LEN) {
                    wifi_ap_results.wifi_aps[wifi_ap_results.num_aps].rssi = list[i].rssi;
                    wifi_ap_results.wifi_aps[wifi_ap_results.num_aps].ssid_idx = wifi_ap_results.num_aps;
                    wifi_ap_results.wifi_aps[wifi_ap_results.num_aps].ssid_length = strlen((char *) list[i].ssid);
                    memcpy(wifi_ap_results.wifi_aps[wifi_ap_results.num_aps].ssid, list[i].ssid, strlen((char *) list[i].ssid)); 
                    ESP_LOGI(TAG, "Append Wifi AP - %s\r\n", wifi_ap_results.wifi_aps[wifi_ap_results.num_aps].ssid);
                    wifi_ap_results.num_aps++;
                }
            }
            free(list);   
        }
    }

    ESP_ERROR_CHECK(esp_wifi_stop());    

    uint8_t data [BUF_SIZE];
    uint8_t tx_packet[30];
    int len;
    tx_packet[0] = ESP_CMD_HEADER1;
    tx_packet[1] = ESP_CMD_HEADER2;

    // uint8_t retry_cnt;

    // Send the scanned APs to MCU and wait for response
    /*
    "============================================================================"
    "      HEADER 1   | HEADER 2  |    LENGTH   | CMD | SSID INDEX |     SSID    "
    "============================================================================"
    */
    if(wifi_ap_results.num_aps > 0)
    {
        for (uint8_t idx=0; idx<wifi_ap_results.num_aps; idx++)
        {
            // retry_cnt = 3;
            // Send to UART here
            // retry_cnt--;
            // Write data back to the UART
            if (wifi_ap_results.wifi_aps[idx].ssid_length <= ESP_SSID_MAX_LEN)
            {
                tx_packet[2] = wifi_ap_results.wifi_aps[idx].ssid_length + ESP_SSID_IDX_SIZE + ESP_CMD_SIZE + 1;
                tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
                tx_packet[4] = 0x01;
                tx_packet[5] = wifi_ap_results.wifi_aps[idx].ssid_idx;
                memcpy(&tx_packet[6], wifi_ap_results.wifi_aps[idx].ssid, wifi_ap_results.wifi_aps[idx].ssid_length);
                uart_write_bytes(UART_NUM_0, (const char *)tx_packet, wifi_ap_results.wifi_aps[idx].ssid_length + 
                                        ESP_CMD_HEADER_SIZE + ESP_CMD_PAYLOAD_LEN_SIZE + ESP_CMD_SIZE + ESP_SSID_IDX_SIZE + 1);
                uart_wait_tx_done(UART_NUM_0, 500);
                // ESP_LOGI(TAG, "ESP_WIFI_SSID: %s", wifi_ap_results.wifi_aps[idx].ssid);

                // Read data from the UART
                while (1) 
                {
                    len = uart_read_bytes(UART_NUM_0, data, BUF_SIZE, 20 / portTICK_RATE_MS);

                    // Check the reponse payload from MCU
                    if((data[0] == ESP_CMD_HEADER1) && (data[1] == ESP_CMD_HEADER2) && (data[3] == ESP_CMD_RESPONSE_ACK))
                    {
                        memset(data, 0, sizeof(data));
                        uart_flush(UART_NUM_0);
                        // ESP_LOGI(TAG, "Receive ACK: %s\r\n", data);
                        break;
                    }
                    else
                    {
                        // uart_write_bytes(UART_NUM_0, (const char *) "FAIL ACK", strlen("FAIL ACK"));
                    }
                }
            }
        }

        // reach the end of the SSID list
        tx_packet[2] = ESP_CMD_SIZE + 1;
        tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
        tx_packet[4] = 0x00;
        uart_write_bytes(UART_NUM_0, (const char *)tx_packet, 5);
        uart_wait_tx_done(UART_NUM_0, 500);
    }
    else
    {
        tx_packet[2] = ESP_CMD_SIZE + 1;
        tx_packet[3] = ESP_CMD_GET_AVAILABLE_SSID;
        tx_packet[4] = 0x00;
        uart_write_bytes(UART_NUM_0, (const char *)tx_packet, 5);
        uart_wait_tx_done(UART_NUM_0, 500);
    }
}


void app_wifi_establish_connection(void)
{
    wifi_mode_t mode;
    mode = WIFI_MODE_NULL;

    if (strlen(ESP_WIFI_AP_SSID) && strlen(ESP_WIFI_SSID)) {
        mode = WIFI_MODE_APSTA;
        ESP_LOGI(TAG, "Wifi mode is set to WIFI_MODE_APSTA");
        wifi_init_sta(mode);
        // wifi_init_softap(mode);
    } else if (strlen(ESP_WIFI_AP_SSID)) {
        mode = WIFI_MODE_AP;
        ESP_LOGI(TAG, "Wifi mode is set to WIFI_MODE_AP");
        wifi_init_softap(mode);
    } else if (strlen(ESP_WIFI_SSID)) {
        mode = WIFI_MODE_STA;
        ESP_LOGI(TAG, "Wifi mode is set to WIFI_MODE_STA");
        wifi_init_sta(mode);
    }

    if (mode == WIFI_MODE_NULL) {
        ESP_LOGW(TAG,"Neither AP or STA have been configured. WiFi will be off.");
        return;
    }

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // Set current WiFi power save type.
}


void app_wifi_main_init(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    // uart_driver_install();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Init UART parameters
    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, ESP_UART_TXD, ESP_UART_RXD, ESP_UART_RTS, ESP_UART_CTS);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    /* Always use WIFI_INIT_CONFIG_DEFAULT macro to init the config to default values, 
     * this can guarantee all the fields got correct value when more fields are added 
     * into wifi_init_config_t in future release. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    /* esp_wifi_init API must be called before all other WiFi API can be called */
    ESP_LOGI(TAG, "Initializing ESP Wifi");
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Scan for the available network at the initialization phase
    // app_wifi_scan_aps();
    xTaskCreate(esp_com_task, "eso-com-task", 4096, NULL, 2, NULL);

    // Wait here untils the the connection estalishment is called
    while(!wifi_init_complete)
    {
        vTaskDelay(20 / portTICK_RATE_MS);
    }
}
