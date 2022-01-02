/*
  * ESPRESSIF MIT License
  *
  * Copyright (c) 2017 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
  *
  * Permission is hereby granted for use on ESPRESSIF SYSTEMS products only, in which case,
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
  *
  */
#ifndef _APP_WIFI_H_
#define _APP_WIFI_H_

#include "esp_netif.h"
#include "driver/uart.h"

extern esp_netif_t *AP_netif;
extern esp_netif_t *STA_netif;

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_SSID_MAX_LEN                (24)
#define ESP_SSID_IDX_SIZE               (1)
#define ESP_CMD_SIZE                    (1)
#define ESP_CMD_HEADER_SIZE             (2)
#define ESP_CMD_PAYLOAD_LEN_SIZE        (1)

#define ESP_CMD_HEADER1                 (0xAB)
#define ESP_CMD_HEADER2                 (0xBA)

#define WIFI_MAX_AVAILABLE_APS  (50)
#define WIFI_MAX_CHARACTER_LEN  (32)
#define BUF_SIZE (1024)
#define WIFI_MAX_IP_LEN         (30)

#define ESP_UART_TXD  (1)
#define ESP_UART_RXD  (3)
#define ESP_UART_RTS  (UART_PIN_NO_CHANGE)
#define ESP_UART_CTS  (UART_PIN_NO_CHANGE)

#define ESP_GET_SSID_REACH_THE_END      (0x00)


#define ESP_CMD_CONNECT                 (0xA0)
#define ESP_CMD_DISCONNECT              (0xA1)
#define ESP_CMD_GET_AVAILABLE_SSID      (0xA2)
#define ESP_CMD_SET_SSID                (0xA3)
#define ESP_CMD_GET_IP                  (0xA4)
#define ESP_CMD_GET_RSSI                (0xA5)

#define ESP_CMD_RESPONSE_ACK            (0xB0)



void app_wifi_main_init(void);
void app_wifi_establish_connection(void);
void app_wifi_scan_aps(void);
void app_wifi_set_ssid(void);

#ifdef __cplusplus
}
#endif

#endif
