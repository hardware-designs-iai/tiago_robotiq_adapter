/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tinyusb.h"
#include "tusb_cdc_acm.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "espnow.h"
#include "espnow_storage.h"
#include "espnow_utils.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "master_espnow.h"


static const char *TAG = "Tiago master";
static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
static uint8_t buf_esp[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];
//34:b7:da:58:7c:b4
//34:b7:da:58:cc:70
//34:b7:da:58:e8:08
//uint8_t mac_address_gripper[] = {0x34, 0xb7, 0xda, 0x58, 0x7c, 0xb4}; // External wiring
//uint8_t mac_address_gripper[] = {0x34, 0xb7, 0xda, 0x58, 0xcc, 0x70}; // Within gripper
uint8_t mac_address_gripper[] = {0x34, 0xb7, 0xda, 0x58, 0xe8, 0x08}; 

int static write_back_port=0;


void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    /* initialization */
    size_t rx_size = 0;

    /* read */
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);

    if (ret == ESP_OK) {
       ESP_LOGI(TAG, "Data from channel %d. len: %d", itf ,rx_size);
       ESP_LOG_BUFFER_HEXDUMP(TAG, buf, rx_size, ESP_LOG_INFO);

      //Allocating memory to send data over esp_now
     // size_t total_length = sizeof(espnow_data_t)+rx_size;
     // espnow_data_t *data_to_send = (espnow_data_t *)malloc(total_length);
      //Copy the payload into the structure
     // memcpy(data_to_send->payload,buf,rx_size);
       write_back_port = itf;
       char *str = (char *)malloc(rx_size + 1);
       memcpy(str, &buf, rx_size);
       str[rx_size] = '\0';  // 
       ESP_LOGI(TAG, "Payload as string: %s  ,data_length  :%d ",  str,rx_size);
     
       if(esp_now_send(&mac_address_gripper,(uint8_t *)str,rx_size) ==ESP_OK){
           ESP_LOGI(TAG, "Data has been sent to mac address: " );
         }
        free(str);

    } else {
        ESP_LOGE(TAG, "Read error");
    }
        
      
         /* write back */ 
         // tinyusb_cdcacm_write_queue(itf,buf,rx_size);

        //  tinyusb_cdcacm_write_flush(itf, 0);
    
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}
static void master_wifi_init(){
    esp_event_loop_create_default();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void master_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status){
  ESP_LOGI(TAG, "Send data");
   
}

static void master_espnow_recv_cb( const esp_now_recv_info_t *recv_info, const uint8_t *data, int len){
uint8_t buf_esp[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];                               
memcpy(buf_esp, data, len);
tinyusb_cdcacm_write_queue(write_back_port,buf_esp,(size_t)len );
tinyusb_cdcacm_write_flush(write_back_port, 0);

  ESP_LOGI(TAG, "Received");

}


static void master_esp_now_init(){
    
ESP_ERROR_CHECK(esp_now_init());
ESP_ERROR_CHECK(esp_now_register_send_cb(master_espnow_send_cb));
ESP_ERROR_CHECK(esp_now_register_recv_cb(master_espnow_recv_cb));
esp_now_peer_info_t peerInfo={};
//register peer
peerInfo.channel = 0;
peerInfo.encrypt = false;
//register first peer
memcpy(peerInfo.peer_addr,mac_address_gripper,6);
ESP_ERROR_CHECK( esp_now_add_peer(&peerInfo) );
ESP_LOGI(TAG, "After peer information");
}


void app_main(void)
{

    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
        .configuration_descriptor = NULL,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    vTaskDelay(10);
    tinyusb_config_cdcacm_t acm_cfg = {
        .usb_dev = TINYUSB_USBDEV_0,
        .cdc_port = TINYUSB_CDC_ACM_0,
        .rx_unread_buf_sz = 64,
        .callback_rx = &tinyusb_cdc_rx_callback, // the first way to register a callback
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };

    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    /* the second way to register a callback */
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    ESP_ERROR_CHECK(tusb_cdc_acm_init(&acm_cfg));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif
   ESP_LOGI(TAG, "USB initialization DONE");

  

  xTaskCreate(espnow_communication_task, "espnow_communication_task", 10240, NULL, 4, NULL); // ESP now task created that is solely responsible to handle ESP Now communication 
   
}

static void espnow_communication_task(void *pvParameter)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
      espnow_storage_init();
      master_wifi_init();
      master_esp_now_init();
  
while(1){

    vTaskDelay(10);
}

}
