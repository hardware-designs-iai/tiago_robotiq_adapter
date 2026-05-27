#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_mac.h"
#include "esp_event.h" 
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "driver/gpio.h"

#include "tinyusb.h"
#include "tinyusb_cdc_acm.h" 
#include "tinyusb_default_config.h"

// ESP-NOW Component headers
#include "espnow.h"
#include "espnow_storage.h"
#include "espnow_utils.h"

#include "master_espnow.h"

static const char *TAG = "Tiago master";

static uint8_t buf[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];

// External wiring / Gripper MAC options
// uint8_t mac_address_gripper[] = {0x34, 0xb7, 0xda, 0x58, 0x7c, 0xb4}; 
// uint8_t mac_address_gripper[] = {0x34, 0xb7, 0xda, 0x58, 0xcc, 0x70}; 
uint8_t mac_address_gripper[] = {0xf0, 0xf5, 0xbd, 0x73, 0x12, 0x1c}; 

static int write_back_port = 0;

void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data from channel %d. len: %d", itf, rx_size);
        ESP_LOG_BUFFER_HEXDUMP(TAG, buf, rx_size, ESP_LOG_INFO);
        
        write_back_port = itf;
        char *str = (char *)malloc(rx_size + 1);
        if (str != NULL) {
            // Use 'buf' directly
            memcpy(str, buf, rx_size); 
            str[rx_size] = '\0';  
            
            ESP_LOGI(TAG, "Payload as string: %s  ,data_length: %d", str, rx_size);
            
            // Pass the array directly without the '&' reference
            if(esp_now_send(mac_address_gripper, (uint8_t *)str, rx_size) == ESP_OK) {
                ESP_LOGI(TAG, "Data has been sent to mac address");
            }
            free(str);
        }
    } else {
        ESP_LOGE(TAG, "Read error");
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

static void master_wifi_init(void) {
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// Updated callback signature for v6.0 / ESP-NOW v2.x
static void master_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    ESP_LOGI(TAG, "Send data status: %d", status);
}

static void master_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    uint8_t local_buf_esp[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];                               
    memcpy(local_buf_esp, data, len);
    
    tinyusb_cdcacm_write_queue(write_back_port, local_buf_esp, (size_t)len);
    tinyusb_cdcacm_write_flush(write_back_port, 0);
    ESP_LOGI(TAG, "Received ESP-NOW data, forwarded to USB");
}

static void master_esp_now_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(master_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(master_espnow_recv_cb));
    
    // Initialize struct to 0 to prevent garbage data in background fields
    esp_now_peer_info_t peerInfo = {0}; 
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, mac_address_gripper, 6);
    
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    ESP_LOGI(TAG, "After peer information setup");
}

static void espnow_communication_task(void *pvParameter)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    espnow_storage_init();
    master_wifi_init();
    master_esp_now_init();
  
    while(1){
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB initialization");
    
    const tinyusb_config_t tusb_cfg = TINYUSB_DEFAULT_CONFIG(); 
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    vTaskDelay(pdMS_TO_TICKS(10));
    
    tinyusb_config_cdcacm_t acm_cfg = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback, 
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    
    // Updated function name for v2.x
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    acm_cfg.cdc_port = TINYUSB_CDC_ACM_1;
    // Updated function name for v2.x
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg));
    
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif

    ESP_LOGI(TAG, "USB initialization DONE");
  
    // Create ESP-NOW communication task
    xTaskCreate(espnow_communication_task, "espnow_communication_task", 10240, NULL, 4, NULL); 
}