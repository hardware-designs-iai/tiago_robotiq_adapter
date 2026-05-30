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

// ==============================================================================
// ESP-NOW SLAVE MAC ADDRESS CONFIGURATION
// ==============================================================================
// Define the total number of receiving ESP32 slave nodes (Robotiq grippers).
#define NUM_SLAVES 2

// ------------------------------------------------------------------------------
// IMPORTANT: Replace the hex values below with the actual Wi-Fi Station MAC 
// addresses of your specific ESP32-S3 slave boards.
//
// Routing Information:
// The index of the MAC address in this array dictates the Modbus RTU routing.
// The Master node (AtomS3U) will map incoming serial data from specific virtual 
// COM ports to the corresponding MAC address below.
// ------------------------------------------------------------------------------
uint8_t slave_macs[NUM_SLAVES][6] = {
    
    // --- Gripper 1 Node ---
    // Modbus routing: Mapped to Virtual COM Port 0
    {0xf0, 0xf5, 0xbd, 0x73, 0x12, 0x54}, 

    // --- Gripper 2 Node ---
    // Modbus routing: Mapped to Virtual COM Port 1
    {0x34, 0xb7, 0xda, 0x58, 0xe8, 0x08}  
};

// ---------------------------------------------------------
// USB RX (PC -> Master -> Specific Slave)
// ---------------------------------------------------------
void tinyusb_cdc_rx_callback(int itf, cdcacm_event_t *event)
{
    size_t rx_size = 0;
    
    // Read into the global buffer
    esp_err_t ret = tinyusb_cdcacm_read(itf, buf, CONFIG_TINYUSB_CDC_RX_BUFSIZE, &rx_size);
    
    if (ret == ESP_OK && rx_size > 0) {
        
        // Allocate memory so concurrent USB ports don't overwrite each other
        char *str = (char *)malloc(rx_size + 1);
        if (str != NULL) {
            memcpy(str, buf, rx_size); 
            str[rx_size] = '\0';  
            
            // Route to the correct MAC based on which USB port received the data
            if (itf == TINYUSB_CDC_ACM_0) {
                esp_now_send(slave_macs[0], (uint8_t *)str, rx_size);
            } 
            else if (itf == TINYUSB_CDC_ACM_1) {
                esp_now_send(slave_macs[1], (uint8_t *)str, rx_size);
            }
            
            free(str); // Free the memory 
    } else {
        ESP_LOGE(TAG, "Read error on channel %d", itf);
    }
}

void tinyusb_cdc_line_state_changed_callback(int itf, cdcacm_event_t *event)
{
    int dtr = event->line_state_changed_data.dtr;
    int rts = event->line_state_changed_data.rts;
    ESP_LOGI(TAG, "Line state changed on channel %d: DTR:%d, RTS:%d", itf, dtr, rts);
}

// ---------------------------------------------------------
// WIFI & ESP-NOW SETUP
// ---------------------------------------------------------
static void master_wifi_init(void) {
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void master_espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status) {
    // Kept quiet
}

// ---------------------------------------------------------
// ESP-NOW RX (Specific Slave -> Master -> Correct COM Port)
// ---------------------------------------------------------
static void master_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    
    // EXACT ORIGINAL LOGIC for receiving data
    uint8_t local_buf_esp[CONFIG_TINYUSB_CDC_RX_BUFSIZE + 1];                               
    memcpy(local_buf_esp, data, len);
    
    // Determine which USB port to send the response to based on the sender's MAC
    int target_usb_port = TINYUSB_CDC_ACM_0; // Default to port 0
    
    if (memcmp(recv_info->src_addr, slave_macs[1], 6) == 0) {
        target_usb_port = TINYUSB_CDC_ACM_1; // Switch to port 1 if Slave 2 sent it
    }

    tinyusb_cdcacm_write_queue(target_usb_port, local_buf_esp, (size_t)len);
    tinyusb_cdcacm_write_flush(target_usb_port, 0);
}

static void master_esp_now_init(void) {
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_send_cb(master_espnow_send_cb));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(master_espnow_recv_cb));
    
    esp_now_peer_info_t peerInfo = {0}; 
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    // Register Slave 1
    memcpy(peerInfo.peer_addr, slave_macs[0], 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    
    // Register Slave 2
    memcpy(peerInfo.peer_addr, slave_macs[1], 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peerInfo));
    
    ESP_LOGI(TAG, "ESP-NOW Peers Setup Complete");
}

// ---------------------------------------------------------
// MAIN TASKS
// ---------------------------------------------------------
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
    
    // Initialize COM Port 0 (Mapped to Slave 1)
    tinyusb_config_cdcacm_t acm_cfg_0 = {
        .cdc_port = TINYUSB_CDC_ACM_0,
        .callback_rx = &tinyusb_cdc_rx_callback, 
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg_0));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_0,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));

    // Initialize COM Port 1 (Mapped to Slave 2)
#if (CONFIG_TINYUSB_CDC_COUNT > 1)
    tinyusb_config_cdcacm_t acm_cfg_1 = {
        .cdc_port = TINYUSB_CDC_ACM_1,
        .callback_rx = &tinyusb_cdc_rx_callback, 
        .callback_rx_wanted_char = NULL,
        .callback_line_state_changed = NULL,
        .callback_line_coding_changed = NULL
    };
    ESP_ERROR_CHECK(tinyusb_cdcacm_init(&acm_cfg_1));
    ESP_ERROR_CHECK(tinyusb_cdcacm_register_callback(
                        TINYUSB_CDC_ACM_1,
                        CDC_EVENT_LINE_STATE_CHANGED,
                        &tinyusb_cdc_line_state_changed_callback));
#endif

    ESP_LOGI(TAG, "USB initialization DONE");
  
    // Create ESP-NOW communication task
    xTaskCreate(espnow_communication_task, "espnow_communication_task", 10240, NULL, 4, NULL); 
}