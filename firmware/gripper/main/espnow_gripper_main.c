#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "espnow_gripper.h"

#define rs485_TASK_STACK_SIZE   (10000)
#define rs485_TASK_PRIO         (10)
#define rs485_UART_PORT         (2)
#define BUF_SIZE                (127)
#define BAUD_RATE               (115200)

#define rs485_TXD   (GPIO_NUM_6)
#define rs485_RXD   (GPIO_NUM_4)
#define rs485_RTS   (GPIO_NUM_5)
#define rs485_CTS   (UART_PIN_NO_CHANGE)

// Set to 20 to prevent Modbus packet fragmentation
#define rs485_READ_TOUT  (20) 

static const char *TAG = "gripper";

// Master Node MAC Address (Updated to your actual MAC)
static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0xd0, 0xcf, 0x13, 0x0f, 0xee, 0x9c };

// Dedicated static memory buffers for safe cross-task data sharing
static uint8_t master_rx_buf[BUF_SIZE];
static int master_rx_len = 0;

static uint8_t gripper_rx_buf[BUF_SIZE];
static int gripper_rx_len = 0;

QueueHandle_t from_espnow;
QueueHandle_t from_rs485;

// Function prototypes
static void rs485_communication_task(void *vParameter);
static void gripper_espnow_task(void *pvParameter);

/* WiFi should start before using ESPNOW */
static void wifi_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(ESPNOW_WIFI_MODE) );
    ESP_ERROR_CHECK( esp_wifi_start());
    ESP_ERROR_CHECK( esp_wifi_set_channel(CONFIG_ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE));
}

// Updated v6.0 Signature
static void gripper_espnow_send_cb(const esp_now_send_info_t *send_info, esp_now_send_status_t status)
{
    // Keeping this silent to prevent console spam at 100Hz
}

// Updated v6.0 Signature
static void gripper_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (data == NULL || len <= 0 || len > BUF_SIZE) {
        ESP_LOGE(TAG, "Receive cb arg error or buffer overflow");
        return;
    }
    
    // Safely copy data into our global static array before the Wi-Fi driver frees 'data'
    memcpy(master_rx_buf, data, len);
    master_rx_len = len;
   
    // Send the length to the queue to trigger the RS485 task
    if (xQueueSend(from_espnow, &len, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Send queue fail");
    }
}

static esp_err_t espnow_init(void)
{
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(gripper_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(gripper_espnow_recv_cb) );
    
    esp_now_peer_info_t peerInfo = {0};
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    memcpy(peerInfo.peer_addr, master_mac, 6);
    
    ESP_ERROR_CHECK( esp_now_add_peer(&peerInfo) );
    ESP_LOGI(TAG, "ESP-NOW Peer Added Successfully");
    return ESP_OK;
}

void app_main(void)
{  
    // Queues hold the integer 'length' of the data, rather than dangerous pointers
    from_espnow  = xQueueCreate(10, sizeof(int));
    from_rs485   = xQueueCreate(10, sizeof(int));
   
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    if (from_espnow != NULL && from_rs485 != NULL) {
        xTaskCreate(gripper_espnow_task, "gripper_espnow_task", 10480, NULL, 4, NULL);
        xTaskCreate(rs485_communication_task, "rs485_communication_task", rs485_TASK_STACK_SIZE, NULL, rs485_TASK_PRIO, NULL);
    }
}

static void gripper_espnow_task(void *pvParameter)
{
    wifi_init();
    espnow_init();   
    int len;

    while(1) {
        // Wait indefinitely for RS485 to receive data and signal this queue
        if(xQueueReceive(from_rs485, &len, portMAX_DELAY) == pdTRUE) {
            if(esp_now_send(master_mac, gripper_rx_buf, gripper_rx_len) != ESP_OK){
                ESP_LOGE(TAG, "ESP_NOW send failed");
            }
        }
    }
}

static void rs485_communication_task(void *vParameter)
{
    const int uart_num = rs485_UART_PORT;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, rs485_TXD, rs485_RXD, rs485_RTS, rs485_CTS));
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, rs485_READ_TOUT));
    
    int len;

    while(1) {
        // 1. Check for incoming ESP-NOW commands from the Master 
        if(xQueueReceive(from_espnow, &len, pdMS_TO_TICKS(1)) == pdTRUE){
            uart_write_bytes(uart_num, master_rx_buf, master_rx_len); 
        }
         
        // 2. Peek at the UART buffer to see if the Gripper has started replying
        size_t available_bytes = 0;
        uart_get_buffered_data_len(uart_num, &available_bytes);
        
        if(available_bytes > 0) {
            // A packet is arriving! Wait 3ms to guarantee the gripper is finished speaking.
            vTaskDelay(pdMS_TO_TICKS(3)); 
            
            // Now suck up the entire completed packet in one go
            int data_length = uart_read_bytes(uart_num, gripper_rx_buf, BUF_SIZE-1, 0);
            
            if(data_length > 0){
                gripper_rx_len = data_length;
                
                // Signal the ESP-NOW task immediately
                if (xQueueSend(from_rs485, &data_length, pdMS_TO_TICKS(5)) != pdTRUE) {
                    ESP_LOGW(TAG, "Sending to queue failed");
                }
            }
        }
        
        // A minimal 1-tick delay to keep the FreeRTOS watchdog happy
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}