
#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <assert.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/timers.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_now.h"
#include "esp_crc.h"
#include "espnow_gripper.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#define PACKET_READ_TICS        (100 / portTICK_PERIOD_MS)
#define rs485_TASK_STACK_SIZE   (10000)
#define rs485_TASK_PRIO         (10)
#define rs485_UART_PORT         (2)
#define BUF_SIZE                (127)
#define BAUD_RATE               (115200)

#define rs485_TXD   (GPIO_NUM_6)
#define rs485_RXD   (GPIO_NUM_4)
#define rs485_RTS  (GPIO_NUM_5)
#define rs485_CTS   (UART_PIN_NO_CHANGE)
#define rs485_READ_TOUT  (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks

static const char *TAG = "gripper";
static QueueHandle_t gripper_espnow_queue;

static uint8_t master_mac[ESP_NOW_ETH_ALEN] = { 0x70, 0x04, 0x1d, 0xd3, 0x4a, 0xe0 };
static uint16_t gripper_espnow_seq[ESPNOW_DATA_MAX] = { 0, 0 };
const uint8_t *received_data_from_master; 
const uint8_t *received_data_from_gripper; 
int  master_received_data_length =0;
int  received_data_length_gripper =0;


QueueHandle_t from_espnow ;
QueueHandle_t from_rs485 ;
 

static void espnow_deinit(gripper_send_param_t *send_param);

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

#if CONFIG_ESPNOW_ENABLE_LONG_RANGE
    ESP_ERROR_CHECK( esp_wifi_set_protocol(ESPNOW_WIFI_IF, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_LR) );
#endif
}


static void gripper_espnow_send_cb(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    ESP_LOGI("TAG","Send from gripper");
}

static void gripper_espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    ESP_LOGI("TAG","Received from master");
    espnow_event_recv_cb_t recv_from_master;
    uint8_t * mac_addr = recv_info->src_addr;
    bool evt;

    if (mac_addr == NULL || data == NULL || len <= 0) {
        ESP_LOGE(TAG, "Receive cb arg error");
         evt =false;
        return;
    }else{
        evt = true ;
    }
   
     received_data_from_master = data;
     master_received_data_length   = len;
   
    if (xQueueSend(from_espnow, data, pdMS_TO_TICKS(10)) != pdTRUE) {
        ESP_LOGW(TAG, "Send send queue fail");
    }

}

static esp_err_t espnow_init(void)
{
    

    /* Initialize ESPNOW and register sending and receiving callback function. */
    ESP_ERROR_CHECK( esp_now_init() );
    ESP_ERROR_CHECK( esp_now_register_send_cb(gripper_espnow_send_cb) );
    ESP_ERROR_CHECK( esp_now_register_recv_cb(gripper_espnow_recv_cb) );
    esp_now_peer_info_t peerInfo={};
    //register peer
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    //register first peer
    memcpy(peerInfo.peer_addr,master_mac,6);
    ESP_ERROR_CHECK( esp_now_add_peer(&peerInfo) );
    ESP_LOGI(TAG, "After peer information");
    return ESP_OK;
}




static void espnow_deinit(gripper_send_param_t *send_param)
{
    free(send_param->buffer);
    free(send_param);
    vSemaphoreDelete(gripper_espnow_queue);
    esp_now_deinit();
}
uint8_t* received_data;



void app_main(void)
{  
   // Allocate buffers for UART
     
      from_espnow  = xQueueCreate(100, sizeof(int));
      from_rs485   = xQueueCreate(100, sizeof(int));
   
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
     if (from_espnow != NULL && from_rs485 != NULL) {
    xTaskCreate(gripper_espnow_task, "gripper_espnow_task", 10480,NULL, 4, NULL);
    xTaskCreate(rs485_communication_task,"rs485_communication_task",rs485_TASK_STACK_SIZE,NULL,rs485_TASK_PRIO,NULL);
     }
 

}

static void gripper_espnow_task(void *pvParameter)
{
   wifi_init();
   espnow_init();   
   bool rs_485_event;

   uint8_t *received_data_from_rs485 = (uint8_t *) malloc(BUF_SIZE);

     while(1){
     // uint8_t test[5] = "Hello";
      // esp_now_send(master_mac,test,5); 


         if(xQueueReceive(from_rs485, received_data_from_rs485, pdMS_TO_TICKS(10)) == pdTRUE) {
                  ESP_LOGI("ESP_Now", "Received data from queue and now sending over espnow");
                  //int length = strlen((char*)received_data_from_gripper);
                   ESP_LOGI("ESp_NOW", "Data received from rs485 %s",(const char *) received_data_from_gripper);
                  if(esp_now_send(master_mac,received_data_from_gripper,received_data_length_gripper)==ESP_OK){
                    ESP_LOGI("ESP_Now", "Successfully send");
                  }else{
                      ESP_LOGI("ESP_Now", "ESP_NOW faild");
                  }
         }
        
         
    vTaskDelay(10);
}


}


static void rs485_communication_task(void *vParameter){
      
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
 
// Configure UART parameters
ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
ESP_LOGI(TAG, "UART set pins, mode and install driver.");
// Set UART pins as per KConfig settings
ESP_ERROR_CHECK(uart_set_pin(uart_num, rs485_TXD, rs485_RXD, rs485_RTS, rs485_CTS));
 // Set RS485 half duplex mode

ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));

// Set read timeout of UART TOUT feature
ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num,rs485_READ_TOUT));

  // Allocate buffers to receive data from espnow and sending over rs485
   uint8_t* received_espnow_data = (uint8_t*) malloc(BUF_SIZE);
   if(received_espnow_data == NULL){
      ESP_LOGE("ESPNOW", "Failed to allocate memory for espnow receive buffer");
      return;
   }
   // Allocate buffers to receive data from espnow and sending over rs485
   uint8_t* received_rs485_data = (uint8_t*) malloc(BUF_SIZE);
  if(received_rs485_data == NULL){
      ESP_LOGE("ESPNOW", "Failed to allocate memory for rs485 received buffer");
  }
  

    while(1) {
     
          if(xQueueReceive(from_espnow,&received_espnow_data,pdMS_TO_TICKS(10)) == pdTRUE){

             uart_write_bytes(uart_num,received_data_from_master,master_received_data_length); 

           }
         
         int data_length = uart_read_bytes(uart_num,received_rs485_data, BUF_SIZE-1, pdMS_TO_TICKS(20));
         if(data_length>0){
           received_data_from_gripper  = received_rs485_data;
           received_data_length_gripper = data_length;
           received_rs485_data[data_length] = '\0'; //Null-terminated 

            /// ESP_LOGI("UART", "Data received from rs485 %s",(const char *) received_gripper_data);
            if (xQueueSend(from_rs485,received_rs485_data, pdMS_TO_TICKS(20)) != pdTRUE) {
                  ESP_LOGW(TAG, "Sending to  queue failed");
              }else{
                 ESP_LOGI("UART", "Sent data to queue");
              }
        
      
         }

          vTaskDelay(10);
    }


}

 