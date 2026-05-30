/* ESPNOW Example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
*/
#pragma once // Best practice to prevent double inclusion

#if CONFIG_ESPNOW_WIFI_MODE_STATION
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#else
#define ESPNOW_WIFI_MODE WIFI_MODE_AP
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_AP
#endif
#define ESPNOW_QUEUE_SIZE           6
#define IS_BROADCAST_ADDR(addr) (memcmp(addr, s_example_broadcast_mac, ESP_NOW_ETH_ALEN) == 0)

typedef enum {
    EXAMPLE_ESPNOW_SEND_CB,
    EXAMPLE_ESPNOW_RECV_CB,
} espnow_event_id_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    esp_now_send_status_t status;
} espnow_event_send_cb_t;

typedef struct {
    uint8_t mac_addr[ESP_NOW_ETH_ALEN];
    uint8_t *data;
    int data_len;
} espnow_event_recv_cb_t;

typedef union {
    espnow_event_send_cb_t send_cb;
    espnow_event_recv_cb_t recv_cb;
} espnow_event_info_t;

typedef struct {
    espnow_event_id_t id;
    espnow_event_info_t info;
} espnow_event_t;

enum {
    ESPNOW_DATA_BROADCAST,
    ESPNOW_DATA_UNICAST,
    ESPNOW_DATA_MAX,
};

typedef struct {
    uint8_t type;                         
    uint8_t state;                        
    uint16_t seq_num;                     
    uint16_t crc;                         
    uint32_t magic;                       
    uint8_t payload[0];                   
} __attribute__((packed)) espnow_data_t;

typedef struct {
    bool unicast;                         
    bool broadcast;                       
    uint8_t state;                        
    uint32_t magic;                       
    uint16_t count;                       
    uint16_t delay;                       
    int len;                              
    uint8_t *buffer;                      
    uint8_t dest_mac[ESP_NOW_ETH_ALEN];   
} master_send_param_t;