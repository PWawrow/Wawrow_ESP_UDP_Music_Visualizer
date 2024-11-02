//INC 1
#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "mdns.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include "driver/rmt.h"

//Global Parameters

//RGBW
#define NUM_OF_PIXELS 72
#define NUM_OF_B_IN_PIX 3
//UDP
#define PORT CONFIG_EXAMPLE_PORT
#define UDP_BUFF_LEN NUM_OF_ROWS*2//2 for 16bit
#define ESP_MDNS_HOSTNAME "ESP_LIGHT"
#define ESP_MDNS_DESC "ESP light music vis"
//Blink
#define BLINK_GPIO 2
//INC 2
#include "rgbw_driver.h"

//Define Tasks Tags
static const char *UDP_SOC_TAG = "UDP";
static const char *MAIN_TAG = "MAIN_TAG";

static void menuTask(void *pvParameter);
static void rgbwSendTask(void *pvParameter);
static void udpServerTask(void *pvParameters);
static void heartBeat(void *pvParameter);
static void start_mdns_service();

//FreeRTOS Stuff
static TaskHandle_t xtaskMenu = NULL;
static TaskHandle_t xtaskHeartBeat = NULL;
static TaskHandle_t xtaskRGBW   = NULL;
static TaskHandle_t xtaskUDP    = NULL;
static QueueHandle_t xUDPtoRGBWq= NULL;
uint8_t bars_vals[NUM_OF_ROWS*2] = {};

SemaphoreHandle_t xMutexUdpRun;
bool UdpRunFlag = false;
void app_main(void)
{
    //Connect to WIFI and RUN UDP SERVER TASK
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    
    xTaskCreate(&menuTask, "Menu", 4*1024, NULL, configMAX_PRIORITIES-1, &xtaskMenu );

    xTaskCreate(&rgbwSendTask, "RGBW_Send", 8*1024, NULL, configMAX_PRIORITIES-2, &xtaskRGBW );

    xTaskCreate(&udpServerTask, "Udp_Server", 4096, (void*)AF_INET, configMAX_PRIORITIES-3, &xtaskUDP);
    //RUN LED Send Task
    xTaskCreate(&heartBeat, "Heart_Beat", 2*1024,NULL,configMAX_PRIORITIES-4,&xtaskHeartBeat);

    xMutexUdpRun = xSemaphoreCreateBinary();
    xUDPtoRGBWq = xQueueCreate(10, UDP_BUFF_LEN);
    if(xUDPtoRGBWq != NULL ) ESP_LOGI(MAIN_TAG, "Created_Q");

    
    start_mdns_service();
}

static void menuTask(void *pvParameter){
   
   vTaskDelay(15000 / portTICK_PERIOD_MS);
     if (xtaskUDP != NULL) {
        vTaskResume(xtaskUDP);
    }
    vTaskDelete(NULL);
}
static void rgbwSendTask(void *pvParameter)
{
    //DEBUG INFO
    ESP_LOGI("RGBW", "Configuring transmitter");
    //Init of RMT periphial
    rmt_tx_int();
    //rgbw_welcome_effect(pixels,1,120,10,50);
   
    while (1) 
    {
        if( xUDPtoRGBWq != NULL )
        {
            // Peek a message on the created queue.  Block for 10 ticks if a
            // message is not immediately available.
            if( xQueueReceive( xUDPtoRGBWq, &( bars_vals ), ( TickType_t ) 10/portTICK_PERIOD_MS ) )
                rgbw_write_bars(bars_vals);
        }
        
    }
    vTaskDelete(NULL);
}
static void udpServerTask(void *pvParameters)
{
    //Init VAR
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    vTaskSuspend(NULL);
    char rx_buffer[UDP_BUFF_LEN];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    bool run = true;
    
    

    while (1) 
    {
        #pragma region //Create Socket + Set Timeout
        
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
        ip_protocol = IPPROTO_IPV6;
    
        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) 
        {
            ESP_LOGE(UDP_SOC_TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(UDP_SOC_TAG, "Socket created");
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 5;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
        
        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) 
        {
            ESP_LOGE(UDP_SOC_TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(UDP_SOC_TAG, "Socket bound, port %d", PORT);
        #pragma endregion
        //Vars to Save Client IP
        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

        while (1) 
        {
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
            if (len < 0) {
                ESP_LOGE(UDP_SOC_TAG, "recvfrom failed: errno %d", errno);
                break;
            }else {
                //IF Queue is created
                    if(xUDPtoRGBWq != NULL )
                    {
                    //Send Data and catch Errors
                    if( xQueueSend(xUDPtoRGBWq, rx_buffer, (TickType_t)10) != pdPASS){
                        ESP_LOGE(UDP_SOC_TAG, "Queue Send FAILED!");
                    }
                    }
            }
            vTaskDelay(16 / portTICK_PERIOD_MS);
        }
    //Catch Socket Stop
    if (sock != -1) {
        ESP_LOGE(UDP_SOC_TAG, "Shutting down socket and restarting...");
        shutdown(sock, 0);
        close(sock);
        memset(rx_buffer,0,sizeof(rx_buffer));
        if(xUDPtoRGBWq != NULL )
        {
        //Send Data and catch Errors
            if( xQueueSend(xUDPtoRGBWq, rx_buffer, (TickType_t)10) != pdPASS){
                ESP_LOGE(UDP_SOC_TAG, "Queue Send FAILED!");
            }
        }
}
    }
    vTaskDelete(NULL);
}
static void heartBeat(void *pvParameter)
{
   
    /* Set the GPIO as a push/pull output */
    
    ESP_LOGI(MAIN_TAG,"Blinky Created");
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while(1) 
    {
        /* Blink off (output low) */
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        /* Blink on (output high) */
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
   // vTaskDelete(NULL);
}
static void start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        printf("MDNS Init failed: %d\n", err);
        return;
    }

    //set hostname
    mdns_hostname_set(ESP_MDNS_HOSTNAME);
    //set default instance
    mdns_instance_name_set(ESP_MDNS_DESC);
}