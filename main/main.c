//INC 1
#include <string.h>
#include <stdio.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include "driver/rmt.h"

//Global Parameters

//RGBW
#define NUM_OF_PIXELS 9
#define NUM_OF_B_IN_PIX 4
//UDP
#define PORT CONFIG_EXAMPLE_PORT
#define UDP_BUFF_LEN NUM_OF_PIXELS*NUM_OF_B_IN_PIX
//Blink
#define BLINK_GPIO 2
//INC 2
#include "rgbw_driver.h"

//Define Tasks Tags
static const char *UDP_SOC_TAG = "UDP";
static const char *MAIN_TAG = "MAIN_TAG";

void rgbwSendTask(void *pvParameter);
static void udpServerTask(void *pvParameters);
void blinkyTask(void *pvParameter);

//FreeRTOS Stuff
TaskHandle_t xtaskBlinky = NULL;
TaskHandle_t xtaskRGBW   = NULL;
TaskHandle_t xtaskUDP    = NULL;
QueueHandle_t xUDPtoRGBWq= NULL;

void app_main(void)
{
    //Connect to WIFI and RUN UDP SERVER TASK
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(example_connect());
    xTaskCreate(&udpServerTask, "udp_server", 4096, (void*)AF_INET, 1, &xtaskUDP);
    //RUN LED Send Task
    xTaskCreate(&rgbwSendTask, "RGBW_SEND", 4*1024, NULL, 2, &xtaskRGBW );
    //
    xTaskCreate(&blinkyTask, "blinky", 2*1024,NULL,3,&xtaskBlinky);
}

void rgbwSendTask(void *pvParameter)
{
    //DEBUG INFO
    ESP_LOGI("RGBW", "Configuring transmitter");
    //Init of RMT periphial
    rmt_tx_int();
    rgbw_welcome_effect(pixels,1,120,3,15);
   
    while (1) 
    {
        if( xUDPtoRGBWq != NULL )
        {
            // Peek a message on the created queue.  Block for 10 ticks if a
            // message is not immediately available.
            if( xQueueReceive( xUDPtoRGBWq, &( pixels ), ( TickType_t ) 10 ) )
                rgbw_write(pixels);
        }
        vTaskDelay(16 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}
static void udpServerTask(void *pvParameters)
{
    //Init VAR
    char rx_buffer[UDP_BUFF_LEN];
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;

    xUDPtoRGBWq = xQueueCreate(10, sizeof( rx_buffer ) );
    if(xUDPtoRGBWq != NULL ) ESP_LOGI(UDP_SOC_TAG, "Created_Q");

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
        timeout.tv_sec = 100;
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
            ESP_LOGI(UDP_SOC_TAG, "Waiting for dat");

            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer), 0, (struct sockaddr *)&source_addr, &socklen);
            // Error occurred during receiving
            ESP_LOGI(UDP_SOC_TAG, "recv bytes: %d", len);
            ESP_LOG_BUFFER_HEX_LEVEL(UDP_SOC_TAG, rx_buffer, sizeof(rx_buffer), 2);
            if (len < 0) 
            {
                ESP_LOGE(UDP_SOC_TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received   
            else 
            {
                //IF Queue is created
                 if(xUDPtoRGBWq != NULL )
                 {
                    //Send Data and catch Errors
                    if( xQueueSend(xUDPtoRGBWq, rx_buffer, (TickType_t)10) != pdPASS)
                    {
                        ESP_LOGE(UDP_SOC_TAG, "Queue Send FAILED!");
                    }

                 }
            }
        }
        //Catch Socket Stop
        if (sock != -1) 
        {
            ESP_LOGE(UDP_SOC_TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}
void blinkyTask(void *pvParameter)
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
}
