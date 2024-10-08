#ifndef RGBW_DRIVER_H // include guard
#define RGBW_DRIVER_H
#pragma once   

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include <math.h>

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO 4

#ifndef NUM_OF_PIXELS
    #define NUM_OF_PIXELS 9
#endif
#ifndef NUM_OF_B_IN_PIX
    #define NUM_OF_B_IN_PIX 4
#endif



//void rgbwSendTask(void *pvParameter);
void rgbw_encode_pixels(uint8_t* pixels, rmt_item32_t* pixels_encoded);
void rmt_tx_int();

extern rmt_item32_t pixels_encoded[(NUM_OF_PIXELS*8*NUM_OF_B_IN_PIX)+2];
extern uint8_t pixels[NUM_OF_PIXELS*NUM_OF_B_IN_PIX];

#endif