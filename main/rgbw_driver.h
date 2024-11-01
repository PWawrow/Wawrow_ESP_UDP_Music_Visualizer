#ifndef RGBW_DRIVER_H // include guard
#define RGBW_DRIVER_H
#pragma once   

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_TX_GPIO 4

#ifndef NUM_OF_PIXELS
    #define NUM_OF_PIXELS 72
#endif
#ifndef NUM_OF_ROWS
    #define NUM_OF_ROWS 9
#endif
#ifndef NUM_OF_B_IN_PIX
    #define NUM_OF_B_IN_PIX 3
#endif

#define MVI_AVG_N_VALS  2
/**
 * @brief Encode pixels array [G1,R1,B1,W1,G2,R2,B2,W2,....] to RMT friendly Array.
 *s
 **/
//HELPERS
static uint32_t map32(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
static uint8_t map8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);
static uint8_t* movingAvg(uint8_t* newValues);
static int sumArray(uint8_t* vals, int size);
static void convertU8toU16(uint8_t *input, uint16_t *output, size_t inputSize);

void rmt_tx_int();
void rgbw_encode_pixels(uint8_t* pixels, rmt_item32_t* pixels_encoded);
void rgbw_write(uint8_t* pixels);
void rgbw_welcome_effect(uint8_t* pixels, uint8_t color, uint8_t maxBrightness, uint8_t brightnessStep, uint16_t delay);
void rgbw_write_bars(uint8_t* bar_array);
extern rmt_item32_t pixels_encoded[(NUM_OF_PIXELS*8*NUM_OF_B_IN_PIX)+2];
extern uint8_t pixels[NUM_OF_PIXELS*NUM_OF_B_IN_PIX];

#endif