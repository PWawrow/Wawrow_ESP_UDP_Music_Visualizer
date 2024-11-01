#include "rgbw_driver.h"


static uint16_t bars16Bit[NUM_OF_ROWS];
uint8_t pixels[NUM_OF_PIXELS*NUM_OF_B_IN_PIX] = 
{

};
uint8_t pixels_empty[NUM_OF_PIXELS*NUM_OF_B_IN_PIX] = 
{

};

rmt_item32_t pixels_encoded[(NUM_OF_PIXELS*8*NUM_OF_B_IN_PIX)+2] = {};

rmt_item32_t bit0 =             {{{3, 1, 9, 0}}};
rmt_item32_t bit1 =             {{{6, 1, 6, 0}}};
rmt_item32_t reset =            {{{0, 0, 800, 0}}};
rmt_item32_t rmt_end_marker =   {{{0, 0, 0, 0}}};

static const char *RMT_TX_TAG = "RMT Tx";
#define MAX_VAL_FROM_FFT 1000
#define MAX_BRIGHTNES 100


void rgbw_encode_pixels(uint8_t* pixels, rmt_item32_t* pixels_encoded)
{
    for(int j = 0; j < NUM_OF_PIXELS*NUM_OF_B_IN_PIX; j++)
    {
        for (int k = 7; k >= 0; k--) {
            if((*pixels >> k) & 1)
            {
                *pixels_encoded = bit1;
            } else
            {
                *pixels_encoded = bit0;
            }
            pixels_encoded++;
        }
        pixels++;
    }
    *pixels_encoded = reset;
    pixels_encoded++;
    *pixels_encoded = rmt_end_marker;

}
void rgbw_write(uint8_t* pixels)
{
   
    rgbw_encode_pixels(pixels, pixels_encoded);
    ESP_ERROR_CHECK(rmt_write_items(RMT_TX_CHANNEL, pixels_encoded, sizeof(pixels_encoded), 0));
    
}

void rgbw_write_bars(uint8_t* bar_array8U){
    memcpy(pixels,pixels_empty,NUM_OF_PIXELS*NUM_OF_B_IN_PIX);
    convertU8toU16(bar_array8U,bars16Bit,NUM_OF_ROWS*2);
    for(int i = 0; i<NUM_OF_ROWS;i++){
        for(int j = 0;j<NUM_OF_PIXELS/NUM_OF_ROWS;j++){
            int val = bars16Bit[i] - (MAX_VAL_FROM_FFT/NUM_OF_ROWS)*j;
            if(val > 0 && val < MAX_BRIGHTNES)
            {
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+1] = (uint8_t)val;
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+2] = (uint8_t)val;
            }
            if(val<0)
            {
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+1] = 0;
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+2] = 0;
            }
            if(val>MAX_BRIGHTNES)
            {
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+1] = (uint8_t)MAX_BRIGHTNES;
                pixels[(i*(NUM_OF_PIXELS/NUM_OF_ROWS)+j)*NUM_OF_B_IN_PIX+2] = (uint8_t)MAX_BRIGHTNES;
            }
        }

    }
    rgbw_write(pixels);
}
void rgbw_welcome_effect(uint8_t* pixels,uint8_t color, uint8_t maxBrightness, uint8_t brightnessStep, uint16_t delay)
{
    uint8_t currVal = 0;
    for(int i = 0; i < NUM_OF_PIXELS/NUM_OF_ROWS; i++)
    {
        while(currVal < maxBrightness)
        {
            
            currVal += brightnessStep;
            for(int j = 0; j < NUM_OF_ROWS; j++)
                pixels[(i+(NUM_OF_PIXELS/NUM_OF_ROWS)*j)*NUM_OF_B_IN_PIX+color] = currVal;
            rgbw_write(pixels);
            vTaskDelay(delay/portTICK_PERIOD_MS);
        }
        currVal = 0;
        
    }
    for(int i = 0; i < NUM_OF_PIXELS*NUM_OF_B_IN_PIX; i++)
        pixels[i] = 0;
    rgbw_write(pixels);
}
void rmt_tx_int()
{
    rmt_config_t config;
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_TX_CHANNEL;
    config.gpio_num = RMT_TX_GPIO;
    config.mem_block_num = 1;
    config.tx_config.loop_en = 1;
    config.tx_config.carrier_en = 1;
    config.tx_config.idle_output_en = 1;
    config.tx_config.idle_level = (rmt_idle_level_t)0;
    config.tx_config.carrier_duty_percent = 50;
    config.tx_config.carrier_freq_hz = 10000;
    config.tx_config.carrier_level = (rmt_carrier_level_t)0;
    config.tx_config.carrier_en = 0;
    // set the maximum clock divider to be able to output
    // RMT pulses in range of about one hundred nanoseconds
    // a divider of 8 will give us 80 MHz / 8 = 10 MHz -> 0.1 us
    config.clk_div = 8;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

//HELPER FUNCTIONS
static void convertU8toU16(uint8_t *input, uint16_t *output, size_t inputSize) {
    for (size_t i = 0; i < inputSize / 2; i++) {
        output[i] = (input[2 * i + 1] << 8) | input[2 * i];
    }
}
static int sumArray(uint8_t* vals, int size)
{
    int sum = 0;
    for(int i=0;i<size;i++)sum+=vals++;
    return sum;
}
static uint32_t map32(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static uint8_t map8(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
static uint8_t* movingAvg(uint8_t* newValues) {
            

    static int current[NUM_OF_ROWS] = {};            // Index for current value
    static int cvalues[NUM_OF_ROWS] = {};            // Count of values read (<= nvalues)
    static int sum[NUM_OF_ROWS] = {};               // Rolling sum
    static uint8_t values[NUM_OF_ROWS][MVI_AVG_N_VALS]={{},{}};
    static uint8_t output[NUM_OF_ROWS]= {};;

    for(int i =0; i<NUM_OF_ROWS;i++){
        sum[i] += newValues[i];
        // If the window is full, adjust the sum by deleting the oldest value
        if (cvalues[i] == MVI_AVG_N_VALS)
        sum[i] -= values[i][current[i]];

        values[i][current[i]] = newValues[i];          // Replace the oldest with the latest

        if (++current[i] >= MVI_AVG_N_VALS)
        current[i] = 0;

        if (cvalues[i] < MVI_AVG_N_VALS)
        cvalues[i] += 1;
        output[i] = sum[i]/cvalues[i];
    }

    return output;
}