#include "rgbw_driver.h"


uint8_t pixels[NUM_OF_PIXELS*NUM_OF_B_IN_PIX] = 
{

};

rmt_item32_t pixels_encoded[(NUM_OF_PIXELS*8*NUM_OF_B_IN_PIX)+2] = {};

rmt_item32_t bit0 =             {{{3, 1, 9, 0}}};
rmt_item32_t bit1 =             {{{6, 1, 6, 0}}};
rmt_item32_t reset =            {{{0, 0, 800, 0}}};
rmt_item32_t rmt_end_marker =   {{{0, 0, 0, 0}}};

static const char *RMT_TX_TAG = "RMT Tx";



void rgbw_encode_pixels(uint8_t* pixels, rmt_item32_t* pixels_encoded)
{

    for(int j = 0; j < NUM_OF_PIXELS*NUM_OF_B_IN_PIX; j++)
    {
       // ESP_LOGI("RGBW", "RECEIVE_Q %d",*pixels);
        
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