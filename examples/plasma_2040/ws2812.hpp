/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
This code is significantly modified from the PIO ws2812 example
found here: https://github.com/raspberrypi/pico-examples/tree/master/pio/ws2812
*/

#pragma once

#include "ws2812.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

namespace plasma {

const uint SERIAL_FREQ_400KHZ = 400000;
const uint SERIAL_FREQ_800KHZ = 800000;
const uint DEFAULT_SERIAL_FREQ = SERIAL_FREQ_400KHZ;

PIO _pio = pio0;
uint _sm = 0;

uint32_t *dma_read_addr;
uint32_t dma_xfer_count = 0;
int dma_channel;
struct repeating_timer timer;

#pragma pack(push, 1)
union alignas(4) RGB {
    struct {
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t w = 0b00000000;
    } ;
    uint32_t srgb;
    void operator=(uint32_t v) {
        srgb = v;
    };
    void brightness(uint8_t b) {};;
    void rgb(uint8_t r, uint8_t g, uint8_t b) {
        this->r = r;
        this->g = g;
        this->b = b;
    }
};
#pragma pack(pop)

bool dma_timer_callback(struct repeating_timer *t) {
    while(dma_channel_is_busy(dma_channel)) {}; // Block waiting for DMA finish
    dma_channel_set_trans_count(dma_channel, dma_xfer_count, false);
    dma_channel_set_read_addr(dma_channel, dma_read_addr, true);
    return true;
}

static inline void setup_pio(PIO pio, uint sm, uint freq, uint pin, uint ignored=0) {
    (void)ignored;
    _pio = pio;
    _sm = sm;

    uint offset = pio_add_program(pio, &ws2812_program);

    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    
    sm_config_set_out_shift(&c, false, true, 24); // Discard first (APA102 global brightness) byte. TODO support RGBW WS281X LEDs
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    int cycles_per_bit = ws2812_T1 + ws2812_T2 + ws2812_T3;
    float div = clock_get_hz(clk_sys) / (freq * cycles_per_bit);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void start_dma(RGB *buffer, uint num_leds, uint fps=60) {
    uint32_t delay_ms = 1000 / fps;

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(_pio, _sm, true));
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    dma_channel_set_trans_count(dma_channel, num_leds, false);
    dma_channel_set_read_addr(dma_channel, (uint32_t *)buffer, false);
    dma_channel_configure(dma_channel, &config, &_pio->txf[_sm], NULL, 0, false);

    dma_read_addr = (uint32_t *)buffer;
    dma_xfer_count = num_leds;

    add_repeating_timer_ms(-delay_ms, dma_timer_callback, NULL, &timer);
}

static inline bool stop_dma() {
    dma_channel_unclaim(dma_channel);
    return cancel_repeating_timer(&timer);
}

}