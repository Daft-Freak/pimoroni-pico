/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
This code is significantly modified from the PIO apa102 example
found here: https://github.com/raspberrypi/pico-examples/tree/master/pio/apa102
*/

#pragma once

#include "apa102.pio.h"

#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

namespace plasma {

const uint DEFAULT_SERIAL_FREQ = 20 * 1000 * 1000; // 20MHz

PIO _pio = pio0;
uint _sm = 0;

uint32_t *dma_read_addr;
uint32_t dma_xfer_count = 0;
int dma_channel;
struct repeating_timer timer;

#pragma pack(push, 1)
union alignas(4) RGB {
    struct {
        uint8_t sof = 0b11101111;
        uint8_t b;
        uint8_t g;
        uint8_t r;
    } ;
    uint32_t srgb;
    void operator=(uint32_t v) {
        srgb = v;
    };
    void brightness(uint8_t b) {
        sof = 0b11100000 | b;
    };
    void rgb(uint8_t r, uint8_t g, uint8_t b) {
        this->r = r;
        this->g = g;
        this->b = b;
    }
};
#pragma pack(pop)

bool dma_timer_callback(struct repeating_timer *t) {
    while(dma_channel_is_busy(dma_channel)) {}; // Block waiting for DMA finish
    _pio->txf[_sm] = 0x00000000; // Output the APA102 start-of-frame bytes
    dma_channel_set_trans_count(dma_channel, dma_xfer_count, false);
    dma_channel_set_read_addr(dma_channel, dma_read_addr, true);
    return true;
}

void __isr dma_complete() {
  if (dma_hw->ints0 & (1u << dma_channel)) {
    dma_hw->ints0 = (1u << dma_channel); // clear irq flag
    _pio->txf[_sm] = 0xFFFFFFFF; // Output the APA102 end-of-frame bytes
  }
}

static inline void setup_pio(PIO pio, uint sm, uint baud, uint pin_dat, uint pin_clk) {
    _pio = pio;
    _sm = sm;

    uint offset = pio_add_program(pio, &apa102_program);

    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_clk) | (1u << pin_dat));
    pio_sm_set_pindirs_with_mask(pio, sm, ~0u, (1u << pin_clk) | (1u << pin_dat));
    pio_gpio_init(pio, pin_clk);
    pio_gpio_init(pio, pin_dat);

    pio_sm_config c = apa102_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin_dat, 1);
    sm_config_set_sideset_pins(&c, pin_clk);

    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // The PIO program transmits 1 bit every 2 execution cycles
    float div = (float)clock_get_hz(clk_sys) / (2 * baud);
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
    dma_channel_configure(dma_channel, &config, &_pio->txf[_sm], NULL, 0, false);

    dma_read_addr = (uint32_t *)buffer;
    dma_xfer_count = num_leds;

    add_repeating_timer_ms(-delay_ms, dma_timer_callback, NULL, &timer);

    /* This outputs a white pixel at the end of the chain, I'm not convinced it's necessary to latch data
    // Use the DMA complete interrupt to send the APA102 end-of-frame bytes
    dma_channel_set_irq0_enabled(dma_channel, true);
    irq_set_enabled(pio_get_dreq(_pio, _sm, true), true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_complete);
    irq_set_enabled(DMA_IRQ_0, true);
    */
}

static inline bool stop_dma() {
    dma_channel_unclaim(dma_channel);
    return cancel_repeating_timer(&timer);
}

}