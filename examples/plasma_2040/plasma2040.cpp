/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <cstdint>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/clocks.h"

#include "common/pimoroni_common.hpp"
#include "breakout_encoder.hpp"
#include "rgbled.hpp"

#include "apa102.pio.h"
#include "ws2812.pio.h"
#include "autorepeat.hpp"

// Uncomment to run APA102 pixels
#define APA102

using namespace pimoroni;

const uint LED_R = 16;
const uint LED_G = 17;
const uint LED_B = 18;

const uint BUTTON_A = 12;
const uint BUTTON_B = 13;

const uint PIN_CLK = 14;
const uint PIN_DIN = 15;


#ifdef APA102
const uint N_LEDS = 120;
const uint BUFFSIZE = N_LEDS + 2;
const uint LEDS_START = 1;
#else
const uint N_LEDS = 26;
const uint BUFFSIZE = N_LEDS;
const uint LEDS_START = 0;
#endif

AutoRepeat ar_button_a;
AutoRepeat ar_button_b;

RGBLED led(LED_R, LED_G, LED_B);

I2C i2c(BOARD::PICO_EXPLORER);
BreakoutEncoder enc(&i2c);

#define SERIAL_FREQ (10 * 1000 * 1000)

enum ENCODER_MODE {
    COLOUR,
    BRIGHTNESS,
    TIME
};

#pragma pack(push, 1)
union alignas(4) RGB {
    struct {
#ifdef APA102
        uint8_t sof = 0b11101111;
        uint8_t b;
        uint8_t g;
        uint8_t r;
#else
        uint8_t r;
        uint8_t g;
        uint8_t b;
        uint8_t sof = 0b00000000;
#endif
    } ;
    uint32_t srgb;
    void operator=(uint32_t v) {
        srgb = v;
    };
    void brightness(uint8_t b) {
        sof = 0b11100000 | b;
    }
};
#pragma pack(pop)

int dma_channel;

RGB buffer[BUFFSIZE];

void set_brightness(uint8_t b) {
    for (auto i = 0u; i < N_LEDS; ++i) {
        buffer[i + LEDS_START].brightness(b);
    }
}

void colour_cycle(uint t) {
    float timer = t / 100.0f;
    float timeg = (t + 150.0f) / 100.0f;
    float timeb = (t + 300.0f) / 100.0f;
    for (auto i = 0u; i < N_LEDS; ++i) {
        buffer[i + LEDS_START].r = sinf(timer + ((float)i / N_LEDS)) * 128 + 64;
        buffer[i + LEDS_START].g = sinf(timeg + ((float)i / N_LEDS)) * 128 + 64;
        buffer[i + LEDS_START].b = sinf(timeb + ((float)i / N_LEDS)) * 128 + 64;
    }
}

void dma_output_buffer(bool block=true) {
    if(dma_channel_is_busy(dma_channel) & !block) return; // Bail immediately if not blocking
    while(dma_channel_is_busy(dma_channel)) {}; // Block waiting for DMA finish
    dma_channel_set_trans_count(dma_channel, BUFFSIZE, false);
    dma_channel_set_read_addr(dma_channel, (uint32_t *)buffer, true);
}

void __isr dma_complete() {
  if (dma_hw->ints0 & (1u << dma_channel)) {
    dma_hw->ints0 = (1u << dma_channel); // clear irq flag
    dma_output_buffer(); // will never block because this is the DMA complete interrupt!
  }
}

bool dma_timer_callback(struct repeating_timer *t) {
    dma_output_buffer(true);
    return true;
}

static inline void setup_ws2812_pio(PIO pio, uint sm, uint pin, float freq) {
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

static inline void setup_apa102_pio(PIO pio, uint sm, uint baud, uint pin_clk, uint pin_din) {
    uint offset = pio_add_program(pio, &apa102_program);

    pio_sm_set_pins_with_mask(pio, sm, 0, (1u << pin_clk) | (1u << pin_din));
    pio_sm_set_pindirs_with_mask(pio, sm, ~0u, (1u << pin_clk) | (1u << pin_din));
    pio_gpio_init(pio, pin_clk);
    pio_gpio_init(pio, pin_din);

    pio_sm_config c = apa102_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin_din, 1);
    sm_config_set_sideset_pins(&c, pin_clk);

    sm_config_set_out_shift(&c, false, true, 32);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // The PIO program transmits 1 bit every 2 execution cycles
    float div = (float)clock_get_hz(clk_sys) / (2 * baud);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void setup_dma_timer(PIO pio, uint sm, uint frame_time_ms=16) {
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    dma_channel_configure(dma_channel, &config, &pio->txf[sm], NULL, 0, false);

    struct repeating_timer timer;
    add_repeating_timer_ms(-frame_time_ms, dma_timer_callback, NULL, &timer);
}

static inline void setup_dma(PIO pio, uint sm, bool use_interrupts=true, bool start=false) {
    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    dma_channel_configure(dma_channel, &config, &pio->txf[sm], NULL, 0, false);

    if(use_interrupts) {
        // Interrupt config
        dma_channel_set_irq0_enabled(dma_channel, true);
        irq_set_enabled(pio_get_dreq(pio, sm, true), true);
        irq_set_exclusive_handler(DMA_IRQ_0, dma_complete);
        irq_set_enabled(DMA_IRQ_0, true);
    }

    if(!start) return;

    // Start the DMA output
    dma_output_buffer();
}

 int main() {
    stdio_init_all();

    gpio_init(BUTTON_A); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_pull_up(BUTTON_B);

#ifdef APA102
    // Set the four bytes of 0x00 SOF and four bytes of 0xFF EOF
    buffer[0] = 0u;             // SOF
    buffer[BUFFSIZE - 1] = ~0u; // EOF
#endif

    PIO pio = pio0;
    uint sm = 0;
#ifdef APA102
    setup_apa102_pio(pio, sm, SERIAL_FREQ, PIN_CLK, PIN_DIN);
    //setup_dma(pio, sm, true, true);
    setup_dma_timer(pio, sm, 16);        // Trigger DMA on a timer, targeting ~60FPS (1000ms / 60 = 16 ... roughly)
#else
    setup_ws2812_pio(pio, sm, PIN_DIN, 400000);
    //setup_dma(pio, sm, false, false);  // Rolling DMA is too fast- data never has time to latch!
    setup_dma_timer(pio, sm, 16);        // Trigger DMA on a timer, targeting ~60FPS (1000ms / 60 = 16 ... roughly)
#endif


    bool encoder_detected = enc.init();
    enc.clear_interrupt_flag();

    uint t = 0;
    uint sleep = 1;
    uint colour = 0;
    bool cycle = true;
    ENCODER_MODE mode = ENCODER_MODE::COLOUR;
    while (true) {
        ++t;
        if(encoder_detected) {
            if(enc.get_interrupt_flag()) {
                uint count = enc.read();
                enc.clear_interrupt_flag();

                while(count < 0) count += 24;

                cycle = false;
                switch(mode) {
                    case ENCODER_MODE::COLOUR:
                        colour = count * 10;
                        break;
                    case ENCODER_MODE::BRIGHTNESS:
                        set_brightness(count);
                        break;
                    case ENCODER_MODE::TIME:
                        sleep = count;
                        break;
                }
                colour_cycle(colour);
            }
        }
        bool a_pressed = ar_button_a.next(t, !gpio_get(BUTTON_A));
        bool b_pressed = ar_button_b.next(t, !gpio_get(BUTTON_B));

        if(b_pressed) cycle = true;

        switch(mode) {
            case ENCODER_MODE::COLOUR:
                led.set_rgb(255, 0, 0);
                if(a_pressed) mode = ENCODER_MODE::BRIGHTNESS;
                break;
            case ENCODER_MODE::BRIGHTNESS:
                led.set_rgb(0, 255, 0);
                if(a_pressed) mode = ENCODER_MODE::TIME;
                break;
            case ENCODER_MODE::TIME:
                led.set_rgb(0, 0, 255);
                if(a_pressed) mode = ENCODER_MODE::COLOUR;
                break;
        }

        if(cycle) colour_cycle(t);

        enc.set_led(buffer[1].r, buffer[1].g, buffer[1].b);
#ifndef APA102
        // If DMA is not trigger on a timer, trigger it manually in the main loop
        //dma_output_buffer();
#endif
        sleep_ms(sleep);
    }
}
