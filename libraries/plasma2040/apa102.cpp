#include "apa102.hpp"

namespace plasma {

APA102::APA102(uint num_leds, PIO pio, uint sm, uint pin_dat, uint pin_clk, uint freq) : num_leds(num_leds), pio(pio), sm(sm) {
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
    float div = (float)clock_get_hz(clk_sys) / (2 * freq);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);

    dma_channel = dma_claim_unused_channel(true);
    dma_channel_config config = dma_channel_get_default_config(dma_channel);
    channel_config_set_bswap(&config, true);
    channel_config_set_dreq(&config, pio_get_dreq(pio, sm, true));
    channel_config_set_transfer_data_size(&config, DMA_SIZE_32);
    channel_config_set_read_increment(&config, true);
    dma_channel_configure(dma_channel, &config, &pio->txf[sm], NULL, 0, false);

    buffer = new RGB[num_leds];
}

bool APA102::dma_timer_callback(struct repeating_timer *t) {
    ((APA102*)t->user_data)->update();
    return true;
}

void APA102::update(bool blocking) {
    while(dma_channel_is_busy(dma_channel)) {}; // Block waiting for DMA finish
    pio->txf[sm] = 0x00000000; // Output the APA102 start-of-frame bytes
    dma_channel_set_trans_count(dma_channel, num_leds, false);
    dma_channel_set_read_addr(dma_channel, buffer, true);
    if (!blocking) return;
    while(dma_channel_is_busy(dma_channel)) {}; // Block waiting for DMA finish
}

bool APA102::start(uint fps) {
    add_repeating_timer_ms(-(1000 / fps), dma_timer_callback, (void*)this, &timer);
    return true;
}

bool APA102::stop() {
    dma_channel_unclaim(dma_channel);
    return cancel_repeating_timer(&timer);
}

void APA102::set_hsv(uint32_t index, float h, float s, float v) {
    float i = floor(h * 6.0f);
    float f = h * 6.0f - i;
    v *= 255.0f;
    uint8_t p = v * (1.0f - s);
    uint8_t q = v * (1.0f - f * s);
    uint8_t t = v * (1.0f - (1.0f - f) * s);

    switch (int(i) % 6) {
      case 0: buffer[index].rgb(v, t, p); break;
      case 1: buffer[index].rgb(q, v, p); break;
      case 2: buffer[index].rgb(p, v, t); break;
      case 3: buffer[index].rgb(p, q, v); break;
      case 4: buffer[index].rgb(t, p, v); break;
      case 5: buffer[index].rgb(v, p, q); break;
    }
}

void APA102::set_rgb(uint32_t index, uint8_t r, uint8_t g, uint8_t b) {
    buffer[index].rgb(r, g, b);
}

void APA102::set_brightness(uint8_t b) {
    for (auto i = 0u; i < num_leds; ++i) {
        buffer[i].brightness(b);
    }
}

}