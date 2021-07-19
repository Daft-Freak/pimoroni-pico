/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <cstdint>

#include "pico/stdlib.h"

// Pick *one*!
#include "apa102.hpp"
//#include "ws2812.hpp"

// Set how many LEDs you have
const uint N_LEDS = 120;

#include "common/pimoroni_common.hpp"
#include "breakout_encoder.hpp"
#include "rgbled.hpp"

#include "autorepeat.hpp"

using namespace pimoroni;

const uint LED_R = 16;
const uint LED_G = 17;
const uint LED_B = 18;

const uint BUTTON_A = 12;
const uint BUTTON_B = 13;

const uint PIN_CLK = 14; // Used only for APA102
const uint PIN_DAT = 15; // Used for both APA102 and WS2812

const float pi = 3.14159265358979323846f;

AutoRepeat ar_button_a;
AutoRepeat ar_button_b;

RGBLED led(LED_R, LED_G, LED_B);

I2C i2c(BOARD::PICO_EXPLORER);
BreakoutEncoder enc(&i2c);

enum ENCODER_MODE {
    COLOUR,
    ANGLE,
    BRIGHTNESS,
    TIME
};

plasma::RGB buffer[N_LEDS];

void hsv_to_rgb(plasma::RGB *led, float h, float s, float v) {
    float i = floor(h * 6.0f);
    float f = h * 6.0f - i;
    v *= 255.0f;
    uint8_t p = v * (1.0f - s);
    uint8_t q = v * (1.0f - f * s);
    uint8_t t = v * (1.0f - (1.0f - f) * s);

    switch (int(i) % 6) {
      case 0: led->r = v; led->g = t; led->b = p; break;
      case 1: led->r = q; led->g = v; led->b = p; break;
      case 2: led->r = p; led->g = v; led->b = t; break;
      case 3: led->r = p; led->g = q; led->b = v; break;
      case 4: led->r = t; led->g = p; led->b = v; break;
      case 5: led->r = v; led->g = p; led->b = q; break;
    }
}

void set_brightness(uint8_t b) {
    for (auto i = 0u; i < N_LEDS; ++i) {
        buffer[i].brightness(b);
    }
}

void colour_cycle(float hue, float t, float angle) {
    t /= 200.0f;

    for (auto i = 0u; i < N_LEDS; ++i) {
        float offset = (pi * i) / N_LEDS;
        offset = sinf(offset + t) * angle;
        hsv_to_rgb(&buffer[i], (hue + offset) / 360.0f, 1.0f, 1.0f);
    }
}

void gauge(uint v, uint vmax = 100) {
    uint light_pixels = N_LEDS * v / vmax;

    for (auto i = 0u; i < N_LEDS; ++i) {
        if(i < light_pixels) {
            buffer[i].rgb(0, 255, 0);
        } else {
            buffer[i].rgb(255, 0, 0);
        }
    }
}

int main() {
    stdio_init_all();

    gpio_init(BUTTON_A); gpio_pull_up(BUTTON_A);
    gpio_init(BUTTON_B); gpio_pull_up(BUTTON_B);

    plasma::setup_pio(pio0, 0, plasma::DEFAULT_SERIAL_FREQ, PIN_DAT, PIN_CLK);

    // Trigger DMA on a timer, targeting ~60FPS
    plasma::start_dma(buffer, N_LEDS, 60);

    bool encoder_detected = enc.init();
    enc.clear_interrupt_flag();

    int speed = 50;
    float hue = 0;
    int angle = 120;
    int8_t brightness = 16;
    bool cycle = true;
    ENCODER_MODE mode = ENCODER_MODE::COLOUR;
    while (true) {
        uint32_t t = millis();
        if(encoder_detected) {
            if(enc.get_interrupt_flag()) {
                int count = enc.read();
                enc.clear_interrupt_flag();
                enc.clear();

                cycle = false;
                switch(mode) {
                    case ENCODER_MODE::COLOUR:
                        hue += count;
                        brightness = std::min((int8_t)359, brightness);
                        brightness = std::max((int8_t)0, brightness);
                        colour_cycle(hue, 0, (float)angle);
                        break;
                    case ENCODER_MODE::ANGLE:
                        angle += count;
                        angle = std::min((int)359, angle);
                        angle = std::max((int)0, angle);
                        colour_cycle(hue, 0, (float)angle);
                        break;
                    case ENCODER_MODE::BRIGHTNESS:
                        brightness += count;
                        brightness = std::min((int8_t)31, brightness);
                        brightness = std::max((int8_t)0, brightness);
                        set_brightness(brightness);
                        gauge(brightness, 31);
                        break;
                    case ENCODER_MODE::TIME:
                        speed += count;
                        speed = std::min((int)100, speed);
                        speed = std::max((int)0, speed);
                        gauge(speed, 100);
                        break;
                }
            }
        }
        bool a_pressed = ar_button_a.next(t, !gpio_get(BUTTON_A));
        bool b_pressed = ar_button_b.next(t, !gpio_get(BUTTON_B));

        if(b_pressed) cycle = true;

        switch(mode) {
            case ENCODER_MODE::COLOUR:
                led.set_rgb(255, 0, 0);
                if(a_pressed) mode = ENCODER_MODE::ANGLE;
                break;
            case ENCODER_MODE::ANGLE:
                led.set_rgb(255, 255, 0);
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

        if(cycle) colour_cycle(hue, t * speed / 100, (float)angle);

        enc.set_led(buffer[1].r, buffer[1].g, buffer[1].b);

        // Sleep time controls the rate at which the LED buffer is updated
        // but *not* the actual framerate at which the buffer is sent to the LEDs
        sleep_ms(1000 / 60);
    }
}
