#pragma once
#include "pico/stdlib.h"

#include "apa102.hpp"
#include "ws2812.hpp"

namespace plasma {
const uint LED_R = 16;
const uint LED_G = 17;
const uint LED_B = 18;

const uint BUTTON_A = 12;
const uint BUTTON_B = 13;

const uint PIN_CLK = 14; // Used only for APA102
const uint PIN_DAT = 15; // Used for both APA102 and WS2812
}