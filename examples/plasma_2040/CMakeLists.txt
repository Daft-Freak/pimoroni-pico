add_executable(plasma2040 plasma2040.cpp)

pico_generate_pio_header(plasma2040 ${CMAKE_CURRENT_LIST_DIR}/apa102.pio)
pico_generate_pio_header(plasma2040 ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_link_libraries(plasma2040
        pico_stdlib
        hardware_pio
        hardware_dma
        breakout_encoder
        pimoroni_i2c
        rgbled
        button
        )

pico_add_extra_outputs(plasma2040)
