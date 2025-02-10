cmake_minimum_required(VERSION 3.13)

# initialize pico-sdk from submodule
# note: this must happen before project()
include(pico-sdk/pico_sdk_init.cmake)

project(flash_led_project)

pico_sdk_init()

add_executable(flash_led
    flash-led.c
)

target_link_libraries(flash_led pico_stdlib)
pico_add_extra_outputs(flash_led)
