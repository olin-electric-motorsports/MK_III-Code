cmake_minimum_required (VERSION 3.7)
set (CMAKE_SYSTEM_NAME Generic)
set (CMAKE_SYSTEM_PROCESSOR avr)
set (CMAKE_C_COMPILER avr-gcc)

add_library (CAN_avr_lib STATIC can_api.c)
set_target_properties (
    CAN_avr_lib
    PROPERTIES
        COMPILE_FLAGS "-mmcu=atmega16m1"
    )
