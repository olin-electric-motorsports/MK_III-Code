CMAKE_MINIMUM_REQUIRED (VERSION 2.8)
include (../avr.cmake)

project (Blinky C)

set (L_FUSE 0x62) # Default: 0x62
set(F_CPU 1000000UL)

new_board()
