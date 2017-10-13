import glob
import os
import re
import sys


CC = 'gcc'
PROGRAMMER = 'avrispmkII'
PORT = 'usb'
AVRDUDE = 'avrdude'
OBJCOPY = 'avr-objcopy'
MCU = 'atmega16m1'
F_CPU = '4000000UL'
COMPILER = 'gnu99'

CFLAGS = '-Os -g -mmcu=' + MCU + ' -std=' + COMPILER + ' -Wall -Werror -ff'

possible_boards = ['Dashboard','BMS','Blinky']

def get_input():
    board = input("Board (i.e. Dashboard): ")
    flash = input("Flash (y/n): ")

    # Test Inputs
    if board not in possible_boards:
        print("Not a possible board -%s-"%(board))
        quit()
    if flash != 'y' and flash !='n':
        print("Not a possible flash setting -" + flash)
        quit()

    return board, flash

if __name__ == "__main__":
    # possible_boards = find_board()  # Go through all the folders and find the boards
    board, flash = get_input()

    test = './boards/%s/*'%board
    print(test)
    t = glob.glob(test)
    main = t[1].split('/')
    print(main)
