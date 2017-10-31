'''
The official build chain for the 2017-2018 Olin Electric Motorsports FSAE Formula Team
For more information on the team: https://www.olinelectricmotorsports.com/

@author: Peter Seger '20

Released under MIT License 2017
'''


import glob
import os
import re
import sys
import subprocess
import shutil


CC = 'avr-gcc'
PROGRAMMER = 'avrispmkII'
PORT = 'usb'
AVRDUDE = 'avrdude'
OBJCOPY = 'avr-objcopy'
MCU = 'atmega16m1'
PART = 'm16m1'
F_CPU = '4000000UL'
COMPILER = 'gnu99'
FUSE = '0x62'

CFLAGS = '-Os -g -mmcu=' + MCU + ' -std=' + COMPILER + ' -Wall -Werror -ff'
LDFLAG = '-mmcu=' + MCU + ' -lm'
AVRFLAGS = '-p ' + MCU + ' -v -c ' + PROGRAMMER + ' -p ' + PART

possible_boards = ['Dashboard','BMS','Blinky']  # TODO change to figure all boards


def get_input():
    board = input("Board (i.e. Dashboard): ")
    flash = input("Flash (y/n) or Set Fuses(fuses): ")

    # Test Inputs
    if board not in possible_boards:
        print("Not a possible board -%s-"%(board))
        quit()
    if flash != 'y' and flash != 'n' and flash != 'fuses':
        print("Not a possible flash setting -" + flash)
        quit()

    return board, flash


def build_boards_list(boards, head):
    '''
    Goes through the /boards directory and adds each board to a list
    '''
    os.chdir(boards)
    boards = []
    bds = glob.glob('*')
    for el in bds:
        boards.append(el)
    os.chdir(head)
    return boards


def make_libs(head):
    os.chdir('./lib/')
    libs = glob.glob('*.c')
    os.chdir(head)
    return libs


def ensure_setup(board, test, head):
    t = os.listdir(test)
    if 'outputs' not in t:
        os.chdir(test)
        os.system('mkdir outs')
        os.chdir(head)


def make_elf(board, test, libs, head):
    os.chdir(test)
    c_files = glob.glob('*.c')
    h_files = glob.glob('*.h')
    os.system('ls')
    out = CC + ' '
    includes = ''
    for item in c_files:
        includes = includes + str(item) + (' ')
    out = out + includes + LDFLAG + ' -o ' + board + '.elf'
    print(out)
    outs = 'outs/'
    # file = open('test', 'a')    #DEBUG
    # file.write(out+"\n")        #DEBUG
    # file.close()                #DEBUG
    os.system(out)            #Write command to system
    # elf = glob.glob('*.elf')[0]
    # src = test + elf
    # dest = test + 'outs/' + elf
    # shutil.move(src, dest)
    cmd = 'mv *.elf outs/'
    os.system(cmd)
    # os.chdir(outs)
    os.chdir(head)


def make_hex(board, test, libs, head):
    '''
    Takes the elf output files and turns them into hex output
    '''
    # $(BUIDIR)/%.hex: $(BUIDIR)/%.elf
    # $(OBJCOPY) -O ihex -R .eeprom $< $@
    outs = 'outs/'
    os.chdir(test)
    os.chdir(outs)
    elf = glob.glob('*.elf')
    out = OBJCOPY + ' -O ihex -R .eeprom ' + elf[0] + ' ' + board +'.hex'
    # file = open('test', 'a')    #DEBUG
    # file.write(out+"\n")        #DEBUG
    # file.close()                #DEBUG
    os.system(out)            #Write command to system
    os.chdir(head)



def flash_board(board, test, libs, head):
    '''
    Takes hex files and uses ARVDUDE w/ ARVFLAGS to flash code onto board
    '''
    # flash: $(BUIDIR)/$(TARGET).hex
    # sudo $(AVRDUDE) $(AVRFLAGS) -U flash:w:$<
    os.chdir(test)
    os.chdir('outs/')
    hex_file = glob.glob('*.hex')[0]
    out = 'sudo ' + AVRDUDE + ' ' + AVRFLAGS + ' -U flash:v:' + hex_file
    # print(out)          #DEBUG
    os.system(out)      #Write command to systems


def set_fuse():
    '''
    Uses ARVDUDE w/ ARVFLAGS to set the fuse
    '''
    out = 'sudo ' + AVRDUDE + ' ' + AVRFLAGS + ' -U hfuse:w:' + FUSE + ':m'
    # print(out)          #DEBUG
    os.system(out)            #Write command to system

def clean(board, test, head):
    '''
    Goes into given directory and deletes all output files for a clean build
    '''
    outs = 'outs/'
    os.chdir(test)
    os.chdir(outs)
    files = glob.glob('*')
    for f in files:
        os.remove(f)
    os.chdir(head)
    print('Clean Build for %s'%(board))


if __name__ == "__main__":
    # TODO
    '''
    -Make argc input when file called and setup logic flow for flashing, clean, and board building
    '''
    cwd = os.getcwd()

    board, flash = get_input()

    if(flash == 'fuses'):
        set_fuse()
        exit()

    boards = './boards/'
    boards_list = build_boards_list(boards, cwd)    # Get a list of all boards

    test = './boards/%s/'%board

    ensure_setup(board, test, cwd)
    libs = make_libs(cwd)
    clean(board, test, cwd)
    make_elf(board, test, libs, cwd)
    make_hex(board, test, libs, cwd)
    # make_hex(board)

    if(flash == 'y'):
        flash_board(board, test, libs, cwd)
