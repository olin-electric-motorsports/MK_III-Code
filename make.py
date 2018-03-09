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
import time


CC = 'avr-gcc'
PROGRAMMER = 'avrispmkII'
PORT = 'usb'
AVRDUDE = 'avrdude'
OBJCOPY = 'avr-objcopy'
MCU = 'atmega16m1'
PART = 'm16'
F_CPU = '4000000UL'
COMPILER = 'gnu99'
FUSE = '0x62'

CFLAGS = '-Os -g -mmcu=' + MCU + ' -std=' + COMPILER + ' -Wall -Werror -ff'
LDFLAG = '-mmcu=' + MCU + ' -lm -std=' + COMPILER
AVRFLAGS = '-p -B2 ' + MCU + ' -v -c ' + PROGRAMMER + ' -p ' + PART

possible_boards = []


def get_input():
    board = input("Board (i.e. Dashboard) or build All (all): ")
    flash = input("Flash (y/n) or Set Fuses(fuses): ")
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


def ensure_setup(board, dir, head):
    t = os.listdir(dir)
    if 'outputs' not in t:
        os.chdir(dir)
        os.system('mkdir outs')
        os.chdir(head)


def make_elf(board, dir, libs, head):
    os.chdir(dir)
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
    os.system(out)            #Write command to system
    cmd = 'mv *.elf outs/'
    os.system(cmd)
    os.chdir(head)


def make_hex(board, dir, libs, head):
    '''
    Takes the elf output files and turns them into hex output
    '''
    outs = 'outs/'
    os.chdir(dir)
    os.chdir(outs)
    elf = glob.glob('*.elf')
    out = OBJCOPY + ' -O ihex -R .eeprom ' + elf[0] + ' ' + board +'.hex'
    os.system(out)            #Write command to system
    os.chdir(head)



def flash_board(board, dir, libs, head):
    '''
    Takes hex files and uses ARVDUDE w/ ARVFLAGS to flash code onto board
    '''
    os.chdir(dir)
    os.chdir('outs/')
    hex_file = glob.glob('*.hex')[0]
    out = 'sudo ' + AVRDUDE + ' ' + AVRFLAGS + ' -U flash:w:' + hex_file
    os.system(out)      #Write command to systems


def set_fuse():
    '''
    Uses ARVDUDE w/ ARVFLAGS to set the fuse
    '''
    out = 'sudo ' + AVRDUDE + ' ' + AVRFLAGS + ' -U lfuse:w:' + FUSE + ':m'
    os.system(out)            #Write command to system

def clean(board, dir, head):
    '''
    Goes into given directory and deletes all output files for a clean build
    '''
    outs = 'outs/'
    os.chdir(dir)
    os.chdir(outs)
    files = glob.glob('*')
    for f in files:
        os.remove(f)
    os.chdir(head)
    print('Clean Build for %s'%(board))

def check_build_date(board, dir, head):
    # TODO
    '''
    Checks to see whether or not the .c or .h files have been modified since the time
    when the output files were made. If not, it returns false otherwise it returns true
    to indicate the need to rebuild
    '''
    sub_head = dir
    os.chdir(dir + 'outs/')
    outs = glob.glob('*')
    if len(outs) < 2:      # No output files made, assumed new board or error
        os.chdir(head)
        return True
    else:
        out_time = os.path.getctime(outs[0])
        os.chdir(dir)
        c_files = glob.glob('*.c')
        h_files = glob.glob('*.h')
        if len(c_files) < 1:
            print('No C files found at %s',dir)
            os.chdir(head)
            return True
        else:
            c_time = os.path.getctime(files[0])
            # if <= c_time <

        print(out_time)
        os.chdir(head)

def make_all(head, boards):
    board_head = './boards/'
    for board in boards:
        dir = './boards/%s/'%board
        ensure_setup(board, dir, cwd)
        libs = make_libs(cwd)
        clean(board, dir, cwd)
        make_elf(board, dir, libs, cwd)
        make_hex(board, dir, libs, cwd)
    print('-------------------------------------')
    print('Build successful! No boards flashed.')
    os.chdir(head)


def get_includes(head, board):
    '''
    This function gathers all the header files from the lib folder for building
    '''
    out  = 'cp lib/* %s'%board
    os.system(out)

def remove_includes(head, board):
    '''
    This function removes the lib files from the directory
    '''
    os.chdir(head + '/lib/')
    includes = glob.glob('*')
    os.chdir(head)
    os.chdir(board)
    out = 'rm '
    for x in includes:
        out = out + x + ' '
    os.system(out)
    os.chdir(head)


if __name__ == "__main__":
    # TODO
    '''
    -Make argc input when file called and setup logic flow for flashing, clean, and board building
    '''
    cwd = os.getcwd()
    boards = './boards/'
    possible_boards = build_boards_list(boards, cwd)    # Get a list of all boards

    board = input("Board (i.e. Dashboard) or build All (all): ")
    if(board == 'all'):
        make_all(cwd, possible_boards)
    else:
        flash = input("Flash (y/n) or Set Fuses(fuses): ")

        if(flash == 'fuses'):
            set_fuse()
            exit()

        if board in possible_boards:
            dir = './boards/%s/'%board

            # check_build_date(board, dir, cwd)

            ensure_setup(board, dir, cwd)
            libs = make_libs(cwd)
            clean(board, dir, cwd)
            get_includes(cwd, dir)
            make_elf(board, dir, libs, cwd)
            make_hex(board, dir, libs, cwd)
            remove_includes(cwd, dir)

            if(flash == 'y'):
                flash_board(board, dir, libs, cwd)
        else:
            print("Not a possible board --%s--"%(board))
