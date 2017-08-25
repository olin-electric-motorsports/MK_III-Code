#!/bin/bash

echo "Setting up the repo... (ie. building everything)"

# TODO: Add a way to check if the correct programs are installed

root_dir=$(pwd) # Store root directory
echo "$root_dir"
for DIRECTORY in $(find ./boards/ -maxdepth 1 -mindepth 1 -type d)
do
    echo "$DIRECTORY"
    cd $DIRECTORY

    # Check if build is already made
    if [ ! -d "build" ]; then
        mkdir build
        cd build

        cmake -D CMAKE_C_COMPILER=avr-gcc .. # Run cmake out-of-source
    else
        echo "Build directory already exists for $DIRECTORY. Please \`rm -r $DIRECTORY/build\` if you wish to do a fresh build. This will just update the build for now."
        cd build
        cmake -D CMAKE_C_COMPILER=avr-gcc .. # Run cmake out-of-source
    fi

    cd $root_dir
done
