#!/bin/sh
# user = $USER

echo "Adding Bashrc for ATmega16m1"
touch /home/$USER/.avrduderc
cp ./.avrduderc /home/$USER/.avrduderc
