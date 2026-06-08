#!/usr/bin/env bash

# master on par with tag v79, stable as of date 2023.11.24
PIGPIO_TAG_OR_BRANCH=master
# argument 1 is the absolute path to where the library will be downloaded
if test -d $1; then
    PIGPIO_INSTALL_DIR="$1/pigpio-$PIGPIO_TAG_OR_BRANCH"
else
    printf "No directory found at $1. (for pigpio installation)"
    exit 1
fi

# if dir exists, skip
if test -d $PIGPIO_INSTALL_DIR; then
    printf "Lib [pigpio] destination path non-empty, skipped! ($PIGPIO_INSTALL_DIR)"
    exit 0
fi

# else download lib, compile and install
printf "downloading pigpio\n"
# depth 1: only the specified state
git clone --depth 1 --branch $PIGPIO_TAG_OR_BRANCH https://github.com/joan2937/pigpio.git $PIGPIO_INSTALL_DIR
cd $PIGPIO_INSTALL_DIR
printf "building pigpio\n"
make
sudo make install
