#!/usr/bin/env bash

set -e              # Exit on error.

printf "downloading casadi\n"
cd /
git clone --branch 3.6.7 https://github.com/casadi/casadi.git --depth=1
cd casadi
printf "building casadi\n"
sudo mkdir -p build
cd build
sudo cmake -D WITH_IPOPT=ON ..
sudo make install -j4
