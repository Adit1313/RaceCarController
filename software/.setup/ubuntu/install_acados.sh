#!/usr/bin/env bash

set -e # Exit on error. Important if git clone fails.

printf "downloading acados\n"
cd /
git clone --branch v0.5.3 https://github.com/acados/acados.git --depth=1
cd acados
git submodule update --recursive --init --depth=1

printf "building acados\n"
sudo mkdir -p build
cd build
sudo cmake -DACADOS_WITH_QPOASES=ON -DACADOS_SILENT=ON ..
sudo make install -j$(nproc --all)

printf "install acados_template python package\n"
cd /
sudo pip3 install acados/interfaces/acados_template
