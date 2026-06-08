#!/usr/bin/env bash

printf "Installing L4acados\n"
cd /
git clone https://github.com/IntelligentControlSystems/l4acados.git
cd l4acados
git checkout 5747ad9e37993e7a3e2d246e61df7d34a0e3cc0a
printf "Cloned L4acados\n"

cd /
# TODO: Installation with optional dependencies [gpytorch-exo] from package directly
pip3 install git+https://github.com/naefjo/linear_operator@feature/exo-gp || exit 1
pip3 install git+https://github.com/naefjo/gpytorch@feature/exo-gp || exit 1
# TODO: editable installation for easier debugging (does not work with setuptools currently)
pip3 install /l4acados || exit 1
printf "Installed L4acados\n"
