#!/bin/bash

# exit on error
set -e

cd ${BASH_SOURCE%/*}  # make sure we run in the folder with the python code

# remove old code
if [ -d ../lib ]; then
    rm -r ../lib
fi

# generate c code
python3 generate_acados_solver_tracking_mpc.py --config solver_tracking_mpc.yaml <<< "y"
