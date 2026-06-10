#!/bin/bash
cd ${BASH_SOURCE%/*}  # make sure we run in the folder with the python code
rm -r ../lib # remove old code

# generate c code
python3 generate_acados_solver.py --config solver.yaml <<< "y"
