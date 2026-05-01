#!/bin/bash
cd ${BASH_SOURCE%/*}  # make sure we run in the folder with the python code
rm -r ../src/c_generated_code # remove old code
rm ../include/forces_pacejka_mpcc_solver/FORCESNLPsolver*

# make virtual environment with dependencies for FORCES code generation
python3 -m venv .venv
source .venv/bin/activate
pip install casadi==3.5.5 scipy==1.4.1 numpy==1.18.3 requests==2.5

# generate c code
python3 generate_forces_solver.py --config solver.yaml
# move generated c code to source. TODO, maybe move headers to include.
mv FORCESNLPsolver ../src/c_generated_code
mv FORCESNLPsolver_info.h ../src/c_generated_code/include
mv FORCESNLPsolver_info.cpp ../src/c_generated_code/include
cp ../src/c_generated_code/include/* ../include/forces_pacejka_mpcc_solver/

# clean up virtual environment
deactivate
rm -r .venv
