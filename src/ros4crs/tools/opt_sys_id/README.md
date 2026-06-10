# Optimization-based system identification for race cars

This repository allows the user to perform system identification using a nonlinear programming approach. It is made to be expandible, allowing users to add their own parametric models. The use of the library is shown in a variety of demonstration jupyter notebooks.

## Structure
    .
    ├── datasets
    │   └── real                    # Contains data sets recorded in real-world experiments
    ├── notebooks                   # Contains jupyter notebooks used to demonstrate the use of the library
    │   └── *.ipynb                 # Jupyter notebooks used to demonstrate the use of the library
    ├── src
    │   ├── opt_sys_id              # The main library for system identification
    │   │   ├── model               # Used to define system models
    │   │   ├── optimizer           # Implementation of the system identification algorithm
    │   │   ├── sensor_model        # Used to define measurement functions
    │   │   ├── simulator           # A simple simulator able to produce open-loop and closed-loop data
    │   │   ├── state_estimator     # An optimization-based state estimator used for validation
    │   │   └── ...
    │   ├── bag_to_np               # A script to extract data from rosbags
    │   └── ...
    └── ...

## Demo files
The repo contains a demonstration notebook that show how the library can be used to run the system identification procedure and visualizes some of the results.

- `opt_demo_real.ipynb`: This notebook loads two data sets collected on the real system. One data set is used as a training data set to find a set of parameters. The second data set is used for validation. Validation is achieved by using both the system parameters used as a prior and the posterior system parameters to generate state predictions for a fixed time horizon. Those predictions are then plotted and the prediction MSE is calculated.
- `opt_demo_sim.ipynb`: This file loads two simulated data sets created using the same target trajectory but with different amounts of process and measurement noise. System identification is preformed on both data sets, using a perturbed version of the true system parameters as prior parameters. The parameters are then validated by plotting predictions made with both the prior and the posterior parameters and by calculating the resulting prediction MSEs.

## How to perform system identification
1. Create an optimizer object, speficying the model, the sampling rate `dt` and the length of the data set `N`. The following optimizers are available:
    1. `MeasurementOptimizer`: This optimizer assumes the measurements to be made as specified in the provided sensor models.
    1. `SubtrajectoryOptimiuer` (*recommended*): Works the same as the `MeasurementOptimizer`, but splits the state trajectory into decoupled segments of length `M`.
1. Use the `find_params` method to perform the optimization. The following arguments are required to perform the optimization.
    1. `y`: An array conatining all measurements as column vectors. Its shape is `(num_measurements, N+1)`. The `FullStateOptimizer` requires the state trajectory `x` of shape `(num_states, N+1)` instead.
    1. `u`: An array containing all inputs as column vectors. Its shape is `(num_inputs, N)`.
    1. `P`: The parameter regularization cost matrix of shape `(num_params, num_params)`.
    1. `Q`: The state error cost matrix of shape `(num_states, num_states)`.
    1. `R`: The measurement error cost matrix of shape `(num_measurements, num_measurements)`.
    1. `S`: The initial state regularization cost matrix of shape `(num_states, num_states)`. Can be set to zero if no initial state regularization is desired.
    1. `t` (*optional*): An array of shape `(N+1)`, containing the timestamps corresponding to the measurements.

    Follow the instructions on how to use `bag_to_np` in order to extract data from rosbags and how to bring it into the correct format.
1. The resulting parameters are stored as the instance variable `theta_hat`.

Refer to `src/opt_demo_real.ipynb` for an example of this procedure in practice.

## How to extract data from rosbags

The optimization framework requires the measurements and state inputs to be provided as `numpy` arrays. The script `bag_to_np` can be used to extract data from rosbags and store them in the desired format. To do so, run the script, specifying the path to the rosbags in question:
```
./bag_to_np --m [measurements] --ns [namespace] -- <path-to-bag1> <path-to-bag2> ...
```

Run `./bag_to_np -h` to see all available options. The script extracts the data and stores the arrays in individual folders at `src/datasets/real`. Some modification may be necessary depending on the topics and message types used.

The script extracts the measurements including their timestamps. The timestamps are then used to find the most recent system input at that time.

## How to add new models
1. Create a new directory in `src/opt_sys_id/model` with the name of the new model you'd like to add.
1. Inside the newly created directory, add `<model_name>.py`. This file should contain a child class of `Model`. The following methods need to be implemented:
    1. `__init__`: Use this method to specify the values of the variables `num_params`, `num_states`, `num_inputs` and `num_measurements`. Also, you can specify a set of parameters that should not be optimized, by storing their indices in the `fixed_params` variable.
    1. `calc_dx`: This method should be used to specify the *dynamic model* of the system in continuous time. Use the current state `x`, inputs `u` and parameters `params` to calculate $\dot{x}(x,u,\theta)$. If `numerical` is set to `False`, the calculations should be done using `casadi` commands, so they can be used by the optimizer.
    1. `calc_y` (*optional*): This method is used to specify the *measurement model*. This method can be omitted if the full state vector is observed. If `numerical` is set to `False`, the calculations should be done using `casadi` commands, so they can be used by the optimizer.
    1. `initial_estimate` (*optional*): This method is used to create an initial estimate of the state trajectory for the optimizer to use, given all observations `y` and state inputs `u`. The default for this method is an all-zero state trajectory. Overwrite this method if evaluating the dynamic model for an all-zero state can lead to numerical problems or if you with to provide a better initial estimate.
1. Optionally, add a file `structures.py`, which can be used to define some structures that make handling the model a bit easier. The same could also directly be done in `<model_name>.py` if preferred.
1. Finally, add a file `__init__.py`, in which you import all the created classes and other methods you would like to make public.

Refer to `src/opt_sys_id/models/awd_model` for an example of a model that has been added using the described process.
