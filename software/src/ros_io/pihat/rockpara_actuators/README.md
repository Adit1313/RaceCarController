# Actuators driver

### PIGPIO lib Installation
Need this once
```
# unzip to lib/
cd lib/
unzip pigpio-master.zip -d .

# make and install
cd pigpio-master/
make
sudo make install
```

### The device driver PCA9685
* `pca9685.h`
* `pca9685.cpp`

The `main()` in the cpp moves the channel 0 from PWM_LENGTH_MIN to PWM_LENGTH_MAX (defined in `pca9685.h`).
To test the device driver directly with the `main()`:
```
cd src/
g++ -I[path/to/]rockpara_actuators/include -Wall -pthread -o tst_pca9685 pca9685.cpp -lpigpio -lrt

# after compiling
./tst_pca9685
```

(Check the file to enable interactive channel and inputs)

Use the interactive channels to find the min and max PWM inputs for the servos.

### The ROS wrapper node

Check the `./config/rocket_default_params.yaml` file for parameters. Set the channel and min/max PWM values.

```
catkin build rockpara_actuators
source [path/to/]devel/setup.bash
roslaunch rockpara_actuators rocket_default.launch
```

And the input topic (for now) is:
```
$ rostopic info /rockpara_actuators_node/auto_commands
Type: geometry_msgs/Vector3Stamped
```
