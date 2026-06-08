# Input given to a car robot.
Header header

# ---- Throttle actuators ----

# References for the throttle actuator. Note that only one of the fields
# "torque" and "velocity" should be set. The other should be set to NaN.
# If both are set, the torque reference will be used.
float64 torque
float64 velocity

# Reference for the steer actuator. This is an angle in radians.
float64 steer

# In some cases the car's low-level steer map should be overriden (e.g., for
# system identification). If this flag is set to true, then the steer field
# is not interpreted as an angle in radians, but as a voltage reference for the
# potentiometer in the cars.
bool steer_override
