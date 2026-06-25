# This message defines metadata for a Lighthouse base station.
# The metadata consists of information that can be obtained from the bitstream of the base station,
# such as the id it is transmitting as and the laser sweep plane offsets.

uint8 id        # ID of the base station, 0 to 15

# Sweep plane offsets
float32 dt1     # Offset of plane 1, such that the total angle is +π/6 + dt1
float32 dt2     # Offset of plane 2, such that the total angle is -π/6 + dt2
