# $1 is first argument, which is the bag file name
# $2 is the namespace of the car

rosbag play $1 -s 0 -q --loop --pause --clock \
 --topics /qualisys/PAPER/pose /qualisys/PAPER/velocity /tf  /$2/control_input  /$2/lighthouse /$2/imu /$2/wheel_encoders # /qualisys/PAPER/odom /tf
#  /$2/trajectory /mocap/DEMO_TRACK /mocap/$2 /tf /tf_static /$2/estimation_node/best_state
