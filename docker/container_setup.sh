#!/bin/bash

# Set up ROS1 environment variables manually since symlinks are broken
export ROS_ROOT=/opt/ros/noetic
export ROS_PACKAGE_PATH=/ros_ws/src
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

# Set Python path for all Ainex modules
export PYTHONPATH="/ros_ws/devel/lib/python3/dist-packages:/ros_ws/src/third_party:/ros_ws/src/ainex_driver/ainex_sdk/src:/ros_ws/src/ainex_driver/ainex_kinematics/src:/ros_ws/devel/.private/ainex_kinematics/lib/python3/dist-packages:/ros_ws/devel/.private/ainex_interfaces/lib/python3/dist-packages:$PYTHONPATH"

# Source ROS1 environment
source /opt/ros/noetic/setup.bash

echo "ROS1 environment set up for container"
echo "PYTHONPATH: $PYTHONPATH"

# Start bash
exec bash 