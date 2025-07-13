#!/bin/bash

# Set Python path for Ainex modules
export PYTHONPATH="/ros_ws/devel/lib/python3/dist-packages:/ros_ws/src/third_party:/ros_ws/src/ainex_driver/ainex_sdk/src:/ros_ws/src/ainex_driver/ainex_kinematics/src:/ros_ws/devel/.private/ainex_kinematics/lib/python3/dist-packages:/ros_ws/devel/.private/ainex_interfaces/lib/python3/dist-packages:$PYTHONPATH"

# Source ROS1 environment
source /opt/ros/noetic/setup.bash

echo "ROS1 environment set up for container"
echo "PYTHONPATH: $PYTHONPATH" 