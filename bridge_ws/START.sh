#!/bin/bash

set -e  # Exit on any error

echo "🚀 Starting ROS1 Bridge..."

source /bridge_ws/install/setup.bash 

ros2 run ros1_bridge dynamic_bridge --bridge-all-topics