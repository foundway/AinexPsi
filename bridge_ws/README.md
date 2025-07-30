# ROS1 Bridge Workspace

[Documentation](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst) · 
[Repo](https://github.com/ros2/ros1_bridge/tree/foxy)

## Clone 
```
cd src && git clone -b foxy https://github.com/ros2/ros1_bridge.git
```

## Build 
```
cd /bridge_ws

# Source ROS 
source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash

# Source Interfaces
source /ros1_interfaces_ws/install_isolated/setup.bash && rosmsg list | grep ainex && rossrv list | grep ainex
source /ros2_interfaces_ws/install/local_setup.bash && ros2 interface list | grep ainex

# (maybe) Remove ROS1 from CMAKE_PREFIX_PATH (see Appendix)
export CMAKE_PREFIX_PATH=/ros2_interfaces_ws/install/ainex_interfaces:/ros1_interfaces_ws/install_isolated

# (maybe) Set environment
export PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig:/usr/lib/pkgconfig:/usr/local/lib/pkgconfig:/ros_ws/devel/lib/pkgconfig:/ros2_ws/ros_ws_copy/devel/lib/pkgconfig && export CMAKE_PREFIX_PATH=/ros2_ws/ros_ws_copy/devel:/ros_ws/devel:/opt/ros/foxy:/opt/ros/noetic && export AMENT_PREFIX_PATH=/opt/ros/foxy PYTHONPATH=/home/ubuntu/ros_ws/devel/lib/python3/dist-packages:$PYTHONPATH && export ROS_PACKAGE_PATH=/ros_ws/src:$ROS_PACKAGE && export LD_LIBRARY_PATH=/opt/ros/noetic/lib:/ros2_ws/ros_ws_copy/devel/lib:/ros_ws/devel/lib:$LD_LIBRARY_PATH 

export MAKEFLAGS=-j1 # limit concurrent job to 1 to prevent disconnection to Raspberry
colcon build --symlink-install --cmake-force-configure --cmake-args -DBUILD_TESTING=OFF 
```

## Run 
```
source install/setup.bash 
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
# echo 'Print mapped pairs:' && ros2 run ros1_bridge dynamic_bridge --bridge-all-topics --print-pairs
```

----

# Appendix

* [CMAKE_PREFIX_PATH must not contain paths from ROS 1 ](https://github.com/ros2/ros1_bridge/tree/foxy#:~:text=The%20bridge%20uses%20pkg%2Dconfig%20to%20find%20ROS%201%20packages.%20ROS%202%20packages%20are%20found%20through%20CMake%20using%20find_package().%20Therefore%20the%20CMAKE_PREFIX_PATH%20must%20not%20contain%20paths%20from%20ROS%201%20which%20would%20overlay%20ROS%202%20packages.)

* [Example of custom interfaces](https://github.com/TommyChangUMD/ros-humble-ros1-bridge-builder/blob/main/Dockerfile)
