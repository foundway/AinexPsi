# Main Package 
Contains nodes for the main robot logics

# Quick Start

## ROS1 Bridge
```bash
cd bridge_ws
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```

## Command Line
```bash
source /ros2_interfaces_ws/install/local_setup.bash
ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand "command: 'start'"
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.03, y: 0.0, angle: 0.0}" -1
ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand "command: 'stop'"
```

## Gait
```bash
source /opt/ros/foxy/setup.bash
cd main_ws
source install/setup.bash
ros2 run main gait
```
[Walking](./README-Walking.md)