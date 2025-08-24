# Main Package 
Contains nodes for the main robot logics

# Quick Start

## Gait

ROS1 Bridge (Required for Gait)
```bash
cd bridge_ws
source install/setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
Gait node (More about [walking](./README-Walking.md))
```bash
source /opt/ros/foxy/setup.bash
source /main_ws/install/setup.bash
ros2 run main gait
```

Command Line
```bash
source /ros2_interfaces_ws/install/local_setup.bash
ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand "command: 'start'"
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.03, y: 0.0, angle: 0.0}" -1
ros2 service call /walking/command ainex_interfaces/srv/SetWalkingCommand "command: 'stop'"
```
## Motion
```bash
source /opt/ros/foxy/setup.bash
source /main_ws/install/setup.bash
ros2 run main motion
```

# Build
```bash
source /opt/ros/foxy/setup.bash
source /ros2_interfaces_ws/install/local_setup.bash
colcon build --symlink-install --cmake-force-configure 
```