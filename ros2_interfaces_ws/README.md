# ROS2 Intrefaces Workspace

To build a pure ROS 2 Interfaces package for [ROS1 Bridge](../bridge_ws/)

## Build

```
cd /ros2_interfaces_ws
source /opt/ros/foxy/setup.bash
colcon build --symlink-install --cmake-force-configure 
```

## Run

```
source install/local_setup.bash && ros2 interface list | grep ainex
```
----
## Note

msg/ and srv/ are mounted as read-only volumes from the original from Hiwonder

Use [mapping_rules.yaml](./src/ainex_interfaces/mapping_rules.yaml) to set mapping explicitly because ros1_bridge doesn't detect `ainex_interfaces` automatically
