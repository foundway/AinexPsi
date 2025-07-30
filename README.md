# AinexΨ

Spin-off of Hiwonder's [Ainex robot](https://www.hiwonder.com/products/ainex?variant=40257678180439&srsltid=AfmBOorHA9Kg-UunToEiFhaJ8Xiszz0VOP-isCvqnjyyccLVizuPH7n6). 

Ainex is running on ROS with no plan for ROS 2 support. Therefore, AinexΨ runs ROS2 in a Docker container with access to the Python modules provided by Hiwonder (e.g., ainex_sdk). Go to [docker](./docker) to [bring up the container](./docker/README.md).

## Build  
* ROS1 Interfaces (for bridge) [↗](./ros1_interfaces_ws)
* ROS2 Interfaces (for bridge) [↗](./ros2_interfaces_ws)
* ROS1 Bridge [↗](./bridge_ws)
* Main [↗]()

## Run

### ROS1 Bridge [↗](./bridge_ws)
```
cd bridge_ws
source /opt/ros/foxy/setup.bash && source install/setup.bash 
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
### Main logic 

TBD

----

## Notes

### Controls
```
from ainex_sdk.hiwonder_servo_controller import HiwonderServoController
servo_controller = HiwonderServoController(port='/dev/ttyAMA0', baudrate=115200)
servo_controller.set_servo_position(servo_id=24, position=500, duration=1000)
```
(Not implemented) to make gait commands:
```
from ainex_kinematics.gait_manager import GaitManager
```

```
cd ~/ros_ws
source devel/setup.sh
```

```
import rospy
from ainex_kinematics.gait_manager import GaitManager

# Initialize the ROS node
rospy.init_node('gait_manager_node', anonymous=True)

# Now create and use the GaitManager
gait_manager = GaitManager()
gait_manager.move(2, 0, 0, 5)
gait_manager.stop()
```

#### Python path for Ainex
```
export PYTHONPATH="/ros_ws/src/third_party:/ros_ws/src/ainex_driver/ainex_sdk/src:/home/ubuntu/ros_ws/devel/lib/python3/dist-packages:$PYTHONPATH"
```

### Clean up space
```
sudo rm -rf /var/lib/snapd/cache/*
docker image prune -a -f
docker builder prune -a -f
```

### Common Test Commands
```
ros2 service call /sensor/led/set_led_state std_srvs/srv/SetBool 'data: true'
```

### Online Resources

* [Ainex Playground](https://github.com/patdeg/ainex_playground/blob/main/ainex_sdk_v2.md)
