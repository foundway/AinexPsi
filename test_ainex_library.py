#!/usr/bin/env python3

import time
import rospy
from ainex_sdk.hiwonder_servo_controller import HiwonderServoController
from ainex_kinematics.gait_manager import GaitManager

servo_controller = HiwonderServoController(port='/dev/ttyAMA0', baudrate=115200)
servo_controller.set_servo_position(servo_id=24, position=300, duration=1000)
time.sleep(1)
servo_controller.set_servo_position(servo_id=24, position=600, duration=1000)

# Initialize the ROS node
rospy.init_node('gait_manager_node', anonymous=True)

# Now create and use the GaitManager
gait_manager = GaitManager()
gait_manager.move(2, 0, 0, 5)
time.sleep(2)
gait_manager.stop()
