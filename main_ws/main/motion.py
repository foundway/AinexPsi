#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ainex_interfaces.msg import MotionCommand
from ainex_sdk.hiwonder_servo_controller import HiwonderServoController
import time
import math

class MotionNode(Node):
    def __init__(self):
        super().__init__('motion_node')
        
        # Create subscriber to listen to /motion/command topic
        self.motion_sub = self.create_subscription(
            MotionCommand,
            '/motion/command',
            self.motion_callback,
            10
        )

        self.servo_controller = HiwonderServoController(port='/dev/ttyAMA0', baudrate=115200)
        
        self.get_logger().info('Motion node initialized - listening to /motion/command topic')

    def motion_callback(self, msg):
        """Callback function that prints received motion commands"""
        for i in range(len(msg.servo_id)):
            self.get_logger().info(f'Received motion command: servo_id={msg.servo_id[i]}, position={msg.position[i]}, duration={msg.duration[i]}')
            self.servo_controller.set_servo_position(servo_id=msg.servo_id[i], position=msg.position[i], duration=msg.duration[i])

def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
