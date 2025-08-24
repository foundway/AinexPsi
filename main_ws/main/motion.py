#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ainex_interfaces.msg import MotionCommand
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
        
        self.get_logger().info('Motion node initialized - listening to /motion/command topic')

    def motion_callback(self, msg):
        """Callback function that prints received motion commands"""
        self.get_logger().info(f'Received motion command: servo_id={msg.servo_id}, position={msg.position}, duration={msg.duration}')
        print(f'Motion Command: servo_id={msg.servo_id}, position={msg.position}, duration={msg.duration}')

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
