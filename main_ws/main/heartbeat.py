#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ainex_interfaces.srv import SetRGB
from ainex_interfaces.msg import RGB
import time
import math


class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('heartbeat_node')
        
        self.client = self.create_client(SetRGB, '/sensor/rgb/set_rgb_state')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /sensor/rgb/set_rgb_state service...')
        
        self.get_logger().info('Service /sensor/rgb/set_rgb_state is available')
        
        # Start the RGB sequence
        self.timer = self.create_timer(0.1, self.rgb_sequence)
        self.sequence_step = 0
        self.last_change_time = time.time()
        self.period = 2.0

    def rgb_sequence(self):
        current_time = time.time()
        time_normalized = current_time % self.period / self.period
        sine_value = math.pow(math.sin(time_normalized * math.pi), 2)
        green_value = abs(int(255 * sine_value))
        self.set_rgb(0, green_value, 0)  
        self.get_logger().info(f'Setting RGB to {green_value}')

    def set_rgb(self, r, g, b):
        """Call the SetRGB service to set RGB values"""
        request = SetRGB.Request()
        request.data.r = r
        request.data.g = g
        request.data.b = b
        
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().debug(f'RGB set successfully: {response.message}')
            else:
                self.get_logger().warn(f'Failed to set RGB: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    node = HeartbeatNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
