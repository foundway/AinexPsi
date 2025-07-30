#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ainex_interfaces.srv import SetRGB

class RGBServiceNode(Node):
    def __init__(self):
        super().__init__('rgb_service_node')
        
        # Create the service
        self.service = self.create_service(
            SetRGB, 
            '/sensor/rgb/set_rgb_state', 
            self.handle_set_rgb
        )
        
        self.get_logger().info('RGB Service node started. Service: /sensor/rgb/set_rgb_state')

    def handle_set_rgb(self, request, response):
        # Log the received RGB values
        self.get_logger().info(
            f'Received RGB request: r={request.data.r}, g={request.data.g}, b={request.data.b}'
        )
        
        # Here you would implement the actual RGB setting logic
        # For now, we'll just acknowledge the request
        
        # Set response
        response.success = True
        response.message = "RGB values set successfully"
        
        self.get_logger().info('RGB service completed successfully')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RGBServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()