#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ainex_interfaces.srv import SetWalkingCommand, GetWalkingState
from ainex_interfaces.msg import WalkingParam, AppWalkingParam
import time
import threading
import readchar

class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')
        
        # Create service clients for walking control
        self.walking_command_client = self.create_client(SetWalkingCommand, '/walking/command')
        self.get_walking_state_client = self.create_client(GetWalkingState, '/walking/is_walking')
        
        # Create publisher for app walking parameters
        self.app_walking_pub = self.create_publisher(AppWalkingParam, '/app/set_walking_param', 10)
        
        # Wait for services to be available
        while not self.walking_command_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /walking/command service...')
        
        while not self.get_walking_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /walking/is_walking service...')
        
        self.get_logger().info('Gait node initialized successfully')
        
        # Initialize walking state
        self.is_walking = False
        self.walking_enabled = False
        
        # Create timer for periodic status check
        self.status_timer = self.create_timer(1.0, self.status_callback)
        
        # Initialize keyboard control
        self.keyboard_running = True
        self.keyboard_thread = None
        self.start_keyboard_control()

    # Keyboard control functions

    def start_keyboard_control(self):
        """Start keyboard control in a separate thread"""
        self.get_logger().info('Starting keyboard control...')
        self.keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.keyboard_thread.start()
        self.get_logger().info('Keyboard control active. Use arrow keys (w/s/a/d) to control walking.')

    def keyboard_loop(self):
        """Main keyboard input loop"""
        self.get_logger().info('Press w/s/a/d for up/down/left/right, q to quit')
        
        try:
            while self.keyboard_running:
                # Use readchar to get a single keystroke
                key = readchar.readkey().lower()
                
                if key == 'w':
                    self.get_logger().info('UP arrow key pressed. Starting walking...')
                    self.start_walking()
                    self.publish_app_walking_param(2, 0.0, 1.0, 0.0, 0.0)
                elif key == 's':
                    self.get_logger().info('DOWN arrow key pressed. Stop walking...')
                    self.stop_walking()
                elif key == 'a':
                    self.get_logger().info('LEFT arrow key pressed')
                    self.start_walking()
                    self.publish_app_walking_param(2, 0.0, 0.0, 0.0, 10.0)
                elif key == 'd':
                    self.get_logger().info('RIGHT arrow key pressed')
                    self.start_walking()
                    self.publish_app_walking_param(2, 0.0, 0.0, 0.0, -10.0)
                elif key == 'q':
                    self.get_logger().info('Quit key pressed - stopping keyboard control')
                    self.keyboard_running = False
                    break
                else:
                    self.get_logger().debug(f'Unknown key pressed: {key}')
                    
        except KeyboardInterrupt:
            self.get_logger().info('Keyboard interrupt received')
        except Exception as e:
            self.get_logger().error(f'Keyboard input error: {str(e)}')

    def stop_keyboard_control(self):
        """Stop keyboard control"""
        self.keyboard_running = False
        if self.keyboard_thread and self.keyboard_thread.is_alive():
            self.keyboard_thread.join(timeout=1.0)
        self.get_logger().info('Keyboard control stopped')

    # Walking control functions

    def publish_app_walking_param(self, speed, height, x, y, angle):
        """Publish walking parameters to /app/set_walking_param topic"""
        msg = AppWalkingParam()
        msg.speed = speed
        msg.height = height
        msg.x = x
        msg.y = y
        msg.angle = angle
        
        self.app_walking_pub.publish(msg)
        self.get_logger().info(f'Published to /app/set_walking_param: speed={speed}, height={height}, x={x}, y={y}, angle={angle}')

    def enable_control(self):
        """Enable walking control system"""
        self.get_logger().info('Enabling walking control...')
        request = SetWalkingCommand.Request()
        request.command = 'enable_control'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def disable_control(self):
        """Disable walking control system"""
        self.get_logger().info('Disabling walking control...')
        request = SetWalkingCommand.Request()
        request.command = 'disable_control'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def enable_walking(self):
        """Enable walking module"""
        self.get_logger().info('Enabling walking module...')
        request = SetWalkingCommand.Request()
        request.command = 'enable'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def disable_walking(self):
        """Disable walking module"""
        self.get_logger().info('Disabling walking module...')
        request = SetWalkingCommand.Request()
        request.command = 'disable'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def start_walking(self):
        """Start walking motion"""
        self.get_logger().info('Starting walking motion...')
        request = SetWalkingCommand.Request()
        request.command = 'start'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    def stop_walking(self):
        """Stop walking motion"""
        self.get_logger().info('Stopping walking motion...')
        request = SetWalkingCommand.Request()
        request.command = 'stop'
        
        future = self.walking_command_client.call_async(request)
        future.add_done_callback(self.service_callback)

    # Callback functions

    def service_callback(self, future):
        """Generic callback for service responses"""
        try:
            response = future.result()
            if response.result:
                self.get_logger().debug('Service call successful')
            else:
                self.get_logger().warn('Service call failed')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {str(e)}')

    def status_callback(self):
        """Periodic callback to check walking status"""
        try:
            future = self.get_walking_state_client.call_async(GetWalkingState.Request())
            future.add_done_callback(self.walking_state_callback)
        except Exception as e:
            self.get_logger().error(f'Failed to get walking state: {str(e)}')

    def walking_state_callback(self, future):
        """Callback for walking state service response"""
        try:
            result = future.result()
            if result.state != self.is_walking:
                self.is_walking = result.state
                self.get_logger().info(f'Walking state changed: {self.is_walking}')
        except Exception as e:
            self.get_logger().error(f'Error in walking state callback: {str(e)}')

    def get_walking_state(self):
        """Get current walking state"""
        try:
            future = self.get_walking_state_client.call_async(GetWalkingState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
            return future.result().state
        except Exception as e:
            self.get_logger().error(f'Failed to get walking state: {str(e)}')
            return False

    # Helper functions

    def wait_for_walking_to_stop(self, timeout=10.0):
        """Wait for walking to stop with timeout"""
        start_time = time.time()
        while self.get_walking_state() and (time.time() - start_time) < timeout:
            time.sleep(0.1)
        
        if self.get_walking_state():
            self.get_logger().warn('Timeout waiting for walking to stop')
            return False
        else:
            self.get_logger().info('Walking stopped')
            return True

    def execute_walking_sequence(self):
        """Execute a complete walking sequence"""
        self.get_logger().info('=== Starting Walking Sequence ===')
        
        # Step 1: Enable control
        self.enable_control()
        time.sleep(1.0)
        
        # Step 2: Enable walking module
        self.enable_walking()
        time.sleep(1.0)
        
        # Step 3: Start walking
        self.start_walking()
        time.sleep(3.0)  # Walk for 3 seconds
        
        # Step 4: Stop walking
        self.stop_walking()
        self.wait_for_walking_to_stop()
        
        # Step 5: Disable walking module
        self.disable_walking()
        time.sleep(1.0)
        
        # Step 6: Disable control
        self.disable_control()
        
        self.get_logger().info('=== Walking Sequence Complete ===')

def main(args=None):
    rclpy.init(args=args)
    
    node = GaitNode()
    
    try:
        # # Execute walking sequence after a short delay
        # node.create_timer(2.0, lambda: node.execute_walking_sequence())
        
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_keyboard_control()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
