# Walking Control Guide: Services and Topics

## Overview

This guide covers the different ways to control the AinexPsi humanoid robot's walking behavior using ROS services and topics. The system provides multiple interfaces for different use cases.

## Simplest Control: `/app/set_walking_param` Topic

The **simplest** way to control walking is using the `/app/set_walking_param` topic with just 5 numbers:

```bash
# Publish walking parameters via topic
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.02, y: 0.0, angle: 0.0}"
```

### Parameters:
- **speed**: Walking speed (1-4)
  - `1` = Fast (300ms cycle)
  - `2` = Medium (400ms cycle) 
  - `3` = Slow (500ms cycle)
  - `4` = Very slow (600ms cycle)
- **height**: Body height in meters (0.0 = default)
- **x**: Forward/backward step in meters (-0.05 to 0.05)
- **y**: Left/right step in meters (-0.05 to 0.05)
- **angle**: Turning angle in degrees (-10 to 10)

### Usage Examples:
```bash
# Walk forward at medium speed
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.02, y: 0.0, angle: 0.0}"

# Walk backward at fast speed
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 1, height: 0.0, x: -0.02, y: 0.0, angle: 0.0}"

# Turn right at medium speed
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.0, y: 0.0, angle: 5.0}"
```

### Python Usage:
```python
import rclpy
from rclpy.node import Node
from ainex_interfaces.msg import AppWalkingParam

class SimpleWalkingController(Node):
    def __init__(self):
        super().__init__('simple_walking_controller')
        self.walking_pub = self.create_publisher(AppWalkingParam, '/app/set_walking_param', 10)
    
    def walk_forward(self, speed=2):
        msg = AppWalkingParam()
        msg.speed = speed
        msg.height = 0.0
        msg.x = 0.02
        msg.y = 0.0
        msg.angle = 0.0
        self.walking_pub.publish(msg)
        self.get_logger().info('Walking forward')

# Usage
controller = SimpleWalkingController()
controller.walk_forward(speed=2)
```

**Note**: This topic automatically bridges between ROS1 and ROS2.

## Service-Based Control: `/walking/command`

For more control over walking states, use the `/walking/command` service:

### Service Interface
- **Service Name**: `/walking/command`
- **Service Type**: `ainex_interfaces/srv/SetWalkingCommand`

### Available Commands

1. **`"enable_control"`** - Enables walking control system (required first)
2. **`"enable"`** - Enables walking module
3. **`"start"`** - Starts walking motion
4. **`"stop"`** - Stops walking motion
5. **`"disable"`** - Disables walking module
6. **`"disable_control"`** - Disables walking control system

### Python Example

```python
import rospy
from ainex_interfaces.srv import SetWalkingCommand

# Create service proxy
walking_command = rospy.ServiceProxy('/walking/command', SetWalkingCommand)

# Complete walking sequence
walking_command('enable_control')
rospy.sleep(1.0)
walking_command('enable')
rospy.sleep(1.0)
walking_command('start')
rospy.sleep(5.0)  # Walk for 5 seconds
walking_command('stop')
walking_command('disable')
walking_command('disable_control')
```

### Command Line Example

```bash
# Enable and start walking
rosservice call /walking/command "command: 'enable_control'"
rosservice call /walking/command "command: 'enable'"
rosservice call /walking/command "command: 'start'"

# Stop walking
rosservice call /walking/command "command: 'stop'"
rosservice call /walking/command "command: 'disable'"
rosservice call /walking/command "command: 'disable_control'"
```

## Parameter Control: `/walking/set_param`

For fine-tuned control, use the `/walking/set_param` service with full `WalkingParam`:

### Python Example

```python
import rospy
from ainex_interfaces.srv import SetWalkingParam
from ainex_interfaces.msg import WalkingParam

def set_walking_parameters():
    rospy.wait_for_service('/walking/set_param')
    set_param = rospy.ServiceProxy('/walking/set_param', SetWalkingParam)
    
    # Create walking parameters
    param = WalkingParam()
    param.period_time = 400  # Medium speed
    param.dsp_ratio = 0.2
    param.x_move_amplitude = 0.02  # Forward step
    param.y_move_amplitude = 0.0   # No side step
    param.angle_move_amplitude = 0.0  # No turning
    param.z_move_amplitude = 0.02
    param.arm_swing_gain = 0.5
    
    response = set_param(param)
    print(f"Parameters set: {response.result}")
```

## Complete Example Node

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ainex_interfaces.srv import SetWalkingCommand
from ainex_interfaces.msg import AppWalkingParam

class WalkingController(Node):
    def __init__(self):
        super().__init__('walking_controller')
        
        # Service client
        self.walking_command = self.create_client(SetWalkingCommand, '/walking/command')
        
        # Topic publisher
        self.walking_pub = self.create_publisher(AppWalkingParam, '/app/set_walking_param', 10)
        
        self.get_logger().info('Walking controller initialized')
    
    def start_walking_sequence(self):
        """Complete walking sequence using services"""
        # Enable control
        self.walking_command.call_async(SetWalkingCommand.Request(command='enable_control'))
        self.get_logger().info('Enabled control')
        
        # Enable walking
        self.walking_command.call_async(SetWalkingCommand.Request(command='enable'))
        self.get_logger().info('Enabled walking')
        
        # Start walking
        self.walking_command.call_async(SetWalkingCommand.Request(command='start'))
        self.get_logger().info('Started walking')
    
    def stop_walking_sequence(self):
        """Stop walking sequence"""
        self.walking_command.call_async(SetWalkingCommand.Request(command='stop'))
        self.get_logger().info('Stopped walking')
        
        self.walking_command.call_async(SetWalkingCommand.Request(command='disable'))
        self.get_logger().info('Disabled walking')
        
        self.walking_command.call_async(SetWalkingCommand.Request(command='disable_control'))
        self.get_logger().info('Disabled control')
    
    def walk_forward(self, speed=2):
        """Simple walking using topic"""
        msg = AppWalkingParam()
        msg.speed = speed
        msg.height = 0.0
        msg.x = 0.02
        msg.y = 0.0
        msg.angle = 0.0
        self.walking_pub.publish(msg)
        self.get_logger().info('Walking forward')

def main(args=None):
    rclpy.init(args=args)
    controller = WalkingController()
    
    try:
        # Start walking sequence
        controller.start_walking_sequence()
        rclpy.sleep(5.0)  # Walk for 5 seconds
        
        # Stop walking
        controller.stop_walking_sequence()
        
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Quick Reference

### Topic Control (Simplest)
```bash
# Walk forward
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.02, y: 0.0, angle: 0.0}"

# Turn right
ros2 topic pub /app/set_walking_param ainex_interfaces/msg/AppWalkingParam "{speed: 2, height: 0.0, x: 0.0, y: 0.0, angle: 5.0}"
```

### Service Control (Full Control)
```bash
# Start walking
rosservice call /walking/command "command: 'enable_control'"
rosservice call /walking/command "command: 'enable'"
rosservice call /walking/command "command: 'start'"

# Stop walking
rosservice call /walking/command "command: 'stop'"
rosservice call /walking/command "command: 'disable'"
rosservice call /walking/command "command: 'disable_control'"
```

## Related Services and Topics

### Services
- `/walking/command` - Control walking states
- `/walking/set_param` - Set detailed walking parameters
- `/walking/get_param` - Get current walking parameters
- `/walking/is_walking` - Check walking state

### Topics
- `/app/set_walking_param` - Simple walking control (5 parameters)
- `/walking/set_param` - Detailed walking parameters
- `/walking/is_walking` - Walking state updates

## Troubleshooting

### Common Issues
1. **Service not available** - Ensure walking controller node is running
2. **Commands not working** - Call `enable_control` first
3. **Walking doesn't start** - Check if parameters are set correctly
4. **Topic not working** - Verify ROS bridge is active

### Debug Commands
```bash
# Check available services
rosservice list | grep walking

# Check walking state
rosservice call /walking/is_walking

# List topics
rostopic list | grep walking
``` 