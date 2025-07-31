# Walking Command Service Usage Guide

## Overview

The `/walking/command` service uses the `ainex_interfaces/srv/SetWalkingCommand` interface to control the robot's walking behavior. This service allows you to start, stop, enable, and disable walking functionality.

## Service Interface

### Service Type
- **Service Name**: `/walking/command`
- **Service Type**: `ainex_interfaces/srv/SetWalkingCommand`

### Request/Response Format
```yaml
# Request
string command

# Response  
bool result
```

## Available Commands

The service accepts the following string commands:

### 1. `"enable_control"`
- **Purpose**: Enables walking control system
- **When to use**: Must be called before any other walking commands
- **Effect**: Initializes the walking control system and allows other commands to work
- **Returns**: `true` if successful

### 2. `"disable_control"`
- **Purpose**: Disables walking control system
- **When to use**: When you want to completely disable walking functionality
- **Effect**: Disables the walking control system
- **Returns**: `true` if successful

### 3. `"enable"`
- **Purpose**: Enables the walking module
- **When to use**: After `enable_control` to activate walking capabilities
- **Effect**: Enables the walking module for motion control
- **Returns**: `true` if successful

### 4. `"disable"`
- **Purpose**: Disables the walking module
- **When to use**: To stop walking and disable the walking module
- **Effect**: Stops any current walking motion and disables the module
- **Returns**: `true` if successful

### 5. `"start"`
- **Purpose**: Starts walking motion
- **When to use**: To begin walking with current parameters
- **Effect**: Initiates walking motion based on configured parameters
- **Returns**: `true` if successful

### 6. `"stop"`
- **Purpose**: Stops walking motion
- **When to use**: To halt current walking motion
- **Effect**: Stops walking motion and returns to standing position
- **Returns**: `true` if successful

## Usage Examples

### Python Example

```python
import rospy
from ainex_interfaces.srv import SetWalkingCommand

# Create service proxy
walking_command = rospy.ServiceProxy('/walking/command', SetWalkingCommand)

# Enable control (required first step)
result = walking_command('enable_control')

# Enable walking module
result = walking_command('enable')

# Start walking
result = walking_command('start')

# Stop walking
result = walking_command('stop')

# Disable walking module
result = walking_command('disable')

# Disable control
result = walking_command('disable_control')
```

### Complete Walking Sequence

```python
import rospy
import time
from ainex_interfaces.srv import SetWalkingCommand, GetWalkingState

class WalkingController:
    def __init__(self):
        self.walking_command = rospy.ServiceProxy('/walking/command', SetWalkingCommand)
        self.get_walking_state = rospy.ServiceProxy('/walking/is_walking', GetWalkingState)
    
    def start_walking_sequence(self):
        """Complete walking sequence"""
        # 1. Enable control
        self.walking_command('enable_control')
        rospy.sleep(1.0)
        
        # 2. Enable walking module
        self.walking_command('enable')
        rospy.sleep(1.0)
        
        # 3. Start walking
        self.walking_command('start')
        
        # 4. Walk for some time
        rospy.sleep(5.0)
        
        # 5. Stop walking
        self.walking_command('stop')
        
        # 6. Wait for walking to stop
        while self.get_walking_state().state:
            rospy.sleep(0.1)
        
        # 7. Disable walking module
        self.walking_command('disable')
        
        # 8. Disable control
        self.walking_command('disable_control')
```

### Command Line Example

```bash
# Enable control
rosservice call /walking/command "command: 'enable_control'"

# Enable walking module
rosservice call /walking/command "command: 'enable'"

# Start walking
rosservice call /walking/command "command: 'start'"

# Stop walking
rosservice call /walking/command "command: 'stop'"

# Disable walking module
rosservice call /walking/command "command: 'disable'"

# Disable control
rosservice call /walking/command "command: 'disable_control'"
```

## Important Notes

### Prerequisites
1. The robot must be in a stable standing position
2. Walking parameters should be properly configured
3. The walking controller node must be running

### Safety Considerations
- Always call `stop` before `disable` to ensure smooth stopping
- Wait for walking to completely stop before disabling
- Use `disable_control` when you're completely done with walking

### Error Handling
- Check the `result` field in the response
- Handle `rospy.ServiceException` for service call failures
- Use timeouts when waiting for walking to stop

### State Management
- The walking system has internal state management
- Commands may be ignored if called in wrong order
- Always follow the proper sequence: enable_control → enable → start → stop → disable → disable_control

## Related Services

- `/walking/is_walking` - Check if robot is currently walking
- `/walking/get_param` - Get current walking parameters
- `/walking/set_param` - Set walking parameters
- `/walking/init_pose` - Initialize robot pose

## Troubleshooting

### Common Issues

1. **Service not available**
   - Ensure the walking controller node is running
   - Check if ROS master is running

2. **Commands not working**
   - Make sure to call `enable_control` first
   - Check if robot is in proper standing position

3. **Walking doesn't start**
   - Verify walking parameters are set correctly
   - Check if walking module is enabled

4. **Walking doesn't stop**
   - Call `stop` command and wait for completion
   - Check for any error messages in logs

### Debug Commands

```bash
# Check if walking service is available
rosservice list | grep walking

# Check walking state
rosservice call /walking/is_walking

# Get current walking parameters
rosservice call /walking/get_param
``` 