# ROS 1 Interfaces Workspace 

To build a pure ROS 1 Interfaces package for [ROS1 Bridge](../bridge_ws/).



## Build
```bash
cd ros1_interfaces_ws
. /opt/ros/noetic/setup.bash
catkin_make_isolated --install
```

## Run
```bash
source install_isolated/setup.bash && rossrv list | grep ainex && rosmsg list | grep ainex
```
----

# Notes

msg/ and srv/ are mounted as read-only volumes from the original from Hiwonder

# Appendix

## Problem

The original `ainex_interfaces` package in `/home/ubuntu/ros_ws/src/ainex_interfaces/` is a pure message/service package that doesn't generate compiled libraries (`.so` files). When building `ros1_bridge`, it fails with linker errors because it can't find the compiled message libraries for custom message types like `ainex_interfaces/SetRGB`.

## Solution

Create a separate workspace with a modified version of `ainex_interfaces` that builds as a library package, providing the necessary compiled libraries for ros1_bridge to link against.

## Setup Process

### 1. Create the Workspace Structure

```bash
# Create workspace
mkdir -p /home/ubuntu/ros2_ws/ros_ws_copy/src
cd /home/ubuntu/ros2_ws/ros_ws_copy/src

# Create package using catkin_create_pkg
catkin_create_pkg ainex_interfaces message_generation message_runtime std_msgs geometry_msgs
```

### 2. Create Symlinks to Original Message/Service Files

```bash
cd /home/ubuntu/ros2_ws/ros_ws_copy/src/ainex_interfaces

# Create symlinks to original message and service files
ln -sf /home/ubuntu/ros_ws/src/ainex_interfaces/msg msg
ln -sf /home/ubuntu/ros_ws/src/ainex_interfaces/srv srv
```

### 3. Create Library Source Files

```bash
# Create include directory
mkdir -p include/ainex_interfaces

# Create source directory
mkdir -p src
```

#### Create Header File: `include/ainex_interfaces/ainex_interfaces.h`
Create a minimal header file with a simple class declaration.

See the file: `src/ainex_interfaces/include/ainex_interfaces/ainex_interfaces.h`

#### Create Source File: `src/ainex_interfaces.cpp`
Create a minimal source file with basic constructor/destructor implementation.

See the file: `src/ainex_interfaces/src/ainex_interfaces.cpp`

### 4. Modify CMakeLists.txt

Copy the CMakeLists.txt from the original package (`/home/ubuntu/ros_ws/src/ainex_interfaces/CMakeLists.txt`) and make the following key changes:

- Uncomment and enable library building sections
- Add message and service file declarations
- Enable `catkin_package()` with `LIBRARIES` and `INCLUDE_DIRS`

The current `src/ainex_interfaces/CMakeLists.txt` contains the modified version with all necessary changes applied.

### 5. Modify package.xml

Update the package.xml to include all necessary dependencies:

- Add `geometry_msgs` (note the 's' at the end)
- Add `message_runtime` as both build and exec dependency
- Add `message_generation` as exec dependency

See the file: `src/ainex_interfaces/package.xml` for the complete modified version.

```

## Integration with ros1_bridge

After building this library package, you can build ros1_bridge with the compiled libraries available:

```bash
# In the ros1_bridge container
export CMAKE_PREFIX_PATH=/opt/ros/foxy:/opt/ros/noetic:/home/ubuntu/ros2_ws/ros_ws_copy/devel
export PKG_CONFIG_PATH=/opt/ros/noetic/lib/pkgconfig:/usr/lib/pkgconfig:/usr/local/lib/pkgconfig:/home/ubuntu/ros2_ws/ros_ws_copy/devel/lib/pkgconfig

# Build ros1_bridge
cd /ros2_ws
colcon build --symlink-install --packages-select ros1_bridge --cmake-args -DBUILD_TESTING=OFF --cmake-force-configure
```

## Key Points

1. **Symlinks**: Use symlinks to avoid duplicating message/service files
2. **Library Structure**: Create minimal library source files to satisfy the build system
3. **Dependencies**: Ensure all dependencies are properly declared in both CMakeLists.txt and package.xml
4. **Environment**: Set up proper environment variables for the build process

## Troubleshooting

- **CMake errors**: Check that all dependencies are listed in both `find_package()` and `catkin_package()`
- **Package.xml errors**: Ensure build and exec dependencies match the CMakeLists.txt requirements
- **Linker errors**: Verify that the library is being built and exported correctly

This approach allows ros1_bridge to find the compiled message libraries for custom message types while keeping the original workspace unchanged. 