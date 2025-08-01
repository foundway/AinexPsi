cmake_minimum_required(VERSION 3.5)
project(ainex_interfaces)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AppWalkingParam.msg"
  "msg/PixelPosition.msg"
  "msg/LineROI.msg"
  "msg/RGB.msg"
  "msg/ColorsDetect.msg"
  "msg/ColorDetect.msg"
  "msg/ROI.msg"
  "msg/HeadState.msg"
  "msg/FingerPosition.msg"
  "msg/WalkingParam.msg"
  "msg/ObjectInfo.msg"
  "msg/ObjectsInfo.msg"
  "srv/SetWalkingParam.srv"
  "srv/SetString.srv"
  "srv/GetWalkingParam.srv"
  "srv/GetWalkingState.srv"
  "srv/SetPoint.srv"
  "srv/SetWalkingCommand.srv"
  "srv/SetRGB.srv"
  "srv/SetInt.srv"
  "srv/SetFloat.srv"
  DEPENDENCIES geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

# Install mapping rules for ros1_bridge
install(FILES mapping_rules.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
