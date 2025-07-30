# Official ROS2 Foxy image for Focal(https://hub.docker.com/layers/library/ros/foxy-ros-base-focal/images/sha256-078ac8eaffc953e97d4abf72f57da1fc33080b720b94bb1dc8c05ef4f7690367)
FROM ros:foxy-ros-base-focal

# Fix datetime issue by synchronizing system time before apt operations
RUN apt-get update && \
    apt-get install -y \
    python3-serial \

    # Needed for building the ros1_bridge
    libyaml-cpp-dev \
    liblog4cxx-dev \
    libboost-all-dev \

    # Needed for ROS1 commands
    python3-gnupg \
    python3-pycryptodome \

    && rm -rf /var/lib/apt/lists/*

# Clone repositories
WORKDIR /bridge_ws/src
RUN git clone -b foxy https://github.com/ros2/ros1_bridge.git

# Set working directory back
WORKDIR /