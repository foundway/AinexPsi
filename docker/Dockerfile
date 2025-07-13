FROM ros:humble-perception

# Install required Python modules and system packages
RUN apt-get update && \
    apt-get install -y python3-serial python3-rpi.gpio python3-rospy python3-rospkg python3-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

# Add container setup to bashrc
RUN echo "source /ros2_ws/docker/container_setup.sh" >> ~/.bashrc

# (Optional) Set up your workspace, copy files, etc. 