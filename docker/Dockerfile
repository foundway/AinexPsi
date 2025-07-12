FROM ros:humble-perception

# Install required Python modules and system packages
RUN apt-get update && \
    apt-get install -y python3-serial python3-rpi.gpio python3-rospy python3-rospkg python3-geometry-msgs && \
    rm -rf /var/lib/apt/lists/*

# (Optional) Set up your workspace, copy files, etc. 