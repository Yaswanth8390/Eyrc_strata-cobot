FROM ros:jazzy

# ROS Jazzy desktop + development tools
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop-full \
    ros-dev-tools \
    git \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Create ROS workspace
RUN mkdir -p /root/ros2_ws/src

# Clone the project
WORKDIR /root/ros2_ws/src
RUN git clone https://github.com/Yaswanth8390/Eyrc_strata-cobot.git .

# Install project-specific dependencies
RUN chmod +x requirements.sh && ./requirements.sh

# Workspace
WORKDIR /root/ros2_ws

# Source ROS automatically
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc

CMD ["/bin/bash"]