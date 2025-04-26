FROM husarion/rosbot-xl-gazebo:humble-0.3.0-20230204

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_WS=/ros2_ws

# Install basic tools and dependencies
RUN apt-get update && apt-get install -y \
    git \
    curl \
    wget \
    lsb-release \
    sudo \
    gnupg2 \
    build-essential \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-dev-tools \
    stm32flash \
    software-properties-common


# Create workspace
RUN mkdir -p $ROS_WS/src
WORKDIR $ROS_WS

# Clone Husarion ROSbot repo
RUN git clone https://github.com/husarion/rosbot_xl_ros src/

# Simulation build flag
ENV HUSARION_ROS_BUILD=simulation

# Source ROS and import dependencies
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    vcs import src < src/rosbot_xl/rosbot_xl_hardware.repos && \
    vcs import src < src/rosbot_xl/rosbot_xl_simulation.repos"

# Workaround: copy only needed ros2_controllers
RUN cp -r src/ros2_controllers/diff_drive_controller src/ && \
    cp -r src/ros2_controllers/imu_sensor_broadcaster src/ && \
    rm -rf src/ros2_controllers

# Install rosdeps
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y"

# Build the workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Auto-source the workspace
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]
