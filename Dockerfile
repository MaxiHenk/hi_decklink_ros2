FROM ros:noetic-ros-core

# Paths
ENV CATKIN_WS /catkin_ws
ENV SRC_ROOT_DIR $CATKIN_WS/src

# Dependencies
RUN apt-get update -y && \
    apt-get -y install build-essential cmake git libboost-exception-dev \
    python3 python3-jinja2 python3-catkin-tools python3-rosdep \
    ros-noetic-image-transport ros-noetic-camera-info-manager ros-noetic-cv-bridge && \
    apt-get clean -y

# Copy source repository and build workspace
COPY . $SRC_ROOT_DIR
WORKDIR $CATKIN_WS
RUN /bin/bash -c 'source /opt/ros/noetic/setup.bash; catkin build hi_decklink_ros'
