FROM ros:humble-ros-core-jammy

LABEL project='colav-interfaces'
LABEL maintainer='Ryan McKee <r.mckee@qub.ac.uk>'
LABEL version='0.0.1'
LABEL description='ROS-based dev container dockerfile for colav-interfaces development and testing'

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
    https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
    https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    ros-humble-ament-cmake \
    pip \
    && rm -rf /var/lib/apt/lists/*

# source ros2 ws and add to .bashrc
RUN /bin/bash -c "echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
    && source /root/.bashrc"

# Keep container running indefinitely
CMD ["tail", "-f", "/dev/null"]