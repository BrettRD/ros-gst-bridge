#
# The purpose of this Dockerfile is to demonstrate the build
# environment and procedure needed to successfully compile the
# ros-gst-bridge package.  Suggestions for improvement are welcome.
#
# Contrast with ../docker-short/Dockerfile which builds FROM ros:galactic-ros-core-focal
# This Dockerfile includes steps needed by users who do not yet have ROS2 installed.
#

FROM ubuntu:focal

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    apt-utils \
    curl \
    git \
    gnupg2 \
    locales \
    lsb-release

#
# https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html
#

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list

# your locale might already be configured this way on an interactive (non-docker) system
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# this will result in fetch of >400 packages in this small docker environment
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y install \
    python3-colcon-common-extensions \
    python3-colcon-core \
    python3-rosdep \
    ros-galactic-ros-core
RUN bash -c '. /opt/ros/galactic/setup.bash && \
    rosdep init '

# The above steps have probably already been done in an interactive
# environment where you have developed ROS2 software in the past

#
#
#

RUN useradd -ms /bin/bash zim && echo "zim ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

ENV HOME /home/zim
USER zim

RUN mkdir -p $HOME/galactic_ws/src
WORKDIR $HOME/galactic_ws/
RUN git clone https://github.com/BrettRD/ros-gst-bridge.git src/ros-gst-bridge

RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    DEBIAN_FRONTEND=noninteractive rosdep update && \
    DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . --ignore-src -r -y'

RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    colcon build '

# this is purely to demonstrate/verify how to set the GST_PLUGIN_PATH
RUN bash -c 'source install/setup.sh && \
    GST_PLUGIN_PATH=install/gst_bridge/lib/gst_bridge gst-inspect-1.0 rosimagesrc'
