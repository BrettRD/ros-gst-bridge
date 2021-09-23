#
# The purpose of this Dockerfile is to demonstrate the build procedure
# needed to successfully compile the ros-gst-bridge package if you
# already have a ROS2 environment.  Suggestions for improvement are
# welcome.
#

FROM ros:galactic-ros-core-focal

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    apt-utils \
    git

RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y install \
    python3-colcon-core \
    python3-colcon-common-extensions \
    python3-rosdep

RUN bash -c '. /opt/ros/galactic/setup.bash && \
    rosdep init '

# The above steps have probably already been done in an interactive
# environment where you have developed ROS2 software in the past

#
#
#

# You would not normally automatically give a NOPASSWD sudo permission
# to a user, but this is a Dockerfile, and there is no way (or reason)
# to prompt for a password when rosdep install needs to install packages
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
