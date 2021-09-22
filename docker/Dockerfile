#
# The purpose of this Dockerfile is to demonstrate the build
# environment and procedure needed to successfully compile the
# ros-gst-bridge package.  Suggestions for improvement are welcome.
#

# galactic-ros-core-focal is a relatively clean ubuntu focal installation
#     with a minimal install of ros-galactic

#https://hub.docker.com/_/ros
#https://github.com/osrf/docker_images/blob/master/ros/galactic/ubuntu/focal/ros-core/Dockerfile
FROM ros:galactic-ros-core-focal

# fetch some additional development tools
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    git \
    python3-rosdep \
    python3-colcon-core \
    python3-colcon-common-extensions

# initialise rosdep, this is normally already done by the system install
RUN bash -c '. /opt/ros/galactic/setup.bash && \
    rosdep init '

# add a user who can call sudo (required for rosdep to use apt)
RUN useradd -ms /bin/bash zim
RUN echo "zim ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers
ENV HOME /home/zim
USER zim

#
#
#

# create a workspace with an empty src directory
RUN mkdir -p $HOME/galactic_ws/src
# enter the workspace
WORKDIR $HOME/galactic_ws

# clone packages into src/
RUN git clone https://github.com/BrettRD/ros-gst-bridge.git src/ros-gst-bridge

# use rosdep to resolve and install pacakge dependencies
RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    DEBIAN_FRONTEND=noninteractive rosdep update && \
    DEBIAN_FRONTEND=noninteractive rosdep install --from-paths src --ignore-src -r -y'

# use colcon to build the package
RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    colcon build '

# inspect the final artifacts
RUN bash -c 'source install/setup.bash && \
    GST_PLUGIN_PATH=install/gst_bridge/lib/gst_bridge gst-inspect-1.0 rosimagesrc'
