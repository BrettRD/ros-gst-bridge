#
# The purpose of this Dockerfile is to demonstrate the build
# environment and procedure needed to successfully compile the
# ros-gst-bridge package.  Suggestions for improvement are welcome.
#

FROM ubuntu:focal as ros_base

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

# this will result in fetch of >1650 packages in this small docker environment
RUN apt-get update && DEBIAN_FRONTEND="noninteractive" apt-get -y install ros-galactic-desktop python3-rosdep python3-colcon-core python3-colcon-common-extensions
RUN bash -c '. /opt/ros/galactic/setup.bash && \
    rosdep init '

#
#
#

FROM ros_base

# install these so rosdep update doesn't asplode
RUN DEBIAN_FRONTEND=noninteractive apt-get -y install gstreamer1.0-tools gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

RUN useradd -ms /bin/bash zim

ENV HOME /home/zim
USER zim

RUN mkdir -p $HOME/galactic_ws/src
WORKDIR $HOME/galactic_ws/src
RUN git clone https://github.com/BrettRD/ros-gst-bridge.git 

RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    DEBIAN_FRONTEND=noninteractive rosdep update && \
    DEBIAN_FRONTEND=noninteractive rosdep install --from-paths . --ignore-src -r -y'

RUN bash -c 'source /opt/ros/galactic/setup.bash && \
    colcon build '

RUN bash -c 'source install/setup.sh && \
    GST_PLUGIN_PATH=install/gst_bridge/lib/gst_bridge gst-inspect-1.0 rosimagesrc'