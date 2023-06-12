FROM osrf/ros:humble-desktop-full-jammy

ARG DEBIAN_FRONTEND=noninteractive

SHELL [ "/bin/bash", "-c" ]

# use a closer mirror in dallas, texas
RUN cp /etc/apt/sources.list /etc/apt/sources.list.original && \
    sed -i -r 's,http://(.*).ubuntu.com,http://mirror.us-tx.kamatera.com,' /etc/apt/sources.list

# install basic dependnecies
RUN apt-get update && \
    apt-get -y --no-install-recommends install \
    mesa-utils \
    mesa-utils-extra \
    python3-pip \
    python-is-python3 \
    software-properties-common \
    wget

# downgrade setuptools to get rid of deprecated message and install python dependencies
RUN pip install setuptools==58.2.0 torch torchvision ultralytics

# install velodyne dependencies
RUN apt-get update && \
    apt-get -y --no-install-recommends install \
    ros-humble-velodyne

# See https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages
RUN apt-get update -y && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# install librealsense w/realsense ros2 wrapper    
RUN apt-get update && \
    apt-get -y --no-install-recommends install \
    librealsense2-dbg \
    librealsense2-dev \
    librealsense2-dkms \
    librealsense2-utils \
    ros-humble-realsense2-*

# create workspace folder and make it the starting folder
RUN mkdir -p /root/colcon_workspace && \
    echo "cd /root/colcon_workspace" >> /root/.bashrc

# source ROS setup.bash
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc