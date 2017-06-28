FROM ros:kinetic-ros-base

# Set environmental variable for repo names
ENV reponame can_interface
ENV workspace /root/driver_ws

# Install prereqs for CAN
RUN apt-get update && apt-get install -y \
    wget \
    ros-kinetic-can-msgs \
    python-catkin-tools \
    linux-headers-generic-hwe-16.04 \
    && rm -rf /var/lib/apt/lists/

# Download, build, and install linuxcan
RUN cd /usr/src \
    && wget http://www.kvaser.com/software/7330130980754/V5_20_0/linuxcan.tar.gz \
    && tar xf linuxcan.tar.gz \
    && cd linuxcan \
    && make canlib \
    && make linlib \
    && make -C canlib install \
    && make -C linlib install

# Create workspace for installing and building packages
RUN mkdir -p ${workspace}/src/${reponame}

# Copy this repo's contents into the workspace folder
ADD . ${workspace}/src/${reponame}

# Set up and build the workspace
RUN cd ${workspace} \
    && catkin init \
    && catkin config --extend /opt/ros/kinetic \
    && catkin build
