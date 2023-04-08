FROM osrf/ros:humble-desktop-full

RUN mkdir -p /autonav/autonav_ws/src
RUN mkdir -p /autonav/setup

COPY autonav_ws/src /autonav/autonav_ws/src
COPY setup /autonav/setup
COPY vectorsecrets.txt /autonav/setup/vectorsecrets.txt

RUN /bin/bash -c "git submodule update --init --recursive"

WORKDIR /autonav/setup
RUN /bin/bash -c "./setup.sh"

WORKDIR /autonav/autonav_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build"