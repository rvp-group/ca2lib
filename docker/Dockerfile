FROM osrf/ros:noetic-desktop-full

RUN apt-get update && \
    apt-get install -y \
    git wget python3-dev \
    python3-pip \
    libeigen3-dev \
    libopencv-dev \
    python3-catkin-tools \
    libspdlog-dev


ENV WORKSPACE /catkin_ws

RUN mkdir -p ${WORKSPACE}/src && \
    cd ${WORKSPACE} && \
    catkin init && \
    catkin config --extend /opt/ros/noetic && \
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

COPY . ${WORKSPACE}/src/ca2lib

RUN cd ${WORKSPACE} && \
    catkin build


ENTRYPOINT cd ${WORKSPACE} && /bin/bash