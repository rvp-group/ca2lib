#!/bin/sh

#recover the name of hte image from build

IMAGE_NAME=$(cat build.sh | grep IMAGE_NAME | grep -v build)
IMAGE_NAME=${IMAGE_NAME#*=}
echo ${IMAGE_NAME}

xhost +

docker run \
    -ti \
    --rm \
    --env="DISPLAY" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -v $1:/workspace/src/ca2lib \
    -v "$2":/working_dir \
    ${IMAGE_NAME} \
    bash -c "source /opt/ros/noetic/setup.bash && bash -c 'roscore > /dev/null &' && /bin/bash"