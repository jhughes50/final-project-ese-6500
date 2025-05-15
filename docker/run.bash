#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "`pwd`/../tanqueray:/home/`whoami`/ws/src/tanqueray" \
    -v "`pwd`/../visualizer:/home/`whoami`/ws/src/visualizer" \
    -v "/home/jason/ROS/bags/:/home/`whoami`/data" \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    --name ese6500-final-project-dev \
    ese6500-project:dev \
    bash
xhost -
