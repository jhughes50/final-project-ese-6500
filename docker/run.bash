#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "`pwd`/../tanqueray:/home/`whoami`/ws/src/tanqueray" \
    -v "`pwd`/../visualizer:/home/`whoami`/ws/src/visualizer" \
    -v "/home/jason/ROS/bags/:/home/`whoami`/data" \
    --name ese6500-final-project-dev \
    ese6500-project:dev \
    bash
xhost -
