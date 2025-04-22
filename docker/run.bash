#!/bin/bash

xhost +
docker run -it --rm \
    --network=host \
    --privileged \
    -v "/dev:/dev" \
    -v "`pwd`/../../final-project-ese-6500:/home/`whoami`/ws/src/project" \
    --name ese6500-final-project-dev \
    ese6500-project:dev \
    bash
xhost -
