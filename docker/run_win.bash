#!/bin/bash

# Run Docker container for Flask map server
docker run -it --rm\
    --privileged \
    -p 5000:5000 \
    -v "`pwd`/../tanqueray:/home/`whoami`/ws/src/tanqueray" \
    -v "`pwd`/../visualizer:/home/`whoami`/ws/src/visualizer" \
    -v "`pwd`/../data:/home/`whoami`/data" \
    --name ese6500-final-project-dev \
    ese6500-project:dev \
    bash