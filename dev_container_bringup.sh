#! /bin/bash
echo "Starting development container for the Plate measurement demonstrator"
xhost +local:docker
docker run -id `# pseudo-interactive (keep running) and detached (background)` \
    --network host `# share network with host` \
    --rm `# remove container after stopping` \
    -e DISPLAY=${DISPLAY} `# set environment variable DISPLAY to same value as on host` \
    -v /tmp/.X11-unix:/tmp/.X11-unix `# share Xserver directory for GUI access` \
    -v `pwd`:/home/ros_ws/src/wp4_plate_measurement_demonstrator `# share repository `\
    --gpus all `# add access to all GPUs` \
    visir/plate_measurement_demonstrator_dev:latest