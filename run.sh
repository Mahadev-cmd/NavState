#!/bin/bash

xhost +local:root  # Allow GUI access from Docker
docker run -it --rm \
  --network=host \
  --privileged \
  --env="DISPLAY" \
  --env="RMW_IMPLEMENTATION=rmw_fastrtps_cpp" \
  --env="ROS_DOMAIN_ID=0" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --device=/dev \
  --name amr-container \
  mahadev1632/nav_state:v2
