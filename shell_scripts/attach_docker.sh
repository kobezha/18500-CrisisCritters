#!/bin/bash

CONTAINER_NAME=${CONTAINER_NAME:-"isaac_ros_dev-aarch64-container"}

docker start $CONTAINER_NAME
docker exec -i $CONTAINER_NAME bash -c 'cat > ~/.Xauthority' < ~/.Xauthority
docker exec -it --env="DISPLAY" $CONTAINER_NAME bash

