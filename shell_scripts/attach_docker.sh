#!/bin/bash

CONTAINER_NAME=${CONTAINER_NAME:-"issac_ros_dev-aarch64-container"}

sudo docker start $CONTAINER_NAME
sudo docker exec -i $CONTAINER_NAME bash -c 'cat > ~/.Xauthority' < ~/.Xauthority
sudo docker exec -it --env="DISPLAY" $CONTAINER_NAME bash


