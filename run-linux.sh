#!/usr/bin/env bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &>/dev/null && pwd)
WORKSPACE_DIR=$(cd -- "$(dirname -- "${SCRIPT_DIR}")" &>/dev/null && pwd)

CONTAINER_NAME=ros2-workspace
IMAGE_NAME=samminhch/ros2-devel
IMAGE_TAG=nvidia

# get docker container ID if exists
CONTAINER_ID=$(docker ps -aqf "name=^/${CONTAINER_NAME}$")

if [ -z "${CONTAINER_ID}" ]; then

    docker run \
        --tty \
        --detach \
        --gpus=all \
        --privileged \
        --network host \
        --shm-size 16G \
        --name ${CONTAINER_NAME} \
        --env "DISPLAY=$DISPLAY" \
        --device "/dev/dri:/dev/dri" \
        --volume "/tmp/.X11-unix:/tmp/.X11-unix" \
        --volume "${WORKSPACE_DIR}:/root/colcon_workspace/" \
        ${IMAGE_NAME}:${IMAGE_TAG}

else
    xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' ${CONTAINER_ID})

    if [ -z $(docker ps -qf "name=^/${CONTAINER_NAME}$") ]; then
        echo "${CONTAINER_NAME} container not running. Starting container..."
        docker start ${CONTAINER_ID}
    else
        echo "Attaching to running ${CONTAINER_NAME} container..."
    fi
    docker exec -it ${CONTAINER_ID} bash
fi
