#!/usr/bin/env bash

WORKSPACE_DIR=$PWD

CONTAINER_NAME=roboboat-2023
IMAGE_NAME=mhseals/roboboat-2023
IMAGE_TAG=nvidia

# get docker container ID if exists
CONTAINER_ID=$(docker ps -aqf "name=^/${CONTAINER_NAME}$")

if [ -z "${CONTAINER_ID}" ]; then

    docker run \
        --rm \
        --tty \
        --privileged \
        --name ${CONTAINER_NAME} \
        --volume /tmp/.X11-unix:/tmp/.X11-unix \
        --volume /mnt/wslg:/mnt/wslg \
        --volume /usr/lib/wsl:/usr/lib/wsl \
        --volume "${WORKSPACE_DIR}:/root/colcon_workspace/" \
        --device=/dev/dxg \
        --env DISPLAY=$DISPLAY \
        --env WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
        --env XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
        --env PULSE_SERVER=$PULSE_SERVER \
        --gpus all \
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
