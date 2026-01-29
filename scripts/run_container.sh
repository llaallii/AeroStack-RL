#!/bin/bash

# Configuration
IMAGE_NAME="aerostack-rl-dev"
CONTAINER_NAME="aerostack-rl-container"
PROJECT_DIR="/mnt/c/Users/ratan/Desktop/AeroStack-RL"
CONTAINER_WORKDIR="/home/ros/workspace"

# 1. Build the image
echo "Building Docker image: $IMAGE_NAME..."
docker build -t $IMAGE_NAME .devcontainer

# 2. Kill existing container if running
if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Stopping existing container..."
    docker stop $CONTAINER_NAME
fi

# 3. Run the container
echo "Launching container..."
# Note: We mount the project directory and pass GPU/Network flags
docker run -it --rm \
    --name $CONTAINER_NAME \
    --net=host \
    --gpus=all \
    --privileged \
    -v "$PROJECT_DIR:$CONTAINER_WORKDIR" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -e DISPLAY=$DISPLAY \
    -e WAYLAND_DISPLAY=$WAYLAND_DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e PULSE_SERVER=$PULSE_SERVER \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    $IMAGE_NAME \
    bash
