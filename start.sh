#!/bin/bash

#-----Config------
IMAGE_NAME="rrg_planner"
CONTAINER_NAME="rrg_planner-con"
DOCKERFILE_PATH="Dockerfile"
WORKSPACE_PATH="/home/bartek/Projects/RRG_Planner"
#SETUP_SCRIPT="./.devcontainer/setup-workspace.sh"

xhost +local:docker
#-----Build image------
echo "Building Docker image: $IMAGE_NAME"
docker build -t $IMAGE_NAME -f $DOCKERFILE_PATH .

#-----Remove existing container if exist
if [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "Removing existing container: $CONTAINER_NAME"
    docker rm -f $CONTAINER_NAME
fi

#-----Run container--------
echo "Starting container: $CONTAINER_NAME"

docker run -dit \
    --cap-add=SYS_PTRACE \
    --security-opt seccomp=unconfined \
    --volume="$HOME/.ssh:/root/.ssh" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix" \
    --network=host \
    --ipc=host \
    --privileged \
    --device=/dev/bus/usb:/dev/bus/usb \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env ROS_DOMAIN_ID=0 \
    --name $CONTAINER_NAME \
    --volume "$WORKSPACE_PATH:/root:cached" \
    -w "/root/ros2_ws" \
    $IMAGE_NAME

#----Run setup script ---
#echo "Running post-create setup script inside the container..."
#docker exec -it $CONTAINER_NAME bash -c "bash $SETUP_SCRIPT"
docker exec -it $CONTAINER_NAME bash
echo "Container $CONTAINER_NAME is ready and running"
