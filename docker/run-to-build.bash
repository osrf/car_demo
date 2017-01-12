#!/usr/bin/env bash

# This script is meant to run after precious-base has been created
# It runs the docker container mounting two volumes: /src and /build
# These are to be used to build the software inside of the container
# Afterwards the container should be saved with 'docker commit'


if [ $# -ne 2 ]
then
    echo "Usage: $0 <dir with sourcecode> <dir to save buildstuff>"
    exit 1
fi

SRC_DIR=$1
BUILD_DIR=$2

if [ ! -f /tmp/.docker.xauth ]
then
    export XAUTH=/tmp/.docker.xauth
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z $xauth_list ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch /tmp/.docker.xauth
    fi
fi

# Use lspci to check for the presence of an nvidia graphics card
has_nvidia=`lspci | grep -i nvidia | wc -l`

# Set docker gpu parameters
if [ ${has_nvidia} -gt 0 ]
then
  # check if nvidia-modprobe is installed
  if ! which nvidia-modprobe > /dev/null
  then
    echo nvidia-docker-plugin requires nvidia-modprobe
    echo please install nvidia-modprobe
    exit -1
  fi
  # check if nvidia-docker-plugin is installed
  if curl -s http://localhost:3476/docker/cli > /dev/null
  then
    DOCKER_GPU_PARAMS=" $(curl -s http://localhost:3476/docker/cli)"
  else
    echo nvidia-docker-plugin not responding on http://localhost:3476/docker/cli
    echo please install nvidia-docker-plugin
    echo https://github.com/NVIDIA/nvidia-docker/wiki/Installation
    exit -1
  fi
else
  DOCKER_GPU_PARAMS=" --device=/dev/dri:/dev/dri"
fi


echo "Build the software, and then docker commit before exiting the container.
The container has been started with --rm, and will auto-delete itself when exited.
(enter to continue)"
read scary_warning

sudo docker run -ti \
  -v "/etc/localtime:/etc/localtime:ro" \
  -e DISPLAY=unix$DISPLAY \
  -e XAUTHORITY=/tmp/.docker.xauth \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  $DOCKER_GPU_PARAMS \
  -v "/tmp/.docker.xauth:/tmp/.docker.xauth" \
  -v "$SRC_DIR:/src_rw" \
  -v "$SRC_DIR:/src:ro" \
  -v "$BUILD_DIR:/build" \
  --rm \
  --entrypoint /bin/bash \
  precious-base
