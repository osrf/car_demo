#!/usr/bin/env bash

# Runs a docker container with the image created by build_demo.bash
# Requires
#   docker
#   nvidia-docker2
#   an X server
# Recommended
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

until nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

mkdir -p /tmp/car_demo_rocker_venv
python3 -m venv /tmp/car_demo_rocker_venv
. /tmp/car_demo_rocker_venv/bin/activate
pip install -U git+https://github.com/osrf/rocker.git


rocker --nvidia --user --exec --devices /dev/input/js0 /dev/input/js1 -- osrf/car_demo
