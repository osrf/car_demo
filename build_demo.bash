#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo docker build -t osrf/car_demo $DIR --build-arg DOCKER_CMD='client 172.17.0.1'
