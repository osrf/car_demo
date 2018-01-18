#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

sudo docker build -t osrf/ros:kinetic-desktop-opengl $DIR
