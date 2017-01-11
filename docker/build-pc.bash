#!/bin/bash -e

components=(ign-math ign-math  ign-common          ign-msgs ign-transport ign-core sdformat gazebo   priuscup)
branches=(  default  ign-math2 depend_on_ign-math3 precious default       gamepad  default  gazebo8  dockerize)
install_only=1

# Run build script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
source $DIR/__build.bash
