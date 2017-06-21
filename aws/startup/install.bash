#!/usr/bin/env bash

# Configures an EC2 instance running ubuntu-nvidia-docker-16GiB

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Need to start an Xorg server with some custom options on :0
# but lightdm already starts an Xserver on that display, so stop it first
echo "stopping lightdm"
sudo service lightdm stop
echo "disapling lightdm"
sudo systemctl disable lightdm
echo "enabling custom Xorg service"
sudo systemctl enable ${dir}/Xorg.service
