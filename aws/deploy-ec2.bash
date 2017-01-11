#!/usr/bin/env bash

if [ $# -ne 3 ]
then
    echo "Usage: $0 <docker image:tag> <path to ssh key> <user name>@<ec2host>"
    exit 1
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

image_name=$1
path_to_key=$2
dest=$3

# Get password before piping stuff
sudo echo "Loading"

# copy the cloudsim_deploy.bash script to the ec2 instance
scp -i $path_to_key $DIR/cloudsim_deploy.bash $dest:code/cloudsim_deploy.bash

# Copy the docker image to an ec2 instance running ubuntu-nvidia-docker AMI
sudo docker save $image_name | bzip2 | pv | ssh -i $path_to_key $dest 'bunzip2 | docker load'


