#!/bin/bash -e
user_id=$(id -u)
image_name=precious-base
image_plus_tag=$image_name:$(date +%Y_%b_%d_%H%M)

sudo docker build -t $image_plus_tag --build-arg user_id=$user_id .
# Make latest point to this build
sudo docker tag $image_plus_tag $image_name

printf "\n\n\n\n"
echo "Precious image is not complete!"
echo "Run ./run-to-build.bash next"
echo "Then build/install ignition, sdformat, gazebo, and priuscup inside of the container"
echo "then docker commit"
