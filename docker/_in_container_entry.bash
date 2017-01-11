#!/usr/bin/env bash

# This is the entry point for the docker container. It start's ignition, the cloudsim-sim server, and gzserver

# Start cloudsim-sim
sudo service redis-server start
cd /priuscup/cloudsim-sim
cp /code/cloudsim-env.bash .env
cp /code/cloudsim-options.json options.json
npm start &

source /usr/local/share/priuscup/setup.sh
ignition --run /priuscup/prius.ign &
gzserver --verbose /usr/local/share/priuscup-0/worlds/sonoma_raceway_box.world

ret=$?
if [ ret -ne 0 ]
then
    echo "Gazebo exited with code $ret"
fi
