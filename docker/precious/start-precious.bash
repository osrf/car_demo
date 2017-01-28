#!/usr/bin/env bash

function cleanup {
    # This should stop it
    sudo pkill -9 ignition
}
trap cleanup EXIT

source /code/aws_s3_user.bash
source /code/aws_s3_keys.bash

source /usr/share/priuscup/setup.sh
ignition --run /opt/priuscup/prius.ign &
gzserver --verbose --server-plugin libUploadDataPlugin.so --server-plugin libMaxTimeToLivePlugin.so --lifespan=2700 /usr/share/priuscup-0/worlds/raceway.world
