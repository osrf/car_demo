#!/usr/bin/env bash

function cleanup {
    # This should stop it
    sudo pkill -9 ignition
}
trap cleanup EXIT

source /usr/share/priuscup/setup.sh
ignition --run /opt/priuscup/prius.ign &
gzserver --verbose /usr/share/priuscup-0/worlds/raceway.world
