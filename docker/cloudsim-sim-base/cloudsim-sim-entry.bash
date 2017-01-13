#!/usr/bin/env bash

# http://redsymbol.net/articles/bash-exit-traps/
function cleanup {
    # Stop cloudsim-sim
    sudo service redis-server stop
    cd /opt/cloudsim-sim
    sudo npm stop
}
trap cleanup EXIT

# Start cloudsim-sim
sudo service redis-server start
cd /opt/cloudsim-sim
cp /code/cloudsim-env.bash .env
cp /code/cloudsim-options.json options.json
npm start &

# Do the CMD
eval "$@"
