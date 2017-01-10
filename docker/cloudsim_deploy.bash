#!/usr/bin/env bash
# launch docker container, providing env and options

until sudo docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

# cloudsim_deploy lives in the same folder as cloudsim_env.bash and cloudsim_options.json
code_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

DOCKER_GPU_PARAMS=" $(curl -s http://localhost:3476/docker/cli)"

sudo docker run -it \
  -v "/etc/localtime:/etc/localtime:ro" \
  -e DISPLAY=unix$DISPLAY \
  $DOCKER_GPU_PARAMS \
  -v "$code_dir:/code:ro" \
  -p 4000:4000 \
  -p 80:8080 \
  precious \
  /priuscup/entry.bash
