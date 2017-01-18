#!/usr/bin/env bash
# launch docker container, providing env and options


extra_opts=""
cmd=""
if [ $# -eq 1 ]
then
    if [ "--debug" = $1 ]
    then
        # make it easy to run gdb inside the container
        extra_opts="-it --security-opt seccomp=unconfined"
        cmd=bash
    fi
fi

until sudo nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done

# cloudsim_deploy lives in the same folder as cloudsim_env.bash and cloudsim_options.json
code_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist :0 | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Display is hard-coded to :0 because that's what the startup scripts on AWS will generate
eval sudo nvidia-docker run \
  -e DISPLAY=unix:0 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "$code_dir:/code:ro" \
  -p 4000:4000 \
  -p 80:8080 \
  $extra_opts precious:latest $cmd
