#!/usr/bin/env bash


until sudo nvidia-docker ps
do
    echo "Waiting for docker server"
    sleep 1
done


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
sudo nvidia-docker run -it \
  -e DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/var/run/spnav.sock:/var/run/spnav.sock" \
  -v "/dev/input:/dev/input" \
  --privileged \
  --rm=true \
  car_demo
