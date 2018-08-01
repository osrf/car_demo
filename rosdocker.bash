#!/bin/bash
function _func_rosdockerserver() {
    pc_addr=$(ip -f inet -o addr show docker0|cut -d\  -f 7 | cut -d/ -f 1)
    # pc_addr=$(ifconfig | grep 'inet addr:' | grep -v '127.0.0.1' | awk -F: '{print $2}' | awk '{print $1}' | head -1)
	export ROS_MASTER_URI=http://${pc_addr}:11311
	export ROS_HOST_NAME=${pc_addr}
	export ROS_IP=${pc_addr}
	export PS1="\[\033[41;1;33m\]<ROS_server>\[\033[0m\]\w$ "

	env | grep "ROS_MASTER_URI"
	env | grep "ROS_HOST_NAME"
	env | grep "ROS_IP"
}

function _func_rosdockerclient() {
    if [ -z "$1" ]; then
        echo "Input the Docker Conntener ID.'"
    else
        pc_addr=$(ifconfig | grep 'inet addr:' | grep -v '127.0.0.1' | awk -F: '{print $2}' | awk '{print $1}' | head -1)
        export ROS_MASTER_URI=http://$1:11311
        export ROS_HOST_NAME=${pc_addr}
        export ROS_IP=${pc_addr}
        export PS1="\[\033[44;1;33m\]<ROS_Docker_client>\[\033[0m\]\w$ "
    fi
    env | grep "ROS_MASTER_URI"
    env | grep "ROS_HOST_NAME"
    env | grep "ROS_IP"
}

function _func_rosdockerexit(){
    export ROS_MASTER_URI=http://localhost:11311
    unset ROS_HOST_NAME
    unset ROS_IP
    export PS1="\u@\h:\w\\$ "
}

if [ $1 = "exit" ]; then
    _func_rosdockerexit
elif [ $1 = "server" ]; then
    _func_rosdockerserver
elif [ $1 = "client" ]; then
    _func_rosdockerclient $2
fi
