FROM osrf/ros:kinetic-desktop

LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

RUN apt-get update \
 && apt-get install -y \
    wget \
    lsb-release \
    sudo \
    mesa-utils \
 && apt-get clean


# Get gazebo binaries
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    gazebo8 \
    ros-kinetic-gazebo8-ros-pkgs \
    ros-kinetic-fake-localization \
    ros-kinetic-joy \
 && apt-get clean

ARG DOCKER_CMD="server"
RUN apt-get install net-tools
RUN mkdir -p /tmp/workspace/script
COPY rosdocker.bash /tmp/workspace/script


RUN mkdir -p /tmp/workspace/src
COPY prius_description /tmp/workspace/src/prius_description
COPY prius_msgs /tmp/workspace/src/prius_msgs
COPY car_demo /tmp/workspace/src/car_demo
RUN /bin/bash -c 'cd /tmp/workspace \
 && source /opt/ros/kinetic/setup.bash \ 
 && catkin_make'


CMD ["/bin/bash", "-c", "shopt -s expand_aliases \
    && source /opt/ros/kinetic/setup.bash \
    && shopt \
    && source /tmp/workspace/script/rosdocker.bash \
    && rosdocker client 172.17.0.1 \
    && source /tmp/workspace/devel/setup.bash \
    && env | grep \"ROS_MASTER_URI\" \
    && env | grep \"ROS_HOST_NAME\" \
    && env | grep \"ROS_IP\" \
    && roslaunch car_demo demo.launch"]
