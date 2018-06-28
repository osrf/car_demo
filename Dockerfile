FROM osrf/ros:melodic-desktop

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys D2486D2DD83DB69272AFE98867170598AF249743

# setup sources.list
RUN . /etc/os-release \
    && echo "deb http://packages.osrfoundation.org/gazebo/$ID-stable `lsb_release -sc` main" > /etc/apt/sources.list.d/gazebo-latest.list

# setup catkin workspace
ENV CATKIN_WS=/root/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# copy source code
COPY prius_description ./prius_description
COPY prius_msgs ./prius_msgs
COPY car_demo ./car_demo
RUN git clone https://github.com/ros-planning/navigation.git
RUN git clone https://github.com/ros-drivers/joystick_drivers.git

# install package dependacies
RUN apt-get -qq update && \
    apt-get -qq install -y \
      python-catkin-tools && \
    rosdep update && \
    rosdep install -y \
      --from-paths . \
      --ignore-src \
      --rosdistro ${ROS_DISTRO} \
      --as-root=apt:false && \
    rm -rf /var/lib/apt/lists/*

# build from source
WORKDIR $CATKIN_WS
ENV TERM xterm
ENV PYTHONIOENCODING UTF-8
RUN catkin config \
      --extend /opt/ros/$ROS_DISTRO && \
    catkin build

# install display dependencies
RUN apt-get update && apt-get install -y \
      libglvnd0 \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

 # nvidia-container-runtime (nvidia-docker2)
 ENV NVIDIA_VISIBLE_DEVICES \
     ${NVIDIA_VISIBLE_DEVICES:-all}
 ENV NVIDIA_DRIVER_CAPABILITIES \
     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# setup entrypoint
COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
