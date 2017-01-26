FROM cloudsim-sim-base:xenial

# This is a dockerfile for priuscup. It provides
#   1) installs nightly version of gazebo/sdformat, ignition libraries
#   2) clean build of all prius cup software

# Tools
RUN apt-get update \
 && apt-get install -y \
    gdb \
 && apt-get clean


# Get stable dependencies first
RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list \
 && wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add - \
 && apt-get update \
 && apt-get install -y \
    libignition-transport3 \
    libignition-transport3-dev \
    libignition-math3 \
    libignition-math3-dev \
    libsdformat5 \
    libsdformat5-dev \
 && apt-get clean


# Install nightly build of gazebo8
# RUN echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-nightly `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-nightly.list \
#  && apt-get update \
#  && apt-get install -y \
#     gazebo8 \
#     gazebo8-common \
#     gazebo8-plugin-base \
#     libgazebo8 \
#     libgazebo8-dev \
#  && apt-get clean


# copy datafiles related to the prius cup
RUN mkdir /opt/priuscup
COPY start-precious.bash /opt/priuscup/start-precious.bash
RUN chmod a+x /opt/priuscup/start-precious.bash
COPY prius.ign /opt/priuscup/prius.ign


# add a user with my id
ARG user_id
ENV USERNAME precious
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME   ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME \
 && chown -R $USERNAME:$USERNAME /opt/priuscup \
 && chown -R $USERNAME:$USERNAME /opt/cloudsim-sim
USER $USERNAME
WORKDIR /opt/priuscup


# Install priuscup source code from repos
ARG ssh_key
RUN mkdir /tmp/code \
 && echo ${ssh_key} > /tmp/code/ssh_key.b64 \
 && base64 --decode /tmp/code/ssh_key.b64 > /tmp/code/ssh_key \
 && chmod 700 /tmp/code/ssh_key \
 && printf '[ui]\nssh = ssh -o StrictHostKeyChecking=no -C -i /tmp/code/ssh_key ' > /tmp/code/hgrc \
 && export HGRCPATH=/tmp/code/hgrc \
 && cd /tmp/code \
 && hg clone ssh://hg@bitbucket.org/ignitionrobotics/ign-msgs -r precious \
 && mkdir build-ign-msgs && cd build-ign-msgs && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/usr /tmp/code/ign-msgs && make -j8 && sudo make install \
 && cd /tmp/code \
 && hg clone ssh://hg@bitbucket.org/ignitionrobotics/ign-common -r default \
 && mkdir build-ign-common && cd build-ign-common && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/usr /tmp/code/ign-common && make -j8 && sudo make install \
 && cd /tmp/code \
 && hg clone ssh://hg@bitbucket.org/ignitionrobotics/ign-core -r precious \
 && mkdir build-ign-core && cd build-ign-core && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/usr /tmp/code/ign-core && sudo make -j8 && sudo make install \
 && cd /tmp/code \
 && hg clone ssh://hg@bitbucket.org/osrf/gazebo -r gazebo8 \
 && mkdir build-gazebo && cd build-gazebo && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/usr /tmp/code/gazebo && make -j8 && sudo make install \
 && cd /tmp/code \
 && hg clone ssh://hg@bitbucket.org/osrf/priuscup -r precious \
 && mkdir build-priuscup && cd build-priuscup && cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_INSTALL_PREFIX:PATH=/usr /tmp/code/priuscup && make -j8 && sudo make install \
 && sudo rm -rf /tmp/code


# expose port used by priuscup webserver
EXPOSE 8080

CMD ["/opt/priuscup/start-precious.bash"]
