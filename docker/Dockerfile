FROM sloretz/dev:gz-base

# This is a dockerfile for priuscup

# Dependencies for cloudsim
RUN apt-get update \
 && apt-get install -y \
        redis-server \
        nodejs \
        nodejs-legacy \
        npm \
        libjansson-dev \
        vim \
 && apt-get clean

# folder to hold datafiles related to the prius cup
RUN mkdir /priuscup \
 && mkdir -p /home/ubuntu/code

# Cloudsim-sim template
RUN cd /priuscup \
 && hg clone https://bitbucket.org/osrf/cloudsim-sim \
 && cd cloudsim-sim \
 && npm install \
 && touch /priuscup/cloudsim-sim/server/keys.zip

COPY _in_container_entry.bash /priuscup/entry.bash
RUN chmod a+x /priuscup/entry.bash
# Note, prius.ign was in ign-core
COPY prius.ign /priuscup/prius.ign

# add a new with my userid and group id
ARG user_id
ENV USERNAME precious
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME   ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME \
 && chown -R $USERNAME:$USERNAME /priuscup
USER $USERNAME
WORKDIR /priuscup

# expose port used by priuscup webserver
EXPOSE 8080

CMD ["/priuscup/entry.bash"]
