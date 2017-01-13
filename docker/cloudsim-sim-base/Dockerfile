FROM simulation-base:xenial

# Starting point for cloudsim simulation projects
#   1) inherits dependencies for building gazebo + ignition + sdformat
#   2) provides a cloudsim-sim install in /opt/cloudsim-sim

# Cloudsim-sim dependencies
RUN apt-get update \
 && apt-get install -y \
        redis-server \
        nodejs \
        nodejs-legacy \
        npm \
 && apt-get clean

# Install cloudsim-sim (default branch)
RUN cd /opt \
 && hg clone https://bitbucket.org/osrf/cloudsim-sim \
 && cd /opt/cloudsim-sim \
 && npm install \
 && touch /opt/cloudsim-sim/server/keys.zip

# Cloudsim-portal provides files in a directory called ~code
# These must get mounted as /code in the container using a docker volume
VOLUME ["/code"]

# Install an entrypoint script which launches cloudsim-sim
# Projects should set their start points with CMD
COPY cloudsim-sim-entry.bash /cloudsim-sim-entry.bash
ENTRYPOINT ["/cloudsim-sim-entry.bash"]
CMD ["echo", "Someone forgot to set the CMD!"]

