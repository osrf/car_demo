# PriusCup Docker Image

## Building
These steps will walk through creating a docker image for PriusCup.

### Step 0 - Prerequisites
1. [Install docker](https://docs.docker.com/engine/installation/)
2. [Install nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

### Step 1 - Make precious-base
At the end of this step there will be a docker image called **precious-base** that has

- all depencencies except sdformat, gazebo, and ignition libraries
- a folder */priuscup* with
    - cloudsim-sim
    - entry.bash

---
1. Download the ssh key to be used on the machine. Zip it, and save the archive as *cloudsim.pem.zip*  in the folder *priuscup/docker*
2. Run the following script to build a docker image
```
cd priuscup/docker
./build-base.bash
```
###### Note: This image builds from [sloretz/dev:gz-base](https://bitbucket.org/sloretz/dev_docker)

### Step 2 - Install Priuscup software
At the end of this step there will be a docker image called **precious** that

- has all gazebo, sdformat, ignition, and PriusCup software installed to */usr/local*
- can be deployed to AWS or tested locally

___
1. Create a folder called *docker_build/*
2. Create a folder called *docker_src/*
    1. Use mercurial to clone the following repositories inside *docker_src/*
    	- https://bitbucket.org/osrf/gazebo
    	- https://bitbucket.org/osrf/sdformat
    	- https://bitbucket.org/ignitionrobotics/ign-common
    	- https://bitbucket.org/ignitionrobotics/ign-core
    	- https://bitbucket.org/ignitionrobotics/ign-math
    	- https://bitbucket.org/ignitionrobotics/ign-msgs
    	- https://bitbucket.org/ignitionrobotics/ign-transport
    	- https://bitbucket.org/osrf/priuscup
3. It is time to launch the **precious-base** image created earlier
`./run-to-build.bash path/to/docker_src path/to/docker_build`
	1. Inside the container there are three important folders:
		1. */src_rw* - links to *docker_src/*
		2. */src* - links to *docker_src/* (read-only)
		3. */build* - links to *docker_build/*
	2. Either build and install all repos in */src* manually **OR** Run this script
	`/src/priuscup/docker/build-pc.bash`
	**Don't exit the container yet!** 
4. Save the container as a new docker image
	1. Use `sudo docker ps` to get the container id
	2. Use `sudo docker commit container_id precious:latest` to save the container as precious latest
	
## Testing Locally
1. Start the docker image
```
cd priuscup/aws
touch cloudsim-env.bash
echo "{}" > cloudsim-options.bash
./cloudsim_deploy.bash
```
2. Open a browser to `http://127.0.0.1/`

## Deploying to AWS
1. Start a g2.2xlarge instance with the AMI *ubuntu-nvidia-docker-16GiB* *ami-b71142d7*
2. Use the following script to copy the image to the instance
```
cd priuscup/aws
./deploy-ec2.bash
```
3. Save the AMI