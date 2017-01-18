# PriusCup Docker Image

## Building
These steps will walk through creating a docker image for PriusCup.

### Step 0 - Prerequisites
1. [Install docker](https://docs.docker.com/engine/installation/)
2. [Install nvidia-docker](https://github.com/NVIDIA/nvidia-docker/wiki/Installation)

### Step 1 - Build simulation-base

```
cd priuscup/docker/simulation-base
./build.bash
```

### Step 2 - Build cloudsim-sim-base
```
cd priuscup/docker/cloudsim-sim-base
./build.bash
```

### Step 3 - Build precious
```
cd priuscup/docker/precious
./build.bash
```
1. This script will output a public ssh key. Add this key to your bitbucket account.
2. Tag the output image to `precious:latest`
```
sudo docker tag precious:2017_Jan_12_1914 precious:latest
```
	
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