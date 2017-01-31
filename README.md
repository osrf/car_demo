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

## S3 bucket file upload

1. To enable uploading prius data to s3 bucket, create an `aws_s3_keys.bash`
file in `priuscup/aws` and export `AWS_ACCESS_KEY_ID` and
`AWS_SECRET_ACCESS_KEY` variables.


## Testing Locally
### option 1: cloudsim_deploy.bash
1. Start the docker image
```
cd priuscup/aws
echo "CLOUDSIM_ADMIN=\"abc\"" > $dir/cloudsim-env.bash
echo "{}" > cloudsim-options.bash
./cloudsim_deploy.bash --no-shutdown
```
2. Open a browser to `http://127.0.0.1/`

### option 2: run-local.bash
```
cd priuscup/aws
./run-local.bash
```
After a few seconds a firefox window will open to `127.0.0.1`

## Deploying to AWS
1. Start a g2.2xlarge instance with the AMI *ubuntu-nvidia-docker-16GiB* *ami-b71142d7*
2. Use the following script to copy the image to the instance
```
cd priuscup/aws
./deploy-ec2.bash
```
3. Save the AMI

## Setting up a local demo
1. Build a docker image (see instructions above)
2. Save the image to a tar archive
    `sudo docker save --output=/tmp/precious.tar precious:latest`
3. Copy the tar archive to the demo machine and load it into docker
    `sudo docker load --input=/tmp/precious.tar`
4. Copy the `aws` folder to the demo machine
5. To start a demo on the local machine, call `./run-local.bash`
