#!/usr/bin/env bash

user_id=$(id -u)
image_name=precious
tag=$(date +%Y_%b_%d_%H%M)
image_plus_tag=$image_name:${tag}
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

function cleanup {
    rm -f $DIR/ssh-key${tag}
    rm -f $DIR/ssh-key${tag}.pub
}
trap cleanup EXIT

ssh-keygen -P "" -f $DIR/ssh-key${tag} -C "temporary for building ${image_plus_tag}"
echo "Temporarily add this ssh-key to your bitbucket account
Once the docker image has finished building, remove the key from your account
(enter once added to continue)"
echo ------------copy key below-------------------
cat $DIR/ssh-key${tag}.pub
echo ------------copy key above-------------------
read junk

private_key=$(base64 --wrap=0 $DIR/ssh-key${tag})

sudo docker build -t $image_plus_tag --build-arg user_id=$user_id $DIR --build-arg "ssh_key=$private_key"
echo "$image_plus_tag"
