#!/bin/bash

if [ "$#" -ne 1 ]
then
  echo "Usage: upload.bash filename"
  exit 1
fi

if [ -z "$1" ]
then
  echo "File to upload is not set"
  exit 1
fi


if [ -z "${PRIUS_USER_ID}" ]
then
  echo "PRIUS_USER_ID is not set"
  exit 1
fi


DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# upload to s3 bucket
TARGET_FILENAME="${PRIUS_USER_ID}_prius_data.txt"
python  ${DIR}/upload.py $1 ${TARGET_FILENAME} ${AWS_ACCESS_KEY_ID} ${AWS_SECRET_ACCESS_KEY}

# hyperdrive webhook
LOG_PATH="http://priusdata.s3-website-us-west-1.amazonaws.com/${TARGET_FILENAME}"
URL=http://onramp.hyperdrive.me/osrf
curl -G -X POST $URL --data-urlencode "token=${PRIUS_USER_ID}" --data-urlencode "path=${LOG_PATH}"

