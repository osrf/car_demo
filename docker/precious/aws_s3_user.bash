#!/bin/bash

# cloudsim-options.json contains a `callback_token` which is supposed to be
# the user id of the simulator. This field will be used to generate unique
# log filenames when uploading to s3 bucket

OPTIONSFILE="/code/cloudsim-options.json"
if [ -f "${OPTIONSFILE}" ]
then
  export PRIUS_USER_ID=`cat ${OPTIONSFILE} | python -c "import sys, json; print json.load(sys.stdin)['callback_token']"`
fi

# backup plan:
# set the user to be CLOUDSIM_ADMIN if callback_token does not exist
if [ "${PRIUS_USER_ID}" == "" ]
then
  ENVFILE="/code/cloudsim-env.bash"
  if [ -f "${OPTIONSFILE}" ]
  then
    PRIUS_USER_ID=`grep CLOUDSIM_ADMIN ${ENVFILE} | awk -F "[=]" '{print $2}'`
    # export var with double quotes removed
    export PRIUS_USER_ID=`echo "${PRIUS_USER_ID//\"}"`
  fi
fi

