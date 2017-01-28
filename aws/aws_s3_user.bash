#!/bin/bash

# cloudsim-options.json contains a `callback_token` which is supposed to be
# the user id of the simulator. This field will be used to generate unique
# log filenames when uploading to s3 bucket

OPTIONSFILE="/code/cloudsim-options.json"
if [ -f "${OPTIONSFILE}" ]
then
  export PRIUS_USER_ID=`cat ${OPTIONSFILE} | python -c "import sys, json; print json.load(sys.stdin)['callback_token']"`
fi


