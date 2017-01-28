#!/bin/bash

OPTIONSFILE="/code/cloudsim-options.json"
if [ -f "${OPTIONSFILE}" ]
then
  export PRIUS_USER_ID=`cat ${OPTIONSFILE} | python -c "import sys, json; print json.load(sys.stdin)['callback_token']"`
fi


