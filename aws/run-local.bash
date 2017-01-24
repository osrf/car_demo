#/usr/bin/env bash

dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROFILEDIR=$(mktemp -p /tmp -d tmp-fx-profile.XXXXXX.d)

function cleanup {
    echo "Post run cleanup!"
    rm -rf $PROFILEDIR
    sudo docker stop $(sudo docker ps -aq --filter 'name=aws_priuscup_')
}
trap cleanup EXIT

# start precious software in the background
echo "{}" > $dir/cloudsim-options.json
touch $dir/cloudsim-env.bash
$dir/cloudsim_deploy.bash --no-shutdown &
echo "Waiting for precious to start"
sleep 10

# start a web browser
firefox -profile $PROFILEDIR -no-remote -new-instance 127.0.0.1

