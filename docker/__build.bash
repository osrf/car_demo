#!/bin/bash -e

red=$(tput setaf 1)
green=$(tput setaf 2)
normal=$(tput sgr0)

cwd=$(pwd)
SRC=/src/
SRC_RW=/src_rw/
BLD=/build/

length=${#components[@]}
for ((i=0;i<$length;i++))
do
    comp=${components[$i]}
    branch=${branches[$i]}
    printf "${green}About to begin $comp on branch $branch${normal}\n"
    cur_branch=$(cat $SRC/$comp/.hg/branch)
    if [ $cur_branch != $branch ]
    then
        cd $SRC_RW/$comp
        hg co $branch
        cd -
        cur_branch=$(cat $SRC/$comp/.hg/branch)
        if [ $cur_branch != $branch ]
        then
            printf "${red}Could not checkout ${comp}:${branch}${normal}\n"
            exit 1
        fi
    fi
    printf "${green}begin $comp${normal}\n"

    # Make a build directory just for this component/branch
    if [ ! -d $BLD/${comp}_$branch ]
    then
        build_exists="false"
        mkdir $BLD/${comp}_$branch
    else
        build_exists="true"
    fi
    
    cd $BLD/${comp}_$branch
    if [ "ign-core" = $comp ]
    then
        # run as root to workaround for ign-core issue #1
        if [ $build_exists = "false" ]
        then
            sudo cmake $SRC/$comp
        fi
        sudo make -j8
    else
        if [ $build_exists = "false" ]
        then
            cmake $SRC/$comp
        fi
        make -j8
    fi
    sudo make install
    printf "${red}end $comp${normal}\n"
done

cd $cwd

