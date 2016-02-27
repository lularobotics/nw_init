#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# copied from get.docker.com
command_exists() {
	command -v "$@" > /dev/null 2>&1
}

user_confirm() {
    NOT_FINISHED=true
    while ${NOT_FINISHED} ;do
        echo -e "$1 [y/n] default($2) "
        read USER_INPUT;
        if [[ "y" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="y";
            NOT_FINISHED=false;
        elif [[ "n" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="n";
            NOT_FINISHED=false;
        elif [[ "" == "${USER_INPUT}" ]];then
            USER_CONFIRM_RESULT="$2";
            NOT_FINISHED=false;
        else
            echo -e "# only y, n, and nothing, are possible choices."
            echo -e "# default is $2"
        fi
    done
}

do_install() {
###############################################################
# we make sure that we have docker installed
###############################################################
if ! command_exists docker; then
    echo -e "###############################################################"
    echo -e "# !!!! IMPORTANT !!!! "
    echo -e "# Docker is not yet installed, we are now installing it."
    echo -e "# This procedure might ask for sudo rights."
    echo -e "# To finish this installation, you have to restart your computer."
    echo -e "# We wait for 10 sec until we start the installation process."
    echo -e "###############################################################"
    sleep 10;
    wget -qO- https://get.docker.com | sh
    sudo usermod -a -G docker $USER
    echo -e "###############################################################"
    echo -e "# !!!! IMPORTANT !!!! "
    echo -e "# We will restart your machine in 10 sec."
    echo -e "# Please RUN this script AGAIN afterwards to finish the initialization."
    echo -e "###############################################################"
    sleep 10;
    sudo shutdown -r now
    exit 0;
fi

if ! command_exists catkin_init_workspace; then
    echo -e "###############################################################"
    echo -e "# ros groovy or indigo is not sourced"
    echo -e "###############################################################"
    if [[ -d "/opt/ros/indigo" ]];then
        echo -e "###############################################################"
        echo -e "# found /opt/ros/indigo/setup.bash"
        echo -e "###############################################################"
        source /opt/ros/indigo/setup.bash

    elif [[ -d "/opt/ros/indigo" ]];then
        echo -e "###############################################################"
        echo -e "# found /opt/ros/groovy/setup.bash"
        echo -e "###############################################################"
        source /opt/ros/groovy/setup.bash
    else
        echo -e "###############################################################"
        echo -e "# ros groove or indigo is require, please install the dependency or source the setup.bash in ,your ros distribution prior to calling this script."
        echo -e "# http://wiki.ros.org/indigo/Installation/Ubuntu"
        echo -e "###############################################################"
        exit 1;
    fi
fi

echo -e "###############################################################"
echo -e "# Set your docker credentials if you haven't already. This only "
echo -e "# Needs to be done once, the first time through:"
echo -e "#   Username: lularoboticsnw"
echo -e "#   Password: private communication"
echo -e "#   Email: your choice :)"
user_confirm "# Update credentials" "n"
if [[ "y" == "${USER_CONFIRM_RESULT}" ]];then
    docker login
fi

echo -e "###############################################################"
echo -e "# We will update binary software package with the latest version"
echo -e "# This could take several minutes the first time through. After"
echo -e "# that the update should be fast."
user_confirm "# Continue" "y"
if [[ "n" == "${USER_CONFIRM_RESULT}" ]];then
    echo -e "stopped since user did want to stop";
    exit 1;
fi

bash ${SCRIPT_DIR}/data/docker_tools.sh --load-image

echo -e "###############################################################"
echo -e "# We will setup a workspace in $(pwd)/lularobotics_ws if not existing"
echo -e "# If existing we will update our repositories"
user_confirm "# Continue" "y"
if [[ "n" == "${USER_CONFIRM_RESULT}" ]];then
    echo -e "stopped since user did not confirm the current directory";
    exit 1;
fi

################################################################################
# setting up our workspace
################################################################################
CUR_DIR=$(pwd)
TMP_PATH=${CUR_DIR}/lularobotics_ws;
if [[ ! -d ${TMP_PATH} ]];then
    mkdir ${TMP_PATH};
fi
TMP_PATH=${CUR_DIR}/lularobotics_ws/src;
if [[ ! -d ${TMP_PATH} ]];then
    mkdir ${TMP_PATH};
fi
cd ${TMP_PATH}
TMP_PATH=${CUR_DIR}/lularobotics_ws/src/CMakeLists.txt;
if [[ ! -e ${TMP_PATH} ]];then
    catkin_init_workspace
fi

TMP_PATH=${CUR_DIR}/lularobotics_ws/src/nw_mico;
if [[ ! -d ${TMP_PATH} ]];then
    echo -e "clone package for nw_mico for urdf and rviz launch"
    git clone https://github.com/lularobotics/nw_mico.git
else
    cd ${TMP_PATH}
    git pull 
fi

TMP_PATH=${CUR_DIR}/lularobotics_ws/src/nw_motion_optimization;
if [[ ! -d ${TMP_PATH} ]];then
    echo -e "clone package for the interaction with the binary distributed motion optimization library"
    git clone https://github.com/lularobotics/nw_motion_optimization.git
else
    cd ${TMP_PATH}
    git pull 
fi
TMP_PATH=${CUR_DIR}/lularobotics_ws/src/nw_mico_client;
if [[ ! -d ${TMP_PATH} ]];then
    echo -e "clone package client example"
    git clone https://github.com/lularobotics/nw_mico_client.git
else
    cd ${TMP_PATH}
    git pull 
fi
TMP_PATH=${CUR_DIR}/lularobotics_ws/src/riemo_move_action;
if [[ ! -d ${TMP_PATH} ]];then
    echo -e "clone package with move action"
    git clone https://github.com/lularobotics/riemo_move_action.git
else
    cd ${TMP_PATH}
    git pull 
fi
cd ${CUR_DIR}/lularobotics_ws;
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo;

echo -e "###############################################################"
echo -e "# Your workspace is now setup in ${CUR_DIR}/lularobotics_ws"
echo -e "###############################################################"
echo -e "# To start the basic example you can call"
echo -e "# bash ${SCRIPT_DIR}/start_example.sh ${CUR_DIR}/lularobotics_ws"
echo -e "###############################################################"
echo -e "#"
echo -e "# To run the basic example do the following:"
echo -e "#"
echo -e "# --- terminal 1:"
echo -e "# source <workspace>/devel/setup.bash"
echo -e "# roslaunch nw_motion_optimization riemo_mico_server_with_robot_emulator.launch"
echo -e "#"
echo -e "# --- terminal 2:"
echo -e "# source <workspace>/devel/setup.bash"
echo -e "# rosrun nw_mico_client run_basic_mico_demos"
echo -e "#"
echo -e "# --- When finished, shutdown with:"
echo -e "# rosrun nw_motion_optimization riemo_mico_server_shutdown.sh"
echo -e "#"
echo -e "###############################################################"
}

# copied from get.docker.com
# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
