#!/bin/sh
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd)"

# copied from get.docker.com
command_exists() {
	command -v "$@" > /dev/null 2>&1
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
    echo -e "# Please restart your computer now"
    echo -e "###############################################################"
    exit 0;
fi

if ! command_exists catkin_init_workspace; then
    echo -e "###############################################################"
    echo -e "# ros groovy or above is not sourced"
    echo -e "###############################################################"
    if [[ -d "/opt/ros/indigo" ]];then
        echo -e "###############################################################"
        echo -e "# found /opt/ros/indigo/setup.bash"
        echo -e "###############################################################"
        source /opt/ros/indigo/setup.bash
    else
        echo -e "###############################################################"
        echo -e "# ros groove >= is require, please install the dependency or source the setup.bash in ,your ros distribution prior to calling this script."
        echo -e "# http://wiki.ros.org/indigo/Installation/Ubuntu"
        echo -e "###############################################################"
        exit 1;
    fi
fi

echo -e "###############################################################"
echo -e "# We will now start loading our binary software package"
echo -e "# this will take several minutes"
echo -e "# Continue [y/N] "
echo -e "###############################################################"
read SETUP_READY;
if [[ "y" != "${SETUP_READY}" ]];then
    echo -e "stopped since you did not confirm";
    exit 1;
fi

bash ${SCRIPT_DIR}/data/docker_tools.sh --load-image

echo -e "###############################################################"
echo -e "# We will now start setting up a new workspace for our example"
echo -e "# in the current directory $(pwd)"
echo -e "# Continue [y/N] "
echo -e "###############################################################"
read SETUP_READY;
if [[ "y" != "${SETUP_READY}" ]];then
    echo -e "stopped since you did not confirm the current directory";
    exit 1;
fi

################################################################################
# setting up our workspace
################################################################################
CUR_DIR=$(pwd)
mkdir ${CUR_DIR}/lularobotics_ws;
mkdir ${CUR_DIR}/lularobotics_ws/src;
cd ${CUR_DIR}/lularobotics_ws/src;
catkin_init_workspace

echo -e "clone package for the interaction with the binary distributed motion optimization library"
git clone git@github.com:lularobotics/nw_motion_optimization.git
echo -e "clone package with messages"
git clone git@github.com:lularobotics/nw_messages.git
echo -e "clone package with an example client, illustrating how to use motion optimization"
git clone git@github.com:lularobotics/nw_example.git
cd ${CUR_DIR}/lularobotics_ws;
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo;

}

# copied from get.docker.com
# wrapped up in a function so that we have some protection against only getting
# half the file during "curl | sh"
do_install
