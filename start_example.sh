#!/bin/bash
set -e

# copied from get.docker.com
command_exists() {
	command -v "$@" > /dev/null 2>&1
}

if [[ $# != 1 ]];then
    echo -e "###############################################################"
    echo -e "# we require exactly one parameters the path to lularobotics_nw"
    echo -e "###############################################################"
    exit 1
fi

DIR_LULAROBOTICS_NW="$1"

if [[ ! -d ${DIR_LULAROBOTICS_NW} ]];then
    echo -e "###############################################################"
    echo -e "# the path to lularobotics_nw ${DIR_LULAROBOTICS_NW} is not a directory"
    echo -e "###############################################################"
    exit 1
fi

cd ${DIR_LULAROBOTICS_NW};

if [[ ! -e ${DIR_LULAROBOTICS_NW}/devel/setup.bash ]];then
    echo -e "###############################################################"
    echo -e "# the workspace ${DIR_LULAROBOTICS_NW} seems not to be a compiled"
    echo -e "# catkin workspace. Please call catkin_make in ${DIR_LULAROBOTICS_NW}"
    echo -e "###############################################################"
    exit 1
fi
source ${DIR_LULAROBOTICS_NW}/devel/setup.bash

if ! command_exists roscore; then
    echo -e "###############################################################"
    echo -e "# ros seems to be not installed properly or"
    echo -e "# the setup.bash was not sourced properly."
    echo -e "###############################################################"
    exit 1
fi

echo -e "###############################################################"
echo -e "# starting up the roscore"
echo -e "###############################################################"
roscore > /dev/null 2>&1&
sleep 1;
echo -e "###############################################################"
echo -e "# starting up rviz"
echo -e "###############################################################"
roslaunch nw_mico_client mico_rviz_only.launch > /dev/null 2>&1&
sleep 4;
echo -e "###############################################################"
echo -e "# starting up motion optimization emulator"
echo -e "###############################################################"
rosrun nw_motion_optimization start_motion_optimization_emulator.sh > /dev/null 2>&1&
echo -e "###############################################################"
echo -e "# starting up motion optimization service"
echo -e "###############################################################"
rosrun nw_motion_optimization start_motion_optimization_service.sh > /dev/null 2>&1&
echo -e "###############################################################"
echo -e "# running the client"
echo -e "###############################################################"
rosrun nw_mico_client run_riemo_move_mico_playground -.3 .5 .3

echo -e "###############################################################"
echo -e "# we are planning in the background please wait"
echo -e "# the robot should move soon :)"
echo -e "###############################################################"
read USER_INPUT;


echo -e "###############################################################"
echo -e "# stopping motion optimization emulator"
echo -e "###############################################################"
rosrun nw_motion_optimization stop_motion_optimization_emulator.sh > /dev/null 2>&1&
echo -e "###############################################################"
echo -e "# stopping motion optimization service"
echo -e "###############################################################"
rosrun nw_motion_optimization stop_motion_optimization_service.sh > /dev/null 2>&1&
