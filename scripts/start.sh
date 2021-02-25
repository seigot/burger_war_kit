#!/bin/bash

# set default level 1
VALUE_L="1"
IS_AUTOTEST="FALSE"

# get args level setting
while getopts l:a OPT
do
  case $OPT in
      "l" ) FLG_L="TRUE" ; VALUE_L="$OPTARG" ;;
      "a" ) IS_AUTOTEST="TRUE";;
  esac
done

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node
declare -a ROSLAUNCH_OPTIONS=("enemy_level:=${VALUE_L}")
if [ "${IS_AUTOTEST}" = "TRUE" ]; then
    ROSLAUNCH_OPTIONS+=("rviz_file:=burger_navigation_autotest.rviz")
fi
roslaunch burger_war sim_robot_run.launch ${ROSLAUNCH_OPTIONS[@]}
