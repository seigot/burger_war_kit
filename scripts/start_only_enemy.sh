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

declare -a ROSLAUNCH_OPTIONS=("enemy_level:=${VALUE_L}")
if [ "${IS_AUTOTEST}" = "TRUE" ]; then
    ROSLAUNCH_OPTIONS+=("rviz_file:=burger_navigation_autotest.rviz")
fi
roslaunch burger_war sim_robot_run_only_enemy.launch ${ROSLAUNCH_OPTIONS[@]}
