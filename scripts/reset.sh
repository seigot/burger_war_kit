#!/bin/bash

### 
#  reset gazebo, judgeserver.
#  
#  usage)
#  $ cd ~/catkin_ws/src/burger_war_kit
#  $ bash scripts/reset.sh
###

# reset judge
# check arg num
if [ $# -ne 2 ]; then
    RED_NAME="you"
    BLUE_NAME="enemy"
else
    RED_NAME=$1
    BLUE_NAME=$2
fi

#### define function
function try_kill_process_main(){
    PROCESS_NAME=$1
    if [ -z "$PROCESS_NAME" ]; then
	return 1
    fi
    
    PROCESS_ID=`ps -e -o pid,cmd | grep ${PROCESS_NAME} | grep -v grep | awk '{print $1}'`
    if [ -z "$PROCESS_ID" ]; then
	echo "no process like... ${PROCESS_NAME}"
	return 2
    fi
    echo "kill process ... ${PROCESS_NAME}"
    #kill -SIGINT $PROCESS_ID
    kill $PROCESS_ID

    return 0
}

IS_SYNC_KILL="false"
function try_kill_process(){
    PROCESS_NAME=$1

    DURATION=2
    LOOP_TIMES=100
    for i in `seq ${LOOP_TIMES}`
    do
	try_kill_process_main ${PROCESS_NAME}
	RET=$?
	if [ $RET == 2 ];then
	    # if no process, break
	    break
	fi

	if [ $IS_SYNC_KILL != "true" ];then
	    # if not sync kill, break
	    break
	fi	
	sleep ${DURATION}
    done
}

# kill process
try_kill_process "start.sh"
sleep 1

# reset judgeserver
bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000  $RED_NAME $BLUE_NAME

# reset gazebo model
rosservice call /gazebo/reset_simulation "{}"

