#!/bin/bash -x

cd $HOME/catkin_ws/src/burger_war_kit
CATKIN_WS_DIR=$HOME/catkin_ws
BURGER_WAR_KIT_REPOSITORY=$HOME/catkin_ws/src/burger_war_kit
BURGER_WAR_DEV_REPOSITORY=$HOME/catkin_ws/src/burger_war_dev
BURGER_WAR_AUTOTEST_LOG_REPOSITORY=$HOME/catkin_ws/src/burger_war_autotest
RESULTLOG=$BURGER_WAR_KIT_REPOSITORY/autotest/result.log
SRC_LOG=$RESULTLOG
TODAY=`date +"%Y%m%d"`
DST_LOG=$BURGER_WAR_AUTOTEST_LOG_REPOSITORY/result/result-${TODAY}.log
LATEST_GITLOG_HASH="xxxx"

echo "iteration, enemy_level, game_time(s), date, my_score, enemy_score, battle_result, my_side" > $RESULTLOG

# get option
LOOP_TIMES=10
IS_CAPTURE_VIDEO="false"
while getopts l:c: OPT
do
  case $OPT in
    "l" ) LOOP_TIMES="$OPTARG" ;;
    "c" ) IS_CAPTURE_VIDEO="$OPTARG" ;;
  esac
done
# echo option parameter
echo "LOOP_TIMES: ${LOOP_TIMES}"
echo "IS_CAPTURE_VIDEO: ${IS_CAPTURE_VIDEO}"


pushd ${BURGER_WAR_KIT_REPOSITORY}
source autotest/slack.sh
popd

function search_window() {
    xdotool search --sync --onlyvisible --name "$1"
}

function adjust_layout() {
    local -ir UNIT=700

    # Gazebo window is shown with maximized
    # Ref: https://stackoverflow.com/questions/23850499/how-to-move-or-resize-x11-windows-even-if-they-are-maximized
    local -r GAZEBO=$(search_window "Gazebo")
    wmctrl -i -r ${GAZEBO} -b remove,maximized_vert,maximized_horz
    xdotool windowunmap --sync ${GAZEBO}
    xdotool windowmap --sync ${GAZEBO}
    wmctrl -i -r ${GAZEBO} -e 0,0,0,${UNIT},${UNIT}
    xdotool windowactivate --sync ${GAZEBO}

    local -r SCORE_BOARD=$(search_window "burger war")
    wmctrl -i -r ${SCORE_BOARD} -e 0,0,${UNIT},${UNIT},${UNIT}
    xdotool windowactivate --sync ${SCORE_BOARD}

    local -r RVIZ=$(search_window "RViz")
    wmctrl -i -r ${RVIZ} -e 0,${UNIT},0,${UNIT},$((2*${UNIT}))
    xdotool windowactivate --sync ${RVIZ}
}

function rename_log(){
    local -r FILE_BASE_NAME="$1"
    local -r LOG_DIR="${HOME}/.ros/log/"
    if [ -L ${LOG_DIR}/latest ]; then
	mv $(realpath ${LOG_DIR}/latest) ${LOG_DIR}/${FILE_BASE_NAME}
	rm ${LOG_DIR}/latest
    fi
}

function do_game(){
    ITERATION=$1
    ENEMY_LEVEL=$2
    GAME_TIME=$3
    MY_SIDE=$4 # myside parameter currently doesn't work..
    if [ -z $MY_SIDE ]; then
	MY_SIDE="r"
    fi

    # change directory
    pushd ${BURGER_WAR_KIT_REPOSITORY}

    # start capture
    do_capture "start"
    
    # wakeup gazebo/judgeserver
    PROCESS_NUM=`ps -ux | grep "sim_with_judge.sh" | grep -v "grep"  | wc -l`
    if [ $PROCESS_NUM -eq 0 ]; then
	# wakeup at once
	gnome-terminal -- bash scripts/sim_with_judge.sh -a # -s ${MY_SIDE}
	sleep 30
    fi
    # start
    gnome-terminal -- bash scripts/start.sh -l ${ENEMY_LEVEL} -a # -s ${MY_SIDE}
    adjust_layout

    # wait game finish
    sleep $GAME_TIME

    # get result
    timeout 30s python autotest/get_score.py > out.log
    if [ $MY_SIDE == "r" ]; then
	MY_SCORE=`cat out.log | grep -w my_score | cut -d'=' -f2`
	ENEMY_SCORE=`cat out.log | grep -w enemy_score | cut -d'=' -f2`
    else
	# MY_SIDE != r, means mybot works enemy side..
	MY_SCORE=`cat out.log | grep -w enemy_score | cut -d'=' -f2`
	ENEMY_SCORE=`cat out.log | grep -w my_score | cut -d'=' -f2`
    fi
    DATE=`date --iso-8601=seconds`
    BATTLE_RESULT="LOSE"
    if [ $MY_SCORE -gt $ENEMY_SCORE ]; then
	BATTLE_RESULT="WIN"
    fi

    # output result
    echo "$ITERATION, $ENEMY_LEVEL, $GAME_TIME, $DATE, $MY_SCORE, $ENEMY_SCORE, $BATTLE_RESULT, $MY_SIDE" >> $RESULTLOG
    tail -1 $RESULTLOG
    send_slack "$ITERATION" "$ENEMY_LEVEL" "$GAME_TIME" "$DATE" "$MY_SCORE" "$ENEMY_SCORE" "$BATTLE_RESULT" "$MY_SIDE"

    # save video
    TODAY=`date +"%Y%m%d"`
    VIDEO_DIRECTORY_PATH="${HOME}/video/${TODAY}/"
    mkdir -p ${VIDEO_DIRECTORY_PATH}
    local -r FILE_BASE_NAME="GAME_${DATE}_${ITERATION}_${ENEMY_LEVEL}_${GAME_TIME}_${MY_SCORE}_${ENEMY_SCORE}_${BATTLE_RESULT}_${MY_SIDE}"
    local -r VIDEO_NAME="${VIDEO_DIRECTORY_PATH}/${FILE_BASE_NAME}.mp4"
    do_capture "stop" "$VIDEO_NAME"

    # upload video to youtube if LOSE
    if [ -f "${VIDEO_NAME}" ] && [ "${BATTLE_RESULT}" = "LOSE" ]; then
	if which youtube-upload > /dev/null &&
		[ -f "${HOME}/.client_secrets.json" ] &&
		[ -f "${HOME}/.youtube-upload-credentials.json" ]; then
	    local VIDEO_TITLE="$(basename ${VIDEO_NAME})"
	    local VIDEO_ID=$(youtube-upload --title "${VIDEO_TITLE}" "${VIDEO_NAME}")
	    if [ $? -eq 0 ]; then
		send_slack_video "${VIDEO_TITLE}" "${VIDEO_ID}"
	    fi
	else
	    echo "${FUNCNAME[0]}: ${LINENO}: youtube-upload is unavailable" > /dev/stderr
	fi
    fi

    ## reset
    #bash scripts/reset.sh
    #sleep 3
    # stop
    stop_game
    rename_log "${FILE_BASE_NAME}"

    popd
}

function stop_game(){
    # stop
    # wait stop until all process is end
    pushd ${BURGER_WAR_KIT_REPOSITORY}
    bash scripts/stop.sh -s "true"
    sleep 10
    popd
}

function do_capture(){

    local CAPTURE_OPTION=$1
    local VIDEO_NAME=$2
    
    # if true, capture video
    if [ ${IS_CAPTURE_VIDEO} != "true" ]; then
	echo "skip capture"
	return 0
    fi

    pushd ${BURGER_WAR_KIT_REPOSITORY}
    if [ ${CAPTURE_OPTION} == "start" ]; then    
	bash scripts/capture.sh -m "start"
    elif [ ${CAPTURE_OPTION} == "stop" ]; then
	bash scripts/capture.sh -m "stop" -n ${VIDEO_NAME}
    else
	echo "invalid option: ${CAPTURE_OPTION}"
    fi
    popd

    return 0
}

# commentout--->
: <<'#COMMENTOUT'
function do_catkin_build(){

    # catkin build
    pushd $CATKIN_WS_DIR
    catkin clean -y
    catkin build
    source $HOME/.bashrc
    popd
}

function check_latest_hash(){

    pushd $BURGER_WAR_KIT_REPOSITORY
    git pull
    GITLOG_HASH=`git log | head -1 | cut -d' ' -f2`
    popd

    # check latest hash
    pushd $BURGER_WAR_DEV_REPOSITORY
    git pull
    GITLOG_HASH=`git log | head -1 | cut -d' ' -f2`
    if [ "$GITLOG_HASH" != "$LATEST_GITLOG_HASH" ];then
	TODAY=`date +"%Y%m%d%I%M%S"`
	echo "#--> latest commit:${GITLOG_HASH} ${TODAY} in burger_war_dev" >> $RESULTLOG
	LATEST_GITLOG_HASH=$GITLOG_HASH
	do_catkin_build
    fi
    popd
}

function do_result_analyzer(){
    INPUTFILE=$1
    OUTPUTFILE=$2
    ANALYZE_FILE_NAME="result_tmp.log"

    pushd $BURGER_WAR_AUTOTEST_LOG_REPOSITORY
    # preprocess
    LATEST_COMMIT_STR=`cat ${INPUTFILE} | grep "latest commit" | tail -1`             # get string
    LATEST_COMMIT_LINE_N=`grep "$LATEST_COMMIT_STR" -n ${INPUTFILE} | cut -d':' -f 1` # get line from string
    tail +${LATEST_COMMIT_LINE_N} ${INPUTFILE} > $ANALYZE_FILE_NAME                                 # get file from line
    # analyze
    python result_analyzer.py > ${OUTPUTFILE}                                         # get analyze matrix
    popd
}

function do_push(){

    # result log push
    pushd $BURGER_WAR_AUTOTEST_LOG_REPOSITORY/result
    git pull
    cp $SRC_LOG $DST_LOG
    git add $DST_LOG
    git commit -m "result.log update"
    git push
    popd

    # result analyze push
    RESULT_ANALYZER_DIR=$BURGER_WAR_AUTOTEST_LOG_REPOSITORY/result/result_analyzer
    pushd $RESULT_ANALYZER_DIR
    TARGET_HASH_ID=`cat ${SRC_LOG} | grep "latest commit" | tail -1 | cut -d':' -f2 | cut -d' ' -f1`
    TODAY=`cat ${SRC_LOG} | grep "latest commit" | tail -1 | cut -d':' -f2 | cut -d' ' -f2`
    RESULT_ANALYZE_DST_LOG=result_analyzer-${TODAY}${TARGET_HASH_ID}.log
    do_result_analyzer $SRC_LOG ${RESULT_ANALYZER_DIR}/${RESULT_ANALYZE_DST_LOG}
    git add $RESULT_ANALYZE_DST_LOG
    git commit -m "result_analyzer.log update"
    git push
    popd
}

#COMMENTOUT
# <---commentout

# main loop
for ((i=0; i<${LOOP_TIMES}; i++));
do
    #check_latest_hash
    do_game ${i} 1 225 # 180 * 5/4 
    do_game ${i} 2 225 # 180 * 5/4 
    do_game ${i} 3 225 # 180 * 5/4
    #do_game ${i} 1 225 "b" # 180 * 5/4 # only enemy level1,2,3 works r side
    #do_game ${i} 2 225 "b" # 180 * 5/4 # 
    #do_game ${i} 3 225 "b" # 180 * 5/4 # 
    do_game ${i} 4 225 # 180 * 5/4
    do_game ${i} 5 225 # 180 * 5/4
    do_game ${i} 6 225 # 180 * 5/4
    do_game ${i} 7 225 # 180 * 5/4
    do_game ${i} 8 225 # 180 * 5/4
    do_game ${i} 9 225 # 180 * 5/4
    do_game ${i} 10 225 # 180 * 5/4
    #do_push
done

