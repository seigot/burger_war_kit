#!/bin/bash

# install below
#   $ sudo apt-get install -y recordmydesktop gtk-recordmydesktop
#   $ sudo apt-get install -y ffmpeg

# default value
FPS=5           # capture frame per sec (ex.4)
PLAYBACK_SPEED=1 # playback speed        (ex.8)
CAPTURE_NAME="capture.mp4"
MODE="start"

# get option
while getopts f:s:m:n: OPT
do
  case $OPT in
    "f" ) FPS="$OPTARG" ;;
    "s" ) PLAYBACK_SPEED="$OPTARG" ;;
    "m" ) MODE="$OPTARG" ;;
    "n" ) CAPTURE_NAME="$OPTARG" ;;
  esac
done

# echo
echo "FPS: ${FPS}"
echo "PLAYBACK_SPEED: ${PLAYBACK_SPEED}"
echo "CAPTURE_NAME: ${CAPTURE_NAME}"
echo "MODE: ${MODE}"

# warning
function output_warning(){
    # check if install package
    array=(
        recordmydesktop
        gtk-recordmydesktop
        ffmpeg
    )
    for e in ${array[@]}; do
        if ! dpkg -l | grep --quiet "${e}"; then
	    echo "---"
            echo "!!! [Warning] ${e} not installed, install by following !!!"
            echo "\$ sudo apt install ${e}"
	    return 0
            echo "!!! --------------------------------------------- !!!"
        fi
    done
}
output_warning

if [ $MODE == "start" ];then
    # capture start
    gnome-terminal -- recordmydesktop --no-sound --no-cursor --fps ${FPS} --width 1400 --height 1400

elif [ $MODE == "stop" ];then
    #capture stop
    TMP_FNAME_OGV="out.ogv"
    TMP_FNAME_MP4="out.mp4"
    TMP2_FNAME_MP4="out2.mp4"    
    rm -f $TMP_FNAME_OGV
    rm -f $TMP_FNAME_MP4
    rm -f $TMP2_FNAME_MP4
    # kill process
    PROCESS_ID=`ps -e -o pid,cmd | grep "recordmydesktop" | grep -v grep | awk '{print $1}'`
    kill $PROCESS_ID
    # wait to kill process
    LOOP_TIMES=3600
    for i in `seq ${LOOP_TIMES}`
    do
	NUM=`ps -e -o pid,cmd | grep "recordmydesktop" | grep -v grep | wc -l`
	if [ $NUM -eq 0 ]; then
	    break;
	fi
	sleep 1
    done
    # save video
    #ffmpeg -i $TMP_FNAME_OGV -vf scale=1024:-1 $TMP_FNAME_MP4
    #ffmpeg -i $TMP_FNAME_MP4 -vf setpts=PTS/3.3 -af atempo=2.0 $TMP2_FNAME_MP4
    ffmpeg -i $TMP_FNAME_OGV -vf setpts=PTS/${PLAYBACK_SPEED} -af atempo=2.0 $TMP2_FNAME_MP4
    mv $TMP2_FNAME_MP4 ${CAPTURE_NAME}
    rm -f $TMP_FNAME_OGV
    rm -f $TMP_FNAME_MP4
    rm -f $TMP2_FNAME_MP4
else
    echo "do nothing"
fi
