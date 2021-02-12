#!/bin/bash

REFERENCE_DATE='2021/01/27'
LOOP_TIMES=1
RESULTLOG="result.log"
echo "repository_name, number_of_commits(since ${REFERENCE_DATE})" > ${RESULTLOG}

REPOSITORY_LIST=(
    # update 02/12
    "seigot"
    "airkei"
    "Arthur-MA2"
    "chef48307"
    "EndoNrak"
    "hi64n"
    "kt-arrow"
    "mashioka"
    "safubuki"
    "adelie7273"
    "aze-taiki"
    "kenjirotorii"
    "takeoverjp"
    "shendongqiang"
    "shunsuke-f"
    "suzukisatoru"
    "takaoh"
    "yfuruya123"
    "ynakayama0908"
    "ysk0811"
    "YusukeMori3250"
    "yuyak0422"
)

# main loop
for ((j=0; j<${LOOP_TIMES}; j++));
do
    # check all repository
    for (( i = 0; i < ${#REPOSITORY_LIST[@]}; i++ ))
    do
	# prepare
	echo ${REPOSITORY_LIST[$i]}

	REPOSITORY_NAME="burger_war_dev.${REPOSITORY_LIST[$i]}"
	if [ ! -d "./${REPOSITORY_NAME}" ]; then
	    git clone http://github.com/${REPOSITORY_LIST[$i]}/burger_war_dev ${REPOSITORY_NAME}
	fi
	pushd ${REPOSITORY_NAME}
	git pull
	COMMIT_NUM=`git log --after ${REFERENCE_DATE} --oneline | wc -l`
	echo ${COMMIT_NUM}
	popd
	echo "${REPOSITORY_NAME},${COMMIT_NUM}" >> ${RESULTLOG}

    done
done