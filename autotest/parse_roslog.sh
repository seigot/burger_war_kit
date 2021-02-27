#!/bin/bash -e

function roslog2csv() {
    grep rosout.machine |
        grep point_changed |
        cut -d' ' -f4-
}

function send_slack_point_history() {
    if [ -z "${SLACK_SH_FILE_UPLOAD_TOKEN}" ]; then
	echo "send_slack_point_history: Please set SLACK_SH_FILE_UPLOAD_TOKEN env var."
	return 1
    fi

    curl -s -F file=@"$1" \
	 -F "initial_comment=$2" \
	 -F "channels=C01MRLH3J5R" \
	 -H "Authorization: Bearer ${SLACK_SH_FILE_UPLOAD_TOKEN}" \
	 https://slack.com/api/files.upload > /dev/null
}

function parse_roslog() {
    if ! dpkg -l python-pandas &> /dev/null; then
	echo "Please install python-pandas"
	return 1
    fi

    local -r MESSAGE="$1"
    local -r IN_FILE="${2:-${HOME}/.ros/log/latest/seigoRun-5.log}"
    if [ ! -f ${IN_FILE} ]; then
	echo "${IN_FILE} not found"
	return 1
    fi

    local -r DIR="$( cd "$( dirname "$BASH_SOURCE" )" && pwd -P )"
    local -r LOG_DIR=$(dirname ${IN_FILE})
    local -r CSV_FILE="${LOG_DIR}/point_changed.csv"
    local -r PNG_FILE="${LOG_DIR}/point_history.png"
    cat "${IN_FILE}" |
        roslog2csv > "${CSV_FILE}"
    python ${DIR}/draw_point_history.py "${CSV_FILE}" "${PNG_FILE}"
    send_slack_point_history "${PNG_FILE}" "${MESSAGE}"
}

# The followings are test code.
# When source this script, return here
return 1 2>/dev/null || true

parse_roslog $@
