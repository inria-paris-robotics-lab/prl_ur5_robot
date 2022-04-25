#!/bin/bash

find_cam_and_record() {
    ## Check arguments ##
    if test -z $4
    then
        echo "Not enough argument provided. Expected [serial_number] [output_path_prefix] [video_size] [framerate]"
        return -1
    fi

    ## Find camera video stream ##
    video_inputs=`ls /dev/video*` # All the available video streams
    video_input_found=""
    for video_input in $video_inputs
    do
        infos=`udevadm info --query=all $video_input`
        sn=`echo "$infos" | sed -n "s/E: ID_SERIAL=\(.*\)/\1/p"` # Get serial numbers
        if [[ "$sn" == "$1" ]] && [[ $infos =~ ID_V4L_CAPABILITIES=.*capture ]] # Serial number match the input and the device can capture
        then
            video_input_found=$video_input
            break
        fi
    done

    ## Check that the video stream has been found ##
    if test -z $video_input_found
    then
        echo "No input stream fount with camera serial number" $1 "and capture capabilities"
        return 1
    fi

    ## Create output file path and name ##
    datetime=`date +%Y-%m-%d-%H-%M-%S`
    filepath=$2_${datetime}.mkv

    ## Recording parameters ##
    video_size=1280x960
    framerate=30

    ## Record ##
    echo "Recording on $video_input_found $video_size @ $framerate fps ..."
    ffmpeg -framerate $framerate -video_size $video_size -input_format mjpeg -i $video_input_found $filepath -y -loglevel fatal
    echo "Done recording."
}

# Package all the code in a function so "return" stops everything (when checking for failures)
find_cam_and_record $@