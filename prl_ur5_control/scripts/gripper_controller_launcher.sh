#!/bin/bash

########
# This script select which controller to run for the gripper (by reading ros param) and launch it.
#  It expects the first argument to be 'left' or 'right' to determine the gripper,
#  and all the other paramters will be forwarded to the controller (if needed).
########

arg_nb=$#
arg_select_min=2 # The first argument is used to determine the side (left/right) thus shoud not be forwarded
arg_select_max=$arg_nb-2 # The last two argument of the script are __log and __name that is not needed for the gripper controller script
arg_to_forward=${@:$arg_select_min:$arg_select_max} #Extract the arguyment to pass to the gripper controller script

gripper_controller=`rosparam get /setup/$1/gripper_controller`

if [ "$gripper_controller" = "onrobot_rg" ]
then
    roslaunch onrobot_control onrobot_gripper.launch $arg_to_forward __ns:="$1_gripper"
elif [ "$gripper_controller" = "fake" ]
then
    rosrun prl_ur5_control fake_gripper_controller.py _joint:="$1_gripper_joint" __name:="gripper_controller" __ns:="$1_gripper"
else
    echo "Unknown controller : $gripper_controller, for gripper : $1" >&2
    echo "Check configuration file !" >&2
    return -1
fi
