#!/bin/bash -e

tmux_single()
{
    local SESSIONEXISTS=$(tmux list-sessions | grep "$1")
    if [ "$SESSIONEXISTS" = "" ]
    then
        echo "No session found: $1"
        tmux new-session -d -s $1
        tmux rename-window -t $1 $2
    else
        echo "Session found: $1"
        tmux new-window -t $1 -n $2
    fi

    tmux send-keys -t $2 "$3" C-m
}

tmux_single_loop()
{
    local SESSIONEXISTS=$(tmux list-sessions | grep "$1")
    if [ "$SESSIONEXISTS" = "" ]
    then
        echo "No session found: $1"
        tmux new-session -d -s $1
        tmux rename-window -t $1 $2
    else
        echo "Session found: $1"
        tmux new-window -t $1 -n $2
    fi
    tmux send-keys -t $2 'while [ true ]; do ' "$3" '; done' C-m
}

tmux_side_start()
{
    local SESSIONEXISTS=$(tmux list-sessions | grep "$1")
    if [ "$SESSIONEXISTS" = "" ]
    then
        echo "No session found: $1"
    else
        echo "Session found: $1"
        tmux select-window -t $1
        tmux split-window -h "$3"
    fi
}

HOME=/home/nvidia

tmux_single_loop "rosmaster" 'rosmaster' 'roscore'
sleep 4
tmux_single_loop "realsense" 'd435i' 'roslaunch avd_mech_final_team23 d435i.launch'
tmux_single_loop "aruco_detector" 'detector' 'python3 /home/nvidia/catkin_ws/src/avd_mech_final_team23/scripts/aruco_detector.py'
sleep 5
tmux_single_loop "serial_driver" 'ths2' 'rosrun avd_mech_final_team23 SerialDriver.py'
sleep 1
tmux_single_loop "udp" 'send' 'python3 /home/nvidia/catkin_ws/src/avd_mech_final_team23/scripts/udpSend.py'
tmux_single_loop "udp" 'recv' 'python3 /home/nvidia/catkin_ws/src/avd_mech_final_team23/scripts/udpRecv.py'
